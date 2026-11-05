"""
faixas.py - acha as linhas da pista na vista de cima (BEV) e devolve o
CENTRO DA FAIXA A (a faixa do carro) como uma curva X(Y) em centimetros.

Como funciona (diferente do gatuno-rapido, que varre fileiras da imagem):

 1. Realce: diferenca de gaussianas (DoG) na BEV, casada com a largura da
    linha. Fica claro so o que e mais claro que o entorno ~largura de linha.
 2. Linhas: votacao tipo Hough em coordenadas do chao. Cada pixel vota em
    (inclinacao, posicao); o pico e uma linha quase vertical. Tracejados
    nao atrapalham (o voto soma por todo o comprimento, buracos nao importam).
    Depois cada linha e refinada (reta ou parabola) so com os pixels dela.
 3. Continua x pontilhada: fracao de trechos da linha que tem tinta
    ("cobertura"). ~1.0 = continua, ~0.5 = pontilhada. Suavizada no tempo.
 4. Identidade pela GEOMETRIA da pista (continua | pontilhada | continua):
        B = continua da faixa vizinha   -> centro da faixa A = x + 1.5*L
        D = pontilhada (entre as faixas)-> centro da faixa A = x + 0.5*L
        A = continua da faixa do carro  -> centro da faixa A = x - 0.5*L
    Cada linha visivel vira UMA estimativa do centro da faixa A; as
    estimativas sao fundidas. Isso funciona mesmo quando o carro cruza a
    pontilhada pra desviar de um cone (nao precisa de maquina de estados).
"""

import math

import cv2
import numpy as np

import config


class Linha:
    """Uma linha do chao: X = poly(Y), valida em [y0, y1] (cm)."""
    __slots__ = ("coef", "y0", "y1", "n", "residuo", "cobertura", "classe",
                 "ident", "peso", "x_ref", "cob_ema")

    def __init__(self, coef, y0, y1, n, residuo):
        self.coef = np.asarray(coef, dtype=np.float64)
        self.y0, self.y1, self.n, self.residuo = y0, y1, n, residuo
        self.cobertura = None      # do quadro atual
        self.cob_ema = None        # suavizada no tempo
        self.classe = None         # 'continua' | 'pontilhada' | None
        self.ident = None          # 'A' | 'D' | 'B'
        self.peso = 1.0
        self.x_ref = 0.0

    def x(self, Y):
        """X na distancia Y; fora do trecho medido segue a tangente (nao explode)."""
        Y = np.asarray(Y, dtype=np.float64)
        Yc = np.clip(Y, self.y0, self.y1)
        base = np.polyval(self.coef, Yc)
        if len(self.coef) > 2:
            der = np.polyval(np.polyder(self.coef), Yc)
        else:
            der = np.full_like(Yc, self.coef[0])
        der = np.clip(der, -0.8, 0.8)
        return base + der * (Y - Yc)

    def inclinacao(self, Y):
        Yc = np.clip(Y, self.y0, self.y1)
        if len(self.coef) > 2:
            return float(np.clip(np.polyval(np.polyder(self.coef), Yc), -0.8, 0.8))
        return float(np.clip(self.coef[0], -0.8, 0.8))


class Centro:
    """Centro da faixa A: X = poly(Y)."""
    __slots__ = ("coef", "y0", "y1", "n_linhas", "grau")

    def __init__(self, coef, y0, y1, n_linhas):
        self.coef = np.asarray(coef, dtype=np.float64)
        self.y0, self.y1, self.n_linhas = y0, y1, n_linhas
        self.grau = len(self.coef) - 1

    def x(self, Y):
        Y = np.asarray(Y, dtype=np.float64)
        Yc = np.clip(Y, self.y0, self.y1)
        base = np.polyval(self.coef, Yc)
        der = np.polyval(np.polyder(self.coef), Yc) if self.grau >= 2 else self.coef[0]
        return base + np.clip(der, -0.8, 0.8) * (Y - Yc)

    def inclinacao(self, Y):
        Yc = np.clip(Y, self.y0, self.y1)
        der = np.polyval(np.polyder(self.coef), Yc) if self.grau >= 2 else self.coef[0]
        return float(np.clip(der, -0.8, 0.8))


class ResultadoFaixas:
    __slots__ = ("centro", "linhas", "motivo", "mascara", "bev", "c_ref")

    def __init__(self):
        self.centro = None      # Centro ou None
        self.linhas = []        # [Linha]
        self.motivo = "ok"
        self.mascara = None     # mascara binaria na BEV (debug)
        self.bev = None         # BEV colorida (debug)
        self.c_ref = None       # X do centro da faixa A no Y de referencia


class DetectorFaixas:
    def __init__(self, cam):
        self.cam = cam
        self._tracks = []           # linhas acompanhadas (EMA da cobertura)
        self.c_prev = 0.0           # ultimo centro conhecido (X no Y de referencia)
        self._tem_prev = False
        self._salto = 0
        self.rng = np.random.default_rng(1)

    # ------------------------------------------------------------------
    def processar(self, frame, debug=False):
        cam = self.cam
        res = ResultadoFaixas()
        bev = cam.para_bev(frame, borda=cv2.BORDER_REPLICATE)
        if debug:
            res.bev = bev
        mascara = self._mascara(bev)
        if debug:
            res.mascara = mascara
        frac = float(mascara.mean())
        if frac > 0.25:
            res.motivo = "branco demais (luz estourada?)"
            return self._perdeu(res)

        linhas = self._achar_linhas(mascara)
        if not linhas:
            res.motivo = "sem linhas"
            return self._perdeu(res)

        self._medir_cobertura(linhas, mascara)
        self._classificar(linhas)
        self._identificar(linhas)
        res.linhas = linhas

        centro = self._fundir(linhas)
        if centro is None:
            res.motivo = "linhas inconsistentes"
            return self._perdeu(res)

        y_ref = self._y_ref()
        c_ref = float(centro.x(y_ref))
        if self._tem_prev and abs(c_ref - self.c_prev) > 0.62 * config.FAIXA_LARGURA_CM:
            # um centro nao pula meia faixa de um quadro pro outro: ignora, a
            # nao ser que persista (ai o carro realmente foi pra la)
            self._salto += 1
            if self._salto < 6:
                res.motivo = "salto de faixa (ignorado)"
                res.linhas = linhas
                return res
        self._salto = 0
        self.c_prev = c_ref
        self._tem_prev = True
        res.centro = centro
        res.c_ref = c_ref
        return res

    def _perdeu(self, res):
        return res

    def _y_ref(self):
        return float(np.clip(config.OLHAR_ADIANTE_CM, self.cam.y_min, self.cam.y_max))

    # ------------------------------------------------------------------
    def _mascara(self, bev):
        cam = self.cam
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        lw = config.LINHA_LARGURA_CM * cam.px_cm
        s_in = max(1.0, lw / 2.5)
        s_out = max(s_in * 3.0, lw * 1.6)
        g1 = cv2.GaussianBlur(gray, (0, 0), s_in).astype(np.float32)
        g2 = cv2.GaussianBlur(gray, (0, 0), s_out).astype(np.float32)
        dog = g1 - g2
        valido = cam.mascara_bev
        vals = dog[valido]
        p = float(np.percentile(vals, config.DOG_PERCENTIL)) if vals.size else 0.0
        limiar = max(float(config.DOG_MIN), config.DOG_FATOR * p)

        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        laranja = (((h <= config.CONE_H_BAIXO) | (h >= config.CONE_H_ALTO)) &
                   (s >= config.CONE_S_MIN) & (v >= config.CONE_V_MIN)).astype(np.uint8)
        laranja = cv2.dilate(laranja, np.ones((7, 7), np.uint8)) > 0

        m = ((dog > limiar) & (gray > config.BRILHO_MIN) & (s < config.SAT_MAX)
             & ~laranja & valido)
        m = m.astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        return m

    # ------------------------------------------------------------------
    def _achar_linhas(self, mascara):
        """Votacao em (inclinacao, posicao). Devolve ate 3 Linhas."""
        cam = self.cam
        ys, xs = np.nonzero(mascara)
        if xs.size < config.LINHA_MIN_PTS:
            return []
        X = xs / cam.px_cm - cam.x_max
        Y = cam.y_max - ys / cam.px_cm
        linhas = []
        y_ref = 0.5 * (cam.y_min + cam.y_max)
        slopes = np.linspace(-0.7, 0.7, 29)
        res_x = 1.0                                         # cm por bin
        margem = config.JANELA_MARGEM_CM
        kernel = np.ones((1, 5), np.float32) / 5.0

        for _ in range(3):
            if X.size < config.LINHA_MIN_PTS:
                break
            idx_sub = np.arange(X.size)
            if X.size > 5000:
                idx_sub = self.rng.choice(X.size, 5000, replace=False)
            Xs, Ys = X[idx_sub], Y[idx_sub]
            x0 = Xs[None, :] - slopes[:, None] * (Ys[None, :] - y_ref)      # (S, N)
            lim = cam.x_max + 0.7 * (cam.y_max - cam.y_min) + 2
            nb = int(2 * lim / res_x) + 1
            ib = np.clip(((x0 + lim) / res_x).astype(np.int32), 0, nb - 1)
            flat = (np.arange(len(slopes))[:, None] * nb + ib).ravel()
            acc = np.bincount(flat, minlength=len(slopes) * nb).reshape(len(slopes), nb)
            acc = cv2.filter2D(acc.astype(np.float32), -1, kernel * 3.0)    # ~ pixels em +-1.5cm
            si, bi = np.unravel_index(int(np.argmax(acc)), acc.shape)
            voto = acc[si, bi] * (X.size / max(1, idx_sub.size))
            if voto < config.LINHA_MIN_PTS:
                break
            s0 = slopes[si]
            x0c = bi * res_x - lim + res_x / 2.0
            linha = self._refinar(X, Y, s0, x0c, y_ref, margem)
            # tira os pixels dessa linha (mesmo se rejeitada, pra nao repetir)
            if linha is not None:
                dist = np.abs(X - linha.x(Y))
            else:
                dist = np.abs(X - (x0c + s0 * (Y - y_ref)))
            manter = dist > margem * 1.1
            if linha is not None:
                linhas.append(linha)
            X, Y = X[manter], Y[manter]
        linhas.sort(key=lambda l: float(l.x(self._y_ref())))
        return linhas

    def _refinar(self, X, Y, s0, x0, y_ref, margem):
        """Reta -> (se der) parabola, usando so os pixels perto da linha."""
        cam = self.cam
        coef = np.array([s0, x0 - s0 * y_ref])      # X = s*Y + b
        sel = None
        for it, gate in enumerate((margem, margem * 0.8, margem * 0.7)):
            d = np.abs(X - np.polyval(coef, Y))
            sel = d < gate
            if sel.sum() < config.LINHA_MIN_PTS * 0.6:
                return None
            Ys, Xs = Y[sel], X[sel]
            extensao = np.ptp(Ys)
            grau = 1
            if (extensao >= config.CURVA_GRAU2_EXTENSAO * (cam.y_max - cam.y_min)
                    and sel.sum() >= 150):
                grau = 2
            coef = np.polyfit(Ys, Xs, grau)
        Ys, Xs = Y[sel], X[sel]
        y0, y1 = float(np.percentile(Ys, 2)), float(np.percentile(Ys, 98))
        residuo = float(np.sqrt(np.mean((Xs - np.polyval(coef, Ys)) ** 2)))
        if (y1 - y0) < config.LINHA_MIN_EXTENSAO_CM:
            return None
        if residuo > config.LINHA_RESIDUO_MAX_CM:
            return None
        if abs(float(np.polyval(np.polyder(coef), 0.5 * (y0 + y1)))) > 0.75:
            return None                            # quase deitada: nao e linha da pista
        return Linha(coef, y0, y1, int(sel.sum()), residuo)

    # ------------------------------------------------------------------
    def _medir_cobertura(self, linhas, mascara):
        """Fracao de trechos da linha que tem tinta (continua ~1, pontilhada ~0.5)."""
        cam = self.cam
        ys, xs = np.nonzero(mascara)
        X = xs / cam.px_cm - cam.x_max
        Y = cam.y_max - ys / cam.px_cm
        K = 12
        borda = np.linspace(cam.y_min, cam.y_max, K + 1)
        alt_seg = (cam.y_max - cam.y_min) / K
        tol = config.LINHA_LARGURA_CM / 2.0 + 1.5
        area_esp = alt_seg * config.LINHA_LARGURA_CM * cam.px_cm ** 2
        for l in linhas:
            l.peso = float(np.clip(l.n / 400.0, 0.3, 3.0)) / (1.0 + l.residuo)
            centros = 0.5 * (borda[:-1] + borda[1:])
            xc = l.x(centros)
            dentro = np.abs(xc) < cam.x_max - 3.0
            # trecho so conta se a imagem cobre ele de verdade
            ok = np.zeros(K, bool)
            for k in range(K):
                if dentro[k]:
                    bu, bv = cam.chao_para_bev(xc[k], centros[k])
                    bu, bv = int(round(bu)), int(round(bv))
                    if 0 <= bv < cam.bev_h and 0 <= bu < cam.bev_w:
                        ok[k] = bool(cam.mascara_bev[bv, bu])
            if ok.sum() < 5:
                l.cobertura = None
                continue
            perto = np.abs(X - l.x(Y)) < tol
            seg = np.clip(((Y[perto] - cam.y_min) / alt_seg).astype(int), 0, K - 1)
            cont = np.bincount(seg, minlength=K)
            com_tinta = cont >= 0.25 * area_esp
            l.cobertura = float((com_tinta & ok).sum() / ok.sum())

    def _classificar(self, linhas):
        """Suaviza a cobertura entre quadros e decide continua/pontilhada."""
        a = config.COBERTURA_SUAVIZACAO
        y_ref = self._y_ref()
        novas = []
        for l in linhas:
            l.x_ref = float(l.x(y_ref))
            tr = None
            melhor = 8.0
            for t in self._tracks:
                d = abs(t["x"] - l.x_ref)
                if d < melhor:
                    melhor, tr = d, t
            if l.cobertura is not None:
                if tr is None or tr["cob"] is None:
                    cob = l.cobertura
                else:
                    cob = a * l.cobertura + (1 - a) * tr["cob"]
            else:
                cob = tr["cob"] if tr is not None else None
            l.cob_ema = cob
            novas.append({"x": l.x_ref, "cob": cob})
            if cob is None:
                l.classe = None
            elif cob >= config.COBERTURA_CONTINUA_MIN:
                l.classe = "continua"
            elif cob <= config.COBERTURA_PONTILHADA_MAX:
                l.classe = "pontilhada"
            else:
                l.classe = None
        self._tracks = novas

    def _identificar(self, linhas):
        """Atribui A / D / B a cada linha pela classe e pela posicao esperada."""
        W = config.FAIXA_LARGURA_CM
        c = self.c_prev if self._tem_prev else 0.0
        esperado = {"B": c - 1.5 * W, "D": c - 0.5 * W, "A": c + 0.5 * W}
        # a classe (continua/pontilhada) e uma PISTA suave: uma linha cortada
        # pela borda da imagem parece pontilhada sem ser. A posicao esperada
        # (a partir do ultimo centro conhecido) pesa mais.
        esperada_classe = {"B": "continua", "D": "pontilhada", "A": "continua"}
        usados = set()
        for l in sorted(linhas, key=lambda l: -l.peso):
            custos = []
            for k, xe in esperado.items():
                dist = abs(l.x_ref - xe)
                if dist > 0.55 * W:
                    continue
                custo = dist / W
                if l.classe is not None and l.classe != esperada_classe[k]:
                    custo += 0.30
                custos.append((custo, k))
            custos.sort()
            l.ident = None
            for _, k in custos:
                if k not in usados:
                    l.ident = k
                    usados.add(k)
                    break

    # ------------------------------------------------------------------
    def _fundir(self, linhas):
        """Cada linha identificada da uma estimativa do centro da faixa A."""
        cam = self.cam
        W = config.FAIXA_LARGURA_CM
        desloc = {"B": +1.5 * W, "D": +0.5 * W, "A": -0.5 * W}
        uteis = [l for l in linhas if l.ident is not None]
        if not uteis:
            return None
        y_ref = self._y_ref()
        # descarta estimativas que discordam muito da mais confiavel
        est = [(l, float(l.x(y_ref)) + desloc[l.ident]) for l in uteis]
        est.sort(key=lambda t: -t[0].peso)
        base = est[0][1]
        est = [(l, c) for (l, c) in est if abs(c - base) <= 0.35 * W]

        Ys = np.linspace(cam.y_min, cam.y_max, 14)
        num = np.zeros_like(Ys)
        den = 0.0
        for l, _ in est:
            num += l.peso * (l.x(Ys) + desloc[l.ident])
            den += l.peso
        Xc = num / den
        y_lo = min(l.y0 for l, _ in est)
        y_hi = max(l.y1 for l, _ in est)
        extensao = y_hi - y_lo
        grau = 2 if (extensao >= config.CURVA_GRAU2_EXTENSAO * (cam.y_max - cam.y_min)) else 1
        coef = np.polyfit(Ys, Xc, grau)
        return Centro(coef, cam.y_min, cam.y_max, len(est))
