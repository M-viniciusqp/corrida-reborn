"""
sim.py - simulador em malha fechada: testa o piloto SEM o carro.

Desenha o que a camera veria (pista, linhas continua/pontilhada, cones),
roda o MESMO Piloto do carro e move um carro (modelo de bicicleta) com o
comando que ele devolve. Serve pra achar defeito de logica e de sinal
(esquerda/direita), nao pra afinar numero: a fisica aqui e otimista.

    python sim.py                      # roda todos os cenarios
    python sim.py cone_centro curva    # so esses
    python sim.py cone_centro --ver    # abre janela (so no PC / tela do Pi)
    python sim.py --erro-cam           # a camera REAL difere da que o piloto supoe
    python sim.py --salvar pasta       # grava imagens dos cenarios
"""

import math
import os
import sys
import time

import cv2
import numpy as np

import config
from geometria import CameraChao, parametros_padrao
from piloto import Piloto

W = config.FAIXA_LARGURA_CM
PISO = (58, 60, 62)
LINHA = (232, 232, 232)
PAREDE = (120, 125, 130)


class Pista:
    """Linha central integrada de trechos (comprimento_cm, curvatura 1/cm; + = direita)."""

    def __init__(self, trechos, cones=(), traco=(12.0, 12.0)):
        self.cones = [dict(x=c[0], y=c[1]) for c in cones]   # (s_cm, d_cm da faixa A, + direita)
        self.traco = traco
        pts, psis, S = [], [], []
        x = y = psi = 0.0
        # reta antes do inicio e depois do fim
        plano = [(150.0, 0.0)] + list(trechos) + [(250.0, 0.0)]
        s = -150.0
        for comp, kappa in plano:
            for _ in range(int(comp)):
                pts.append((x, y)); psis.append(psi); S.append(s)
                psi += kappa
                x += math.sin(psi); y += math.cos(psi)
                s += 1.0
        self.P = np.array(pts); self.PSI = np.array(psis); self.S = np.array(S)
        self.N = np.stack([np.cos(self.PSI), -np.sin(self.PSI)], 1)   # normal a direita
        self.comp_total = sum(t[0] for t in trechos)
        # cones em coordenadas do mundo
        for c in self.cones:
            i = int(np.argmin(np.abs(self.S - c["x"])))
            c["w"] = self.P[i] + c["y"] * self.N[i]

    def lane(self, p):
        """posicao -> (s, d) em relacao a linha central (d + = direita)."""
        i = int(np.argmin(np.hypot(self.P[:, 0] - p[0], self.P[:, 1] - p[1])))
        d = float(np.dot(np.asarray(p) - self.P[i], self.N[i]))
        return float(self.S[i]), d, i

    def linhas(self):
        """[(offset d, 'continua'|'pontilhada')] da faixa A + vizinha."""
        return [(-1.5 * W, "continua"), (-0.5 * W, "pontilhada"), (0.5 * W, "continua")]


class Carro:
    def __init__(self, pista, d0=0.0, psi0=0.0):
        self.Lb = 20.0                       # distancia entre eixos (cm)
        # eixo TRASEIRO no inicio da faixa (s = 0), deslocado d0 cm pra direita
        self.p = pista.P[150] + pista.N[150] * d0
        self.psi = psi0
        self.delta = 0.0
        self.v = 0.0

    def frente(self):
        return self.p + self.Lb * np.array([math.sin(self.psi), math.cos(self.psi)])

    def passo(self, turn, speed, dt, fator_v=1.0, atraso=0.12):
        delta_cmd = math.radians(config.ESTERCO_MAX_GRAUS) * turn
        if config.ESTERCO_INVERTER:
            delta_cmd = -delta_cmd
        alvo_v = speed * config.V_CMS_NA_VEL_1 * fator_v
        n = max(1, int(dt / 0.01))
        h = dt / n
        for _ in range(n):
            self.delta += (delta_cmd - self.delta) * min(1.0, h / atraso)
            self.v += (alvo_v - self.v) * min(1.0, h / 0.15)
            self.psi += self.v * math.tan(self.delta) / self.Lb * h
            self.p = self.p + self.v * h * np.array([math.sin(self.psi), math.cos(self.psi)])


class Renderizador:
    def __init__(self, cam_real, res):
        self.cam = cam_real
        self.res = res
        self.px = 2.0
        self.x_rng = 110.0
        self.y_ini, self.y_fim = 4.0, 190.0
        self.cw = int(2 * self.x_rng * self.px)
        self.ch = int((self.y_fim - self.y_ini) * self.px)
        cantos = [(-self.x_rng, self.y_fim), (self.x_rng, self.y_fim),
                  (self.x_rng, self.y_ini), (-self.x_rng, self.y_ini)]
        img = np.array([cam_real.chao_para_pixel(X, Y) for X, Y in cantos], np.float32)
        can = np.array([[0, 0], [self.cw - 1, 0], [self.cw - 1, self.ch - 1], [0, self.ch - 1]], np.float32)
        self.H = cv2.getPerspectiveTransform(can, img)
        self.rng = np.random.default_rng(3)

    def _c(self, X, Y):
        return int(round((X + self.x_rng) * self.px)), int(round((self.y_fim - Y) * self.px))

    def quadro(self, pista, carro, claridade=1.0, ruido=4.0):
        f = carro.frente()
        psi = carro.psi
        r = np.array([math.cos(psi), -math.sin(psi)])
        fw = np.array([math.sin(psi), math.cos(psi)])
        canvas = np.full((self.ch, self.cw, 3), PISO, np.uint8)
        s0, _, i0 = pista.lane(f)
        lo, hi = i0 - 60, i0 + 260
        lo = max(lo, 0); hi = min(hi, len(pista.P))
        idx = np.arange(lo, hi)
        esp = max(1, int(round(config.LINHA_LARGURA_CM * self.px)))
        for d, tipo in pista.linhas():
            pts = pista.P[idx] + d * pista.N[idx]
            rel = pts - f
            X = rel @ r
            Y = rel @ fw
            S = pista.S[idx]
            if tipo == "continua":
                seg = [np.arange(len(idx))]
            else:
                on, off = pista.traco
                fase = np.mod(S, on + off) < on
                seg, atual = [], []
                for k in range(len(idx)):
                    if fase[k]:
                        atual.append(k)
                    elif atual:
                        seg.append(np.array(atual)); atual = []
                if atual:
                    seg.append(np.array(atual))
            for ks in seg:
                if len(ks) < 2:
                    continue
                poly = np.array([self._c(X[k], Y[k]) for k in ks], np.int32)
                cv2.polylines(canvas, [poly], False, LINHA, esp, cv2.LINE_AA)
        img = cv2.warpPerspective(canvas, self.H, self.res, flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=PAREDE)
        # cones (triangulos em pe)
        cam = self.cam
        for c in pista.cones:
            rel = c["w"] - f
            X, Y = float(rel @ r), float(rel @ fw)
            Yb = Y - config.CONE_BASE_OFFSET_CM
            cw2 = config.CONE_LARGURA_CM / 2.0
            ptsc = [cam.ponto3d_para_pixel(X - cw2, Yb, 0), cam.ponto3d_para_pixel(X + cw2, Yb, 0),
                    cam.ponto3d_para_pixel(X, Yb + 3, 26.0)]
            if any(p is None for p in ptsc) or Yb < 3:
                continue
            poly = np.array(ptsc, np.int32)
            cv2.fillConvexPoly(img, poly, (20, 95, 255))        # laranja (BGR)
        img = img.astype(np.float32) * claridade
        img += self.rng.normal(0, ruido, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


def pose_stats(pista, carro, cones_mundo):
    f = carro.frente()
    s, d, _ = pista.lane(f)
    meia = config.CARRO_LARGURA_CM / 2.0
    fora_dir = d + meia - (W / 2.0)           # > 0: passou da continua
    fora_esq = (-1.5 * W) - (d - meia)        # > 0: passou da continua da outra faixa
    # distancia do cone ao retangulo do carro
    psi = carro.psi
    fw = np.array([math.sin(psi), math.cos(psi)]); r = np.array([math.cos(psi), -math.sin(psi)])
    folga = 1e9
    for c in cones_mundo:
        rel = c - carro.p
        a, b = float(rel @ r), float(rel @ fw)           # lateral, longitudinal (a partir do eixo traseiro)
        dx = max(abs(a) - meia, 0.0)
        dy = max(-6.0 - b, b - (carro.Lb + 6.0), 0.0)
        folga = min(folga, math.hypot(dx, dy) - config.CONE_LARGURA_CM / 2.0)
    return s, d, max(fora_dir, fora_esq), folga


def cenario(nome):
    cw = config.CONE_LARGURA_CM
    if nome == "reta":
        return Pista([(450, 0.0)]), dict(d0=0, psi0=0)
    if nome == "reta_desalinhado":
        return Pista([(450, 0.0)]), dict(d0=7, psi0=math.radians(8))
    if nome == "cone_centro":
        return Pista([(520, 0.0)], cones=[(300, 0)]), dict()
    if nome == "cone_direita":
        return Pista([(520, 0.0)], cones=[(300, 8)]), dict()
    if nome == "cone_esquerda":
        return Pista([(520, 0.0)], cones=[(300, -9)]), dict()
    if nome == "cone_outra_faixa":
        return Pista([(520, 0.0)], cones=[(300, -29)]), dict()
    if nome == "dois_cones":
        return Pista([(700, 0.0)], cones=[(280, 3), (470, -4)]), dict()
    if nome == "curva_dir":
        return Pista([(150, 0.0), (180, 1 / 130.0), (200, 0.0)]), dict()
    if nome == "curva_esq":
        return Pista([(150, 0.0), (180, -1 / 130.0), (200, 0.0)]), dict()
    if nome == "curva_cone":
        return Pista([(120, 0.0), (220, 1 / 150.0), (200, 0.0)], cones=[(260, 2)]), dict()
    raise SystemExit(f"cenario desconhecido: {nome}")


NOMES = ["reta", "reta_desalinhado", "cone_centro", "cone_direita", "cone_esquerda",
         "cone_outra_faixa", "dois_cones", "curva_dir", "curva_esq", "curva_cone"]


def rodar(nome, ver=False, erro_cam=False, salvar=None, fps=12.0, fator_v=1.0, quiet=False):
    pista, kw = cenario(nome)
    params = parametros_padrao()
    res = config.CAMERA_RES
    piloto = Piloto(params, res)
    real = dict(params)
    if erro_cam:     # a camera de verdade nao esta exatamente como se mediu
        real["inclinacao"] += 2.0
        real["altura_cm"] += 1.5
        real["guinada"] += 1.5
        real["fov_h"] -= 3.0
    cam_real = CameraChao(real, res)
    rend = Renderizador(cam_real, res)
    carro = Carro(pista, **kw)
    cones_mundo = [c["w"] for c in pista.cones]

    dt = 1.0 / fps
    t = 0.0
    max_err = 0.0; max_fora = -1e9; min_folga = 1e9; sumd = 0.0; n = 0
    fim = pista.comp_total
    ok = True
    motivo = "completou"
    tempo_max = fim / (config.VELOCIDADE_MIN * config.V_CMS_NA_VEL_1) + 20
    eventos = []
    while True:
        s, d, fora, folga = pose_stats(pista, carro, cones_mundo)
        if s >= fim:
            break
        if t > tempo_max:
            ok, motivo = False, "tempo esgotado (parou?)"
            break
        frame = rend.quadro(pista, carro)
        turn, speed = piloto.step(t, frame, fps=fps, debug=ver or bool(salvar))
        eventos += [(t, e) for e in piloto.eventos]
        carro.passo(turn, speed, dt, fator_v)
        t += dt
        max_err = max(max_err, abs(d)); sumd += abs(d); n += 1
        max_fora = max(max_fora, fora); min_folga = min(min_folga, folga)
        if folga < -2.0:
            ok, motivo = False, f"BATEU no cone (s={s:.0f})"
            break
        if fora > 6.0:
            ok, motivo = False, f"saiu da pista (s={s:.0f}, d={d:+.0f})"
            break
        if ver or salvar:
            vis = _compor(piloto, frame)
            if ver:
                cv2.imshow("sim", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if salvar and n % 15 == 0:
                os.makedirs(salvar, exist_ok=True)
                cv2.imwrite(os.path.join(salvar, f"{nome}_{n:04d}.png"), vis)
    if not quiet:
        flag = "OK " if ok else "FALHA"
        print(f"[{flag}] {nome:18s} {motivo:34s} desvio max {max_err:5.1f}cm  "
              f"medio {sumd / max(n, 1):4.1f}cm  invasao max {max(max_fora, -99):+5.1f}cm  "
              f"folga cone min {('%.1f' % min_folga) if min_folga < 1e8 else '-':>5}cm  t={t:.1f}s")
        if "-v" in sys.argv:
            for te, e in eventos:
                print(f"       {te:5.1f}s {e}")
    return ok


def _compor(piloto, frame):
    import visual
    return visual.compor(piloto, frame)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    salvar = None
    if "--salvar" in sys.argv:
        i = sys.argv.index("--salvar")
        salvar = sys.argv[i + 1]
        args = [a for a in args if a != salvar]
    nomes = args or NOMES
    t0 = time.time()
    resultados = [rodar(n, ver="--ver" in sys.argv, erro_cam="--erro-cam" in sys.argv,
                        salvar=salvar) for n in nomes]
    print(f"\n{sum(resultados)}/{len(resultados)} cenarios OK em {time.time() - t0:.0f}s")
    sys.exit(0 if all(resultados) else 1)
