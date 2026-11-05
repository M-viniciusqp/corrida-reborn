"""
cones.py - acha cones na imagem e converte a BASE deles em posicao no chao (cm).

Diferente do gatuno-rapido (que estimava distancia pela altura do cone e
precisava de uma segunda camera), aqui a posicao vem da geometria da camera:
o ponto mais baixo do cone na imagem toca o chao, e o modelo da camera
(geometria.py) diz onde esse pixel esta em cm. Dois testes de coerencia
descartam falso positivo:
  * a largura do cone, medida no chao, tem que bater com CONE_LARGURA_CM;
  * o cone tem que aparecer em quadros seguidos no mesmo lugar.
"""

import cv2
import numpy as np

import config


class Cone:
    __slots__ = ("x", "y", "largura", "truncado", "bbox", "id", "vistos",
                 "perdido", "lado")

    def __init__(self, x, y, largura, truncado, bbox):
        self.x, self.y, self.largura = x, y, largura
        self.truncado = truncado
        self.bbox = bbox
        self.id = -1
        self.vistos = 1
        self.perdido = 0
        self.lado = None        # decisao do planejador (congelada por cone)


class DetectorCones:
    def __init__(self, cam):
        self.cam = cam
        self.mascara = None      # debug
        self._k_abre = np.ones((3, 3), np.uint8)
        self._k_fecha = np.ones((5, 5), np.uint8)

    def detectar(self, frame, debug=False):
        cam = self.cam
        h_img, w_img = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = (((h <= config.CONE_H_BAIXO) | (h >= config.CONE_H_ALTO)) &
             (s >= config.CONE_S_MIN) & (v >= config.CONE_V_MIN)).astype(np.uint8)
        topo = int(config.CONE_TOPO_PCT * h_img)
        hor = cam.linha_horizonte()
        if hor is not None:
            topo = max(topo, int(hor))      # nada acima do horizonte toca o chao
        m[:topo, :] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._k_abre)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._k_fecha)
        if debug:
            self.mascara = m

        escala = (w_img * h_img) / (320.0 * 240.0)
        area_min = config.CONE_AREA_MIN_PX * escala
        n, rot, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        achados = []
        for i in range(1, n):
            x, y, w, hh, area = stats[i]
            if area < area_min or w < 3 or hh < 3:
                continue
            aspecto = hh / float(w)
            if not (config.CONE_ASPECTO_MIN <= aspecto <= config.CONE_ASPECTO_MAX):
                continue
            if area / float(w * hh) < config.CONE_PREENCHIMENTO_MIN:
                continue
            truncado = (y + hh) >= h_img - 2
            u_base = x + w / 2.0
            v_base = float(y + hh)
            pt = cam.pixel_para_chao(u_base, v_base)
            if pt is None:
                continue
            larg = cam.largura_chao_em_pixels(u_base, v_base, float(w))
            if larg is None:
                continue
            razao = larg / config.CONE_LARGURA_CM
            lo, hi = config.CONE_LARG_TOL
            if not truncado and not (lo <= razao <= hi):
                continue
            # a borda de baixo do cone e a borda da frente da base: o centro do
            # cone fica um pouco mais longe (CONE_BASE_OFFSET_CM)
            Y = pt[1] + (0.0 if truncado else config.CONE_BASE_OFFSET_CM)
            achados.append(Cone(pt[0], Y, larg, truncado, (int(x), int(y), int(w), int(hh))))
        return achados


class RastreadorCones:
    """Associa deteccoes entre quadros, suaviza e confirma."""

    def __init__(self):
        self.cones = []
        self._prox_id = 1

    def atualizar(self, deteccoes):
        a = config.CONE_SUAVIZACAO
        usados = set()
        for c in self.cones:
            melhor, mi = 1e9, None
            for j, d in enumerate(deteccoes):
                if j in usados:
                    continue
                dist = np.hypot(d.x - c.x, d.y - c.y)
                if dist < melhor:
                    melhor, mi = dist, j
            if mi is not None and melhor < config.CONE_CONFIRMA_DIST_CM * 2.0:
                d = deteccoes[mi]
                usados.add(mi)
                c.x = a * d.x + (1 - a) * c.x
                c.y = a * d.y + (1 - a) * c.y
                c.largura = d.largura
                c.truncado = d.truncado
                c.bbox = d.bbox
                c.vistos += 1
                c.perdido = 0
            else:
                c.perdido += 1
        for j, d in enumerate(deteccoes):
            if j not in usados:
                d.id = self._prox_id
                self._prox_id += 1
                self.cones.append(d)
        self.cones = [c for c in self.cones if c.perdido <= config.CONE_PERDE_QUADROS]
        return self.confirmados()

    def confirmados(self):
        return sorted([c for c in self.cones
                       if c.vistos >= config.CONE_CONFIRMA_QUADROS and c.perdido == 0],
                      key=lambda c: c.y)

    def recentes(self):
        """Cones confirmados que sumiram ha pouco (ainda no rastreio)."""
        return [c for c in self.cones
                if c.vistos >= config.CONE_CONFIRMA_QUADROS and c.perdido > 0]
