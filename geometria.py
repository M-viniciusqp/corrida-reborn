"""
geometria.py - modelo da camera e conversao imagem <-> chao (em cm).

A camera deste carro tem outro angulo, entao nada aqui assume "imagem reta".
Dados a altura, a inclinacao, o campo de visao (e opcionalmente guinada,
rolagem e deslocamento), o modelo calcula pra QUALQUER pixel abaixo do
horizonte onde ele cai no chao, em cm, no referencial do carro:

    X = pra direita     Y = pra frente     origem = chao sob o eixo dianteiro

Com isso:
  * a vista de cima (BEV) e so uma reamostragem da imagem no chao;
  * a posicao de um cone sai da base dele na imagem (sem estimar por tamanho);
  * erro lateral e proa saem direto em cm e radianos.

Parametros: valores do config.py, sobrescritos por dados/camera.json
(gravado por calibrar_camera.py).
"""

import json
import math
import os

import cv2
import numpy as np

import config

PASTA_DADOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dados")
ARQ_CAMERA = os.path.join(PASTA_DADOS, "camera.json")

CHAVES = ("altura_cm", "inclinacao", "guinada", "rolagem", "x_cm", "y_cm", "fov_h")


def parametros_padrao():
    return {
        "altura_cm": config.CAM_ALTURA_CM, "inclinacao": config.CAM_INCLINACAO,
        "guinada": config.CAM_GUINADA, "rolagem": config.CAM_ROLAGEM,
        "x_cm": config.CAM_X_CM, "y_cm": config.CAM_Y_CM, "fov_h": config.CAM_FOV_H,
    }


def carregar_parametros():
    """config.py + dados/camera.json por cima. Devolve (dict, origem)."""
    p = parametros_padrao()
    origem = "config.py"
    try:
        with open(ARQ_CAMERA, "r", encoding="utf-8") as f:
            salvo = json.load(f)
        for k in CHAVES:
            if k in salvo:
                p[k] = float(salvo[k])
        origem = "dados/camera.json"
    except (OSError, ValueError):
        pass
    return p, origem


def salvar_parametros(p):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    with open(ARQ_CAMERA, "w", encoding="utf-8") as f:
        json.dump({k: float(p[k]) for k in CHAVES}, f, indent=2)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


class CameraChao:
    """Camera pinhole olhando o chao. Tamanho da imagem = res (largura, altura)."""

    def __init__(self, params=None, res=None):
        if params is None:
            params, _ = carregar_parametros()
        self.p = dict(params)
        self.res = tuple(res or config.CAMERA_RES)
        w, h = self.res
        self.w, self.h = w, h
        self.cx, self.cy = (w - 1) / 2.0, (h - 1) / 2.0
        self.f = (w / 2.0) / math.tan(math.radians(self.p["fov_h"]) / 2.0)
        self.cam = np.array([self.p["x_cm"], self.p["y_cm"], self.p["altura_cm"]], dtype=np.float64)

        # camera -> mundo. Camera: x direita, y baixo, z frente. Mundo: X dir, Y frente, Z cima.
        base = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)  # colunas = eixos da cam no mundo
        a = math.radians(self.p["rolagem"])
        c, s = math.cos(a), math.sin(a)
        roll = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)   # em torno do eixo z da camera
        pitch = _rot_x(-math.radians(self.p["inclinacao"]))                     # inclina a frente pra baixo
        yaw = _rot_z(math.radians(self.p["guinada"]))
        self.R = yaw @ pitch @ base @ roll        # camera -> mundo
        self.Rt = self.R.T

        self._montar_homografias()

    # ------------------------------------------------------------------
    def pixel_para_chao(self, u, v):
        """(u, v) pixel -> (X, Y) em cm. Devolve None se o raio nao toca o chao
        (acima do horizonte)."""
        raio = self.R @ np.array([(u - self.cx) / self.f, (v - self.cy) / self.f, 1.0])
        if raio[2] >= -1e-6:
            return None
        t = -self.cam[2] / raio[2]
        return (self.cam[0] + t * raio[0], self.cam[1] + t * raio[1])

    def ponto3d_para_pixel(self, X, Y, Z):
        """Ponto (X, Y, Z) em cm (Z = altura acima do chao) -> (u, v). None se atras."""
        c = self.Rt @ np.array([X - self.cam[0], Y - self.cam[1], Z - self.cam[2]])
        if c[2] <= 1e-6:
            return None
        return (self.cx + self.f * c[0] / c[2], self.cy + self.f * c[1] / c[2])

    def chao_para_pixel(self, X, Y):
        """(X, Y) cm no chao -> (u, v) pixel. None se atras da camera."""
        return self.ponto3d_para_pixel(X, Y, 0.0)

    def linha_horizonte(self):
        """Fileira (v) do horizonte; negativa/ > h se fora da imagem."""
        # raio com direcao horizontal pra frente (mundo): (0,1,0)
        c = self.Rt @ np.array([0.0, 1.0, 0.0])
        if c[2] <= 1e-6:
            return None
        return self.cy + self.f * c[1] / c[2]

    # ------------------------------------------------------------------
    def _faixa_visivel(self):
        """Y minimo/maximo do chao que a imagem enxerga (coluna central)."""
        pct = config.BEV_TOPO_PCT
        v_topo = pct * (self.h - 1)
        hor = self.linha_horizonte()
        if hor is not None:
            v_topo = max(v_topo, hor + 0.06 * self.h)     # fica longe do horizonte (explode)
        p_baixo = self.pixel_para_chao(self.cx, self.h - 1)
        p_topo = self.pixel_para_chao(self.cx, min(v_topo, self.h - 3))
        if p_baixo is None or p_topo is None:
            raise ValueError("A camera nao enxerga o chao: confira CAM_INCLINACAO/ALTURA.")
        return p_baixo[1], p_topo[1]

    def _montar_homografias(self):
        y_min, y_max = self._faixa_visivel()
        if config.BEV_Y_MIN_CM is not None:
            y_min = config.BEV_Y_MIN_CM
        if config.BEV_Y_MAX_CM is not None:
            y_max = config.BEV_Y_MAX_CM
        if y_max <= y_min + 5:
            raise ValueError("Faixa de chao visivel vazia: ajuste a inclinacao da camera.")
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.x_max = float(config.BEV_X_MAX_CM)
        self.px_cm = float(config.BEV_PX_POR_CM)
        self.bev_w = int(round(2 * self.x_max * self.px_cm))
        self.bev_h = int(round((self.y_max - self.y_min) * self.px_cm))

        # cantos da BEV no chao -> pixels da imagem
        cantos_chao = [(-self.x_max, self.y_max), (self.x_max, self.y_max),
                       (self.x_max, self.y_min), (-self.x_max, self.y_min)]
        img = []
        for (X, Y) in cantos_chao:
            pt = self.chao_para_pixel(X, Y)
            if pt is None:
                raise ValueError("Canto da BEV atras da camera: reduza BEV_Y_MIN_CM.")
            img.append(pt)
        img = np.array(img, dtype=np.float32)
        bev = np.array([[0, 0], [self.bev_w - 1, 0],
                        [self.bev_w - 1, self.bev_h - 1], [0, self.bev_h - 1]], dtype=np.float32)
        self.H_img_bev = cv2.getPerspectiveTransform(img, bev)       # imagem -> BEV
        self.H_bev_img = np.linalg.inv(self.H_img_bev)
        # mascara do que a imagem realmente cobre dentro da BEV
        branco = np.full((self.h, self.w), 255, np.uint8)
        self.mascara_bev = cv2.warpPerspective(branco, self.H_img_bev, (self.bev_w, self.bev_h),
                                               flags=cv2.INTER_NEAREST) > 0

    # ------------------------------------------------------------------
    def para_bev(self, img, interp=cv2.INTER_LINEAR, borda=cv2.BORDER_CONSTANT):
        return cv2.warpPerspective(img, self.H_img_bev, (self.bev_w, self.bev_h),
                                   flags=interp, borderMode=borda)

    # conversao BEV <-> chao (cm)
    def bev_para_chao(self, bu, bv):
        X = bu / self.px_cm - self.x_max
        Y = self.y_max - bv / self.px_cm
        return X, Y

    def chao_para_bev(self, X, Y):
        return (X + self.x_max) * self.px_cm, (self.y_max - Y) * self.px_cm

    def y_chao_de_linha_bev(self, bv):
        return self.y_max - bv / self.px_cm

    def largura_chao_em_pixels(self, u, v, largura_px):
        """Largura (cm) no chao de um segmento horizontal de largura_px pixels
        com centro em (u, v). None se acima do horizonte."""
        a = self.pixel_para_chao(u - largura_px / 2.0, v)
        b = self.pixel_para_chao(u + largura_px / 2.0, v)
        if a is None or b is None:
            return None
        return abs(b[0] - a[0])

    def resumo(self):
        hor = self.linha_horizonte()
        return (f"camera: altura {self.p['altura_cm']:.0f}cm inclin {self.p['inclinacao']:.0f}deg "
                f"fov {self.p['fov_h']:.0f}deg | chao visivel Y {self.y_min:.0f}..{self.y_max:.0f}cm "
                f"| horizonte v={hor if hor is None else round(hor)} | BEV {self.bev_w}x{self.bev_h}px")
