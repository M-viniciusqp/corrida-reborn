"""
visual.py - desenha o que o piloto esta pensando (janela --ver, fotos de log, simulador).
"""

import cv2
import numpy as np

import config

COR_IDENT = {"A": (0, 200, 255), "D": (255, 160, 0), "B": (200, 100, 255), None: (140, 140, 140)}


def _txt(img, texto, pos, cor=(255, 255, 255), esc=0.45):
    cv2.putText(img, texto, pos, cv2.FONT_HERSHEY_SIMPLEX, esc, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, texto, pos, cv2.FONT_HERSHEY_SIMPLEX, esc, cor, 1, cv2.LINE_AA)


def desenhar_bev(piloto, escala=1.0):
    cam, res = piloto.cam, piloto.res
    if res is None or res.bev is None:
        return np.zeros((cam.bev_h, cam.bev_w, 3), np.uint8)
    img = res.bev.copy()
    if res.mascara is not None:
        img[res.mascara > 0] = (0.4 * img[res.mascara > 0] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)
    # grade a cada 10 cm
    for X in range(-int(cam.x_max) // 10 * 10, int(cam.x_max) + 1, 10):
        u, _ = cam.chao_para_bev(X, 0)
        cv2.line(img, (int(u), 0), (int(u), cam.bev_h), (70, 70, 70), 1)
    y = int(np.ceil(cam.y_min / 10.0) * 10)
    while y <= cam.y_max:
        _, v = cam.chao_para_bev(0, y)
        cv2.line(img, (0, int(v)), (cam.bev_w, int(v)), (70, 70, 70), 1)
        _txt(img, f"{y}", (2, int(v) - 2), (180, 180, 180), 0.35)
        y += 10
    Ys = np.linspace(cam.y_min, cam.y_max, 30)
    for l in res.linhas:
        pts = [cam.chao_para_bev(float(x), float(yy)) for x, yy in zip(l.x(Ys), Ys)]
        pts = np.array(pts, np.int32)
        cv2.polylines(img, [pts], False, COR_IDENT.get(l.ident), 2, cv2.LINE_AA)
        u, v = cam.chao_para_bev(float(l.x(Ys[len(Ys) // 2])), float(Ys[len(Ys) // 2]))
        rot = (l.ident or "?") + ("" if l.cob_ema is None else f" {l.cob_ema:.2f}")
        _txt(img, rot, (int(u) + 4, int(v)), COR_IDENT.get(l.ident), 0.38)
    if res.centro is not None:
        pts = np.array([cam.chao_para_bev(float(res.centro.x(yy)), float(yy)) for yy in Ys], np.int32)
        cv2.polylines(img, [pts], False, (0, 255, 0), 2, cv2.LINE_AA)
        o = piloto.plan.o_cmd
        pts2 = np.array([cam.chao_para_bev(float(res.centro.x(yy)) + o, float(yy)) for yy in Ys], np.int32)
        cv2.polylines(img, [pts2], False, (0, 255, 255), 2, cv2.LINE_AA)
    for c in piloto.cones:
        u, v = cam.chao_para_bev(c.x, c.y)
        r = int(config.CONE_LARGURA_CM / 2 * cam.px_cm)
        cv2.circle(img, (int(u), int(v)), r, (0, 100, 255), 2, cv2.LINE_AA)
        _txt(img, f"#{c.id} {c.y:.0f}cm", (int(u) + r + 2, int(v)), (0, 160, 255), 0.38)
    # carro (retangulo na posicao X=0, Y=0..-comprimento, quando visivel)
    u0, v0 = cam.chao_para_bev(0, 0)
    cv2.line(img, (int(u0), cam.bev_h - 1), (int(u0), cam.bev_h - 14), (255, 255, 255), 2)
    if escala != 1.0:
        img = cv2.resize(img, None, fx=escala, fy=escala)
    return img


def desenhar_camera(piloto, frame):
    cam = piloto.cam
    img = frame.copy()
    hor = cam.linha_horizonte()
    if hor is not None and 0 <= hor < cam.h:
        cv2.line(img, (0, int(hor)), (cam.w, int(hor)), (255, 255, 0), 1)
    for c in piloto.cones:
        x, y, w, h = c.bbox
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        _txt(img, f"#{c.id} x{c.x:+.0f} y{c.y:.0f}", (x, max(10, y - 4)), (0, 255, 255), 0.4)
    res = piloto.res
    if res is not None and res.centro is not None:
        Ys = np.linspace(cam.y_min, cam.y_max, 20)
        pts = []
        for yy in Ys:
            p = cam.chao_para_pixel(float(res.centro.x(yy) + piloto.plan.o_cmd), float(yy))
            if p is not None:
                pts.append(p)
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 255, 255), 2, cv2.LINE_AA)
    return img


def compor(piloto, frame):
    """Camera (com marcas) | BEV, mais uma linha de texto com o estado."""
    cam_img = desenhar_camera(piloto, frame)
    bev = desenhar_bev(piloto)
    h = max(cam_img.shape[0], bev.shape[0])
    def preenche(im):
        if im.shape[0] < h:
            im = np.vstack([im, np.zeros((h - im.shape[0], im.shape[1], 3), np.uint8)])
        return im
    mosaico = np.hstack([preenche(cam_img), preenche(bev)])
    estado = (f"erro {piloto.erro_cm:+.1f}cm proa {piloto.proa_graus:+.1f} turn {piloto.turn:+.2f} "
              f"vel {piloto.speed:.2f} desvio {piloto.plan.o_cmd:+.1f}cm [{piloto.plan.estado}]"
              + ("" if piloto.motivo == "ok" else f" | {piloto.motivo}"))
    _txt(mosaico, estado, (6, 14), (0, 255, 255))
    return mosaico
