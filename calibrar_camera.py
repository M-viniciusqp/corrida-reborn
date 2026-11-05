"""
calibrar_camera.py - ajusta a geometria da camera DESTE carro (altura, angulo...)
olhando a vista de cima ao vivo. Rode no terminal da AREA DE TRABALHO do Pi.

    python3 calibrar_camera.py
    python3 calibrar_camera.py --camera 1
    python3 calibrar_camera.py --foto minha_foto.jpg     # usa uma foto (PC)

Como fazer:
  1. Ponha o carro NA FAIXA, parado, reto, no comeco da pista, com as linhas
     visiveis. (Se tiver um cone, ponha ele a ~40 cm a frente: bom pra conferir.)
  2. Janela da direita = vista de cima. As linhas da pista tem que aparecer
     RETAS, PARALELAS e na vertical. A grade e de 10 em 10 cm.
  3. Ajuste ate bater:
        w / s   inclinacao da camera +1 / -1 grau   (linhas abrindo em V -> mexa aqui)
        r / f   altura da camera       +1 / -1 cm
        t / g   campo de visao         +2 / -2 graus (largura da pista errada -> aqui)
        z / x   guinada (camera olhando de lado)  +0.5 / -0.5
        c / v   rolagem (horizonte torto)         +0.5 / -0.5
        j / l   camera deslocada do eixo  -1 / +1 cm   (carro "no centro" mas erro != 0)
        i / k   camera a frente do eixo dianteiro +1 / -1 cm
  4. A linha de cima mostra a distancia medida entre as linhas da pista:
     compare com FAIXA_LARGURA_CM (medida com trena). Clique em 2 pontos da
     vista de cima pra medir qualquer distancia em cm.
  5. 'p' grava em dados/camera.json (o programa passa a usar). 'q' sai.

     m   liga/desliga a mascara verde das linhas
"""

import argparse
import sys

import cv2
import numpy as np

import config
import geometria
import visual
from camera import abrir_camera, ajustar, CameraThread
from piloto import Piloto

PASSOS = {
    ord('w'): ("inclinacao", +1.0), ord('s'): ("inclinacao", -1.0),
    ord('r'): ("altura_cm", +1.0), ord('f'): ("altura_cm", -1.0),
    ord('t'): ("fov_h", +2.0), ord('g'): ("fov_h", -2.0),
    ord('z'): ("guinada", +0.5), ord('x'): ("guinada", -0.5),
    ord('c'): ("rolagem", +0.5), ord('v'): ("rolagem", -0.5),
    ord('j'): ("x_cm", -1.0), ord('l'): ("x_cm", +1.0),
    ord('i'): ("y_cm", +1.0), ord('k'): ("y_cm", -1.0),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=None)
    ap.add_argument("--foto", default=None)
    a = ap.parse_args()

    params, origem = geometria.carregar_parametros()
    print(f"Parametros iniciais ({origem}).")
    foto = None
    thread = None
    if a.foto:
        foto = cv2.imread(a.foto)
        if foto is None:
            print("Nao consegui abrir a foto.")
            return
    else:
        cap = abrir_camera(a.camera)
        if not cap.isOpened():
            print("Camera nao abriu.")
            return
        thread = CameraThread(cap)

    cliques = []
    estado = {"mascara": True, "bev_x0": 0}

    def mouse(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN and x >= estado["bev_x0"]:
            cliques.append((x - estado["bev_x0"], y))
            del cliques[:-2]
    cv2.namedWindow("calibrar")
    cv2.setMouseCallback("calibrar", mouse)

    piloto = None
    while True:
        if piloto is None:
            try:
                piloto = Piloto(params, config.CAMERA_RES)
                erro_geo = None
            except ValueError as e:
                erro_geo = str(e)
        if thread is not None:
            frame, _, _ = thread.read()
            if frame is None:
                cv2.waitKey(10)
                continue
        else:
            frame = foto
        frame = ajustar(frame)

        if piloto is not None:
            piloto.step(0.1, frame, fps=15.0, debug=True)
            if not estado["mascara"]:
                piloto.res.mascara = None
            vis = visual.compor(piloto, frame)
            estado["bev_x0"] = frame.shape[1]
            cam = piloto.cam
            info = _largura_pista(piloto)
            vis = np.vstack([np.zeros((34, vis.shape[1], 3), np.uint8), vis])
            visual._txt(vis, f"incl {params['inclinacao']:.0f}  altura {params['altura_cm']:.0f}cm  "
                             f"fov {params['fov_h']:.0f}  guin {params['guinada']:.1f}  "
                             f"rol {params['rolagem']:.1f}  x {params['x_cm']:.0f}  y {params['y_cm']:.0f}",
                        (6, 13), (255, 255, 255), 0.45)
            visual._txt(vis, info, (6, 28), (0, 255, 255), 0.45)
            for (cx, cy) in cliques:
                cv2.circle(vis, (cx + estado["bev_x0"], cy + 34), 4, (0, 0, 255), -1)
            if len(cliques) == 2:
                p0 = cam.bev_para_chao(*cliques[0])
                p1 = cam.bev_para_chao(*cliques[1])
                d = float(np.hypot(p1[0] - p0[0], p1[1] - p0[1]))
                dx = abs(p1[0] - p0[0])
                visual._txt(vis, f"clique: distancia {d:.1f}cm (lateral {dx:.1f}cm)",
                            (6, vis.shape[0] - 8), (0, 200, 255), 0.5)
                cv2.line(vis, (cliques[0][0] + estado["bev_x0"], cliques[0][1] + 34),
                         (cliques[1][0] + estado["bev_x0"], cliques[1][1] + 34), (0, 0, 255), 1)
        else:
            vis = np.zeros((120, 480, 3), np.uint8)
            visual._txt(vis, "geometria invalida: " + str(erro_geo), (6, 40), (0, 0, 255), 0.45)
            visual._txt(vis, "mexa em w/s (inclinacao) ou r/f (altura)", (6, 70), (255, 255, 255), 0.45)
        cv2.imshow("calibrar", vis)

        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break
        if k == ord('m'):
            estado["mascara"] = not estado["mascara"]
        elif k == ord('p'):
            geometria.salvar_parametros(params)
            print(f"Gravado em {geometria.ARQ_CAMERA}: {params}")
        elif k in PASSOS:
            chave, passo = PASSOS[k]
            params[chave] = params[chave] + passo
            params["fov_h"] = max(30.0, min(140.0, params["fov_h"]))
            params["altura_cm"] = max(3.0, params["altura_cm"])
            piloto = None
            cliques.clear()
    if thread:
        thread.release()
    cv2.destroyAllWindows()


def _largura_pista(piloto):
    """Distancia (cm) entre linhas vizinhas identificadas, comparada com a esperada."""
    res = piloto.res
    if res is None or not res.linhas:
        return "nenhuma linha detectada (a pista esta visivel? a mascara verde pega as linhas?)"
    y = piloto.faixas._y_ref()
    por_id = {l.ident: float(l.x(y)) for l in res.linhas if l.ident}
    partes = []
    if "D" in por_id and "A" in por_id:
        partes.append(f"pontilhada->continua {por_id['A'] - por_id['D']:.1f}cm")
    if "B" in por_id and "D" in por_id:
        partes.append(f"continua->pontilhada {por_id['D'] - por_id['B']:.1f}cm")
    nomes = " ".join(f"{k}:{v:+.0f}" for k, v in sorted(por_id.items()))
    esperado = f"(esperado {config.FAIXA_LARGURA_CM:.0f}cm)"
    if not partes:
        return f"linhas {nomes or '?'} - preciso ver 2 linhas pra medir a largura {esperado}"
    return " | ".join(partes) + f" {esperado} | linhas {nomes}"


if __name__ == "__main__":
    main()
