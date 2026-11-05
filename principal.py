"""
principal.py - loop do carro autonomo (reborn-autonomos).

    python3 principal.py --seco --ver     # 1o teste: NAO move nada, so mostra o que decidiria
    python3 principal.py --tempo 10       # anda 10 s e para sozinho
    python3 principal.py                  # anda ate Ctrl+C (ou 'q' na janela)

Opcoes:
    --seco        motores e servo parados (so visao e log)
    --ver         abre as janelas no PROPRIO Raspberry (terminal da area de trabalho)
    --tempo S     para sozinho depois de S segundos
    --camera N    numero da camera (ou caminho de um video, pra testar no PC)
    --sem-log     nao grava a pasta logs/
    --simulado    GPIO de mentira (testa o programa sem o carro)

Este arquivo so faz entrada/saida: le a camera, entrega ao Piloto (piloto.py),
manda (turn, speed) pro servo e pros motores. Qualquer problema (quadro velho,
erro, Ctrl+C) para os motores e centraliza o servo.
"""

import argparse
import os
import time

import cv2

import config
import hardware
from camera import abrir_camera, ajustar, CameraThread
from geometria import carregar_parametros
from piloto import Piloto
from registro import Registro
import visual


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seco", action="store_true")
    ap.add_argument("--ver", action="store_true")
    ap.add_argument("--tempo", type=float, default=0.0)
    ap.add_argument("--camera", default=None)
    ap.add_argument("--sem-log", action="store_true")
    ap.add_argument("--simulado", action="store_true")
    a = ap.parse_args()

    ver = a.ver
    if ver and os.name != "nt" and not os.environ.get("DISPLAY"):
        print("Sem tela neste terminal (DISPLAY vazio): janelas desligadas. "
              "Rode pelo terminal da area de trabalho do Pi.")
        ver = False

    params, origem = carregar_parametros()
    piloto = Piloto(params, config.CAMERA_RES)
    print(f"Geometria da camera ({origem}):\n  {piloto.cam.resumo()}")

    cap = abrir_camera(a.camera)
    if not cap.isOpened():
        print("Erro: a camera nao abriu (v4l2-ctl --list-devices).")
        return
    thread = CameraThread(cap)

    servo = motores = None
    if not a.seco:
        servo = hardware.Servo(simulado=a.simulado)
        motores = hardware.Motores(simulado=a.simulado)
        print(f"Servo ({servo.modo}): centro {servo.cal['centro']:.0f} "
              f"esq {servo.cal['esquerda']:.0f} dir {servo.cal['direita']:.0f}  [{servo.origem}]")
        if not hardware.hardware_real():
            print("(hardware simulado: nada vai se mexer)")
    else:
        print("MODO SECO: motores e servo desligados.")

    registro = None if a.sem_log else Registro()
    if registro:
        print(f"Log em {registro.pasta}")

    ultimo_idx = -1
    t_ini = time.time()
    n, fps_medido = 0, 15.0
    t_fps, frames = time.time(), 0
    print("Rodando. Ctrl+C (ou 'q' na janela) para parar.")
    try:
        while True:
            frame, idx, idade = thread.read()
            if frame is None or idx == ultimo_idx:
                time.sleep(0.003)
                continue
            ultimo_idx = idx
            if idade > config.QUADRO_VELHO_S:
                if motores:
                    motores.parar()
                continue
            frame = ajustar(frame)

            agora = time.time()
            n += 1
            mostrar = ver and (n % config.DEBUG_CADA == 0)
            turn, speed = piloto.step(agora, frame, fps=fps_medido,
                                      debug=mostrar or registro is not None)
            if servo:
                servo.virar(turn)
                motores.andar(speed)

            for linha in piloto.eventos:
                print("  " + linha)
                if registro:
                    registro.evento(linha)
                    registro.foto(linha.split("]")[0].strip("[ ") if "]" in linha else "ev",
                                  visual.compor(piloto, frame) if piloto.res.bev is not None else None)
            if registro:
                registro.telemetria(piloto, fps_medido)

            frames += 1
            if time.time() - t_fps >= 2.0:
                fps_medido = frames / (time.time() - t_fps)
                frames, t_fps = 0, time.time()
                _resumo(piloto, fps_medido)

            if mostrar:
                cv2.imshow("reborn-autonomos", visual.compor(piloto, frame))
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
            if a.tempo and time.time() - t_ini >= a.tempo:
                print(f"Tempo de {a.tempo:.0f}s acabou.")
                break
    except KeyboardInterrupt:
        print("\nParada de emergencia (Ctrl+C).")
    finally:
        if motores:
            motores.fechar()
        if servo:
            servo.fechar()
        hardware.limpar()
        thread.release()
        if registro:
            registro.fechar()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        print("Desligado com seguranca.")


def _resumo(p, fps):
    r = p.res
    linhas = ""
    if r is not None and r.linhas:
        linhas = " ".join(f"{l.ident or '?'}:{l.x_ref:+.0f}" for l in r.linhas)
    cones = " ".join(f"#{c.id}({c.x:+.0f},{c.y:.0f})" for c in p.cones) or "-"
    print(f"FPS {fps:4.1f} | {'OK' if p.motivo == 'ok' else 'PERDIDO: ' + p.motivo:<24s} | "
          f"erro {p.erro_cm:+6.1f}cm proa {p.proa_graus:+5.1f} | turn {p.turn:+.2f} vel {p.speed:.2f} "
          f"| desvio {p.plan.o_cmd:+5.1f}cm [{p.plan.estado}] | linhas {linhas or '-'} | cones {cones}")


if __name__ == "__main__":
    main()
