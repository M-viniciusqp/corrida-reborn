"""
camera.py - abre a camera (ou um arquivo de video/foto pra testar no PC) e
guarda sempre o quadro MAIS NOVO numa thread: o laco principal nunca le
quadro velho do buffer da camera.
"""

import os
import sys
import threading
import time

import cv2

import config


class _CapFoto:
    """Faz uma foto parada se comportar como camera (pra testar no PC)."""

    def __init__(self, caminho):
        self.img = cv2.imread(caminho)

    def isOpened(self):
        return self.img is not None

    def read(self):
        time.sleep(1 / 30.0)
        return self.img is not None, (None if self.img is None else self.img.copy())

    def get(self, _):
        return 0

    def set(self, *_):
        return True

    def release(self):
        pass


def abrir_camera(fonte=None, res=None, fps=None):
    """fonte: numero da camera (0, 1...) ou caminho de um video/foto. None = config."""
    res = res or config.CAMERA_RES
    fps = fps or config.CAMERA_FPS
    fonte = config.CAMERA_ID if fonte is None else fonte
    if isinstance(fonte, str) and fonte.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
        print(f"Fonte de imagem: foto {fonte}")
        return _CapFoto(fonte)
    if isinstance(fonte, str) and not fonte.isdigit():
        cap = cv2.VideoCapture(fonte)
        print(f"Fonte de imagem: arquivo {fonte}")
        return cap
    indice = int(fonte)
    if sys.platform.startswith("linux"):
        cap = cv2.VideoCapture(indice, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(indice)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # MJPG: bem mais leve no USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fcc = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
    print(f"Camera {indice}: {w}x{h} formato={fcc.strip()}")
    return cap


def ajustar(frame, res=None):
    """Aplica espelho/giro do config e redimensiona pro tamanho que o piloto espera."""
    res = res or config.CAMERA_RES
    if config.CAMERA_GIRAR_180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    if config.CAMERA_ESPELHAR:
        frame = cv2.flip(frame, 1)
    if (frame.shape[1], frame.shape[0]) != tuple(res):
        frame = cv2.resize(frame, tuple(res), interpolation=cv2.INTER_AREA)
    return frame


class CameraThread:
    def __init__(self, cap, repetir_arquivo=True):
        self.cap = cap
        self._frame = None
        self._idx = 0
        self._t = 0.0
        self._lock = threading.Lock()
        self._rodando = True
        self._repetir = repetir_arquivo
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while self._rodando:
            ok, frame = self.cap.read()
            if not ok:
                if self._repetir and self.cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
                self._idx += 1
                self._t = time.time()

    def read(self):
        """(quadro, numero, idade_em_s). quadro None = ainda nao chegou nenhum."""
        with self._lock:
            if self._frame is None:
                return None, 0, float("inf")
            return self._frame, self._idx, time.time() - self._t

    def release(self):
        self._rodando = False
        self._th.join(timeout=1.0)
        self.cap.release()
