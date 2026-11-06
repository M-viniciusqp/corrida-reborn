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


def _abrir_indice(indice, res, fps):
    if sys.platform.startswith("linux"):
        cap = cv2.VideoCapture(indice, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(indice)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # MJPG: bem mais leve no USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def _procurar_outra(preferida, res, fps):
    """Depois de uma queda de energia/USB a webcam costuma voltar com OUTRO numero (video2 em vez de
    video0). Tenta os /dev/video* que existem e fica com o primeiro que entrega imagem de verdade."""
    import glob
    import re
    nums = sorted(int(m.group(1)) for p in glob.glob("/dev/video*")
                  if (m := re.search(r"video(\d+)$", p)))
    candidatos = [n for n in nums if n != preferida and n < 20]
    if not candidatos:
        return None
    try:                                   # silencia os avisos do OpenCV nos que nao sao camera
        nivel = cv2.utils.logging.getLogLevel()
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except Exception:
        nivel = None
    achada = None
    try:
        for n in candidatos:
            cap = _abrir_indice(n, res, fps)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None and frame.ndim == 3 and frame.shape[1] >= 160:
                    achada = (n, cap)
                    break
            cap.release()
    finally:
        if nivel is not None:
            cv2.utils.logging.setLogLevel(nivel)
    if achada:
        print(f"AVISO: a camera {preferida} nao abre; usando /dev/video{achada[0]} (que entrega imagem). "
              f"Se isso se repetir, ponha CAMERA_ID = {achada[0]} no config.py.")
        return achada[1]
    return None


def abrir_camera(fonte=None, res=None, fps=None):
    """fonte: numero da camera (0, 1...) ou caminho de um video/foto. None = config."""
    res = res or config.CAMERA_CAPTURA_RES
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
    cap = _abrir_indice(indice, res, fps)
    if not cap.isOpened():
        cap = _procurar_outra(indice, res, fps) or cap       # a camera pode ter mudado de numero
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fcc = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
    fcc = "".join(c if 32 <= ord(c) < 127 else "?" for c in fcc)       # formato invalido nao pode quebrar o print
    print(f"Camera {indice}: {w}x{h} formato={fcc.strip()}")
    pw, ph = config.CAMERA_RES
    if h and abs(w / h - pw / ph) > 0.05:
        print(f"AVISO: a camera entrega {w}x{h} (proporcao {w / h:.2f}) mas o piloto usa "
              f"{pw}x{ph} ({pw / ph:.2f}). A imagem sera ACHATADA e a geometria erra. "
              f"Ajuste CAMERA_RES (e CAMERA_CAPTURA_RES) no config.py para a mesma proporcao.")
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
    """Le a camera em segundo plano. Se ela parar de entregar quadros (cabo, falta de energia,
    USB), FECHA e REABRE sozinha em vez de ficar parada pra sempre."""

    def __init__(self, cap, repetir_arquivo=True, reabrir=None, falhas_max=3, silencio_max_s=2.0):
        self.cap = cap
        self._frame = None
        self._idx = 0
        self._t = 0.0
        self._lock = threading.Lock()
        self._rodando = True
        self._repetir = repetir_arquivo
        self._reabrir = reabrir            # funcao que devolve uma camera nova (ou None = nao reabre)
        self._falhas_max = falhas_max
        self._silencio_max = silencio_max_s
        self.reaberturas = 0
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _tentar_reabrir(self):
        if self._reabrir is None:
            return False
        print("AVISO: a camera parou de entregar quadros (energia? USB? outro programa usando?). "
              "Reabrindo...")
        try:
            self.cap.release()
        except Exception:
            pass
        time.sleep(0.6)
        novo = self._reabrir()
        if novo is not None and novo.isOpened():
            self.cap = novo
            self.reaberturas += 1
            return True
        return False

    def _loop(self):
        falhas = 0
        t_ok = time.time()
        while self._rodando:
            ok, frame = self.cap.read()
            if not ok:
                if self._repetir and self.cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    time.sleep(0.01)
                    continue
                falhas += 1
                if falhas >= self._falhas_max or (time.time() - t_ok) > self._silencio_max:
                    if self._tentar_reabrir():
                        falhas = 0
                        t_ok = time.time()
                    else:
                        time.sleep(1.0)
                else:
                    time.sleep(0.02)
                continue
            falhas = 0
            t_ok = time.time()
            with self._lock:
                self._frame = frame
                self._idx += 1
                self._t = t_ok

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
