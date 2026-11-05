"""
registro.py - grava logs/<data>/ pra analisar depois de uma corrida:
  eventos.txt    o que o piloto decidiu (desvio, perdeu faixa...)
  telemetria.csv uma linha por quadro
  fotos/         imagem camera|vista-de-cima a cada evento importante
Mande essa pasta inteira pra analise em vez de prints do terminal.
"""

import csv
import os
import time

import cv2

import config


class Registro:
    def __init__(self, max_fotos=60):
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.LOG_PASTA)
        self.pasta = os.path.join(base, time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.join(self.pasta, "fotos"), exist_ok=True)
        self._ev = open(os.path.join(self.pasta, "eventos.txt"), "w", encoding="utf-8")
        self._csv_f = open(os.path.join(self.pasta, "telemetria.csv"), "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(["t", "fps", "erro_cm", "proa_graus", "desvio_cm", "turn", "speed",
                            "linhas", "cones", "perdidos", "motivo"])
        self._t0 = time.time()
        self._fotos = 0
        self._max_fotos = max_fotos
        self._n = 0

    def evento(self, texto):
        self._ev.write(f"{time.time() - self._t0:7.2f}s {texto}\n")
        self._ev.flush()

    def telemetria(self, p, fps):
        self._n += 1
        r = p.res
        self._csv.writerow([f"{time.time() - self._t0:.2f}", f"{fps:.1f}", f"{p.erro_cm:.2f}",
                            f"{p.proa_graus:.2f}", f"{p.plan.o_cmd:.2f}", f"{p.turn:.3f}",
                            f"{p.speed:.3f}", len(r.linhas) if r else 0, len(p.cones),
                            p.perdidos, p.motivo])
        if self._n % 20 == 0:
            self._csv_f.flush()

    def foto(self, tag, imagem):
        if self._fotos >= self._max_fotos or imagem is None:
            return
        self._fotos += 1
        nome = f"{self._fotos:03d}_{time.time() - self._t0:.1f}s_{tag}.jpg"
        cv2.imwrite(os.path.join(self.pasta, "fotos", nome), imagem)

    def fechar(self):
        for f in (self._ev, self._csv_f):
            try:
                f.close()
            except Exception:
                pass
