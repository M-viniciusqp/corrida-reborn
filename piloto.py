"""
piloto.py - o CEREBRO: recebe uma imagem, devolve (turn, speed).

Nao conhece GPIO nem camera: o MESMO codigo roda no carro (principal.py) e no
simulador (sim.py).

    imagem --> faixas.py   (centro da faixa A em cm)
          \\--> cones.py    (cones em cm no chao)
                 |
              planejador.py (deslocamento lateral pra desviar)
                 |
              controle.py   (Stanley -> turn, velocidade)
"""

import math
import time

import numpy as np

import config
from geometria import CameraChao
from faixas import DetectorFaixas
from cones import DetectorCones, RastreadorCones
from planejador import Planejador
from controle import Direcao, velocidade


class Piloto:
    def __init__(self, params=None, res=None):
        self.cam = CameraChao(params, res)
        self.faixas = DetectorFaixas(self.cam)
        self.detector = DetectorCones(self.cam)
        self.rastreador = RastreadorCones()
        self.plan = Planejador()
        self.direcao = Direcao()

        self.turn = 0.0
        self.speed = 0.0
        self.res = None            # ResultadoFaixas do ultimo quadro
        self.cones = []            # cones confirmados visiveis
        self.erro_cm = 0.0
        self.proa_graus = 0.0
        self.perdidos = 0
        self.motivo = "ok"
        self.eventos = []
        self._t_ant = None
        self._perdido_avisado = False

    # ------------------------------------------------------------------
    def step(self, t, frame, fps=30.0, debug=False):
        self.eventos = []
        dt = 0.05 if self._t_ant is None else min(0.3, max(0.01, t - self._t_ant))
        self._t_ant = t
        cam = self.cam

        res = self.faixas.processar(frame, debug=debug)
        self.res = res
        self.cones = self.rastreador.atualizar(self.detector.detectar(frame, debug=debug))
        # velocidade estimada (sem encoder): do comando anterior, nunca abaixo do
        # minimo (parado/arrancando o ganho do Stanley explodiria)
        v_cms = max(self.speed, config.VELOCIDADE_MIN) * config.V_CMS_NA_VEL_1

        if res.centro is not None:
            if self.perdidos >= config.PERDIDO_QUADROS_DEVAGAR:
                self.eventos.append("[pista] faixa reencontrada")
            self.perdidos = 0
            self._perdido_avisado = False
            self.motivo = "ok"
            centro = res.centro
            o = self.plan.atualizar(dt, self.cones, centro, v_cms)
            self.eventos.extend(self.plan.eventos)

            y_e = max(cam.y_min, 0.0)
            y_la = float(np.clip(config.OLHAR_ADIANTE_CM, y_e + 8.0, cam.y_max))
            erro = float(centro.x(y_e)) + o
            inclin = (float(centro.x(y_la)) - float(centro.x(y_e))) / (y_la - y_e)
            proa = math.atan(inclin)
            self.erro_cm = erro
            self.proa_graus = math.degrees(proa)

            turn = self.direcao.atualizar(dt, erro, proa, v_cms)
            speed = velocidade(turn, self.plan.fator_vel, fps)
        else:
            self.perdidos += 1
            self.motivo = res.motivo
            self.plan.atualizar(dt, [], None, v_cms)
            if self.perdidos == 1:
                self.eventos.append(f"[pista] perdi a faixa ({res.motivo})")
            if self.perdidos >= config.PERDIDO_QUADROS_PARAR:
                if not self._perdido_avisado:
                    self.eventos.append("[pista] sem faixa ha muito tempo -> PAROU")
                    self._perdido_avisado = True
                turn = self.direcao.decair(0.8)
                speed = 0.0
            else:
                turn = self.direcao.decair(0.97)
                speed = config.VELOCIDADE_PERDIDO

        self.turn, self.speed = turn, speed
        return turn, speed
