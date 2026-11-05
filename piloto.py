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
        self.curvatura = 0.0       # 1/cm, da faixa a frente (pra frear antes da curva)
        self.perdidos = 0
        self.motivo = "ok"
        self.eventos = []
        self._t_ant = None
        self._perdido_avisado = False
        self._t_kick = None         # inicio do empurrao de largada
        self._v_ant = 0.0           # velocidade (sem o empurrao) do quadro anterior

    # ------------------------------------------------------------------
    def step(self, t, frame, fps=30.0, debug=False):
        self.eventos = []
        dt = 0.05 if self._t_ant is None else min(0.3, max(0.01, t - self._t_ant))
        self._t_ant = t
        cam = self.cam

        res = self.faixas.processar(frame, debug=debug)
        self.res = res
        if config.CONES_ATIVOS:
            self.cones = self.rastreador.atualizar(self.detector.detectar(frame, debug=debug))
        else:
            self.cones = []      # corrida pura: cone nem e processado
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

            # erro lateral NO EIXO (Y=0): se a camera so enxerga o chao a partir de
            # y_min, a faixa e prolongada ate o eixo pela tangente (curto e seguro).
            # Proa = inclinacao media da faixa numa janela de OLHAR_ADIANTE_CM a
            # partir do que a camera ve (nao fica refem de 2 pontos colados).
            y_v = max(cam.y_min, 0.0)
            y_la = float(min(cam.y_max, y_v + config.OLHAR_ADIANTE_CM))
            if y_la - y_v < 8.0:
                y_v = max(0.0, y_la - 8.0)
            erro = float(centro.x(0.0)) + o
            inclin = (float(centro.x(y_la)) - float(centro.x(y_v))) / (y_la - y_v)
            proa = math.atan(inclin)
            self.erro_cm = erro
            self.proa_graus = math.degrees(proa)
            # curvatura aproximada: variacao da inclinacao entre a metade perto e a longe
            y_far = float(cam.y_max)
            y_mid = 0.5 * (y_v + y_far)
            if y_far - y_v > 20.0:
                s1 = (float(centro.x(y_mid)) - float(centro.x(y_v))) / (y_mid - y_v)
                s2 = (float(centro.x(y_far)) - float(centro.x(y_mid))) / (y_far - y_mid)
                k = abs(s2 - s1) / (0.5 * (y_far - y_v))
                self.curvatura = 0.5 * self.curvatura + 0.5 * k

            turn = self.direcao.atualizar(dt, erro, proa, v_cms)
            speed = velocidade(turn, self.plan.fator_vel, fps, self.proa_graus, self.erro_cm,
                               self.curvatura)
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

        # empurrao de largada: o motor parado nao vence o atrito na velocidade de
        # cruzeiro. So na saida do repouso, por ARRANQUE_S.
        if speed > 0.05 and self._v_ant <= 0.05:
            self._t_kick = t
        self._v_ant = speed
        if (self._t_kick is not None and speed > 0.05 and
                t - self._t_kick < config.ARRANQUE_S):
            speed = max(speed, config.ARRANQUE_VEL)

        self.turn, self.speed = turn, speed
        return turn, speed
