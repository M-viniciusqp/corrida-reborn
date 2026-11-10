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

import cv2
import numpy as np

import config
from geometria import CameraChao
from faixas import DetectorFaixas
from cones import DetectorCones, RastreadorCones
from planejador import Planejador
from controle import Direcao, velocidade
from motor_auto import GovernadorMotor, pequeno_cinza
from adaptacao import Adaptacao


class Piloto:
    def __init__(self, params=None, res=None, piso=None):
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
        self.curvatura_sinal = 0.0 # idem COM SINAL (+ = faixa dobrando pra direita): descontar o giro da pista
        self.perdidos = 0
        self.motivo = "ok"
        self.eventos = []
        self._t_ant = None
        self._perdido_avisado = False
        self.gov = GovernadorMotor(piso)   # piso de potencia aprendido + empurrao de largada
        self.espelhar = False      # continua a ESQUERDA: espelha a imagem (o resto assume continua a direita)
        self.frame_proc = None     # quadro realmente processado (espelhado ou nao), pro HUD
        self.deslocamento_cm = 0.0 # alvo lateral aplicado (vies da pontilhada + desvio), pro HUD
        self.protecoes = True      # False so na calibracao antiga (comandos de teste desviam o carro de proposito)
        self._sat = 0              # quadros seguidos com o erro colado no maximo (rastreio furado)
        self._erro_f = None        # erro e proa SUAVIZADOS (o Stanley usa estes)
        self._proa_f = None
        self._t_ok0 = None         # instante da primeira leitura boa da faixa
        self._n_ok = 0             # quadros com faixa lida desde a largada (olha antes de manobrar)
        self.adapt = Adaptacao(self)   # calibracao ANDANDO: sentido do volante, puxao, ganho, lado da continua

    def _alternar_espelho(self):
        """A faixa continua esta do outro lado: espelha e recomeca o rastreio do centro."""
        self.espelhar = not self.espelhar
        f = self.faixas
        f.c_prev, f._tem_prev, f._tracks, f._evid, f._sem_centro = 0.0, True, [], 0.0, 4
        self.direcao.reset()
        self.adapt._zerar()

    # ------------------------------------------------------------------
    def step(self, t, frame, fps=30.0, debug=False):
        self.eventos = []
        dt = 0.05 if self._t_ant is None else min(0.3, max(0.01, t - self._t_ant))
        self._t_ant = t
        cam = self.cam
        if self.espelhar:
            frame = cv2.flip(frame, 1)
        self.frame_proc = frame
        self.direcao.espelho = self.espelhar

        res = self.faixas.processar(frame, debug=debug)
        self.res = res
        if config.CONES_ATIVOS:
            self.cones = self.rastreador.atualizar(self.detector.detectar(frame, debug=debug))
        else:
            self.cones = []      # corrida pura: cone nem e processado
        # velocidade estimada (sem encoder): do comando anterior, nunca abaixo do
        # minimo (parado/arrancando o ganho do Stanley explodiria)
        v_cms = max(self.speed, config.PISO_MOTOR_MIN) * config.V_CMS_NA_VEL_1

        # ---- ERRO SATURADO: erro colado no maximo por varios quadros nao e curva, e rastreio furado
        # (a faixa trocou de lado, linha errada): em vez de virar o volante com confianca total, admite
        # que se perdeu, zera o rastreio e segue o caminho de 'faixa perdida'
        if res.centro is not None and self.protecoes:
            if abs(float(res.centro.x(0.0))) >= config.SAT_ERRO_FRAC * self.faixas.W:
                self._sat += 1
            else:
                self._sat = 0
            if self._sat > config.MAX_FRAMES_SATURADO:
                self._sat = 0
                self.faixas.reiniciar()
                res.centro = None
                res.motivo = "erro saturado"
        else:
            self._sat = 0

        if res.centro is not None:
            if self.perdidos >= config.PERDIDO_QUADROS_DEVAGAR:
                self.eventos.append("[pista] faixa reencontrada")
            self.perdidos = 0
            self._perdido_avisado = False
            self.motivo = "ok"
            centro = res.centro
            o = self.plan.atualizar(dt, self.cones, centro, v_cms)
            # linha de corrida: mais perto da pontilhada (sempre a ESQUERDA no mundo ja espelhado)
            vies = -min(0.9, max(0.0, config.VIES_PONTILHADA_FRAC)) * 0.5 * self.faixas.W
            if config.ADAPTACAO and not self.adapt.direcao_ok:
                vies = 0.0            # ate confirmar o sentido do volante: sem manobra, so segue o centro
            o += vies
            self.deslocamento_cm = o
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
                ks = (s2 - s1) / (0.5 * (y_far - y_v))
                self.curvatura = 0.5 * self.curvatura + 0.5 * abs(ks)
                self.curvatura_sinal = 0.5 * self.curvatura_sinal + 0.5 * ks

            # erro e proa SUAVIZADOS (media movel exponencial): tira o tremido de uma leitura isolada
            if self._erro_f is None:
                self._erro_f, self._proa_f = erro, proa
            else:
                a_s = config.ERRO_SUAV
                self._erro_f = a_s * erro + (1.0 - a_s) * self._erro_f
                self._proa_f = a_s * proa + (1.0 - a_s) * self._proa_f
            turn = self.direcao.atualizar(dt, self._erro_f, self._proa_f, v_cms)
            self._n_ok += 1
            if self._t_ok0 is None:
                self._t_ok0 = t
            if (config.ADAPTACAO and not self.adapt.lado_travado
                    and (t - self._t_ok0) <= config.LARGADA_OLHAR_S
                    and abs(self.erro_cm) < config.LARGADA_OLHAR_ERRO_CM
                    and abs(self.proa_graus) < config.LARGADA_OLHAR_PROA):
                # lado da faixa continua ainda incerto: nos primeiros quadros anda RETO (so o puxao
                # conhecido), pra a evidencia de lado chegar antes de qualquer manobra
                self.direcao.turn = 0.0
                turn = self.direcao.saida()
            speed = velocidade(turn, self.plan.fator_vel, fps, self.proa_graus, self.erro_cm,
                               self.curvatura)

            # ---- calibracao ANDANDO (nada de esperar antes de largar)
            self.adapt.atualizar(t, dt, self.proa_graus, self.curvatura_sinal, self.speed, self.gov.parado, self.erro_cm)
            if self.adapt.lado(res.linhas, self.faixas._y_ref(), dt):
                self._alternar_espelho()
            self.eventos.extend(self.adapt.eventos)
            if self.adapt.virou_agora:
                turn = self.direcao.saida()        # volante invertido agora: a saida deste mesmo quadro ja sai certa
        else:
            self.perdidos += 1
            self.motivo = res.motivo
            self._erro_f = self._proa_f = None      # leitura nova depois do buraco: sem misturar com a velha
            self.adapt.sem_leitura()
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

        # ate confirmar o sentido do volante: devagar. O limite vale ANTES do governador (ele precisa
        # saber a potencia que de fato esta sendo pedida pra julgar se o carro anda) e DEPOIS (o empurrao
        # de largada tambem respeita).
        limitada = config.ADAPTACAO and not self.adapt.direcao_ok
        if limitada and speed > 0.05:
            speed = min(speed, max(config.LARGADA_VEL_MAX, self.gov.piso))
        # governador do motor: piso aprendido (carro parado -> sobe a potencia) e empurrao
        speed = self.gov.atualizar(t, dt, pequeno_cinza(frame), speed, turn,
                                   self.erro_cm, self.proa_graus)
        self.eventos.extend(self.gov.eventos)
        if limitada and speed > 0.05:
            speed = min(speed, max(config.LARGADA_VEL_MAX, self.gov.piso))

        self.turn, self.speed = turn, speed
        return turn, speed
