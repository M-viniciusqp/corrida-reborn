"""
controle.py - direcao (Stanley) e velocidade.

Stanley (o controlador do carro autonomo de Stanford):

    volante = proa + atan(K * erro_lateral / (V0 + v))

  erro_lateral : cm entre o centro do carro e o alvo (centro da faixa + desvio)
  proa         : angulo entre o carro e a direcao da faixa a frente (rad)
  v            : velocidade estimada (cm/s) - nao ha encoder, sai do comando

Na curva, a proa vem da inclinacao MEDIA da faixa entre o ponto mais perto e o
ponto de OLHAR_ADIANTE_CM, entao o carro ja comeca a virar antes da curva.
O resultado (rad) vira turn -1..+1 dividindo pelo esterco maximo da roda.
"""

import math

import config


class Direcao:
    def __init__(self):
        self.turn = 0.0
        self.delta_graus = 0.0
        self.termos = (0.0, 0.0)       # (proa, lateral) em graus, so pra log

    def reset(self):
        self.turn = 0.0

    def atualizar(self, dt, erro_cm, proa_rad, v_cms):
        if abs(erro_cm) < config.ERRO_ZONA_MORTA_CM:
            erro_cm = 0.0
        t_proa = config.PROA_PESO * proa_rad
        t_lat = math.atan2(config.STANLEY_K * erro_cm, config.STANLEY_V0_CMS + v_cms)
        delta = t_proa + t_lat
        self.delta_graus = math.degrees(delta)
        self.termos = (math.degrees(t_proa), math.degrees(t_lat))
        alvo = max(-1.0, min(1.0, self.delta_graus / config.ESTERCO_MAX_GRAUS))
        if config.ESTERCO_INVERTER:
            alvo = -alvo
        a = config.ESTERCO_SUAVIZACAO
        novo = a * alvo + (1 - a) * self.turn
        passo = config.ESTERCO_SLEW * dt
        novo = max(self.turn - passo, min(self.turn + passo, novo))
        self.turn = max(-1.0, min(1.0, novo))
        return self.turn

    def decair(self, fator=0.9):
        self.turn *= fator
        return self.turn


def velocidade(turn, fator_cone=1.0, fps=30.0):
    """Velocidade (0..1): reduz com o volante, com cone perto e com FPS baixo."""
    v = config.VELOCIDADE_BASE * (1.0 - config.VEL_REDUCAO_CURVA * abs(turn))
    v *= max(0.0, min(1.0, fator_cone))
    if fps < config.FPS_MINIMO:
        v *= max(0.5, fps / config.FPS_MINIMO)
    return max(config.VELOCIDADE_MIN, v)
