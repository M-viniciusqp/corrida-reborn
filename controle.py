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
    """Stanley. O ESTADO (self.turn) fica no mundo LOGICO: + = virar pra direita no mundo ja
    espelhado em que o piloto enxerga. O sinal fisico (volante invertido, imagem espelhada) e o
    puxao do carro (trim) entram so na SAIDA, pra que a adaptacao online consiga medir o efeito
    do comando no giro do carro sem confundir com eles."""

    def __init__(self):
        self.turn = 0.0                           # estado logico (-1..1)
        self.delta_graus = 0.0
        self.termos = (0.0, 0.0)                  # (proa, lateral) em graus, so pra log
        # Aprendidos ANDANDO (adaptacao.py) ou lidos de dados/auto.json; o config.py e so o padrao.
        self.inverter = config.ESTERCO_INVERTER   # o volante vira pro lado oposto ao comandado
        self.trim = 0.0                           # comando fisico que faz o carro andar RETO (-1..1)
        self.ganho = 1.0                          # multiplica o esterco (carro de direcao fraca/forte)
        self.espelho = False                      # imagem espelhada (continua a esquerda)

    def reset(self):
        self.turn = 0.0

    def sinal(self):
        return -1.0 if (self.inverter != self.espelho) else 1.0

    def saida(self):
        """Comando FISICO pro servo: sinal * estado + trim."""
        return max(-1.0, min(1.0, self.sinal() * self.turn + self.trim))

    def atualizar(self, dt, erro_cm, proa_rad, v_cms):
        if abs(erro_cm) < config.ERRO_ZONA_MORTA_CM:
            erro_cm = 0.0
        t_proa = config.PROA_PESO * proa_rad
        t_lat = math.atan2(config.STANLEY_K * erro_cm, config.STANLEY_V0_CMS + v_cms)
        delta = t_proa + t_lat
        self.delta_graus = math.degrees(delta)
        self.termos = (math.degrees(t_proa), math.degrees(t_lat))
        alvo = max(-1.0, min(1.0, self.ganho * self.delta_graus / config.ESTERCO_MAX_GRAUS))
        a = config.ESTERCO_SUAVIZACAO
        novo = a * alvo + (1 - a) * self.turn
        passo = config.ESTERCO_SLEW * dt
        novo = max(self.turn - passo, min(self.turn + passo, novo))
        self.turn = max(-1.0, min(1.0, novo))
        return self.saida()

    def decair(self, fator=0.9):
        self.turn *= fator
        return self.saida()


def velocidade(turn, fator_cone=1.0, fps=30.0, proa_graus=0.0, erro_cm=0.0, curvatura=0.0):
    """Velocidade (0..1): reduz com o volante, com o carro torto/longe do centro
    (proa e erro grandes), com cone perto e com FPS baixo."""
    incerto = max(abs(proa_graus) / config.INCERTO_PROA_GRAUS,
                  abs(erro_cm) / config.INCERTO_ERRO_CM)
    v = config.VELOCIDADE_BASE * (1.0 - config.VEL_REDUCAO_CURVA * abs(turn))
    # freia ANTES da curva: curvatura da faixa la na frente (1/cm; raio = 1/curvatura)
    v *= 1.0 - config.VEL_REDUCAO_CURVATURA * min(1.0, abs(curvatura) / config.CURVATURA_CHEIA)
    v *= 1.0 - config.VEL_REDUCAO_INCERTO * min(1.0, incerto)
    v *= max(0.0, min(1.0, fator_cone))
    v = max(config.VELOCIDADE_MIN, v)          # piso de CRUZEIRO: so as freadas de curva/torto/cone
    if fps < config.FPS_MINIMO:                # freio de seguranca por FPS baixo pode ir ate o piso do motor
        v *= max(0.5, fps / config.FPS_MINIMO)
    return max(config.PISO_MOTOR_MIN, v)
