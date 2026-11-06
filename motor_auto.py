"""
motor_auto.py - o carro aprende SOZINHO a potencia minima em que ele realmente anda.

Problema real: abaixo de certa potencia o motor nao vence o atrito (bateria,
queda na ponte H, peso) e o carro PARA, com o volante todo virado, e a visao
fica olhando a mesma cena pra sempre. Nenhuma estrategia de controle resolve um
carro que nao se mexe.

Solucao: o governador olha a propria camera. Se o comando manda andar e a cena
nao muda (diferenca entre o quadro de agora e o de ~0.3 s atras), o carro esta
PATINANDO/PARADO: sobe o piso de potencia de 5 em 5% ate a cena mexer. O piso
aprendido vale pelo resto da corrida e fica salvo em dados/motor.json.

Tambem faz o empurrao de largada (sair do repouso pede mais que andar).
"""

import collections

import cv2
import numpy as np

import config


def pequeno_cinza(frame):
    return cv2.cvtColor(cv2.resize(frame, (64, 36), interpolation=cv2.INTER_AREA),
                        cv2.COLOR_BGR2GRAY).astype(np.int16)


class GovernadorMotor:
    def __init__(self, piso=None, ruido=0.6):
        # nunca abaixo de PISO_MOTOR_MIN, nem se o arquivo salvo tiver um valor menor
        self.piso = max(float(config.PISO_MOTOR_MIN), float(config.PISO_MOTOR_MIN if piso is None else piso))
        self.piso_inicial = self.piso
        self.ruido = ruido              # diferenca tipica do quadro PARADO (autocalibracao mede)
        self.mov = None                 # diferenca media (0-255) vs ~0.3 s atras
        self.parado = False             # True enquanto detecta que nao anda
        self._hist = collections.deque()
        self._t_cmd = None              # quando o comando de andar comecou
        self._acum = 0.0
        self._acum_fraco = 0.0
        self.eventos = []

    def limiar(self):
        return max(config.STALL_MOV_MIN, 3.0 * self.ruido)

    def atualizar(self, t, dt, pequeno, speed_cmd, turn=0.0, erro=0.0, proa=0.0):
        """Devolve a velocidade FINAL (com piso e empurrao). pequeno = pequeno_cinza(frame).

        Dois tipos de prova de que o carro NAO anda:
          FORTE : volante virado (|turn| >= STALL_TURN_MIN) e o erro lateral e a proa
                  nao mudam: andando com volante virado a pose muda sempre.
          FRACA : so a imagem parada, sem volante virado (reta lisa tambem parece
                  parada andando). So conta com potencia baixa e tem teto menor."""
        self.eventos = []
        self._hist.append((t, pequeno, erro, proa))
        while len(self._hist) > 1 and t - self._hist[1][0] >= config.STALL_JANELA_S:
            self._hist.popleft()
        t0, ref, e0, p0 = self._hist[0]
        janela_ok = (t - t0) >= 0.8 * config.STALL_JANELA_S
        self.mov = float(np.abs(pequeno - ref).mean()) if janela_ok else None

        if speed_cmd <= 0.05:           # parado de proposito
            self._t_cmd = None
            self._acum = 0.0
            self._acum_fraco = 0.0
            self.parado = False
            return 0.0

        if self._t_cmd is None:
            self._t_cmd = t
        desde = t - self._t_cmd
        s = max(speed_cmd, self.piso)
        if desde < config.ARRANQUE_S:
            s = max(s, config.ARRANQUE_VEL)

        self.parado = False
        if desde >= config.STALL_GRACA_S and janela_ok:
            # mov == 0 exato = quadro IDENTICO ao anterior = camera congelada/repetindo, nao carro parado
            # (um carro parado ainda tem ruido de sensor). Sem isso o aprendizado subia o piso a toa.
            congelada = self.mov < 1e-6
            cena_parada = self.mov < self.limiar() and not congelada
            pose_parada = (abs(erro - e0) < config.STALL_D_ERRO_CM and
                           abs(proa - p0) < config.STALL_D_PROA_GRAUS)
            forte = cena_parada and pose_parada and abs(turn) >= config.STALL_TURN_MIN
            fraco = cena_parada and abs(turn) < config.STALL_TURN_MIN and s < config.STALL_FRACO_VEL_MAX
            self._acum = (self._acum + dt) if forte else max(0.0, self._acum - 2.0 * dt)
            self._acum_fraco = (self._acum_fraco + dt) if fraco else max(0.0, self._acum_fraco - 2.0 * dt)
            self.parado = self._acum > 0.25 or self._acum_fraco > 1.0
            teto = None
            if self._acum >= config.STALL_ACUM_S:
                teto, motivo = config.PISO_MAX, "volante virado e a pose nao muda"
            elif self._acum_fraco >= config.STALL_ACUM_FRACO_S:
                teto, motivo = config.PISO_MAX_FRACO, "cena parada"
            if teto is not None:
                novo = min(teto, self.piso + config.PISO_PASSO)
                if novo > self.piso:
                    self.eventos.append(
                        f"[motor] carro nao anda ({motivo}; mov {self.mov:.1f}): "
                        f"piso {self.piso:.2f} -> {novo:.2f}")
                    self.piso = novo
                self._acum = 0.0
                self._acum_fraco = 0.0
                self._t_cmd = t - config.ARRANQUE_S + 0.15   # novo empurraozinho
        return s
