"""
hardware.py - servo de direcao e motores (L298N).

Pinagem: config.py (a tabela "Pinagem sugerida").
Servo: valores de dados/servo.json (gravado por teste_servo.py); se nao existir,
usa os de PARTIDA do config.py.

Se o RPi.GPIO nao existir (PC) ou voce passar simulado=True, usa um GPIO
de mentira: o programa roda igual, so que sem mexer em nada.

Servo: se o `pigpio` estiver rodando (sudo pigpiod) o pulso e gerado por DMA
(sem tremor). Senao usa PWM por software do RPi.GPIO (funciona, mas o servo
pode tremer um pouco).
"""

import json
import os
import time

import config

PASTA_DADOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dados")
ARQ_SERVO = os.path.join(PASTA_DADOS, "servo.json")


# ---------------------------------------------------------------------
# GPIO real ou de mentira
# ---------------------------------------------------------------------
class _PWMFalso:
    def __init__(self, pino, hz): self.pino, self.hz, self.duty = pino, hz, 0.0
    def start(self, d): self.duty = d
    def ChangeDutyCycle(self, d): self.duty = d
    def stop(self): pass


class _GPIOFalso:
    BCM = "BCM"; OUT = "OUT"; HIGH = 1; LOW = 0
    def setmode(self, m): pass
    def setwarnings(self, f): pass
    def setup(self, pino, modo): pass
    def output(self, pino, v): pass
    def PWM(self, pino, hz): return _PWMFalso(pino, hz)
    def cleanup(self): pass


_gpio = None
_gpio_real = False


def gpio(simulado=False):
    """Devolve o modulo GPIO (real, ou de mentira se nao der / se simulado)."""
    global _gpio, _gpio_real
    if _gpio is None:
        if not simulado:
            try:
                import RPi.GPIO as G
                G.setmode(G.BCM)
                G.setwarnings(False)
                _gpio, _gpio_real = G, True
            except (ImportError, RuntimeError):
                print("AVISO: RPi.GPIO indisponivel -> hardware SIMULADO (nada vai se mexer).")
        if _gpio is None:
            _gpio = _GPIOFalso()
    return _gpio


def hardware_real():
    return _gpio_real


def limpar():
    """Libera os pinos. Chame uma vez, no fim do programa."""
    global _gpio, _gpio_real
    if _gpio is not None:
        try:
            _gpio.cleanup()
        except Exception:
            pass
    _gpio, _gpio_real = None, False


# ---------------------------------------------------------------------
# calibracao do servo
# ---------------------------------------------------------------------
def carregar_servo():
    """Devolve (dict com centro/esquerda/direita, origem)."""
    p = {"centro": config.SERVO_CENTRO_PADRAO, "esquerda": config.SERVO_ESQ_PADRAO,
         "direita": config.SERVO_DIR_PADRAO}
    origem = "config.py (valores de partida - rode teste_servo.py)"
    try:
        with open(ARQ_SERVO, "r", encoding="utf-8") as f:
            salvo = json.load(f)
        for k in p:
            if k in salvo:
                p[k] = float(salvo[k])
        origem = "dados/servo.json"
    except (OSError, ValueError):
        pass
    return p, origem


def salvar_servo(centro, esquerda, direita):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    with open(ARQ_SERVO, "w", encoding="utf-8") as f:
        json.dump({"centro": float(centro), "esquerda": float(esquerda),
                   "direita": float(direita)}, f, indent=2)


# ---------------------------------------------------------------------
# servo
# ---------------------------------------------------------------------
class Servo:
    def __init__(self, simulado=False, pino=None):
        self.pino = config.SERVO_PIN if pino is None else pino
        self.G = gpio(simulado)
        self.cal, self.origem = carregar_servo()
        self.angulo_atual = None
        self._pi = None
        self._pwm = None
        if hardware_real():
            try:
                import pigpio
                pi = pigpio.pi()
                if pi.connected:
                    self._pi = pi
            except ImportError:
                pass
        if self._pi is None:
            self.G.setup(self.pino, self.G.OUT)
            self._pwm = self.G.PWM(self.pino, 50)
            self._pwm.start(0)
        self.modo = "pigpio" if self._pi is not None else "RPi.GPIO"
        self.angulo(self.cal["centro"], forcar=True)

    def angulo(self, a, forcar=False):
        lo, hi = config.SERVO_LIMITE_ABS
        a = max(lo, min(hi, float(a)))
        if (not forcar and self.angulo_atual is not None
                and abs(a - self.angulo_atual) < config.SERVO_DEADBAND_DEG):
            return self.angulo_atual
        self.angulo_atual = a
        p0, p1 = config.SERVO_PULSO_US
        us = p0 + (a / 180.0) * (p1 - p0)
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(self.pino, us)
        else:
            self._pwm.ChangeDutyCycle(us / 20000.0 * 100.0)
        return a

    def virar(self, turn):
        """turn -1 (esquerda total) .. +1 (direita total), usando a calibracao."""
        turn = max(-1.0, min(1.0, turn))
        c = self.cal["centro"]
        alvo = self.cal["esquerda"] if turn < 0 else self.cal["direita"]
        return self.angulo(c + abs(turn) * (alvo - c))

    def solto(self):
        """Para de mandar pulso (servo relaxa, nao esquenta/zumbe)."""
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(self.pino, 0)
        else:
            self._pwm.ChangeDutyCycle(0)
        self.angulo_atual = None

    def fechar(self):
        try:
            self.angulo(self.cal["centro"], forcar=True)
            time.sleep(0.3)
            self.solto()
            if self._pi is not None:
                self._pi.stop()
            elif self._pwm is not None:
                self._pwm.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------
# motores (L298N)
# ---------------------------------------------------------------------
class Motores:
    """Os dois motores de tracao andam juntos (mesmo comando)."""

    def __init__(self, simulado=False):
        G = self.G = gpio(simulado)
        for p in (config.ENA, config.IN1, config.IN2, config.ENB, config.IN3, config.IN4):
            G.setup(p, G.OUT)
        for p in (config.IN1, config.IN2, config.IN3, config.IN4):
            G.output(p, G.LOW)
        self.pwm_a = G.PWM(config.ENA, config.MOTOR_PWM_HZ)
        self.pwm_b = G.PWM(config.ENB, config.MOTOR_PWM_HZ)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        self.ultimo = 0.0

    def motor(self, qual, speed):
        """qual 'A' ou 'B'; speed -1..1 (positivo = pra frente)."""
        G = self.G
        speed = max(-1.0, min(1.0, speed))
        if qual == "A":
            in1, in2, pwm = config.IN1, config.IN2, self.pwm_a
            inv = config.MOTOR_A_INVERTIDO
        else:
            in1, in2, pwm = config.IN3, config.IN4, self.pwm_b
            inv = config.MOTOR_B_INVERTIDO
        frente = (speed >= 0) != inv
        G.output(in1, G.HIGH if frente else G.LOW)
        G.output(in2, G.LOW if frente else G.HIGH)
        pwm.ChangeDutyCycle(abs(speed) * 100.0)

    def andar(self, speed):
        self.motor("A", speed)
        self.motor("B", speed)
        self.ultimo = speed

    def parar(self):
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        G = self.G
        for p in (config.IN1, config.IN2, config.IN3, config.IN4):
            G.output(p, G.LOW)
        self.ultimo = 0.0

    def fechar(self):
        self.parar()
        time.sleep(0.1)
        self.pwm_a.stop()
        self.pwm_b.stop()
