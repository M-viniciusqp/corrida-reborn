"""
teste_servo.py - o servo nos ANGULOS EXATOS do carro (sem calibrar).

    python3 teste_servo.py              # 0 (esquerda) -> 35 (reto) -> 65 (direita), 2 s em cada, em loop
    python3 teste_servo.py --ajustar    # so se precisar mudar: a/d = -/+1 grau, A/D = -/+5, q = sai e mostra o angulo
    python3 teste_servo.py --simulado   # ensaio no PC (nao mexe em nada)

E o mesmo teste que voce ja usa (gpiozero.AngularServo no pino 18, pulso 0.5 a 2.5 ms), so que lendo os
angulos do config.py (SERVO_ESQ_PADRAO / SERVO_CENTRO_PADRAO / SERVO_DIR_PADRAO = 0 / 35 / 65).
Ctrl+C encerra e volta o servo pro centro.

Esquerda/direita sao do ponto de vista de quem esta SENTADO no carro. Se as rodas virarem pro lado
trocado nao precisa mexer em nada: o carro percebe sozinho andando e inverte o volante.
"""

import sys
from time import sleep

import config
import hardware

if sys.platform == "win32":
    import msvcrt

    def ler_tecla():
        return msvcrt.getwch()
else:
    import termios
    import tty

    def ler_tecla():
        fd = sys.stdin.fileno()
        antigo = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, antigo)


def loop_fixo(servo):
    esq, cen, dire = servo.cal["esquerda"], servo.cal["centro"], servo.cal["direita"]
    try:
        while True:
            print(f"Posição: {esq:.0f}° (esquerda)")
            servo.angulo(esq, forcar=True)
            sleep(2)

            print(f"Posição: {cen:.0f}° (reto)")
            servo.angulo(cen, forcar=True)
            sleep(2)

            print(f"Posição: {dire:.0f}° (direita)")
            servo.angulo(dire, forcar=True)
            sleep(2)
    except KeyboardInterrupt:
        print("Teste encerrado")


def ajustar(servo):
    ang = float(servo.cal["centro"])
    servo.angulo(ang, forcar=True)
    print("a/d = -/+1 grau | A/D = -/+5 | q = sair. Anote o angulo em que a roda fica RETA, na ESQUERDA total e na "
          "DIREITA total e ponha em config.py (SERVO_CENTRO_PADRAO, SERVO_ESQ_PADRAO, SERVO_DIR_PADRAO).")
    lo, hi = config.SERVO_LIMITE_ABS
    try:
        while True:
            print(f"\r  angulo {ang:5.1f}   ", end="", flush=True)
            k = ler_tecla()
            if k in ("q", "\x03"):
                break
            ang += {"a": -1, "d": 1, "A": -5, "D": 5}.get(k, 0)
            ang = max(lo, min(hi, ang))
            servo.angulo(ang, forcar=True)
    except KeyboardInterrupt:
        pass
    print(f"\nUltimo angulo: {ang:.0f}")


def main():
    servo = hardware.Servo(simulado="--simulado" in sys.argv)
    print(f"Servo no pino BCM {servo.pino} ({servo.modo}): esquerda {servo.cal['esquerda']:.0f} | "
          f"reto {servo.cal['centro']:.0f} | direita {servo.cal['direita']:.0f}  [{servo.origem}]")
    try:
        if "--ajustar" in sys.argv:
            ajustar(servo)
        else:
            loop_fixo(servo)
    finally:
        servo.fechar()
        hardware.limpar()


if __name__ == "__main__":
    main()
