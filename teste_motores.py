"""
teste_motores.py - confere a FIACAO dos motores (L298N) antes de rodar o carro.

    python3 teste_motores.py            # com o carro SUSPENSO (rodas fora do chao!)
    python3 teste_motores.py --simulado

Testa cada motor sozinho, pra frente e pra tras, devagar (35%). Voce responde
se a roda girou pra frente. Se girou ao contrario, ele diz o que mudar no
config.py (MOTOR_A_INVERTIDO / MOTOR_B_INVERTIDO). Se um motor nao girou:
confira ENA/ENB (jumper removido), IN1-IN4 e o GND comum.
"""

import sys
import time

import config
import hardware

POTENCIA = 0.35
TEMPO = 1.2


def perguntar(txt):
    try:
        return input(txt).strip().lower()
    except EOFError:
        return "n"


def main():
    simulado = "--simulado" in sys.argv
    print(__doc__)
    print(f"Pinos (BCM): ENA {config.ENA} IN1 {config.IN1} IN2 {config.IN2} | "
          f"ENB {config.ENB} IN3 {config.IN3} IN4 {config.IN4}")
    if perguntar("O carro esta com as rodas FORA DO CHAO? (s/n) ") != "s":
        print("Levante o carro e rode de novo.")
        return
    m = hardware.Motores(simulado=simulado)
    try:
        for nome in ("A", "B"):
            input(f"\nEnter pra girar o MOTOR {nome} pra FRENTE por {TEMPO:.1f}s...")
            m.motor(nome, POTENCIA)
            time.sleep(TEMPO)
            m.parar()
            r = perguntar(f"Motor {nome}: a roda girou pra FRENTE? (s = sim / n = ao contrario / x = nao girou) ")
            if r == "n":
                var = "MOTOR_A_INVERTIDO" if nome == "A" else "MOTOR_B_INVERTIDO"
                print(f"  -> no config.py coloque {var} = True")
            elif r == "x":
                print("  -> nao girou: confira ENA/ENB (jumper tirado), IN1..IN4, bateria e GND comum.")
            else:
                print("  ok")
            time.sleep(0.4)
        input("\nEnter pra girar os DOIS pra frente juntos...")
        m.andar(POTENCIA)
        time.sleep(TEMPO)
        m.parar()
        print("Fim do teste.")
    except KeyboardInterrupt:
        print("\nInterrompido.")
    finally:
        m.fechar()
        hardware.limpar()


if __name__ == "__main__":
    main()
