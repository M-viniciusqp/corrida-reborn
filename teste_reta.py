"""
teste_reta.py - o carro anda RETO? (sem camera, sem visao, so servo no centro + motores)

    python3 teste_reta.py              # 0.45 de potencia por 2.5 s
    python3 teste_reta.py 0.6 3        # potencia 0.6 por 3 s
    python3 teste_reta.py --simulado

Coloque o carro no chao, num espaco livre de uns 2 m, apontado pra frente.
Ele anda de verdade, entao fique com a mao perto. Ctrl+C para na hora.

Isso separa problema MECANICO de problema de VISAO:
  * anda reto            -> servo centrado e motores equilibrados; o problema e outro
  * puxa pra ESQUERDA    -> centro do servo desregulado OU motor esquerdo/direito desigual
  * puxa pra DIREITA     -> idem pro outro lado
Depois de rodar, ele pergunta pra que lado o carro foi e ajusta o centro do
servo em passos de 2 graus (so o centro; esquerda/direita ficam como estao).
"""

import sys
import time

import hardware

args = [a for a in sys.argv[1:] if not a.startswith("--")]
POT = float(args[0]) if len(args) > 0 else 0.45
TEMPO = float(args[1]) if len(args) > 1 else 2.5


def pergunta(txt):
    try:
        return input(txt).strip().lower()
    except EOFError:
        return ""


def main():
    simulado = "--simulado" in sys.argv
    servo = hardware.Servo(simulado=simulado)
    motores = hardware.Motores(simulado=simulado)
    cal = servo.cal
    print(f"Servo [{servo.origem}]: centro {cal['centro']:.0f} esq {cal['esquerda']:.0f} dir {cal['direita']:.0f}")
    try:
        while True:
            servo.angulo(cal["centro"], forcar=True)
            input(f"\nCarro no chao, apontado pra frente, area livre. Enter pra andar "
                  f"reto {TEMPO:.1f}s a {POT:.2f} (Ctrl+C para)...")
            motores.andar(POT)
            time.sleep(TEMPO)
            motores.parar()
            r = pergunta("Pra onde o carro foi? (e = esquerda / d = direita / r = foi reto / q = sair): ")
            if r == "q" or r == "r":
                if r == "r":
                    print("Anda reto: servo e motores estao bons. O problema nao e mecanico.")
                break
            if r not in ("e", "d"):
                continue
            # nao sei em qual direcao do SERVO fica a esquerda: usa a calibracao.
            # esquerda do servo = cal['esquerda'] (pode ser maior ou menor que o centro)
            sentido_esq = 1.0 if cal["esquerda"] > cal["centro"] else -1.0
            # se o carro puxa pra esquerda, o centro precisa ir um pouco pro lado DIREITO
            passo = -sentido_esq * 2.0 if r == "e" else sentido_esq * 2.0
            cal["centro"] = cal["centro"] + passo
            print(f"  -> centro ajustado para {cal['centro']:.0f}. Repete o teste.")
            servo.cal = cal
        if abs(cal["centro"] - hardware.carregar_servo()[0]["centro"]) > 0.1:
            if pergunta(f"Salvar centro {cal['centro']:.0f} em dados/servo.json? (s/n) ") == "s":
                hardware.salvar_servo(cal["centro"], cal["esquerda"], cal["direita"])
                print("Salvo.")
    except KeyboardInterrupt:
        print("\nInterrompido.")
    finally:
        motores.fechar()
        servo.fechar()
        hardware.limpar()


if __name__ == "__main__":
    main()
