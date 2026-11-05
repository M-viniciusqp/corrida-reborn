"""
teste_servo.py - descobre o CENTRO, o limite ESQUERDO e o limite DIREITO do servo.

    python3 teste_servo.py              # modo manual (teclas)
    python3 teste_servo.py --varrer     # primeiro faz uma varredura lenta pra voce ver
    python3 teste_servo.py --simulado   # ensaio sem o carro (nao mexe em nada)

Rode com os MOTORES DE TRACAO DESLIGADOS (este teste nem liga eles) e o carro
no chao ou na mao, de frente pra voce olhando as rodas da frente.

Teclas (esquerda/direita = do ponto de vista de quem esta SENTADO no carro):
    a / d      angulo  -1 / +1 grau          (A / D = -5 / +5, maiusculas)
    c          marca o angulo atual como CENTRO (roda bem reta)
    l          marca o angulo atual como limite ESQUERDO (roda virada toda pra esquerda)
    r          marca o angulo atual como limite DIREITO  (roda virada toda pra direita)
    1 2 3      vai pro esquerdo / centro / direito ja marcados
    v          demonstracao: esquerdo -> centro -> direito -> centro
    o          solta o servo (para o pulso: sem zumbido/esquentar)
    s          salva em dados/servo.json e sai
    q          sai sem salvar

CUIDADO: va devagar nos limites. Pare ANTES da roda bater no batente da
direcao: forcar o servo contra o batente esquenta, zumbe e pode quebrar a engrenagem.
O limite que voce marca ja e o que o carro vai usar como "esterco total".
"""

import sys
import time

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


def mostrar(servo, marcas):
    m = " ".join(f"{k}={'--' if v is None else '%.0f' % v}" for k, v in marcas.items())
    ang = servo.angulo_atual
    print(f"\r  angulo {'solto' if ang is None else '%5.1f' % ang}   [{m}]      ",
          end="", flush=True)


def varrer(servo, centro, passo=5.0, alcance=35.0, pausa=0.8):
    """Varredura lenta em volta do centro de partida, imprimindo cada angulo."""
    lo, hi = config.SERVO_LIMITE_ABS
    print(f"\nVarredura: {centro:.0f} -> {centro - alcance:.0f} -> {centro + alcance:.0f} -> {centro:.0f}"
          f" (passo {passo:.0f} graus). Anote: onde a roda fica reta e onde ela chega no batente.")
    print("Ctrl+C interrompe.\n")
    trajeto = []
    a = centro
    while a > max(lo, centro - alcance):
        a -= passo; trajeto.append(a)
    while a < min(hi, centro + alcance):
        a += passo; trajeto.append(a)
    while a > centro:
        a -= passo; trajeto.append(max(a, centro))
    servo.angulo(centro, forcar=True)
    time.sleep(1.0)
    for a in trajeto:
        servo.angulo(a, forcar=True)
        print(f"  servo em {a:5.1f} graus")
        time.sleep(pausa)


def main():
    simulado = "--simulado" in sys.argv
    servo = hardware.Servo(simulado=simulado)
    print(f"Servo no pino BCM {servo.pino} ({servo.modo}). Calibracao atual [{servo.origem}]: "
          f"centro {servo.cal['centro']:.0f} esq {servo.cal['esquerda']:.0f} dir {servo.cal['direita']:.0f}")
    if not hardware.hardware_real():
        print("(hardware SIMULADO: o servo de verdade nao vai se mexer)")
    marcas = {"esquerdo": None, "centro": None, "direito": None}
    ang = float(servo.cal["centro"])
    servo.angulo(ang, forcar=True)
    try:
        if "--varrer" in sys.argv:
            varrer(servo, ang)
            servo.angulo(ang, forcar=True)
        print("Teclas" + __doc__.split("Teclas")[1].split("CUIDADO")[0].rstrip())
        mostrar(servo, marcas)
        while True:
            k = ler_tecla()
            if k in ("q", "\x03"):
                print("\nSaiu sem salvar.")
                return
            elif k == "a": ang -= 1
            elif k == "d": ang += 1
            elif k == "A": ang -= 5
            elif k == "D": ang += 5
            elif k == "c": marcas["centro"] = ang
            elif k == "l": marcas["esquerdo"] = ang
            elif k == "r": marcas["direito"] = ang
            elif k in ("1", "2", "3"):
                alvo = {"1": "esquerdo", "2": "centro", "3": "direito"}[k]
                if marcas[alvo] is not None:
                    ang = marcas[alvo]
            elif k == "v":
                if None in marcas.values():
                    print("\n  (marque os tres primeiro: l, c, r)")
                else:
                    print()
                    for alvo in ("esquerdo", "centro", "direito", "centro"):
                        servo.angulo(marcas[alvo], forcar=True)
                        print(f"  {alvo}: {marcas[alvo]:.0f}")
                        time.sleep(1.0)
                    ang = marcas["centro"]
            elif k == "o":
                servo.solto(); mostrar(servo, marcas); continue
            elif k == "s":
                if None in marcas.values():
                    print("\n  Falta marcar: " + ", ".join(n for n, v in marcas.items() if v is None))
                    continue
                e, c, d = marcas["esquerdo"], marcas["centro"], marcas["direito"]
                if not (min(e, d) < c < max(e, d)):
                    print("\n  O centro precisa ficar ENTRE os dois limites. Confira as marcas.")
                    continue
                if abs(d - e) < 10:
                    print("\n  Os limites estao muito proximos (<10 graus). Confira as marcas.")
                    continue
                hardware.salvar_servo(c, e, d)
                print(f"\nSalvo em dados/servo.json: centro {c:.0f} | esquerdo {e:.0f} | direito {d:.0f}")
                print(f"  esquerda total = {e:.0f} ({abs(c - e):.0f} graus do centro) | "
                      f"direita total = {d:.0f} ({abs(d - c):.0f} graus do centro)")
                if abs(abs(c - e) - abs(d - c)) > 8:
                    print("  Obs.: os lados sao assimetricos; o programa trata cada lado separado.")
                if e > d:
                    print("  Obs.: servo 'invertido' (esquerdo > direito): ja esta tratado.")
                print("  Copie esses numeros pro config.py (SERVO_*_PADRAO) so se quiser um padrao no repositorio.")
                return
            else:
                continue
            lo, hi = config.SERVO_LIMITE_ABS
            ang = max(lo, min(hi, ang))
            servo.angulo(ang, forcar=True)
            mostrar(servo, marcas)
    except KeyboardInterrupt:
        print("\nInterrompido.")
    finally:
        servo.fechar()
        hardware.limpar()


if __name__ == "__main__":
    main()
