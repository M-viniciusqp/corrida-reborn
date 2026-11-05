"""
teste_autodirecao.py - testa no SIMULADOR a calibracao automatica da direcao.

Cria carros "defeituosos" (volante invertido, puxando pra um lado) sem avisar o
piloto, deixa ele se calibrar sozinho e depois corre uma pista. Serve pra provar
que a calibracao acerta o sentido do volante e compensa o puxao.

    python teste_autodirecao.py
"""

import sys

import config
import sim
import calibracao_direcao as cd
from geometria import CameraChao, parametros_padrao
from piloto import Piloto


def calibrar_no_sim(invertido, vies_graus, fps=20.0, limiar=0.0):
    pista, kw = sim.cenario("reta")
    params = parametros_padrao()
    res = config.CAMERA_RES
    rend = sim.Renderizador(CameraChao(params, res), res)
    carro = sim.Carro(pista, **kw)
    carro.invertido, carro.vies_graus, carro.limiar = invertido, vies_graus, limiar
    piloto = Piloto(params, res)
    estado = {"t": 0.0}

    def passo(turn_raw, speed):
        carro.passo(turn_raw, speed, 1.0 / fps)
        estado["t"] += 1.0 / fps
        return rend.quadro(pista, carro), estado["t"]

    return cd.calibrar(piloto, passo, log=lambda *_: None)


def correr(nome, invertido, vies_graus, cal):
    """Corre o cenario com o carro defeituoso e o piloto configurado pela calibracao."""
    pista, kw = sim.cenario(nome)
    params = parametros_padrao()
    res = config.CAMERA_RES
    rend = sim.Renderizador(CameraChao(params, res), res)
    carro = sim.Carro(pista, **kw)
    carro.invertido, carro.vies_graus = invertido, vies_graus
    piloto = Piloto(params, res)
    if cal is not None and cal.get("ok"):
        piloto.direcao.inverter = cal["inverter"]
        piloto.direcao.trim = cal["trim"]
    else:
        piloto.direcao.inverter = False
        piloto.direcao.trim = 0.0
    t, dt = 0.0, 1.0 / 12.0
    fim = pista.comp_total
    max_d = 0.0
    while True:
        s, d, fora, _ = sim.pose_stats(pista, carro, [])
        if s >= fim:
            return True, max_d, "completou"
        if t > 40:
            return False, max_d, "tempo esgotado"
        if fora > 6.0:
            return False, max_d, f"saiu da pista (s={s:.0f})"
        turn, speed = piloto.step(t, rend.quadro(pista, carro), fps=12)
        carro.passo(turn, speed, dt)
        t += dt
        max_d = max(max_d, abs(d))


def main():
    config.CONES_ATIVOS = False
    casos = [("normal", False, 0.0), ("volante invertido", True, 0.0),
             ("puxa p/ direita 3 graus", False, 3.0), ("puxa p/ esquerda 3 graus", False, -3.0),
             ("invertido + puxa 4 graus", True, 4.0), ("puxa p/ esquerda 6 graus", False, -6.0)]
    tudo_ok = True
    for nome, inv, vies in casos:
        cal = calibrar_no_sim(inv, vies)
        if not cal["ok"]:
            print(f"[FALHA] {nome:28s} calibracao falhou: {cal['motivo']}")
            tudo_ok = False
            continue
        acertou_inv = cal["inverter"] == inv
        ok1, d1, m1 = correr("curva_dir", inv, vies, cal)
        ok2, d2, m2 = correr("reta_desalinhado", inv, vies, cal)
        ok = acertou_inv and ok1 and ok2
        tudo_ok &= ok
        print(f"[{'OK ' if ok else 'FALHA'}] {nome:28s} inverter={cal['inverter']!s:5} (real {inv!s:5}) "
              f"trim {cal['trim']:+.2f} K {cal['K']:+6.0f} | curva_dir: {m1} (desv {d1:.1f}cm) | "
              f"reta: {m2} (desv {d2:.1f}cm)")
    # sem calibrar, pra mostrar o que ela resolve
    print("\n-- SEM calibrar (piloto assume volante normal e sem puxao):")
    for nome, inv, vies in casos[1:]:
        ok1, d1, m1 = correr("curva_dir", inv, vies, None)
        print(f"   {nome:28s} curva_dir: {m1} (desv {d1:.1f}cm)")
    return 0 if tudo_ok else 1


if __name__ == "__main__":
    sys.exit(main())
