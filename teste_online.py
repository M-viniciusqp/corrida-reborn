"""
teste_online.py - o carro LARGA NA HORA, sem nenhuma calibracao previa, e se ajusta andando.

Simula carros com defeitos (volante invertido, puxando pra um lado, direcao fraca/forte, faixa de
largura diferente da do config, pista espelhada) e confere se ele completa a pista mesmo assim.

    python teste_online.py          # a bateria
    python teste_online.py -v       # mostra o que ele foi aprendendo
"""

import sys

import config
import piloto as P
import sim

CARROS = [
    ("normal", dict()),
    ("volante INVERTIDO", dict(invertido=True)),
    ("puxa 4 graus p/ direita", dict(vies_graus=4.0)),
    ("puxa 4 graus p/ esquerda", dict(vies_graus=-4.0)),
    ("direcao FRACA (x0.5)", dict(ganho_fisico=0.5)),
    ("direcao FORTE (x1.6)", dict(ganho_fisico=1.6)),
    ("faixa REAL de 34 cm (config 28)", dict(largura=34.0)),
    ("invertido + puxa 3 graus", dict(invertido=True, vies_graus=3.0)),
]
PISTAS = ("reta", "curva_dir", "curva_esq")
FPS = 20.0          # o Pi real roda a 17-30 FPS (o simulador a 12 e pessimista)


def main():
    config.CONES_ATIVOS = False
    verbose = "-v" in sys.argv
    capturados = []
    orig_init = P.Piloto.__init__

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        capturados.append(self)
    P.Piloto.__init__ = init
    total = ok_total = 0
    for nome, defeito in CARROS:
        resultados, estados = [], []
        for pista in PISTAS:
            capturados.clear()
            ok = sim.rodar(pista, quiet=True, online=True, verbose_eventos=verbose, fps=FPS, **defeito)
            resultados.append(ok)
            estados.append(capturados[-1].adapt.estado() if capturados else {})
            total += 1
            ok_total += int(ok)
        e = estados[-1]
        print(f"[{'OK ' if all(resultados) else 'FALHA'}] {nome:34s} reta/curva_dir/curva_esq: "
              f"{' '.join('ok' if r else 'FALHOU' for r in resultados):18s} | aprendeu: inverter={e.get('inverter')} "
              f"trim={e.get('trim'):+.2f} ganho={e.get('ganho'):.2f} W={e.get('W'):.1f}")
    # pista espelhada (continua a esquerda), sem avisar
    res_esp = [sim.rodar(p, quiet=True, online=True, fps=FPS) for p in ("reta_espelhada", "curva_dir_espelhada", "curva_esq_espelhada")]
    total += 3
    ok_total += sum(res_esp)
    print(f"[{'OK ' if all(res_esp) else 'FALHA'}] {'pista ESPELHADA (continua a esquerda)':34s} reta/curva_dir/curva_esq: "
          f"{' '.join('ok' if r else 'FALHOU' for r in res_esp)}")
    print(f"\n{ok_total}/{total} corridas completas SEM calibrar antes")
    return 0 if ok_total == total else 1


if __name__ == "__main__":
    sys.exit(main())
