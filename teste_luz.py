"""
teste_luz.py - a visao aguenta QUALQUER luminosidade? (simulador)

Aplica nas imagens da camera: escuro, claro, estourado, sombra em degrade, luz amarela/azulada,
ruido forte, desfoque e reflexo, e confere se o carro ainda completa a pista (largando sem calibrar).

    python teste_luz.py
"""

import sys

import cv2
import numpy as np

import config
import sim

PISTAS = ("reta_desalinhado", "curva_dir", "curva_esq")


def _ruido(img, sigma, rng):
    return np.clip(img.astype(np.float32) + rng.normal(0, sigma, img.shape), 0, 255)


def condicao(ganho=1.0, vies=0.0, sigma=4.0, degrade=None, tinta=(1.0, 1.0, 1.0), desfoque=0, reflexo=False,
             gama=1.0):
    rng = np.random.default_rng(11)

    def f(img):
        x = img.astype(np.float32) / 255.0
        x = np.power(np.clip(x, 0, 1), gama)
        x = x * np.array(tinta, np.float32)[None, None, :]
        h, w = x.shape[:2]
        if degrade is not None:                     # sombra: de uma lado escuro pro outro claro
            rampa = np.linspace(degrade[0], degrade[1], w, dtype=np.float32)[None, :, None]
            x = x * rampa
        x = x * ganho * 255.0 + vies
        if reflexo:
            ov = np.zeros((h, w, 3), np.float32)
            cv2.circle(ov, (int(w * 0.35), int(h * 0.78)), int(h * 0.22), (90, 90, 90), -1)
            x = x + cv2.GaussianBlur(ov, (0, 0), 14)
        x = np.clip(x, 0, 255)
        if desfoque:
            k = np.zeros((desfoque, 1), np.float32)
            k[:, 0] = 1.0 / desfoque
            x = cv2.filter2D(x, -1, k)
        x = _ruido(x, sigma, rng)
        return np.clip(x, 0, 255).astype(np.uint8)
    return f


CONDICOES = [
    ("normal", condicao()),
    ("escuro 0.50", condicao(ganho=0.50, sigma=6)),
    ("escuro 0.30", condicao(ganho=0.30, sigma=8)),
    ("MUITO escuro 0.15", condicao(ganho=0.15, sigma=6)),
    ("claro 1.6", condicao(ganho=1.6)),
    ("estourado 2.2", condicao(ganho=2.2)),
    ("sombra em degrade", condicao(degrade=(0.35, 1.15))),
    ("luz amarela", condicao(tinta=(0.75, 0.95, 1.2))),
    ("luz azulada", condicao(tinta=(1.25, 1.0, 0.8))),
    ("ruido forte 25", condicao(sigma=25)),
    ("escuro + ruido + sombra", condicao(ganho=0.35, sigma=12, degrade=(0.5, 1.1))),
    ("gama 2.0 (contraste achatado)", condicao(gama=2.0)),
    ("desfoque + reflexo", condicao(desfoque=7, reflexo=True)),
]


def main():
    config.CONES_ATIVOS = False
    orig = sim.Renderizador.quadro
    total = ok_total = 0
    for nome, filtro in CONDICOES:
        def q(self, pista, carro, claridade=1.0, ruido=4.0, _f=filtro):
            return _f(orig(self, pista, carro, 1.0, 0.0))
        sim.Renderizador.quadro = q
        res = [sim.rodar(p, quiet=True, online=True, fps=20.0) for p in PISTAS]
        total += len(res)
        ok_total += sum(res)
        print(f"[{'OK ' if all(res) else 'FALHA'}] {nome:32s} " + " ".join("ok" if r else "FALHOU" for r in res))
    sim.Renderizador.quadro = orig
    print(f"\n{ok_total}/{total} corridas completas em condicoes de luz diferentes")
    return 0 if ok_total == total else 1


if __name__ == "__main__":
    sys.exit(main())
