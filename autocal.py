"""
autocal.py - autocalibracao da ESCALA ao ligar o carro.

Ideia: a pista tem uma medida conhecida (FAIXA_LARGURA_CM, de linha a linha).
Com o carro PARADO dentro da faixa, o programa mede a distancia entre a
pontilhada e a continua (e entre a continua externa e a pontilhada, se vir) por
~3 s. Se a medida difere da esperada, o erro e de ESCALA, e escala no chao e
proporcional a altura da camera (com inclinacao e campo de visao fixos). Entao
ele corrige a altura e recomeca com a geometria certa.

O que isto NAO corrige (precisa de voce, uma vez so, com calibrar_camera.py):
inclinacao, guinada e rolagem da camera. Dessas o carro nao tem como saber
sozinho (nao ha como diferenciar "camera torta" de "carro torto na pista").

Resultado so vale se: carro na faixa A, reto, as duas linhas (pontilhada e
continua) visiveis. Se nao vir as duas, mantem a calibracao salva.
"""

import time

import numpy as np

import config
from piloto import Piloto


def _larguras(piloto, frame):
    res = piloto.faixas.processar(frame)
    y = piloto.faixas._y_ref()
    por = {l.ident: float(l.x(y)) for l in res.linhas if l.ident}
    v = []
    if "D" in por and "A" in por:
        v.append(por["A"] - por["D"])
    if "B" in por and "D" in por:
        v.append(por["D"] - por["B"])
    return v


def autocalibrar(params, ler_frame, duracao=3.0, minimo=10, log=print):
    """ler_frame() -> quadro BGR no tamanho do piloto (ou None). Devolve (params, info).
    info = dict(fator, largura_medida, aplicada, motivo)."""
    esperado = config.FAIXA_LARGURA_CM
    p = Piloto(params, config.CAMERA_RES)
    t0, n = time.time(), 0
    amostras = []
    while time.time() - t0 < duracao:
        frame = ler_frame()
        if frame is None:
            time.sleep(0.02)
            continue
        n += 1
        v = _larguras(p, frame)
        if n > 6:                      # as primeiras leituras ainda estao assentando
            amostras += v
        time.sleep(0.03)
    info = {"fator": 1.0, "largura_medida": None, "aplicada": False, "motivo": ""}
    if len(amostras) < minimo:
        info["motivo"] = (f"vi as duas linhas em poucos quadros ({len(amostras)}); "
                          f"mantive a calibracao salva")
        return params, info
    med = float(np.median(amostras))
    dispersao = float(np.percentile(amostras, 75) - np.percentile(amostras, 25))
    info["largura_medida"] = med
    fator = esperado / med
    info["fator"] = fator
    if dispersao > 0.25 * esperado:
        info["motivo"] = f"leitura instavel (medidas de {med - dispersao / 2:.0f} a {med + dispersao / 2:.0f} cm); nao apliquei"
        return params, info
    if abs(fator - 1.0) < 0.03:
        info["motivo"] = f"largura {med:.1f} cm bate com {esperado:.0f} cm (ja calibrado)"
        return params, info
    if not (0.7 <= fator <= 1.4):
        info["motivo"] = (f"largura medida {med:.1f} cm esta MUITO longe de {esperado:.0f} cm: "
                          f"confira FAIXA_LARGURA_CM e a calibracao da camera; nao apliquei")
        return params, info
    novo = dict(params)
    novo["altura_cm"] = params["altura_cm"] * fator
    info["aplicada"] = True
    info["motivo"] = (f"largura {med:.1f} cm -> {esperado:.0f} cm: altura da camera "
                      f"{params['altura_cm']:.1f} -> {novo['altura_cm']:.1f} cm")
    return novo, info
