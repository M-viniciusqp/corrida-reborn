"""
autocal.py - autocalibracao ao ligar o carro (carro PARADO, reto, na faixa A).

Duas coisas que a propria pista ensina:

 1. INCLINACAO da camera: linhas paralelas da pista tem que ficar PARALELAS na
    vista de cima. Se a inclinacao suposta esta errada, elas abrem ou fecham em
    "V". O programa testa varias inclinacoes sobre as mesmas imagens e fica com
    a que deixa a distancia entre as linhas igual de perto e de longe.
    (Independe da escala, da posicao do carro e do giro da camera.)

 2. ESCALA: com a inclinacao certa, mede a distancia pontilhada <-> continua e
    compara com FAIXA_LARGURA_CM. No chao a escala e proporcional a altura da
    camera, entao corrige a altura.

NAO corrige guinada, rolagem e deslocamento lateral da camera: dessas o carro
nao tem como saber sozinho (nao diferencia "camera torta" de "carro torto").
Se vir poucas leituras, leitura instavel ou diferenca absurda, NAO aplica e
mantem o que esta salvo (calibrar_camera.py grava dados/camera.json).
"""

import math
import time

import numpy as np

import config
from faixas import DetectorFaixas, espacamento_perp
from geometria import CameraChao
from motor_auto import pequeno_cinza
from piloto import Piloto


def _pares(det, cam, frame):
    """Detecta linhas e devolve [(x_perto, x_longe, y_perto, y_longe)] pra cada linha
    com extensao suficiente (so linhas bem medidas servem pra checar paralelismo)."""
    res = det.processar(frame)
    y_p = cam.y_min + 10.0
    y_l = cam.y_max - 10.0
    out = []
    for l in res.linhas:
        if (l.y1 - l.y0) >= 0.4 * (cam.y_max - cam.y_min):
            out.append((float(l.x(y_p)), float(l.x(y_l)), y_p, y_l, l.peso))
    out.sort(key=lambda t: t[0])
    return out


def _custo_paralelismo(params, frames):
    """Mediana de |ln(espaco_longe / espaco_perto)| entre a melhor dupla de linhas
    vizinhas. None se nao achou dupla."""
    cam = CameraChao(params, config.CAMERA_RES)
    det = DetectorFaixas(cam)
    W = config.FAIXA_LARGURA_CM
    custos = []
    for fr in frames:
        linhas = _pares(det, cam, fr)
        melhor = None
        for i in range(len(linhas) - 1):
            a, b = linhas[i], linhas[i + 1]
            ep = b[0] - a[0]
            el = b[1] - a[1]
            if ep < 0.4 * W or el < 0.3 * W or ep > 2.0 * W or el > 2.5 * W:
                continue
            peso = a[4] + b[4]
            if melhor is None or peso > melhor[0]:
                melhor = (peso, abs(math.log(el / ep)))
        if melhor is not None:
            custos.append(melhor[1])
    if len(custos) < max(2, len(frames) // 3):
        return None
    return float(np.median(custos))


def _acertar_inclinacao(params, frames):
    """Varre a inclinacao em torno da atual. Devolve (nova_inclinacao ou None, texto)."""
    frames = frames[::max(1, len(frames) // 4)][:4]          # poucos quadros: o Pi e lento
    p0 = params["inclinacao"]
    candidatos = [p0 + d for d in np.arange(-14.0, 14.01, 2.0)]
    pontos = []
    for inc in candidatos:
        pr = dict(params)
        pr["inclinacao"] = float(inc)
        try:
            c = _custo_paralelismo(pr, frames)
        except ValueError:          # geometria impossivel (camera nao ve o chao)
            c = None
        if c is not None:
            pontos.append((c, float(inc)))
    if len(pontos) < 4:
        return None, "poucas leituras das duas linhas pra checar o paralelismo"
    pontos.sort()
    melhor_c, melhor_inc = pontos[0]
    # refina em torno do melhor, passo fino
    for inc in np.arange(melhor_inc - 1.0, melhor_inc + 1.01, 0.5):
        pr = dict(params)
        pr["inclinacao"] = float(inc)
        try:
            c = _custo_paralelismo(pr, frames)
        except ValueError:
            c = None
        if c is not None and c < melhor_c:
            melhor_c, melhor_inc = c, float(inc)
    try:
        atual = _custo_paralelismo(params, frames)
    except ValueError:
        atual = None
    if melhor_c > 0.12:
        return None, f"linhas nao ficam paralelas em nenhuma inclinacao (melhor custo {melhor_c:.2f}); mantive"
    if atual is not None and atual - melhor_c < 0.02:
        return None, f"inclinacao {p0:.1f} graus ja deixa as linhas paralelas"
    return melhor_inc, f"inclinacao {p0:.1f} -> {melhor_inc:.1f} graus (linhas paralelas)"


def _largura(params, frames):
    """Mediana da distancia entre linhas vizinhas identificadas (D-A, B-D) com a
    geometria dada, passando as imagens duas vezes pra classificacao assentar."""
    p = Piloto(params, config.CAMERA_RES)
    amostras = []
    for passada in range(2):
        for fr in frames:
            res = p.faixas.processar(fr)
            if passada == 0:
                continue
            y = p.faixas._y_ref()
            por = {l.ident: l for l in res.linhas if l.ident}
            if "D" in por and "A" in por:
                amostras.append(espacamento_perp(por["D"], por["A"], y))
            if "B" in por and "D" in por:
                amostras.append(espacamento_perp(por["B"], por["D"], y))
    return amostras


def autocalibrar(params, ler_frame, duracao=2.0, log=print):
    """ler_frame() -> quadro BGR no tamanho do piloto (ou None).
    Devolve (params, info); info['aplicada'] diz se mudou algo, info['motivo'] explica."""
    esperado = config.FAIXA_LARGURA_CM
    frames = []
    t0 = time.time()
    while time.time() - t0 < duracao and len(frames) < 10:
        f = ler_frame()
        if f is None:
            time.sleep(0.02)
            continue
        frames.append(f.copy())
        time.sleep(0.12)                      # quadros distintos
    # ruido do quadro PARADO (o governador do motor usa pra saber o que e 'cena parada')
    peq = [pequeno_cinza(f) for f in frames]
    ruido = float(np.mean([np.abs(peq[i + 1] - peq[i]).mean() for i in range(len(peq) - 1)]))         if len(peq) > 1 else 0.6
    info = {"aplicada": False, "motivo": "", "largura_medida": None, "ruido": ruido}
    if len(frames) < 4:
        info["motivo"] = "poucos quadros da camera; mantive a calibracao salva"
        return params, info

    novo = dict(params)
    partes = []

    # 1) inclinacao pelo paralelismo (DESLIGADA por padrao: no carro real ela se enganou)
    if config.AUTOCAL_INCLINACAO:
        try:
            inc, txt = _acertar_inclinacao(novo, frames)
        except ValueError as e:
            inc, txt = None, f"geometria invalida ({e})"
        if inc is not None and abs(inc - novo["inclinacao"]) > config.AUTOCAL_INCLINACAO_MAX:
            inc, txt = None, f"inclinacao sugerida {inc:.0f} graus muito longe da atual; mantive"
    else:
        inc, txt = None, f"inclinacao mantida em {novo['inclinacao']:.0f} graus"
    partes.append(txt)
    if inc is not None:
        novo["inclinacao"] = inc
        info["aplicada"] = True

    # 2) escala pela largura da faixa
    try:
        amostras = _largura(novo, frames)
    except ValueError:
        amostras = []
    if len(amostras) < 6:
        partes.append(f"largura: vi as duas linhas em poucos quadros ({len(amostras)}); escala mantida")
    else:
        med = float(np.median(amostras))
        disp = float(np.percentile(amostras, 75) - np.percentile(amostras, 25))
        info["largura_medida"] = med
        fator = esperado / med
        if disp > 0.25 * esperado:
            partes.append(f"largura: leitura instavel ({med - disp / 2:.0f} a {med + disp / 2:.0f} cm); escala mantida")
        elif abs(fator - 1.0) < 0.03:
            partes.append(f"largura {med:.1f} cm bate com {esperado:.0f} cm (escala ok)")
        elif abs(fator - 1.0) > config.AUTOCAL_ESCALA_MAX:
            partes.append(f"largura medida {med:.1f} cm esta longe de {esperado:.0f} cm (>{100 * config.AUTOCAL_ESCALA_MAX:.0f}%): "
                          f"confira FAIXA_LARGURA_CM, se o carro esta reto na faixa e rode calibrar_camera.py; escala mantida")
        else:
            novo["altura_cm"] = novo["altura_cm"] * fator
            info["aplicada"] = True
            partes.append(f"largura {med:.1f} -> {esperado:.0f} cm: altura {params['altura_cm']:.1f} -> "
                          f"{novo['altura_cm']:.1f} cm")
    info["motivo"] = " | ".join(partes)
    return (novo if info["aplicada"] else params), info
