"""
calibracao_direcao.py - o carro descobre SOZINHO, andando um pouco, duas coisas:

  1. PRA QUE LADO o volante vira (inverter ou nao): ele manda um comando conhecido
     (direita) e mede, pela camera, pra que lado o carro girou.
  2. Se o carro PUXA pra um lado com o volante "reto" (centro do servo desregulado,
     rodas desalinhadas, motores desiguais): mede o giro com o comando em zero e
     calcula o comando de compensacao (trim).

Como mede o giro: a camera enxerga a direcao da faixa em relacao ao carro (proa).
Se o carro gira pra direita, a faixa parece girar pra esquerda: giro = -d(proa)/dt.
Nao usa nenhum sensor alem da camera.

Sequencia (~1.7 s, ~1 m de pista RETA, carro reto na faixa no comeco):
    arranque  -> reto, empurrao (nao mede)
    direita   -> comando +CAL_DIR_TURN
    esquerda  -> comando -CAL_DIR_TURN
Ganho K = (giro_direita - giro_esquerda) / (2 * CAL_DIR_TURN) [graus/s por unidade].
    K > 0 -> volante normal | K < 0 -> INVERTIDO
Giro com comando 0 (puxao do carro) = media dos dois giros.
    trim  = -giro_base / K   (comando que zera o giro)

O resultado fica em dados/direcao.json e vale nas proximas corridas (a nao ser que
voce rode com --recalibrar). Se a faixa sumir ou o carro nao responder, nada e salvo.
"""

import json
import os

import numpy as np

import config

PASTA_DADOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dados")
ARQ_DIRECAO = os.path.join(PASTA_DADOS, "direcao.json")


def carregar():
    try:
        with open(ARQ_DIRECAO, "r", encoding="utf-8") as f:
            d = json.load(f)
        return {"inverter": bool(d["inverter"]), "trim": float(d.get("trim", 0.0)),
                "K": float(d.get("K", 0.0))}
    except (OSError, ValueError, KeyError):
        return None


def salvar(res):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    with open(ARQ_DIRECAO, "w", encoding="utf-8") as f:
        json.dump({"inverter": bool(res["inverter"]), "trim": round(float(res["trim"]), 3),
                   "K": round(float(res["K"]), 2)}, f, indent=2)


def _taxa_giro(ts, proas):
    """Giro do carro em graus/s (+ = pra direita) = -d(proa)/dt, por regressao linear."""
    if len(ts) < 4 or (max(ts) - min(ts)) < 0.18:
        return None
    return -float(np.polyfit(ts, proas, 1)[0])


def calibrar(piloto, passo, log=print):
    """piloto: Piloto novo (so usado pra ler a proa da faixa).
    passo(turn_raw, speed) -> (frame, t): aplica o comando CRU (sem Stanley, sem inverter,
    sem trim) por um quadro e devolve o proximo quadro e a hora.
    Devolve dict(ok, inverter, trim, K, motivo)."""
    V = max(config.CAL_DIR_VEL, config.VELOCIDADE_MIN)
    fases = [("arranque", 0.0, 0.5, False),
             ("direita", +config.CAL_DIR_TURN, 0.6, True),
             ("esquerda", -config.CAL_DIR_TURN, 0.6, True)]
    taxas = {}
    sem_faixa = 0
    for nome, raw, dur, medir in fases:
        ts, ps = [], []
        t0 = None
        while True:
            vel = V
            if nome == "arranque" and (t0 is None or (t - t0) < config.ARRANQUE_S):
                vel = config.ARRANQUE_VEL           # empurrao pra sair do repouso
            frame, t = passo(raw, vel)
            if t0 is None:
                t0 = t
            piloto.step(t, frame, fps=20.0, debug=False)
            ok = piloto.res is not None and piloto.res.centro is not None
            sem_faixa = 0 if ok else sem_faixa + 1
            if sem_faixa >= 8:
                return {"ok": False, "motivo": f"perdi a faixa na fase '{nome}' (carro reto e na faixa? "
                                               f"trecho reto da pista?)"}
            if medir and ok and (t - t0) >= 0.2:        # pula o tempo de resposta do servo
                ts.append(t - t0)
                ps.append(piloto.proa_graus)
            if (t - t0) >= dur:
                break
        if medir:
            taxa = _taxa_giro(ts, ps)
            if taxa is None:
                return {"ok": False, "motivo": f"poucas leituras da faixa na fase '{nome}'"}
            taxas[nome] = taxa
            log(f"  fase {nome:8s}: giro {taxa:+6.1f} graus/s ({len(ts)} leituras)")

    yr, yl = taxas["direita"], taxas["esquerda"]
    K = (yr - yl) / (2.0 * config.CAL_DIR_TURN)
    if abs(K) < config.CAL_DIR_KMIN:
        return {"ok": False, "K": K,
                "motivo": f"o carro quase nao respondeu ao volante (ganho {K:+.1f} graus/s; minimo "
                          f"{config.CAL_DIR_KMIN:.0f}). Servo solto, roda travada ou pouca potencia?"}
    base = 0.5 * (yr + yl)                       # giro com comando 0 (interpolado entre os dois testes)
    trim = float(np.clip(-base / K, -config.CAL_DIR_TRIM_MAX, config.CAL_DIR_TRIM_MAX))
    espelho = bool(getattr(piloto, "espelhar", False))     # imagem espelhada inverte o sinal medido
    inverter = (K < 0) != espelho
    return {"ok": True, "inverter": inverter, "trim": trim, "K": K,
            "motivo": (f"volante {'INVERTIDO' if inverter else 'normal'} (ganho {K:+.0f} graus/s por unidade); "
                       f"carro {'puxa pra ' + ('direita' if base > 0 else 'esquerda') if abs(base) > 3 else 'anda reto'}"
                       f" ({base:+.1f} graus/s) -> compensacao {trim:+.2f}")}
