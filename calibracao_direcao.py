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


def _assinatura(servo_cal):
    return [round(float(servo_cal["centro"]), 1), round(float(servo_cal["esquerda"]), 1),
            round(float(servo_cal["direita"]), 1)]


def carregar(servo_cal=None):
    """Calibracao salva, ou None. Se servo_cal for dado, so vale se foi feita COM ESSE servo
    (centro/esquerda/direita iguais): trocou o servo ou refez a calibracao dele = a direcao
    mudou e a medida antiga nao serve mais."""
    try:
        with open(ARQ_DIRECAO, "r", encoding="utf-8") as f:
            d = json.load(f)
        if servo_cal is not None and d.get("servo") != _assinatura(servo_cal):
            return None
        return {"inverter": bool(d["inverter"]), "trim": float(d.get("trim", 0.0)),
                "K": float(d.get("K", 0.0))}
    except (OSError, ValueError, KeyError):
        return None


def existe():
    return os.path.exists(ARQ_DIRECAO)


def salvar(res, servo_cal=None):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    d = {"inverter": bool(res["inverter"]), "trim": round(float(res["trim"]), 3),
         "K": round(float(res["K"]), 2)}
    if servo_cal is not None:
        d["servo"] = _assinatura(servo_cal)
    with open(ARQ_DIRECAO, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=2)


def _taxa_giro(ts, proas):
    """Giro do carro em graus/s (+ = pra direita) = -d(proa)/dt, por regressao linear."""
    if len(ts) < 4 or (max(ts) - min(ts)) < 0.18:
        return None
    return -float(np.polyfit(ts, proas, 1)[0])


class _Falha(Exception):
    pass


def calibrar(piloto, passo, log=print):
    """piloto: Piloto novo (so usado pra ler a proa da faixa).
    passo(turn_raw, speed) -> (frame, t): aplica o comando CRU (sem Stanley, sem inverter,
    sem trim) por um quadro e devolve o proximo quadro e a hora.
    Duas voltas (direita, esquerda); a 2a ja usa a compensacao estimada pela 1a (o carro anda
    mais reto) e as duas medidas de ganho sao CONFERIDAS entre si antes de aceitar.
    Devolve dict(ok, inverter, trim, K, motivo)."""
    piloto.adapt.desligar()      # os comandos de teste confundiriam o aprendizado andando
    piloto.protecoes = False     # e o desvio natural do teste nao pode virar 'erro saturado'
    V = max(config.CAL_DIR_VEL, config.PISO_MOTOR_MIN)
    giro = config.CAL_DIR_TURN
    estado = {"sem_faixa": 0}

    def fase(nome, raw, dur, medir, empurrao=False):
        ts, ps = [], []
        t0, t = None, 0.0
        while True:
            vel = V
            if empurrao and (t0 is None or (t - t0) < config.ARRANQUE_S):
                vel = config.ARRANQUE_VEL           # empurrao pra sair do repouso
            frame, t = passo(raw, vel)
            if t0 is None:
                t0 = t
            piloto.step(t, frame, fps=20.0, debug=False)
            ok = piloto.res is not None and piloto.res.centro is not None
            estado["sem_faixa"] = 0 if ok else estado["sem_faixa"] + 1
            if estado["sem_faixa"] >= 8:
                raise _Falha(f"perdi a faixa na fase '{nome}' (carro reto e na faixa? trecho reto da pista?)")
            if medir and ok and (t - t0) >= 0.2:        # pula o tempo de resposta do servo
                ts.append(t - t0)
                ps.append(piloto.proa_graus)
            if (t - t0) >= dur:
                break
        if not medir:
            return None
        taxa = _taxa_giro(ts, ps)
        if taxa is None:
            raise _Falha(f"poucas leituras da faixa na fase '{nome}'")
        log(f"  fase {nome:9s}: giro {taxa:+6.1f} graus/s ({len(ts)} leituras)")
        return taxa

    espelho = bool(getattr(piloto, "espelhar", False))     # imagem espelhada inverte o sinal medido

    def recentrar(dur):
        """Anda alguns instantes com o controlador de verdade (ja com o sentido do volante e o puxao que a
        1a volta descobriu) pra voltar ao centro da faixa. Se a medida estava errada isto piora, e a 2a
        volta e rejeitada."""
        t0, t, cmd = None, 0.0, 0.0
        while True:
            frame, t = passo(cmd, V)
            if t0 is None:
                t0 = t
            cmd, _ = piloto.step(t, frame, fps=20.0, debug=False)
            ok = piloto.res is not None and piloto.res.centro is not None
            estado["sem_faixa"] = 0 if ok else estado["sem_faixa"] + 1
            if estado["sem_faixa"] >= 8:
                raise _Falha("perdi a faixa ao recentrar o carro entre as voltas")
            if (t - t0) >= dur:
                return

    try:
        fase("arranque", 0.0, 0.5, False, empurrao=True)
        trim = 0.0
        ks = []
        for volta in (1, 2):
            yr = fase(f"direita{volta}", float(np.clip(trim + giro, -1, 1)), 0.6, True)
            yl = fase(f"esquerda{volta}", float(np.clip(trim - giro, -1, 1)), 0.6, True)
            k = (yr - yl) / (2.0 * giro)
            if abs(k) < config.CAL_DIR_KMIN:
                return {"ok": False, "K": k,
                        "motivo": f"o carro quase nao respondeu ao volante (ganho {k:+.1f} graus/s; minimo "
                                  f"{config.CAL_DIR_KMIN:.0f}). Servo solto, roda travada ou pouca potencia?"}
            ks.append(k)
            base = 0.5 * (yr + yl)                       # giro com o comando 'trim' (interpolado)
            trim = float(np.clip(trim - base / k, -config.CAL_DIR_TRIM_MAX, config.CAL_DIR_TRIM_MAX))
            if volta == 1:
                piloto.direcao.inverter = (k < 0) != espelho
                piloto.direcao.trim = trim
                piloto.direcao.reset()
                recentrar(0.7)
    except _Falha as e:
        return {"ok": False, "motivo": str(e)}

    k1, k2 = ks
    if k1 * k2 <= 0 or not (0.4 <= abs(k1 / k2) <= 2.5):
        return {"ok": False, "K": 0.5 * (k1 + k2),
                "motivo": (f"as duas voltas deram resultados diferentes (ganho {k1:+.0f} e {k2:+.0f} graus/s): "
                           f"medida inconsistente (camera estavel? faixa visivel o tempo todo?). Nao salvei.")}
    K = 0.5 * (k1 + k2)
    inverter = (K < 0) != espelho
    return {"ok": True, "inverter": inverter, "trim": trim, "K": K,
            "motivo": (f"volante {'INVERTIDO' if inverter else 'normal'} (ganho {K:+.0f} graus/s por unidade, "
                       f"duas voltas {k1:+.0f}/{k2:+.0f}); compensacao do puxao do carro {trim:+.2f}")}
