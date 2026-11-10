"""
adaptacao.py - o carro se CALIBRA ANDANDO (nada de esperar antes de largar).

Larga na hora com padroes razoaveis e, enquanto segue a faixa, aprende pela propria camera:

  1. SENTIDO DO VOLANTE (invertido ou nao), PUXAO e FORCA da direcao.
     O giro do carro sai da camera: se o carro gira pra direita a faixa "gira" pra esquerda em
     relacao a ele, entao  giro = -d(proa)/dt + (giro da propria pista = v * curvatura).
     Uma regressao com esquecimento (~2 s) liga o comando de volante ao giro medido:
            giro = K * comando + b
        K < 0 (e persistente)  -> o volante esta INVERTIDO: inverte sozinho
        K > 0                  -> sentido confirmado; a razao K / K_nominal ajusta o GANHO
                                  (direcao fraca/forte) e b (giro com comando zero) ajusta o TRIM
                                  (carro que puxa pra um lado)
     O proprio controle da pista (que corrige o tempo todo) ja excita o carro: nao precisa de
     manobra de teste. Sem variacao de comando nao decide nada (nao chuta).

  2. LADO DA FAIXA CONTINUA (como o gatuno-rapido, que aprende qual lado tem buracos).
     Compara a cobertura da linha mais perto de cada lado; se a evidencia mandar o contrario do
     que o carro assume por varios quadros, espelha a imagem; depois de concordar por tempo
     suficiente TRAVA a decisao (um trecho ruim nao pode mudar de ideia no meio da corrida).

  3. LARGURA DA FAIXA: aprendida em faixas.py (DetectorFaixas.W).

O que foi aprendido vira dados/auto.json ao parar e e lido na proxima largada; mesmo sem o
arquivo o carro larga e se ajusta em poucos segundos (com velocidade limitada ate o sentido do
volante ser confirmado).
"""

import collections
import json
import math
import os

import numpy as np

import config


class Adaptacao:
    def __init__(self, piloto):
        self.p = piloto
        self.ativa = True
        self.virou_agora = False
        self.confirmada = False            # sentido do volante confirmado nesta corrida (ou lido do arquivo)
        self.lado_travado = config.LADO_CONTINUO in ("direita", "esquerda")
        self._lado_ema = 0.0
        self._lado_neg = 0.0
        self._lado_pos = 0.0
        self.K = 0.0
        self.b = 0.0
        self.eventos = []
        self._h_proa = collections.deque()      # (t, proa) da janela curta
        self._h_u = collections.deque()         # (t, comando logico) do ultimo segundo
        self._ae_ant = None
        self._taxa_ema = 0.0
        self._h_taxa = collections.deque()      # (t, taxa de crescimento do erro)
        self._div_s = 0.0
        self._zerar()
        self._neg_s = 0.0
        self._pos_s = 0.0
        self._t_flip = -99.0
        self._t_ajuste = 0.0
        self._t_ini = None              # quando comecou a observar (janela do detector rapido)

    # ------------------------------------------------------------------
    def _zerar(self):
        self.Sw = self.Sx = self.Sy = self.Sxx = self.Sxy = 0.0
        self._u_reta = 0.0              # media (suavizada) do comando logico nas RETAS
        self._t_reta = 0.0
        self._h_proa.clear()
        self._h_u.clear()
        self._ae_ant = None
        self._taxa_ema = 0.0
        self._h_taxa.clear()
        self._div_s = 0.0
        self._neg_s = 0.0
        self._pos_s = 0.0
        self.K = 0.0
        self.b = 0.0

    def sem_leitura(self):
        """Faixa perdida/parado: corta a continuidade da derivada (nao mede giro com buraco)."""
        self._h_proa.clear()
        self._h_u.clear()
        self._ae_ant = None

    def desligar(self):
        """Desliga o aprendizado (ex.: durante a calibracao antiga com comandos de teste)."""
        self.ativa = False
        self.lado_travado = True
        self.confirmada = True

    @property
    def direcao_ok(self):
        return self.confirmada or not self.ativa

    # ------------------------------------------------------------------
    def atualizar(self, t, dt, proa_graus, curvatura_sinal, speed_cmd, parado, erro_cm=0.0):
        """Chame a cada quadro COM leitura boa da faixa. Mede o giro e atualiza K, b."""
        self.eventos = []
        self.virou_agora = False
        if not config.ADAPTACAO or not self.ativa:
            return
        if dt < 0.02 or dt > 0.25 or parado or speed_cmd < 0.3:
            self.sem_leitura()
            return
        p = self.p
        u_now = p.direcao.turn
        if self._t_ini is None:
            self._t_ini = t
        self._h_proa.append((t, proa_graus))
        self._h_u.append((t, u_now))
        while self._h_proa and t - self._h_proa[0][0] > config.ADAPT_JANELA_S:
            self._h_proa.popleft()
        while self._h_u and t - self._h_u[0][0] > 1.2:
            self._h_u.popleft()

        # ---- media do volante nas RETAS: o que o controle precisa manter pra andar reto e o puxao do carro
        if abs(curvatura_sinal) < config.ADAPT_RETA_CURV and abs(proa_graus) < config.ADAPT_RETA_PROA:
            a_m = 1.0 - math.exp(-dt / config.ADAPT_TRIM_TAU_S)
            self._u_reta += a_m * (u_now - self._u_reta)
            self._t_reta += dt

        # ---- detector RAPIDO de volante invertido: o erro CRESCE enquanto o comando tenta REDUZI-LO
        # (erro > 0 = o centro esta a direita; u > 0 = o controle manda virar a direita; se mesmo assim
        # o erro aumenta, o volante faz o contrario do que se pensa)
        ae = abs(erro_cm)
        taxa = 0.0 if self._ae_ant is None else (ae - self._ae_ant) / dt
        self._ae_ant = ae
        self._taxa_ema = 0.6 * self._taxa_ema + 0.4 * taxa
        reduzindo = (abs(u_now) >= config.ADAPT_DIV_U_MIN and u_now * erro_cm > 0)
        # com a evidencia de LADO contradizendo o que se assume (continua do outro lado), o carro erra
        # porque enxerga a pista espelhada, nao porque o volante e invertido: deixa o lado resolver
        lado_ok = self.lado_travado or self._lado_ema > -0.5 * config.LADO_MARGEM
        # veto: se a regressao do giro JA mostra sentido normal (ganho positivo), o erro crescendo e so
        # correcao fraca (puxao do carro, curva), nao volante invertido
        k0 = (self.Sxy / self.Sxx) if (self.Sw >= 4 and self.Sxx / max(self.Sw, 1e-9) >= config.ADAPT_DIV_VETO_RMS2
                                       and self.Sxx > 1e-9) else 0.0
        veto = config.ADAPT_DIV_VETO and k0 > config.ADAPT_DIV_VETO_K
        # O erro de um volante invertido ACELERA (realimentacao positiva: a taxa de crescimento sobe, ex.
        # 5 -> 20 cm/s em 1 s); o de um carro que so puxa pra um lado cresce a taxa ~constante. Compara a
        # taxa de agora com a de ~0.3 s atras.
        self._h_taxa.append((t, self._taxa_ema))
        while self._h_taxa and t - self._h_taxa[0][0] > 0.7:
            self._h_taxa.popleft()
        antes = [x for x in self._h_taxa if t - x[0] >= 0.3]
        taxa_antes = antes[-1][1] if antes else None
        crescendo = (taxa_antes is not None and self._taxa_ema > config.ADAPT_DIV_TAXA_CMS
                     and self._taxa_ema > config.ADAPT_DIV_ACEL * max(taxa_antes, 1.0)
                     and ae >= config.ADAPT_DIV_ERRO_MIN)
        # nos primeiros segundos vale o gatuilho simples (erro crescendo rapido); depois so se o erro ACELERA
        janela_div = (t - self._t_ini) < config.ADAPT_DIV_JANELA_S
        rapido = janela_div and self._taxa_ema > config.ADAPT_DIV_TAXA_CMS and ae >= 1.0
        if reduzindo and lado_ok and not veto and (crescendo or rapido) and not self.confirmada:
            self._div_s += dt
        else:
            self._div_s = max(0.0, self._div_s - 2.0 * dt)
        if self._div_s >= config.ADAPT_DIV_PERSISTE_S and t - self._t_flip > 2.0:
            self._inverter(t, f"o erro acelera ({ae:.0f} cm, +{self._taxa_ema:.0f} cm/s) mesmo com o volante tentando reduzir")
            return

        if len(self._h_proa) < 4:
            return
        # ---- giro do carro: inclinacao da proa numa janela curta (menos ruido que quadro a quadro)
        ts = np.array([x[0] for x in self._h_proa])
        ps = np.array([x[1] for x in self._h_proa])
        if ts[-1] - ts[0] < 0.6 * config.ADAPT_JANELA_S:
            return
        dproa = float(np.polyfit(ts - ts[0], ps, 1)[0])
        v = speed_cmd * config.V_CMS_NA_VEL_1
        omega = float(np.clip(-dproa + math.degrees(v * curvatura_sinal), -200.0, 200.0))
        # comando de quando o carro COMECOU a responder: centro da janela menos a demora do servo
        t_u = ts[-1] - 0.5 * config.ADAPT_JANELA_S - config.ADAPT_ATRASO_S
        tu = np.array([x[0] for x in self._h_u])
        uu = np.array([x[1] for x in self._h_u])
        if t_u < tu[0]:
            return
        u = float(np.interp(t_u, tu, uu))

        lam = math.exp(-dt / config.ADAPT_TAU_S)
        for nome in ("Sw", "Sx", "Sy", "Sxx", "Sxy"):
            setattr(self, nome, getattr(self, nome) * lam)
        self.Sw += 1.0
        self.Sx += u
        self.Sy += omega
        self.Sxx += u * u
        self.Sxy += u * omega

        if self.Sw < config.ADAPT_N_MIN:
            return
        # ganho "pela origem": K = sum(u*w) / sum(u*u). So pede comandos de modulo razoavel com sinal
        # consistente (nao precisa de variacao). Num carro que PUXA pra um lado o puxao vira intercepto e
        # nao muda o sinal; num volante invertido de verdade fica negativo desde os primeiros instantes.
        if self.Sxx / self.Sw < config.ADAPT_RMS2_MIN or self.Sxx < 1e-9:
            self._neg_s = max(0.0, self._neg_s - dt)
            self._pos_s = max(0.0, self._pos_s - dt)
            return                                               # comando pequeno demais: nao decide
        self.K = self.Sxy / self.Sxx
        self.b = 0.0

        # ---- sentido do volante
        if self.K < -config.ADAPT_K_FLIP:
            self._neg_s += dt
            self._pos_s = 0.0
        elif self.K > config.ADAPT_K_OK:
            self._pos_s += dt
            self._neg_s = 0.0
        else:
            self._neg_s = max(0.0, self._neg_s - dt)
            self._pos_s = max(0.0, self._pos_s - dt)
        if self._neg_s >= config.ADAPT_PERSISTE_S and t - self._t_flip > 2.0:
            self._inverter(t, f"ganho medido {self.K:+.0f} graus/s por unidade")
            return
        if self._pos_s >= config.ADAPT_PERSISTE_S and not self.confirmada:
            self.confirmada = True
            self.eventos.append(f"[auto] sentido do volante CONFIRMADO (ganho {self.K:+.0f} graus/s por unidade)")

        # ---- trim e ganho (devagar, so com o sentido confirmado)
        if self.confirmada and t - self._t_ajuste >= config.ADAPT_AJUSTE_S:
            self._t_ajuste = t
            d = p.direcao
            if self._t_reta >= config.ADAPT_TRIM_MIN_RETA_S:
                # o controle precisa manter u_reta de volante (logico) so pra andar reto = puxao do carro:
                # passa isso pro trim (comando fisico = sinal * logico + trim) e o controle se alivia.
                passo = config.ADAPT_TRIM_PASSO * self._u_reta
                d.trim = float(np.clip(d.trim + d.sinal() * passo, -config.CAL_DIR_TRIM_MAX, config.CAL_DIR_TRIM_MAX))
                self._u_reta -= passo
            if config.ADAPT_GANHO_ATIVO and self.K > config.ADAPT_K_OK and self.Sw >= 1.4 * config.ADAPT_N_MIN:
                nominal = math.degrees(v * math.tan(math.radians(config.ESTERCO_MAX_GRAUS)) / config.CARRO_ENTREEIXOS_CM)
                alvo_ganho = float(np.clip(nominal / self.K, config.ADAPT_GANHO_MIN, config.ADAPT_GANHO_MAX))
                d.ganho += 0.15 * (alvo_ganho - d.ganho)

    def _inverter(self, t, motivo):
        d = self.p.direcao
        d.inverter = not d.inverter      # o comando logico fica: so o sinal FISICO muda, na hora
        self.virou_agora = True
        self._t_flip = t
        self._t_ini = t                     # rearma a janela do detector rapido
        self.confirmada = False
        self.eventos.append(f"[auto] volante parece INVERTIDO ({motivo}): invertendo -> "
                            f"{'invertido' if d.inverter else 'normal'}")
        self._zerar()

    # ------------------------------------------------------------------
    def lado(self, linhas, y_ref, dt=0.05):
        """Evidencia de qual lado e a CONTINUA. Tudo em TEMPO (igual em qualquer FPS).
        Devolve True se for pra ALTERNAR o espelho."""
        if self.lado_travado:
            return False
        esq = [l for l in linhas if l.cobertura is not None and -0.9 * self.p.faixas.W < float(l.x(y_ref)) < 0]
        dire = [l for l in linhas if l.cobertura is not None and 0 < float(l.x(y_ref)) < 0.9 * self.p.faixas.W]
        if not esq or not dire:
            return False
        c_esq = max(esq, key=lambda l: float(l.x(y_ref))).cobertura        # a mais perto, a esquerda
        c_dir = min(dire, key=lambda l: float(l.x(y_ref))).cobertura       # a mais perto, a direita
        a = 1.0 - math.exp(-max(dt, 0.005) / config.LADO_TAU_S)
        self._lado_ema += a * ((c_dir - c_esq) - self._lado_ema)
        # no mundo em que o piloto enxerga a continua deve estar a DIREITA: evidencia > 0
        if self._lado_ema < -config.LADO_MARGEM:
            self._lado_neg += dt
            self._lado_pos = 0.0
        elif self._lado_ema > config.LADO_MARGEM:
            self._lado_pos += dt
            self._lado_neg = 0.0
        else:
            self._lado_neg = max(0.0, self._lado_neg - dt)
            self._lado_pos = max(0.0, self._lado_pos - dt)
        if self._lado_pos >= config.LADO_TRAVA_S:
            self.lado_travado = True
            self.eventos.append("[auto] lado da faixa continua TRAVADO (evidencia consistente)")
            return False
        if self._lado_neg >= config.LADO_TROCA_S:
            self._lado_neg = self._lado_pos = 0.0
            self._lado_ema = 0.0
            self.eventos.append("[auto] a faixa continua esta do OUTRO lado: espelhando a imagem")
            return True
        return False

    # ------------------------------------------------------------------
    def estado(self):
        d = self.p.direcao
        return {"inverter": bool(d.inverter), "trim": round(float(d.trim), 3), "ganho": round(float(d.ganho), 3),
                "W": round(float(self.p.faixas.W), 2), "W_amostras": int(self.p.faixas.W_amostras),
                "espelhar": bool(self.p.espelhar), "lado_travado": bool(self.lado_travado),
                "confirmada": bool(self.confirmada)}

    def aplicar(self, e):
        """Aplica o que foi aprendido numa corrida anterior (dados/auto.json)."""
        d = self.p.direcao
        if e.get("confirmada"):
            d.inverter = bool(e.get("inverter", d.inverter))
            d.trim = float(e.get("trim", 0.0))
            d.ganho = float(np.clip(e.get("ganho", 1.0), config.ADAPT_GANHO_MIN, config.ADAPT_GANHO_MAX))
            self.confirmada = True
        if e.get("W_amostras", 0) >= 30:
            f = self.p.faixas
            f.W = float(np.clip(e.get("W", f.W0), 0.7 * f.W0, 1.5 * f.W0))
            f.W_amostras = 30
        if e.get("lado_travado") and config.LADO_CONTINUO == "auto":
            self.p.espelhar = bool(e.get("espelhar", False))
            self.lado_travado = True


# ---------------------------------------------------------------------
# o que foi aprendido andando fica em dados/auto.json (nao vai pro git)
# ---------------------------------------------------------------------
PASTA_DADOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dados")
ARQ_AUTO = os.path.join(PASTA_DADOS, "auto.json")


def _assinatura(servo_cal):
    return [round(float(servo_cal["centro"]), 1), round(float(servo_cal["esquerda"]), 1),
            round(float(servo_cal["direita"]), 1)]


def carregar_auto(servo_cal=None):
    """Estado aprendido numa corrida anterior, ou None. A parte da DIRECAO (sentido do volante,
    trim) so vale pro MESMO servo/calibracao de servo; largura e lado da faixa valem sempre."""
    try:
        with open(ARQ_AUTO, "r", encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    if servo_cal is not None and d.get("servo") != _assinatura(servo_cal):
        for k in ("inverter", "trim", "ganho", "confirmada"):
            d.pop(k, None)
    return d


def salvar_auto(estado, servo_cal=None):
    os.makedirs(PASTA_DADOS, exist_ok=True)
    d = dict(estado)
    if servo_cal is not None:
        d["servo"] = _assinatura(servo_cal)
    with open(ARQ_AUTO, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=2)
