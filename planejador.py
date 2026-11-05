"""
planejador.py - decide o DESLOCAMENTO LATERAL (cm) do alvo pra desviar de cones.

O carro sempre segue o centro da faixa A + um deslocamento `o` (cm, + = direita).
Sem cone, o = 0. Com cone:

  1. Posicao do cone em coordenadas da faixa: rel = x_cone - centro(y_cone)
     (0 = centro da faixa A, -L/2 = pontilhada, +L/2 = continua).
  2. Folga necessaria entre o centro do carro e o centro do cone:
         folga = (cone + carro)/2 + margem
     Se |rel| >= folga o carro ja passa sem mudar nada -> ignora
     (cone da outra faixa, longe da pista, etc).
  3. Senao, dois jeitos de passar:
         esquerda:  o = rel - folga   (cruza a pontilhada, permitido)
         direita :  o = rel + folga   (nao pode encostar na continua)
     Vale o jeito que cabe na pista; se os dois cabem, o de menor desvio.
     Se nenhum cabe com a margem cheia, tenta com margem menor ("espremer").
     A decisao fica congelada por cone (sem trocar de lado no meio).
  4. O desvio e aplicado gradualmente com a distancia (smoothstep entre
     DESVIO_INICIO_CM e DESVIO_COMPLETO_CM) e suavizado por limite de taxa.
  5. O cone some da imagem antes do carro terminar de passar. Por isso o
     desvio e SEGURADO por (distancia ate o cone + comprimento do carro)
     contada pela velocidade estimada, e so depois volta pro centro.

Nao ha parada: e um desvio continuo (o gatuno-rapido para, analisa e desvia).
"""

import math

import config


def _smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class Planejador:
    def __init__(self):
        self.o_cmd = 0.0          # deslocamento aplicado (cm)
        self.o_alvo = 0.0         # deslocamento desejado agora
        self.fator_vel = 1.0      # 1 = livre; menor = reduzir (cone perto/apertado)
        self.cone_alvo = None     # cone que esta mandando
        self.eventos = []
        # memoria do cone que acabou de sumir da imagem
        self._mem_o = 0.0
        self._mem_dist = 0.0
        self._lados = {}          # id do cone -> 'E' | 'D' | None
        self._estado = "livre"

    # ------------------------------------------------------------------
    def limites(self):
        W = config.FAIXA_LARGURA_CM
        meia_carro = config.CARRO_LARGURA_CM / 2.0
        o_max_dir = W / 2.0 - meia_carro - config.MARGEM_CONTINUA_CM
        o_max_esq = -(1.5 * W - meia_carro - config.MARGEM_CONTINUA_CM)
        return o_max_esq, o_max_dir

    def _escolher(self, cone, rel):
        """Devolve (o_final, lado, apertado) ou None se nao precisa desviar."""
        o_min, o_max = self.limites()
        for fator_margem in (1.0, 0.35):
            folga = ((config.CONE_LARGURA_CM + config.CARRO_LARGURA_CM) / 2.0
                     + config.CONE_MARGEM_CM * fator_margem)
            if abs(rel) >= folga:
                return None
            o_esq = rel - folga
            o_dir = rel + folga
            cabe_e = o_esq >= o_min
            cabe_d = o_dir <= o_max
            if cabe_e or cabe_d:
                lado = self._lados.get(cone.id)
                if lado == "E" and cabe_e:
                    return o_esq, "E", fator_margem < 1.0
                if lado == "D" and cabe_d:
                    return o_dir, "D", fator_margem < 1.0
                if cabe_e and cabe_d:
                    lado = "E" if abs(o_esq) <= abs(o_dir) else "D"
                else:
                    lado = "E" if cabe_e else "D"
                return (o_esq if lado == "E" else o_dir), lado, fator_margem < 1.0
        # nenhum cabe: fica com o lado que mais se aproxima de caber (menor
        # invasao), preso aos limites da pista, e vai devagar
        folga = ((config.CONE_LARGURA_CM + config.CARRO_LARGURA_CM) / 2.0
                 + config.CONE_MARGEM_CM * 0.35)
        falta_e = o_min - (rel - folga)       # quanto falta pro lado esquerdo caber
        falta_d = (rel + folga) - o_max
        if falta_e <= falta_d:
            return max(rel - folga, o_min), "E", True
        return min(rel + folga, o_max), "D", True

    # ------------------------------------------------------------------
    def atualizar(self, dt, cones, centro, v_cms):
        """cones: confirmados visiveis. centro: faixas.Centro (ou None)."""
        self.eventos = []
        L = config.CARRO_COMPRIMENTO_CM
        alvo_visivel = 0.0
        self.fator_vel = 1.0
        self.cone_alvo = None
        agindo = None

        if centro is not None:
            # so decide com o cone DENTRO da zona onde a faixa foi medida (+ uma
            # folga curta): mais longe, a faixa extrapolada em curva erra dezenas de cm
            y_decisao = min(config.DESVIO_INICIO_CM + 25.0, centro.y1 + 30.0)
            for c in cones:                         # ordenados do mais perto ao mais longe
                if c.y > y_decisao:
                    continue
                rel = float(c.x - centro.x(c.y))
                esc = self._escolher(c, rel)
                if esc is None:
                    continue
                o_fin, lado, apertado = esc
                if c.id not in self._lados or self._lados[c.id] is None:
                    self._lados[c.id] = lado
                    self.eventos.append(
                        f"[desvio] cone #{c.id} a {c.y:.0f}cm, {rel:+.0f}cm do centro da faixa "
                        f"-> passa pela {'ESQUERDA' if lado == 'E' else 'DIREITA'} "
                        f"(o={o_fin:+.0f}cm){' APERTADO' if apertado else ''}")
                s = _smoothstep((config.DESVIO_INICIO_CM - c.y) /
                                (config.DESVIO_INICIO_CM - config.DESVIO_COMPLETO_CM))
                o_i = s * o_fin
                if agindo is None:
                    agindo = c
                    alvo_visivel = o_i
                    self.cone_alvo = c
                    self.fator_vel = 1.0 - config.VEL_REDUCAO_CONE * s * (1.5 if apertado else 1.0)
                    # memoria: enquanto o cone esta a vista ela se renova
                    self._mem_o = o_fin * (1.0 if s > 0.05 else 0.0)
                    self._mem_dist = max(c.y, 0.0) + L + config.DESVIO_SEGURAR_EXTRA_CM
                elif abs(o_i) > abs(alvo_visivel) and (o_i * alvo_visivel >= 0):
                    alvo_visivel = o_i

        # cone sumiu da imagem: segura o desvio ate o carro passar
        alvo = alvo_visivel
        if agindo is None and self._mem_dist > 0 and self._mem_o != 0.0:
            self._mem_dist -= max(v_cms, 3.0) * dt
            alvo = self._mem_o
            self.fator_vel = min(self.fator_vel, 1.0 - 0.5 * config.VEL_REDUCAO_CONE)
            if self._mem_dist <= 0:
                self._mem_o = 0.0
                self.eventos.append("[desvio] passou do cone -> voltando pro centro da faixa")
        elif agindo is not None:
            pass
        else:
            self._mem_o = 0.0

        self.o_alvo = alvo
        # taxa limitada: afastando do centro e voltando tem velocidades diferentes
        vel = (config.DESVIO_SLEW_CMS if abs(alvo) > abs(self.o_cmd)
               else config.DESVIO_RETORNO_CMS)
        passo = vel * dt
        if abs(alvo - self.o_cmd) <= passo:
            self.o_cmd = alvo
        else:
            self.o_cmd += passo if alvo > self.o_cmd else -passo

        estado = "desviando" if abs(self.o_cmd) > 1.0 else "livre"
        if estado != self._estado:
            self._estado = estado
        # esquece lados de cones que sumiram ha tempo
        ids = {c.id for c in cones}
        for k in list(self._lados):
            if k not in ids and self._mem_dist <= 0:
                del self._lados[k]
        return self.o_cmd

    @property
    def estado(self):
        return self._estado
