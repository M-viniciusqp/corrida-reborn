# reborn-autonomos

Carro autônomo (Raspberry Pi) que **segue a faixa e desvia de cones** com **uma câmera**.
Faz a mesma tarefa do `gatuno-rapido`, mas com outra lógica, pensada para o carro novo
(câmera em outro ângulo). Não precisa ser rápido: roda em 320x240.

| | gatuno-rapido | reborn-autonomos |
|---|---|---|
| Câmeras | 2 (pista + cone) | 1 |
| Faixa | varredura de fileiras da imagem | vista de cima (BEV) + votação de linhas + ajuste de curva |
| Medidas | "unidades" da imagem | **centímetros no chão** (modelo da câmera) |
| Direção | PID no erro | **Stanley** (erro lateral + proa) |
| Cone | para, analisa, desvia | desvio **contínuo** planejado em cm (sem parar) |

## Pinagem (BCM) — confira antes de ligar

| Função | GPIO | Pino físico | Liga em |
|---|---|---|---|
| Servo (sinal) | 18 | 12 | fio de sinal do servo |
| ENA (PWM motor A) | 12 | 32 | L298N ENA (**remover o jumper**) |
| IN1 | 23 | 16 | L298N IN1 |
| IN2 | 24 | 18 | L298N IN2 |
| ENB (PWM motor B) | 13 | 33 | L298N ENB (**remover o jumper**) |
| IN3 | 25 | 22 | L298N IN3 |
| IN4 | 16 | 36 | L298N IN4 |
| GND | — | 6/9/14… | GND da L298N **e** GND do servo/bateria (terra comum) |

Está em `config.py` (seção 1). Servo alimentado pela bateria/BEC (5–6 V), **não** pelo 5 V do Pi.

## Passo a passo no Raspberry Pi

```bash
git pull                      # no Pi, na pasta do repositório
cd reborn-autonomos
sudo apt install python3-opencv python3-numpy   # se ainda não tiver
# opcional (servo sem tremor):  sudo apt install pigpio python3-pigpio && sudo systemctl enable --now pigpiod
```

1. **Fiação dos motores** (carro suspenso, rodas fora do chão):
   `python3 teste_motores.py`
2. **Servo — descobrir centro, esquerda e direita:**
   `python3 teste_servo.py --varrer`
   Vá com `a`/`d` (±1°) e `A`/`D` (±5°), marque `c` (roda reta), `l` (esquerda total),
   `r` (direita total), `s` salva em `dados/servo.json`. Pare **antes** da roda bater no batente.
3. **Câmera — calibrar a geometria do ângulo atual** (no terminal da área de trabalho do Pi):
   `python3 calibrar_camera.py`
   Meça com trena a **altura da lente** e a **largura da faixa** e ponha em `config.py`
   (`CAM_ALTURA_CM`, `FAIXA_LARGURA_CM`...). Depois ajuste ao vivo (`w/s` inclinação, `r/f` altura,
   `t/g` campo de visão) até as linhas aparecerem **retas e paralelas** na vista de cima e a
   largura medida bater com a trena. `p` grava `dados/camera.json`.
4. **Primeiro teste sem mover nada:**
   `python3 principal.py --seco --ver`
5. **Andar 10 s:** `python3 principal.py --tempo 10` (o carro para sozinho; `Ctrl+C` também para).

Mande a pasta `logs/<data>/` (eventos, telemetria e fotos) quando precisar de ajuda.

## Como funciona

```
camera -> piloto.py ──> faixas.py     centro da faixa A (curva X(Y), cm)
                   ├──> cones.py      cones em cm (base do cone -> chão)
                   ├──> planejador.py desloc. lateral para desviar
                   └──> controle.py   Stanley -> turn, velocidade
principal.py = só entrada/saída (câmera, servo, motores).   sim.py roda o MESMO piloto.
```

* **Geometria** (`geometria.py`): altura, inclinação, campo de visão (+ guinada, rolagem,
  deslocamento) → qualquer pixel abaixo do horizonte vira (X, Y) em cm no chão, origem sob o
  eixo dianteiro. Por isso mudar o ângulo da câmera é só mudar números.
* **Faixas** (`faixas.py`): vista de cima → realce de linha por diferença de gaussianas →
  votação tipo Hough em coordenadas do chão → reta/parábola por linha. *Contínua × pontilhada* =
  fração de trechos da linha com tinta. Pista `contínua | pontilhada | contínua`: cada linha
  visível dá uma estimativa do centro da faixa do carro (`+1.5L`, `+0.5L` ou `−0.5L`), então
  funciona mesmo quando o carro cruza a pontilhada.
* **Cones** (`cones.py`): cor HSV → base do cone → posição em cm; confere a largura do cone no
  chão e exige 2 quadros seguidos.
* **Desvio** (`planejador.py`): cone na faixa do carro → passa pelo lado que cabe (cruza a
  pontilhada; a contínua nunca); cone na outra faixa/fora do caminho → ignora. O desvio é
  segurado até o carro inteiro passar (o cone sai da imagem antes) e depois volta ao centro.
* **Controle** (`controle.py`): `volante = proa + atan(K·erro/(V0+v))`, filtro e limite de taxa.

## O que ajustar (tudo em `config.py`)

* Carro/pista (medir): `CARRO_LARGURA_CM`, `FAIXA_LARGURA_CM`, `CONE_LARGURA_CM`, `LINHA_LARGURA_CM`.
* Direção: `ESTERCO_MAX_GRAUS` (esterco real da roda), `ESTERCO_INVERTER` se virar pro lado errado.
* Velocidade: `VELOCIDADE_BASE` (começa baixa, 0.38; suba de 0.05 em 0.05).
* Cone não detectado / falso positivo: `CONE_H_*`, `CONE_S_MIN`, `CONE_V_MIN` (HSV da cor do cone).
* Linhas não detectadas: `DOG_MIN`, `BRILHO_MIN`, `SAT_MAX`.
* Raspou no cone: suba `CONE_MARGEM_CM`. Desvia cedo/tarde demais: `DESVIO_INICIO_CM`/`DESVIO_COMPLETO_CM`.

## Testar sem o carro

```bash
python sim.py                 # 10 cenários (retas, curvas, cones) em malha fechada
python sim.py cone_centro -v  # um cenário, com o log de decisões
python sim.py --erro-cam      # câmera "real" diferente da medida (teste de robustez)
python sim.py cone_centro --ver
```

O simulador usa física simples e câmera ideal: serve para achar erro de lógica e de sinal
(esquerda/direita), **não** para afinar números. A validação real é na pista.
