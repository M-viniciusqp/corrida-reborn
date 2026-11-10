# reborn-autonomos

Carro autônomo (Raspberry Pi) que **segue a faixa** com **uma câmera**.
Faz a mesma tarefa do `gatuno-rapido`, mas com outra lógica, pensada para o carro novo
(câmera em outro ângulo). Não precisa ser rápido: processa em 320x180.

> **Modo atual: só corrida.** Cones estão DESLIGADOS (`CONES_ATIVOS = False` no `config.py`):
> não são detectados nem desviados. O código de cones fica guardado; `True` liga de novo.

| | gatuno-rapido | reborn-autonomos |
|---|---|---|
| Câmeras | 2 (pista + cone) | 1 |
| Faixa | varredura de fileiras da imagem | vista de cima (BEV) + votação de linhas + ajuste de curva |
| Medidas | "unidades" da imagem | **centímetros no chão** (modelo da câmera) |
| Direção | PID no erro | **Stanley** (erro lateral + proa) |
| Cone | para, analisa, desvia | desvio **contínuo** planejado em cm (sem parar) |

## Relação com o gatuno-rapido (mesmo objetivo, lógica nova)

Este projeto parte do `gatuno-rapido` (seguir a pista entre as faixas, o mais rápido possível, com
1 câmera) e mantém **o que ele faz**, mas refaz **como**:

| Função do gatuno-rapido | Aqui |
|---|---|
| Achar o centro da pista | BEV em **cm** + votação de linhas + curva (em vez de varredura de fileiras) |
| Direção | **Stanley** (erro lateral + proa), em vez de PID |
| Descobrir sozinho qual lado é a faixa **contínua** (por buracos da pontilhada) e travar | `LADO_CONTINUO = "auto"`: mede a **cobertura** das linhas ao ligar e, se a contínua estiver à esquerda, **espelha a imagem** (o resto do código não muda) |
| Andar mais perto da **pontilhada** que do centro (`LANE_BIAS_FRAC`) | `VIES_PONTILHADA_FRAC` (0,50), em cm |
| Velocidade máxima, freio em curva fechada, freio por FPS baixo | Reta no máximo; **freia antes da curva** (curvatura à frente), piso de potência, freio por FPS |
| Perdeu a pista: decai o volante e anda devagar, depois para | Igual, mas a parada é mais cedo e nunca abaixo do piso do motor |
| `teste_servo`, `test_control`, `test_vision` | `teste_servo.py`, `teste_motores.py`, `teste_reta.py`, `calibrar_camera.py`, `teste_autodirecao.py` e o **simulador** (`sim.py`) |

Novo em relação ao original: calibração automática (escala da câmera, sentido do volante, puxão do
carro, potência mínima), pista com a contínua dos dois lados e o simulador em malha fechada.

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

Está em `config.py` (seção 1). Servo alimentado pela bateria/BEC (5–6 V), **não** pelo 5 V do Pi. Cuidado com o aviso de baixa tensão: ele derruba câmera e servo.

## Passo a passo no Raspberry Pi

```bash
git pull                      # no Pi, na pasta do repositório (ou git clone na primeira vez)
sudo apt install python3-opencv python3-numpy python3-gpiozero   # se ainda não tiver
# opcional (servo sem tremor):  sudo apt install pigpio python3-pigpio && sudo systemctl enable --now pigpiod
python3 principal.py          # coloque o carro na faixa e rode: ele já larga e se calibra andando
```

Só isso. **Não há etapa de calibração antes de largar** (servo: ângulos fixos; câmera: geometria do `config.py`;
direção, puxão, largura da pista, lado da faixa contínua e luz: aprendidos/relativos andando).

Se quiser conferir antes (opcional):
1. **Fiação dos motores** (carro suspenso, rodas fora do chão): `python3 teste_motores.py`
2. **Servo**: `python3 teste_servo.py` (0° → 35° → 65°, como o seu teste)
3. **Câmera** (no terminal da área de trabalho do Pi): `python3 calibrar_camera.py`. Meça a **altura da lente** e ponha em
   `CAM_ALTURA_CM`; ajuste ao vivo (`w/s` inclinação, `r/f` altura, `t/g` campo de visão) até as linhas ficarem
   **retas e paralelas** na vista de cima; `p` grava `dados/camera.json`. (A câmera é a única medida física que importa.)
4. **Sem mover nada**: `python3 principal.py --seco --ver`

Opções do dia a dia: `--tempo 20` (para sozinho), `--vel 0.7`, `--maximo`, `--esquecer` (aprende do zero), `--ver`.
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

## Servo: ângulos exatos, sem calibrar

Direção usa os ângulos do seu teste (`config.py`): **0° esquerda · 35° reto · 65° direita**, com o mesmo modelo
de código (`gpiozero.AngularServo`, pino 18, 0–180°, pulso 0,5–2,5 ms). `python3 teste_servo.py` repete o seu
teste (0 → 35 → 65, 2 s em cada, `Ctrl+C` encerra); `--ajustar` só se um dia precisar mudar. Se a roda virar
para o lado trocado, não precisa mexer em nada: o carro percebe andando e inverte o volante.
Instalar o `gpiozero`: `sudo apt install python3-gpiozero` (e `pigpio` opcional, deixa o servo sem tremor).

## Larga na hora: o carro se calibra ENQUANTO ANDA

Como o `gatuno-rapido`, **não existe etapa de calibração antes de largar**: liga o carro e ele já segue a
faixa. O que o original aprende andando (largura da pista, lado da faixa contínua) ele continua aprendendo, e
este aprende também o que o original não precisa (`adaptacao.py`):

| O que aprende, andando | Como |
|---|---|
| **Sentido do volante** (invertido ou não) | mede o giro do carro pela câmera e compara com o comando; se o erro cresce enquanto o volante tenta reduzi-lo, ou o ganho medido é negativo, **inverte sozinho** |
| **Puxão do carro** para um lado (centro do servo, rodas, motores) | nas retas, o volante médio que o controle precisa manter vira o `trim` |
| **Largura da faixa** | distância perpendicular entre as linhas (em `faixas.py`) |
| **Lado da faixa contínua** | compara a cobertura das linhas (a pontilhada tem buracos); se estiver do outro lado, **espelha a imagem**; depois de concordar por um tempo, **trava** a decisão |
| **Potência mínima para andar** | se manda andar com o volante virado e a cena não muda, sobe o piso (`motor_auto.py`) |

Até o sentido do volante ser **confirmado** (alguns segundos), ele anda devagar (`LARGADA_VEL_MAX`, o piso do
motor) e sem o desvio para a pontilhada. Depois, velocidade total. O aprendido fica em `dados/auto.json` e é
usado já na largada seguinte (`--esquecer` ignora e aprende do zero). Se o servo for recalibrado, a parte de
direção do arquivo é descartada sozinha.

Opcional (não é necessário): `--autocal` (escala da câmera antes de largar), `--recalibrar` (calibração da
direção andando uns 2,5 m antes de largar), `calibrar_camera.py` (ajuste manual da câmera).

Uma vez só (fica salvo em `dados/`):
- inclinação/guinada/rolagem da câmera → `calibrar_camera.py` (`p` grava `dados/camera.json`)
- servo: **nada** (ângulos fixos 0°/35°/65° no `config.py`; `teste_servo.py` só para conferir as rodas)

Testar sem o carro: `python teste_online.py` (carros simulados com volante invertido, puxando, direção
fraca/forte, faixa de largura diferente e pista espelhada, **todos largando sem calibrar**).

## Qualquer luminosidade e proteções

A visão **não usa nenhum limite fixo de luz**: o contraste mínimo é medido no próprio quadro (ruído da imagem), o
brilho da linha é comparado ao brilho das linhas candidatas daquele instante e a saturação é relativa (luz amarela ou
azulada). `python teste_luz.py` simula escuro, claro, estourado, sombra em degradê, luz amarela/azulada, ruído forte,
desfoque e reflexo (39 corridas simuladas). A largura da pista é aprendida andando; nada a medir.

| Proteção | Onde |
|---|---|
| freio automático quando o FPS cai (`FPS_MINIMO`) | `controle.py` |
| erro e proa suavizados (`ERRO_SUAV`) e comando de volante filtrado | `piloto.py`, `controle.py` |
| sem derivada ruidosa: a proa vem de uma janela da faixa (inclinação média), não de diferença quadro a quadro | `piloto.py` |
| erro **saturado** (colado no máximo por 5 quadros) = rastreio furado: reinicia e conta como faixa perdida | `piloto.py` |
| **parada total** depois de 12 quadros sem faixa (`PERDIDO_QUADROS_PARAR`) | `piloto.py` |
| o controle só roda com leitura **fresca** (sem faixa, só decai o volante) | `piloto.py` |
| câmera parou de entregar quadros: motores param e ela reabre sozinha | `camera.py`, `principal.py` |

## Velocidade

- **Reta:** `VELOCIDADE_BASE = 1.00` (100%).
- **Curvas:** o carro freia pouco, só até `VELOCIDADE_MIN = 0.85` (piso de cruzeiro). Freios:
  `VEL_REDUCAO_CURVATURA` (curva à frente), `VEL_REDUCAO_CURVA` (volante virado) e `VEL_REDUCAO_INCERTO`
  (carro torto/longe do centro). Para frear **ainda menos**: reduza esses três ou suba `VELOCIDADE_MIN`.
- **Piso do motor:** `PISO_MOTOR_MIN = 0.55`: nunca manda menos que isso (nem ao perder a faixa nem por FPS
  baixo); o carro ainda aprende sozinho se precisa de mais (`dados/motor.json`).
- `python3 principal.py --maximo`: 100% o tempo todo, sem nenhuma freada (só nessa corrida).
- `python3 principal.py --vel 0.6`: velocidade base mais baixa só nessa corrida.
