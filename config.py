"""
config.py - TODOS os numeros ajustaveis do reborn-autonomos num lugar so.

Este projeto faz a MESMA tarefa do gatuno-rapido (seguir a faixa entre a
pontilhada e a continua e desviar de cones), mas com outra abordagem:

  gatuno-rapido            ->  reborn-autonomos
  ---------------------------------------------------------------------
  2 cameras                ->  1 camera
  varredura por linhas     ->  vista de cima (BEV) + ajuste de curva
  erro em "unidades"       ->  tudo em CENTIMETROS no chao
  PID no erro              ->  controle Stanley (cross-track + proa)
  para-analisa-desvia      ->  desvio continuo planejado em cm (sem parar)

O que depende do carro/camera fica na secao 2 (GEOMETRIA DA CAMERA). Como a
camera deste carro esta em outro angulo, rode  `python3 calibrar_camera.py`
e ajuste ate a vista de cima mostrar as faixas paralelas e retas.
"""

# =====================================================================
# 1. HARDWARE (pinos BCM) - conforme a tabela "Pinagem sugerida"
# =====================================================================
SERVO_PIN = 18          # fio de sinal do servo (pino fisico 12)
ENA = 12                # PWM motor A  (pino fisico 32)  - remover jumper da L298N
IN1 = 23                # pino fisico 16
IN2 = 24                # pino fisico 18
ENB = 13                # PWM motor B  (pino fisico 33)  - remover jumper da L298N
IN3 = 25                # pino fisico 22
IN4 = 16                # pino fisico 36
# GND da L298N + GND do servo/bateria + GND do Pi: terra comum (pino 6/9/14...)

MOTOR_PWM_HZ = 100
MOTOR_A_INVERTIDO = False   # se um motor girar pro lado errado, troque aqui
MOTOR_B_INVERTIDO = False   # (em vez de trocar fios)

# =====================================================================
# 2. SERVO (graus) - valores de PARTIDA. Os valores reais saem do
#    `python3 teste_servo.py`, que grava dados/servo.json e o programa
#    passa a usar esse arquivo automaticamente (ele manda sobre isto aqui).
#    Os 3 numeros sao independentes: nao importa se esquerda < centro < direita
#    ou o contrario (servo montado invertido).
# =====================================================================
SERVO_CENTRO_PADRAO   = 90.0
SERVO_ESQ_PADRAO      = 60.0
SERVO_DIR_PADRAO      = 120.0
SERVO_LIMITE_ABS      = (5.0, 175.0)    # NUNCA manda angulo fora disso
SERVO_PULSO_US        = (500, 2500)     # largura de pulso em 0 e 180 graus
SERVO_DEADBAND_DEG    = 0.7             # so reescreve o pulso se mudou mais que isso

# =====================================================================
# 3. CAMERA
# =====================================================================
CAMERA_ID   = 0
CAMERA_RES  = (320, 240)        # largura, altura. Nao precisa ser rapido.
CAMERA_FPS  = 30
CAMERA_ESPELHAR = False         # True se a imagem sair de cabeca pra baixo/espelhada
CAMERA_GIRAR_180 = False

# =====================================================================
# 4. GEOMETRIA DA CAMERA (a parte que muda de carro pra carro)
# Origem do chao: ponto no CHAO embaixo do EIXO DIANTEIRO, no meio do carro.
#   X = pra direita (cm)   Y = pra frente (cm)
# Meça com uma trena e depois refine com calibrar_camera.py (que grava
# dados/camera.json e passa a mandar sobre estes valores).
# =====================================================================
CAM_ALTURA_CM   = 22.0      # altura da lente em relacao ao chao
CAM_INCLINACAO  = 30.0     # graus pra BAIXO em relacao a horizontal (0 = olha reto)
CAM_GUINADA     = 0.0       # graus pra esquerda(+) / direita(-) em relacao a frente
CAM_ROLAGEM     = 0.0       # graus de giro da imagem (horizonte torto)
CAM_X_CM        = 0.0       # deslocamento lateral da camera do eixo do carro (+ direita)
CAM_Y_CM        = 0.0       # camera a frente(+) / atras(-) do eixo dianteiro
CAM_FOV_H       = 70.0      # campo de visao HORIZONTAL em graus (C920 ~70, C925e ~78)

# ---- area do chao que vira vista de cima (BEV) ----
BEV_X_MAX_CM   = 40.0       # enxerga de -40 a +40 cm do eixo do carro
BEV_Y_MIN_CM   = None       # None = automatico: borda de baixo da imagem
BEV_Y_MAX_CM   = None       # None = automatico: ate BEV_TOPO_PCT da imagem
BEV_TOPO_PCT   = 0.20       # (automatico) ignora o topo da imagem: horizonte/parede
BEV_PX_POR_CM  = 3.0        # resolucao da vista de cima

# =====================================================================
# 5. A PISTA (MEDIR). Duas faixas lado a lado: continua | pontilhada | continua.
#    O carro anda na faixa A: pontilhada a ESQUERDA, continua a DIREITA.
# =====================================================================
FAIXA_LARGURA_CM  = 29.0    # de linha a linha (centro a centro) de UMA faixa
LINHA_LARGURA_CM  = 3.0     # largura da tinta/fita branca
CARRO_LARGURA_CM  = 20.0    # largura do carro (rodas externas)
CARRO_COMPRIMENTO_CM = 28.0 # do eixo dianteiro ate a traseira (p/ saber quando passou do cone)
CONE_LARGURA_CM   = 18.0    # largura da BASE do cone

# =====================================================================
# 6. VISAO DAS FAIXAS
# Realce de linha por "diferenca de gaussianas" (centro - entorno) em vez de
# limiar simples: ignora variacao lenta de luz e acha tiras claras estreitas.
# =====================================================================
DOG_MIN             = 14      # contraste minimo (0-255) pra ser linha
DOG_PERCENTIL       = 99.0    # o limiar sobe se a cena inteira estiver brilhante
DOG_FATOR           = 0.40
BRILHO_MIN          = 90      # a linha tem que ser mais clara que isso (0-255)
SAT_MAX             = 110     # branco tem pouca saturacao (tira cones/ladrilho colorido)

# busca por janelas deslizantes na BEV
JANELAS_N           = 9
JANELA_MARGEM_CM    = 7.0
JANELA_MIN_PX       = 14
LINHA_MIN_PTS       = 45      # pixels minimos pra aceitar uma linha
LINHA_MIN_EXTENSAO_CM = 6.0   # comprimento minimo (cm) de tinta alinhada (um tracejado)
LINHA_RESIDUO_MAX_CM = 3.0    # erro medio do ajuste acima disso = rejeita
CURVA_GRAU2_EXTENSAO = 0.6    # so ajusta parabola se a linha cobrir isso da BEV

# continua x pontilhada: fracao dos trechos (janelas) que tem tinta
COBERTURA_CONTINUA_MIN  = 0.80
COBERTURA_PONTILHADA_MAX = 0.72
COBERTURA_SUAVIZACAO    = 0.6     # EMA da cobertura entre quadros (1 = so o quadro atual)

# onde olhar adiante pra decidir a direcao (distancia a frente do eixo, cm)
OLHAR_ADIANTE_CM    = 38.0

# =====================================================================
# 7. CONTROLE (Stanley)
#   volante = erro_de_proa + atan(K * erro_lateral / (V0 + v))
# =====================================================================
STANLEY_K           = 2.2
STANLEY_V0_CMS      = 12.0    # evita divisao por ~0 em baixa velocidade
V_CMS_NA_VEL_1      = 80.0    # velocidade estimada (cm/s) com speed=1.0 (so p/ o controle)
ESTERCO_MAX_GRAUS   = 28.0    # esterco REAL maximo da roda (mede no carro)
PROA_PESO           = 1.0     # peso do termo de proa
ERRO_ZONA_MORTA_CM  = 0.6
ESTERCO_SUAVIZACAO  = 0.55    # EMA do comando de volante (1 = sem filtro)
ESTERCO_SLEW        = 6.0     # maximo de variacao do turn (-1..1) por segundo
ESTERCO_INVERTER    = False   # True se o carro esterca pro lado errado

# =====================================================================
# 8. VELOCIDADE (0..1)
# =====================================================================
VELOCIDADE_BASE     = 0.38
VELOCIDADE_MIN      = 0.26    # abaixo disso o motor nem vence o atrito
VELOCIDADE_PERDIDO  = 0.22
VEL_REDUCAO_CURVA   = 0.45    # tira ate isso da velocidade com o volante todo
VEL_REDUCAO_CONE    = 0.25    # tira ate isso com cone bem perto
FPS_MINIMO          = 6.0     # abaixo disso o carro reduz a velocidade
QUADRO_VELHO_S      = 0.5     # quadro mais velho que isso = para

# perdeu as faixas
PERDIDO_QUADROS_DEVAGAR = 10  # anda devagar segurando o volante
PERDIDO_QUADROS_PARAR   = 45  # depois disso para

# =====================================================================
# 9. CONES
# =====================================================================
# cor (HSV do OpenCV, H 0..179). Vermelho/laranja "dobra": H perto de 0 e de 179.
CONE_H_BAIXO     = 14
CONE_H_ALTO      = 165
CONE_S_MIN       = 110
CONE_V_MIN       = 70
CONE_AREA_MIN_PX = 60         # em 320x240 (escala sozinho com a resolucao)
CONE_ASPECTO_MIN = 0.7        # altura/largura do retangulo
CONE_ASPECTO_MAX = 3.5
CONE_PREENCHIMENTO_MIN = 0.35 # area / area do retangulo (triangulo ~0.5)
CONE_TOPO_PCT    = 0.05       # ignora o topo da imagem
CONE_BASE_OFFSET_CM = 4.0     # centro do cone fica isso alem da borda da frente da base
CONE_LARG_TOL    = (0.45, 2.2)  # largura medida no chao / CONE_LARGURA_CM: fora disso = falso
CONE_CONFIRMA_QUADROS = 2     # quadros seguidos no mesmo lugar pra confirmar
CONE_CONFIRMA_DIST_CM = 15.0
CONE_SUAVIZACAO  = 0.6        # EMA da posicao (1 = so o quadro atual)
CONE_PERDE_QUADROS = 3        # quadros sem ver = cone sumiu da imagem

# =====================================================================
# 10. PLANEJADOR DE DESVIO
# Regras (mesmas do gatuno): cone na faixa do carro -> cruza a pontilhada;
# cone na outra faixa -> ignora; a continua nunca e cruzada.
# =====================================================================
CONE_MARGEM_CM        = 9.0   # folga entre a lateral do carro e o cone
MARGEM_CONTINUA_CM    = 3.0   # a lateral do carro nao chega mais perto que isso da continua
CONE_TOL_FAIXA_CM     = 4.0   # cone ate isso alem da pontilhada ainda conta como "minha faixa"
DESVIO_INICIO_CM      = 130.0  # comeca a desviar com o cone a esta distancia a frente
DESVIO_COMPLETO_CM    = 75.0  # desvio todo aplicado nesta distancia
DESVIO_SLEW_CMS       = 55.0  # variacao maxima do alvo lateral por segundo (suaviza)
DESVIO_RETORNO_CMS    = 30.0  # idem, voltando pro centro da faixa
DESVIO_SEGURAR_EXTRA_CM = 10.0  # distancia extra segurando o desvio depois do cone passar
CONE_FRENAGEM_ALCANCE_CM = 60.0

# =====================================================================
# 11. DEPURACAO
# =====================================================================
DEBUG_CADA = 2            # --ver: desenha a cada N quadros
LOG_PASTA = "logs"
