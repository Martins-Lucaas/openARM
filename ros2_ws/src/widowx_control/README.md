# widowx_control

Pacote ROS 2 (Humble) para controle manual do braço **WidowX Mark II**
(Interbotix/Trossen) através da placa **ArbotiX-M** com o firmware de fábrica
*InterbotixArmLinkSerial* (protocolo ArmLink, 115200 baud).

Não é preciso reflashear a placa: o driver fala o protocolo ArmLink em modo
*backhoe* (controle junta a junta) direto pela serial USB (cabo FTDI).

## Uso

```bash
cd ~/openARM/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch widowx_control widowx_control.launch.py port:=/dev/ttyUSB0
```

Argumentos: `port` (ArbotiX), `home_on_start` (ida automática à home ao
ligar, padrão `true`), `touch_port` (touch sensor STM32; vazio =
auto-detect), `sensor` (`4` ou `5`, grade do touch sensor) e `poses_file`
(JSON do teach pendant; vazio = `~/.widowx_poses.json`).

A GUI é inspirada no Dynamixel Wizard: barra de ferramentas no topo, lista
de servos à esquerda (estado em tempo real: verde OK, vermelho alerta) e
cinco abas:

- **Controle manual** — modelo 2D do braço (destaca em laranja o motor em
  uso) + sliders das 6 juntas + duração do movimento.
- **Servo** — detalhes do servo selecionado na lista: posição, alvo,
  tensão, temperatura, carga, torque, limite de torque e avisos, com um
  slider individual.
- **Diagnóstico** — tabela de todos os servos, atualizada continuamente
  (o driver varre um servo por vez nos intervalos entre movimentos).
- **Poses** — teach pendant: grava poses do braço e as encadeia em
  movimentos, com o tempo de permanência de CADA passo (ver abaixo).
- **Sensores** — os gráficos do touch sensor STM32 (heatmap, raster RA/SA,
  I_final e cuneiformes), ocupando a aba inteira.

## Aba Poses (teach pendant)

Portada do `touch_pack` (repositório cr10twin) e adaptada ao WidowX.

**Gravar uma pose:** solte o torque (**Dormir**), ponha o braço na posição
com a mão e clique em **◉ Capturar do braço** — com o torque solto o driver
publica a posição real e os sliders a acompanham. Com o braço ativo,
**⌨ Dos sliders** grava a posição dos sliders.

**Montar um movimento:** **+ Novo**, depois duplo clique nas poses (ou
**+ Adicionar**) para empilhá-las na sequência; **↑ ↓** reordenam e **−**
remove. Cada passo tem o seu **tempo de permanência** — quanto o braço fica
parado ali antes de ir para o próximo — editável no campo à direita, com
**→ todos** para copiar o valor para a sequência inteira. O **percurso**
(quanto leva para ir de uma pose à seguinte) é do movimento e vira o `dtime`
do ArbotiX. O total da sequência aparece embaixo e na lista de movimentos.

**Executar:** **▶ Executar** (uma passagem) ou **↻ Em laço**; **■ Parar**
interrompe no passo em curso. Durante a execução os sliders e o desenho 2D
acompanham a pose atual e o passo em curso fica destacado na sequência. Só
roda com o braço **ativo** (modo backhoe).

Atalhos: `F2` renomeia · `Del` exclui · `Ctrl+N` captura · `Ctrl+D` duplica ·
`Ctrl+Enter` vai para a pose · `F5` executa · `Esc` para · `Alt+↑/↓`
reordena o passo.

Tudo é gravado na hora em `~/.widowx_poses.json` (mude com o argumento
`poses_file` do launch). As poses guardam **unidades brutas de servo**, já
saneadas pelos limites do `armlink` — um arquivo editado à mão com valores
fora de faixa é corrigido na leitura, não na hora de mover.

## Aba Sensores

Porte da aba homônima do `touch_pack`: o touch sensor STM32 chega por USB
(CDC nativa, auto-detectada; force com o argumento `touch_port`) e a grade
vem do argumento `sensor` (`4` = 4×4 com neurônio pós, `5` = 5×5 com os
cuneiformes).

Os quatro gráficos ocupam a aba inteira, e o botão **⧉ Destacar** os manda
para uma **janela própria** — é assim que se olha o toque e o teach pendant
ao mesmo tempo (um notebook mostra uma aba de cada vez, e o dado tátil só é
útil junto com a pose que o produziu). Destacada, a figura anima
independentemente da aba que estiver à frente; fechar a janela (ou
**⤢ Reacoplar**) traz os gráficos de volta. A leitura serial não é
interrompida na troca. **Não há painel de célula de
carga**: o WidowX não tem célula, e o card de 270 px que ela ocupa no
cr10twin foi devolvido aos gráficos. O escalar I_final e a origem do dado
(porta serial, `/touch_sensor/value` ou "sem sensor") ficam na faixa do
cabeçalho.

O desenho da figura se autorregula: o custo do frame é medido e o intervalo
da animação é ajustado para o desenho nunca passar de ~40% da thread do Tk
(a figura escala com o tamanho da janela, e a taxa fixa de 30 fps do
original saturava o laço e travava a GUI inteira).

## Inicialização: direto para a home pose

Ao conectar, o driver entra em modo backhoe e leva o braço **direto para a
home pose** (o neutro do usuário, se salvo em `~/.widowx_neutral.json`;
senão o neutro de fábrica), num percurso lento de 2 s. Enquanto essa ida
acontece:

- alvos publicados em `/widowx/joint_targets` são **descartados**;
- `activate`, `home`, `sleep`, `set_neutral` e `rearm` respondem
  *"inicializacao em andamento"*;
- **`estop` passa** — bloquear a parada durante o boot trocaria uma pose
  indesejada por um braço que não para. O estop também cancela a ida.

A GUI acompanha o estado do driver: quando ele termina, os sliders são
habilitados já alinhados com a home, sem ninguém precisar clicar em Ativar
(clicar reentraria em backhoe e levaria o braço ao neutro de fábrica — a
pose intermediária que isto existe para evitar).

Desligue com `home_on_start:=false` no launch. **Ressalva honesta:** a
entrada em modo backhoe é do firmware ArmLink, e ele leva o braço ao neutro
de fábrica antes de aceitar qualquer alvo. Se a sua home for diferente do
neutro de fábrica, o braço passa por ele — não há como pedir backhoe sem
esse movimento. O que o driver garante é que **nada além disso** aconteça
até a home ser alcançada.

Botões: **Ativar** (modo backhoe; o firmware move o braço ao neutro de
fábrica), **Posição neutra** (vai ao neutro do usuário, se salvo),
**Definir neutro aqui** (salva a pose atual em `~/.widowx_neutral.json`),
**Dormir** (recolhe e solta torque), **Rearmar proteção** (restaura o
torque de servos que entraram em shutdown por sobrecarga) e **PARADA**.

Enquanto o braço está com o torque solto, os sliders acompanham a posição
física real (você pode mover o braço com a mão e ver os valores).

## Interface ROS

| Nome | Tipo | Sentido | Descrição |
|---|---|---|---|
| `/widowx/joint_targets` | `sensor_msgs/JointState` | entrada | alvos em unidades brutas de servo |
| `/widowx/dtime` | `std_msgs/Int32` | entrada | interpolação, unidades de 16 ms (1–255) |
| `/widowx/joint_states` | `sensor_msgs/JointState` | saída | posição real em radianos (0 = neutro) |
| `/widowx/status` | `std_msgs/String` | saída | estado da conexão |
| `/widowx/diag` | `std_msgs/String` (JSON) | saída | tensão/temp/carga/torque/limite por servo, contínuo |
| `/widowx/neutral` | `sensor_msgs/JointState` (latched) | saída | posição neutra vigente (unidades brutas) |
| `/touch_sensor/value` | `std_msgs/Float32` | entrada | escalar do toque, quando não há serial local |
| `/widowx/{activate,home,sleep,estop}` | `std_srvs/Trigger` | serviço | modo backhoe / neutro / dormir / parada |
| `/widowx/diagnostics` | `std_srvs/Trigger` | serviço | último diagnóstico em texto |
| `/widowx/set_neutral` | `std_srvs/Trigger` | serviço | salva a pose atual como neutro |
| `/widowx/rearm` | `std_srvs/Trigger` | serviço | restaura torque de servos em proteção |

Todo o tráfego serial passa por uma thread dedicada no driver: comandos de
movimento têm prioridade absoluta e as leituras de posição/diagnóstico
preenchem o tempo ocioso — o firmware do ArbotiX descarta pacotes que
chegam durante uma interpolação, então o driver também espaça os envios
pelo tempo de interpolação vigente.

Juntas: `base` (0–4095), `shoulder`, `elbow`, `wrist_angle` (1024–3072),
`wrist_rotate` (0–1023), `gripper` (0–512). Neutro: 2048/2048/2048/2048/512/256.

## Notas de hardware

- Abrir a porta serial **reinicia o ArbotiX** (pulso DTR); o firmware recolhe
  o braço e solta o torque na inicialização.
- O Dynamixel Wizard **não** funciona através do ArbotiX — para configurar os
  servos diretamente seria necessário um U2D2 ligado ao barramento TTL.
