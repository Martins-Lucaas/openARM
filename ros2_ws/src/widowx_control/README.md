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

A GUI é inspirada no Dynamixel Wizard: barra de ferramentas no topo, lista
de servos à esquerda (estado em tempo real: verde OK, vermelho alerta) e
três abas:

- **Controle manual** — modelo 2D do braço (destaca em laranja o motor em
  uso) + sliders das 6 juntas + duração do movimento.
- **Servo** — detalhes do servo selecionado na lista: posição, alvo,
  tensão, temperatura, carga, torque, limite de torque e avisos, com um
  slider individual.
- **Diagnóstico** — tabela de todos os servos, atualizada continuamente
  (o driver varre um servo por vez nos intervalos entre movimentos).

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
