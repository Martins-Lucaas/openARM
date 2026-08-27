# openARM

Controle do braço robótico **WidowX Mark II** (Interbotix) via ROS 2.

- `ros2_ws/` — workspace ROS 2 Humble com o pacote
  [`widowx_control`](ros2_ws/src/widowx_control/README.md): driver serial
  (protocolo ArmLink/ArbotiX-M) + GUI de controle manual.
- `legacy/` — material anterior do projeto (códigos Arduino/Vespa, esquemático,
  manuais de montagem, app Flutter `motor_control` e simulação Webots
  `ws_fruit_sorting`).

## Início rápido

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch widowx_control widowx_control.launch.py
```
