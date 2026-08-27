"""No driver do WidowX: dono da porta serial do ArbotiX-M.

Topicos:
  /widowx/joint_targets (sensor_msgs/JointState, entrada) - posicoes-alvo em
      UNIDADES BRUTAS de servo (base 0-4095, ombro/cotovelo/punho 1024-3072,
      rotacao do punho 0-1023, garra 0-512).
  /widowx/dtime (std_msgs/Int32, entrada) - tempo de interpolacao em
      unidades de 16 ms (1-255).
  /widowx/joint_states (sensor_msgs/JointState, saida) - posicao real das
      juntas em radianos (0 = neutro).
  /widowx/status (std_msgs/String, saida) - estado da conexao.

Saidas adicionais:
  /widowx/diag (std_msgs/String, JSON) - tensao/temperatura/carga/torque por
      servo, atualizado a cada 5 s quando o braco esta parado.
  /widowx/neutral (sensor_msgs/JointState, transient_local) - posicao neutra
      do usuario em unidades brutas.

Inicializacao (parametro home_on_start, ligado por padrao):
  ao conectar, o driver entra em modo backhoe e leva o braco DIRETO para a
  home pose (o neutro do usuario, se salvo). Durante essa ida, alvos e
  servicos de movimento sao RECUSADOS - so a parada de emergencia passa.

Servicos (std_srvs/Trigger):
  /widowx/activate    - entra no modo backhoe (o braco VAI AO NEUTRO, ~2 s)
  /widowx/home        - move para a posicao neutra (a do usuario, se salva)
  /widowx/sleep       - recolhe o braco e solta o torque
  /widowx/estop       - parada de emergencia
  /widowx/diagnostics - le e publica o diagnostico agora
  /widowx/set_neutral - salva a pose atual como nova posicao neutra
"""

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from . import armlink
from .armlink import ArmLink, ArmLinkError

# Percurso da ida a home na inicializacao, em unidades de 16 ms do ArmLink.
# 125 = 2 s: o braco parte de onde o firmware o deixou ao reiniciar, e essa
# primeira ida e a que menos se quer rapida.
_BOOT_DTIME = 125


class WidowXDriver(Node):

    def __init__(self):
        super().__init__("widowx_driver")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", armlink.DEFAULT_BAUD)
        self.declare_parameter("send_rate", 25.0)
        self.declare_parameter("neutral_file",
                               str(Path.home() / ".widowx_neutral.json"))
        # Ao conectar, vai direto para a home pose e recusa tudo mais ate
        # chegar (ver _boot_to_home).
        self.declare_parameter("home_on_start", True)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.arm = ArmLink(port, baud)
        self.active = False
        # Enquanto True, NADA move o braco a nao ser a propria rotina de
        # inicializacao (ver _boot_to_home). A parada de emergencia e a
        # unica excecao - ela e o que interrompe um boot que deu errado.
        self.booting = False
        self._boot_abort = False
        self.targets = dict(armlink.NEUTRAL)
        self.dtime = 15  # 15*16 ms = 240 ms de interpolacao
        self.dirty = False
        self.last_move_time = 0.0

        self.neutral_file = Path(self.get_parameter("neutral_file").value)
        self.user_neutral = self._load_neutral()

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_states = self.create_publisher(JointState,
                                                "/widowx/joint_states", 10)
        self.pub_status = self.create_publisher(String, "/widowx/status", 10)
        self.pub_diag = self.create_publisher(String, "/widowx/diag", 10)
        self.pub_neutral = self.create_publisher(JointState,
                                                 "/widowx/neutral", latched)
        self._publish_neutral()
        self.create_subscription(JointState, "/widowx/joint_targets",
                                 self.on_targets, 10)
        self.create_subscription(Int32, "/widowx/dtime", self.on_dtime, 10)

        self.create_service(Trigger, "/widowx/activate", self.on_activate)
        self.create_service(Trigger, "/widowx/home", self.on_home)
        self.create_service(Trigger, "/widowx/sleep", self.on_sleep)
        self.create_service(Trigger, "/widowx/estop", self.on_estop)
        self.create_service(Trigger, "/widowx/diagnostics",
                            self.on_diagnostics)
        self.create_service(Trigger, "/widowx/set_neutral",
                            self.on_set_neutral)
        self.create_service(Trigger, "/widowx/rearm", self.on_rearm)

        self.pos_cache = {}
        self.diag_cache = {}
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"Conectando ao ArbotiX em {port} ({baud} baud)... "
            "a placa reinicia e o braco solta o torque.")
        try:
            self.arm.connect()
            name = armlink.ARM_NAMES.get(self.arm.arm_id, "desconhecido")
            self.get_logger().info(f"Braco conectado: {name} "
                                   f"(ARMID={self.arm.arm_id})")
        except (ArmLinkError, OSError) as exc:
            self.get_logger().error(f"Falha ao conectar: {exc}")

        if self.arm.connected and self.get_parameter("home_on_start").value:
            # Thread propria: a ida leva ~4 s (backhoe + percurso) e o
            # construtor do no nao pode bloquear tudo isso.
            threading.Thread(target=self._boot_to_home, daemon=True,
                             name="widowx-boot").start()

        # Todo o trafego serial sai desta thread: movimento tem prioridade
        # absoluta; leituras de posicao/diagnostico preenchem o tempo ocioso.
        self._running = True
        self._worker = threading.Thread(target=self._serial_worker,
                                        daemon=True)
        self._worker.start()

    def stop(self):
        self._running = False

    # -------------------------------------------------------- inicializacao
    def _boot_to_home(self):
        """Energizou, conectou: vai para a home pose e SO para ela.

        Abrir a serial ja reinicia o ArbotiX (pulso de DTR) e o firmware
        recolhe o braco e solta o torque. A partir dai o braco fica onde
        parou ate alguem mandar algo - e "alguem" podia ser um alvo antigo
        no topico, um slider da GUI ou o proprio operador clicando Ativar,
        cada um levando o braco a uma pose diferente da home.

        Aqui isso vira uma coisa so: enquanto `self.booting`, alvos sao
        descartados e os servicos de movimento respondem "inicializacao em
        andamento". A UNICA excecao e a parada de emergencia - bloquear o
        estop durante o boot seria trocar uma pose indesejada por um braco
        que nao para.

        HONESTAMENTE: a entrada em modo backhoe e do FIRMWARE, e ele leva o
        braco ao neutro DE FABRICA antes de aceitar qualquer alvo (protocolo
        ArmLink, EXT_BACKHOE). Se a home do usuario for diferente do neutro
        de fabrica, o braco passa por ele - nao ha como pedir backhoe sem
        esse movimento. O que este codigo garante e que nada ALEM disso
        aconteca ate a home estar alcancada."""
        home = dict(self.user_neutral or armlink.NEUTRAL)
        self.booting = True
        self._boot_abort = False
        self.publish_status()
        self.get_logger().info(
            "INICIALIZANDO: modo backhoe e ida para a home pose. "
            "Alvos e servicos de movimento ficam bloqueados ate chegar.")
        try:
            self.arm.set_backhoe()
            if self._boot_abort:
                raise ArmLinkError("parada de emergencia durante o boot")
            self.targets = dict(home)
            self.dirty = False
            self.arm.move(home, dtime=_BOOT_DTIME)
            self.last_move_time = time.time()
            # Espera a interpolacao terminar antes de liberar o resto: o
            # firmware descarta pacotes que chegam durante o movimento, e
            # liberar antes deixaria o primeiro comando do operador cair no
            # vazio (parece que a GUI travou).
            deadline = time.time() + _BOOT_DTIME * 0.016
            while time.time() < deadline and not self._boot_abort:
                time.sleep(0.02)
            if self._boot_abort:
                raise ArmLinkError("parada de emergencia durante o boot")
            self.active = True
            self.get_logger().info("Braco na home pose - pronto para operar.")
        except (ArmLinkError, OSError) as exc:
            self.active = False
            self.get_logger().error(
                f"Inicializacao NAO chegou a home pose: {exc}. O braco esta "
                "onde parou; use Ativar quando a via estiver livre.")
        finally:
            self.booting = False
            self.publish_status()

    # ------------------------------------------------------------- entradas
    def on_targets(self, msg):
        if self.booting:
            self.get_logger().warning(
                "alvo ignorado: inicializacao em andamento (indo para a home)",
                throttle_duration_sec=2.0)
            return
        if not self.active:
            self.get_logger().warning(
                "alvo ignorado: braco inativo (chame /widowx/activate)",
                throttle_duration_sec=5.0)
            return
        for name, pos in zip(msg.name, msg.position):
            if name in armlink.JOINTS:
                self.targets[name] = armlink.clamp(name, pos)
        self.dirty = True

    def on_dtime(self, msg):
        self.dtime = min(max(msg.data, 1), 255)

    # ------------------------------------------------------------- servicos
    def _guarded(self, response, action, label, *, durante_boot=False):
        if self.booting and not durante_boot:
            response.success = False
            response.message = ("inicializacao em andamento: o braco esta "
                                "indo para a home pose")
            return response
        if not self.arm.connected:
            response.success = False
            response.message = "sem conexao com o ArbotiX"
            return response
        try:
            action()
            response.success = True
            response.message = label
        except (ArmLinkError, OSError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"{label}: {exc}")
        return response

    def on_activate(self, request, response):
        def do():
            self.arm.set_backhoe()
            self.targets = dict(armlink.NEUTRAL)
            self.dirty = False
            self.active = True
            self.last_move_time = time.time()
        return self._guarded(response, do, "modo backhoe ativo (braco no neutro)")

    def on_home(self, request, response):
        def do():
            if self.active and self.user_neutral is not None:
                # neutro do usuario: move suave no proprio modo backhoe
                self.arm.move(self.user_neutral, dtime=max(self.dtime, 125))
                self.targets = dict(self.user_neutral)
            else:
                self.arm.move_home()
                self.targets = dict(armlink.NEUTRAL)
            self.dirty = False
            self.last_move_time = time.time()
        return self._guarded(response, do, "braco na posicao neutra")

    def on_set_neutral(self, request, response):
        def do():
            if self.active:
                units = dict(self.targets)
            else:
                pos = self.arm.read_positions()
                if pos is None or set(pos) != set(armlink.JOINTS):
                    raise ArmLinkError("falha ao ler a posicao atual")
                units = {j: armlink.clamp(j, pos[j]) for j in armlink.JOINTS}
            self.user_neutral = units
            self._save_neutral()
            self._publish_neutral()
        return self._guarded(response, do,
                             "pose atual salva como nova posicao neutra")

    def on_sleep(self, request, response):
        def do():
            self.arm.sleep_arm()
            self.active = False
        return self._guarded(response, do, "braco recolhido, torque solto")

    def on_estop(self, request, response):
        def do():
            self.arm.estop()
            self.active = False
            self.dirty = False
        # `durante_boot=True`: a parada e a unica coisa que atravessa a
        # inicializacao. `_boot_abort` faz a rotina desistir da home em vez
        # de retomar o percurso depois do estop.
        self._boot_abort = True
        return self._guarded(response, do, "PARADA DE EMERGENCIA executada",
                             durante_boot=True)

    def on_diagnostics(self, request, response):
        lines = []
        for joint in armlink.JOINTS:
            d = self.diag_cache.get(joint)
            sid = armlink.SERVO_IDS[joint]
            if not d:
                lines.append(f"{joint} (ID {sid}): SEM LEITURA")
                continue
            suffix = " <<< " + ", ".join(d["avisos"]) if d["avisos"] else ""
            lines.append(
                f"{joint} (ID {sid}): {d['v']:.1f} V, {d['temp']} graus C, "
                f"carga {d['load']:.0f}%, "
                f"torque {'ON' if d['torque'] else 'OFF'}{suffix}")
        response.success = True
        response.message = "\n".join(lines)
        return response

    # ------------------------------------------------------- worker serial
    def _serial_worker(self):
        idx = 0
        while self._running:
            try:
                if not self.arm.connected:
                    time.sleep(0.5)
                    continue
                now = time.time()
                interval = min(max(self.dtime * 0.016, 0.06), 2.0)
                if self.dirty and self.active:
                    if now - self.last_move_time >= interval:
                        self.arm.move(self.targets, self.dtime)
                        self.dirty = False
                        self.last_move_time = time.time()
                    else:
                        time.sleep(0.01)
                    continue
                # o firmware descarta pacotes que chegam enquanto ainda
                # interpola o movimento anterior: espera passar
                if now - self.last_move_time < max(2 * self.dtime * 0.016,
                                                   0.35):
                    time.sleep(0.02)
                    continue
                joint = armlink.JOINTS[idx % len(armlink.JOINTS)]
                idx += 1
                if not self.active:
                    # torque solto: prioriza posicoes para os sliders
                    # acompanharem o braco movido a mao
                    for j in armlink.JOINTS:
                        if self.dirty and self.active:
                            break
                        pos = self.arm.read_register(
                            armlink.SERVO_IDS[j],
                            armlink.AX_REG_PRESENT_POSITION, 2)
                        if pos is not None:
                            self.pos_cache[j] = pos
                self._read_joint(joint)
                self._publish_readings()
                time.sleep(0.02)
            except (OSError, ArmLinkError) as exc:
                self.get_logger().error(f"erro serial: {exc}")
                time.sleep(1.0)

    def _read_joint(self, joint):
        sid = armlink.SERVO_IDS[joint]
        pos = self.arm.read_register(sid, armlink.AX_REG_PRESENT_POSITION, 2)
        if pos is not None:
            self.pos_cache[joint] = pos
        if self.dirty and self.active:
            return  # movimento pendente tem prioridade
        d = self.arm.read_diagnostics(joint)
        if d is None:
            self.diag_cache[joint] = None
            return
        avisos = []
        if d["torque_limit"] == 0:
            avisos.append("PROTECAO ATIVA (sobrecarga) - use Rearmar")
        if d["temp"] >= 60:
            avisos.append("TEMPERATURA ALTA")
        if not 10.0 <= d["voltage"] <= 14.0:
            avisos.append("TENSAO FORA DA FAIXA")
        if d["load_pct"] >= 60:
            avisos.append("CARGA ALTA")
        self.diag_cache[joint] = {"v": d["voltage"], "temp": d["temp"],
                                  "load": d["load_pct"],
                                  "torque": d["torque_on"],
                                  "tl": d["torque_limit"],
                                  "avisos": avisos}

    def _publish_readings(self):
        if self.pos_cache:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [j for j in armlink.JOINTS if j in self.pos_cache]
            msg.position = [armlink.units_to_rad(j, self.pos_cache[j])
                            for j in msg.name]
            self.pub_states.publish(msg)
        self.pub_diag.publish(String(data=json.dumps(
            {"stamp": time.time(), "servos": self.diag_cache})))

    def on_rearm(self, request, response):
        """Restaura o torque_limit dos servos que entraram em protecao
        (sobrecarga zera o registrador 34 e o LED fica piscando)."""
        if not self.arm.connected:
            response.success = False
            response.message = "sem conexao com o ArbotiX"
            return response
        rearmed = []
        for joint in armlink.JOINTS:
            sid = armlink.SERVO_IDS[joint]
            tl = self.arm.read_register(sid, armlink.AX_REG_TORQUE_LIMIT, 2)
            if tl == 0:
                self.arm.set_register(sid, armlink.AX_REG_TORQUE_LIMIT, 1023)
                rearmed.append(f"{joint} (ID {sid})")
                time.sleep(0.05)
        self.last_move_time = time.time()
        response.success = True
        if rearmed:
            response.message = ("protecao rearmada: " + ", ".join(rearmed) +
                                ". O LED continua piscando ate religar a "
                                "fonte.")
        else:
            response.message = "nenhum servo estava em protecao"
        self.get_logger().info(response.message)
        return response

    def _load_neutral(self):
        try:
            raw = json.loads(self.neutral_file.read_text())
            return {j: armlink.clamp(j, raw[j]) for j in armlink.JOINTS}
        except (OSError, ValueError, KeyError):
            return None

    def _save_neutral(self):
        self.neutral_file.write_text(json.dumps(self.user_neutral, indent=1))
        self.get_logger().info(f"neutro salvo em {self.neutral_file}")

    def _publish_neutral(self):
        neutral = self.user_neutral or armlink.NEUTRAL
        msg = JointState()
        msg.name = list(armlink.JOINTS)
        msg.position = [float(neutral[j]) for j in armlink.JOINTS]
        self.pub_neutral.publish(msg)

    # -------------------------------------------------------------- timers
    def publish_status(self):
        if self.arm.connected:
            name = armlink.ARM_NAMES.get(self.arm.arm_id, "?")
            if self.booting:
                state = "INICIALIZANDO - indo para a home pose"
            else:
                state = "ATIVO (backhoe)" if self.active else "torque solto"
            text = f"conectado | {name} | {state}"
        else:
            text = "desconectado"
        self.pub_status.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = WidowXDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.arm.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
