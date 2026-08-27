"""Protocolo ArmLink (ArbotiX-M / InterbotiX) para o braço WidowX Mark II.

O ArbotiX-M roda o firmware InterbotixArmLinkSerial a 115200 baud.
Pacote de comando (17 bytes):

    0xFF Xh Xl Yh Yl Zh Zl WAh WAl WRh WRl Gh Gl DTIME BOTOES EXT CHK
    CHK = 255 - (soma dos bytes 1..15) % 256

No modo backhoe os campos X/Y/Z/WA/WR/G carregam a posicao bruta de cada
servo (base, ombro, cotovelo, punho, rotacao do punho, garra).
Respostas do firmware (ID e leitura de registrador) tem 5 bytes:

    0xFF CMD H L CHK  com CHK = 255 - (CMD+H+L) % 256
"""

import math
import threading
import time

import serial

DEFAULT_BAUD = 115200

# Valores do byte EXT (instrucao estendida)
EXT_MOVE = 0x00       # comando de movimento no modo atual
EXT_ESTOP = 0x11      # parada de emergencia
EXT_BACKHOE = 0x40    # muda para modo backhoe (junta a junta) e vai ao neutro
EXT_HOME = 0x50       # move para a posicao neutra
EXT_SLEEP = 0x60      # recolhe o braco e solta o torque
EXT_ID = 0x70         # solicita pacote de identificacao
EXT_REG_READ = 0x81   # le registrador de um servo (X=id, Y=reg, Z=tamanho)
EXT_REG_WRITE = 0x82  # escreve registrador (X=id, Y=reg, Z=tamanho, WA=valor)

ARM_NAMES = {1: "PhantomX Pincher", 2: "PhantomX Reactor", 3: "WidowX"}
IK_MODES = {0: "cartesiano", 1: "cartesiano 90", 2: "cilindrico",
            3: "cilindrico 90", 4: "backhoe"}

AX_REG_TORQUE_ENABLE = 24
AX_REG_TORQUE_LIMIT = 34
AX_REG_PRESENT_POSITION = 36
AX_REG_PRESENT_LOAD = 40
AX_REG_PRESENT_VOLTAGE = 42
AX_REG_PRESENT_TEMP = 43

JOINTS = ["base", "shoulder", "elbow", "wrist_angle", "wrist_rotate", "gripper"]

SERVO_IDS = {"base": 1, "shoulder": 2, "elbow": 3,
             "wrist_angle": 4, "wrist_rotate": 5, "gripper": 6}

SERVO_MODELS = {"base": "MX-28", "shoulder": "MX-64", "elbow": "MX-64",
                "wrist_angle": "MX-28", "wrist_rotate": "AX-12",
                "gripper": "AX-12"}

# Limites e neutros em unidades brutas de servo (firmware GlobalArm.h, WIDOWX)
LIMITS = {"base": (0, 4095), "shoulder": (1024, 3072), "elbow": (1024, 3072),
          "wrist_angle": (1024, 3072), "wrist_rotate": (0, 1023),
          "gripper": (0, 512)}
NEUTRAL = {"base": 2048, "shoulder": 2048, "elbow": 2048,
           "wrist_angle": 2048, "wrist_rotate": 512, "gripper": 256}

# Resolucao angular: MX = 360 graus/4096 passos, AX = 300 graus/1023 passos
_MX_STEP = 2.0 * math.pi / 4096.0
_AX_STEP = (300.0 / 1023.0) * math.pi / 180.0
STEP_RAD = {"base": _MX_STEP, "shoulder": _MX_STEP, "elbow": _MX_STEP,
            "wrist_angle": _MX_STEP, "wrist_rotate": _AX_STEP,
            "gripper": _AX_STEP}


def clamp(joint, units):
    lo, hi = LIMITS[joint]
    return min(max(int(round(units)), lo), hi)


def units_to_rad(joint, units):
    return (units - NEUTRAL[joint]) * STEP_RAD[joint]


def rad_to_units(joint, rad):
    return clamp(joint, NEUTRAL[joint] + rad / STEP_RAD[joint])


def build_packet(x=2048, y=2048, z=2048, wa=2048, wr=512, g=256,
                 dtime=125, buttons=0, ext=EXT_MOVE):
    payload = [
        (x >> 8) & 0xFF, x & 0xFF,
        (y >> 8) & 0xFF, y & 0xFF,
        (z >> 8) & 0xFF, z & 0xFF,
        (wa >> 8) & 0xFF, wa & 0xFF,
        (wr >> 8) & 0xFF, wr & 0xFF,
        (g >> 8) & 0xFF, g & 0xFF,
        dtime & 0xFF, buttons & 0xFF, ext & 0xFF,
    ]
    chk = 255 - (sum(payload) % 256)
    return bytes([0xFF] + payload + [chk])


def _find_response(buf):
    """Procura um frame valido de 5 bytes (0xFF CMD H L CHK) em buf."""
    for i in range(len(buf) - 4):
        if buf[i] != 0xFF:
            continue
        cmd, hi, lo, chk = buf[i + 1], buf[i + 2], buf[i + 3], buf[i + 4]
        if chk == 255 - ((cmd + hi + lo) % 256):
            return cmd, hi, lo
    return None


class ArmLinkError(Exception):
    pass


class ArmLink:
    """Conexao serial com o ArbotiX-M falando o protocolo ArmLink."""

    def __init__(self, port="/dev/ttyUSB0", baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self._ser = None
        self._lock = threading.Lock()
        self.arm_id = None
        self.ik_mode = None

    @property
    def connected(self):
        return self._ser is not None and self._ser.is_open

    def connect(self, boot_timeout=7.0):
        """Abre a porta. Abrir a serial reinicia o ArbotiX (pulso de DTR);
        o firmware entao recolhe o braco e solta o torque dos servos."""
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        deadline = time.time() + boot_timeout
        banner = b""
        while time.time() < deadline:
            banner += self._ser.read(64)
            if b"Online" in banner:
                break
        resp = _find_response(banner)
        if resp is None:
            # placa ja estava ligada e nao reiniciou: pergunta a identidade
            resp = self._query(build_packet(ext=EXT_ID))
        if resp is None:
            self.close()
            raise ArmLinkError(
                f"ArbotiX nao respondeu em {self.port} a {self.baud} baud")
        # resposta de ID: CMD = ARMID, H = modo IK
        self.arm_id, self.ik_mode = resp[0], resp[1]
        return self.arm_id

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def _write(self, packet):
        with self._lock:
            self._ser.write(packet)
            self._ser.flush()

    def _query(self, packet, timeout=1.5):
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(packet)
            self._ser.flush()
            deadline = time.time() + timeout
            buf = b""
            while time.time() < deadline:
                # resposta tem 5 bytes; ler em blocos pequenos evita esperar
                # o timeout da serial com a resposta ja no buffer
                chunk = self._ser.read(5 if not buf else 1)
                if not chunk:
                    continue
                buf += chunk
                resp = _find_response(buf)
                if resp is not None:
                    return resp
        return None

    def ping(self):
        resp = self._query(build_packet(ext=EXT_ID))
        if resp is None:
            return None
        self.arm_id, self.ik_mode = resp[0], resp[1]
        return resp[0]

    def set_backhoe(self):
        """Entra no modo backhoe. ATENCAO: o braco se move para o neutro
        (~2 s). Bloqueia ate o firmware confirmar."""
        resp = self._query(build_packet(ext=EXT_BACKHOE), timeout=6.0)
        if resp is None:
            raise ArmLinkError("sem resposta ao mudar para modo backhoe")
        self.ik_mode = resp[1]

    def move_home(self):
        resp = self._query(build_packet(ext=EXT_HOME), timeout=6.0)
        if resp is None:
            raise ArmLinkError("sem resposta ao comando de posicao neutra")

    def sleep_arm(self):
        resp = self._query(build_packet(ext=EXT_SLEEP), timeout=6.0)
        if resp is None:
            raise ArmLinkError("sem resposta ao comando de dormir")

    def estop(self):
        resp = self._query(build_packet(ext=EXT_ESTOP), timeout=2.0)
        if resp is None:
            raise ArmLinkError("sem resposta a parada de emergencia")

    def move(self, targets, dtime=60):
        """Envia posicoes-alvo (dict junta -> unidades) no modo backhoe.
        dtime*16 ms e o tempo de interpolacao do movimento."""
        u = {j: clamp(j, targets.get(j, NEUTRAL[j])) for j in JOINTS}
        pkt = build_packet(x=u["base"], y=u["shoulder"], z=u["elbow"],
                           wa=u["wrist_angle"], wr=u["wrist_rotate"],
                           g=u["gripper"], dtime=dtime, ext=EXT_MOVE)
        self._write(pkt)

    def read_register(self, servo_id, reg, length=2, timeout=0.25, retries=3):
        """Le um registrador via ArbotiX. 0xFFFF indica falha de leitura no
        barramento (o firmware repassa o -1 do ax12GetRegister); leituras em
        sequencia rapida falham com frequencia, dai o retry com espacamento."""
        pkt = build_packet(x=servo_id, y=reg, z=length, ext=EXT_REG_READ)
        for _ in range(retries):
            resp = self._query(pkt, timeout=timeout)
            if resp is not None and resp[0] == EXT_REG_READ:
                val = (resp[1] << 8) | resp[2]
                if val != 0xFFFF:
                    return val
            time.sleep(0.05)
        return None

    def set_register(self, servo_id, reg, value, length=2):
        """Escreve um registrador de um servo via ArbotiX (sem resposta)."""
        self._write(build_packet(x=servo_id, y=reg, z=length, wa=value,
                                 ext=EXT_REG_WRITE))

    def read_diagnostics(self, joint):
        """Le tensao (V), temperatura (C), carga (%) e torque de um servo.
        Retorna dict ou None se alguma leitura falhar."""
        sid = SERVO_IDS[joint]
        out = {}
        volt = self.read_register(sid, AX_REG_PRESENT_VOLTAGE, 1)
        temp = self.read_register(sid, AX_REG_PRESENT_TEMP, 1)
        load = self.read_register(sid, AX_REG_PRESENT_LOAD, 2)
        torque = self.read_register(sid, AX_REG_TORQUE_ENABLE, 1)
        tlimit = self.read_register(sid, AX_REG_TORQUE_LIMIT, 2)
        if None in (volt, temp, load, torque, tlimit):
            return None
        out["voltage"] = volt / 10.0
        out["temp"] = temp
        # carga: bits 0-9 = magnitude, bit 10 = sentido
        out["load_pct"] = (load & 0x3FF) / 1023.0 * 100.0
        out["torque_on"] = bool(torque)
        # torque_limit zerado = servo entrou em protecao (sobrecarga)
        out["torque_limit"] = tlimit
        return out

    def read_positions(self):
        """Le a posicao atual das juntas. Retorna dict (possivelmente
        parcial) ou None se nenhum servo respondeu."""
        out = {}
        for joint in JOINTS:
            val = self.read_register(SERVO_IDS[joint],
                                     AX_REG_PRESENT_POSITION, 2)
            if val is not None:
                out[joint] = val
            time.sleep(0.01)
        return out or None
