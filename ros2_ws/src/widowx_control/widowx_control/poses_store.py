"""Poses e movimentos do teach pendant: schema, saneamento e persistência.

Tudo aqui é lógica pura (sem Tk e sem ROS) para poder ser testado e para o
arquivo em disco ter UM dono. O formato:

    {
      "version": 1,
      "poses": [
        {"id": 1, "name": "Pose 1",
         "units": {"base": 2048, ..., "gripper": 256}}
      ],
      "movements": [
        {"id": 1, "name": "Movimento 1", "move_ms": 800,
         "steps": [{"pose_id": 1, "dwell_s": 1.0}]}
      ]
    }

As poses guardam UNIDADES BRUTAS de servo, não graus: é o que o driver
consome e o que os limites do `armlink` descrevem. Guardar graus obrigaria a
uma conversão de ida e volta a cada execução, e o passo do MX (360°/4096) e o
do AX (300°/1023) não são o mesmo — o erro de arredondamento apareceria como
uma pose que "anda sozinha" alguns passos a cada ciclo de gravação.

O tempo é POR PASSO (`dwell_s`): quanto o braço FICA parado naquela pose antes
de ir para a próxima. O tempo de PERCURSO (`move_ms`) é do movimento inteiro,
porque é ele que vira o `dtime` do ArbotiX.
"""

import json
import os
import tempfile

from . import armlink

VERSION = 1

# Faixa do tempo de percurso. O ArbotiX recebe dtime em unidades de 16 ms
# (1-255), então acima de ~4 s o valor satura no firmware.
MOVE_MS_MIN = 80
MOVE_MS_MAX = 4000
MOVE_MS_DEFAULT = 800

# Faixa do tempo de permanência, em segundos.
DWELL_MIN = 0.0
DWELL_MAX = 600.0
DWELL_DEFAULT = 1.0


def default_path() -> str:
    """Mesma convenção do neutro do usuário (~/.widowx_neutral.json)."""
    return os.path.join(os.path.expanduser("~"), ".widowx_poses.json")


def clamp_move_ms(value) -> int:
    try:
        v = int(round(float(value)))
    except (TypeError, ValueError):
        return MOVE_MS_DEFAULT
    return min(max(v, MOVE_MS_MIN), MOVE_MS_MAX)


def clamp_dwell(value) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return DWELL_DEFAULT
    return round(min(max(v, DWELL_MIN), DWELL_MAX), 2)


def move_ms_to_dtime(move_ms) -> int:
    """Tempo de percurso (ms) → dtime do ArmLink (unidades de 16 ms, 1-255)."""
    return min(max(int(clamp_move_ms(move_ms) // 16), 1), 255)


def clamp_units(units) -> dict:
    """Pose sempre dentro dos limites do WidowX.

    Saneada na ENTRADA, não na hora de mover: uma pose fora de faixa é um
    pacote que o firmware recusa, e descobrir isso no meio de uma sequência
    deixa o braço parado no passo anterior sem nada na tela explicando."""
    out = {}
    for joint in armlink.JOINTS:
        raw = (units or {}).get(joint, armlink.NEUTRAL[joint])
        try:
            out[joint] = armlink.clamp(joint, float(raw))
        except (TypeError, ValueError):
            out[joint] = armlink.NEUTRAL[joint]
    return out


def _coerce_step(step) -> dict | None:
    """Aceita tanto {'pose_id': N, 'dwell_s': X} quanto um id solto.

    O id solto é o formato do teach pendant do cr10twin (`pose_ids` é uma
    lista de inteiros). Um arquivo escrito à mão ou vindo de lá continua
    abrindo, com o dwell padrão."""
    if isinstance(step, dict):
        pid = step.get("pose_id")
        dwell = step.get("dwell_s", DWELL_DEFAULT)
    else:
        pid, dwell = step, DWELL_DEFAULT
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return None
    return {"pose_id": pid, "dwell_s": clamp_dwell(dwell)}


def normalize(data) -> dict:
    """Devolve um dicionário no schema, doa o que vier. Um JSON corrompido
    não pode derrubar a GUI inteira na abertura."""
    if not isinstance(data, dict):
        data = {}
    poses = []
    for raw in data.get("poses") or []:
        if not isinstance(raw, dict) or "id" not in raw:
            continue
        try:
            pid = int(raw["id"])
        except (TypeError, ValueError):
            continue
        poses.append({
            "id": pid,
            "name": str(raw.get("name") or f"Pose {pid}"),
            "units": clamp_units(raw.get("units")),
        })
    known = {p["id"] for p in poses}

    movements = []
    for raw in data.get("movements") or []:
        if not isinstance(raw, dict) or "id" not in raw:
            continue
        try:
            mid = int(raw["id"])
        except (TypeError, ValueError):
            continue
        # aceita 'steps' (formato daqui) ou 'pose_ids' (formato do cr10twin)
        raw_steps = raw.get("steps")
        if raw_steps is None:
            raw_steps = raw.get("pose_ids") or []
        steps = [s for s in (_coerce_step(s) for s in raw_steps)
                 if s is not None and s["pose_id"] in known]
        movements.append({
            "id": mid,
            "name": str(raw.get("name") or f"Movimento {mid}"),
            "move_ms": clamp_move_ms(raw.get("move_ms", MOVE_MS_DEFAULT)),
            "steps": steps,
        })
    return {"version": VERSION, "poses": poses, "movements": movements}


def load(path: str) -> dict:
    try:
        with open(path) as fh:
            return normalize(json.load(fh))
    except (FileNotFoundError, json.JSONDecodeError, OSError, ValueError):
        return normalize(None)


def save(path: str, data: dict) -> None:
    """Escrita atômica: a gravação acontece a cada tecla mexida no dwell, e um
    Ctrl+C no meio de um json.dump direto sobre o arquivo deixaria o usuário
    sem nenhuma pose."""
    payload = normalize(data)
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".widowx_poses.",
                               suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(payload, fh, indent=2)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def next_id(items) -> int:
    return max((int(i.get("id", 0)) for i in items), default=0) + 1


def by_id(items, item_id):
    for item in items:
        if item["id"] == item_id:
            return item
    return None


def total_seconds(movement, poses) -> float:
    """Duração de uma passagem: percurso + permanência de cada passo.

    O percurso do PRIMEIRO passo também conta: o braço tem de sair de onde
    está para chegar à primeira pose."""
    move_s = clamp_move_ms(movement.get("move_ms", MOVE_MS_DEFAULT)) / 1000.0
    known = {p["id"] for p in poses}
    total = 0.0
    for step in movement.get("steps") or []:
        if step.get("pose_id") not in known:
            continue
        total += move_s + clamp_dwell(step.get("dwell_s", DWELL_DEFAULT))
    return total


def format_hms(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    if seconds < 60.0:
        return f"{seconds:.1f} s"
    minutes, rest = divmod(seconds, 60.0)
    return f"{int(minutes)} min {rest:04.1f} s"
