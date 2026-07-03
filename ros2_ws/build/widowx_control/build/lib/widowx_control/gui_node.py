"""GUI de controle manual do WidowX, inspirada no Dynamixel Wizard.

Layout: barra de ferramentas no topo, lista de servos a esquerda (com estado
em tempo real), abas a direita (Controle manual / Servo / Diagnostico) e
barra de status embaixo. Tema claro.

Publica alvos em /widowx/joint_targets (unidades brutas de servo) e chama os
servicos do driver. O diagnostico chega continuamente por /widowx/diag.
"""

import json
import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from . import armlink

SLIDERS = [
    ("Base", "base"),
    ("Ombro", "shoulder"),
    ("Cotovelo", "elbow"),
    ("Punho flexão", "wrist_angle"),
    ("Punho rotação", "wrist_rotate"),
    ("Garra", "gripper"),
]
JOINT_LABEL = {j: l for l, j in SLIDERS}

PUBLISH_PERIOD_MS = 100

# Comprimentos dos elos (mm), do firmware GlobalArm.h
BASE_H, L1, L2, L3 = 125.0, 150.0, 142.0, 155.0

# ------------------------------------------------------------ tema claro
BG = "#f4f6f8"          # fundo geral
CARD = "#ffffff"        # paineis
INK = "#263238"         # texto
MUTED = "#78909c"       # texto secundario
ACCENT = "#1976d2"      # azul principal
OK = "#2e7d32"          # verde
WARN = "#c62828"        # vermelho
LINE = "#e0e4e8"        # bordas


def units_to_deg(joint, units):
    return armlink.units_to_rad(joint, units) * 180.0 / math.pi


def deg_to_units(joint, deg):
    return armlink.rad_to_units(joint, deg * math.pi / 180.0)


class GuiNode(Node):

    def __init__(self):
        super().__init__("widowx_gui")
        self.pub_targets = self.create_publisher(JointState,
                                                 "/widowx/joint_targets", 10)
        self.pub_dtime = self.create_publisher(Int32, "/widowx/dtime", 10)
        self.cli = {name: self.create_client(Trigger, f"/widowx/{name}")
                    for name in ("activate", "home", "sleep", "estop",
                                 "set_neutral", "rearm")}
        self.status_text = "aguardando driver..."
        self.joint_states = {}
        self.diag = None
        self.user_neutral = dict(armlink.NEUTRAL)
        self.create_subscription(String, "/widowx/status", self._on_status, 10)
        self.create_subscription(JointState, "/widowx/joint_states",
                                 self._on_states, 10)
        self.create_subscription(String, "/widowx/diag", self._on_diag, 10)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/widowx/neutral",
                                 self._on_neutral, latched)

    def _on_status(self, msg):
        self.status_text = msg.data

    def _on_states(self, msg):
        self.joint_states.update(zip(msg.name, msg.position))

    def _on_diag(self, msg):
        try:
            self.diag = json.loads(msg.data)
        except ValueError:
            pass

    def _on_neutral(self, msg):
        self.user_neutral = {n: p for n, p in zip(msg.name, msg.position)
                             if n in armlink.JOINTS}

    def call(self, name, done_cb):
        client = self.cli[name]
        if not client.service_is_ready():
            done_cb(False, "driver indisponível")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: done_cb(f.result().success, f.result().message))


class ArmModel:
    """Desenho 2D do braco: vista lateral da cadeia + mostrador da base."""

    def __init__(self, parent, width=640, height=250):
        self.w, self.h = width, height
        self.canvas = tk.Canvas(parent, width=width, height=height, bg=CARD,
                                highlightthickness=1,
                                highlightbackground=LINE)
        self.canvas.pack(fill="x", padx=8, pady=8)
        self.scale = (height - 40) / (BASE_H + L1 + L2 + L3)

    def draw(self, deg, gripper_units, active_joint):
        c = self.canvas
        c.delete("all")

        dial_cx, dial_cy, r = 66, 74, 42
        cor = WARN if False else (ACCENT if active_joint != "base"
                                  else "#ff6f00")
        c.create_oval(dial_cx - r, dial_cy - r, dial_cx + r, dial_cy + r,
                      outline=cor, width=2)
        a = math.radians(deg["base"])
        c.create_line(dial_cx, dial_cy,
                      dial_cx + r * math.sin(a), dial_cy - r * math.cos(a),
                      fill=cor, width=3, arrow="last")
        c.create_text(dial_cx, dial_cy + r + 12, text="Base — vista de cima",
                      fill=MUTED, font=("TkDefaultFont", 8))
        c.create_text(dial_cx, dial_cy - r - 10, text=f"{deg['base']:+.0f}°",
                      fill=cor)

        k = self.scale
        ox = self.w * 0.60
        oy = self.h - 18.0
        c.create_line(ox - 100, oy, ox + 100, oy, fill=LINE, width=3)
        shoulder = (ox, oy - BASE_H * k)
        c.create_line(ox, oy, *shoulder, fill="#455a64", width=9)

        a1 = math.radians(deg["shoulder"])
        a2 = a1 + math.radians(deg["elbow"])
        a3 = a2 + math.radians(deg["wrist_angle"])

        def step(p, ang, length):
            return (p[0] + length * k * math.sin(ang),
                    p[1] - length * k * math.cos(ang))

        elbow = step(shoulder, a1, L1)
        wrist = step(elbow, a2, L2)
        tip = step(wrist, a3, L3)

        for p, q in ((shoulder, elbow), (elbow, wrist), (wrist, tip)):
            c.create_line(*p, *q, fill="#455a64", width=6, capstyle="round")

        gap = 3 + (gripper_units / 512.0) * 14
        nx, ny = math.cos(a3), math.sin(a3)
        cor_g = "#ff6f00" if active_joint == "gripper" else ACCENT
        for s in (1, -1):
            fx, fy = tip[0] + s * gap * nx, tip[1] + s * gap * ny
            c.create_line(fx, fy, fx + 16 * math.sin(a3),
                          fy - 16 * math.cos(a3), fill=cor_g, width=4)

        cor_r = "#ff6f00" if active_joint == "wrist_rotate" else ACCENT
        mx, my = (wrist[0] + tip[0]) / 2, (wrist[1] + tip[1]) / 2
        wr = math.radians(deg["wrist_rotate"])
        c.create_line(mx - 12 * math.cos(wr), my - 12 * math.sin(wr),
                      mx + 12 * math.cos(wr), my + 12 * math.sin(wr),
                      fill=cor_r, width=3)

        for joint, nome, p in (("shoulder", "Ombro", shoulder),
                               ("elbow", "Cotovelo", elbow),
                               ("wrist_angle", "Punho", wrist)):
            cor = "#ff6f00" if active_joint == joint else ACCENT
            rj = 9 if active_joint == joint else 7
            c.create_oval(p[0] - rj, p[1] - rj, p[0] + rj, p[1] + rj,
                          fill=cor, outline=CARD, width=2)
            c.create_text(p[0] + 14, p[1] - 12, text=nome, anchor="w",
                          fill=cor, font=("TkDefaultFont", 9))
        if active_joint:
            c.create_text(self.w - 12, 16, anchor="e",
                          text=f"Movendo: {JOINT_LABEL[active_joint]}",
                          fill="#ff6f00", font=("TkDefaultFont", 10, "bold"))


class WidowXGui:

    def __init__(self, node):
        self.node = node
        self.active = False
        self.pending = False
        self.active_joint = None
        self.dtime_published = False
        self.selected = "base"
        self._started = False

        self.root = tk.Tk()
        self.root.title("openARM — WidowX Mark II")
        self.root.geometry("1020x700")
        self.root.configure(bg=BG)
        self._style()

        self._toolbar()

        body = ttk.Frame(self.root, style="Bg.TFrame")
        body.pack(fill="both", expand=True, padx=10, pady=(0, 4))

        self._sidebar(body)

        self.nb = ttk.Notebook(body)
        self.nb.pack(side="left", fill="both", expand=True, padx=(10, 0))
        self._tab_control()
        self._tab_servo()
        self._tab_diag()

        self._statusbar()

        self._set_sliders_to(armlink.NEUTRAL)
        self._set_sliders_enabled(False)
        self.root.after(PUBLISH_PERIOD_MS, self._tick)
        # a selecao inicial da sidebar dispara _on_select na abertura;
        # so troca de aba automaticamente depois que a janela estabilizar
        self.root.after(500, lambda: setattr(self, "_started", True))

    # ------------------------------------------------------------- visual
    def _style(self):
        s = ttk.Style(self.root)
        s.theme_use("clam")
        s.configure(".", background=BG, foreground=INK, font=("TkDefaultFont", 10))
        s.configure("Bg.TFrame", background=BG)
        s.configure("Card.TFrame", background=CARD, relief="solid",
                    borderwidth=1, bordercolor=LINE)
        s.configure("TNotebook", background=BG, borderwidth=0)
        s.configure("TNotebook.Tab", padding=(16, 7), background="#e3e8ed",
                    foreground=INK)
        s.map("TNotebook.Tab", background=[("selected", CARD)],
              foreground=[("selected", ACCENT)])
        s.configure("TButton", padding=(12, 6), background="#e3e8ed",
                    borderwidth=0)
        s.map("TButton", background=[("active", "#d3dbe2")])
        s.configure("Accent.TButton", background=ACCENT, foreground="white")
        s.map("Accent.TButton", background=[("active", "#1565c0")])
        s.configure("Stop.TButton", background=WARN, foreground="white",
                    font=("TkDefaultFont", 10, "bold"))
        s.map("Stop.TButton", background=[("active", "#b71c1c")])
        s.configure("Card.TLabel", background=CARD)
        s.configure("Muted.TLabel", background=CARD, foreground=MUTED,
                    font=("TkDefaultFont", 9))
        s.configure("Title.TLabel", background=CARD, foreground=INK,
                    font=("TkDefaultFont", 12, "bold"))
        s.configure("Horizontal.TScale", background=CARD,
                    troughcolor="#e3e8ed")
        s.configure("Treeview", background=CARD, fieldbackground=CARD,
                    foreground=INK, rowheight=30, borderwidth=0)
        s.configure("Treeview.Heading", background="#e3e8ed",
                    foreground=INK, padding=6, borderwidth=0)

    def _toolbar(self):
        bar = ttk.Frame(self.root, style="Bg.TFrame")
        bar.pack(fill="x", padx=10, pady=8)
        ttk.Button(bar, text="▶  Ativar", style="Accent.TButton",
                   command=lambda: self._service("activate")).pack(
            side="left", padx=(0, 4))
        ttk.Button(bar, text="⌂  Posição neutra",
                   command=lambda: self._service("home")).pack(side="left",
                                                               padx=4)
        ttk.Button(bar, text="📍  Definir neutro aqui",
                   command=self._set_neutral_here).pack(side="left", padx=4)
        ttk.Button(bar, text="💤  Dormir",
                   command=lambda: self._service("sleep")).pack(side="left",
                                                                padx=4)
        ttk.Button(bar, text="🔧  Rearmar proteção",
                   command=lambda: self._service("rearm")).pack(side="left",
                                                                padx=4)
        ttk.Button(bar, text="⛔  PARADA", style="Stop.TButton",
                   command=lambda: self._service("estop")).pack(side="right")

    def _sidebar(self, parent):
        card = ttk.Frame(parent, style="Card.TFrame")
        card.pack(side="left", fill="y")
        ttk.Label(card, text="Servos", style="Title.TLabel").pack(
            anchor="w", padx=12, pady=(10, 4))
        self.tree = ttk.Treeview(card, columns=("estado",), show="tree",
                                 selectmode="browse", height=len(SLIDERS))
        self.tree.column("#0", width=210)
        for _, joint in SLIDERS:
            sid = armlink.SERVO_IDS[joint]
            model = armlink.SERVO_MODELS[joint]
            self.tree.insert("", "end", iid=joint,
                             text=f"●  ID {sid} · {JOINT_LABEL[joint]} "
                                  f"({model})")
        self.tree.pack(fill="both", expand=False, padx=8, pady=4)
        self.tree.selection_set("base")
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        self.tree.tag_configure("ok", foreground=OK)
        self.tree.tag_configure("warn", foreground=WARN)
        self.tree.tag_configure("off", foreground=MUTED)

    # --------------------------------------------------------------- abas
    def _tab_control(self):
        tab = ttk.Frame(self.nb, style="Card.TFrame")
        self.nb.add(tab, text="  Controle manual  ")
        self.model = ArmModel(tab)

        grid = ttk.Frame(tab, style="Card.TFrame")
        grid.pack(fill="both", expand=True, padx=12, pady=4)
        self.scales, self.value_vars = {}, {}
        for row, (label, joint) in enumerate(SLIDERS):
            ttk.Label(grid, text=f"{label}  ·  ID "
                      f"{armlink.SERVO_IDS[joint]}",
                      style="Card.TLabel").grid(row=row, column=0,
                                                sticky="w", pady=5)
            lo, hi = armlink.LIMITS[joint]
            frm, to = ((float(lo), float(hi)) if joint == "gripper" else
                       (units_to_deg(joint, lo), units_to_deg(joint, hi)))
            scale = ttk.Scale(grid, from_=frm, to=to, orient="horizontal",
                              command=lambda v, j=joint: self._on_slider(j))
            scale.grid(row=row, column=1, sticky="ew", padx=10)
            scale.bind("<Enter>", lambda e, j=joint: self._hover(j))
            scale.bind("<Leave>", lambda e: self._hover(None))
            var = tk.StringVar()
            ttk.Label(grid, textvariable=var, width=8,
                      style="Card.TLabel").grid(row=row, column=2)
            self.scales[joint], self.value_vars[joint] = scale, var
        grid.columnconfigure(1, weight=1)

        speed = ttk.Frame(tab, style="Card.TFrame")
        speed.pack(fill="x", padx=12, pady=(2, 10))
        ttk.Label(speed, text="Duração do movimento",
                  style="Muted.TLabel").pack(side="left")
        self.speed_var = tk.StringVar(value="240 ms")
        self.speed_scale = ttk.Scale(speed, from_=80, to=2000,
                                     orient="horizontal",
                                     command=lambda v: self._on_speed())
        self.speed_scale.pack(side="left", fill="x", expand=True, padx=10)
        ttk.Label(speed, textvariable=self.speed_var, width=8,
                  style="Muted.TLabel").pack(side="left")
        self.speed_scale.set(240)

    def _tab_servo(self):
        tab = ttk.Frame(self.nb, style="Card.TFrame")
        self.nb.add(tab, text="  Servo  ")
        self.servo_title = ttk.Label(tab, text="", style="Title.TLabel")
        self.servo_title.pack(anchor="w", padx=16, pady=(14, 2))
        self.servo_sub = ttk.Label(tab, text="", style="Muted.TLabel")
        self.servo_sub.pack(anchor="w", padx=16)

        grid = ttk.Frame(tab, style="Card.TFrame")
        grid.pack(anchor="w", padx=16, pady=12)
        self.servo_vars = {}
        campos = [("Posição atual", "pos"), ("Alvo (slider)", "alvo"),
                  ("Tensão", "v"), ("Temperatura", "temp"),
                  ("Carga", "load"), ("Torque", "torque"),
                  ("Limite de torque", "tl"), ("Avisos", "avisos")]
        for row, (nome, key) in enumerate(campos):
            ttk.Label(grid, text=nome, style="Muted.TLabel").grid(
                row=row, column=0, sticky="w", pady=3, padx=(0, 18))
            var = tk.StringVar(value="—")
            lbl = ttk.Label(grid, textvariable=var, style="Card.TLabel",
                            font=("TkDefaultFont", 11))
            lbl.grid(row=row, column=1, sticky="w")
            self.servo_vars[key] = (var, lbl)

        ttk.Label(tab, text="Mover este servo",
                  style="Muted.TLabel").pack(anchor="w", padx=16,
                                             pady=(10, 0))
        self.servo_scale = ttk.Scale(
            tab, from_=0, to=1, orient="horizontal",
            command=lambda v: self._on_servo_scale())
        self.servo_scale.pack(fill="x", padx=16, pady=(2, 16))
        self._servo_scale_sync = False

    def _tab_diag(self):
        tab = ttk.Frame(self.nb, style="Card.TFrame")
        self.nb.add(tab, text="  Diagnóstico  ")
        cols = ("id", "modelo", "pos", "v", "temp", "load", "torque", "tl",
                "avisos")
        self.diag_tree = ttk.Treeview(tab, columns=cols, show="headings",
                                      height=len(SLIDERS))
        heads = {"id": ("ID", 40), "modelo": ("Modelo", 70),
                 "pos": ("Posição", 70), "v": ("Tensão", 70),
                 "temp": ("Temp.", 60), "load": ("Carga", 60),
                 "torque": ("Torque", 60), "tl": ("Limite", 60),
                 "avisos": ("Avisos", 260)}
        for col, (txt, w) in heads.items():
            self.diag_tree.heading(col, text=txt)
            self.diag_tree.column(col, width=w, anchor="center")
        self.diag_tree.column("avisos", anchor="w")
        for _, joint in SLIDERS:
            self.diag_tree.insert("", "end", iid=joint,
                                  values=(armlink.SERVO_IDS[joint],
                                          armlink.SERVO_MODELS[joint],
                                          "—", "—", "—", "—", "—", "—", ""))
        self.diag_tree.tag_configure("warn", foreground=WARN)
        self.diag_tree.pack(fill="both", expand=True, padx=12, pady=12)
        ttk.Label(tab, text="Atualização contínua: o driver varre um servo "
                            "por vez nos intervalos entre movimentos.",
                  style="Muted.TLabel").pack(anchor="w", padx=14,
                                             pady=(0, 10))

    def _statusbar(self):
        bar = tk.Frame(self.root, bg=CARD, highlightthickness=1,
                       highlightbackground=LINE)
        bar.pack(fill="x", side="bottom")
        self.status_dot = tk.Label(bar, text="●", bg=CARD, fg=MUTED)
        self.status_dot.pack(side="left", padx=(10, 2), pady=3)
        self.status_var = tk.StringVar(value="aguardando driver...")
        tk.Label(bar, textvariable=self.status_var, bg=CARD,
                 fg=INK).pack(side="left")
        self.diag_age_var = tk.StringVar(value="")
        tk.Label(bar, textvariable=self.diag_age_var, bg=CARD,
                 fg=MUTED).pack(side="right", padx=10)

    # ------------------------------------------------------------- helpers
    def _set_sliders_to(self, units):
        for _, joint in SLIDERS:
            u = armlink.clamp(joint, units.get(joint, armlink.NEUTRAL[joint]))
            self.scales[joint].set(u if joint == "gripper"
                                   else units_to_deg(joint, u))
        self._refresh_labels()

    def _set_sliders_enabled(self, enabled):
        state = ["!disabled"] if enabled else ["disabled"]
        for scale in self.scales.values():
            scale.state(state)
        self.servo_scale.state(state)

    def _refresh_labels(self):
        for _, joint in SLIDERS:
            v = self.scales[joint].get()
            self.value_vars[joint].set(f"{v:.0f}" if joint == "gripper"
                                       else f"{v:+.1f}°")

    def _slider_units(self):
        return {joint: (armlink.clamp(joint, self.scales[joint].get())
                        if joint == "gripper"
                        else deg_to_units(joint, self.scales[joint].get()))
                for _, joint in SLIDERS}

    def _slider_degs(self):
        return {joint: (0.0 if joint == "gripper"
                        else self.scales[joint].get())
                for _, joint in SLIDERS}

    # ------------------------------------------------------------ eventos
    def _hover(self, joint):
        self.active_joint = joint

    def _on_slider(self, joint):
        self._refresh_labels()
        self.active_joint = joint
        if self.active:
            self.pending = True

    def _on_servo_scale(self):
        if self._servo_scale_sync:
            return
        self.scales[self.selected].set(self.servo_scale.get())

    def _on_select(self, _event):
        sel = self.tree.selection()
        if sel:
            self.selected = sel[0]
            lo, hi = armlink.LIMITS[self.selected]
            frm, to = ((float(lo), float(hi)) if self.selected == "gripper"
                       else (units_to_deg(self.selected, lo),
                             units_to_deg(self.selected, hi)))
            self._servo_scale_sync = True
            self.servo_scale.configure(from_=frm, to=to)
            self.servo_scale.set(self.scales[self.selected].get())
            self._servo_scale_sync = False
            if self._started:
                self.nb.select(1)

    def _on_speed(self):
        ms = int(self.speed_scale.get())
        self.speed_var.set(f"{ms} ms")
        self.node.pub_dtime.publish(Int32(data=max(1, min(255, ms // 16))))

    def _set_neutral_here(self):
        if messagebox.askokcancel(
                "Definir neutro",
                "Salvar a pose ATUAL do braço como nova posição neutra?\n"
                "O botão \"Posição neutra\" passará a mover o braço para cá."):
            self._service("set_neutral")

    def _service(self, name):
        if name == "activate" and not messagebox.askokcancel(
                "Ativar braço",
                "O braço vai se mover para a posição neutra de fábrica "
                "(~2 s).\nGaranta que a área ao redor está livre."):
            return

        def done(ok, msg):
            self.root.after(0, self._service_done, name, ok, msg)

        self.node.call(name, done)

    def _service_done(self, name, ok, msg):
        if not ok:
            messagebox.showerror("Erro", f"{name}: {msg}")
            return
        if name == "activate":
            self.active = True
            self._set_sliders_to(armlink.NEUTRAL)
            self._set_sliders_enabled(True)
        elif name == "home":
            self._set_sliders_to(self.node.user_neutral)
        elif name in ("sleep", "estop"):
            self.active = False
            self._set_sliders_enabled(False)
        elif name == "set_neutral":
            messagebox.showinfo("Neutro salvo", msg)
        elif name == "rearm":
            messagebox.showinfo("Rearmar proteção", msg)

    # --------------------------------------------------------------- loop
    def _tick(self):
        self._refresh_status()
        if self.active and self.pending:
            self.pending = False
            units = self._slider_units()
            msg = JointState()
            msg.name = list(armlink.JOINTS)
            msg.position = [float(units[j]) for j in armlink.JOINTS]
            self.node.pub_targets.publish(msg)
        elif not self.active and self.node.joint_states:
            for _, joint in SLIDERS:
                rad = self.node.joint_states.get(joint)
                if rad is None:
                    continue
                units = armlink.rad_to_units(joint, rad)
                self.scales[joint].set(units if joint == "gripper"
                                       else units_to_deg(joint, units))
            self._refresh_labels()
        if not self.dtime_published and \
                self.node.pub_dtime.get_subscription_count():
            self._on_speed()
            self.dtime_published = True
        self._refresh_diag()
        self._refresh_servo_tab()
        self.model.draw(self._slider_degs(),
                        armlink.clamp("gripper",
                                      self.scales["gripper"].get()),
                        self.active_joint)
        self.root.after(PUBLISH_PERIOD_MS, self._tick)

    def _refresh_status(self):
        text = self.node.status_text
        self.status_var.set(text)
        if "ATIVO" in text:
            self.status_dot.config(fg=OK)
        elif "conectado" in text:
            self.status_dot.config(fg=ACCENT)
        else:
            self.status_dot.config(fg=WARN)

    def _diag_of(self, joint):
        if not self.node.diag:
            return None
        return self.node.diag.get("servos", {}).get(joint)

    def _refresh_diag(self):
        diag = self.node.diag
        if not diag:
            return
        age = time.time() - diag.get("stamp", 0)
        self.diag_age_var.set(f"diagnóstico contínuo · última leitura há "
                              f"{age:.0f} s")
        for _, joint in SLIDERS:
            d = self._diag_of(joint)
            sid = armlink.SERVO_IDS[joint]
            label = f"ID {sid} · {JOINT_LABEL[joint]} " \
                    f"({armlink.SERVO_MODELS[joint]})"
            if d is None:
                self.tree.item(joint, text=f"●  {label}", tags=("off",))
                self.diag_tree.item(joint, values=(
                    sid, armlink.SERVO_MODELS[joint], "—", "—", "—", "—",
                    "—", "—", "sem leitura"), tags=())
                continue
            tag = "warn" if d["avisos"] else "ok"
            self.tree.item(joint, text=f"●  {label}", tags=(tag,))
            pos = self.node.joint_states.get(joint)
            pos_txt = ("—" if pos is None else
                       f"{armlink.rad_to_units(joint, pos)}")
            self.diag_tree.item(joint, values=(
                sid, armlink.SERVO_MODELS[joint], pos_txt,
                f"{d['v']:.1f} V", f"{d['temp']}°C", f"{d['load']:.0f}%",
                "ON" if d["torque"] else "off", d.get("tl", "—"),
                ", ".join(d["avisos"])),
                tags=("warn",) if d["avisos"] else ())

    def _refresh_servo_tab(self):
        joint = self.selected
        sid = armlink.SERVO_IDS[joint]
        self.servo_title.config(
            text=f"{JOINT_LABEL[joint]} — ID {sid} "
                 f"({armlink.SERVO_MODELS[joint]})")
        lo, hi = armlink.LIMITS[joint]
        self.servo_sub.config(
            text=f"faixa {lo}–{hi} · neutro {armlink.NEUTRAL[joint]} "
                 "(unidades de servo)")

        pos = self.node.joint_states.get(joint)
        if pos is None:
            self._servo_set("pos", "—")
        else:
            u = armlink.rad_to_units(joint, pos)
            deg = units_to_deg(joint, u)
            self._servo_set("pos", f"{u} un  ({deg:+.1f}°)")
        alvo = self._slider_units()[joint]
        self._servo_set("alvo", f"{alvo} un "
                        f"({units_to_deg(joint, alvo):+.1f}°)")
        d = self._diag_of(joint)
        if d is None:
            for key in ("v", "temp", "load", "torque", "tl", "avisos"):
                self._servo_set(key, "—")
        else:
            self._servo_set("v", f"{d['v']:.1f} V")
            self._servo_set("temp", f"{d['temp']} °C",
                            warn=d["temp"] >= 60)
            self._servo_set("load", f"{d['load']:.0f} %",
                            warn=d["load"] >= 60)
            self._servo_set("torque", "ligado" if d["torque"] else "desligado")
            self._servo_set("tl", str(d.get("tl", "—")),
                            warn=d.get("tl") == 0)
            self._servo_set("avisos", ", ".join(d["avisos"]) or "nenhum",
                            warn=bool(d["avisos"]))
        if not self.active:
            self._servo_scale_sync = True
            self.servo_scale.set(self.scales[joint].get())
            self._servo_scale_sync = False

    def _servo_set(self, key, text, warn=False):
        var, lbl = self.servo_vars[key]
        var.set(text)
        lbl.configure(foreground=WARN if warn else INK)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    gui = WidowXGui(node)

    def spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass
        finally:
            # Ctrl+C no launch derruba o contexto ROS; fecha a janela junto
            try:
                gui.root.after(0, gui.root.quit)
            except Exception:
                pass

    threading.Thread(target=spin, daemon=True).start()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
