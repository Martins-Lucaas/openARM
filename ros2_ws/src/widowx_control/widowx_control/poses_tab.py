"""Aba "Poses" — teach pendant do WidowX.

Porte do teach pendant do `touch_pack` (repositório cr10twin, a aba
"Poses & Motions" do `palpation_gui.py`), com a mesma divisão de tela:
POSES à esquerda (captura, renomear, excluir) e MOVIMENTOS à direita
(sequência de poses, execução, laço).

Três diferenças em relação ao original, todas deliberadas:

* **WidowX no lugar do CR10.** As poses são as 6 juntas do braço em unidades
  brutas de servo, saneadas pelos limites do `armlink`. Não há mão COVVI, não
  há Gazebo e não há modo espelho: quem executa é sempre o driver ArmLink,
  pelos mesmos `/widowx/joint_targets` + `/widowx/dtime` que a aba de controle
  manual usa.
* **Tempo POR PASSO.** No original o movimento inteiro tinha um único
  `dur_s`. Aqui cada passo da sequência guarda quanto tempo o braço PERMANECE
  naquela pose antes de ir para a próxima (o percurso continua sendo um só,
  porque é ele que vira o `dtime` do ArbotiX).
* **Edição mais direta.** Duplo clique, atalhos de teclado, "ir para a pose",
  "atualizar pose com a posição atual" e o total da sequência sempre à vista
  — ver _SHORTCUTS_HINT.
"""

import logging
import math
import threading
import tkinter as tk
from tkinter import messagebox, ttk

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from . import armlink, poses_store
from .theme import (BG, CARD, INK, MUTED, ACCENT, OK, WARN, ALERT, LINE,
                    FIELD, FONT_HEAD, FONT_LBL, FONT_SMALL, FONT_MONO_S)

log = logging.getLogger("widowx_control.poses")

_SHORTCUTS_HINT = ("duplo clique numa pose adiciona à sequência  ·  "
                   "F2 renomeia  ·  Del exclui  ·  Ctrl+N captura  ·  "
                   "F5 executa  ·  Esc para")

# Período do laço que atualiza o destaque do passo em execução (ms).
_PROGRESS_MS = 120


class PosesTabMixin:
    """Espera do hospedeiro: `self.root`, `self.nb`, `self.node`,
    `self.active`, `self.scales`, `_set_sliders_to`, `_slider_units`,
    `_suppress_pending`, `flash` e `_on_speed`."""

    # ------------------------------------------------------------- ciclo
    def init_poses(self, path: str = "") -> None:
        self._poses_path = path or poses_store.default_path()
        data = poses_store.load(self._poses_path)
        self._poses = data["poses"]
        self._movements = data["movements"]
        self._poses_lbx = None
        self._movs_lbx = None
        self._seq_lbx = None
        self._mov_detail_outer = None
        self._mov_detail_inner = None
        self._current_mov_id = None
        # Execução
        self._exec_thread = None
        self._exec_stop = threading.Event()
        self._exec_mov_id = None
        self._exec_step = -1          # passo em curso (índice), -1 = parado
        self._exec_phase = ""         # "percurso" | "permanência"

    def stop_poses(self) -> None:
        self._exec_stop.set()

    # -------------------------------------------------------------- tela
    def _tab_poses(self) -> None:
        tab = tk.Frame(self.nb, bg=BG)
        self.nb.add(tab, text="  Poses  ")
        self._poses_tab_frame = tab

        # `pack` antes das colunas e com side="bottom": em `place` a dica
        # flutuava por cima dos botões da coluna das poses.
        tk.Label(tab, text=_SHORTCUTS_HINT, bg=BG, fg=MUTED,
                 font=FONT_SMALL).pack(side="bottom", fill="x", pady=(0, 4))

        left = tk.Frame(tab, bg=BG, width=310)
        left.pack(side="left", fill="y", padx=(12, 6), pady=12)
        left.pack_propagate(False)

        right = tk.Frame(tab, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(6, 12),
                   pady=12)

        self._build_poses_column(left)
        self._build_motions_column(right)

        self._bind_poses_shortcuts()
        self._refresh_poses_list()
        self._refresh_movements_list()
        self.root.after(_PROGRESS_MS, self._refresh_exec_progress)

    def _btn(self, parent, text, command, *, kind="neutral", **pack):
        colors = {"neutral": ("#e3e8ed", INK), "accent": (ACCENT, "white"),
                  "ok": (OK, "white"), "alert": (ALERT, "white"),
                  "danger": (WARN, "white")}
        bg, fg = colors[kind]
        btn = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                        activebackground=bg, activeforeground=fg,
                        font=FONT_SMALL, relief="flat", bd=0, padx=8, pady=4,
                        cursor="hand2", highlightthickness=0)
        btn.pack(**pack)
        return btn

    def _listbox(self, parent, **kw):
        return tk.Listbox(parent, bg=FIELD, fg=INK, font=FONT_MONO_S,
                          selectbackground=ACCENT, selectforeground="white",
                          relief="flat", bd=0, highlightthickness=1,
                          highlightbackground=LINE, activestyle="none", **kw)

    def _build_poses_column(self, left: tk.Frame) -> None:
        tk.Label(left, text="Poses", bg=BG, fg=INK,
                 font=FONT_HEAD).pack(anchor="w")
        tk.Frame(left, bg=LINE, height=1).pack(fill="x", pady=(4, 8))

        cap = tk.Frame(left, bg=BG)
        cap.pack(fill="x", pady=(0, 6))
        # Capturar do BRAÇO é o gesto do teach pendant: solta o torque
        # (botão Dormir), põe o braço na posição com a mão e captura. Com o
        # torque solto o driver publica a posição real em joint_states, que é
        # de onde isto lê.
        self._btn(cap, "◉ Capturar do braço", self._capture_pose_arm,
                  kind="accent", side="left", padx=(0, 4))
        self._btn(cap, "⌨ Dos sliders", self._capture_pose_sliders,
                  side="left")

        edit = tk.Frame(left, bg=BG)
        edit.pack(fill="x", pady=(0, 8))
        self._btn(edit, "⟳ Atualizar", self._update_selected_pose,
                  side="left", padx=(0, 4))
        self._btn(edit, "⧉ Duplicar", self._duplicate_selected_pose,
                  side="left", padx=(0, 4))
        self._btn(edit, "✏ Renomear", self._rename_selected_pose,
                  side="left")

        lbx_frame = tk.Frame(left, bg=BG)
        lbx_frame.pack(fill="both", expand=True)
        scroll = ttk.Scrollbar(lbx_frame, orient="vertical")
        scroll.pack(side="right", fill="y")
        self._poses_lbx = self._listbox(lbx_frame, yscrollcommand=scroll.set)
        self._poses_lbx.pack(side="left", fill="both", expand=True)
        scroll.config(command=self._poses_lbx.yview)
        # Duplo clique adiciona à sequência do movimento aberto: é o gesto
        # mais repetido ao montar um movimento e, no original, exigia
        # selecionar de um lado e clicar num botão do outro.
        self._poses_lbx.bind("<Double-Button-1>",
                             lambda e: self._add_selected_pose_to_seq())

        act = tk.Frame(left, bg=BG)
        act.pack(fill="x", pady=(8, 0))
        self._btn(act, "▶ Ir para a pose", self._goto_selected_pose,
                  kind="ok", side="left", padx=(0, 4))
        self._btn(act, "✖ Excluir", self._delete_selected_pose,
                  kind="danger", side="right")

    def _build_motions_column(self, right: tk.Frame) -> None:
        hdr = tk.Frame(right, bg=BG)
        hdr.pack(fill="x")
        tk.Label(hdr, text="Movimentos", bg=BG, fg=INK,
                 font=FONT_HEAD).pack(side="left", anchor="w")
        self._btn(hdr, "+ Novo", self._new_movement, kind="accent",
                  side="right")
        tk.Frame(right, bg=LINE, height=1).pack(fill="x", pady=(4, 8))

        lbx_frame = tk.Frame(right, bg=BG, height=110)
        lbx_frame.pack(fill="x")
        lbx_frame.pack_propagate(False)
        scroll = ttk.Scrollbar(lbx_frame, orient="vertical")
        scroll.pack(side="right", fill="y")
        self._movs_lbx = self._listbox(lbx_frame, yscrollcommand=scroll.set)
        self._movs_lbx.pack(side="left", fill="both", expand=True)
        scroll.config(command=self._movs_lbx.yview)
        self._movs_lbx.bind("<<ListboxSelect>>", self._on_movement_select)

        self._mov_detail_outer = tk.Frame(right, bg=BG)
        self._mov_detail_outer.pack(fill="both", expand=True, pady=(8, 0))

    # ------------------------------------------------------------ rótulos
    def _pose_label(self, pose: dict) -> str:
        units = pose["units"]
        # Só as quatro primeiras juntas, e sem a garra: com as seis o rótulo
        # passa da largura da coluna e o fim some (a Listbox do Tk corta, não
        # rola). O detalhe completo de cada pose está no JSON.
        parts = " ".join(
            f"{joint[:3].upper()}="
            f"{math.degrees(armlink.units_to_rad(joint, units[joint])):+.0f}°"
            for joint in armlink.JOINTS[:4])
        return f"{pose['name']}  [{parts}]"

    def _movement_label(self, mov: dict) -> str:
        total = poses_store.total_seconds(mov, self._poses)
        n = len(mov["steps"])
        return (f"{mov['name']}   ·  {n} passo{'s' if n != 1 else ''}  ·  "
                f"{poses_store.format_hms(total)}")

    def _step_label(self, index: int, step: dict) -> str:
        pose = poses_store.by_id(self._poses, step["pose_id"])
        name = pose["name"] if pose else f"[excluída:{step['pose_id']}]"
        return f"{index + 1:2d}. {name}   ·  {step['dwell_s']:g} s"

    # ------------------------------------------------------------ listas
    def _save(self) -> None:
        poses_store.save(self._poses_path,
                         {"poses": self._poses, "movements": self._movements})

    def _refresh_poses_list(self, select_index: int | None = None) -> None:
        lbx = self._poses_lbx
        if lbx is None:
            return
        keep = lbx.curselection()
        lbx.delete(0, "end")
        for pose in self._poses:
            lbx.insert("end", self._pose_label(pose))
        target = select_index if select_index is not None else (
            keep[0] if keep else None)
        if target is not None and 0 <= target < len(self._poses):
            lbx.selection_set(target)
            lbx.see(target)

    def _refresh_movements_list(self, select_id: int | None = None) -> None:
        lbx = self._movs_lbx
        if lbx is None:
            return
        lbx.delete(0, "end")
        for mov in self._movements:
            lbx.insert("end", self._movement_label(mov))
        want = select_id if select_id is not None else self._current_mov_id
        for i, mov in enumerate(self._movements):
            if mov["id"] == want:
                lbx.selection_set(i)
                lbx.see(i)
                break

    def _selected_pose(self) -> dict | None:
        lbx = self._poses_lbx
        sel = lbx.curselection() if lbx is not None else ()
        return self._poses[sel[0]] if sel else None

    def _current_movement(self) -> dict | None:
        return poses_store.by_id(self._movements, self._current_mov_id)

    def _on_movement_select(self, _event=None) -> None:
        sel = self._movs_lbx.curselection()
        if not sel:
            return
        mov = self._movements[sel[0]]
        if mov["id"] != self._current_mov_id:
            self._current_mov_id = mov["id"]
            self._refresh_movement_detail()

    # ----------------------------------------------------- detalhe do mov
    def _refresh_movement_detail(self) -> None:
        outer = self._mov_detail_outer
        if outer is None:
            return
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()
            self._mov_detail_inner = None
            self._seq_lbx = None
        mov = self._current_movement()
        if mov is None:
            return

        inner = tk.Frame(outer, bg=CARD, highlightthickness=1,
                         highlightbackground=LINE)
        inner.pack(fill="both", expand=True)
        self._mov_detail_inner = inner

        hdr = tk.Frame(inner, bg=CARD)
        hdr.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(hdr, text=mov["name"], bg=CARD, fg=INK,
                 font=FONT_HEAD).pack(side="left")
        self._btn(hdr, "✏", lambda: self._rename_movement(mov["id"]),
                  side="left", padx=(8, 0))
        self._btn(hdr, "✖ Excluir",
                  lambda: self._delete_movement(mov["id"]),
                  kind="danger", side="right")
        tk.Frame(inner, bg=LINE, height=1).pack(fill="x")

        body = tk.Frame(inner, bg=CARD)
        body.pack(fill="both", expand=True, padx=12, pady=8)

        # A coluna dos controles vem primeiro: o `pack` serve os slaves na
        # ordem em que os recebe e a lista da sequência pede a largura toda —
        # empacotada antes, ela deixava os botões de executar com zero pixel.
        self._build_run_column(body, mov)
        self._build_sequence_column(body, mov)
        self._refresh_sequence()

    def _build_sequence_column(self, body: tk.Frame, mov: dict) -> None:
        col = tk.Frame(body, bg=CARD)
        col.pack(side="left", fill="both", expand=True, padx=(0, 12))

        tk.Label(col, text="Sequência", bg=CARD, fg=MUTED,
                 font=FONT_SMALL).pack(anchor="w")

        frame = tk.Frame(col, bg=CARD)
        frame.pack(fill="both", expand=True, pady=(4, 0))
        scroll = ttk.Scrollbar(frame, orient="vertical")
        scroll.pack(side="right", fill="y")
        self._seq_lbx = self._listbox(frame, yscrollcommand=scroll.set,
                                      height=7)
        self._seq_lbx.pack(side="left", fill="both", expand=True)
        scroll.config(command=self._seq_lbx.yview)
        self._seq_lbx.bind("<<ListboxSelect>>", self._on_step_select)
        # Duplo clique num passo leva o braço até aquela pose — conferir o
        # ponto sem ter de achá-lo de novo na lista da esquerda.
        self._seq_lbx.bind("<Double-Button-1>", lambda e: self._goto_step())

        btns = tk.Frame(col, bg=CARD)
        btns.pack(fill="x", pady=(6, 0))
        self._btn(btns, "+ Adicionar", self._add_selected_pose_to_seq,
                  side="left", padx=(0, 4))
        self._btn(btns, "−", self._remove_step, side="left", padx=(0, 4))
        self._btn(btns, "↑", lambda: self._move_step(-1), side="left",
                  padx=(0, 4))
        self._btn(btns, "↓", lambda: self._move_step(+1), side="left")

    def _build_run_column(self, body: tk.Frame, mov: dict) -> None:
        col = tk.Frame(body, bg=CARD, width=210)
        col.pack(side="right", fill="y")
        col.pack_propagate(False)

        # ── Tempo de PERMANÊNCIA do passo selecionado ────────────────────
        tk.Label(col, text="Permanência na pose (s)", bg=CARD, fg=INK,
                 font=FONT_LBL).pack(anchor="w")
        tk.Label(col, text="quanto o braço FICA parado neste passo",
                 bg=CARD, fg=MUTED, font=FONT_SMALL, wraplength=195,
                 justify="left").pack(anchor="w")
        dwell_row = tk.Frame(col, bg=CARD)
        dwell_row.pack(fill="x", pady=(2, 2))
        self._dwell_var = tk.StringVar(value="—")
        self._dwell_spin = tk.Spinbox(
            dwell_row, from_=poses_store.DWELL_MIN, to=poses_store.DWELL_MAX,
            increment=0.5, textvariable=self._dwell_var, width=8,
            font=FONT_MONO_S, relief="flat", bd=1, state="disabled",
            command=self._on_dwell_changed)
        self._dwell_spin.pack(side="left")
        self._dwell_spin.bind("<Return>", lambda e: self._on_dwell_changed())
        self._dwell_spin.bind("<FocusOut>", lambda e: self._on_dwell_changed())
        self._btn(dwell_row, "→ todos", self._apply_dwell_to_all,
                  side="left", padx=(6, 0))

        # ── Tempo de PERCURSO (do movimento) ─────────────────────────────
        tk.Label(col, text="Percurso entre poses (ms)", bg=CARD, fg=INK,
                 font=FONT_LBL).pack(anchor="w", pady=(10, 0))
        tk.Label(col, text="vira o dtime do ArbotiX",
                 bg=CARD, fg=MUTED, font=FONT_SMALL, wraplength=195,
                 justify="left").pack(anchor="w")
        self._move_var = tk.StringVar(value=str(mov["move_ms"]))
        move_spin = tk.Spinbox(
            col, from_=poses_store.MOVE_MS_MIN, to=poses_store.MOVE_MS_MAX,
            increment=40, textvariable=self._move_var, width=8,
            font=FONT_MONO_S, relief="flat", bd=1,
            command=self._on_move_ms_changed)
        move_spin.pack(anchor="w", pady=(2, 2))
        move_spin.bind("<Return>", lambda e: self._on_move_ms_changed())
        move_spin.bind("<FocusOut>", lambda e: self._on_move_ms_changed())

        self._total_lbl = tk.Label(col, text="", bg=CARD, fg=MUTED,
                                   font=FONT_SMALL, justify="left")
        self._total_lbl.pack(anchor="w", pady=(8, 8))

        self._btn(col, "▶ Executar (F5)",
                  lambda: self._start_movement(mov["id"], loop=False),
                  kind="ok", fill="x", pady=(0, 4))
        self._btn(col, "↻ Em laço",
                  lambda: self._start_movement(mov["id"], loop=True),
                  kind="alert", fill="x", pady=(0, 4))
        self._btn(col, "■ Parar (Esc)", self._stop_execution,
                  kind="danger", fill="x")

        self._exec_lbl = tk.Label(col, text="", bg=CARD, fg=MUTED,
                                  font=FONT_SMALL, justify="left",
                                  wraplength=195)
        self._exec_lbl.pack(anchor="w", pady=(8, 0))

    def _refresh_sequence(self, select_index: int | None = None) -> None:
        lbx = self._seq_lbx
        mov = self._current_movement()
        if lbx is None or mov is None:
            return
        keep = lbx.curselection()
        lbx.delete(0, "end")
        for i, step in enumerate(mov["steps"]):
            lbx.insert("end", self._step_label(i, step))
        target = select_index if select_index is not None else (
            keep[0] if keep else None)
        if target is not None and 0 <= target < len(mov["steps"]):
            lbx.selection_set(target)
            lbx.see(target)
        self._refresh_totals()
        self._on_step_select()

    def _refresh_totals(self) -> None:
        mov = self._current_movement()
        if mov is None or not hasattr(self, "_total_lbl"):
            return
        total = poses_store.total_seconds(mov, self._poses)
        n = len(mov["steps"])
        self._total_lbl.config(
            text=f"Total da sequência: {poses_store.format_hms(total)}\n"
                 f"({n} passo{'s' if n != 1 else ''}, percurso + permanência)")
        self._refresh_movements_list()

    def _selected_step_index(self) -> int | None:
        lbx = self._seq_lbx
        sel = lbx.curselection() if lbx is not None else ()
        return sel[0] if sel else None

    def _on_step_select(self, _event=None) -> None:
        """O spinbox de permanência edita o passo SELECIONADO."""
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None or not (0 <= idx < len(mov["steps"])):
            self._dwell_var.set("—")
            self._dwell_spin.config(state="disabled")
            return
        self._dwell_spin.config(state="normal")
        self._dwell_var.set(f"{mov['steps'][idx]['dwell_s']:g}")

    def _on_dwell_changed(self) -> None:
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None or not (0 <= idx < len(mov["steps"])):
            return
        try:
            value = poses_store.clamp_dwell(float(self._dwell_var.get()))
        except (ValueError, tk.TclError):
            return                       # campo a meio de digitação
        if value == mov["steps"][idx]["dwell_s"]:
            return
        mov["steps"][idx]["dwell_s"] = value
        self._save()
        self._refresh_sequence(select_index=idx)

    def _apply_dwell_to_all(self) -> None:
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None:
            self.flash("Selecione um passo para copiar o tempo.", ALERT)
            return
        value = mov["steps"][idx]["dwell_s"]
        for step in mov["steps"]:
            step["dwell_s"] = value
        self._save()
        self._refresh_sequence(select_index=idx)
        self.flash(f"Permanência de {value:g} s aplicada a todos os passos.",
                   OK)

    def _on_move_ms_changed(self) -> None:
        mov = self._current_movement()
        if mov is None:
            return
        try:
            value = poses_store.clamp_move_ms(float(self._move_var.get()))
        except (ValueError, tk.TclError):
            return
        if value == mov["move_ms"]:
            return
        mov["move_ms"] = value
        self._save()
        self._refresh_totals()

    # ------------------------------------------------------ captura/edição
    def _arm_units(self) -> dict | None:
        """Posição REAL do braço em unidades, ou None se o driver ainda não
        publicou joint_states."""
        states = dict(self.node.joint_states)
        if not states:
            return None
        return {joint: armlink.rad_to_units(joint, states[joint])
                for joint in armlink.JOINTS if joint in states}

    def _capture_pose_arm(self) -> None:
        units = self._arm_units()
        if units is None or len(units) < len(armlink.JOINTS):
            self.flash("Sem leitura de /widowx/joint_states — use "
                       "\"Dos sliders\".", ALERT)
            return
        self._add_pose(units, prefix="Braço")

    def _capture_pose_sliders(self) -> None:
        self._add_pose(self._slider_units(), prefix="Pose")

    def _add_pose(self, units: dict, prefix: str = "Pose") -> None:
        pid = poses_store.next_id(self._poses)
        pose = {"id": pid, "name": f"{prefix} {pid}",
                "units": poses_store.clamp_units(units)}
        self._poses.append(pose)
        self._save()
        self._refresh_poses_list(select_index=len(self._poses) - 1)
        self.flash(f'Pose "{pose["name"]}" capturada.', OK)

    def _update_selected_pose(self) -> None:
        """Regrava a pose selecionada com a posição atual — o gesto de
        "quase certo, deixa eu corrigir" sem ter de excluir e recapturar
        (e sem perder o lugar dela nas sequências, que é o que a exclusão
        custava no original)."""
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose para atualizar.", ALERT)
            return
        units = self._arm_units() or self._slider_units()
        pose["units"] = poses_store.clamp_units(units)
        self._save()
        idx = self._poses.index(pose)
        self._refresh_poses_list(select_index=idx)
        self._refresh_sequence()
        self.flash(f'Pose "{pose["name"]}" atualizada com a posição atual.',
                   OK)

    def _duplicate_selected_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose para duplicar.", ALERT)
            return
        pid = poses_store.next_id(self._poses)
        copy = {"id": pid, "name": f"{pose['name']} (cópia)",
                "units": dict(pose["units"])}
        self._poses.insert(self._poses.index(pose) + 1, copy)
        self._save()
        self._refresh_poses_list(select_index=self._poses.index(copy))
        self.flash(f'Pose "{copy["name"]}" criada.', OK)

    def _rename_selected_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose para renomear.", ALERT)
            return
        name = self._ask_name("Renomear pose", pose["name"])
        if not name:
            return
        pose["name"] = name
        self._save()
        self._refresh_poses_list(select_index=self._poses.index(pose))
        self._refresh_sequence()

    def _delete_selected_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose para excluir.", ALERT)
            return
        used = sum(1 for m in self._movements
                   for s in m["steps"] if s["pose_id"] == pose["id"])
        if used and not messagebox.askokcancel(
                "Excluir pose",
                f'A pose "{pose["name"]}" está em {used} passo(s) de '
                "movimentos. Excluir remove esses passos também."):
            return
        for mov in self._movements:
            mov["steps"] = [s for s in mov["steps"]
                            if s["pose_id"] != pose["id"]]
        self._poses.remove(pose)
        self._save()
        self._refresh_poses_list()
        self._refresh_sequence()
        self.flash(f'Pose "{pose["name"]}" excluída.', OK)

    def _goto_selected_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose.", ALERT)
            return
        self._goto_units(pose["units"], pose["name"])

    def _goto_step(self) -> None:
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None:
            return
        pose = poses_store.by_id(self._poses, mov["steps"][idx]["pose_id"])
        if pose is not None:
            self._goto_units(pose["units"], pose["name"])

    def _goto_units(self, units: dict, name: str) -> None:
        if not self.active:
            self.flash("Braço inativo — clique em Ativar antes de mover.",
                       ALERT)
            return
        if self._exec_running():
            self.flash("Execução em andamento — pare antes.", ALERT)
            return
        self._apply_units_to_sliders(units)
        self._publish_units(units)
        self.flash(f'Indo para "{name}".', OK)

    # -------------------------------------------------------- movimentos
    def _new_movement(self) -> None:
        mid = poses_store.next_id(self._movements)
        name = self._ask_name("Novo movimento", f"Movimento {mid}")
        if not name:
            return
        mov = {"id": mid, "name": name,
               "move_ms": poses_store.MOVE_MS_DEFAULT, "steps": []}
        self._movements.append(mov)
        self._current_mov_id = mid
        self._save()
        self._refresh_movements_list(select_id=mid)
        self._refresh_movement_detail()

    def _rename_movement(self, mov_id: int) -> None:
        mov = poses_store.by_id(self._movements, mov_id)
        if mov is None:
            return
        name = self._ask_name("Renomear movimento", mov["name"])
        if not name:
            return
        mov["name"] = name
        self._save()
        self._refresh_movements_list(select_id=mov_id)
        self._refresh_movement_detail()

    def _delete_movement(self, mov_id: int) -> None:
        mov = poses_store.by_id(self._movements, mov_id)
        if mov is None:
            return
        if not messagebox.askokcancel(
                "Excluir movimento", f'Excluir "{mov["name"]}"?'):
            return
        self._movements.remove(mov)
        if self._current_mov_id == mov_id:
            self._current_mov_id = None
        self._save()
        self._refresh_movements_list()
        if self._mov_detail_inner is not None:
            self._mov_detail_inner.destroy()
            self._mov_detail_inner = None
            self._seq_lbx = None
        self.flash(f'Movimento "{mov["name"]}" excluído.', OK)

    def _add_selected_pose_to_seq(self) -> None:
        mov = self._current_movement()
        if mov is None:
            self.flash("Selecione (ou crie) um movimento à direita.", ALERT)
            return
        pose = self._selected_pose()
        if pose is None:
            self.flash("Selecione uma pose na lista da esquerda.", ALERT)
            return
        # Herda o tempo do último passo: numa sequência de sete poses com a
        # mesma permanência, digitar o valor uma vez basta.
        dwell = (mov["steps"][-1]["dwell_s"] if mov["steps"]
                 else poses_store.DWELL_DEFAULT)
        mov["steps"].append({"pose_id": pose["id"], "dwell_s": dwell})
        self._save()
        self._refresh_sequence(select_index=len(mov["steps"]) - 1)
        self.flash(f'"{pose["name"]}" adicionada à sequência '
                   f'({dwell:g} s de permanência).', OK)

    def _remove_step(self) -> None:
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None:
            return
        del mov["steps"][idx]
        self._save()
        self._refresh_sequence(select_index=min(idx, len(mov["steps"]) - 1))

    def _move_step(self, delta: int) -> None:
        mov = self._current_movement()
        idx = self._selected_step_index()
        if mov is None or idx is None:
            return
        new = idx + delta
        if not (0 <= new < len(mov["steps"])):
            return
        mov["steps"][idx], mov["steps"][new] = \
            mov["steps"][new], mov["steps"][idx]
        self._save()
        self._refresh_sequence(select_index=new)

    # ---------------------------------------------------------- execução
    def _exec_running(self) -> bool:
        return self._exec_thread is not None and self._exec_thread.is_alive()

    def _start_movement(self, mov_id: int, loop: bool = False) -> None:
        if self._exec_running():
            self.flash("Execução em andamento — pare antes.", ALERT)
            return
        mov = poses_store.by_id(self._movements, mov_id)
        if mov is None:
            return
        if not mov["steps"]:
            self.flash("Adicione poses à sequência antes de executar.", ALERT)
            return
        if not self.active:
            self.flash("Braço inativo — clique em Ativar antes de executar.",
                       ALERT)
            return
        # Os sliders NAO sao desabilitados aqui, ainda que um esbarrao neles
        # durante a sequência seja indesejável: `set()` numa ttk.Scale
        # desabilitada é ignorado em silêncio (ver _scale_set no gui_node), e
        # é por eles que a pose em curso aparece na tela e no desenho 2D do
        # braço. Quem impede o esbarrão de virar movimento é o laço da GUI,
        # que não publica alvo nenhum enquanto _exec_running().
        self._exec_stop.clear()
        self._exec_mov_id = mov_id
        # Cópia profunda dos passos: a thread não pode ler a lista que a GUI
        # edita enquanto o movimento roda.
        snapshot = {
            "name": mov["name"],
            "move_ms": mov["move_ms"],
            "steps": [dict(s) for s in mov["steps"]],
        }
        self._exec_thread = threading.Thread(
            target=self._execute_worker, args=(snapshot, loop),
            daemon=True, name="widowx-poses")
        self._exec_thread.start()
        self.flash(f'Executando "{mov["name"]}"'
                   f'{"  (laço)" if loop else ""}...', OK)

    def _stop_execution(self) -> None:
        if not self._exec_running():
            return
        self._exec_stop.set()
        self.flash("Execução interrompida.", ALERT)

    def _execute_worker(self, mov: dict, loop: bool) -> None:
        try:
            self._run_once(mov)
            while loop and not self._exec_stop.is_set():
                self._run_once(mov)
        except Exception as exc:
            log.warning("execução de movimento falhou: %s", exc)
            self.root.after(0, lambda e=str(exc): self.flash(
                f"Execução falhou: {e}", WARN))
        finally:
            self._exec_step = -1
            self._exec_phase = ""
            self._exec_mov_id = None
            self.root.after(0, self._on_exec_finished)

    def _run_once(self, mov: dict) -> None:
        """Uma passagem pela sequência.

        Cada passo é: publica o alvo, espera o PERCURSO, espera a
        PERMANÊNCIA. O driver espaça os envios pelo tempo de interpolação
        vigente (o firmware do ArbotiX descarta pacotes que chegam durante um
        movimento), e como esperamos o percurso inteiro antes do passo
        seguinte, o envio nunca cai dentro da janela de descarte."""
        move_s = poses_store.clamp_move_ms(mov["move_ms"]) / 1000.0
        dtime = poses_store.move_ms_to_dtime(mov["move_ms"])
        self.node.pub_dtime.publish(Int32(data=dtime))
        for index, step in enumerate(mov["steps"]):
            if self._exec_stop.is_set():
                return
            pose = poses_store.by_id(self._poses, step["pose_id"])
            if pose is None:
                continue          # pose excluída no meio: pula o passo
            units = pose["units"]
            self._exec_step = index
            self._exec_phase = "percurso"
            self.root.after(0, self._apply_units_to_sliders, dict(units))
            self._publish_units(units)
            if self._exec_stop.wait(move_s):
                return
            self._exec_phase = "permanência"
            if self._exec_stop.wait(poses_store.clamp_dwell(step["dwell_s"])):
                return

    def _publish_units(self, units: dict) -> None:
        msg = JointState()
        msg.name = list(armlink.JOINTS)
        msg.position = [float(armlink.clamp(joint, units[joint]))
                        for joint in armlink.JOINTS]
        self.node.pub_targets.publish(msg)

    def _apply_units_to_sliders(self, units: dict) -> None:
        """Move os sliders junto (a aba de controle manual e o desenho do
        braço leem deles). `_suppress_pending` evita que o próprio `set()`
        marque um alvo novo para o laço da GUI republicar."""
        self._suppress_pending = True
        try:
            self._set_sliders_to(units)
        finally:
            self._suppress_pending = False

    def _on_exec_finished(self) -> None:
        if hasattr(self, "_exec_lbl"):
            self._exec_lbl.config(text="")
        self._refresh_sequence()
        # O dtime volta ao valor do slider da aba manual: sem isto o próximo
        # movimento manual herda a cadência do movimento executado.
        self._on_speed()

    def _refresh_exec_progress(self) -> None:
        """Destaca o passo em curso e diz em que fase ele está."""
        try:
            if not hasattr(self, "_exec_lbl") or self._seq_lbx is None:
                return
            running = self._exec_running()
            same_mov = (self._exec_mov_id is not None
                        and self._exec_mov_id == self._current_mov_id)
            step = self._exec_step
            self._seq_lbx.selection_clear(0, "end")
            if running and same_mov and step >= 0:
                if step < self._seq_lbx.size():
                    self._seq_lbx.selection_set(step)
                    self._seq_lbx.see(step)
                total = self._seq_lbx.size()
                self._exec_lbl.config(
                    text=f"passo {step + 1}/{total} · {self._exec_phase}",
                    fg=OK)
            elif not running:
                self._exec_lbl.config(text="", fg=MUTED)
        finally:
            self.root.after(_PROGRESS_MS, self._refresh_exec_progress)

    # ----------------------------------------------------------- atalhos
    def _bind_poses_shortcuts(self) -> None:
        """Atalhos só valem com a aba Poses à vista: F2 e Del ligados à
        janela inteira roubariam teclas das outras abas."""
        bindings = {
            "<F2>": self._rename_selected_pose,
            "<Delete>": self._delete_selected_pose,
            "<Control-n>": self._capture_pose_arm,
            "<Control-d>": self._duplicate_selected_pose,
            "<Control-Return>": self._goto_selected_pose,
            "<F5>": lambda: self._start_movement(self._current_mov_id or -1),
            "<Escape>": self._stop_execution,
            "<Alt-Up>": lambda: self._move_step(-1),
            "<Alt-Down>": lambda: self._move_step(+1),
        }
        for sequence, action in bindings.items():
            self.root.bind(sequence, self._when_poses_visible(action))

    def _when_poses_visible(self, action):
        def handler(_event=None):
            frame = getattr(self, "_poses_tab_frame", None)
            try:
                visible = (frame is not None
                           and str(self.nb.select()) == str(frame))
            except Exception:
                visible = False
            if visible:
                action()
            return None
        return handler

    # ----------------------------------------------------------- diálogo
    def _ask_name(self, title: str, initial: str = "") -> str | None:
        result = [None]
        dlg = tk.Toplevel(self.root)
        dlg.title(title)
        dlg.configure(bg=BG)
        dlg.resizable(False, False)
        dlg.transient(self.root)
        dlg.grab_set()

        tk.Label(dlg, text=title, bg=BG, fg=INK,
                 font=FONT_HEAD).pack(padx=24, pady=(16, 8))
        var = tk.StringVar(value=initial)
        entry = tk.Entry(dlg, textvariable=var, font=FONT_LBL, width=32)
        entry.pack(padx=24, pady=(0, 8))
        entry.select_range(0, "end")
        entry.focus_set()

        def ok(_=None):
            value = var.get().strip()
            if value:
                result[0] = value
            dlg.destroy()

        row = tk.Frame(dlg, bg=BG)
        row.pack(pady=(0, 16))
        self._btn(row, "OK", ok, kind="accent", side="left", padx=4)
        self._btn(row, "Cancelar", dlg.destroy, side="left", padx=4)
        entry.bind("<Return>", ok)
        entry.bind("<Escape>", lambda e: dlg.destroy())
        dlg.wait_window()
        return result[0]
