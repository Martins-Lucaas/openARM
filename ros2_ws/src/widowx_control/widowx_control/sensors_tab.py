"""Aba "Sensores" — porte da aba homônima do `touch_pack` (repositório
cr10twin, `palpation_gui.py`).

Os quatro gráficos do Izhikevich (heatmap de tensão, raster RA/SA, I_final e
o raster cuneiforme/neurônio pós) ocupam a aba INTEIRA. A lógica de
atualização é a da original: a figura é desenhada por uma FuncAnimation com
blit, pausada quando a aba não está à vista, e um laço `after` de 80 ms cuida
do cabeçalho.

Duas diferenças em relação ao original:

* a paleta é a clara desta GUI (ver theme.py), não a escura do cr10twin;
* **não há painel de célula de carga.** No cr10twin um card de 270 px ao lado
  mostra a força ao vivo com sparkline; o WidowX não tem célula, e o espaço
  vale mais para os gráficos (decidido em 27/08/2026). O escalar I_final e o
  estado da fonte ficaram na faixa do cabeçalho. Se a célula entrar aqui um
  dia, o painel inteiro está em `_update_sensors_panel`, no
  `palpation_gui.py` do outro repositório.
"""

import logging
import threading
import time
import tkinter as tk

from .theme import (BG, CARD, INK, MUTED, OK, DANGER, ALERT, LINE,
                    FONT_HEAD, FONT_LBL, FONT_SMALL, FONT_MONO)

try:
    from .touch_source import TouchSensorSource, TouchFigure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.animation import FuncAnimation
    _TOUCH_PLOT_OK = True
except Exception:  # pragma: no cover - matplotlib/pyserial ausentes
    TouchSensorSource = None
    TouchFigure = None
    FigureCanvasTkAgg = None
    FuncAnimation = None
    _TOUCH_PLOT_OK = False

log = logging.getLogger("widowx_control.sensors")

# Ritmo da animação do touch sensor — ver _retune_touch_anim. O período do
# desenho é custo_medido/_TOUCH_ANIM_DUTY, limitado a esta faixa: nunca mais
# que 30 fps (piso de 33 ms) nem menos que 4 fps (teto de 250 ms).
_TOUCH_ANIM_MIN_MS = 33
_TOUCH_ANIM_MAX_MS = 250
_TOUCH_ANIM_DUTY = 0.40

# Período do laço de números da aba (ms).
_SENSORS_PERIOD_MS = 80


class SensorsTabMixin:
    """Aba Sensores. Espera do hospedeiro: `self.root` (Tk), `self.nb`
    (ttk.Notebook) e `self.node` (o nó ROS com as leituras da célula)."""

    # ------------------------------------------------------------- ciclo
    def init_sensors(self, port: str = "", grid: int = 5) -> None:
        """Cria a fonte serial ANTES de montar a aba (a figura é construída a
        partir dela). Não abre a porta ainda — ver start_sensors()."""
        self._touch_source = None
        self._touch_figure = None
        self._touch_canvas = None
        self._touch_anim = None
        self._touch_anim_running = False
        self._sensors_tab_frame = None
        self._sensors_after = None
        # Painel destacável: `_sensors_panel` é o container do que se vê
        # (cabeçalho + figura) e `_sensors_window` é o Toplevel quando ele
        # está flutuando — ver _toggle_sensors_window.
        self._sensors_panel = None
        self._sensors_window = None
        # Medição do custo do frame da figura (s) — alimenta _retune_touch_anim.
        self._touch_frame_t0 = 0.0
        self._touch_frame_cost = None
        self._touch_frame_ema = None
        # Escalar do toque vindo da própria serial (o callback roda na thread
        # do leitor, daí o lock).
        self._touch_lock = threading.Lock()
        self._touch_value = 0.0
        self._touch_last_ts = 0.0

        # grid=4 → 4×4 com linha TOTAL (neurônio pós); grid=5 → 5×5 sem TOTAL
        # (o escalar é a média das tensões e o raster é o cuneiforme). Mesma
        # convenção do parâmetro `sensor` do launch do touch_pack.
        if int(grid) == 4:
            self._touch_rows, self._touch_cols, self._touch_has_total = 4, 4, True
        else:
            self._touch_rows, self._touch_cols, self._touch_has_total = 5, 5, False

        if _TOUCH_PLOT_OK:
            self._touch_source = TouchSensorSource(
                port=(port or None),
                on_sample=self._on_touch_sample,
                rows=self._touch_rows, cols=self._touch_cols,
                has_total=self._touch_has_total)

    def start_sensors(self) -> None:
        """Abre a serial do STM32 e começa o laço da aba. Chamado depois de a
        janela existir: sem o STM32 a aba ainda diz, em vermelho, que não há
        sensor — que é a informação que importa quando o cabo caiu."""
        src = self._touch_source
        if src is not None and not src.start():
            log.warning("[TOUCH] fonte serial não iniciou: %s", src.error)
        self._sensors_after = self.root.after(200, self._refresh_sensors_tab)

    def stop_sensors(self) -> None:
        if self._sensors_window is not None:
            try:
                self._sensors_window.destroy()
            except Exception:
                pass
            self._sensors_window = None
        if self._sensors_after is not None:
            try:
                self.root.after_cancel(self._sensors_after)
            except Exception:
                pass
            self._sensors_after = None
        if self._touch_source is not None:
            self._touch_source.stop()

    def _on_touch_sample(self, i_final: float) -> None:
        """Callback da fonte serial (thread do leitor)."""
        with self._touch_lock:
            self._touch_value = float(i_final)
            self._touch_last_ts = time.time()

    # -------------------------------------------------------------- tela
    def _tab_sensors(self) -> None:
        """A aba é só o HOSPEDEIRO do painel — ele também sabe morar numa
        janela própria (ver _toggle_sensors_window)."""
        tab = tk.Frame(self.nb, bg=BG)
        self.nb.add(tab, text="  Sensores  ")
        self._sensors_tab_frame = tab
        self._build_sensors_panel(tab)

    def _build_sensors_panel(self, parent) -> None:
        """Cabeçalho (origem do dado + escalar + botão de destacar) e a
        figura. Chamado tanto com a aba quanto com o Toplevel como pai."""
        destacado = self._sensors_window is not None
        panel = tk.Frame(parent, bg=BG)
        panel.pack(fill="both", expand=True)
        self._sensors_panel = panel

        hdr = tk.Frame(panel, bg=BG)
        hdr.pack(fill="x", padx=10, pady=(8, 0))
        tk.Label(hdr, text="Touch Sensor (STM32) — Izhikevich",
                 font=FONT_HEAD, bg=BG, fg=INK).pack(side="left")
        tk.Button(hdr,
                  text="⤢ Reacoplar" if destacado else "⧉ Destacar",
                  command=self._toggle_sensors_window,
                  bg="#e3e8ed", fg=INK, activebackground="#d3dbe2",
                  font=FONT_SMALL, relief="flat", bd=0, padx=8, pady=2,
                  cursor="hand2", highlightthickness=0).pack(side="left",
                                                             padx=(12, 0))
        self._sens_touch_status_lbl = tk.Label(
            hdr, text="", font=FONT_SMALL, bg=BG, fg=MUTED)
        self._sens_touch_status_lbl.pack(side="right")
        self._sens_touch_lbl = tk.Label(
            hdr, text="I_final  —", font=FONT_MONO, bg=BG, fg=INK)
        self._sens_touch_lbl.pack(side="right", padx=(0, 18))

        plot_holder = tk.Frame(panel, bg=CARD, highlightthickness=1,
                               highlightbackground=LINE)
        plot_holder.pack(fill="both", expand=True, padx=10, pady=(6, 10))
        self._build_touch_figure(plot_holder)

    # ------------------------------------------------- painel destacável
    def _toggle_sensors_window(self) -> None:
        """Manda o painel para uma janela própria, e de volta.

        É o que permite olhar o toque AO MESMO TEMPO que o teach pendant: um
        notebook mostra uma aba de cada vez, e o dado tátil só é útil junto
        com a pose que o produziu.

        O painel é DESTRUÍDO e reconstruído no novo pai em vez de
        reparentado: o Tk não reparenteia widgets, e a figura do matplotlib
        vive dentro de um canvas que é filho do painel. A FONTE serial não é
        tocada — ela continua lendo o STM32 durante a troca, então nenhum
        quadro se perde."""
        if self._sensors_window is None:
            self._teardown_sensors_panel()
            win = tk.Toplevel(self.root)
            win.title("Sensores — Touch Sensor (STM32)")
            win.configure(bg=BG)
            # Deslocada da janela principal: nascendo exatamente por cima,
            # a primeira impressão é de que a aba abriu um modal, não de que
            # agora há duas janelas para pôr lado a lado.
            win.geometry(f"900x680+{self.root.winfo_rootx() + 60}"
                         f"+{self.root.winfo_rooty() + 60}")
            win.protocol("WM_DELETE_WINDOW", self._toggle_sensors_window)
            self._sensors_window = win
            self._build_sensors_panel(win)
            # Marcador na aba vazia: sem isto ela fica em branco e parece
            # que a aba quebrou.
            self._sensors_placeholder = tk.Label(
                self._sensors_tab_frame,
                text="Os gráficos do toque estão numa janela separada.\n"
                     "Feche-a (ou use ⤢ Reacoplar) para trazê-los de volta.",
                bg=BG, fg=MUTED, font=FONT_LBL, justify="center")
            self._sensors_placeholder.pack(expand=True)
        else:
            self._teardown_sensors_panel()
            win, self._sensors_window = self._sensors_window, None
            win.destroy()
            ph = getattr(self, "_sensors_placeholder", None)
            if ph is not None:
                ph.destroy()
                self._sensors_placeholder = None
            self._build_sensors_panel(self._sensors_tab_frame)

    def _teardown_sensors_panel(self) -> None:
        """Desmonta a figura ANTES de destruir os widgets: a FuncAnimation
        guarda um timer do Tk apontando para o canvas, e destruir o canvas
        com o timer vivo levanta TclError no próximo disparo."""
        anim, self._touch_anim = self._touch_anim, None
        if anim is not None:
            try:
                anim.event_source.stop()
            except Exception:
                pass
        self._touch_anim_running = False
        self._touch_canvas = None
        self._touch_figure = None
        # O custo do frame muda com o tamanho novo: recomeça a medição em vez
        # de arrastar a média do tamanho anterior.
        self._touch_frame_ema = None
        self._touch_frame_cost = None
        self._touch_frame_t0 = 0.0
        panel, self._sensors_panel = self._sensors_panel, None
        if panel is not None:
            panel.destroy()

    def _build_touch_figure(self, holder: tk.Frame) -> None:
        if not (_TOUCH_PLOT_OK and self._touch_source is not None):
            tk.Label(holder,
                     text="matplotlib/pyserial ausentes — instale-os para "
                          "ver os gráficos do toque.",
                     font=FONT_LBL, bg=CARD, fg=MUTED).pack(expand=True,
                                                            pady=40)
            return
        try:
            self._touch_figure = TouchFigure(self._touch_source,
                                             facecolor=CARD)
            # O tamanho pedido pelo widget do canvas é o da figura, e o padrão
            # (9,5×7,0 in @100 dpi = 950×700 px) faria a janela nascer maior
            # que a tela em monitores modestos. Com a aba inteira para si a
            # figura pode pedir bem mais que antes; assim que o `pack` a
            # acomoda, ela passa a seguir o tamanho real do painel.
            self._touch_figure.fig.set_size_inches(8.0, 5.6)
            # O tight_layout do TouchFigure foi calculado no tamanho antigo;
            # sem refazê-lo a cada draw, o título do heatmap e os rótulos dos
            # eixos entram por cima da colorbar quando o painel encolhe. Só
            # roda em draw() cheio (abertura e redimensionamento) — o caminho
            # do blit não passa por aqui.
            self._touch_figure.fig.set_tight_layout(True)
            self._touch_canvas = FigureCanvasTkAgg(self._touch_figure.fig,
                                                   master=holder)
            self._touch_canvas.get_tk_widget().pack(fill="both", expand=True)
            self._touch_canvas.draw()
            # blit=True: só os artistas animados são redesenhados. O redraw
            # completo custa ~80 ms por frame nesta figura (4 eixos + colorbar
            # + legenda); pedido dezenas de vezes por segundo, satura o laço do
            # Tk e a GUI INTEIRA trava. Só é válido porque os limites dos eixos
            # são fixos: o raster usa tempo relativo a agora, não absoluto.
            #
            # O intervalo abaixo é só o CHUTE INICIAL — a partir do primeiro
            # frame quem manda é o custo medido, em _retune_touch_anim.
            self._touch_anim = FuncAnimation(
                self._touch_figure.fig,
                self._touch_anim_frame,
                init_func=self._touch_figure.init_blit,
                interval=_TOUCH_ANIM_MIN_MS, blit=True,
                cache_frame_data=False)
            self._touch_anim_running = True
            self._instrument_touch_blit()
        except Exception as exc:
            log.warning("[TOUCH] falha ao embutir figura: %s", exc)
            self._touch_figure = None
            self._touch_canvas = None
            self._touch_anim = None
            tk.Label(holder, text=f"Figure unavailable: {exc}",
                     font=FONT_LBL, bg=CARD, fg=MUTED).pack(expand=True,
                                                            pady=40)

    # -------------------------------------------------------------- laço
    def _refresh_sensors_tab(self) -> None:
        """Laço da aba. A figura é desenhada pela FuncAnimation (blit); aqui
        só a pausamos/retomamos conforme a aba esteja visível (poupa CPU) e
        atualizamos os números da célula."""
        try:
            if self._sensors_panel is None:
                return          # painel a meio de uma troca de janela
            visible = self._sensors_visible()
            self._set_touch_anim(visible)
            if visible:
                # Daqui, e não de dentro do callback da animação: ver a
                # advertência sobre o timer em _retune_touch_anim.
                self._retune_touch_anim()
                self._update_sensors_panel()
        finally:
            self._sensors_after = self.root.after(_SENSORS_PERIOD_MS,
                                                  self._refresh_sensors_tab)

    def _sensors_visible(self) -> bool:
        """Destacado, o painel anima INDEPENDENTE da aba selecionada — é
        justamente para ser olhado enquanto o teach pendant está à frente.
        Minimizado, não: aí ninguém o está vendo."""
        if self._sensors_window is not None:
            try:
                return self._sensors_window.state() != "iconic"
            except tk.TclError:
                return False
        frame = self._sensors_tab_frame
        if frame is None:
            return False
        try:
            return str(self.nb.select()) == str(frame)
        except Exception:
            return False

    def _set_touch_anim(self, run: bool) -> None:
        """Liga/desliga a animação do touch sensor (idempotente)."""
        anim = self._touch_anim
        if anim is None or run == self._touch_anim_running:
            return
        try:
            if run:
                anim.resume()
            else:
                anim.pause()
            self._touch_anim_running = run
            # O frame em curso não fecha atravessando uma pausa: sem isto o
            # próximo blit mediria também o tempo com a aba escondida.
            self._touch_frame_t0 = 0.0
            self._touch_frame_cost = None
        except Exception as exc:
            log.debug("touch anim toggle falhou: %s", exc)

    def _instrument_touch_blit(self) -> None:
        """Cronometra o frame REAL da figura: do início do callback da
        FuncAnimation até o último blit entregue ao Tk — o custo do desenho,
        não só o de mexer nos dados (que é ~0,5 ms; a rasterização é o resto).

        Com blit=True o matplotlib chama canvas.blit() uma vez POR EIXO
        (quatro por frame), então o valor bom é o da ÚLTIMA chamada. Por isso
        ele fica só anotado aqui e é consumido no callback seguinte, quando o
        frame anterior já fechou."""
        canvas = self._touch_canvas
        if canvas is None:
            return
        orig_blit = canvas.blit

        def timed_blit(bbox=None):
            orig_blit(bbox)
            t0 = self._touch_frame_t0
            if t0:
                self._touch_frame_cost = time.perf_counter() - t0

        canvas.blit = timed_blit

    def _touch_anim_frame(self, *args):
        """Callback da FuncAnimation: fecha a medição do frame anterior e
        desenha o atual. A média é exponencial (α=0,2) porque o custo oscila
        com a quantidade de spikes na janela e não se quer o intervalo
        pulando a cada frame."""
        cost = self._touch_frame_cost
        if cost is not None:
            self._touch_frame_cost = None
            ema = self._touch_frame_ema
            self._touch_frame_ema = (cost if ema is None
                                     else 0.8 * ema + 0.2 * cost)
        self._touch_frame_t0 = time.perf_counter()
        return self._touch_figure.update(*args)

    def _retune_touch_anim(self) -> None:
        """Ajusta o intervalo da animação ao custo MEDIDO do frame, para o
        desenho do toque nunca passar de _TOUCH_ANIM_DUTY da thread do Tk.

        Um intervalo fixo supõe um custo de frame fixo, e ele não é: escala
        com a área em pixels da figura (que segue o tamanho da janela) e com a
        grade do sensor (25 textos no 5×5 contra 16 no 4×4). Medido no
        cr10twin em 27/08/2026, com a janela maximizada e grade 5×5, o frame
        custava 25,9 ms p50 contra um intervalo fixo de 33 ms — a thread que
        pinta a GUI INTEIRA ficava ocupada ~44% só com esta figura e a aba
        respondia com atraso. Com o período em custo/_TOUCH_ANIM_DUTY sobra
        sempre ~60% do laço para o resto, em qualquer máquina.

        SÓ PODE SER CHAMADO DE FORA do callback da animação. O setter de
        `interval` reinicia o timer, e o TimerTk reagenda outro ao fim do
        callback: chamado lá dentro, os dois se somam e a taxa dobra a cada
        frame. Daqui (laço `after` da aba) há um único timer pendente."""
        anim = self._touch_anim
        src = getattr(anim, "event_source", None) if anim is not None else None
        ema = self._touch_frame_ema
        if src is None or not ema:
            return
        cost_ms = ema * 1e3
        period = min(max(cost_ms / _TOUCH_ANIM_DUTY, _TOUCH_ANIM_MIN_MS),
                     _TOUCH_ANIM_MAX_MS)
        want = max(int(period - cost_ms), 1)
        cur = getattr(src, "interval", 0)
        # Histerese: mexer no interval reinicia o timer do Tk, então só quando
        # a correção for de verdade — não a cada 80 ms.
        if cur <= 0 or abs(want - cur) / cur > 0.20:
            # Os DOIS, nesta ordem: `TimedAnimation._step` reescreve
            # `event_source.interval = self._interval` ao fim de CADA frame,
            # então mexer só no timer dura um frame e some. `_interval` é o
            # campo que a animação restaura; o setter do timer é o que faz o
            # novo valor valer já no próximo disparo.
            anim._interval = want
            src.interval = want

    # ------------------------------------------------------------ painel
    def _update_sensors_panel(self) -> None:
        """Cabeçalho: escalar do toque + de onde ele vem."""
        with self._touch_lock:
            touch_val = self._touch_value
            touch_ts = self._touch_last_ts
        if touch_ts <= 0.0:
            # Sem serial, o escalar ainda pode vir de /touch_sensor/value.
            touch_val, touch_ts = self.node.touch_scalar()
        fresh = touch_ts > 0.0 and (time.time() - touch_ts) < 3.0
        self._sens_touch_lbl.config(
            text=f"I_final  {touch_val:+.3f}" if fresh else "I_final  —",
            fg=INK if fresh else MUTED)

        label, fg = self._touch_source_status(fresh)
        self._sens_touch_status_lbl.config(text=label, fg=fg)

    def _touch_source_status(self, scalar_fresh: bool):
        """Texto/cor honestos da fonte do toque, do estado AO VIVO da fonte."""
        src = self._touch_source
        if src is not None and src.connected:
            base = f"serial {src.port}"
            if src.is_fresh():
                # Frames truncados não aparecem em lugar nenhum se não forem
                # ditos aqui: o descarte é correto, mas uma coleta que perdeu
                # 19% dos frames não pode parecer verde.
                bad, ok = src.frames_bad, src.frames_ok
                total = bad + ok
                if bad and total:
                    pct = 100.0 * bad / total
                    return (f"{base} — {pct:.1f}% dos frames perdidos "
                            f"({bad}/{total})", ALERT if pct < 1.0 else DANGER)
                return base, OK
            # Ligado mas mudo: porta serial errada ou STM mudo.
            return f"{base} (sem dados)", ALERT
        if scalar_fresh:
            return "via /touch_sensor/value", OK
        # Sem serial não há tátil nenhum. Vermelho, não cinza: em cinza isto
        # passa por "ainda não ligou".
        return "SEM SENSOR DE TOQUE (confira o cabo USB)", DANGER
