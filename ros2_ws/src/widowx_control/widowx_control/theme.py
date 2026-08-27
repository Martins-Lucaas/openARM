"""Paleta e fontes da GUI, num só lugar.

Estas constantes viviam no topo do `gui_node.py`. Saíram para cá quando as
abas Sensores e Poses passaram a ser módulos próprios: três arquivos pintando
com três cópias da mesma paleta divergem na primeira vez que alguém acerta um
tom só num deles.

Sobre WARN/ALERT: o vermelho aqui sempre se chamou WARN, e é o que o
`gui_node.py` usa. O código portado do cr10twin distingue DOIS níveis — âmbar
para "olha isto" e vermelho para "isto está errado". Daí os três nomes:
WARN e DANGER são o MESMO vermelho (DANGER é o apelido que o código portado
usa) e ALERT é o âmbar, que não existia nesta GUI.
"""

BG = "#f4f6f8"          # fundo geral
CARD = "#ffffff"        # paineis
INK = "#263238"         # texto
MUTED = "#78909c"       # texto secundario
ACCENT = "#1976d2"      # azul principal
OK = "#2e7d32"          # verde
WARN = "#c62828"        # vermelho
DANGER = WARN           # apelido do vermelho no código portado do cr10twin
ALERT = "#ef6c00"       # âmbar: aviso que não impede o uso
LINE = "#e0e4e8"        # bordas
FIELD = "#eceff1"       # fundo de campo/lista dentro de um card

FONT_HEAD = ("TkDefaultFont", 11, "bold")
FONT_LBL = ("TkDefaultFont", 10)
FONT_SMALL = ("TkDefaultFont", 9)
FONT_BIG = ("TkDefaultFont", 20, "bold")
FONT_MONO = ("TkFixedFont", 10)
FONT_MONO_S = ("TkFixedFont", 9)
