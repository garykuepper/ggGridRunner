"""Shared drawing style for the ggGridRunner layout sheets (fonts, ink colours, sheet frame)."""
import os

from matplotlib import font_manager as fm
from matplotlib.patches import Rectangle

# Optional drafting fonts; falls back to matplotlib defaults when absent.
FONTS = os.environ.get("GGRUNNER_FONT_DIR", os.path.join(os.path.dirname(__file__), "fonts"))


def font(name, family="sans-serif"):
    path = os.path.join(FONTS, name)
    if os.path.exists(path):
        fm.fontManager.addfont(path)
        return fm.FontProperties(fname=path)
    return fm.FontProperties(family=[family])


LABEL = font("Jura-Light.ttf")
LABEL_M = font("Jura-Medium.ttf")
MONO = font("IBMPlexMono-Regular.ttf", "monospace")
SERIF = font("InstrumentSerif-Regular.ttf", "serif")
SERIF_I = font("InstrumentSerif-Italic.ttf", "serif")

PAPER, INK, GREY, STEEL, VERM = "#F3EFE6", "#1E262D", "#8A9199", "#3F6C8C", "#C4452B"
HAIR, THIN, MED = 0.35, 0.6, 1.1


def sheet_frame(fig):
    """Double border with zone ticks, on a 17 x 11 sheet."""
    fr = fig.add_axes([0, 0, 1, 1], facecolor="none")
    fr.set_xlim(0, 17); fr.set_ylim(0, 11); fr.axis("off")
    fr.add_patch(Rectangle((0.45, 0.45), 16.1, 10.1, fill=False, lw=THIN, ec=INK))
    fr.add_patch(Rectangle((0.53, 0.53), 15.94, 9.94, fill=False, lw=HAIR, ec=INK))
    for i in range(1, 8):
        x = 0.45 + i * 16.1 / 8
        fr.plot([x, x], [0.45, 0.53], lw=HAIR, c=INK)
        fr.plot([x, x], [10.47, 10.55], lw=HAIR, c=INK)
        fr.text(x - 16.1 / 16, 0.49, str(i), fontproperties=MONO, fontsize=5, color=GREY,
                ha="center", va="center")
    for j, ch in enumerate("ABCDE"):
        y = 0.45 + (j + 0.5) * 10.1 / 5
        fr.text(0.49, y, ch, fontproperties=MONO, fontsize=5, color=GREY, ha="center",
                va="center")
    return fr


def title_block(tb, subtitle, date, status="DRAFT"):
    """Title block in the lower part of a 0..100 axes."""
    tb.add_patch(Rectangle((0, 0), 100, 26, fill=False, lw=THIN, ec=INK))
    tb.plot([64, 64], [0, 26], lw=HAIR, c=INK)
    tb.text(3, 17.5, "ggGridRunner", fontproperties=SERIF, fontsize=21, color=INK, va="center")
    tb.text(3, 6.5, subtitle, fontproperties=SERIF_I, fontsize=10, color=GREY, va="center")
    for k, (a, b) in enumerate((("SHEET", "1 / 1"), ("DATE", date), ("STATUS", status))):
        y = 20.5 - k * 7.5
        tb.text(67, y, a, fontproperties=LABEL, fontsize=5.6, color=GREY, va="center")
        tb.text(97, y, b, fontproperties=MONO, fontsize=6.2, color=INK, va="center", ha="right")
