"""ggGridRunner corner & chassis layout study — plan + front elevation.

Draws the +/-STEER sweep of each tire + motor unit (4WIS, one servo per corner) and
reports how much the crosswise battery clears the nearest sweep. Parameters are
design targets from the chassis brainstorm; MOTOR_L is an estimate until measured.

    python simulation/corner_layout_study.py
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Arc, Polygon
from matplotlib.lines import Line2D

# Optional drafting fonts; falls back to matplotlib defaults when absent.
FONTS = os.environ.get("GGRUNNER_FONT_DIR", os.path.join(os.path.dirname(__file__), "fonts"))


def font(name, family="sans-serif"):
    path = os.path.join(FONTS, name)
    if os.path.exists(path):
        fm.fontManager.addfont(path)
        return fm.FontProperties(fname=path)
    return fm.FontProperties(family=family)


LABEL = font("Jura-Light.ttf")
LABEL_M = font("Jura-Medium.ttf")
MONO = font("IBMPlexMono-Regular.ttf", "monospace")
SERIF = font("InstrumentSerif-Regular.ttf", "serif")
SERIF_I = font("InstrumentSerif-Italic.ttf", "serif")

PAPER, INK, GREY, STEEL, VERM = "#F3EFE6", "#1E262D", "#8A9199", "#3F6C8C", "#C4452B"
HAIR, THIN, MED = 0.35, 0.6, 1.1

# ---- parameters (mm) -------------------------------------------------------
T, WB = 240.0, 240.0            # track, wheelbase (kingpin centres)
TIRE_D, TIRE_W = 75.0, 30.0
ADAPT = 10.0                    # 4 mm -> 12 mm hex adapter
MOTOR_L, MOTOR_D = 70.0, 25.0   # JGA25-370 + encoder (ESTIMATE)
STEER = 50.0                    # mechanical clearance, deg
BAT_X, BAT_Y, BAT_Z = 144.0, 65.0, 36.0
RIDE = 30.0
ENV = 140.0

M0 = TIRE_W / 2 + ADAPT          # motor inboard start from kingpin
M1 = M0 + MOTOR_L


def unit_polys():
    """Tire, adapter, motor outlines in kingpin frame, motor toward +x."""
    tire = np.array([[-15, -37.5], [15, -37.5], [15, 37.5], [-15, 37.5]])
    adap = np.array([[15, -6], [M0, -6], [M0, 6], [15, 6]])
    mot = np.array([[M0, -12.5], [M1, -12.5], [M1, 12.5], [M0, 12.5]])
    return tire, adap, mot


def place(poly, kx, ky, ang, mirror):
    p = poly.copy()
    if mirror:
        p[:, 0] *= -1
    a = np.radians(ang)
    r = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    p = p @ r.T
    p[:, 0] += kx
    p[:, 1] += ky
    return p


def band_half():
    """Nearest approach of any corner sweep to the chassis x-centreline band."""
    tire, adap, mot = unit_polys()
    edge = np.vstack([np.linspace(mot[i], mot[(i + 1) % 4], 60) for i in range(4)])
    ymin = 1e9
    for ang in np.linspace(-STEER, STEER, 401):
        q = place(edge, -T / 2, WB / 2, ang, False)
        m = np.abs(q[:, 0]) <= BAT_X / 2
        if m.any():
            ymin = min(ymin, q[m, 1].min())
    return ymin


YB = band_half()
CLEAR = YB - BAT_Y / 2

fig = plt.figure(figsize=(17, 11), dpi=200, facecolor=PAPER)

# frame
fr = fig.add_axes([0, 0, 1, 1], facecolor="none")
fr.set_xlim(0, 17); fr.set_ylim(0, 11); fr.axis("off")
fr.add_patch(Rectangle((0.45, 0.45), 16.1, 10.1, fill=False, lw=THIN, ec=INK))
fr.add_patch(Rectangle((0.53, 0.53), 15.94, 9.94, fill=False, lw=HAIR, ec=INK))
for i in range(1, 8):
    x = 0.45 + i * 16.1 / 8
    fr.plot([x, x], [0.45, 0.53], lw=HAIR, c=INK); fr.plot([x, x], [10.47, 10.55], lw=HAIR, c=INK)
    fr.text(x - 16.1 / 16, 0.49, str(i), fontproperties=MONO, fontsize=5, color=GREY,
            ha="center", va="center")
for j, ch in enumerate("ABCDE"):
    y = 0.45 + (j + 0.5) * 10.1 / 5
    fr.text(0.49, y, ch, fontproperties=MONO, fontsize=5, color=GREY, ha="center", va="center")

# ============================ PLAN VIEW ======================================
ax = fig.add_axes([0.045, 0.085, 0.53, 0.83], facecolor="none")
ax.set_aspect("equal"); ax.axis("off")
ax.set_xlim(-235, 235); ax.set_ylim(-238, 232)

# faint 20 mm dot grid
gx, gy = np.meshgrid(np.arange(-220, 221, 20), np.arange(-220, 221, 20))
ax.scatter(gx, gy, s=0.25, c=GREY, alpha=0.45, lw=0)

# spin-in-place circle through the kingpins
R_spin = np.hypot(T / 2, WB / 2)
ax.add_patch(Circle((0, 0), R_spin, fill=False, lw=HAIR, ec=GREY, ls=(0, (8, 3, 1, 3))))

# centrelines
for a in ([-225, 225], [0, 0]), ([0, 0], [-225, 225]):
    ax.plot(*a, lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))

# rails + modules (upper level, steel)
for sx in (-1, 1):
    ax.add_patch(Rectangle((sx * 52 - 4, -170), 8, 340, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
for yc, name in ((WB / 2 + 5, "FRONT MODULE"), (-(WB / 2 + 5), "REAR MODULE")):
    ax.add_patch(Rectangle((-ENV / 2, yc - 55), ENV, 110, fc=STEEL, alpha=0.05, ec="none"))
    ax.add_patch(Rectangle((-ENV / 2, yc - 55), ENV, 110, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
for sx in (-1, 1):  # battery side cradles
    ax.add_patch(Rectangle((sx * 77 - 7, -36), 14, 72, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))

# upper A-arms (plan)
for sx in (-1, 1):
    for sy in (-1, 1):
        kx, ky = sx * T / 2, sy * WB / 2
        for (px, dy) in ((60, 22), (65, 14)):
            for d in (-1, 1):
                ax.plot([kx - sx * 14, sx * px], [ky, ky + d * dy], lw=THIN, c=STEEL)
            ax.add_patch(Circle((sx * px, ky + dy), 2.2, fc=PAPER, ec=STEEL, lw=THIN))
            ax.add_patch(Circle((sx * px, ky - dy), 2.2, fc=PAPER, ec=STEEL, lw=THIN))

# battery (wheel level)
bx0, by0 = -BAT_X / 2, -BAT_Y / 2
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc=INK, alpha=0.06, ec="none"))
for i in range(8):
    ax.add_patch(Rectangle((bx0 + i * 18 + 0.8, by0 + 0.8), 16.4, BAT_Y - 1.6,
                           fc="none", ec=INK, lw=HAIR))
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc="none", ec=INK, lw=MED))

# corner sweeps
tire, adap, mot = unit_polys()
for sx in (-1, 1):
    for sy in (-1, 1):
        kx, ky, mir = sx * T / 2, sy * WB / 2, sx > 0
        for ang in np.arange(-STEER, STEER + 0.01, 2.5):
            for p in (tire, mot):
                ax.add_patch(Polygon(place(p, kx, ky, ang, mir), closed=True, fill=False,
                                     ec=VERM, lw=0.22, alpha=0.55))
        for ang in (-STEER, STEER):
            for p in (tire, mot):
                ax.add_patch(Polygon(place(p, kx, ky, ang, mir), closed=True, fill=False,
                                     ec=VERM, lw=THIN))
        ax.add_patch(Polygon(place(tire, kx, ky, 0, mir), fc=INK, ec=INK, lw=THIN))
        ax.add_patch(Polygon(place(adap, kx, ky, 0, mir), fc=PAPER, ec=INK, lw=THIN))
        ax.add_patch(Polygon(place(mot, kx, ky, 0, mir), fc=PAPER, ec=INK, lw=MED))
        enc = np.array([[M1 - 14, -12.5], [M1, -12.5], [M1, 12.5], [M1 - 14, 12.5]])
        ax.add_patch(Polygon(place(enc, kx, ky, 0, mir), fc=INK, alpha=0.18, ec="none"))
        # kingpin
        ax.add_patch(Circle((kx, ky), 5, fc=PAPER, ec=INK, lw=THIN, zorder=6))
        ax.plot([kx - 9, kx + 9], [ky, ky], lw=HAIR, c=INK, zorder=7)
        ax.plot([kx, kx], [ky - 9, ky + 9], lw=HAIR, c=INK, zorder=7)
        # spin tangent direction
        tang = np.array([-ky, kx]) / R_spin
        ax.annotate("", xy=(kx + tang[0] * 30, ky + tang[1] * 30), xytext=(kx, ky),
                    arrowprops=dict(arrowstyle="-|>", lw=THIN, color=INK, mutation_scale=7),
                    zorder=8)

# spin angle arc at FR
ax.add_patch(Arc((T / 2, WB / 2), 58, 58, theta1=90, theta2=135, lw=THIN, ec=INK))
ax.text(T / 2 - 20, WB / 2 + 36, f"{np.degrees(np.arctan(WB / T)):.1f}°",
        fontproperties=MONO, fontsize=6.5, color=INK, ha="center")


def dim(a, p0, p1, off, text, horiz=True, col=INK):
    if horiz:
        y = off
        a.plot([p0, p0], [y - 4, y + 4], lw=HAIR, c=col)
        a.plot([p1, p1], [y - 4, y + 4], lw=HAIR, c=col)
        a.annotate("", xy=(p0, y), xytext=(p1, y),
                   arrowprops=dict(arrowstyle="<|-|>", lw=HAIR, color=col, mutation_scale=6))
        a.text((p0 + p1) / 2, y + 4, text, fontproperties=MONO, fontsize=6.5, color=col,
               ha="center", va="bottom", bbox=dict(fc=PAPER, ec="none", pad=0.6))
    else:
        x = off
        a.plot([x - 4, x + 4], [p0, p0], lw=HAIR, c=col)
        a.plot([x - 4, x + 4], [p1, p1], lw=HAIR, c=col)
        a.annotate("", xy=(x, p0), xytext=(x, p1),
                   arrowprops=dict(arrowstyle="<|-|>", lw=HAIR, color=col, mutation_scale=6))
        a.text(x - 5, (p0 + p1) / 2, text, fontproperties=MONO, fontsize=6.5, color=col,
               ha="right", va="center", rotation=90, bbox=dict(fc=PAPER, ec="none", pad=0.6))


dim(ax, -T / 2, T / 2, 218, f"TRACK {T:.0f}")
dim(ax, -WB / 2, WB / 2, -212, f"WHEELBASE {WB:.0f}", horiz=False)
dim(ax, -ENV / 2, ENV / 2, -200, f"≤ {ENV:.0f} PRINT ENVELOPE", col=STEEL)

# battery clearance callout
cx = -30
ax.plot([cx, cx], [BAT_Y / 2, YB], lw=MED, c=VERM)
ax.plot([cx - 5, cx + 5], [YB, YB], lw=THIN, c=VERM)
ax.text(cx + 7, YB + 4, f"{CLEAR:.1f} mm", fontproperties=MONO, fontsize=7, color=VERM,
        ha="left", va="bottom", bbox=dict(fc=PAPER, ec="none", pad=0.6))

# plan labels
def lab(a, x, y, s, col=INK, ha="center", size=6.2, fp=LABEL_M):
    a.text(x, y, s, fontproperties=fp, fontsize=size, color=col, ha=ha, va="center",
           bbox=dict(fc=PAPER, ec="none", pad=0.8, alpha=0.92))

lab(ax, 0, 0, "BATTERY  4S4P  144 × 65 × 36", size=6.4)
lab(ax, 0, WB / 2 + 45, "FRONT MODULE", col=STEEL)
lab(ax, 0, -(WB / 2 + 45), "REAR MODULE", col=STEEL)
lab(ax, 0, WB / 2 + 70, "FORWARD  +Y", col=GREY, size=5.8)
lab(ax, 52, 52, "RAIL", col=STEEL, size=5.6)
lab(ax, 77, -44, "CRADLE", col=STEEL, size=5.6)
lab(ax, -77, -44, "CRADLE", col=STEEL, size=5.6)
ax.text(-R_spin, -40, "SPIN-IN-PLACE CIRCLE", fontproperties=LABEL_M, fontsize=5.6, color=GREY,
        ha="center", va="center", rotation=90, bbox=dict(fc=PAPER, ec="none", pad=0.8))
lab(ax, -150, 52, f"±{STEER:.0f}° SWEEP", col=VERM, size=5.8)

ax.text(-232, 226, "PLAN", fontproperties=SERIF, fontsize=17, color=INK, va="top")
ax.text(-232, 208, "view from above  ·  true proportion  ·  mm", fontproperties=LABEL, fontsize=6.5,
        color=GREY, va="top")

# ============================ FRONT ELEVATION ================================
ex = fig.add_axes([0.6, 0.43, 0.37, 0.49], facecolor="none")
ex.set_aspect("equal"); ex.axis("off")
ex.set_xlim(-62, 176); ex.set_ylim(-26, 205)

KX = T / 2
AXLE = TIRE_D / 2
# level bands
ex.add_patch(Rectangle((-60, 0), 220, 80, fc=VERM, alpha=0.035, ec="none"))
ex.plot([-60, 160], [80, 80], lw=HAIR, c=VERM, ls=(0, (3, 2)))
ex.text(-60, 76, "WHEEL LEVEL · KEEP-OUT", fontproperties=LABEL_M, fontsize=5.4, color=VERM,
        ha="left", va="top")
ex.text(-60, 84, "SUSPENSION LEVEL", fontproperties=LABEL_M, fontsize=5.4, color=STEEL,
        ha="left", va="bottom")
# ground
ex.plot([-60, 160], [0, 0], lw=MED, c=INK)
for x in np.arange(-56, 161, 5):
    ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
# centreline & kingpin axis
ex.plot([0, 0], [-10, 196], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
ex.plot([KX, KX], [-10, 196], lw=HAIR, c=INK, ls=(0, (12, 3, 2, 3)))
ex.text(KX + 2, 198, "KINGPIN AXIS", fontproperties=LABEL, fontsize=5.4, color=INK, ha="left")
ex.text(1.5, 198, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.4, color=GREY, ha="left")

# battery behind (dashed)
ex.add_patch(Rectangle((0, RIDE), BAT_X / 2, BAT_Z, fc=INK, alpha=0.05, ec=INK, lw=THIN,
                       ls=(0, (3, 2))))
ex.text(3, RIDE + BAT_Z - 3, "BATTERY  (behind)", fontproperties=LABEL, fontsize=5.2,
        color=INK, ha="left", va="top")

# travel ghosts
for dz in (-12.5, 12.5):
    ex.add_patch(FancyBboxPatch((KX - 15, dz + 0.5), 30, TIRE_D - 1,
                                boxstyle="round,pad=0,rounding_size=6", fc="none",
                                ec=GREY, lw=HAIR, ls=(0, (2, 2))))
# tire
ex.add_patch(FancyBboxPatch((KX - 15, 0), 30, TIRE_D, boxstyle="round,pad=0,rounding_size=6",
                            fc=INK, ec=INK, lw=THIN))
for z in np.arange(6, TIRE_D - 4, 5):
    ex.plot([KX - 15, KX + 15], [z, z], lw=HAIR, c=PAPER, alpha=0.35)
# adapter + motor
ex.add_patch(Rectangle((KX - M0, AXLE - 6), ADAPT, 12, fc=PAPER, ec=INK, lw=THIN))
ex.add_patch(Rectangle((KX - M1, AXLE - 12.5), MOTOR_L, MOTOR_D, fc=PAPER, ec=INK, lw=MED))
ex.add_patch(Rectangle((KX - M1, AXLE - 12.5), 14, MOTOR_D, fc=INK, alpha=0.18, ec="none"))
ex.plot([KX - M0 - 21, KX - M0 - 21], [AXLE - 12.5, AXLE + 12.5], lw=HAIR, c=INK)
# yoke (rotates with wheel)
yoke = np.array([[KX - 62, AXLE - 16], [KX - 44, AXLE - 16], [KX - 44, 80], [KX + 6, 80],
                 [KX + 6, 88], [KX - 52, 88], [KX - 52, AXLE + 16], [KX - 62, AXLE + 16]])
ex.add_patch(Polygon(yoke, closed=True, fc=VERM, alpha=0.12, ec=VERM, lw=THIN))
# upright + servo + bearing
UP0, UP1 = 90, 144
ex.add_patch(Rectangle((KX - 16, UP0), 32, UP1 - UP0, fc=STEEL, alpha=0.08, ec=STEEL, lw=MED))
ex.add_patch(Rectangle((KX - 10, 102), 20, 38, fc=PAPER, ec=INK, lw=THIN))
ex.add_patch(Rectangle((KX - 7, UP0 + 1.5), 14, 7, fc=INK, alpha=0.25, ec=INK, lw=HAIR))
ex.plot([KX, KX], [88, 102], lw=MED * 1.4, c=INK, solid_capstyle="butt")
# arms
LA_Z, UA_Z = 98, 136
for (px, z) in ((60, LA_Z), (65, UA_Z)):
    ex.plot([px, KX - 16], [z, z], lw=MED * 1.6, c=STEEL, solid_capstyle="round")
    for x in (px, KX - 16):
        ex.add_patch(Circle((x, z), 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=5))
# chassis module wall + shock tower
ex.add_patch(Rectangle((0, 90), 66, 56, fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN,
                       ls=(0, (4, 2))))
tower = np.array([[30, 146], [44, 146], [44, 184], [30, 184]])
ex.add_patch(Polygon(tower, closed=True, fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN,
                     ls=(0, (4, 2))))
s0, s1 = np.array([96.0, LA_Z]), np.array([38.0, 178.0])
u = (s1 - s0) / np.linalg.norm(s1 - s0); n = np.array([-u[1], u[0]])
body0, body1 = s0 + u * 12, s0 + u * 62
ex.plot(*zip(s0, s1), lw=THIN, c=INK)
ex.add_patch(Polygon([body0 + n * 5, body1 + n * 5, body1 - n * 5, body0 - n * 5], fc=PAPER,
                     ec=INK, lw=THIN, zorder=4))
for k in np.linspace(0.12, 0.9, 11):
    p = body0 + (body1 - body0) * k
    ex.plot(*zip(p + n * 5, p - n * 5), lw=HAIR, c=INK, zorder=5)
for p in (s0, s1):
    ex.add_patch(Circle(p, 2.4, fc=PAPER, ec=INK, lw=THIN, zorder=6))


def elab(x, y, s, tx, ty, col=INK, ha="left"):
    ex.annotate(s, xy=(x, y), xytext=(tx, ty), fontproperties=LABEL_M, fontsize=5.6, color=col,
                ha=ha, va="center",
                arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=0),
                bbox=dict(fc=PAPER, ec="none", pad=0.6))


elab(KX + 10, 120, "MG996R  (in upright)", 142, 170)
elab(KX + 7, UP0 + 5, "BEARING  (takes load)", 142, 106)
elab(KX + 6, 84, "YOKE  (steers)", 142, 88, col=VERM)
elab(KX + 15, 55, "TIRE  75 × 30", 142, 60)
elab(KX - 60, AXLE - 12.5, f"JGA25-370  ~{MOTOR_L:.0f} long", 60, -16)
elab(90, LA_Z, "LOWER ARM", 90, 72, col=STEEL, ha="center")
elab(88, UA_Z, "UPPER ARM", 86, 158, col=STEEL, ha="center")
elab(72, 131, "SHOCK", 12, 160, col=INK, ha="center")

# heights
for z, t in ((AXLE, f"{AXLE:.1f}"), (80, "80"), (UP1, f"{UP1:.0f}")):
    ex.plot([160, 166], [z, z], lw=HAIR, c=GREY)
    ex.text(168, z, t, fontproperties=MONO, fontsize=5.4, color=GREY, va="center")
ex.plot([163, 163], [0, UP1], lw=HAIR, c=GREY)

ex.text(-60, 204, "ELEVATION", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        ha="left")
ex.text(-60, 190, "front-left corner, looking rearward  ·  mm", fontproperties=LABEL,
        fontsize=6.5, color=GREY, va="top")

# ============================ SCHEDULE + TITLE ===============================
tb = fig.add_axes([0.6, 0.085, 0.355, 0.31], facecolor="none")
tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")

rows = [
    ("Track, kingpin c-c", f"{T:.0f} mm"),
    ("Wheelbase", f"{WB:.0f} mm   (was 230)"),
    ("Spin-in-place angle", f"{np.degrees(np.arctan(WB / T)):.1f}°"),
    ("Steering: commanded / clearance", f"±45° / ±{STEER:.0f}°"),
    ("Battery clearance to sweep", f"{CLEAR:.1f} mm per side"),
    ("Motor + encoder length", f"{MOTOR_L:.0f} mm  ESTIMATE"),
    ("Print envelope (Mini, margin)", f"{ENV:.0f} × {ENV:.0f} × 145"),
]
tb.text(0, 97, "SCHEDULE", fontproperties=LABEL_M, fontsize=7, color=INK, va="top")
tb.plot([0, 100], [91, 91], lw=THIN, c=INK)
for i, (k, v) in enumerate(rows):
    y = 85 - i * 7.2
    warn = "ESTIMATE" in v or "clearance to" in k
    tb.text(0, y, k, fontproperties=LABEL, fontsize=7, color=INK, va="center")
    tb.text(100, y, v, fontproperties=MONO, fontsize=6.8, color=VERM if warn else INK,
            ha="right", va="center")
    tb.plot([0, 100], [y - 3.6, y - 3.6], lw=HAIR, c=GREY, alpha=0.5)

tb.add_patch(Rectangle((0, 0), 100, 26, fill=False, lw=THIN, ec=INK))
tb.plot([64, 64], [0, 26], lw=HAIR, c=INK)
tb.text(3, 17.5, "ggGridRunner", fontproperties=SERIF, fontsize=21, color=INK, va="center")
tb.text(3, 6.5, "corner & chassis layout  —  study 01", fontproperties=SERIF_I, fontsize=10,
        color=GREY, va="center")
for k, (a, b) in enumerate((("SHEET", "1 / 1"), ("DATE", "2026-09-24"), ("STATUS", "DRAFT"))):
    y = 20.5 - k * 7.5
    tb.text(67, y, a, fontproperties=LABEL, fontsize=5.6, color=GREY, va="center")
    tb.text(97, y, b, fontproperties=MONO, fontsize=6.2, color=INK, va="center", ha="right")

# legend key
kx0 = 0
for j, (col, txt, style) in enumerate(((INK, "wheel level · solid", "-"),
                                       (VERM, "steering sweep", "-"),
                                       (STEEL, "suspension level · chassis", "--"))):
    x = kx0 + j * 34
    tb.plot([x, x + 6], [33, 33], lw=MED, c=col, ls=style)
    tb.text(x + 8, 33, txt, fontproperties=LABEL, fontsize=5.8, color=INK, va="center")

fig.savefig(os.path.join(os.path.dirname(__file__), "corner_layout_study.png"), facecolor=PAPER)
print(f"band half {YB:.1f}  clearance {CLEAR:.1f}")
