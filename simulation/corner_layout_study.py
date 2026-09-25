"""ggGridRunner corner & chassis layout study: plan + front elevation.

Corner as in the orange prototype: a knuckle plate bolts to the gearbox face and sits inside the
wheel's hub pocket. Long parallel double wishbones reach into the pocket to its top and bottom
joints (the kingpin). The motor lies between the arms and steers with the wheel. Each corner's
MG996R sits on the chassis spine and steers through a tie rod parallel to the arms and the same
length, which keeps bump steer near zero. The shock runs from the upper arm to a central tower,
above the motor's sweep.

Draws the +/-STEER sweep of each corner, solves the spin-in-place angle (the contact point
swings about the kingpin by the scrub offset), and reports clearances. Wheel and motor
dimensions are measured.

    python simulation/corner_layout_study.py
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Arc, Polygon

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

# ---- parameters (mm) -------------------------------------------------------
TIRE_D, TIRE_W = 120.0, 42.0    # measured
HUB_D = 81.0                    # hub pocket diameter (measured)
HEX_IN = 33.0                   # hex mating face -> wheel inner (motor-side) face (measured)
GBX_FACE = 22.0                 # gearbox face -> hex mating face (measured)
MOTOR_BODY = 61.0               # gearbox + motor 51, + 10 encoder clearance (measured)
MOTOR_D = 25.0
ARM = 95.0                      # wishbone length, pivot to kingpin (prototype ~100)
PIVOT = 30.0                    # inner pivots (both arms, servo, tie rod) from centreline
WB = 230.0                      # wheelbase (kingpin c-c fore-aft)
STEER = 50.0                    # mechanical clearance, deg
LO_Z, UP_Z = 32.0, 90.0         # knuckle joints: lower / upper arm (inside the hub pocket)
TR_Z = 80.0                     # tie rod height (between motor top and upper arm)
STEER_ARM = 18.0                # knuckle steering arm = servo horn length (parallelogram)
LO_SPREAD, UP_SPREAD = 20.0, 15.0  # half-spread of the wishbone inner pivots, fore-aft
SHOCK_AT = 0.4                  # shock mount on upper arm, fraction of arm from the knuckle
BUMP, DROOP = 15.0, 10.0        # wheel travel from ride height (25 total)
RISE = 20.0                     # inner pivots sit this much above the knuckle joints: the
                                # chassis rides higher, wheels hang lower; >= BUMP keeps the
                                # tie rod rising away from the motor at every point of travel
BEND_A = 29.0                   # boomerang bend, along the arm from the knuckle (upper arm);
                                # the lower arm is the same part flipped: bend at ARM - BEND_A
# Link shapes: (distance inboard from the kingpin, height above the knuckle joint).
SHAPES = {
    "upper": ([0.0, BEND_A, ARM], [0.0, RISE, RISE]),
    "lower": ([0.0, ARM - BEND_A, ARM], [0.0, 0.0, RISE]),
    "tie rod": ([0.0, ARM], [0.0, RISE]),
}
SHOCK_LEN = 100.0               # shock eye to eye at ride height
ARM_HALF, TR_HALF = 4.0, 2.0    # half-thickness of an arm / the tie rod (elevation)
TOWER_X = 10.0                  # shock tower top mount, from centreline
BAT_L, BAT_W, BAT_Z = 144.0, 65.0, 36.0   # 4S4P brick: two layers of 8 cells along BAT_L
BAT_X, BAT_Y = BAT_W, BAT_L     # plan size: long side fore-aft
ENV = 140.0

# Drawing coords: X lateral, Y forward (plan reads forward-up). Rover frame: +X forward,
# +Y left, +Z up. Local corner frame: origin at kingpin, +x inboard, +y outward fore-aft.
AXLE = TIRE_D / 2
KP_X = PIVOT + ARM              # kingpin from centreline; kingpin = gearbox face
TIRE_IN = HEX_IN - GBX_FACE     # wheel inner face, inboard of the kingpin (kingpin is in the pocket)
TIRE_OUT = TIRE_IN - TIRE_W
SCRUB = -(TIRE_IN + TIRE_OUT) / 2  # kingpin -> tire centre
T = 2 * (KP_X + SCRUB)          # track, tire c-c
HEX_X = -GBX_FACE
TAIL_X = MOTOR_BODY


def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


TIRE = rect(TIRE_OUT, TIRE_IN, -TIRE_D / 2, TIRE_D / 2)
HUB = rect(TIRE_IN, TIRE_IN - HEX_IN, -HUB_D / 2, HUB_D / 2)
MOTOR = rect(max(TIRE_IN, 0), TAIL_X, -MOTOR_D / 2, MOTOR_D / 2)    # visible part
MOTOR_HID = rect(0, max(TIRE_IN, 0), -MOTOR_D / 2, MOTOR_D / 2)
COUPLER = rect(HEX_X, 0, -6, 6)
PLATE = rect(-5, 0, -22, 22)                                   # knuckle plate
SERVO = rect(ARM - 10, ARM + 10, -10, 30)                      # MG996R on the spine, fixed


def place(poly, kx, ky, ang, mirror_x, mirror_y=False):
    """Local corner frame -> chassis frame."""
    p = poly.copy()
    a = np.radians(ang)
    r = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    p = p @ r.T
    if mirror_x:
        p[:, 0] *= -1
    if mirror_y:
        p[:, 1] *= -1
    p[:, 0] += kx
    p[:, 1] += ky
    return p


def spin_angle():
    """Steer angle at the front-left wheel that rolls it tangent to a spin about the centre."""
    kx, ky = -KP_X, WB / 2
    a = np.radians(np.linspace(-STEER, STEER, 40001))
    cx = kx - SCRUB * np.cos(a)
    cy = ky - SCRUB * np.sin(a)
    e = np.abs(cx * -np.sin(a) + cy * np.cos(a)) / np.hypot(cx, cy)
    i = e.argmin()
    return abs(np.degrees(a[i])), (cx[i], cy[i])


def edge_points(poly, n=80):
    return np.vstack([np.linspace(poly[i], poly[(i + 1) % 4], n) for i in range(4)])


def rect_gap(pts, x0, x1, y0, y1):
    dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
    dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
    return np.hypot(dx, dy).min()


def clearances():
    tire_mot = np.vstack([edge_points(TIRE), edge_points(MOTOR)])
    mot = edge_points(MOTOR)
    bat, servo = 1e9, 1e9
    for ang in np.linspace(-STEER, STEER, 401):
        q = place(tire_mot, -KP_X, WB / 2, ang, False)
        bat = min(bat, rect_gap(q, -BAT_X / 2, BAT_X / 2, -BAT_Y / 2, BAT_Y / 2))
        m = place(mot, 0, 0, ang, False)        # servo is fixed in the local frame
        servo = min(servo, rect_gap(m, SERVO[:, 0].min(), SERVO[:, 0].max(),
                                    SERVO[:, 1].min(), SERVO[:, 1].max()))
    front = np.vstack([place(edge_points(TIRE), -KP_X, WB / 2, a, False)
                       for a in np.linspace(-STEER, STEER, 81)])
    return bat, servo, 2 * front[:, 1].min()


SPIN, SPIN_CP = spin_angle()
CLEAR, SERVO_CLEAR, FA_GAP = clearances()
PIV_BOSS = 5.0
PIV_GAP = WB / 2 - LO_SPREAD - PIV_BOSS - BAT_Y / 2
TAIL_R = np.hypot(TAIL_X, MOTOR_D / 2)


def shape(a, name):
    """Height of a link above its knuckle joint at distance a (inboard) from the kingpin."""
    return np.interp(a, *SHAPES[name])


def travel_gaps():
    """Smallest vertical gap from the motor to each link over the full travel.

    The arms are a parallelogram, so the motor stays level and rises dz, while a link rises
    only dz * (1 - a / ARM) at distance a from the kingpin. The motor can sit anywhere from
    the kingpin out to its swept tail radius (conservative: any steer angle).
    """
    a = np.linspace(0, TAIL_R, 300)
    top, bot = AXLE + MOTOR_D / 2, AXLE - MOTOR_D / 2
    g = {"upper arm": 1e9, "tie rod": 1e9, "lower arm": 1e9}
    for dz in np.linspace(-DROOP, BUMP, 101):
        close = dz * a / ARM
        g["upper arm"] = min(g["upper arm"],
                             (UP_Z + shape(a, "upper") - ARM_HALF - top - close).min())
        g["tie rod"] = min(g["tie rod"], (TR_Z + shape(a, "tie rod") - TR_HALF - top - close).min())
        g["lower arm"] = min(g["lower arm"],
                             (bot - LO_Z - shape(a, "lower") - ARM_HALF + close).min())
    return g


TRAVEL_GAPS = travel_gaps()
_a = np.linspace(0, ARM, 400)
ROD_TO_ARM = (UP_Z + shape(_a, "upper") - ARM_HALF - TR_Z - shape(_a, "tie rod") - TR_HALF).min()
SHOCK_A = ARM * SHOCK_AT
SHOCK_LO = np.array([KP_X - SHOCK_A, UP_Z + shape(SHOCK_A, "upper")])  # elevation coords
TOWER_Z = SHOCK_LO[1] + np.sqrt(SHOCK_LEN ** 2 - (SHOCK_LO[0] - TOWER_X) ** 2)
BELLY = LO_Z + RISE - 8         # spine underside near the lower pivots
MOD_Y0 = BAT_Y / 2 + 4

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

gx, gy = np.meshgrid(np.arange(-220, 221, 20), np.arange(-220, 221, 20))
ax.scatter(gx, gy, s=0.25, c=GREY, alpha=0.45, lw=0)

R_spin = np.hypot(*SPIN_CP)
ax.add_patch(Circle((0, 0), R_spin, fill=False, lw=HAIR, ec=GREY, ls=(0, (8, 3, 1, 3))))
for a in ([-225, 225], [0, 0]), ([0, 0], [-225, 225]):
    ax.plot(*a, lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))

# spines, rails, cradles
MOD_L = WB / 2 + 50 - MOD_Y0
for yc in (MOD_Y0 + MOD_L / 2, -(MOD_Y0 + MOD_L / 2)):
    ax.add_patch(Rectangle((-ENV / 2, yc - MOD_L / 2), ENV, MOD_L, fc=STEEL, alpha=0.05,
                           ec="none"))
    ax.add_patch(Rectangle((-ENV / 2, yc - MOD_L / 2), ENV, MOD_L, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
for sx in (-1, 1):
    ax.add_patch(Rectangle((sx * 52 - 4, -170), 8, 340, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
    ax.add_patch(Rectangle((sx * (BAT_X / 2 + 8) - 7, -BAT_Y / 2 + 4), 14, BAT_Y - 8,
                           fc="none", ec=STEEL, lw=THIN, ls=(0, (4, 2))))

# battery
bx0, by0 = -BAT_X / 2, -BAT_Y / 2
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc=INK, alpha=0.06, ec="none"))
for i in range(8):
    ax.add_patch(Rectangle((bx0 + 0.8, by0 + i * 18 + 0.8), BAT_X - 1.6, 16.4,
                           fc="none", ec=INK, lw=HAIR))
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc="none", ec=INK, lw=MED))

for sx in (-1, 1):
    for sy in (-1, 1):
        kx, ky, mx, my = sx * KP_X, sy * WB / 2, sx > 0, sy < 0
        P = lambda poly, ang=0.0: place(np.asarray(poly, float), kx, ky, ang, mx, my)
        # wishbones: kingpin -> inner pivots
        for spread, lw in ((LO_SPREAD, THIN), (UP_SPREAD, HAIR * 1.4)):
            for d in (-1, 1):
                seg = P([[0, 0], [ARM, d * spread]])
                ax.plot(seg[:, 0], seg[:, 1], lw=lw, c=STEEL)
                ax.add_patch(Circle(seg[1], 2.2, fc=PAPER, ec=STEEL, lw=THIN, zorder=5))
        # shock (plan): upper arm -> tower
        sh = P([[ARM * SHOCK_AT, 0], [KP_X - TOWER_X, 0]])
        ax.plot(sh[:, 0], sh[:, 1], lw=MED * 1.8, c=INK, alpha=0.35, solid_capstyle="butt")
        # sweeps
        for ang in np.arange(-STEER, STEER + 0.01, 2.5):
            for p in (TIRE, MOTOR):
                ax.add_patch(Polygon(P(p, ang), closed=True, fill=False, ec=VERM, lw=0.22,
                                     alpha=0.55))
        for ang in (-STEER, STEER):
            for p in (TIRE, MOTOR):
                ax.add_patch(Polygon(P(p, ang), closed=True, fill=False, ec=VERM, lw=THIN))
        # straight-ahead parts
        ax.add_patch(Polygon(P(TIRE), fc=INK, ec=INK, lw=THIN))
        for p in (HUB, MOTOR_HID, COUPLER):
            ax.add_patch(Polygon(P(p), fc="none", ec=PAPER, lw=HAIR, ls=(0, (2, 1.5))))
        ax.add_patch(Polygon(P(PLATE), fc=VERM, alpha=0.5, ec=VERM, lw=THIN, zorder=4))
        ax.add_patch(Polygon(P(MOTOR), fc=PAPER, ec=INK, lw=MED))
        # steering: knuckle arm, tie rod, servo horn, servo
        link = P([[0, 0], [0, STEER_ARM], [ARM, STEER_ARM], [ARM, 0]])
        ax.plot(link[:, 0], link[:, 1], lw=THIN, c=VERM, zorder=6)
        for q in link[1:3]:
            ax.add_patch(Circle(q, 1.8, fc=PAPER, ec=VERM, lw=THIN, zorder=7))
        ax.add_patch(Polygon(P(SERVO), fc=STEEL, alpha=0.12, ec=STEEL, lw=THIN, zorder=3))
        ax.add_patch(Circle(link[3], 3, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
        ax.add_patch(Circle((kx, ky), 3.5, fc=PAPER, ec=INK, lw=THIN, zorder=8))
        ax.plot([kx - 7, kx + 7], [ky, ky], lw=HAIR, c=INK, zorder=9)
        ax.plot([kx, kx], [ky - 7, ky + 7], lw=HAIR, c=INK, zorder=9)

# spin angle arc at FR kingpin
ax.add_patch(Arc((KP_X, WB / 2), 50, 50, theta1=90, theta2=90 + SPIN, lw=THIN, ec=INK, zorder=9))
ax.text(KP_X - 16, WB / 2 + 36, f"{SPIN:.1f}°", fontproperties=MONO, fontsize=6.5, color=INK,
        ha="center", bbox=dict(fc=PAPER, ec="none", pad=0.6))


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


dim(ax, -T / 2, T / 2, 222, f"TRACK {T:.0f}  (tire c-c)")
dim(ax, -KP_X, KP_X, -222, f"KINGPIN C-C {2 * KP_X:.0f}")
dim(ax, -WB / 2, WB / 2, -224, f"WHEELBASE {WB:.0f}", horiz=False)
dim(ax, -ENV / 2, ENV / 2, -200, f"≤ {ENV:.0f} PRINT ENVELOPE", col=STEEL)

ax.text(-100, 14, f"battery to sweep  {CLEAR:.1f} mm\nbattery to arm pivot  {PIV_GAP:.1f} mm\n"
        f"motor tail to servo  {SERVO_CLEAR:.1f} mm",
        fontproperties=MONO, fontsize=6.5, color=VERM, ha="center", va="top", linespacing=1.6,
        bbox=dict(fc=PAPER, ec="none", pad=0.6))


def lab(a, x, y, s, col=INK, ha="center", size=6.2, fp=LABEL_M, rot=0):
    a.text(x, y, s, fontproperties=fp, fontsize=size, color=col, ha=ha, va="center", rotation=rot,
           bbox=dict(fc=PAPER, ec="none", pad=0.8, alpha=0.92))


lab(ax, 0, 0, "BATTERY  4S4P  144 × 65 × 36", size=6.4, rot=90)
lab(ax, 0, WB / 2 + 40, "FRONT SPINE", col=STEEL)
lab(ax, 0, -(WB / 2 + 40), "REAR SPINE", col=STEEL)
lab(ax, 52, 60, "RAIL", col=STEEL, size=5.6)
lab(ax, BAT_X / 2 + 8, -BAT_Y / 2 - 6, "CRADLE", col=STEEL, size=5.6)
lab(ax, -BAT_X / 2 - 8, -BAT_Y / 2 - 6, "CRADLE", col=STEEL, size=5.6)
lab(ax, -KP_X + ARM, WB / 2 + 42, "MG996R", col=STEEL, size=5.6)
lab(ax, -KP_X + ARM / 2, WB / 2 + STEER_ARM + 8, "TIE ROD", col=VERM, size=5.6)
ax.text(-R_spin - 3, 0, "SPIN-IN-PLACE CIRCLE", fontproperties=LABEL_M, fontsize=5.6,
        color=GREY, ha="center", va="center", rotation=90, bbox=dict(fc=PAPER, ec="none", pad=0.8))
lab(ax, -150, 40, f"±{STEER:.0f}° SWEEP", col=VERM, size=5.8)

ax.text(0.005, 0.995, "PLAN", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        transform=ax.transAxes)
ax.text(0.005, 0.955, "view from above  ·  true proportion  ·  mm  ·  forward up the page  ·  "
        "+X forward  ·  +Y left  ·  +Z up", fontproperties=LABEL, fontsize=6.5, color=GREY,
        va="top", transform=ax.transAxes)

# ============================ FRONT ELEVATION ================================
ex = fig.add_axes([0.6, 0.43, 0.37, 0.49], facecolor="none")
ex.set_aspect("equal"); ex.axis("off")
ex.set_xlim(-40, 198); ex.set_ylim(-26, 205)

KX = KP_X                        # kingpin = gearbox face
T_IN, T_OUT = KX - TIRE_IN, KX - TIRE_OUT
TC = (T_IN + T_OUT) / 2
TAIL_E = KX - TAIL_X

ex.plot([-38, 196], [0, 0], lw=MED, c=INK)
for x in np.arange(-34, 197, 5):
    ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
ex.plot([0, 0], [-10, 176], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
ex.plot([KX, KX], [-10, 176], lw=HAIR, c=INK, ls=(0, (12, 3, 2, 3)))
ex.text(KX + 2, 178, "KINGPIN AXIS", fontproperties=LABEL, fontsize=5.4, color=INK, ha="left")
ex.text(16, 178, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.4, color=GREY, ha="left")

# battery behind (dashed)
ex.add_patch(Rectangle((0, BELLY), BAT_X / 2, BAT_Z, fc=INK, alpha=0.05, ec=INK, lw=THIN,
                       ls=(0, (3, 2))))

# tire + travel ghosts
for dz in (-DROOP, BUMP):
    ex.add_patch(FancyBboxPatch((T_IN, dz + 0.5), TIRE_W, TIRE_D - 1,
                                boxstyle="round,pad=0,rounding_size=8", fc="none",
                                ec=GREY, lw=HAIR, ls=(0, (2, 2))))
ex.add_patch(FancyBboxPatch((T_IN, 0), TIRE_W, TIRE_D, boxstyle="round,pad=0,rounding_size=8",
                            fc=INK, ec=INK, lw=THIN))
for z in np.arange(8, TIRE_D - 6, 6):
    ex.plot([T_IN, T_OUT], [z, z], lw=HAIR, c=PAPER, alpha=0.25)
# hub pocket opened up to show what sits inside it
ex.add_patch(Rectangle((T_IN, AXLE - HUB_D / 2), HEX_IN, HUB_D, fc=PAPER, ec=INK, lw=THIN,
                       ls=(0, (2, 1.5)), zorder=2))
ex.add_patch(Rectangle((KX, AXLE - 6), GBX_FACE, 12, fc=PAPER, ec=INK, lw=THIN, zorder=3))
# motor
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), TAIL_X, MOTOR_D, fc=PAPER, ec=INK, lw=MED,
                       zorder=3))
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), 10, MOTOR_D, fc=INK, alpha=0.18, ec="none",
                       zorder=3))
# knuckle plate + joints (steers)
ex.add_patch(Polygon([[KX, LO_Z - 4], [KX + 5, LO_Z - 4], [KX + 5, UP_Z + 4], [KX, UP_Z + 4]],
                     closed=True, fc=VERM, alpha=0.3, ec=VERM, lw=THIN, zorder=4))
for z in (LO_Z, UP_Z):
    ex.add_patch(Circle((KX, z), 3.2, fc=PAPER, ec=INK, lw=THIN, zorder=7))
# steering arm + tie rod (steers) + servo on spine
ex.plot([KX, KX + 2], [TR_Z, TR_Z], lw=MED, c=VERM, zorder=6)


def link_pts(z0, name):
    a, h = map(np.asarray, SHAPES[name])
    return np.column_stack([KX - a, z0 + h])


def swing(pts, dz):
    """Rotate a link about its inner pivot so its knuckle end rises dz."""
    piv = pts[-1]
    phi = np.arcsin(dz / ARM)
    r = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    return (pts - piv) @ r.T + piv


# travel ghosts: links and motor at full bump and full droop
for dz in (-DROOP, BUMP):
    for z0, name, col in ((LO_Z, "lower", STEEL), (UP_Z, "upper", STEEL), (TR_Z, "tie rod", VERM)):
        q = swing(link_pts(z0, name), dz)
        ex.plot(q[:, 0], q[:, 1], lw=HAIR, c=col, ls=(0, (2, 2)), zorder=2)
    ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2 + dz), TAIL_X, MOTOR_D, fc="none",
                           ec=GREY, lw=HAIR, ls=(0, (2, 2)), zorder=2))
tr = link_pts(TR_Z, "tie rod")
ex.plot(tr[:, 0], tr[:, 1], lw=THIN * 1.4, c=VERM, zorder=6)
for x, z in ((PIVOT, TR_Z + RISE), (KX, TR_Z)):
    ex.add_patch(Circle((x, z), 1.8, fc=PAPER, ec=VERM, lw=THIN, zorder=7))
ex.add_patch(Rectangle((PIVOT - 10, TR_Z + RISE - 40), 20, 37, fc=STEEL, alpha=0.12, ec=STEEL, lw=THIN,
                       zorder=4))
ex.plot([PIVOT, PIVOT], [TR_Z + RISE - 3, TR_Z + RISE], lw=MED, c=STEEL, zorder=5)
# arms
for z, name in ((LO_Z, "lower"), (UP_Z, "upper")):
    q = link_pts(z, name)
    ex.plot(q[:, 0], q[:, 1], lw=MED * 1.8, c=STEEL, solid_capstyle="round",
            solid_joinstyle="round", zorder=5)
    ex.add_patch(Circle((PIVOT, z + RISE), 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
# spine + central shock tower
ex.add_patch(Rectangle((0, BELLY), PIVOT + 14, UP_Z - LO_Z + 16, fc=STEEL, alpha=0.05,
                       ec=STEEL, lw=THIN, ls=(0, (4, 2))))
ex.add_patch(Rectangle((0, UP_Z + RISE + 8), TOWER_X + 4, TOWER_Z - UP_Z - RISE, fc=STEEL, alpha=0.05,
                       ec=STEEL, lw=THIN, ls=(0, (4, 2))))
s0 = SHOCK_LO
s1 = np.array([TOWER_X, TOWER_Z])
SHOCK_L = np.linalg.norm(s1 - s0)
u = (s1 - s0) / SHOCK_L; n = np.array([-u[1], u[0]])
body0, body1 = s0 + u * 12, s0 + u * (SHOCK_L * 0.62)
ex.plot(*zip(s0, s1), lw=THIN, c=INK)
ex.add_patch(Polygon([body0 + n * 5, body1 + n * 5, body1 - n * 5, body0 - n * 5], fc=PAPER,
                     ec=INK, lw=THIN, zorder=6))
for k in np.linspace(0.12, 0.9, 11):
    p = body0 + (body1 - body0) * k
    ex.plot(*zip(p + n * 5, p - n * 5), lw=HAIR, c=INK, zorder=7)
for p in (s0, s1):
    ex.add_patch(Circle(p, 2.4, fc=PAPER, ec=INK, lw=THIN, zorder=8))


def elab(x, y, s, tx, ty, col=INK, ha="left"):
    ex.annotate(s, xy=(x, y), xytext=(tx, ty), fontproperties=LABEL_M, fontsize=5.6, color=col,
                ha=ha, va="center",
                arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=0),
                bbox=dict(fc=PAPER, ec="none", pad=0.6))


elab(KX + 3, UP_Z - 10, "KNUCKLE PLATE  (in hub, steers)", 150, 150, col=VERM, ha="center")
elab(T_OUT - 4, 110, f"TIRE  {TIRE_D:.0f} × {TIRE_W:.0f}", 184, 132, ha="center")
elab(TAIL_E + 30, AXLE + 12.5, f"JGA25-370 + ENC  {MOTOR_BODY:.0f}", 98, 140, ha="center")
elab(60, LO_Z, f"LOWER ARM  {ARM:.0f}", 70, 14, col=STEEL, ha="center")
elab(KX - 60, UP_Z + RISE, f"UPPER ARM  {ARM:.0f}  (boomerang)", 98, 132, col=STEEL,
     ha="center")
elab(PIVOT + 20, TR_Z + shape(ARM - 20, "tie rod"), "TIE ROD", -12, 112, col=VERM,
     ha="center")
elab(PIVOT - 10, TR_Z + RISE - 30, "MG996R  (spine)", -4, 30, col=STEEL, ha="center")
elab((s0[0] + s1[0]) / 2 + 6, (s0[1] + s1[1]) / 2, f"SHOCK  {SHOCK_L:.0f}", 70, 160, ha="center")

ex.plot([KX, KX], [-2, -12], lw=HAIR, c=VERM)
ex.plot([TC, TC], [-2, -12], lw=HAIR, c=VERM)
ex.text((KX + TC) / 2 + 12, -13, f"SCRUB {SCRUB:.0f}", fontproperties=MONO, fontsize=5.4,
        color=VERM, ha="left", va="top")

for z, t in ((LO_Z, f"{LO_Z:.0f}"), (AXLE, f"{AXLE:.0f}"), (UP_Z, f"{UP_Z:.0f}"),
             (TIRE_D, f"{TIRE_D:.0f}"), (TOWER_Z, f"{TOWER_Z:.0f}")):
    ex.plot([-34, -28], [z, z], lw=HAIR, c=GREY)
    ex.text(-27, z + 2, t, fontproperties=MONO, fontsize=5.4, color=GREY, va="bottom")
ex.plot([-31, -31], [0, TOWER_Z], lw=HAIR, c=GREY)

ex.text(-38, 204, "ELEVATION", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        ha="left")
ex.text(-38, 190, "front-left corner, looking rearward  ·  hub pocket cut open  ·  mm",
        fontproperties=LABEL, fontsize=6.5, color=GREY, va="top")

# ============================ SCHEDULE + TITLE ===============================
tb = fig.add_axes([0.6, 0.085, 0.355, 0.31], facecolor="none")
tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")

rows = [
    ("Track (tire c-c) / wheelbase", f"{T:.0f} / {WB:.0f} mm"),
    ("Kingpin c-c / scrub radius", f"{2 * KP_X:.0f} / {SCRUB:.0f} mm"),
    ("Spin-in-place angle  /  clearance swept", f"{SPIN:.1f}°  /  ±{STEER:.0f}°"),
    ("Battery to sweep / to arm pivot", f"{CLEAR:.1f} / {PIV_GAP:.1f} mm"),
    ("Motor tail to servo (swept)", f"{SERVO_CLEAR:.1f} mm"),
    (f"Motor to tie rod / upper / lower, +{BUMP:.0f} −{DROOP:.0f} travel",
     f"{TRAVEL_GAPS['tie rod']:.1f} / {TRAVEL_GAPS['upper arm']:.1f} / "
     f"{TRAVEL_GAPS['lower arm']:.1f} mm"),
    ("Arms = tie rod / inner rise / belly height", f"{ARM:.0f} / {RISE:.0f} / {BELLY:.0f} mm"),
    ("Wheel / motor + enc, measured", f"{TIRE_D:.0f}×{TIRE_W:.0f} / {MOTOR_BODY:.0f} mm"),
]
tb.text(0, 97, "SCHEDULE", fontproperties=LABEL_M, fontsize=7, color=INK, va="top")
tb.plot([0, 100], [91, 91], lw=THIN, c=INK)
for i, (k, v) in enumerate(rows):
    y = 85 - i * 6.4
    warn = k.startswith(("Battery", "Motor"))
    tb.text(0, y, k, fontproperties=LABEL, fontsize=7, color=INK, va="center")
    tb.text(100, y, v, fontproperties=MONO, fontsize=6.8, color=VERM if warn else INK,
            ha="right", va="center")
    tb.plot([0, 100], [y - 3.2, y - 3.2], lw=HAIR, c=GREY, alpha=0.5)

tb.add_patch(Rectangle((0, 0), 100, 26, fill=False, lw=THIN, ec=INK))
tb.plot([64, 64], [0, 26], lw=HAIR, c=INK)
tb.text(3, 17.5, "ggGridRunner", fontproperties=SERIF, fontsize=21, color=INK, va="center")
tb.text(3, 6.5, "corner & chassis layout  —  study 03", fontproperties=SERIF_I, fontsize=10,
        color=GREY, va="center")
for k, (a, b) in enumerate((("SHEET", "1 / 1"), ("DATE", "2026-09-25"), ("STATUS", "DRAFT"))):
    y = 20.5 - k * 7.5
    tb.text(67, y, a, fontproperties=LABEL, fontsize=5.6, color=GREY, va="center")
    tb.text(97, y, b, fontproperties=MONO, fontsize=6.2, color=INK, va="center", ha="right")

for j, (col, txt, style) in enumerate(((INK, "wheel · motor · shock", "-"),
                                       (VERM, "steers: sweep · knuckle · tie rod", "-"),
                                       (STEEL, "suspension · chassis · servo", "--"))):
    x = j * 34
    tb.plot([x, x + 6], [33, 33], lw=MED, c=col, ls=style)
    tb.text(x + 8, 33, txt, fontproperties=LABEL, fontsize=5.8, color=INK, va="center")

fig.savefig(os.path.join(os.path.dirname(__file__), "corner_layout_study.png"), facecolor=PAPER)
print(f"T {T:.0f}  kingpin x {KP_X:.0f}  scrub {SCRUB:.1f}  spin {SPIN:.2f}  battery {CLEAR:.1f}  "
      f"pivot gap {PIV_GAP:.1f}  servo {SERVO_CLEAR:.1f}  fore-aft {FA_GAP:.1f}  "
      f"tail R {TAIL_R:.1f}  shock {SHOCK_L:.0f}  gaps {TRAVEL_GAPS}  rod-arm {ROD_TO_ARM:.1f}")
