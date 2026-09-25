"""ggGridRunner corner & chassis layout study, option B: plan + front elevation.

The kingpin sits just inboard of the wheel and passes through the part of the motor that
sticks out past the wheel. A knuckle clamps the motor and steers inside a C-shaped upright
(nylon-bushed pin below the motor, MG996R above it), with the arms at about axle height.
Draws the +/-STEER sweep of each corner, solves the spin-in-place angle (the contact
point moves as the wheel steers because of the scrub offset), and reports battery
clearance. Wheel and motor dimensions are measured.

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
T, WB = 320.0, 230.0            # track (tire centre c-c), wheelbase (axle c-c)
TIRE_D, TIRE_W = 120.0, 42.0    # measured
HUB_D = 81.0                    # hub pocket diameter (measured)
HEX_IN = 33.0                   # hex mating face -> wheel inner (motor-side) face (measured)
GBX_FACE = 22.0                 # gearbox face -> hex mating face (measured)
MOTOR_BODY = 61.0               # gearbox + motor 51, + 10 encoder clearance (measured)
MOTOR_TOTAL, MOTOR_D = GBX_FACE + MOTOR_BODY, 25.0  # hex mating face -> encoder back
KP_IN = 18.0                    # kingpin inboard of wheel inner face (knuckle clamp room)
STEER = 50.0                    # mechanical clearance, deg
BAT_L, BAT_W, BAT_Z = 144.0, 65.0, 36.0   # 4S4P brick: two layers of 8 cells along BAT_L
BAT_LONG = True                 # True: long side fore-aft; False: crosswise
BAT_X, BAT_Y = (BAT_W, BAT_L) if BAT_LONG else (BAT_L, BAT_W)  # plan size (X lateral, Y fwd)
MOD_Y0 = BAT_Y / 2 + 4          # front/rear spine modules start just past the battery
RIDE = 36.0                     # battery underside / chassis belly
ENV = 140.0
PIVOT_LO, PIVOT_UP = 22.0, 28.0  # inner arm pivots, from chassis centreline
LO_SPREAD, UP_SPREAD = 20.0, 15.0  # half-spread of the wishbone inner pivots, fore-aft

# Drawing coords: X lateral, Y forward (plan reads forward-up). Rover frame on the sheet:
# +X forward, +Y left, +Z up.
# Local corner frame: origin at kingpin, +x inboard, +y forward.
SCRUB = KP_IN + TIRE_W / 2      # kingpin -> tire centre (ground-level scrub radius)
TIRE_IN, TIRE_OUT = -KP_IN, -(KP_IN + TIRE_W)
HEX_X = TIRE_IN - HEX_IN        # hex mating face
TAIL_X = HEX_X + MOTOR_TOTAL    # encoder back
KP_X = T / 2 - SCRUB            # kingpin distance from chassis centreline
UPR_BACK = float(np.ceil(np.hypot(TAIL_X, MOTOR_D / 2) + 6))  # clear tail sweep
ARM_LO = KP_X - UPR_BACK - PIVOT_LO
ARM_UP = KP_X - UPR_BACK - PIVOT_UP
AXLE = TIRE_D / 2


def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


TIRE = rect(TIRE_OUT, TIRE_IN, -TIRE_D / 2, TIRE_D / 2)
MOTOR = rect(TIRE_IN, TAIL_X, -MOTOR_D / 2, MOTOR_D / 2)      # visible part
MOTOR_HID = rect(HEX_X, TIRE_IN, -MOTOR_D / 2, MOTOR_D / 2)   # inside hub pocket
HUB = rect(TIRE_IN - HEX_IN, TIRE_IN, -HUB_D / 2, HUB_D / 2)
KNUCKLE = rect(-12, 10, -17, 17)
UPRIGHT = rect(UPR_BACK - 4, UPR_BACK + 4, -14, 14)
SERVO = rect(-10, 30.5, -10, 10)                              # MG996R, shaft on kingpin


def place(poly, kx, ky, ang, mirror):
    """Local corner frame -> chassis frame. Left corners have +x inboard = +X."""
    p = poly.copy()
    a = np.radians(ang)
    r = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    p = p @ r.T
    if mirror:
        p[:, 0] *= -1
    p[:, 0] += kx
    p[:, 1] += ky
    return p


def spin_angle():
    """Steer angle at the front-left wheel that makes it roll tangent to a spin about the
    chassis centre. Contact point = tire centre, which swings about the kingpin."""
    kx, ky = -KP_X, WB / 2
    best, err = 0.0, 1e9
    for ang in np.linspace(-STEER, STEER, 20001):
        a = np.radians(ang)
        cx = kx + (-SCRUB) * np.cos(a)
        cy = ky + (-SCRUB) * np.sin(a)
        hx, hy = -np.sin(a), np.cos(a)          # rolling direction
        e = abs(cx * hx + cy * hy) / np.hypot(cx, cy)
        if e < err:
            best, err, cp = ang, e, (cx, cy)
    return best, cp


def edge_points(poly, n=80):
    return np.vstack([np.linspace(poly[i], poly[(i + 1) % 4], n) for i in range(4)])


def battery_clearance():
    """Smallest gap from any corner sweep (tire + motor) to the battery outline."""
    pts = np.vstack([edge_points(TIRE), edge_points(MOTOR)])
    gap = 1e9
    for ang in np.linspace(-STEER, STEER, 401):
        q = place(pts, -KP_X, WB / 2, ang, False)
        dx = np.maximum(np.abs(q[:, 0]) - BAT_X / 2, 0)
        dy = np.maximum(np.abs(q[:, 1]) - BAT_Y / 2, 0)
        gap = min(gap, np.hypot(dx, dy).min())
    return gap


def fore_aft_gap():
    """Smallest gap between front and rear tire sweeps on one side."""
    pts = edge_points(TIRE)
    front = np.vstack([place(pts, -KP_X, WB / 2, a, False) for a in np.linspace(-STEER, STEER, 81)])
    return 2 * front[:, 1].min()


SPIN_RAW, SPIN_CP = spin_angle()
SPIN = abs(SPIN_RAW)
CLEAR = battery_clearance()
FA_GAP = fore_aft_gap()
PIV_BOSS = 5.0                  # printed boss radius round each inner pivot
PIV_GAP = WB / 2 - LO_SPREAD - PIV_BOSS - BAT_Y / 2
SWEEP_R = np.hypot(-TIRE_OUT, TIRE_D / 2)
TAIL_R = np.hypot(TAIL_X, MOTOR_D / 2)

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

# rails, modules, cradles (chassis, steel)
for sx in (-1, 1):
    ax.add_patch(Rectangle((sx * 52 - 4, -170), 8, 340, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
    ax.add_patch(Rectangle((sx * (BAT_X / 2 + 8) - 7, -BAT_Y / 2 + 4), 14, BAT_Y - 8,
                           fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
MOD_L = WB / 2 + 55 - MOD_Y0
for yc in (MOD_Y0 + MOD_L / 2, -(MOD_Y0 + MOD_L / 2)):
    ax.add_patch(Rectangle((-ENV / 2, yc - MOD_L / 2), ENV, MOD_L, fc=STEEL, alpha=0.05,
                           ec="none"))
    ax.add_patch(Rectangle((-ENV / 2, yc - MOD_L / 2), ENV, MOD_L, fc="none", ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))

# battery
bx0, by0 = -BAT_X / 2, -BAT_Y / 2
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc=INK, alpha=0.06, ec="none"))
for i in range(8):
    if BAT_LONG:
        cell = (bx0 + 0.8, by0 + i * 18 + 0.8), BAT_X - 1.6, 16.4
    else:
        cell = (bx0 + i * 18 + 0.8, by0 + 0.8), 16.4, BAT_Y - 1.6
    ax.add_patch(Rectangle(*cell, fc="none", ec=INK, lw=HAIR))
ax.add_patch(Rectangle((bx0, by0), BAT_X, BAT_Y, fc="none", ec=INK, lw=MED))

for sx in (-1, 1):
    for sy in (-1, 1):
        kx, ky, mir = sx * KP_X, sy * WB / 2, sx > 0
        # wishbones (lower wide, upper narrow), chassis -> upright joint
        for piv, spread, lw in ((PIVOT_LO, LO_SPREAD, THIN), (PIVOT_UP, UP_SPREAD, HAIR * 1.4)):
            j = place(np.array([[UPR_BACK, 0.0]]), kx, ky, 0, mir)[0]
            for d in (-1, 1):
                ax.plot([j[0], sx * piv], [j[1], ky + d * spread], lw=lw, c=STEEL)
                ax.add_patch(Circle((sx * piv, ky + d * spread), 2.2, fc=PAPER, ec=STEEL,
                                    lw=THIN, zorder=5))
        # sweeps
        for ang in np.arange(-STEER, STEER + 0.01, 2.5):
            for p in (TIRE, MOTOR):
                ax.add_patch(Polygon(place(p, kx, ky, ang, mir), closed=True, fill=False,
                                     ec=VERM, lw=0.22, alpha=0.55))
        for ang in (-STEER, STEER):
            for p in (TIRE, MOTOR):
                ax.add_patch(Polygon(place(p, kx, ky, ang, mir), closed=True, fill=False,
                                     ec=VERM, lw=THIN))
        # straight-ahead parts
        ax.add_patch(Polygon(place(TIRE, kx, ky, 0, mir), fc=INK, ec=INK, lw=THIN))
        ax.add_patch(Polygon(place(HUB, kx, ky, 0, mir), fc="none", ec=PAPER, lw=HAIR,
                             ls=(0, (2, 1.5))))
        ax.add_patch(Polygon(place(MOTOR_HID, kx, ky, 0, mir), fc="none", ec=PAPER, lw=HAIR,
                             ls=(0, (2, 1.5))))
        ax.add_patch(Polygon(place(MOTOR, kx, ky, 0, mir), fc=PAPER, ec=INK, lw=MED))
        ax.add_patch(Polygon(place(KNUCKLE, kx, ky, 0, mir), fc=VERM, alpha=0.18, ec=VERM,
                             lw=THIN, zorder=4))
        ax.add_patch(Polygon(place(UPRIGHT, kx, ky, 0, mir), fc=STEEL, alpha=0.25, ec=STEEL,
                             lw=THIN, zorder=4))
        ax.add_patch(Polygon(place(SERVO, kx, ky, 0, mir), fc="none", ec=STEEL, lw=THIN,
                             ls=(0, (3, 1.5)), zorder=4))
        ax.add_patch(Circle((kx, ky), 4, fc=PAPER, ec=INK, lw=THIN, zorder=6))
        ax.plot([kx - 8, kx + 8], [ky, ky], lw=HAIR, c=INK, zorder=7)
        ax.plot([kx, kx], [ky - 8, ky + 8], lw=HAIR, c=INK, zorder=7)
        # spin direction at the contact point
        cp = np.array([sx * -SPIN_CP[0], sy * SPIN_CP[1]])
        tang = np.array([-cp[1], cp[0]]) / np.hypot(*cp)
        ax.annotate("", xy=cp + tang * 30, xytext=cp,
                    arrowprops=dict(arrowstyle="-|>", lw=THIN, color=PAPER, mutation_scale=7),
                    zorder=8)

# spin angle arc at FR kingpin
ax.add_patch(Arc((KP_X, WB / 2), 50, 50, theta1=90, theta2=90 + SPIN, lw=THIN, ec=INK, zorder=9))
ax.text(KP_X - 14, WB / 2 + 34, f"{SPIN:.1f}°", fontproperties=MONO, fontsize=6.5, color=INK,
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

# battery clearance callout
ax.text(-150, 12, f"battery to sweep  {CLEAR:.1f} mm\nbattery to arm pivot  {PIV_GAP:.1f} mm",
        fontproperties=MONO, fontsize=6.5, color=VERM, ha="center", va="top", linespacing=1.6,
        bbox=dict(fc=PAPER, ec="none", pad=0.6))


def lab(a, x, y, s, col=INK, ha="center", size=6.2, fp=LABEL_M):
    a.text(x, y, s, fontproperties=fp, fontsize=size, color=col, ha=ha, va="center",
           bbox=dict(fc=PAPER, ec="none", pad=0.8, alpha=0.92))


ax.text(0, 0, "BATTERY  4S4P  144 × 65 × 36", fontproperties=LABEL_M, fontsize=6.4, color=INK,
        ha="center", va="center", rotation=90 if BAT_LONG else 0,
        bbox=dict(fc=PAPER, ec="none", pad=0.8, alpha=0.92))
for sy, name in ((1, "FRONT MODULE  (spine)"), (-1, "REAR MODULE  (spine)")):
    ax.text(0, sy * (WB / 2 + 42), name, fontproperties=LABEL_M, fontsize=6.2, color=STEEL,
            ha="center", va="center", bbox=dict(fc=PAPER, ec="none", pad=0.8))
lab(ax, 52, 60, "RAIL", col=STEEL, size=5.6)
lab(ax, BAT_X / 2 + 8, -BAT_Y / 2 - 6, "CRADLE", col=STEEL, size=5.6)
lab(ax, -BAT_X / 2 - 8, -BAT_Y / 2 - 6, "CRADLE", col=STEEL, size=5.6)
ax.text(-R_spin - 3, 0, "SPIN-IN-PLACE CIRCLE", fontproperties=LABEL_M, fontsize=5.6,
        color=GREY, ha="center", va="center", rotation=90, bbox=dict(fc=PAPER, ec="none", pad=0.8))
lab(ax, -150, 20, f"±{STEER:.0f}° SWEEP", col=VERM, size=5.8)


ax.text(0.005, 0.995, "PLAN", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        transform=ax.transAxes)
ax.text(0.005, 0.955, "view from above  ·  true proportion  ·  mm  ·  forward up the page  ·  +X forward  ·  +Y left  ·  +Z up",
        fontproperties=LABEL, fontsize=6.5, color=GREY, va="top", transform=ax.transAxes)

# ============================ FRONT ELEVATION ================================
ex = fig.add_axes([0.6, 0.43, 0.37, 0.49], facecolor="none")
ex.set_aspect("equal"); ex.axis("off")
ex.set_xlim(-36, 202); ex.set_ylim(-26, 205)

KX = KP_X
TC = T / 2
T_IN, T_OUT = TC - TIRE_W / 2, TC + TIRE_W / 2
HEX_E = T_IN + HEX_IN
TAIL_E = HEX_E - MOTOR_TOTAL
LA_Z, UA_Z = 38.0, 87.0         # lower / upper arm joint heights at the upright
UB = KX - UPR_BACK

# ground
ex.plot([-34, 200], [0, 0], lw=MED, c=INK)
for x in np.arange(-30, 201, 5):
    ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
ex.plot([0, 0], [-10, 196], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
ex.plot([KX, KX], [-10, 186], lw=HAIR, c=INK, ls=(0, (12, 3, 2, 3)))
ex.text(KX + 2, 188, "KINGPIN AXIS", fontproperties=LABEL, fontsize=5.4, color=INK, ha="left")
ex.text(1.5, 176, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.4, color=GREY, ha="left")

# battery behind (dashed)
ex.add_patch(Rectangle((0, RIDE), BAT_X / 2, BAT_Z, fc=INK, alpha=0.05, ec=INK, lw=THIN,
                       ls=(0, (3, 2))))
ex.text(3, RIDE + BAT_Z - 3, "BATTERY (behind)", fontproperties=LABEL, fontsize=5.2,
        color=INK, ha="left", va="top")

# travel ghosts + tire
for dz in (-12.5, 12.5):
    ex.add_patch(FancyBboxPatch((T_IN, dz + 0.5), TIRE_W, TIRE_D - 1,
                                boxstyle="round,pad=0,rounding_size=8", fc="none",
                                ec=GREY, lw=HAIR, ls=(0, (2, 2))))
ex.add_patch(FancyBboxPatch((T_IN, 0), TIRE_W, TIRE_D, boxstyle="round,pad=0,rounding_size=8",
                            fc=INK, ec=INK, lw=THIN))
for z in np.arange(8, TIRE_D - 6, 6):
    ex.plot([T_IN, T_OUT], [z, z], lw=HAIR, c=PAPER, alpha=0.3)
# hub pocket + motor inside it
ex.add_patch(Rectangle((T_IN, AXLE - HUB_D / 2), HEX_IN, HUB_D, fc="none", ec=PAPER, lw=HAIR,
                       ls=(0, (2, 1.5))))
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), MOTOR_TOTAL, MOTOR_D, fc="none", ec=PAPER,
                       lw=HAIR, ls=(0, (2, 1.5))))
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), T_IN - TAIL_E, MOTOR_D, fc=PAPER, ec=INK,
                       lw=MED, zorder=3))
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), 12, MOTOR_D, fc=INK, alpha=0.18, ec="none",
                       zorder=3))

# knuckle (steers): clamp ring around motor + top/bottom bosses
kn = np.array([[KX - 12, AXLE - 20], [KX + 10, AXLE - 20], [KX + 10, AXLE + 20],
               [KX - 12, AXLE + 20]])
ex.add_patch(Polygon(kn, closed=True, fc=VERM, alpha=0.14, ec=VERM, lw=THIN, zorder=4))
ex.plot([KX - 12, KX + 10], [AXLE, AXLE], lw=HAIR, c=VERM, zorder=4)
# upright C (moves with suspension, does not steer)
UZ0, UZ1 = LA_Z - 4, AXLE + 27
upr = np.array([[KX + 8, UZ0], [KX + 8, UZ0 + 6], [UB + 4, UZ0 + 6], [UB + 4, UZ1 - 6],
                [KX + 8, UZ1 - 6], [KX + 8, UZ1], [UB - 4, UZ1], [UB - 4, UZ0]])
ex.add_patch(Polygon(upr, closed=True, fc=STEEL, alpha=0.12, ec=STEEL, lw=MED, zorder=5))
# pins + bushings on the kingpin axis
for z0, z1 in ((UZ0 - 2, AXLE - 20), (AXLE + 20, UZ1 + 2)):
    ex.plot([KX, KX], [z0, z1], lw=MED * 1.6, c=INK, solid_capstyle="butt", zorder=6)
for zb in (UZ0 + 3, UZ1 - 3):
    ex.add_patch(Rectangle((KX - 5, zb - 3), 10, 6, fc=INK, alpha=0.3, ec=INK, lw=HAIR, zorder=6))
# servo on top of the upright, body running inboard
SV0 = UZ1 + 1
ex.add_patch(Rectangle((KX - 30.5, SV0), 40.5, 37, fc=PAPER, ec=INK, lw=THIN, zorder=5))
ex.add_patch(Rectangle((KX - 37.5, SV0 + 26), 54.5, 2.5, fc=PAPER, ec=INK, lw=THIN, zorder=5))

# arms
for (piv, z) in ((PIVOT_LO, LA_Z), (PIVOT_UP, UA_Z)):
    ex.plot([piv, UB], [z, z], lw=MED * 1.6, c=STEEL, solid_capstyle="round")
    for x in (piv, UB):
        ex.add_patch(Circle((x, z), 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
# spine module + shock tower
ex.add_patch(Rectangle((0, RIDE - 4), 32, 64, fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN,
                       ls=(0, (4, 2))))
tower = np.array([[4, 96], [16, 96], [16, 150], [4, 150]])
ex.add_patch(Polygon(tower, closed=True, fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN,
                     ls=(0, (4, 2))))
s0, s1 = np.array([UB - 12, LA_Z]), np.array([10.0, 144.0])
u = (s1 - s0) / np.linalg.norm(s1 - s0); n = np.array([-u[1], u[0]])
body0, body1 = s0 + u * 14, s0 + u * 70
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


elab(KX - 10, SV0 + 20, "MG996R  (on upright)", 118, 162)
elab(UB - 4, AXLE + 10, "UPRIGHT  C", 50, 118, col=STEEL, ha="center")
elab(KX + 4, UZ0 + 3, "NYLON BUSHINGS", 106, 22, col=INK, ha="center")
elab(KX + 9, AXLE + 16, "KNUCKLE  (steers)", 150, 142, col=VERM, ha="center")
elab(T_OUT - 4, 108, f"TIRE  {TIRE_D:.0f} × {TIRE_W:.0f}", 186, 150, ha="center")
elab(TAIL_E + 6, AXLE - 12.5, f"JGA25-370 + ENCODER  {MOTOR_TOTAL:.0f}  (33 in hub)", 60, 12)
elab(46, LA_Z, f"LOWER ARM  {ARM_LO:.0f}", 44, 24, col=STEEL, ha="center")
elab(66, UA_Z, f"UPPER ARM  {ARM_UP:.0f}", 64, 102, col=STEEL, ha="center")
elab(30, 120, "SHOCK", 18, 172, col=INK, ha="center")

# scrub dimension at ground
ex.plot([KX, KX], [-2, -12], lw=HAIR, c=VERM)
ex.plot([TC, TC], [-2, -12], lw=HAIR, c=VERM)
ex.annotate("", xy=(KX, -10), xytext=(TC, -10),
            arrowprops=dict(arrowstyle="<|-|>", lw=HAIR, color=VERM, mutation_scale=5))
ex.text((KX + TC) / 2, -13, f"SCRUB {SCRUB:.0f}", fontproperties=MONO, fontsize=5.4, color=VERM,
        ha="center", va="top")

# heights
top = SV0 + 37
for z, t in ((AXLE, f"{AXLE:.0f}"), (TIRE_D, f"{TIRE_D:.0f}"), (top, f"{top:.0f}")):
    ex.plot([-30, -24], [z, z], lw=HAIR, c=GREY)
    ex.text(-23, z + (3 if z != TIRE_D else -3), t, fontproperties=MONO, fontsize=5.4, color=GREY,
            va="bottom" if z != TIRE_D else "top")
ex.plot([-27, -27], [0, max(top, TIRE_D)], lw=HAIR, c=GREY)

ex.text(-34, 204, "ELEVATION", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        ha="left")
ex.text(-34, 190, "front-left corner, looking rearward  ·  mm", fontproperties=LABEL,
        fontsize=6.5, color=GREY, va="top")

# ============================ SCHEDULE + TITLE ===============================
tb = fig.add_axes([0.6, 0.085, 0.355, 0.31], facecolor="none")
tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")

rows = [
    ("Track (tire c-c) / wheelbase", f"{T:.0f} / {WB:.0f} mm"),
    ("Kingpin c-c / scrub radius", f"{2 * KP_X:.0f} / {SCRUB:.0f} mm"),
    ("Spin-in-place angle", f"{SPIN:.1f}°"),
    ("Steering: clearance swept", f"±{STEER:.0f}°"),
    ("Battery to sweep / to arm pivot", f"{CLEAR:.1f} / {PIV_GAP:.1f} mm"),
    ("Arms lower / upper", f"{ARM_LO:.0f} / {ARM_UP:.0f} mm"),
    ("Wheel / motor+hex, measured", f"{TIRE_D:.0f}×{TIRE_W:.0f} / {MOTOR_TOTAL:.0f} mm"),
]
tb.text(0, 97, "SCHEDULE", fontproperties=LABEL_M, fontsize=7, color=INK, va="top")
tb.plot([0, 100], [91, 91], lw=THIN, c=INK)
for i, (k, v) in enumerate(rows):
    y = 85 - i * 7.2
    warn = "Battery" in k
    tb.text(0, y, k, fontproperties=LABEL, fontsize=7, color=INK, va="center")
    tb.text(100, y, v, fontproperties=MONO, fontsize=6.8, color=VERM if warn else INK,
            ha="right", va="center")
    tb.plot([0, 100], [y - 3.6, y - 3.6], lw=HAIR, c=GREY, alpha=0.5)

tb.add_patch(Rectangle((0, 0), 100, 26, fill=False, lw=THIN, ec=INK))
tb.plot([64, 64], [0, 26], lw=HAIR, c=INK)
tb.text(3, 17.5, "ggGridRunner", fontproperties=SERIF, fontsize=21, color=INK, va="center")
tb.text(3, 6.5, "corner & chassis layout  —  study 02 (B)", fontproperties=SERIF_I, fontsize=10,
        color=GREY, va="center")
for k, (a, b) in enumerate((("SHEET", "1 / 1"), ("DATE", "2026-09-25"), ("STATUS", "DRAFT"))):
    y = 20.5 - k * 7.5
    tb.text(67, y, a, fontproperties=LABEL, fontsize=5.6, color=GREY, va="center")
    tb.text(97, y, b, fontproperties=MONO, fontsize=6.2, color=INK, va="center", ha="right")

for j, (col, txt, style) in enumerate(((INK, "wheel · motor", "-"),
                                       (VERM, "steers: sweep · knuckle", "-"),
                                       (STEEL, "suspension · chassis", "--"))):
    x = j * 34
    tb.plot([x, x + 6], [33, 33], lw=MED, c=col, ls=style)
    tb.text(x + 8, 33, txt, fontproperties=LABEL, fontsize=5.8, color=INK, va="center")

fig.savefig(os.path.join(os.path.dirname(__file__), "corner_layout_study.png"), facecolor=PAPER)
print(f"kingpin x {KP_X:.1f}  scrub {SCRUB:.1f}  spin {SPIN:.2f}  clearance {CLEAR:.1f}  "
      f"pivot gap {PIV_GAP:.1f}  fore-aft gap {FA_GAP:.1f}  arms {ARM_LO:.0f}/{ARM_UP:.0f}  sweep R {SWEEP_R:.1f}  "
      f"tail R {TAIL_R:.1f}  top {top:.0f}")
