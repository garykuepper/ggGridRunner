"""ggGridRunner corner & chassis layout study (study 05): plan + front elevation.

Corner with a separate upright: the knuckle plate bolts to the gearbox face inside the hub
pocket and steers on a kingpin held by a C-shaped upright (prongs above and below the motor,
back outside the motor's sweep). Straight parallel arms hinge on the upright's back; the MG996R
rides on the upright and steers the knuckle through a 1:1 parallelogram link. See
rover_geometry.py for the numbers and suspension_travel_study.py for the 3D clearance check.

Draws the +/-STEER sweep of each corner, solves the spin-in-place angle (the contact point
swings about the kingpin by the scrub offset), and reports plan clearances.

    python simulation/corner_layout_study.py
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Arc, Polygon

from drafting import *  # noqa: E402,F401,F403  fonts, inks, sheet frame
from rover_geometry import *  # noqa: E402,F401,F403  shared dimensions

# Drawing coords: X lateral, Y forward (plan reads forward-up).
TIRE = rect(TIRE_OUT, TIRE_IN, -TIRE_D / 2, TIRE_D / 2)
HUB = rect(TIRE_IN, POCKET_BOTTOM, -HUB_D / 2, HUB_D / 2)
MOTOR = rect(TIRE_IN, TAIL_X, -MOTOR_D / 2, MOTOR_D / 2)       # visible part
MOTOR_HID = rect(0, TIRE_IN, -MOTOR_D / 2, MOTOR_D / 2)
COUPLER = rect(HEX_X, 0, -6, 6)
PLATE = rect(-5, 0, -22, 22)                                   # knuckle plate (steers)
PRONG = rect(0, UPR_BACK[0], -PRONG_W / 2, PRONG_W / 2)       # upright prongs (plan)
BACK = rect(UPR_BACK[0], UPR_BACK[1], -UPR_BACK_W, UPR_BACK_W)
SERVO = rect(*SERVO_BOX[0], *SERVO_BOX[1])


def place(poly, kx, ky, ang, mirror_x, mirror_y=False):
    """Local corner frame -> chassis frame."""
    p = np.asarray(poly, float).copy()
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


def rect_gap(pts, poly):
    x0, x1 = poly[:, 0].min(), poly[:, 0].max()
    y0, y1 = poly[:, 1].min(), poly[:, 1].max()
    dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
    dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
    return np.hypot(dx, dy).min()


def clearances():
    tire_mot = np.vstack([edge_points(TIRE), edge_points(MOTOR)])
    bat, back = 1e9, 1e9
    bat_poly = rect(-BAT_X / 2, BAT_X / 2, -BAT_Y / 2, BAT_Y / 2)
    for ang in np.linspace(-STEER, STEER, 401):
        q = place(tire_mot, -KP_X, WB / 2, ang, False)
        bat = min(bat, rect_gap(q, bat_poly))
        m = place(edge_points(MOTOR), 0, 0, ang, False)   # upright is fixed in this frame
        back = min(back, rect_gap(m, BACK))
    front = np.vstack([place(edge_points(TIRE), -KP_X, WB / 2, a, False)
                       for a in np.linspace(-STEER, STEER, 81)])
    return bat, back, 2 * front[:, 1].min()


SPIN, SPIN_CP = spin_angle()
CLEAR, BACK_CLEAR, FA_GAP = clearances()
PIV_GAP = WB / 2 - ARM_W_IN - PIV_BOSS - BAT_Y / 2
MOD_Y0 = BAT_Y / 2 + 4

fig = plt.figure(figsize=(17, 11), dpi=200, facecolor=PAPER)
sheet_frame(fig)

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
        P = lambda poly, ang=0.0: place(poly, kx, ky, ang, mx, my)
        # arms: two rails each, upright hinge -> chassis pivots
        for d in (-1, 1):
            seg = P([[HINGE_A, d * ARM_W_OUT], [HINGE_A + ARM, d * ARM_W_IN]])
            ax.plot(seg[:, 0], seg[:, 1], lw=MED, c=STEEL)
            ax.add_patch(Circle(seg[1], 2.2, fc=PAPER, ec=STEEL, lw=THIN, zorder=5))
        for a_ in (HINGE_A, HINGE_A + ARM):
            w = ARM_W_OUT if a_ == HINGE_A else ARM_W_IN
            seg = P([[a_, -w], [a_, w]])
            ax.plot(seg[:, 0], seg[:, 1], lw=HAIR, c=STEEL, ls=(0, (3, 1.5)))
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
        # upright (prongs + back) and servo on it
        ax.add_patch(Polygon(P(PRONG), fc=STEEL, alpha=0.35, ec=STEEL, lw=THIN, zorder=5))
        ax.add_patch(Polygon(P(BACK), fc=STEEL, alpha=0.55, ec=STEEL, lw=THIN, zorder=5))
        ax.add_patch(Polygon(P(SERVO), fc="none", ec=STEEL, lw=THIN, ls=(0, (3, 1.5)),
                             zorder=6))
        # steering link: knuckle arm -> link -> horn
        link = P([[0, 0], [0, STEER_ARM], [SERVO_A, STEER_ARM], [SERVO_A, 0]])
        ax.plot(link[:, 0], link[:, 1], lw=THIN, c=VERM, zorder=7)
        for q in link[1:3]:
            ax.add_patch(Circle(q, 1.8, fc=PAPER, ec=VERM, lw=THIN, zorder=8))
        ax.add_patch(Circle(link[3], 2.6, fc=PAPER, ec=STEEL, lw=THIN, zorder=8))
        ax.add_patch(Circle((kx, ky), 3.5, fc=PAPER, ec=INK, lw=THIN, zorder=9))
        ax.plot([kx - 7, kx + 7], [ky, ky], lw=HAIR, c=INK, zorder=10)
        ax.plot([kx, kx], [ky - 7, ky + 7], lw=HAIR, c=INK, zorder=10)

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
dim(ax, -WB / 2, WB / 2, -226, f"WHEELBASE {WB:.0f}", horiz=False)
dim(ax, -ENV / 2, ENV / 2, -200, f"≤ {ENV:.0f} PRINT ENVELOPE", col=STEEL)

ax.text(-100, 14, f"battery to sweep  {CLEAR:.1f} mm\nbattery to arm pivot  {PIV_GAP:.1f} mm\n"
        f"motor tail to upright  {BACK_CLEAR:.1f} mm",
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
lab(ax, -KP_X + UPR_BACK[1] + 2, WB / 2 - 26, "UPRIGHT", col=STEEL, size=5.6)
lab(ax, -KP_X + SERVO_A / 2, WB / 2 + STEER_ARM + 9, "LINK", col=VERM, size=5.6)
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
ex.set_xlim(-30, 208); ex.set_ylim(-26, 205)

KX = KP_X
T_IN, T_OUT = KX - TIRE_IN, KX - TIRE_OUT
TC = (T_IN + T_OUT) / 2
TAIL_E = KX - TAIL_X
HX = KX - HINGE_A                   # outer hinges, elevation x
PX = PIVOT                          # chassis pivots

ex.plot([-28, 206], [0, 0], lw=MED, c=INK)
for x in np.arange(-24, 207, 5):
    ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
ex.plot([0, 0], [-10, 180], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
ex.plot([KX, KX], [-10, 180], lw=HAIR, c=INK, ls=(0, (12, 3, 2, 3)))
ex.text(KX + 2, 182, "KINGPIN AXIS", fontproperties=LABEL, fontsize=5.4, color=INK, ha="left")
ex.text(16, 172, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.4, color=GREY, ha="left")

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
ex.add_patch(Rectangle((T_IN, AXLE - HUB_D / 2), HEX_IN, HUB_D, fc=PAPER, ec=INK, lw=THIN,
                       ls=(0, (2, 1.5)), zorder=2))
ex.add_patch(Rectangle((KX, AXLE - 6), GBX_FACE, 12, fc=PAPER, ec=INK, lw=THIN, zorder=3))
# motor
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), TAIL_X, MOTOR_D, fc=PAPER, ec=INK, lw=MED,
                       zorder=3))
ex.add_patch(Rectangle((TAIL_E, AXLE - MOTOR_D / 2), 10, MOTOR_D, fc=INK, alpha=0.18, ec="none",
                       zorder=3))
# knuckle plate (steers)
ex.add_patch(Polygon([[KX, KP_LO_Z + 3], [KX + 5, KP_LO_Z + 3], [KX + 5, KP_UP_Z - 3],
                      [KX, KP_UP_Z - 3]], closed=True, fc=VERM, alpha=0.3, ec=VERM, lw=THIN,
                     zorder=4))
# upright: prongs + back (moves with suspension, does not steer)
for z0, z1 in (PRONG_LO_Z, PRONG_UP_Z):
    ex.add_patch(Rectangle((KX - UPR_BACK[0], z0), UPR_BACK[0] + 4, z1 - z0, fc=STEEL, alpha=0.3,
                           ec=STEEL, lw=THIN, zorder=5))
ex.add_patch(Rectangle((KX - UPR_BACK[1], UPR_Z[0]), UPR_BACK[1] - UPR_BACK[0],
                       UPR_Z[1] - UPR_Z[0], fc=STEEL, alpha=0.45, ec=STEEL, lw=THIN, zorder=5))
for z in (KP_LO_Z, KP_UP_Z):
    ex.add_patch(Circle((KX, z), 2.6, fc=PAPER, ec=INK, lw=THIN, zorder=7))
# servo on the upright + link
(sa0, sa1), _, (sz0, sz1) = SERVO_BOX
ex.add_patch(Rectangle((KX - sa1, sz0), sa1 - sa0, sz1 - sz0, fc=PAPER, ec=INK, lw=THIN,
                       zorder=5))
ex.add_patch(Rectangle((KX - sa1 - 7, sz0 + 26), sa1 - sa0 + 14, 2.5, fc=PAPER, ec=INK, lw=THIN,
                       zorder=5))
ex.plot([KX - SERVO_A] * 2, [HORN_Z, sz0], lw=MED, c=INK, zorder=6)
ex.plot([KX - SERVO_A, KX], [HORN_Z, HORN_Z], lw=THIN * 1.4, c=VERM, zorder=6)
for x in (KX - SERVO_A, KX):
    ex.add_patch(Circle((x, HORN_Z), 1.8, fc=PAPER, ec=VERM, lw=THIN, zorder=7))
# arms: straight, parallel, rising to the chassis
for z in (LO_H_Z, UP_H_Z):
    ex.plot([HX, PX], [z, z + RISE], lw=MED * 1.8, c=STEEL, solid_capstyle="round", zorder=5)
    ex.add_patch(Circle((HX, z), 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
    ex.add_patch(Circle((PX, z + RISE), 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
# spine + shock tower
ex.add_patch(Rectangle((0, BELLY), PX + 12, UP_H_Z - LO_H_Z + 16, fc=STEEL, alpha=0.05,
                       ec=STEEL, lw=THIN, ls=(0, (4, 2))))
ex.add_patch(Rectangle((0, UP_H_Z + RISE + 8), TOWER_X + 4, SHOCK_TOP_Z - UP_H_Z - RISE,
                       fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN, ls=(0, (4, 2))))
s0 = np.array([HX - SHOCK_ON_ARM, LO_H_Z + RISE * SHOCK_ON_ARM / ARM])
s1 = np.array([TOWER_X, SHOCK_TOP_Z])
u = (s1 - s0) / SHOCK_LEN; n = np.array([-u[1], u[0]])
body0, body1 = s0 + u * 12, s0 + u * (SHOCK_LEN * 0.62)
ex.plot(*zip(s0, s1), lw=THIN, c=INK)
ex.add_patch(Polygon([body0 + n * SHOCK_R, body1 + n * SHOCK_R, body1 - n * SHOCK_R,
                      body0 - n * SHOCK_R], fc=PAPER, ec=INK, lw=THIN, zorder=6))
for k in np.linspace(0.12, 0.9, 11):
    p = body0 + (body1 - body0) * k
    ex.plot(*zip(p + n * SHOCK_R, p - n * SHOCK_R), lw=HAIR, c=INK, zorder=7)
for p in (s0, s1):
    ex.add_patch(Circle(p, 2.4, fc=PAPER, ec=INK, lw=THIN, zorder=8))


def elab(x, y, s, tx, ty, col=INK, ha="center"):
    ex.annotate(s, xy=(x, y), xytext=(tx, ty), fontproperties=LABEL_M, fontsize=5.6, color=col,
                ha=ha, va="center",
                arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=0),
                bbox=dict(fc=PAPER, ec="none", pad=0.6))


elab(KX + 3, KP_LO_Z + 10, "KNUCKLE PLATE  (steers)", 176, 26, col=VERM)
elab(T_OUT - 4, 110, f"TIRE  {TIRE_D:.0f} × {TIRE_W:.0f}", 190, 136)
elab(KX - SERVO_A + 14, sz1 - 8, "MG996R  (on upright)", 150, 160)
elab(KX - UPR_BACK[1] + 2, UPR_Z[0] + 8, "UPRIGHT", 92, 12, col=STEEL)
elab(KX - SERVO_A / 2, HORN_Z, "LINK  1:1", 118, 112, col=VERM)
elab(TAIL_E + 30, AXLE - MOTOR_D / 2, f"JGA25-370 + ENC  {MOTOR_BODY:.0f}", 60, 24)
elab((HX + PX) / 2, UP_H_Z + RISE / 2, f"UPPER ARM  {ARM:.0f}", 36, 142, col=STEEL)
elab((HX + PX) / 2 + 6, LO_H_Z + RISE / 2 + 1, f"LOWER ARM  {ARM:.0f}", 16, 22, col=STEEL)
elab((s0[0] + s1[0]) / 2 - 4, (s0[1] + s1[1]) / 2, f"SHOCK  {SHOCK_LEN:.0f}", 64, 172)

ex.plot([KX, KX], [-2, -12], lw=HAIR, c=VERM)
ex.plot([TC, TC], [-2, -12], lw=HAIR, c=VERM)
ex.text(TC + 3, -13, f"SCRUB {SCRUB:.0f}", fontproperties=MONO, fontsize=5.4, color=VERM,
        ha="left", va="top")

for z in (BELLY, AXLE, TIRE_D, SHOCK_TOP_Z):
    ex.plot([-26, -20], [z, z], lw=HAIR, c=GREY)
    ex.text(-19, z + 2, f"{z:.0f}", fontproperties=MONO, fontsize=5.4, color=GREY, va="bottom")
ex.plot([-23, -23], [0, SHOCK_TOP_Z], lw=HAIR, c=GREY)

ex.text(-28, 204, "ELEVATION", fontproperties=SERIF, fontsize=17, color=INK, va="top",
        ha="left")
ex.text(-28, 190, "front-left corner, looking rearward  ·  hub pocket cut open  ·  mm",
        fontproperties=LABEL, fontsize=6.5, color=GREY, va="top")

# ============================ SCHEDULE + TITLE ===============================
tb = fig.add_axes([0.6, 0.085, 0.355, 0.31], facecolor="none")
tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")

rows = [
    ("Track (tire c-c) / wheelbase", f"{T:.0f} / {WB:.0f} mm"),
    ("Kingpin c-c / scrub radius", f"{2 * KP_X:.0f} / {SCRUB:.0f} mm"),
    ("Spin-in-place angle  /  clearance swept", f"{SPIN:.1f}°  /  ±{STEER:.0f}°"),
    ("Battery to sweep / to arm pivot", f"{CLEAR:.1f} / {PIV_GAP:.1f} mm"),
    ("Motor tail to upright back (swept)", f"{BACK_CLEAR:.1f} mm"),
    ("Arms (straight, parallel) / rise", f"{ARM:.0f} / {RISE:.0f} mm"),
    ("Belly height / shock tower top", f"{BELLY:.0f} / {SHOCK_TOP_Z:.0f} mm"),
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

title_block(tb, "corner & chassis layout  —  study 05", "2026-09-25")

for j, (col, txt, style) in enumerate(((INK, "wheel · motor · shock", "-"),
                                       (VERM, "steers: sweep · knuckle · link", "-"),
                                       (STEEL, "upright · arms · chassis", "--"))):
    x = j * 34
    tb.plot([x, x + 6], [33, 33], lw=MED, c=col, ls=style)
    tb.text(x + 8, 33, txt, fontproperties=LABEL, fontsize=5.8, color=INK, va="center")

fig.savefig(os.path.join(os.path.dirname(__file__), "corner_layout_study.png"), facecolor=PAPER)
print(f"T {T:.0f}  kingpin x {KP_X:.0f}  scrub {SCRUB:.1f}  spin {SPIN:.2f}  battery {CLEAR:.1f}  "
      f"pivot gap {PIV_GAP:.1f}  back {BACK_CLEAR:.1f}  fore-aft {FA_GAP:.1f}  belly {BELLY:.0f}  "
      f"tower {SHOCK_TOP_Z:.0f}")
