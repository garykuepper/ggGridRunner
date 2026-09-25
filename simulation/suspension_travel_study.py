"""ggGridRunner suspension travel study (study 06): is anything touching at full bump?

Moves the front-left corner through its travel and steering range in 3D and measures the true
surface-to-surface gap between every pair of parts that could touch:

- moves with the suspension, does not steer: upright (two prongs + back), MG996R on it
- moves with the suspension and steers: knuckle, motor (cylinder), wheel (tire + hub pocket as
  a solid of revolution), steering link and horn
- rotates about the chassis pivots: both arms (two rails each), the shock's lower eye

Both arms have the same length and rise, so the upright translates without tilting. Draws the
corner at full bump in front elevation and plan, and tabulates the smallest gap per pair at
full bump and over the whole travel.

    python simulation/suspension_travel_study.py
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Polygon

from drafting import *  # noqa: E402,F401,F403  fonts, inks, sheet frame
from rover_geometry import *  # noqa: E402,F401,F403  shared dimensions

TIGHT = 2.0                     # gaps below this are flagged
N_STEER = 41                    # steering samples over +/-STEER
N_TRAVEL = 26                   # travel samples over -DROOP..+BUMP

LINK_LEN = np.hypot(ARM, RISE)
ALPHA0 = np.arctan2(-RISE, -ARM)   # outer hinge seen from the chassis pivot (x, z)
PIV_A = HINGE_A + ARM              # chassis pivots, along a


# ---------------------------- kinematics ---------------------------------------
def link_angle(dz):
    """Rotation of both arms about their chassis pivots that lifts the upright by dz."""
    return -np.pi - np.arcsin((dz - RISE) / LINK_LEN) - ALPHA0


def upright_offset(dz):
    """Upright (and everything on it) translation (x, z) at wheel travel dz."""
    return np.array([PIV_A + LINK_LEN * np.cos(ALPHA0 + link_angle(dz)) - HINGE_A, dz])


def rotate_xz(p, cx, cz, phi):
    q = p.copy()
    dx, dz = p[:, 0] - cx, p[:, 2] - cz
    q[:, 0] = cx + dx * np.cos(phi) - dz * np.sin(phi)
    q[:, 2] = cz + dx * np.sin(phi) + dz * np.cos(phi)
    return q


def steer_xy(p, th):
    t = np.radians(th)
    q = p.copy()
    q[:, 0] = p[:, 0] * np.cos(t) - p[:, 1] * np.sin(t)
    q[:, 1] = p[:, 0] * np.sin(t) + p[:, 1] * np.cos(t)
    return q


def on_upright(p, dz):
    ox, oz = upright_offset(dz)
    return p + np.array([ox, 0.0, oz])


# ---------------------------- part point sets ------------------------------------
def box_points(x, y, z, n=14):
    """Surface samples of an axis-aligned box."""
    g = [np.linspace(lo, hi, n) for lo, hi in (x, y, z)]
    pts = []
    for i in range(3):
        j, k = [m for m in range(3) if m != i]
        A, B = np.meshgrid(g[j], g[k])
        for v in (g[i][0], g[i][-1]):
            p = np.empty((A.size, 3))
            p[:, i], p[:, j], p[:, k] = v, A.ravel(), B.ravel()
            pts.append(p)
    return np.vstack(pts)


PRONG_UP_BOX = ((0.0, UPR_BACK[0]), (-PRONG_W / 2, PRONG_W / 2), PRONG_UP_Z)
PRONG_LO_BOX = ((0.0, UPR_BACK[0]), (-PRONG_W / 2, PRONG_W / 2), PRONG_LO_Z)
BACK_BOX = (UPR_BACK, (-UPR_BACK_W, UPR_BACK_W), UPR_Z)
UPRIGHT_PARTS = {"upper prong": PRONG_UP_BOX, "lower prong": PRONG_LO_BOX, "upright back": BACK_BOX,
                 "servo": SERVO_BOX}


def upright_part(name, dz):
    return on_upright(box_points(*UPRIGHT_PARTS[name]), dz)


def arm_rails(z_hinge, dz, n=120):
    t = np.linspace(0, 1, n)
    rails = []
    for d in (-1, 1):
        p = np.column_stack([HINGE_A + ARM * t, d * (ARM_W_OUT + (ARM_W_IN - ARM_W_OUT) * t),
                             z_hinge + RISE * t])
        rails.append(rotate_xz(p, PIV_A, z_hinge + RISE, link_angle(dz)))
    return np.vstack(rails)


def link_points(dz, th, n=120):
    """Steering link (knuckle ball -> horn ball) and the horn, on the upright."""
    v = steer_xy(np.array([[0.0, STEER_ARM, 0.0]]), th)[0]
    k = np.array([0.0, 0.0, HORN_Z]) + v
    h = np.array([SERVO_A, 0.0, HORN_Z]) + v
    s = np.array([SERVO_A, 0.0, HORN_Z])
    t = np.linspace(0, 1, n)[:, None]
    return on_upright(np.vstack([k + t * (h - k), s + t * (h - s)]), dz)


def shock_points(dz, n=120):
    lo = np.array([[HINGE_A + SHOCK_ON_ARM, 0.0, LO_H_Z + RISE * SHOCK_ON_ARM / ARM]])
    lo = rotate_xz(lo, PIV_A, LO_H_Z + RISE, link_angle(dz))[0]
    top = np.array([KP_X - TOWER_X, 0.0, SHOCK_TOP_Z])
    return lo + np.linspace(0, 1, n)[:, None] * (top - lo)


# ---------------------------- distance fields ------------------------------------
def to_wheel(p, dz, th):
    """Chassis-local points -> steered knuckle frame (x along the axle, inboard +)."""
    ox, oz = upright_offset(dz)
    q = p - np.array([ox, 0.0, oz])
    t = np.radians(th)
    x = q[:, 0] * np.cos(t) + q[:, 1] * np.sin(t)
    y = -q[:, 0] * np.sin(t) + q[:, 1] * np.cos(t)
    return x, y, q[:, 2]


def rect_dist(x, r, x0, x1, r0, r1):
    dx = np.maximum(np.maximum(x0 - x, x - x1), 0)
    dr = np.maximum(np.maximum(r0 - r, r - r1), 0)
    inside = (dx == 0) & (dr == 0)
    depth = np.minimum.reduce([x - x0, x1 - x, r - r0, r1 - r])
    return np.where(inside, -depth, np.hypot(dx, dr))


def dist_motor(p, dz, th):
    x, y, z = to_wheel(p, dz, th)
    return rect_dist(x, np.hypot(y, z - AXLE), 0.0, TAIL_X, 0.0, MOTOR_D / 2)


def dist_wheel(p, dz, th):
    """Tire annulus round the pocket, plus the web at the pocket bottom."""
    x, y, z = to_wheel(p, dz, th)
    r = np.hypot(y, z - AXLE)
    return np.minimum(rect_dist(x, r, TIRE_OUT, TIRE_IN, HUB_D / 2, TIRE_D / 2),
                      rect_dist(x, r, TIRE_OUT, POCKET_BOTTOM, 0.0, TIRE_D / 2))


def dist_box(p, box, dz):
    """Distance to an upright-mounted box at travel dz."""
    ox, oz = upright_offset(dz)
    q = p - np.array([ox, 0.0, oz])
    d = [np.maximum(np.maximum(lo - q[:, i], q[:, i] - hi), 0) for i, (lo, hi) in enumerate(box)]
    return np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)


def motor_surface(dz, th, n=40):
    xs = np.linspace(0, TAIL_X, n)
    ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    X, A = np.meshgrid(xs, ang)
    p = np.column_stack([X.ravel(), (MOTOR_D / 2 * np.cos(A)).ravel(),
                         (AXLE + MOTOR_D / 2 * np.sin(A)).ravel()])
    return on_upright(steer_xy(np.vstack([p, [[TAIL_X, 0, AXLE]]]), th), dz)


# ---------------------------- the check ------------------------------------------
def pair_gaps(dz, th):
    parts = {n: (upright_part(n, dz), 0.0) for n in UPRIGHT_PARTS}
    parts["link"] = (link_points(dz, th), LINK_HALF)
    parts["upper arm"] = (arm_rails(UP_H_Z, dz), ARM_HALF)
    parts["lower arm"] = (arm_rails(LO_H_Z, dz), ARM_HALF)
    parts["shock"] = (shock_points(dz), SHOCK_R)
    out = {}
    for name, (pts, rad) in parts.items():
        d = dist_motor(pts, dz, th) - rad
        out[("motor", name)] = (d.min(), pts[d.argmin()])
        d = dist_wheel(pts, dz, th) - rad
        out[("wheel", name)] = (d.min(), pts[d.argmin()])
    for a, b in (("link", "upper prong"), ("link", "upright back"), ("shock", "upper arm"),
                 ("shock", "servo"), ("upper arm", "servo")):
        pa, ra = parts[a]
        if b in UPRIGHT_PARTS:
            d = dist_box(pa, UPRIGHT_PARTS[b], dz) - ra
            out[(a, b)] = (d.min(), pa[d.argmin()])
        else:
            pb, rb = parts[b]
            dd = np.linalg.norm(pa[::2, None, :] - pb[None, ::2, :], axis=2) - ra - rb
            i = np.unravel_index(dd.argmin(), dd.shape)
            out[(a, b)] = (dd.min(), pa[::2][i[0]])
    return out


def sweep(dzs, ths):
    worst = {}
    for dz in dzs:
        for th in ths:
            for k, (g, p) in pair_gaps(dz, th).items():
                if k not in worst or g < worst[k][0]:
                    worst[k] = (g, p, dz, th)
    return worst


STEERS = np.linspace(-STEER, STEER, N_STEER)
AT_BUMP = sweep([BUMP], STEERS)
OVER_TRAVEL = sweep(np.linspace(-DROOP, BUMP, N_TRAVEL), STEERS)
sh = lambda dz: np.linalg.norm(shock_points(dz)[-1] - shock_points(dz)[0])
SH_BUMP, SH_DROOP = sh(BUMP), sh(-DROOP)
UO_BUMP = upright_offset(BUMP)

PAIRS = [("motor", "upper prong"), ("motor", "lower prong"), ("motor", "upright back"),
         ("motor", "link"), ("motor", "lower arm"), ("wheel", "upper prong"),
         ("wheel", "lower prong"), ("wheel", "link"), ("wheel", "servo"), ("wheel", "upper arm"),
         ("wheel", "lower arm"), ("wheel", "shock"), ("link", "upper prong"),
         ("link", "upright back"), ("shock", "upper arm"), ("upper arm", "servo")]

# ============================ SHEET ==========================================
fig = plt.figure(figsize=(17, 11), dpi=200, facecolor=PAPER)
sheet_frame(fig)

# ---------------------------- elevation at full bump -------------------------
ex = fig.add_axes([0.04, 0.09, 0.56, 0.82], facecolor="none")
ex.set_aspect("equal"); ex.axis("off")
ex.set_xlim(-14, 204); ex.set_ylim(-34, 218)

E = lambda p: np.column_stack([KP_X - p[:, 0], p[:, 2]])   # local -> elevation (x outboard)
ox, oz = UO_BUMP
KX = KP_X - ox

ex.plot([-12, 202], [0, 0], lw=MED, c=INK)
for x in np.arange(-8, 203, 5):
    ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
ex.plot([0, 0], [-10, 196], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
ex.text(1.5, 192, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.6, color=GREY)

# ride-height ghost
ex.add_patch(FancyBboxPatch((KP_X - TIRE_IN, 0), TIRE_W, TIRE_D,
                            boxstyle="round,pad=0,rounding_size=8", fc="none", ec=GREY, lw=HAIR,
                            ls=(0, (2, 2))))
for z in (LO_H_Z, UP_H_Z):
    ex.plot([KP_X - HINGE_A, PIVOT], [z, z + RISE], lw=HAIR, c=GREY, ls=(0, (2, 2)))
ex.add_patch(Rectangle((KP_X - UPR_BACK[1], UPR_Z[0]), UPR_BACK[1] - UPR_BACK[0],
                       UPR_Z[1] - UPR_Z[0], fc="none", ec=GREY, lw=HAIR, ls=(0, (2, 2))))

# chassis (fixed)
ex.add_patch(Rectangle((0, BELLY), PIVOT + 12, UP_H_Z - LO_H_Z + 16, fc=STEEL, alpha=0.05,
                       ec=STEEL, lw=THIN, ls=(0, (4, 2))))
ex.add_patch(Rectangle((0, UP_H_Z + RISE + 8), TOWER_X + 4, SHOCK_TOP_Z - UP_H_Z - RISE,
                       fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN, ls=(0, (4, 2))))

# wheel at full bump, pocket cut open
T_IN, T_OUT = KX - TIRE_IN, KX - TIRE_OUT
ex.add_patch(FancyBboxPatch((T_IN, oz), TIRE_W, TIRE_D, boxstyle="round,pad=0,rounding_size=8",
                            fc=INK, ec=INK, lw=THIN))
for z in np.arange(8, TIRE_D - 6, 6):
    ex.plot([T_IN, T_OUT], [z + oz, z + oz], lw=HAIR, c=PAPER, alpha=0.25)
ex.add_patch(Rectangle((T_IN, AXLE + oz - HUB_D / 2), HEX_IN, HUB_D, fc=PAPER, ec=INK, lw=THIN,
                       ls=(0, (2, 1.5))))
ex.add_patch(Rectangle((KX, AXLE + oz - 6), GBX_FACE, 12, fc=PAPER, ec=INK, lw=THIN))

# motor: straight ahead (solid) and at full lock (projected, dashed)
ex.add_patch(Rectangle((KX - TAIL_X, AXLE + oz - MOTOR_D / 2), TAIL_X, MOTOR_D, fc=PAPER, ec=INK,
                       lw=MED, zorder=3))
ex.add_patch(Rectangle((KX - TAIL_X, AXLE + oz - MOTOR_D / 2), 10, MOTOR_D, fc=INK, alpha=0.18,
                       ec="none", zorder=3))
reach = TAIL_X * np.cos(np.radians(STEER)) + MOTOR_D / 2 * np.sin(np.radians(STEER))
ex.add_patch(Rectangle((KX - reach, AXLE + oz - MOTOR_D / 2), reach, MOTOR_D, fc="none", ec=VERM,
                       lw=THIN, ls=(0, (3, 2)), zorder=3))
ex.add_patch(Polygon([[KX, KP_LO_Z + oz + 3], [KX + 5, KP_LO_Z + oz + 3],
                      [KX + 5, KP_UP_Z + oz - 3], [KX, KP_UP_Z + oz - 3]], closed=True, fc=VERM,
                     alpha=0.35, ec=VERM, lw=THIN, zorder=4))

# upright + servo + link at full bump
for z0, z1 in (PRONG_LO_Z, PRONG_UP_Z):
    ex.add_patch(Rectangle((KX - UPR_BACK[0], z0 + oz), UPR_BACK[0] + 4, z1 - z0, fc=STEEL,
                           alpha=0.3, ec=STEEL, lw=THIN, zorder=5))
ex.add_patch(Rectangle((KX - UPR_BACK[1], UPR_Z[0] + oz), UPR_BACK[1] - UPR_BACK[0],
                       UPR_Z[1] - UPR_Z[0], fc=STEEL, alpha=0.45, ec=STEEL, lw=THIN, zorder=5))
(sa0, sa1), _, (sz0, sz1) = SERVO_BOX
ex.add_patch(Rectangle((KX - sa1, sz0 + oz), sa1 - sa0, sz1 - sz0, fc=PAPER, ec=INK, lw=THIN,
                       zorder=5))
ex.plot([KX - SERVO_A, KX], [HORN_Z + oz] * 2, lw=THIN * 1.4, c=VERM, zorder=6)
ex.plot([KX - SERVO_A] * 2, [HORN_Z + oz, sz0 + oz], lw=MED, c=INK, zorder=6)
for z in (KP_LO_Z, KP_UP_Z):
    ex.add_patch(Circle((KX, z + oz), 2.6, fc=PAPER, ec=INK, lw=THIN, zorder=7))

# arms + shock at full bump
for zh in (LO_H_Z, UP_H_Z):
    q = E(arm_rails(zh, BUMP)[:120])
    ex.plot(q[:, 0], q[:, 1], lw=MED * 1.8, c=STEEL, solid_capstyle="round", zorder=5)
    for p in (q[0], q[-1]):
        ex.add_patch(Circle(p, 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
s = E(shock_points(BUMP))
u = (s[-1] - s[0]) / np.linalg.norm(s[-1] - s[0]); nrm = np.array([-u[1], u[0]])
b0, b1 = s[0] + u * 12, s[0] + u * 0.62 * SHOCK_LEN
ex.plot(s[:, 0], s[:, 1], lw=THIN, c=INK)
ex.add_patch(Polygon([b0 + nrm * SHOCK_R, b1 + nrm * SHOCK_R, b1 - nrm * SHOCK_R,
                      b0 - nrm * SHOCK_R], fc=PAPER, ec=INK, lw=THIN, zorder=6))
for k in np.linspace(0.12, 0.9, 11):
    p = b0 + (b1 - b0) * k
    ex.plot(*zip(p + nrm * SHOCK_R, p - nrm * SHOCK_R), lw=HAIR, c=INK, zorder=7)
for p in (s[0], s[-1]):
    ex.add_patch(Circle(p, 2.4, fc=PAPER, ec=INK, lw=THIN, zorder=8))

# closest approaches at full bump, projected into the elevation
MARK = {("wheel", "upper prong"): (10, 64), ("wheel", "lower prong"): (30, -52),
        ("motor", "upright back"): (-54, -40), ("motor", "link"): (-40, 44),
        ("wheel", "servo"): (34, 42), ("shock", "upper arm"): (-26, 48)}
for k, (dx, dy) in MARK.items():
    g, p, dz, th = AT_BUMP[k]
    xy = E(p[None, :])[0]
    col = VERM if g < TIGHT else INK
    ex.add_patch(Circle(xy, 1.6, fc=col, ec=PAPER, lw=0.4, zorder=10))
    ex.annotate(f"{k[0]} / {k[1]}  {g:.1f} mm  ({th:+.0f}°)", xy=xy, xytext=(xy[0] + dx, xy[1] + dy),
                fontproperties=MONO, fontsize=6, color=col, ha="center", va="center",
                arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=1.5),
                bbox=dict(fc=PAPER, ec="none", pad=0.6), zorder=11)

tx = T_OUT + 8
ex.plot([T_OUT + 2, tx + 4], [TIRE_D, TIRE_D], lw=HAIR, c=GREY)
ex.plot([T_OUT + 2, tx + 4], [TIRE_D + oz, TIRE_D + oz], lw=HAIR, c=GREY)
ex.annotate("", xy=(tx, TIRE_D), xytext=(tx, TIRE_D + oz),
            arrowprops=dict(arrowstyle="<|-|>", lw=HAIR, color=INK, mutation_scale=6))
ex.text(tx + 3, TIRE_D + oz / 2, f"+{BUMP:.0f}\nBUMP", fontproperties=MONO, fontsize=6,
        color=INK, va="center", linespacing=1.3)


def elab(x, y, s, tx, ty, col=INK, ha="center"):
    ex.annotate(s, xy=(x, y), xytext=(tx, ty), fontproperties=LABEL_M, fontsize=6, color=col,
                ha=ha, va="center",
                arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=0),
                bbox=dict(fc=PAPER, ec="none", pad=0.6))


elab(KX - UPR_BACK[1] + 3, UPR_Z[0] + oz + 6, "UPRIGHT  (rides with the wheel)", 70, 14,
     col=STEEL)
elab(KX - sa1 + 6, sz1 + oz - 6, "MG996R  (on upright)", 96, 176, col=INK)
elab(s[0, 0] - 14, s[0, 1] + 30, f"SHOCK  {SH_BUMP:.0f}  (ride {SHOCK_LEN:.0f})", 40, 186)
elab(T_OUT - 4, 112 + oz, "WHEEL AT BUMP", 184, 190)
elab(KX - reach + 3, AXLE + oz - MOTOR_D / 2, f"MOTOR AT ±{STEER:.0f}°  (projected)", 126, -24,
     col=VERM)

ex.text(-12, 216, "FULL BUMP", fontproperties=SERIF, fontsize=17, color=INK, va="top")
ex.text(-12, 207, f"front-left corner, looking rearward  ·  +{BUMP:.0f} mm wheel travel  ·  "
        "ride height dashed  ·  hub pocket cut open  ·  mm", fontproperties=LABEL, fontsize=6.5,
        color=GREY, va="top")

# ---------------------------- plan of the corner at full bump ------------------
pl = fig.add_axes([0.62, 0.55, 0.35, 0.36], facecolor="none")
pl.set_aspect("equal"); pl.axis("off")
pl.set_xlim(-8, 204); pl.set_ylim(-62, 92)

P = lambda p: np.column_stack([KP_X - p[:, 0], p[:, 1]])
tire2d = rect(TIRE_OUT, TIRE_IN, -TIRE_D / 2, TIRE_D / 2)
mot2d = rect(0, TAIL_X, -MOTOR_D / 2, MOTOR_D / 2)


def steer2d(poly, th):
    t = np.radians(th)
    r = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    q = poly @ r.T
    return np.column_stack([KX - q[:, 0], q[:, 1]])


for th in np.arange(-STEER, STEER + 0.01, 5):
    for poly in (tire2d, mot2d):
        pl.add_patch(Polygon(steer2d(poly, th), closed=True, fill=False, ec=VERM, lw=0.25,
                             alpha=0.6))
pl.add_patch(Polygon(steer2d(tire2d, 0), fc=INK, ec=INK, lw=THIN))
pl.add_patch(Polygon(steer2d(mot2d, 0), fc=PAPER, ec=INK, lw=MED))
for box, a in ((PRONG_UP_BOX, 0.35), (BACK_BOX, 0.55)):
    (x0, x1), (y0, y1), _ = box
    pl.add_patch(Rectangle((KX - x1, y0), x1 - x0, y1 - y0, fc=STEEL, alpha=a, ec=STEEL, lw=THIN,
                           zorder=4))
(x0, x1), (y0, y1), _ = SERVO_BOX
pl.add_patch(Rectangle((KX - x1, y0), x1 - x0, y1 - y0, fc="none", ec=STEEL, lw=THIN,
                       ls=(0, (3, 1.5)), zorder=5))
q = P(arm_rails(LO_H_Z, BUMP))
for half in (q[:120], q[120:]):
    pl.plot(half[:, 0], half[:, 1], lw=MED, c=STEEL)
    pl.add_patch(Circle(half[-1], 2.2, fc=PAPER, ec=STEEL, lw=THIN, zorder=5))
for th, lw, a in ((-STEER, HAIR, 0.7), (STEER, HAIR, 0.7), (0.0, THIN * 1.4, 1.0)):
    q = P(link_points(BUMP, th))
    pl.plot(q[:120, 0], q[:120, 1], lw=lw, c=VERM, alpha=a, zorder=6)
    pl.plot(q[120:, 0], q[120:, 1], lw=lw, c=VERM, alpha=a, zorder=6)
q = P(shock_points(BUMP))
pl.plot(q[:, 0], q[:, 1], lw=MED * 2.2, c=INK, alpha=0.3, solid_capstyle="butt")
pl.add_patch(Circle((KX, 0), 3, fc=PAPER, ec=INK, lw=THIN, zorder=8))
pl.plot([0, 0], [-56, 86], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
pl.text(-6, 90, "PLAN AT FULL BUMP", fontproperties=SERIF, fontsize=13, color=INK, va="top")
pl.text(-6, 80, f"link at 0° and ±{STEER:.0f}°  ·  lower arm shown  ·  outward is up  ·  mm",
        fontproperties=LABEL, fontsize=6, color=GREY, va="top")

# ---------------------------- clearance table ----------------------------------
tb = fig.add_axes([0.62, 0.085, 0.345, 0.45], facecolor="none")
tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")
tb.text(0, 99, "CLEARANCE  ·  surface to surface, 3D", fontproperties=LABEL_M, fontsize=7,
        color=INK, va="top")
tb.text(52, 94, f"AT +{BUMP:.0f}", fontproperties=LABEL_M, fontsize=5.8, color=GREY, ha="right")
tb.text(100, 94, f"WORST OVER −{DROOP:.0f}…+{BUMP:.0f}  (where)", fontproperties=LABEL_M,
        fontsize=5.8, color=GREY, ha="right")
tb.plot([0, 100], [91.5, 91.5], lw=THIN, c=INK)
for i, k in enumerate(PAIRS):
    y = 88.5 - i * 3.65
    gb = AT_BUMP[k][0]
    gw, _, dzw, thw = OVER_TRAVEL[k]
    tb.text(0, y, f"{k[0]}  /  {k[1]}", fontproperties=LABEL, fontsize=6.4, color=INK,
            va="center")
    tb.text(52, y, f"{gb:5.1f}", fontproperties=MONO, fontsize=6.4,
            color=VERM if gb < TIGHT else INK, ha="right", va="center")
    tb.text(100, y, f"{gw:5.1f}   ({dzw:+.0f} mm, {thw:+.0f}°)", fontproperties=MONO,
            fontsize=6.4, color=VERM if gw < TIGHT else INK, ha="right", va="center")
    tb.plot([0, 100], [y - 1.82, y - 1.82], lw=HAIR, c=GREY, alpha=0.4)
tb.text(0, 28.5, f"shock {SH_DROOP:.1f} at droop · {SHOCK_LEN:.0f} ride · {SH_BUMP:.1f} at bump"
        f"  ·  upright moves {-UO_BUMP[0]:.1f} mm outboard at bump  ·  red = under {TIGHT:.0f} mm",
        fontproperties=LABEL, fontsize=5.6, color=GREY, va="bottom")
title_block(tb, "suspension travel check  —  study 06", "2026-09-25")

fig.savefig(os.path.join(os.path.dirname(__file__), "suspension_travel_study.png"),
            facecolor=PAPER)
for k in PAIRS:
    g, _, dz, th = OVER_TRAVEL[k]
    print(f"{k[0]:>9} - {k[1]:<13} bump {AT_BUMP[k][0]:6.1f}   worst {g:6.1f} at dz {dz:+.1f} "
          f"th {th:+.1f}")
print(f"shock {SH_DROOP:.1f} / {SHOCK_LEN:.0f} / {SH_BUMP:.1f}  upright x at bump {UO_BUMP[0]:.2f}")
