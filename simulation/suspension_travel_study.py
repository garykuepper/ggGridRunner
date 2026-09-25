"""ggGridRunner suspension travel study (study 08): is anything touching at full bump?

Moves the front-left corner (study 07 geometry, see rover_geometry.py) through its travel and
steering range in 3D and measures the true surface-to-surface gap between every pair of parts
that could touch:

- steers and moves with the wheel: motor (cylinder), wheel (tire + hub pocket as a solid of
  revolution), knuckle
- moves with the wheel, does not steer: the two kingpin blocks
- rotates about the chassis pivots: both boomerang arms (single stem, then two legs)
- tie rod: from the knuckle steering arm to the servo horn on the spine
- fixed to the chassis: MG996R body, shock tower top (the shock runs from the upper arm)

Both arms and the tie rod have the same length and rise, so the knuckle translates without
tilting. Draws the corner at full bump in front elevation and plan, and tabulates the smallest
gap per pair at full bump and over the whole travel.

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
ALPHA0 = np.arctan2(-RISE, -ARM)   # knuckle hinge seen from the chassis pivot (x, z)


# ---------------------------- kinematics ---------------------------------------
def link_angle(dz):
    """Rotation of every link about its chassis pivot that lifts the knuckle by dz."""
    return -np.pi - np.arcsin((dz - RISE) / LINK_LEN) - ALPHA0


def knuckle_offset(dz):
    """Knuckle (and wheel, motor, blocks) translation (x, z) at wheel travel dz."""
    return np.array([ARM + LINK_LEN * np.cos(ALPHA0 + link_angle(dz)), dz])


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


def on_knuckle(p, dz):
    ox, oz = knuckle_offset(dz)
    return p + np.array([ox, 0.0, oz])


# ---------------------------- part point sets ------------------------------------
def box_points(x, y, z, n=10):
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


def block(z):
    h = BLOCK / 2
    return ((-h, h), (-h, h), (z - h, z + h))


def arm_legs(z0, name, spread, dz, n=160):
    """Both legs of a Y-stem boomerang arm at travel dz (the stem is shared)."""
    a = np.linspace(0, ARM, n)
    legs = []
    for d in (-1, 1):
        y = d * spread * np.clip((a - STEM) / (ARM - STEM), 0, 1)
        p = np.column_stack([a, y, z0 + shape(a, name)])
        legs.append(rotate_xz(p, ARM, z0 + RISE, link_angle(dz)))
    return legs


def tie_rod(dz, th, n=160):
    v = steer_xy(np.array([[0.0, STEER_ARM, 0.0]]), th)[0]
    k = on_knuckle((np.array([0.0, 0.0, TR_Z]) + v)[None, :], dz)[0]
    h = np.array([ARM, 0.0, HORN_Z]) + v
    return k + np.linspace(0, 1, n)[:, None] * (h - k)


def shock(dz, n=120):
    lo = np.array([[SHOCK_AT, 0.0, KP_UP_Z + shape(SHOCK_AT, "upper") + SHOCK_EYE_UP]])
    lo = rotate_xz(lo, ARM, KP_UP_Z + RISE, link_angle(dz))[0]
    top = np.array([KP_X - TOWER_X, 0.0, SHOCK_TOP_Z])
    return lo + np.linspace(0, 1, n)[:, None] * (top - lo)


# ---------------------------- distance fields ------------------------------------
def to_wheel(p, dz, th):
    ox, oz = knuckle_offset(dz)
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
    x, y, z = to_wheel(p, dz, th)
    r = np.hypot(y, z - AXLE)
    return np.minimum(rect_dist(x, r, TIRE_OUT, TIRE_IN, HUB_D / 2, TIRE_D / 2),
                      rect_dist(x, r, TIRE_OUT, POCKET_BOTTOM, 0.0, TIRE_D / 2))


def dist_box(p, box):
    d = [np.maximum(np.maximum(lo - p[:, i], p[:, i] - hi), 0) for i, (lo, hi) in enumerate(box)]
    return np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)


def min_pair(pa, ra, pb, rb):
    dd = np.linalg.norm(pa[::2, None, :] - pb[None, ::2, :], axis=2) - ra - rb
    i = np.unravel_index(dd.argmin(), dd.shape)
    return dd.min(), pa[::2][i[0]]


# ---------------------------- the check ------------------------------------------
def pair_gaps(dz, th):
    up = np.vstack(arm_legs(KP_UP_Z, "upper", UP_SPREAD, dz))
    lo = np.vstack(arm_legs(KP_LO_Z, "lower", LO_SPREAD, dz))
    tr = tie_rod(dz, th)
    sh = shock(dz)
    parts = {"upper arm": (up, ARM_HALF), "lower arm": (lo, ARM_HALF), "tie rod": (tr, TR_HALF),
             "shock": (sh, SHOCK_R),
             "upper block": (on_knuckle(box_points(*block(KP_UP_Z)), dz), 0.0),
             "lower block": (on_knuckle(box_points(*block(KP_LO_Z)), dz), 0.0)}
    out = {}
    for name, (pts, rad) in parts.items():
        d = dist_motor(pts, dz, th) - rad
        out[("motor", name)] = (d.min(), pts[d.argmin()])
        if "block" not in name:        # the blocks sit on the kingpin inside the pocket
            d = dist_wheel(pts, dz, th) - rad
            out[("wheel", name)] = (d.min(), pts[d.argmin()])
    # The tie rod's inner end sits on the horn above the servo, so skip its last 10 %.
    for a, (pa, ra) in (("upper arm", parts["upper arm"]), ("lower arm", parts["lower arm"]),
                        ("tie rod", (tr[: int(len(tr) * 0.9)], TR_HALF)), ("shock", parts["shock"])):
        d = dist_box(pa, SERVO_BOX) - ra
        out[("servo", a)] = (d.min(), pa[d.argmin()])
    out[("tie rod", "upper arm")] = min_pair(tr, TR_HALF, up, ARM_HALF)
    out[("tie rod", "lower arm")] = min_pair(tr, TR_HALF, lo, ARM_HALF)
    out[("shock", "tie rod")] = min_pair(sh, SHOCK_R, tr, TR_HALF)
    # The shock's lower eye is on the upper arm: only check the body past the eye.
    out[("shock", "upper arm")] = min_pair(sh[int(len(sh) * 0.15):], SHOCK_R, up, ARM_HALF)
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
sh_len = lambda dz: np.linalg.norm(shock(dz)[-1] - shock(dz)[0])
SH_BUMP, SH_DROOP = sh_len(BUMP), sh_len(-DROOP)
KO_BUMP = knuckle_offset(BUMP)

PAIRS = [("motor", "tie rod"), ("motor", "upper arm"), ("motor", "lower arm"),
         ("motor", "upper block"), ("motor", "lower block"), ("wheel", "upper arm"),
         ("wheel", "lower arm"), ("wheel", "tie rod"), ("wheel", "shock"),
         ("tie rod", "upper arm"), ("tie rod", "lower arm"), ("servo", "upper arm"),
         ("servo", "lower arm"), ("servo", "tie rod"), ("servo", "shock"), ("shock", "upper arm"),
         ("shock", "tie rod")]

if __name__ == "__main__":
    # ============================ SHEET ==========================================
    fig = plt.figure(figsize=(17, 11), dpi=200, facecolor=PAPER)
    sheet_frame(fig)

    ex = fig.add_axes([0.04, 0.09, 0.56, 0.82], facecolor="none")
    ex.set_aspect("equal"); ex.axis("off")
    ex.set_xlim(-14, 204); ex.set_ylim(-36, 218)

    E = lambda p: np.column_stack([KP_X - p[:, 0], p[:, 2]])   # local -> elevation
    ox, oz = KO_BUMP
    KX = KP_X - ox

    ex.plot([-12, 202], [0, 0], lw=MED, c=INK)
    for x in np.arange(-8, 203, 5):
        ex.plot([x, x - 4], [0, -4], lw=HAIR, c=INK)
    ex.plot([0, 0], [-10, 196], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
    ex.text(1.5, 192, "CL  CHASSIS", fontproperties=LABEL, fontsize=5.6, color=GREY)

    # ride-height ghost
    ex.add_patch(FancyBboxPatch((KP_X - TIRE_IN, 0), TIRE_W, TIRE_D,
                                boxstyle="round,pad=0,rounding_size=8", fc="none", ec=GREY,
                                lw=HAIR, ls=(0, (2, 2))))
    for z0, name, spread in ((KP_UP_Z, "upper", UP_SPREAD), (KP_LO_Z, "lower", LO_SPREAD)):
        q = E(arm_legs(z0, name, spread, 0.0)[0])
        ex.plot(q[:, 0], q[:, 1], lw=HAIR, c=GREY, ls=(0, (2, 2)))
    ex.add_patch(Rectangle((KP_X - TAIL_X, AXLE - MOTOR_D / 2), TAIL_X, MOTOR_D, fc="none",
                           ec=GREY, lw=HAIR, ls=(0, (2, 2))))

    # chassis: spine, servo, tower
    ex.add_patch(Rectangle((0, BELLY), PIVOT + 14, KP_UP_Z - KP_LO_Z + 16, fc=STEEL, alpha=0.05,
                           ec=STEEL, lw=THIN, ls=(0, (4, 2))))
    ex.add_patch(Rectangle((0, KP_UP_Z + RISE + 8), TOWER_X + 4,
                           SHOCK_TOP_Z - KP_UP_Z - RISE, fc=STEEL, alpha=0.05, ec=STEEL, lw=THIN,
                           ls=(0, (4, 2))))
    (sx0, sx1), _, (sz0, sz1) = SERVO_BOX
    ex.add_patch(Rectangle((KP_X - sx1, sz0), sx1 - sx0, sz1 - sz0, fc=STEEL, alpha=0.14,
                           ec=STEEL, lw=THIN))

    # wheel at full bump
    T_IN, T_OUT = KX - TIRE_IN, KX - TIRE_OUT
    ex.add_patch(FancyBboxPatch((T_IN, oz), TIRE_W, TIRE_D,
                                boxstyle="round,pad=0,rounding_size=8", fc=INK, ec=INK, lw=THIN))
    for z in np.arange(8, TIRE_D - 6, 6):
        ex.plot([T_IN, T_OUT], [z + oz, z + oz], lw=HAIR, c=PAPER, alpha=0.25)
    ex.add_patch(Rectangle((T_IN, AXLE + oz - HUB_D / 2), HEX_IN, HUB_D, fc=PAPER, ec=INK,
                           lw=THIN, ls=(0, (2, 1.5))))
    ex.add_patch(Rectangle((KX, AXLE + oz - 6), GBX_FACE, 12, fc=PAPER, ec=INK, lw=THIN))
    ex.add_patch(Rectangle((KX - TAIL_X, AXLE + oz - MOTOR_D / 2), TAIL_X, MOTOR_D, fc=PAPER,
                           ec=INK, lw=MED, zorder=3))
    ex.add_patch(Rectangle((KX - TAIL_X, AXLE + oz - MOTOR_D / 2), 10, MOTOR_D, fc=INK,
                           alpha=0.18, ec="none", zorder=3))
    reach = TAIL_X * np.cos(np.radians(STEER)) + MOTOR_D / 2 * np.sin(np.radians(STEER))
    ex.add_patch(Rectangle((KX - reach, AXLE + oz - MOTOR_D / 2), reach, MOTOR_D, fc="none",
                           ec=VERM, lw=THIN, ls=(0, (3, 2)), zorder=3))
    ex.add_patch(Polygon([[KX, KP_LO_Z + oz + 5], [KX + 5, KP_LO_Z + oz + 5],
                          [KX + 5, KP_UP_Z + oz - 5], [KX, KP_UP_Z + oz - 5]], closed=True,
                         fc=VERM, alpha=0.35, ec=VERM, lw=THIN, zorder=4))
    for z in (KP_LO_Z, KP_UP_Z):
        ex.add_patch(Rectangle((KX - BLOCK / 2, z + oz - BLOCK / 2), BLOCK, BLOCK, fc=STEEL,
                               alpha=0.5, ec=STEEL, lw=THIN, zorder=6))
        ex.add_patch(Circle((KX, z + oz), 1.8, fc=PAPER, ec=INK, lw=THIN, zorder=7))

    # links at full bump
    for z0, name, spread in ((KP_UP_Z, "upper", UP_SPREAD), (KP_LO_Z, "lower", LO_SPREAD)):
        q = E(arm_legs(z0, name, spread, BUMP)[0])
        ex.plot(q[:, 0], q[:, 1], lw=MED * 1.8, c=STEEL, solid_capstyle="round",
                solid_joinstyle="round", zorder=5)
        ex.add_patch(Circle(q[-1], 2.4, fc=PAPER, ec=STEEL, lw=THIN, zorder=7))
    q = E(tie_rod(BUMP, 0.0))
    ex.plot(q[:, 0], q[:, 1], lw=THIN * 1.5, c=VERM, zorder=6)
    for p in (q[0], q[-1]):
        ex.add_patch(Circle(p, 1.8, fc=PAPER, ec=VERM, lw=THIN, zorder=7))
    s = E(shock(BUMP))
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

    # closest approaches at full bump: numbered markers, keyed below the view
    worst6 = sorted(PAIRS, key=lambda k: AT_BUMP[k][0])[:6]
    for i, k in enumerate(worst6, 1):
        g, p, dz, th = AT_BUMP[k]
        xy = E(p[None, :])[0]
        col = VERM if g < TIGHT else INK
        ex.add_patch(Circle(xy, 3.4, fc=PAPER, ec=col, lw=THIN, zorder=10))
        ex.text(*xy, str(i), fontproperties=MONO, fontsize=5.6, color=col, ha="center",
                va="center", zorder=11)
        ex.text(-10, -14 - (i - 1) * 3.4, f"{i}  {k[0]} / {k[1]}  {g:.1f} mm at {th:+.0f}°",
                fontproperties=MONO, fontsize=6, color=col, va="center")
    ex.text(-10, -10, f"TIGHTEST AT +{BUMP:.0f} MM  (markers)", fontproperties=LABEL_M,
            fontsize=5.8, color=GREY, va="center")

    tx = T_OUT + 8
    ex.plot([T_OUT + 2, tx + 4], [TIRE_D, TIRE_D], lw=HAIR, c=GREY)
    ex.plot([T_OUT + 2, tx + 4], [TIRE_D + oz, TIRE_D + oz], lw=HAIR, c=GREY)
    ex.annotate("", xy=(tx, TIRE_D), xytext=(tx, TIRE_D + oz),
                arrowprops=dict(arrowstyle="<|-|>", lw=HAIR, color=INK, mutation_scale=6))
    ex.text(tx + 3, TIRE_D + oz / 2, f"+{BUMP:.0f}\nBUMP", fontproperties=MONO, fontsize=6,
            color=INK, va="center", linespacing=1.3)

    def elab(x, y, s_, tx_, ty_, col=INK):
        ex.annotate(s_, xy=(x, y), xytext=(tx_, ty_), fontproperties=LABEL_M, fontsize=6,
                    color=col, ha="center", va="center",
                    arrowprops=dict(arrowstyle="-", lw=HAIR, color=col, shrinkA=1, shrinkB=0),
                    bbox=dict(fc=PAPER, ec="none", pad=0.6))

    elab(KP_X - sx1 + 4, sz0 + 6, "MG996R  (spine)", 18, 24, col=STEEL)
    elab(KX - BLOCK / 2, KP_UP_Z + oz + 2, "KINGPIN BLOCK", 150, 196, col=STEEL)
    elab(T_OUT - 4, 112 + oz, "WHEEL AT BUMP", 188, 176)
    elab(KX - reach + 3, AXLE + oz - MOTOR_D / 2, f"MOTOR AT ±{STEER:.0f}°  (projected)", 126,
         -24, col=VERM)
    elab(s[0, 0] - 30, s[0, 1] + 26, f"SHOCK  {SH_BUMP:.0f}  (ride {SHOCK_LEN:.0f})", 40, 204)

    ex.text(-12, 216, "FULL BUMP", fontproperties=SERIF, fontsize=17, color=INK, va="top")
    ex.text(-12, 207, f"front-left corner, looking rearward  ·  +{BUMP:.0f} mm wheel travel  ·  "
            "ride height dashed  ·  hub pocket cut open  ·  mm", fontproperties=LABEL,
            fontsize=6.5, color=GREY, va="top")

    # ---------------------------- plan at full bump ------------------------------
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
    (x0, x1), (y0, y1), _ = SERVO_BOX
    pl.add_patch(Rectangle((KP_X - x1, y0), x1 - x0, y1 - y0, fc=STEEL, alpha=0.14, ec=STEEL,
                           lw=THIN))
    for z0, name, spread, lw in ((KP_LO_Z, "lower", LO_SPREAD, MED),
                                 (KP_UP_Z, "upper", UP_SPREAD, THIN)):
        for leg in arm_legs(z0, name, spread, BUMP):
            q = P(leg)
            pl.plot(q[:, 0], q[:, 1], lw=lw, c=STEEL)
            pl.add_patch(Circle(q[-1], 2.2, fc=PAPER, ec=STEEL, lw=THIN, zorder=5))
    for th, lw, a in ((-STEER, HAIR, 0.7), (STEER, HAIR, 0.7), (0.0, THIN * 1.4, 1.0)):
        q = P(tie_rod(BUMP, th))
        pl.plot(q[:, 0], q[:, 1], lw=lw, c=VERM, alpha=a, zorder=6)
    q = P(shock(BUMP))
    pl.plot(q[:, 0], q[:, 1], lw=MED * 2.2, c=INK, alpha=0.3, solid_capstyle="butt")
    pl.add_patch(Rectangle((KX - BLOCK / 2, -BLOCK / 2), BLOCK, BLOCK, fc=STEEL, alpha=0.5,
                           ec=STEEL, lw=THIN, zorder=8))
    pl.plot([0, 0], [-56, 86], lw=HAIR, c=GREY, ls=(0, (12, 3, 2, 3)))
    pl.text(-6, 90, "PLAN AT FULL BUMP", fontproperties=SERIF, fontsize=13, color=INK, va="top")
    pl.text(-6, 80, f"tie rod at 0° and ±{STEER:.0f}°  ·  both arms  ·  outward is up  ·  mm",
            fontproperties=LABEL, fontsize=6, color=GREY, va="top")

    # ---------------------------- clearance table --------------------------------
    tb = fig.add_axes([0.62, 0.085, 0.345, 0.45], facecolor="none")
    tb.set_xlim(0, 100); tb.set_ylim(0, 100); tb.axis("off")
    tb.text(0, 99, "CLEARANCE  ·  surface to surface, 3D", fontproperties=LABEL_M, fontsize=7,
            color=INK, va="top")
    tb.text(52, 94, f"AT +{BUMP:.0f}", fontproperties=LABEL_M, fontsize=5.8, color=GREY,
            ha="right")
    tb.text(100, 94, f"WORST OVER −{DROOP:.0f}…+{BUMP:.0f}  (where)", fontproperties=LABEL_M,
            fontsize=5.8, color=GREY, ha="right")
    tb.plot([0, 100], [91.5, 91.5], lw=THIN, c=INK)
    for i, k in enumerate(PAIRS):
        y = 88.5 - i * 3.45
        gb = AT_BUMP[k][0]
        gw, _, dzw, thw = OVER_TRAVEL[k]
        tb.text(0, y, f"{k[0]}  /  {k[1]}", fontproperties=LABEL, fontsize=6.2, color=INK,
                va="center")
        tb.text(52, y, f"{gb:5.1f}", fontproperties=MONO, fontsize=6.2,
                color=VERM if gb < TIGHT else INK, ha="right", va="center")
        tb.text(100, y, f"{gw:5.1f}   ({dzw:+.0f} mm, {thw:+.0f}°)", fontproperties=MONO,
                fontsize=6.2, color=VERM if gw < TIGHT else INK, ha="right", va="center")
        tb.plot([0, 100], [y - 1.72, y - 1.72], lw=HAIR, c=GREY, alpha=0.4)
    tb.text(0, 28.5, f"shock {SH_DROOP:.1f} at droop · {SHOCK_LEN:.0f} ride · {SH_BUMP:.1f} at "
            f"bump  ·  knuckle moves {-KO_BUMP[0]:.1f} mm outboard at bump  ·  red = under "
            f"{TIGHT:.0f} mm", fontproperties=LABEL, fontsize=5.6, color=GREY, va="bottom")
    title_block(tb, "suspension travel check  —  study 08", "2026-09-25")

    fig.savefig(os.path.join(os.path.dirname(__file__), "suspension_travel_study.png"),
                facecolor=PAPER)
    for k in PAIRS:
        g, _, dz, th = OVER_TRAVEL[k]
        print(f"{k[0]:>7} - {k[1]:<12} bump {AT_BUMP[k][0]:6.1f}   worst {g:6.1f} at dz {dz:+.1f} "
              f"th {th:+.1f}")
    print(f"shock {SH_DROOP:.1f} / {SHOCK_LEN:.0f} / {SH_BUMP:.1f}  knuckle x at bump "
          f"{KO_BUMP[0]:.2f}  T {T:.0f}  tower {SHOCK_TOP_Z:.0f}")
