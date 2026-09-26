"""Shared ggGridRunner chassis and corner geometry (mm) for the layout drawings.

Corner (study 07): the knuckle plate bolts to the gearbox face inside the wheel's hub pocket.
Each boomerang arm ends in a single stem with a fore-aft hinge pin into a small KINGPIN BLOCK;
a vertical kingpin pin joins the block to the knuckle (two plain pins = a pin universal joint,
no ball joints). Only the knuckle, motor, wheel and the two blocks move with the wheel.
The MG996R sits on the chassis spine, between the upper and lower arm pivots, and steers the
knuckle through a tie rod as long as the arms and parallel to them (1:1 parallelogram, ~zero
bump steer). The tie rod ends are pin universals too.

Local corner frame: origin on the kingpin at ground level, +x inboard ("a"), +y fore-aft
(outward), +z up. Rover frame on the sheets: +X forward, +Y left, +Z up.
"""
import numpy as np

# ---- measured parts ---------------------------------------------------------
TIRE_D, TIRE_W = 120.0, 42.0
HUB_D = 81.0                    # hub pocket diameter
HEX_IN = 33.0                   # hex mating face -> wheel inner (motor-side) face
GBX_FACE = 22.0                 # gearbox face -> hex mating face
MOTOR_BODY = 61.0               # gearbox + motor 51, + 10 encoder clearance
MOTOR_D = 25.0

# ---- layout -----------------------------------------------------------------
WB = 230.0                      # wheelbase (kingpin c-c fore-aft)
STEER = 46.0                    # mechanical steering clearance, deg (±45 commanded, stops ±47)
BUMP, DROOP = 15.0, 10.0        # wheel travel from ride height (25 total)
AXLE = TIRE_D / 2

# ---- kingpin blocks on the knuckle (move with the wheel) -----------------------
KP_UP_Z, KP_LO_Z = 86.0, 32.0   # upper / lower kingpin block: arm hinge + kingpin pin
BLOCK = 10.0                    # block size (cube edge)

# ---- arms: boomerangs with a single stem at the knuckle ------------------------
ARM = 95.0                      # stem hinge to chassis pivot, along x
PIVOT = 35.0                    # chassis pivots (arms, servo) from the centreline
RISE = 15.0                     # chassis pivots sit this much above the knuckle hinges
                                # (= BUMP, so the tie rod never closes on the motor)
STEM = 35.0                     # single stem from the knuckle, then the legs split
LO_SPREAD, UP_SPREAD = 20.0, 22.0  # half-spread of the legs at the chassis pivots
ARM_HALF = 3.5                  # arm section half-size (7 mm where it passes the pocket rim)
# Boomerang shapes: (distance inboard from the kingpin, height above the knuckle hinge).
# Upper climbs gently from the knuckle (it must stay inside the hub pocket until it is out of
# the wheel) and then runs flat; lower runs flat under the motor and climbs past its tail.
BEND_UP, BEND_LO = 76.0, 66.0
SHAPES = {
    "upper": ([0.0, BEND_UP, ARM], [0.0, RISE, RISE]),
    "lower": ([0.0, BEND_LO, ARM], [0.0, 0.0, RISE]),
    "tie rod": ([0.0, ARM], [0.0, RISE]),
}

# ---- steering: MG996R on the spine, tie rod parallel to the arms ----------------
TR_Z = 78.0                     # tie rod knuckle end (steering arm on the knuckle)
STEER_ARM = 12.0                # knuckle steering arm = servo horn, pointing fore-aft
TR_HALF = 2.0
HORN_Z = TR_Z + RISE            # horn on top of the servo, on the spine
SERVO_STANDOFF = 12.0           # horn above the servo body top, so the tie rod clears it
# Body lies along the arm, output 10 mm from its outboard end, between the leg pivots.
SERVO_BOX = ((ARM - 10, ARM + 30.7), (-10.0, 10.0),
             (HORN_Z - SERVO_STANDOFF - 37, HORN_Z - SERVO_STANDOFF))  # body, local

# ---- shock: upper arm to a central tower -------------------------------------
SHOCK_AT = 40.0                 # mount on the upper arm, this far in from the knuckle
SHOCK_EYE_UP = 10.0             # lower eye sits on a boss this far above the arm centreline
SHOCK_LEN = 100.0               # eye to eye at ride height
SHOCK_R = 5.0
TOWER_X = 10.0                  # tower top mount, from centreline
PIV_BOSS = 5.0

# ---- battery / print ----------------------------------------------------------
BAT_L, BAT_W, BAT_Z = 144.0, 65.0, 36.0   # 4S4P brick: two layers of 8 cells along BAT_L
BAT_X, BAT_Y = BAT_W, BAT_L     # plan size: long side fore-aft
ENV = 140.0                     # print envelope, per part

# ---- derived ------------------------------------------------------------------
KP_X = PIVOT + ARM              # kingpin from centreline; kingpin = gearbox face
TIRE_IN = HEX_IN - GBX_FACE     # wheel inner face, inboard of the kingpin
TIRE_OUT = TIRE_IN - TIRE_W
SCRUB = -(TIRE_IN + TIRE_OUT) / 2  # kingpin -> tire centre
T = 2 * (KP_X + SCRUB)          # track, tire c-c
HEX_X = -GBX_FACE
TAIL_X = MOTOR_BODY
TAIL_R = float(np.hypot(MOTOR_BODY, MOTOR_D / 2))
POCKET_BOTTOM = TIRE_IN - HEX_IN   # = HEX_X: the wheel web
BELLY = KP_LO_Z + RISE - 8      # spine underside near the lower pivots


def shape(a, name):
    """Height of a link above its knuckle hinge at distance a (inboard) from the kingpin."""
    return np.interp(a, *SHAPES[name])


SHOCK_LO = np.array([KP_X - SHOCK_AT, KP_UP_Z + float(shape(SHOCK_AT, "upper")) + SHOCK_EYE_UP])
SHOCK_TOP_Z = float(SHOCK_LO[1] + np.sqrt(SHOCK_LEN ** 2 - (SHOCK_LO[0] - TOWER_X) ** 2))


def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)
