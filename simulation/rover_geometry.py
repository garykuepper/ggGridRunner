"""Shared ggGridRunner chassis and corner geometry (mm) for the layout drawings.

Corner (study 05): a knuckle plate bolts to the gearbox face inside the wheel's hub pocket and
steers on a vertical kingpin held by a separate UPRIGHT. The upright is a C: two prongs reach
into the pocket above and below the motor to the kingpin pivots, joined by a back that sits
outside the motor's steering sweep. The suspension arms hinge on the upright's back; the MG996R
rides on the upright and steers the knuckle through a short 1:1 parallelogram link. The motor,
wheel, upright and servo move together through the travel, so only steering changes the gaps
between them.

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
STEER = 50.0                    # mechanical steering clearance, deg
BUMP, DROOP = 15.0, 10.0        # wheel travel from ride height (25 total)

# ---- upright (moves with the suspension, does not steer) ----------------------
AXLE = TIRE_D / 2
KP_R = 28.0                     # kingpin pivots this far above / below the axle
KP_UP_Z, KP_LO_Z = AXLE + KP_R, AXLE - KP_R
PRONG_T, PRONG_W = 8.0, 10.0    # prong section: height x width
TAIL_R = float(np.hypot(MOTOR_BODY, MOTOR_D / 2))   # motor tail swing radius
UPR_BACK = (float(np.ceil(TAIL_R + 4)), float(np.ceil(TAIL_R + 4)) + 8.0)  # back, along a
UPR_BACK_W = 12.0               # back half-width, fore-aft
UPR_Z = (KP_LO_Z - PRONG_T / 2, KP_UP_Z + PRONG_T / 2)  # upright bottom / top
PRONG_LO_Z = (KP_LO_Z - PRONG_T / 2, KP_LO_Z + PRONG_T / 2)
PRONG_UP_Z = (KP_UP_Z - PRONG_T / 2, KP_UP_Z + PRONG_T / 2)

# ---- steering: MG996R on the upright, 1:1 parallelogram link -------------------
SERVO_A = 48.0                  # servo output shaft, inboard of the kingpin
SERVO_BOX = ((SERVO_A - 10, SERVO_A + 30.7), (-10.0, 10.0),
             (UPR_Z[1] + 1, UPR_Z[1] + 38))            # body sits on the top prong
HORN_Z = 78.0                   # horn / link height: between motor top and top prong
STEER_ARM = 15.0                # knuckle steering arm = servo horn, pointing fore-aft
LINK_HALF = 2.0

# ---- arms (straight, parallel, equal; hinge on the upright back) ----------------
HINGE_A = UPR_BACK[1] + 3       # outer hinge axis, inboard of the kingpin
LO_H_Z, UP_H_Z = 38.0, 82.0     # outer hinge heights on the upright back
ARM = 50.0                      # hinge to chassis pivot, along x
RISE = 20.0                     # chassis pivots sit this much above the outer hinges:
                                # the chassis rides higher and the wheels hang lower
PIVOT = 15.0                    # chassis pivots from the centreline
ARM_W_OUT, ARM_W_IN = 10.0, 20.0  # half-width of the arm frame at the upright / chassis
ARM_HALF = 4.0                  # arm rail half-thickness
PIV_BOSS = 5.0

# ---- shock: lower arm to a central tower -------------------------------------
SHOCK_ON_ARM = 10.0             # mount on the lower arm, this far in from the outer hinge
SHOCK_LEN = 100.0               # eye to eye at ride height
SHOCK_R = 5.0
TOWER_X = 10.0                  # tower top mount, from centreline

# ---- battery / print ----------------------------------------------------------
BAT_L, BAT_W, BAT_Z = 144.0, 65.0, 36.0   # 4S4P brick: two layers of 8 cells along BAT_L
BAT_X, BAT_Y = BAT_W, BAT_L     # plan size: long side fore-aft
ENV = 140.0                     # print envelope, per part

# ---- derived ------------------------------------------------------------------
KP_X = PIVOT + ARM + HINGE_A    # kingpin from centreline; kingpin = gearbox face
TIRE_IN = HEX_IN - GBX_FACE     # wheel inner face, inboard of the kingpin
TIRE_OUT = TIRE_IN - TIRE_W
SCRUB = -(TIRE_IN + TIRE_OUT) / 2  # kingpin -> tire centre
T = 2 * (KP_X + SCRUB)          # track, tire c-c
HEX_X = -GBX_FACE
TAIL_X = MOTOR_BODY
POCKET_BOTTOM = TIRE_IN - HEX_IN   # = HEX_X: the wheel web
BELLY = LO_H_Z + RISE - 8       # spine underside near the lower pivots
_SH_LO = np.array([KP_X - HINGE_A - SHOCK_ON_ARM, LO_H_Z + RISE * SHOCK_ON_ARM / ARM])
SHOCK_TOP_Z = float(_SH_LO[1] + np.sqrt(SHOCK_LEN ** 2 - (_SH_LO[0] - TOWER_X) ** 2))


def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)
