## FUNCTIONS

# Write your code here :-)
import math
import time
from servo import Servo, servo2040, ServoCluster
import gc

# Define constants
# BUG: walking used height 100 while standing used 70, causing a pose jump.
# FIX: share the existing main.py standing height across both modules.
Robot_height = 70
path_radius = 50
angle_L1 = math.radians(30)
angle_L2 = math.radians(0)
angle_L3 = math.radians(-30)
angle_R1 = math.radians(150)
angle_R2 = math.radians(180)
angle_R3 = math.radians(-150)

path_center_L1 = [path_radius * math.cos(angle_L1), 175, Robot_height]
path_center_L2 = [path_radius * math.cos(angle_L2), 175, Robot_height]
path_center_L3 = [path_radius * math.cos(angle_L3), 175, Robot_height]
path_center_R1 = [path_radius * math.cos(angle_R1), 175, Robot_height]
path_center_R2 = [path_radius * math.cos(angle_R2), 175, Robot_height]
path_center_R3 = [path_radius * math.cos(angle_R3), 175, Robot_height]
motor_distance = 150
hex_radius = motor_distance + 200

R1o = [-20, -12, -12]  # deg #recalibrate servo 1
# BUG: these corrections differed between standing and walking.
# FIX: use main.py's existing corrections as the shared calibration.
# These measured offsets still need verification on the physical robot.
R2o = [-20, 15, -10]  # deg
R3o = [-37, 0, -20]  # deg
L1o = [-25, 5, 0]  # deg
L2o = [-15, 3, 4]  # deg
L3o = [-20, 0, 0]  # deg

# Web calibration saves offsets separately so source edits do not erase them.
# Validate the complete file before applying any saved values.
try:
    import calibration_store
except ImportError:
    calibration_store = None  # Existing Thonny-only installations still work.
if calibration_store is not None:
    saved_offsets = calibration_store.load()
    if saved_offsets is not None:
        L1o, L2o, L3o = (saved_offsets[key] for key in ('L1o', 'L2o', 'L3o'))
        R1o, R2o, R3o = (saved_offsets[key] for key in ('R1o', 'R2o', 'R3o'))

# Create legs
Left1 = [0, 1, 2]
Left2 = [3, 4, 5]
Left3 = [6, 7, 8]
Right1 = [9, 10, 11]
Right2 = [12, 13, 14]
Right3 = [15, 16, 17]

gc.collect()

# Create a servo cluster for pins 0 to 18, using PIO 0 and State Machine 0
# servo_cluster.value(0 -> 17, 0)
START_PIN = servo2040.SERVO_1
END_PIN = servo2040.SERVO_18
servo_cluster = ServoCluster(pio=0, sm=0, pins=list(range(START_PIN, END_PIN + 1)))

# Joint output limits in degrees (after calibration offsets) for the
# (hip, upper leg, lower leg) servos. HARD_JOINT_LIMITS is the widest range the
# web app's limit editor may save; JOINT_LIMITS is what poses are checked
# against. Limits saved from the app (joint_limits.json) replace the defaults.
# The default servo calibration silently clamps value() to -90..90, so allow
# extrapolation past it (0 deg stays 1500 us; 180 deg is 3500 us). The bridge
# reads each value back and stops if the driver did not apply it.
HARD_JOINT_LIMITS = ((-180, 180), (-180, 180), (-180, 180))
JOINT_LIMITS = HARD_JOINT_LIMITS
# Older calibration_store.py files have no limit storage.
if calibration_store is not None and hasattr(calibration_store, 'load_limits'):
    saved_limits = calibration_store.load_limits(HARD_JOINT_LIMITS)
    if saved_limits is not None:
        JOINT_LIMITS = saved_limits
for _pin in range(START_PIN, END_PIN + 1):
    _cal = servo_cluster.calibration(_pin)
    _cal.limit_to_calibration(False, False)
    servo_cluster.calibration(_pin, _cal)


# linspace emulator
def linspace_emulation(start, stop, num):
    # BUG: speed calculations return floats; range(float) raises TypeError.
    # FIX: round up to an integer and require at least two endpoints.
    if not math.isfinite(num) or num < 2:
        raise ValueError("num must be finite and at least 2")
    num = int(math.ceil(num))
    step = (stop - start) / (num - 1)
    array = [start]
    for i in range(1, num):
        array.append(start + i * step)
    # FIX: preserve the exact endpoint instead of accumulating rounding error.
    array[-1] = stop
    return array


# Servo Enable (input = array)
def enab(servos):
    for i in servos:
        servo_cluster.enable(i)


# Servo Disable (input = array)
def disab(servos):
    for i in servos:
        servo_cluster.disable(i)


# IK FUNCTION
def calculate_angles(x_p, y_p, z_p):
    """
    Calculate the motor angles (S1, S2, S3) for a robot leg given its xyz coordinates.

    Parameters:
    x_p, y_p, z_p (float): xyz coordinates of the robot leg

    Returns:
    tuple: (S1, S2, S3) motor angles in degrees
    """
    # BUG: invalid targets reached division/acos with invalid arguments.
    # FIX: reject non-finite or unreachable targets before computing angles.
    if not all(math.isfinite(v) for v in (x_p, y_p, z_p)):
        raise ValueError("Leg coordinates must be finite")

    # Leg Lengths
    L1 = 47
    L2 = 88.46
    L3 = 148.8

    A_p = math.sqrt(x_p**2 + y_p**2)
    # The hip angle is undefined for a foot on the hip's vertical axis.
    if A_p < 1:
        raise ValueError("Foot is too close to the hip axis (X and Y near 0)")
    B_p = math.sqrt((A_p - L1) ** 2 + z_p**2)

    if B_p < abs(L2 - L3) - 1e-6 or B_p > L2 + L3 + 1e-6:
        raise ValueError("Leg target is outside the reachable workspace")

    # FIX: clamp roundoff at reachable boundaries, after checking reachability.
    def acos_degrees(value):
        return math.degrees(math.acos(max(-1.0, min(1.0, value))))

    theta_1 = math.degrees(
        math.atan2(y_p, x_p)
    )  # Use arctan2 to handle quadrant determination
    beta = acos_degrees(((L2**2) + (B_p**2) - (L3**2)) / (2 * L2 * B_p))
    epsilon = math.degrees(math.atan2((A_p - L1), z_p))

    sigma_2 = beta - (90 - epsilon)
    sigma_3 = acos_degrees(((L2**2) + (L3**2) - (B_p**2)) / (2 * L2 * L3))

    # Calculate motor angles
    S1 = 90 - theta_1
    S2 = sigma_2
    S3 = 180 - sigma_3
    return [S1, S2, S3]


def standby_position(x_p, y_p, z_p, sleep, leg, order, O):
    """
    O = servo correction
    """

    # BUG: the requested delay was ignored (only 10 microseconds per joint).
    # FIX: validate and calculate the full pose first, then honor its delay.
    if order not in ("left", "right"):
        raise ValueError("order must be left or right")
    if not math.isfinite(sleep) or sleep < 0:
        raise ValueError("sleep must be finite and non-negative")
    angles = calculate_angles(x_p, y_p, z_p)
    sign = 1 if order == "left" else -1
    targets = (angles[0] + O[0], sign * angles[1] + O[1],
               sign * (angles[2] - 90) + O[2])
    for pin, target in zip(leg, targets):
        servo_cluster.value(pin, target)
        time.sleep(sleep)


def get_res_from_speed(V_hex):
    V_m_angular = 315  # degrees/s
    V_m = (315 / 360) * (2 * math.pi * 200)  # mm/s
    C_unit = 300  # units per second
    U_step = 2 * 50

    # BUG: zero speed divides by zero; excessive speed produces negative sleep.
    # FIX: reject speeds outside the model's positive operating range.
    if not math.isfinite(V_hex) or not 0 < V_hex <= V_m:
        raise ValueError("Speed must be positive and no greater than servo speed")

    # calculate U_d
    U_d = V_m / (C_unit * (1 + V_m * ((1 / V_hex) - (1 / V_m))))

    # calculate sleep time
    step_interval = U_d * ((1 / V_hex) - (1 / V_m))

    # calculate resolution
    path_resolution = U_step / (((1 / C_unit) - step_interval) * V_m)
    return [path_resolution, step_interval]


step_interval = get_res_from_speed(1000)[1]

# Pre_gait is used to transition from the standing pose to a pose from which to start walking
def Pre_gait(
    leg,
    O,
    path_center,
    path_radius,
    direction_vector,
    leg_angle,
    hex_radius,
    order,
    side,
    i2,
    j2,
    k2,
):
    global step_interval

    try:
        if order == "drag":
            #####-------DRAG-------#####
            # Drag Forward
            x_L_F = (0 - j2) * math.cos(leg_angle)
            y_L_F = (0 - j2) * math.sin(leg_angle)
            z_L_F = 0

            #Turn
            x_L_T = -(0 - 249 * k2 * math.tan(math.pi / 18))
            y_L_T = 0

            # Drag
            x_L = (
                path_center[0]
                + direction_vector[1] * x_L_F
                + direction_vector[0] * x_L_T
            )

            y_L = (
                path_center[1]
                + direction_vector[1] * y_L_F
                + abs(direction_vector[0] * y_L_T)
            )

            z_L = path_center[2]

            if side == "left":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], L_angles[2] + O[2] - 90)
                time.sleep(step_interval)

            if side == "right":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], -L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], -(L_angles[2] - O[2] - 90))
                time.sleep(step_interval)
                
        elif order == "lift":
            #####-------LIFT-------#####
            # # Lift Forward
            x_L_lift_F = (0 + j2) * math.cos(leg_angle)
            y_L_lift_F = (0 + j2) * math.sin(leg_angle)
            z_L_lift_F = -path_radius * math.sin(i2)

            #Turning
            x_L_T = -(0 + 249 * k2 * math.tan(math.pi / 18))
            y_L_T = 0

            # Lift
            x_L = (
                path_center[0]
                + direction_vector[1] * x_L_lift_F
                + direction_vector[0] * x_L_T
            )

            y_L = (
                path_center[1]
                + direction_vector[1] * y_L_lift_F
                + abs(direction_vector[0] * y_L_T)
            )

            z_L = path_center[2] - path_radius * math.sin(i2)

            if side == "left":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], L_angles[2] + O[2] - 90)
                time.sleep(step_interval)

            if side == "right":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], -L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], -(L_angles[2] - O[2] - 90))
                time.sleep(step_interval)

    except (ValueError, TypeError) as e:
        print(f"Error occurred: {e}")
        print(f"x_L: {x_L}, y_L: {y_L}, z_L: {z_L}")
        raise  # Rethrow the exception for debugging purposes


def Gait(
    leg,
    O,
    path_center,
    path_radius,
    direction_vector,
    leg_angle,
    hex_radius,
    order,
    side,
    i,
    j,
    k,
):
    global step_interval

    try:
        if order == "drag":
            #####-------DRAG-------#####
            # Drag Forward
            x_L_F = (path_radius - j) * math.cos(leg_angle)
            y_L_F = (path_radius - j) * math.sin(leg_angle)
            z_L_F = 0

            x_L_T = -(
                249 * math.tan(math.pi / 18) - 2 * 249 * k * math.tan(math.pi / 18)
            )
            y_L_T = 0

            # Drag
            x_L = (
                path_center[0]
                + direction_vector[1] * x_L_F
                + direction_vector[0] * x_L_T
            )

            y_L = (
                path_center[1]
                + direction_vector[1] * y_L_F
                + abs(direction_vector[0] * y_L_T)
            )
            z_L = path_center[2]

            if side == "left":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], L_angles[2] + O[2] - 90)
                time.sleep(step_interval)

            if side == "right":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], -L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], -(L_angles[2] - O[2] - 90))
                time.sleep(step_interval)

        elif order == "lift":
            # Lift Forward
            x_L_lift_F = -path_radius * math.cos(i) * math.cos(leg_angle)
            y_L_lift_F = -path_radius * math.cos(i) * math.sin(leg_angle)
            z_L_lift_F = -path_radius * math.sin(i)

            x_L_T = -(
                -249 * math.tan(math.pi / 18) + 2 * 249 * k * math.tan(math.pi / 18)
            )
            y_L_T = 0

            # Lift
            x_L = (
                path_center[0]
                + direction_vector[1] * x_L_lift_F
                + direction_vector[0] * x_L_T
            )

            y_L = (
                path_center[1]
                + direction_vector[1] * y_L_lift_F
                + abs(direction_vector[0] * y_L_T)
            )

            z_L = path_center[2] - 1.5 * path_radius * math.sin(i)

            if side == "left":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], L_angles[2] + O[2] - 90)
                time.sleep(step_interval)

            if side == "right":
                L_angles = calculate_angles(x_L, y_L, z_L)
                servo_cluster.value(leg[0], L_angles[0] + O[0])
                time.sleep(step_interval)
                servo_cluster.value(leg[1], -L_angles[1] + O[1])
                time.sleep(step_interval)
                servo_cluster.value(leg[2], -(L_angles[2] - O[2] - 90))
                time.sleep(step_interval)

    except (ValueError, TypeError) as e:
        print(f"Error occurred: {e}")
        print(f"x_L: {x_L}, y_L: {y_L}, z_L: {z_L}")
        raise  # Rethrow the exception for debugging purposes


def get_unit_vector(realvector):
    # BUG: NaN/Infinity could silently stop or corrupt a gait.
    # FIX: validate controller coordinates before normalization.
    if len(realvector) != 2 or not all(math.isfinite(v) for v in realvector):
        raise ValueError("Direction must contain two finite numbers")
    vector = realvector
    magnitude = math.sqrt((vector[1] ** 2) + (vector[0] ** 2))
    if magnitude > 300:
        unit_vector = [-vector[0] / magnitude, -vector[1] / magnitude]
    else:
        unit_vector = [0, 0]
    return unit_vector


def Pre_walk(directionvector):
    direction_vector = get_unit_vector(directionvector)
    speed = 500
    # BUG: joint delays always used the global speed of 1000.
    # FIX: use this gait's speed for both resolution and timing.
    global step_interval
    path_resolution, step_interval = get_res_from_speed(speed)
    i2_values = linspace_emulation(
        0, math.pi, (path_resolution / 2)
    )  # points for lifting
    j2_values = linspace_emulation(
        0, path_radius, (path_resolution / 2)
    )  # points for dragging
    k2_values = linspace_emulation(0.01, 1, (path_resolution / 2))  # points for turning
    magnitude = math.sqrt((direction_vector[0] ** 2) + (direction_vector[1] ** 2))
    if magnitude > 0:
        for i, j, k in zip(i2_values, j2_values, k2_values):

            # Calculate sleep time
            error = abs((math.pi / 2) - i)
            gain = 10
            sleep_time = step_interval * (1 + gain * error)

            # Walk
            Pre_gait(
                Left1,
                L1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_L1,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Left2,
                L2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_L2,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Left3,
                L3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_L3,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right1,
                R1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_R1,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right2,
                R2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_R2,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right3,
                R3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_R3,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            time.sleep(sleep_time)

        for i, j, k in zip(i2_values, j2_values, k2_values):
            # BUG: phase two reused phase one's final delay.
            # FIX: recalculate the delay for the current phase angle.
            sleep_time = step_interval * (1 + 10 * abs(math.pi / 2 - i))
            Pre_gait(
                Left1,
                L1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_L1,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Left2,
                L2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_L2,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Left3,
                L3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_L3,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right1,
                R1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_R1,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right2,
                R2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_R2,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Pre_gait(
                Right3,
                R3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_R3,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            time.sleep(sleep_time)


def walk(directionvector):
    print("walking")
    direction_vector = get_unit_vector(directionvector)
    # direction_vector = [0,1]
    speed = 700
    # BUG: joint delays always used the global speed of 1000.
    # FIX: use this gait's speed for both resolution and timing.
    global step_interval
    path_resolution, step_interval = get_res_from_speed(speed)
    i_values = linspace_emulation(0, math.pi, path_resolution)  # points for lifting
    j_values = linspace_emulation(
        0, 2 * path_radius, path_resolution
    )  # points for dragging
    k_values = linspace_emulation(0.01, 1, path_resolution)  # points for turning
    magnitude = math.sqrt((direction_vector[0] ** 2) + (direction_vector[1] ** 2))

    print(direction_vector)
    print(step_interval)

    if magnitude > 0:
        for i, j, k in zip(i_values, j_values, k_values):
            # Calculate sleep time
            error = abs((math.pi / 2) - i)
            gain = 10
            sleep_time = step_interval * (1 + gain * error)
            # Walk
            Gait(
                Left1,
                L1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_L1,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Left2,
                L2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_L2,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Left3,
                L3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_L3,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right1,
                R1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_R1,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right2,
                R2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_R2,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right3,
                R3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_R3,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            time.sleep(sleep_time)

        for i, j, k in zip(i_values, j_values, k_values):
            # BUG: phase two omitted the per-frame delay entirely.
            # FIX: apply the same timing profile as phase one.
            sleep_time = step_interval * (1 + 10 * abs(math.pi / 2 - i))
            Gait(
                Left1,
                L1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_L1,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Left2,
                L2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_L2,
                hex_radius,
                "drag",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Left3,
                L3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_L3,
                hex_radius,
                "lift",
                "left",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right1,
                R1o,
                path_center_L1,
                path_radius,
                direction_vector,
                angle_R1,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right2,
                R2o,
                path_center_L2,
                path_radius,
                direction_vector,
                angle_R2,
                hex_radius,
                "lift",
                "right",
                i,
                j,
                k,
            )
            # time.sleep(step_interval)
            Gait(
                Right3,
                R3o,
                path_center_L3,
                path_radius,
                direction_vector,
                angle_R3,
                hex_radius,
                "drag",
                "right",
                i,
                j,
                k,
            )
            time.sleep(sleep_time)
