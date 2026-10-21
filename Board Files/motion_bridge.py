"""Non-blocking USB pose player. Positions are commands, never sensor readings."""
import json
import math
import select
import sys
import time
import FUNCTIONS as robot

PROTOCOL = 'hexapod-motion-v1'
NAMES = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
PINS = (robot.Left1, robot.Left2, robot.Left3, robot.Right1, robot.Right2, robot.Right3)
OFFSETS = (robot.L1o, robot.L2o, robot.L3o, robot.R1o, robot.R2o, robot.R3o)
HOME = [[0, 175, robot.Robot_height] for _ in NAMES]
# Full leg reach on every axis; calculate_angles rejects unreachable points.
LIMITS = ((-284, 284), (-284, 284), (-284, 284))
TIMEOUT_MS = 3000
# Transition time per pose, shared with the web app (model.js).
MIN_DURATION_MS, MAX_DURATION_MS = 100, 10000
JOINTS = ('hip', 'upper leg', 'lower leg')
# A servo whose read-back differs by more than this did not apply the command.
READBACK_TOLERANCE = 0.5
# Leg calibration (from the calibration web app's web_bridge.py): one leg holds
# the home reference pose while its offsets change by at most this per command.
MAX_ADJUSTMENT = 15
OFFSET_KEYS = ('L1o', 'L2o', 'L3o', 'R1o', 'R2o', 'R3o')

_sensors = None  # None = not set up yet, False = unavailable on this board/firmware


def read_power():
    """Return {'voltage': V, 'current': A} from the Servo 2040 sensors, or None.

    Never raises: a sensor problem must not interrupt movement or the watchdog.
    """
    global _sensors
    if _sensors is False:
        return None
    try:
        if _sensors is None:
            from servo import servo2040
            from pimoroni import Analog, AnalogMux
            from machine import Pin
            mux = AnalogMux(servo2040.ADC_ADDR_0, servo2040.ADC_ADDR_1,
                            servo2040.ADC_ADDR_2, muxed_pin=Pin(servo2040.SHARED_ADC))
            current = Analog(servo2040.SHARED_ADC, servo2040.CURRENT_GAIN,
                             servo2040.SHUNT_RESISTOR, servo2040.CURRENT_OFFSET)
            voltage = Analog(servo2040.SHARED_ADC, servo2040.VOLTAGE_GAIN)
            _sensors = (mux, current, voltage,
                        servo2040.CURRENT_SENSE_ADDR, servo2040.VOLTAGE_SENSE_ADDR)
        mux, current, voltage, current_addr, voltage_addr = _sensors
        mux.select(current_addr)
        amps = current.read_current()
        mux.select(voltage_addr)
        volts = voltage.read_voltage()
        return {'voltage': round(volts, 2), 'current': round(amps, 3)}
    except Exception:
        _sensors = False
        return None


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def leg_angles(index, xyz, offsets=None):
    """Corrected servo angles for one leg, checked against the joint limits."""
    try:
        a = robot.calculate_angles(*xyz)
    except ValueError as error:
        raise ValueError('%s: %s' % (NAMES[index], error))
    sign = 1 if index < 3 else -1
    base = (a[0], sign * a[1], sign * (a[2] - 90))
    offsets = OFFSETS[index] if offsets is None else offsets
    angles = [base[j] + offsets[j] for j in range(3)]
    for joint, value, (low, high) in zip(JOINTS, angles, robot.JOINT_LIMITS):
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError('%s %s %.1f deg is outside its joint angle limit %g..%g; adjust the pose or calibration'
                             % (NAMES[index], joint, value, low, high))
    return angles


def pose_angles(pose):
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError('A pose must contain XYZ for all six legs')
    result = []
    for index, xyz in enumerate(pose):
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError('Each leg needs X, Y and Z')
        for value, limits in zip(xyz, LIMITS):
            if not number(value) or not limits[0] <= value <= limits[1]:
                raise ValueError('Foot coordinate outside training workspace')
        result.append(leg_angles(index, xyz))
    return result


def steps(duration):
    # Interpolate every 20 ms, capped at 200 steps so long transitions stay
    # quick to preflight. Must match interpolationSteps() in model.js.
    return max(1, min(200, duration // 20))


# Live walking: the board computes every foot's path itself (inverse kinematics
# every 20 ms), so the robot walks continuously instead of stepping between
# poses. Must match walkPose() in model.js (checked by tests/walk_fixture.json).
# gait: (fraction of the cycle each foot is on the ground,
#        cycle phase at which each leg L1, L2, L3, R1, R2, R3 starts its swing)
WALK_GAITS = {
    'tripod': (1 / 2, (0, 1 / 2, 0, 1 / 2, 0, 1 / 2)),
    'ripple': (2 / 3, (2 / 3, 1 / 3, 0, 1 / 6, 5 / 6, 1 / 2)),
    'wave': (5 / 6, (2 / 6, 1 / 6, 0, 5 / 6, 4 / 6, 3 / 6)),
}
LEG_ANGLES = tuple(math.radians(a) for a in (30, 0, -30, 150, 180, -150))
WALK_RAMP = 1.0  # cycles to speed up from standing, and to slow down to a stop
WALK_SLEW = 1.0  # direction change per cycle while steering
WALK_WRITE_MS = 20
# How fast new settings are reached while walking, per cycle.
WALK_SETTING_SLEW = (('stride', 40.0), ('lift', 30.0), ('hip_swing', 20.0))


def approach(value, goal, limit):
    return value + max(-limit, min(limit, goal - value))


def walk_pose(base, gait, direction, stride, lift, hip_swing, amplitude, t):
    """Foot positions at cycle time t (0..1). amplitude 0 is the base stance.

    Each foot swings forward along a raised arc, then pushes straight back on
    the ground. direction is (forward, sideways left, turn left), each -1..1.
    """
    stance, phases = WALK_GAITS[gait]
    forward, lateral, turn = direction
    swing = 1 - stance
    pose = []
    for i in range(6):
        p = (t - phases[i]) % 1.0
        if p < swing:
            s = p / swing
            k = -0.5 + (1 - math.cos(math.pi * s)) / 2
            up = math.sin(math.pi * s)
        else:
            k = 0.5 - (p - swing) / stance
            up = 0.0
        k *= amplitude
        th = LEG_ANGLES[i]
        # Step direction in this leg's frame: X across the leg (hip rotation), Y along it.
        across = forward * math.cos(th) - lateral * math.sin(th) + turn
        along = forward * math.sin(th) + lateral * math.cos(th)
        bx, by, bz = base[i]
        if hip_swing > 0:
            sw = math.radians(hip_swing)
            reach = math.sqrt(bx * bx + by * by) + k * along * 175 * sw  # MicroPython has no hypot()
            angle = math.atan2(bx, by) + k * across * sw
            x, y = reach * math.sin(angle), reach * math.cos(angle)
        else:
            x, y = bx + k * stride * across, by + k * stride * along
        pose.append([x, y, bz - amplitude * lift * up])
    return pose


def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def walk_amplitude(cycle, stop_at):
    """Speed up over the first cycle; slow to a stop over the last."""
    amplitude = smoothstep(cycle / WALK_RAMP)
    if stop_at is not None:
        amplitude = min(amplitude, smoothstep(1 - (cycle - stop_at) / WALK_RAMP))
    return amplitude


def ease(fraction, mode):
    # 'smooth' slows the start and end of a transition along the same path.
    if mode == 'smooth':
        return 0.5 - 0.5 * math.cos(math.pi * fraction)
    return fraction


def blend(start, end, fraction):
    return [[start[i][j] + (end[i][j] - start[i][j]) * fraction
             for j in range(3)] for i in range(6)]


class Controller:
    def __init__(self):
        self.active = False
        self.current = None
        self.angles = None
        self.transition = None
        # One pose may wait behind the current transition and start the moment
        # it ends, so a sequence plays without a pause between poses.
        self.queued = None
        self.walk = None  # Live walk computed on the board, see walk_pose()
        # True after a walk was interrupted mid-step: some feet may be in the
        # air, so that pose must not become the stance of a new walk.
        self.mid_step = False
        self.released = set()  # Legs whose outputs were released individually.
        self.cal_leg = None  # Leg selected for calibration
        self.cal_holding = False  # True while that leg alone holds the reference pose
        self.last_contact = time.ticks_ms()
        self.last_tick = self.last_contact
        self.off()

    def stop_motion(self):
        if self.walk is not None:
            self.mid_step = True
        self.transition = None
        self.queued = None
        self.walk = None

    def off(self):
        self.stop_motion()
        errors = []
        for pins in PINS:
            for pin in pins:
                try:
                    robot.servo_cluster.disable(pin)
                except Exception as error:
                    errors.append(str(error))
        if errors:
            raise RuntimeError('Some outputs could not be disabled: ' + '; '.join(errors))
        self.active = False
        self.released = set()
        self.cal_holding = False

    def write_servo(self, pin, value):
        robot.servo_cluster.value(pin, value)
        applied = robot.servo_cluster.value(pin)
        if not abs(applied - value) <= READBACK_TOLERANCE:
            raise ValueError('Servo %d did not accept %.1f deg (driver reports %.1f); '
                             'motors disabled' % (pin, value, applied))

    def write_pose(self, pose, angles):
        # Validate the complete pose before this method. Never partly apply an
        # invalid pose. Driver failures stop outputs and terminate the request.
        try:
            for pins, values in zip(PINS, angles):
                for pin, value in zip(pins, values):
                    self.write_servo(pin, value)
        except Exception:
            self.off()
            raise
        self.current = [xyz[:] for xyz in pose]
        self.angles = [values[:] for values in angles]

    def state(self, include_config=False):
        state = {'protocol': PROTOCOL, 'active': self.active,
                 'moving': self.transition is not None or self.walk is not None,
                 'queued': self.queued is not None,
                 'walking': self.walk_state(),
                 'mid_step': self.mid_step,
                 'positions': self.current, 'angles': self.angles,
                 'released': sorted(self.released),
                 'joint_limits': robot.JOINT_LIMITS,
                 # Offsets are live: calibration changes them for every later pose.
                 'offsets': OFFSETS,
                 'calibration': {'leg': self.cal_leg, 'holding': self.cal_holding,
                                 'reference': HOME[0], 'max_adjustment': MAX_ADJUSTMENT,
                                 'targets': self.cal_targets()},
                 'power': read_power()}
        if include_config:
            state.update({'names': NAMES, 'offsets': OFFSETS, 'home': HOME,
                          'limits': LIMITS, 'hard_joint_limits': robot.HARD_JOINT_LIMITS,
                          'channels': PINS})
        return state

    def handle(self, request):
        if not isinstance(request, dict):
            raise ValueError('Expected an object')
        op = request.get('op')
        if op in ('hello', 'status', 'ping'):
            pass
        elif op == 'arm':
            if self.walk is not None:
                raise ValueError('Stop walking first')
            if self.transition is not None:
                raise ValueError('Pause the current movement first')
            angles = pose_angles(HOME)
            self.write_pose(HOME, angles)
            self.active = True
            self.released = set()
            self.cal_holding = False
            self.mid_step = False
        elif op == 'pose':
            if not self.active or self.current is None:
                raise ValueError('Enable at home before sending movement')
            if self.walk is not None:
                raise ValueError('Stop walking first')
            queue = request.get('queue') is True
            if self.transition is not None and not queue:
                raise ValueError('Wait for this pose to finish, or pause it')
            if queue and self.queued is not None:
                raise ValueError('A pose is already queued')
            duration = request.get('duration_ms')
            if type(duration) is not int or not MIN_DURATION_MS <= duration <= MAX_DURATION_MS:
                raise ValueError('Duration must be %d..%d milliseconds' % (MIN_DURATION_MS, MAX_DURATION_MS))
            mode = request.get('ease', 'linear')
            if mode not in ('linear', 'smooth'):
                raise ValueError('Ease must be linear or smooth')
            target = request.get('positions')
            pose_angles(target)
            # A queued pose starts where the current transition ends.
            start = self.transition['target'] if self.transition is not None else self.current
            # Preflight the path at the same fixed interpolation points used
            # during playback. At most one transition and one queued pose are in RAM.
            count = steps(duration)
            for step in range(1, count + 1):
                pose_angles(blend(start, target, step / count))
            motion = {'start': [p[:] for p in start],
                      'target': [p[:] for p in target],
                      'duration': duration, 'steps': count, 'ease': mode,
                      'started': time.ticks_ms(), 'last_step': 0}
            if self.transition is None:
                self.transition = motion
            else:
                self.queued = motion
            self.mid_step = False  # Moving to a pose leaves the interrupted step.
        elif op == 'pause':
            self.stop_motion()  # Hold the last commanded pose.
        elif op == 'off':
            self.off()
        elif op == 'release_leg':
            # Free one trapped leg. Its position is then unknown, so movement
            # needs Enable at home again; the other legs keep holding.
            leg = self.leg_index(request.get('leg'))
            self.stop_motion()
            for pin in PINS[leg]:
                robot.servo_cluster.disable(pin)
            self.released.add(leg)
            self.current = None
        elif op == 'jog_joint':
            # Move one servo to an absolute angle, to find its real travel.
            # Leg positions no longer match the IK, so movement needs Enable at home again.
            if not self.active or self.angles is None:
                raise ValueError('Enable at home before testing a joint')
            if self.transition is not None or self.walk is not None:
                raise ValueError('Wait for this pose to finish, or pause it')
            leg = self.leg_index(request.get('leg'))
            joint = request.get('joint')
            angle = request.get('angle')
            if leg in self.released:
                raise ValueError('%s is released; Enable at home first' % NAMES[leg])
            if type(joint) is not int or not 0 <= joint <= 2:
                raise ValueError('Joint must be 0 (hip), 1 (upper leg) or 2 (lower leg)')
            low, high = robot.HARD_JOINT_LIMITS[joint]
            if not number(angle) or not low <= angle <= high:
                raise ValueError('Test angle must be within %g..%g' % (low, high))
            if abs(angle - self.angles[leg][joint]) > 10:
                raise ValueError('Test angles move at most 10 deg per step')
            try:
                self.write_servo(PINS[leg][joint], angle)
            except Exception:
                self.off()
                raise
            self.angles[leg][joint] = angle
            self.current = None
        elif op == 'cal_select':
            # Selecting a leg releases every output, as in the calibration app.
            leg = self.leg_index(request.get('leg'))
            self.off()
            self.cal_leg = leg
        elif op == 'cal_move':
            # Hold only the selected leg at the reference pose; the rest stay off.
            leg = self.cal_selected()
            angles = leg_angles(leg, HOME[leg])
            self.stop_motion()
            try:
                for other, pins in enumerate(PINS):
                    if other != leg:
                        for pin in pins:
                            robot.servo_cluster.disable(pin)
                for pin, value in zip(PINS[leg], angles):
                    self.write_servo(pin, value)
            except Exception:
                self.off()
                raise
            self.active = True
            self.cal_holding = True
            self.released = set(i for i in range(len(NAMES)) if i != leg)
            self.current = None
            self.angles = None
        elif op == 'cal_adjust':
            leg = self.cal_selected()
            if self.active and not self.cal_holding:
                raise ValueError('Motors are enabled for movement; disable them or select the leg again before calibrating')
            joint, delta = request.get('joint'), request.get('delta')
            if type(joint) is not int or not 0 <= joint <= 2 or not number(delta) \
                    or abs(delta) > MAX_ADJUSTMENT:
                raise ValueError('Use joint 0..2 and a change of -%d..%d degrees'
                                 % (MAX_ADJUSTMENT, MAX_ADJUSTMENT))
            updated = OFFSETS[leg][:]
            updated[joint] += delta
            angles = leg_angles(leg, HOME[leg], updated)
            # Write just the joint being calibrated; offsets change only if it succeeds.
            if self.cal_holding:
                try:
                    self.write_servo(PINS[leg][joint], angles[joint])
                except Exception:
                    self.off()
                    raise
            OFFSETS[leg][joint] = updated[joint]
        elif op == 'cal_save':
            store = robot.calibration_store
            if store is None:
                raise ValueError('Copy calibration_store.py to the board to save calibration')
            store.save({key: list(OFFSETS[i]) for i, key in enumerate(OFFSET_KEYS)})
        elif op == 'walk':
            self.start_or_steer(request)
        elif op == 'walk_stop':
            # Slow down over one cycle and finish in the starting stance. A
            # walk with a set number of cycles can also be stopped early.
            walk = self.walk
            if walk is not None:
                if walk['stop_at'] is None or walk['cycle'] < walk['stop_at']:
                    walk['stop_at'] = walk['cycle']
        elif op == 'set_joint_limits':
            store = robot.calibration_store
            if store is None or not hasattr(store, 'save_limits'):
                raise ValueError('Copy the current calibration_store.py to the board to save limits')
            if self.transition is not None:
                raise ValueError('Wait for this pose to finish, or pause it')
            if self.walk is not None:
                raise ValueError('Stop walking first')
            limits = store.validate_limits(request.get('joint_limits'), robot.HARD_JOINT_LIMITS)
            if self.active and self.angles is not None:
                for leg, values in enumerate(self.angles):
                    for joint, value in enumerate(values):
                        low, high = limits[joint]
                        if leg not in self.released and not low <= value <= high:
                            raise ValueError('%s %s is at %.1f deg, outside the new limits; move it first'
                                             % (NAMES[leg], JOINTS[joint], value))
            robot.JOINT_LIMITS = store.save_limits(limits, robot.HARD_JOINT_LIMITS)
        else:
            raise ValueError('Unknown training command')
        self.last_contact = time.ticks_ms()
        return self.state(op in ('hello', 'set_joint_limits'))

    @staticmethod
    def walk_settings(request, defaults):
        """Stride (mm), step height (mm), stride angle (deg) and cycle time (ms).

        While walking, any setting left out keeps its current value.
        """
        stride = request.get('stride', defaults.get('stride'))
        lift = request.get('lift', defaults.get('lift'))
        hip_swing = request.get('hip_swing', defaults.get('hip_swing', 0))
        period = request.get('period_ms', defaults.get('period'))
        if not number(stride) or not 5 <= stride <= 150:
            raise ValueError('Stride must be 5..150 mm')
        if not number(lift) or not 0 <= lift <= 60:
            raise ValueError('Step height must be 0..60 mm')
        if not number(hip_swing) or not 0 <= hip_swing <= 90:
            raise ValueError('Stride angle must be 0..90 degrees')
        if type(period) is not int or not 400 <= period <= 20000:
            raise ValueError('Cycle time must be 400..20000 milliseconds')
        return stride, lift, hip_swing, period

    @staticmethod
    def walk_direction(direction):
        if not isinstance(direction, list) or len(direction) != 3 or \
                not all(number(v) and -1 <= v <= 1 for v in direction) or \
                not any(direction):
            raise ValueError('Direction needs forward, sideways and turn values in -1..1')
        return [float(v) for v in direction]

    def start_or_steer(self, request):
        walk = self.walk
        if walk is not None:
            # While walking: steer and/or change stride, step height, stride
            # angle and speed. Each new setting is checked, then reached
            # gradually so planted feet never jump.
            if self.walk_stopping():
                raise ValueError('The walk is stopping')
            direction = self.walk_direction(request.get('direction', walk['target']))
            defaults = dict(walk['goal'])
            defaults['period'] = walk['period']
            stride, lift, hip_swing, period = self.walk_settings(request, defaults)
            self.check_walk(walk['base'], walk['gait'], direction, stride, lift, hip_swing)
            walk['target'] = direction
            walk['goal'] = {'stride': stride, 'lift': lift, 'hip_swing': hip_swing}
            walk['period'] = period  # The cycle position accumulates, so speed changes are smooth.
            return
        gait = request.get('gait')
        if gait not in WALK_GAITS:
            raise ValueError('Gait must be tripod, ripple or wave')
        direction = self.walk_direction(request.get('direction'))
        stride, lift, hip_swing, period = self.walk_settings(request, {})
        cycles = request.get('cycles', 0)
        if type(cycles) is not int or not 0 <= cycles <= 100:
            raise ValueError('Cycles must be 0 (until stopped) to 100')
        if not self.active or self.current is None:
            raise ValueError('Enable at home before walking')
        if self.mid_step:
            raise ValueError('The last walk stopped mid-step; press Stand (or send a pose) before walking again')
        if self.transition is not None:
            raise ValueError('Wait for this pose to finish, or pause it')
        base = [p[:] for p in self.current]
        self.check_walk(base, gait, direction, stride, lift, hip_swing)
        self.walk = {'base': base, 'gait': gait, 'stride': stride, 'lift': lift,
                     'hip_swing': hip_swing, 'period': period,
                     'goal': {'stride': stride, 'lift': lift, 'hip_swing': hip_swing},
                     'direction': direction, 'target': direction[:],
                     'cycle': 0.0, 'last_time': time.ticks_ms(), 'last_write': None,
                     # Finite walks: speed up, walk the cycles, then slow down.
                     'stop_at': WALK_RAMP + cycles if cycles else None}

    @staticmethod
    def check_walk(base, gait, direction, stride, lift, hip_swing):
        # Check the foot paths at half and full speed before moving.
        for amplitude in (0.5, 1.0):
            for n in range(48):
                pose_angles(walk_pose(base, gait, direction, stride, lift, hip_swing, amplitude, n / 48))

    def walk_stopping(self):
        stop_at = self.walk['stop_at']
        return stop_at is not None and self.walk['cycle'] >= stop_at

    def walk_state(self):
        walk = self.walk
        if walk is None:
            return None
        return {'gait': walk['gait'], 'direction': walk['direction'], 'target': walk['target'],
                'stride': walk['stride'], 'lift': walk['lift'], 'hip_swing': walk['hip_swing'],
                'period': walk['period'], 'cycle': walk['cycle'], 'stopping': self.walk_stopping()}

    def walk_tick(self, now):
        walk = self.walk
        if walk['last_write'] is not None and time.ticks_diff(now, walk['last_write']) < WALK_WRITE_MS:
            return None
        # Advance by elapsed time at the current speed, so changing the speed
        # never jumps the legs to another point of the cycle.
        advance = time.ticks_diff(now, walk['last_time']) / walk['period']
        walk['last_time'] = now
        walk['cycle'] += advance
        cycle = walk['cycle']
        # Steer and change settings gradually so planted feet never jump.
        limit = advance * WALK_SLEW
        walk['direction'] = [approach(d, t, limit) for d, t in zip(walk['direction'], walk['target'])]
        for key, rate in WALK_SETTING_SLEW:
            walk[key] = approach(walk[key], walk['goal'][key], advance * rate)
        walk['last_write'] = now
        done = walk['stop_at'] is not None and cycle >= walk['stop_at'] + WALK_RAMP
        amplitude = 0.0 if done else walk_amplitude(cycle, walk['stop_at'])
        pose = walk_pose(walk['base'], walk['gait'], walk['direction'], walk['stride'],
                         walk['lift'], walk['hip_swing'], amplitude, cycle % 1.0)
        self.write_pose(pose, pose_angles(pose))
        if done:
            self.walk = None
            return 'walked'
        return None

    def cal_selected(self):
        if self.cal_leg is None:
            raise ValueError('Select a leg to calibrate first')
        if self.transition is not None:
            raise ValueError('Wait for this pose to finish, or pause it')
        return self.cal_leg

    def cal_targets(self):
        if self.cal_leg is None:
            return None
        try:
            return leg_angles(self.cal_leg, HOME[self.cal_leg])
        except ValueError:
            return None

    @staticmethod
    def leg_index(leg):
        if type(leg) is not int or not 0 <= leg < len(NAMES):
            raise ValueError('Leg must be 0..5')
        return leg

    def tick(self):
        now = time.ticks_ms()
        if self.active and time.ticks_diff(now, self.last_contact) > TIMEOUT_MS:
            self.off()
            return 'timeout'
        if self.walk is not None:
            return self.walk_tick(now)
        transition = self.transition
        if transition is None:
            return None
        elapsed = max(0, time.ticks_diff(now, transition['started']))
        step = min(transition['steps'], elapsed * transition['steps'] // transition['duration'])
        if step <= transition['last_step']:
            return None
        pose = blend(transition['start'], transition['target'],
                     ease(step / transition['steps'], transition['ease']))
        self.write_pose(pose, pose_angles(pose))
        transition['last_step'] = step
        if step == transition['steps']:
            # Start the queued pose on schedule (from this one's end time, not
            # from now), so a sequence keeps its timing without drifting.
            queued, self.queued = self.queued, None
            if queued is not None:
                queued['started'] = time.ticks_add(transition['started'], transition['duration'])
            self.transition = queued
            return 'arrived'
        return None


def send(data):
    print(json.dumps(data))


def reply(controller, text):
    """Handle one request line and return the reply to send."""
    request = None
    try:
        request = json.loads(text)
        state = controller.handle(request)
        return {'id': request.get('id'), 'ok': True, 'state': state}
    # Driver failures (OSError, RuntimeError) are reported like bad input,
    # so the bridge keeps answering after a servo error.
    except Exception as error:
        return {'id': request.get('id') if isinstance(request, dict) else None,
                'ok': False, 'error': str(error)}


def run():
    controller = Controller()
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    pending, discard = '', False
    send({'event': 'ready', 'state': controller.state(True)})
    try:
        while True:
            try:
                event = controller.tick()
            except Exception as error:
                # Stop the transition and report it; write failures have already
                # disabled the outputs. The last valid pose stays held otherwise.
                controller.stop_motion()
                send({'event': 'error', 'error': str(error), 'state': controller.state()})
                event = None
            if event:
                send({'event': event, 'state': controller.state()})
            if not poller.poll(5):
                continue
            char = sys.stdin.read(1)
            if not char:
                continue
            if char in '\r\n':
                if discard:
                    pending, discard = '', False
                    send({'ok': False, 'id': None, 'error': 'Command too long'})
                    continue
                if not pending:
                    continue
                send(reply(controller, pending))
                pending = ''
            elif not discard:
                pending += char
                if len(pending) > 2048:
                    pending, discard = '', True
    finally:
        controller.off()


if __name__ == '__main__':
    run()
