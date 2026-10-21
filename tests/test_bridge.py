"""Motion bridge regression tests; no serial device or motors are accessed."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

BOARD = Path(__file__).resolve().parents[1] / 'Board Files'


def module(name, file):
    spec = importlib.util.spec_from_file_location(name, BOARD / file)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class Clock:
    def __init__(self):
        self.now = 0
    def ticks_ms(self):
        return self.now
    @staticmethod
    def ticks_diff(a, b):
        return a - b
    @staticmethod
    def ticks_add(a, b):
        return a + b


class Cluster:
    def __init__(self, **kwargs):
        self.values = {}
        self.disabled = set()
        self.writes = 0
    def value(self, pin, value=None):
        if value is None:
            return self.values[pin]
        assert math.isfinite(value) and -180 <= value <= 180
        self.values[pin] = value
        self.disabled.discard(pin)
        self.writes += 1
    def disable(self, pin):
        self.disabled.add(pin)
    def calibration(self, pin, cal=None):
        return Calibration() if cal is None else None


class Calibration:
    def limit_to_calibration(self, lower, upper):
        pass


class BridgeTests(unittest.TestCase):
    def setUp(self):
        fake_servo = types.SimpleNamespace(Servo=object, ServoCluster=Cluster,
                                          servo2040=types.SimpleNamespace(SERVO_1=0, SERVO_18=17))
        store = types.SimpleNamespace(load=lambda: None)
        with patch.dict(sys.modules, servo=fake_servo, calibration_store=store):
            self.robot = module('test_robot_functions', 'FUNCTIONS.py')
        with patch.dict(sys.modules, FUNCTIONS=self.robot):
            self.bridge = module('test_motion_bridge', 'motion_bridge.py')
        self.clock = Clock()
        self.bridge.time = self.clock
        self.controller = self.bridge.Controller()
        self.servos = self.robot.servo_cluster

    def test_boot_and_status_never_enable_motors(self):
        state = self.controller.handle({'op': 'hello'})
        self.assertFalse(state['active'])
        self.assertIsNone(state['positions'])
        self.assertEqual(self.servos.writes, 0)
        self.assertEqual(self.servos.disabled, set(range(18)))
        self.assertEqual(len(state['offsets']), 6)

    def test_pose_requires_explicit_arm(self):
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 700})
        self.assertEqual(self.servos.writes, 0)

    def test_nonblocking_transition_and_pause(self):
        self.controller.handle({'op': 'arm'})
        target = [p[:] for p in self.bridge.HOME]
        target[0][0] = 20
        writes = self.servos.writes
        state = self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 1000})
        self.assertTrue(state['moving'])
        self.assertEqual(self.servos.writes, writes)
        self.clock.now = 500
        self.controller.tick()
        self.assertAlmostEqual(self.controller.current[0][0], 10)
        self.controller.handle({'op': 'pause'})
        paused = [p[:] for p in self.controller.current]
        self.clock.now = 1500
        self.controller.tick()
        self.assertEqual(self.controller.current, paused)
        self.assertTrue(self.controller.active)

    def test_timeout_releases_all_outputs_mid_transition(self):
        self.controller.handle({'op': 'arm'})
        self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 4000})
        self.clock.now = 3001
        self.assertEqual(self.controller.tick(), 'timeout')
        self.assertFalse(self.controller.active)
        self.assertIsNone(self.controller.transition)
        self.assertEqual(self.servos.disabled, set(range(18)))

    def test_heartbeat_allows_long_transition_to_finish(self):
        self.controller.handle({'op': 'arm'})
        target = [p[:] for p in self.bridge.HOME]
        target[5][2] = 60
        self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 4000})
        for now in range(500, 4001, 500):
            self.clock.now = now
            self.controller.handle({'op': 'status'})
            self.controller.tick()
        self.assertIsNone(self.controller.transition)
        self.assertEqual(self.controller.current, target)

    def test_invalid_last_leg_is_rejected_before_any_servo_write(self):
        self.controller.handle({'op': 'arm'})
        target = [p[:] for p in self.bridge.HOME]
        target[5][0] = 1000
        writes = self.servos.writes
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 1000})
        self.assertEqual(self.servos.writes, writes)
        self.assertIsNone(self.controller.transition)
        for duration in (True, 0, 99, 10001, float('nan')):
            with self.assertRaises(ValueError):
                self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': duration})

    def test_driver_failure_disables_all_motors(self):
        with patch.object(self.servos, 'value', side_effect=OSError('driver failed')):
            with self.assertRaises(OSError):
                self.controller.handle({'op': 'arm'})
        self.assertFalse(self.controller.active)
        self.assertEqual(self.servos.disabled, set(range(18)))

    def test_all_generated_browser_templates_match_firmware(self):
        fixtures = json.loads((Path(__file__).parent / 'templates.json').read_text())
        self.controller.handle({'op': 'arm'})
        for program in fixtures:
            for frame in program['frames']:
                angles = self.bridge.pose_angles(frame['positions'])
                for actual, expected in zip(angles, frame['angles']):
                    for a, b in zip(actual, expected):
                        self.assertAlmostEqual(a, b, places=6)
                self.controller.handle({'op': 'pose', 'positions': frame['positions'], 'duration_ms': frame['duration_ms']})
                self.clock.now += frame['duration_ms']
                self.controller.handle({'op': 'status'})
                self.controller.tick()
                for actual, expected in zip(self.controller.current, frame['positions']):
                    for a, b in zip(actual, expected):
                        self.assertAlmostEqual(a, b, places=6)


    def test_hello_reports_workspace_and_joint_limits(self):
        state = self.controller.handle({'op': 'hello'})
        self.assertEqual(state['limits'], self.bridge.LIMITS)
        self.assertEqual(state['joint_limits'], self.robot.JOINT_LIMITS)

    def test_negative_coordinates_and_hip_axis(self):
        pose = [p[:] for p in self.bridge.HOME]
        pose[1] = [-60, 120, -40]
        self.assertEqual(len(self.bridge.pose_angles(pose)), 6)
        pose[1] = [0, 0, 100]
        with self.assertRaisesRegex(ValueError, 'L2: Foot is too close to the hip axis'):
            self.bridge.pose_angles(pose)

    def test_joint_error_names_leg_joint_and_angle(self):
        pose = [p[:] for p in self.bridge.HOME]
        pose[0] = [-100, -5, 60]
        with self.assertRaisesRegex(ValueError, r'L1 hip -?\d+\.\d deg is outside its joint angle limit'):
            self.bridge.pose_angles(pose)

    def test_rejected_servo_value_disables_outputs(self):
        original = self.servos.value
        def ignore_writes(pin, value=None):
            if value is None:
                return 0.0  # Driver kept its previous value.
            original(pin, value)
        with patch.object(self.servos, 'value', side_effect=ignore_writes):
            with self.assertRaisesRegex(ValueError, 'did not accept'):
                self.controller.handle({'op': 'arm'})
        self.assertFalse(self.controller.active)
        self.assertEqual(self.servos.disabled, set(range(18)))


    def use_limit_store(self):
        real = module('test_calibration_store', 'calibration_store.py')
        saved = []
        def save(data, hard):
            saved.append(data)
            return real.validate_limits(data, hard)
        self.robot.calibration_store = types.SimpleNamespace(
            validate_limits=real.validate_limits, save_limits=save)
        return saved

    def test_smooth_ease_follows_same_path_with_slow_ends(self):
        self.controller.handle({'op': 'arm'})
        target = [p[:] for p in self.bridge.HOME]
        target[0][0] = 40
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 1000, 'ease': 'bouncy'})
        self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 1000, 'ease': 'smooth'})
        self.clock.now = 250
        self.controller.tick()
        self.assertAlmostEqual(self.controller.current[0][0], 40 * (0.5 - 0.5 * math.cos(math.pi * 12 / 50)))  # step 12 of 50
        self.clock.now = 500
        self.controller.tick()
        self.assertAlmostEqual(self.controller.current[0][0], 20)
        self.clock.now = 1000
        self.assertEqual(self.controller.tick(), 'arrived')
        self.assertEqual(self.controller.current, target)

    def test_release_one_leg_keeps_others_and_requires_rehome(self):
        self.controller.handle({'op': 'arm'})
        state = self.controller.handle({'op': 'release_leg', 'leg': 1})
        self.assertEqual(self.servos.disabled, {3, 4, 5})
        self.assertTrue(state['active'])
        self.assertEqual(state['released'], [1])
        self.assertIsNone(state['positions'])
        with self.assertRaisesRegex(ValueError, 'Enable at home'):
            self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 700})
        with self.assertRaisesRegex(ValueError, 'released'):
            self.controller.handle({'op': 'jog_joint', 'leg': 1, 'joint': 0, 'angle': 0})
        for leg in (-1, 6, 1.0, None):
            with self.assertRaises(ValueError):
                self.controller.handle({'op': 'release_leg', 'leg': leg})
        state = self.controller.handle({'op': 'arm'})
        self.assertEqual(state['released'], [])
        self.assertEqual(self.servos.disabled, set())

    def test_jog_joint_moves_one_servo_in_small_steps(self):
        with self.assertRaisesRegex(ValueError, 'Enable at home'):
            self.controller.handle({'op': 'jog_joint', 'leg': 0, 'joint': 1, 'angle': 0})
        self.controller.handle({'op': 'arm'})
        start = self.controller.angles[0][1]
        writes = self.servos.writes
        state = self.controller.handle({'op': 'jog_joint', 'leg': 0, 'joint': 1, 'angle': start + 5})
        self.assertEqual(self.servos.writes, writes + 1)
        self.assertAlmostEqual(self.servos.values[1], start + 5)
        self.assertAlmostEqual(state['angles'][0][1], start + 5)
        self.assertIsNone(state['positions'])
        for bad in ({'joint': 1, 'angle': start + 25}, {'joint': 3, 'angle': 0},
                    {'joint': 1, 'angle': float('nan')}, {'joint': 1, 'angle': 200}):
            with self.assertRaises(ValueError):
                self.controller.handle(dict(op='jog_joint', leg=0, **bad))

    def test_joint_limits_are_saved_and_checked(self):
        saved = self.use_limit_store()
        state = self.controller.handle({'op': 'set_joint_limits', 'joint_limits': [[-120, 120], [-100, 110], [-90, 90]]})
        self.assertEqual(state['joint_limits'], ((-120, 120), (-100, 110), (-90, 90)))
        self.assertEqual(self.robot.JOINT_LIMITS, ((-120, 120), (-100, 110), (-90, 90)))
        self.assertEqual(len(saved), 1)
        for bad in ([[10, -10], [-90, 90], [-90, 90]], [[-200, 90], [-90, 90], [-90, 90]], [[-90, 90]], 'x'):
            with self.assertRaises(ValueError):
                self.controller.handle({'op': 'set_joint_limits', 'joint_limits': bad})
        self.controller.handle({'op': 'arm'})
        with self.assertRaisesRegex(ValueError, 'outside the new limits'):
            self.controller.handle({'op': 'set_joint_limits', 'joint_limits': [[-5, 5], [-90, 90], [-90, 90]]})
        self.assertEqual(len(saved), 1)

    def test_old_calibration_store_cannot_save_limits(self):
        self.robot.calibration_store = types.SimpleNamespace(load=lambda: None)
        with self.assertRaisesRegex(ValueError, 'calibration_store.py'):
            self.controller.handle({'op': 'set_joint_limits', 'joint_limits': [[-90, 90]] * 3})

    def test_saved_limits_load_at_startup(self):
        fake_servo = types.SimpleNamespace(Servo=object, ServoCluster=Cluster,
                                          servo2040=types.SimpleNamespace(SERVO_1=0, SERVO_18=17))
        store = types.SimpleNamespace(load=lambda: None, load_limits=lambda hard: ((-100, 100), (-90, 90), (-80, 80)))
        with patch.dict(sys.modules, servo=fake_servo, calibration_store=store):
            robot = module('test_robot_saved_limits', 'FUNCTIONS.py')
        self.assertEqual(robot.JOINT_LIMITS, ((-100, 100), (-90, 90), (-80, 80)))


    def test_calibration_preview_then_hold_one_leg(self):
        with self.assertRaisesRegex(ValueError, 'Select a leg'):
            self.controller.handle({'op': 'cal_move'})
        self.controller.handle({'op': 'arm'})
        state = self.controller.handle({'op': 'cal_select', 'leg': 4})
        self.assertFalse(state['active'])
        self.assertEqual(self.servos.disabled, set(range(18)))
        self.assertEqual(state['calibration']['leg'], 4)
        before = self.bridge.OFFSETS[4][1]
        writes = self.servos.writes
        state = self.controller.handle({'op': 'cal_adjust', 'joint': 1, 'delta': 2.5})
        self.assertEqual(self.servos.writes, writes, 'preview must not move a servo')
        self.assertAlmostEqual(state['offsets'][4][1], before + 2.5)
        state = self.controller.handle({'op': 'cal_move'})
        self.assertTrue(state['active'])
        self.assertTrue(state['calibration']['holding'])
        self.assertEqual(self.servos.disabled, set(range(18)) - {12, 13, 14})
        expected = self.bridge.leg_angles(4, self.bridge.HOME[4])
        for pin, angle in zip((12, 13, 14), expected):
            self.assertAlmostEqual(self.servos.values[pin], angle)
        self.assertEqual(state['released'], [0, 1, 2, 3, 5])
        writes = self.servos.writes
        self.controller.handle({'op': 'cal_adjust', 'joint': 0, 'delta': -1})
        self.assertEqual(self.servos.writes, writes + 1)
        self.assertAlmostEqual(self.servos.values[12], expected[0] - 1)
        with self.assertRaisesRegex(ValueError, 'Enable at home'):
            self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 700})

    def test_calibration_rejects_bad_adjustments_and_keeps_offsets(self):
        self.controller.handle({'op': 'cal_select', 'leg': 0})
        before = list(self.bridge.OFFSETS[0])
        for joint, delta in ((0, 16), (3, 1), (1, float('nan')), (True, 1)):
            with self.assertRaises(ValueError):
                self.controller.handle({'op': 'cal_adjust', 'joint': joint, 'delta': delta})
        self.robot.JOINT_LIMITS = ((-180, 180), (-180, 180), (-30, 30))
        with self.assertRaisesRegex(ValueError, 'joint angle limit'):
            self.controller.handle({'op': 'cal_adjust', 'joint': 2, 'delta': 15})
        self.assertEqual(list(self.bridge.OFFSETS[0]), before)

    def test_calibration_offsets_are_used_by_motion_and_saved(self):
        saved = []
        self.robot.calibration_store = types.SimpleNamespace(save=saved.append)
        self.controller.handle({'op': 'cal_select', 'leg': 2})
        self.controller.handle({'op': 'cal_adjust', 'joint': 1, 'delta': 3})
        self.controller.handle({'op': 'arm'})
        moved = self.controller.angles[2][1]
        self.controller.handle({'op': 'cal_select', 'leg': 2})
        self.controller.handle({'op': 'cal_adjust', 'joint': 1, 'delta': -3})
        self.controller.handle({'op': 'arm'})
        self.assertAlmostEqual(moved - self.controller.angles[2][1], 3)
        self.controller.handle({'op': 'cal_save'})
        self.assertEqual(sorted(saved[0]), ['L1o', 'L2o', 'L3o', 'R1o', 'R2o', 'R3o'])
        self.assertEqual(saved[0]['L3o'], list(self.bridge.OFFSETS[2]))
        self.robot.calibration_store = None
        with self.assertRaisesRegex(ValueError, 'calibration_store.py'):
            self.controller.handle({'op': 'cal_save'})

    def test_calibration_hold_times_out(self):
        self.controller.handle({'op': 'cal_select', 'leg': 1})
        self.controller.handle({'op': 'cal_move'})
        self.clock.now = 3001
        self.assertEqual(self.controller.tick(), 'timeout')
        state = self.controller.state()
        self.assertFalse(state['calibration']['holding'])
        self.assertEqual(state['calibration']['leg'], 1)
        self.assertEqual(self.servos.disabled, set(range(18)))


    def test_fast_and_slow_transitions(self):
        self.controller.handle({'op': 'arm'})
        target = [p[:] for p in self.bridge.HOME]
        target[0][0] = 40
        self.controller.handle({'op': 'pose', 'positions': target, 'duration_ms': 100})
        self.assertEqual(self.controller.transition['steps'], 5)
        self.clock.now = 100
        self.assertEqual(self.controller.tick(), 'arrived')
        self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 10000})
        self.assertEqual(self.controller.transition['steps'], 200)
        self.clock.now += 50
        self.controller.handle({'op': 'status'})
        self.controller.tick()
        self.assertAlmostEqual(self.controller.current[0][0], 39.8)


    def test_driver_errors_are_replies_not_crashes(self):
        with patch.object(self.servos, 'value', side_effect=OSError('driver failed')):
            answer = self.bridge.reply(self.controller, '{"id": 7, "op": "arm"}')
        self.assertEqual(answer, {'id': 7, 'ok': False, 'error': 'driver failed'})
        self.assertEqual(self.bridge.reply(self.controller, '[1]')['ok'], False)
        self.assertEqual(self.bridge.reply(self.controller, 'not json')['id'], None)
        self.assertTrue(self.bridge.reply(self.controller, '{"id": 8, "op": "status"}')['ok'])

    def test_calibration_never_adjusts_a_leg_powered_for_movement(self):
        self.controller.handle({'op': 'cal_select', 'leg': 0})
        self.controller.handle({'op': 'arm'})
        before = list(self.bridge.OFFSETS[0])
        with self.assertRaisesRegex(ValueError, 'Motors are enabled'):
            self.controller.handle({'op': 'cal_adjust', 'joint': 0, 'delta': 5})
        self.assertEqual(list(self.bridge.OFFSETS[0]), before)
        # Holding for calibration turns the other five legs off, not just reports them off.
        state = self.controller.handle({'op': 'cal_move'})
        self.assertEqual(self.servos.disabled, set(range(3, 18)))
        self.assertEqual(state['released'], [1, 2, 3, 4, 5])
        self.controller.handle({'op': 'cal_adjust', 'joint': 0, 'delta': 5})
        self.assertEqual(self.bridge.OFFSETS[0][0], before[0] + 5)


    def test_queued_pose_starts_exactly_when_the_current_one_ends(self):
        self.controller.handle({'op': 'arm'})
        first = [p[:] for p in self.bridge.HOME]
        first[0][0] = 20
        second = [p[:] for p in self.bridge.HOME]
        second[0][0] = 40
        self.controller.handle({'op': 'pose', 'positions': first, 'duration_ms': 200})
        with self.assertRaisesRegex(ValueError, 'Wait for this pose'):
            self.controller.handle({'op': 'pose', 'positions': second, 'duration_ms': 200})
        state = self.controller.handle({'op': 'pose', 'positions': second, 'duration_ms': 200, 'queue': True})
        self.assertTrue(state['queued'])
        with self.assertRaisesRegex(ValueError, 'already queued'):
            self.controller.handle({'op': 'pose', 'positions': second, 'duration_ms': 200, 'queue': True})
        # The board ticks late (at 205 ms); the queued pose still starts at 200 ms.
        self.clock.now = 205
        self.assertEqual(self.controller.tick(), 'arrived')
        self.assertEqual(self.controller.current, first)
        self.assertEqual(self.controller.transition['started'], 200)
        self.assertFalse(self.controller.state()['queued'])
        self.clock.now = 300
        self.controller.handle({'op': 'status'})
        self.controller.tick()
        self.assertAlmostEqual(self.controller.current[0][0], 30)
        self.clock.now = 400
        self.controller.tick()
        self.assertEqual(self.controller.current, second)
        self.assertIsNone(self.controller.transition)

    def test_queued_path_is_checked_from_the_current_target_and_pause_clears_it(self):
        self.controller.handle({'op': 'arm'})
        first = [p[:] for p in self.bridge.HOME]
        first[0][0] = 20
        self.controller.handle({'op': 'pose', 'positions': first, 'duration_ms': 1000})
        bad = [p[:] for p in self.bridge.HOME]
        bad[0][0] = 1000
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'pose', 'positions': bad, 'duration_ms': 200, 'queue': True})
        self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 200, 'queue': True})
        state = self.controller.handle({'op': 'pause'})
        self.assertFalse(state['moving'])
        self.assertFalse(state['queued'])


    def test_walk_paths_match_the_web_app(self):
        fixture = json.loads((Path(__file__).parent / 'walk_fixture.json').read_text())
        for case in fixture['cases']:
            p = case['params']
            pose = self.bridge.walk_pose(case['base'], p['gait'], p['direction'], p['stride'],
                                         p['lift'], p['hipSwing'], case['amplitude'], case['t'])
            for actual, expected in zip(pose, case['pose']):
                for a, b in zip(actual, expected):
                    self.assertAlmostEqual(a, b, places=9)
        for cycle, free, stopping in fixture['amplitudes']:
            self.assertAlmostEqual(self.bridge.walk_amplitude(cycle, None), free, places=12)
            self.assertAlmostEqual(self.bridge.walk_amplitude(cycle, 1.5), stopping, places=12)

    def walk(self, **extra):
        request = {'op': 'walk', 'gait': 'tripod', 'direction': [1, 0, 0], 'stride': 40,
                   'lift': 20, 'period_ms': 1000}
        request.update(extra)
        return self.controller.handle(request)

    def run_until(self, end, step=5):
        events = []
        while self.clock.now < end:
            self.clock.now += step
            if self.clock.now % 500 == 0:
                self.controller.handle({'op': 'status'})
            event = self.controller.tick()
            if event:
                events.append((self.clock.now, event))
        return events

    def test_walk_moves_continuously_and_ends_in_the_starting_stance(self):
        with self.assertRaisesRegex(ValueError, 'Enable at home'):
            self.walk()
        self.controller.handle({'op': 'arm'})
        state = self.walk(cycles=2)
        self.assertTrue(state['moving'])
        self.assertEqual(state['walking']['gait'], 'tripod')
        positions = []
        writes = self.servos.writes
        while self.controller.walk is not None:
            self.clock.now += 5
            if self.clock.now % 500 == 0:
                self.controller.handle({'op': 'status'})
            self.controller.tick()
            if self.servos.writes != writes:
                writes = self.servos.writes
                positions.append([p[:] for p in self.controller.current])
            self.assertLess(self.clock.now, 10000)
        # Ramp up 1 cycle, 2 full cycles, slow down 1 cycle: 4 s at 20 ms per write.
        self.assertAlmostEqual(self.clock.now, 4000, delta=25)
        self.assertGreater(len(positions), 190)
        self.assertEqual(self.controller.current, self.bridge.HOME)
        # Continuous: no foot moves more than a few mm between 20 ms writes.
        jumps = [max(math.dist(a, b) for a, b in zip(p, q)) for p, q in zip(positions, positions[1:])]
        self.assertLess(max(jumps), 6)
        self.assertGreater(max(abs(p[0][0]) for p in positions), 15)

    def test_walk_stop_steer_and_guards(self):
        self.controller.handle({'op': 'arm'})
        self.walk()
        for blocked in ({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 700},
                        {'op': 'arm'}):
            with self.assertRaisesRegex(ValueError, 'Stop walking first'):
                self.controller.handle(blocked)
        self.run_until(1500)
        # Steering turns gradually toward the new direction.
        self.walk(direction=[0, 0, 1])
        self.run_until(1700)
        direction = self.controller.walk['direction']
        self.assertTrue(0 < direction[2] < 1 and 0 < direction[0] < 1)
        self.run_until(3000)
        self.assertEqual(self.controller.walk['direction'], [0.0, 0.0, 1.0])
        with self.assertRaises(ValueError):
            self.walk(direction=[0, 0, 0])
        self.controller.handle({'op': 'walk_stop'})
        with self.assertRaisesRegex(ValueError, 'stopping'):
            self.walk(direction=[1, 0, 0])
        events = self.run_until(4100)
        self.assertEqual(events[-1][1], 'walked')
        self.assertIsNone(self.controller.walk)
        self.assertEqual(self.controller.current, self.bridge.HOME)
        # Pause stops at once and holds; unreachable walks are refused before moving.
        self.walk(cycles=0)
        self.run_until(4500)
        state = self.controller.handle({'op': 'pause'})
        self.assertFalse(state['moving'])
        writes = self.servos.writes
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'walk', 'gait': 'tripod', 'direction': [1, 1, 1],
                                    'stride': 150, 'lift': 60, 'period_ms': 1000})
        self.assertEqual(self.servos.writes, writes)
        for bad in ({'gait': 'gallop'}, {'stride': 200}, {'period_ms': 100}, {'cycles': -1}):
            with self.assertRaises(ValueError):
                self.walk(**bad)

    def test_walk_watchdog_releases_outputs(self):
        self.controller.handle({'op': 'arm'})
        self.walk()
        self.clock.now = 3001
        self.assertEqual(self.controller.tick(), 'timeout')
        self.assertIsNone(self.controller.walk)
        self.assertEqual(self.servos.disabled, set(range(18)))


    def test_a_walk_with_set_cycles_can_be_steered_and_stopped_early(self):
        self.controller.handle({'op': 'arm'})
        state = self.walk(cycles=5)
        self.assertFalse(state['walking']['stopping'])
        self.run_until(1200)
        self.walk(direction=[0, 1, 0])
        self.controller.handle({'op': 'walk_stop'})
        self.assertTrue(self.controller.state()['walking']['stopping'])
        events = self.run_until(2500)
        self.assertEqual(events[-1][1], 'walked')
        self.assertAlmostEqual(events[-1][0], 2200, delta=25)  # one cycle after the stop, give or take a 20 ms update
        self.assertEqual(self.controller.current, self.bridge.HOME)


    def test_walk_settings_change_smoothly_while_walking(self):
        self.controller.handle({'op': 'arm'})
        self.walk(hip_swing=15)
        self.run_until(1500)
        # Faster, higher steps, wider stride angle: accepted, then blended in.
        state = self.controller.handle({'op': 'walk', 'period_ms': 500, 'lift': 35, 'hip_swing': 25})
        self.assertEqual(state['walking']['period'], 500)
        self.assertEqual(self.controller.walk['target'], [1.0, 0.0, 0.0], 'direction kept')
        positions, jumps = [], []
        writes = self.servos.writes
        while self.clock.now < 3500:
            self.clock.now += 5
            if self.clock.now % 500 == 0:
                self.controller.handle({'op': 'status'})
            self.controller.tick()
            if self.servos.writes != writes:
                writes = self.servos.writes
                if positions:
                    jumps.append(max(math.dist(a, b) for a, b in zip(positions[-1], self.controller.current)))
                positions.append([p[:] for p in self.controller.current])
        walk = self.controller.walk
        self.assertEqual((walk['lift'], walk['hip_swing']), (35, 25))
        # Even at the faster speed, no foot jumps between 20 ms updates.
        self.assertLess(max(jumps), 12)
        # An unreachable setting is refused and the walk keeps its settings.
        with self.assertRaises(ValueError):
            self.controller.handle({'op': 'walk', 'hip_swing': 90, 'lift': 60, 'direction': [1, 1, 1]})
        self.assertEqual(walk['goal'], {'stride': 40, 'lift': 35, 'hip_swing': 25})
        for bad in ({'lift': 61}, {'hip_swing': -1}, {'period_ms': 300}):
            with self.assertRaises(ValueError):
                self.controller.handle(dict(op='walk', **bad))


    def test_a_walk_paused_mid_step_cannot_restart_from_lifted_feet(self):
        self.controller.handle({'op': 'arm'})
        self.walk(hip_swing=15, lift=25)
        self.run_until(1620)
        state = self.controller.handle({'op': 'pause'})
        self.assertTrue(state['mid_step'])
        self.assertTrue(any(p[2] < 69 for p in state['positions']), 'some feet are in the air')
        with self.assertRaisesRegex(ValueError, 'mid-step'):
            self.walk()
        # Moving to a proper stance (any pose, e.g. Stand) allows walking again.
        self.controller.handle({'op': 'pose', 'positions': self.bridge.HOME, 'duration_ms': 500})
        self.run_until(2200)
        self.assertFalse(self.controller.state()['mid_step'])
        self.assertTrue(self.walk()['walking'])
        # A walk that finishes normally (■) does not block the next one.
        self.controller.handle({'op': 'walk_stop'})
        self.run_until(3400)
        self.assertFalse(self.controller.state()['mid_step'])
        self.assertTrue(self.walk()['walking'])



class MicroPythonCompatibilityTests(unittest.TestCase):
    """The board runs MicroPython, whose math module lacks some CPython functions."""
    MICROPYTHON_MATH = {'acos', 'asin', 'atan', 'atan2', 'ceil', 'copysign', 'cos', 'degrees', 'e',
                        'exp', 'fabs', 'floor', 'fmod', 'frexp', 'isfinite', 'isinf', 'isnan',
                        'ldexp', 'log', 'modf', 'pi', 'pow', 'radians', 'sin', 'sqrt', 'tan', 'trunc'}

    def test_board_files_use_only_micropython_math(self):
        import re
        for path in BOARD.glob('*.py'):
            used = set(re.findall(r'\bmath\.(\w+)', path.read_text()))
            self.assertEqual(used - self.MICROPYTHON_MATH, set(), '%s uses math functions MicroPython lacks' % path.name)

    def test_board_files_compile_with_micropython(self):
        # Uses MicroPython's own compiler when installed (pip install mpy-cross).
        import shutil, subprocess, tempfile
        compiler = shutil.which('mpy-cross')
        if compiler is None:
            self.skipTest('mpy-cross is not installed')
        with tempfile.TemporaryDirectory() as out:
            for path in BOARD.glob('*.py'):
                result = subprocess.run([compiler, '-o', str(Path(out) / (path.stem + '.mpy')), str(path)],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, '%s: %s' % (path.name, result.stderr))


if __name__ == '__main__':
    unittest.main()
