"""Persist calibration separately from source code, with an atomic replacement."""
import json
import math
import os

PATH = 'calibration_offsets.json'
KEYS = ('L1o', 'L2o', 'L3o', 'R1o', 'R2o', 'R3o')


def validate(data):
    if not isinstance(data, dict) or set(data) != set(KEYS):
        raise ValueError('Calibration must contain all six leg offsets')
    result = {}
    for key in KEYS:
        values = data[key]
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError('Each leg needs three offsets')
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values):
            raise ValueError('Offsets must be finite numbers')
        result[key] = list(values)
    return result


def load():
    try:
        with open(PATH) as source:
            return validate(json.load(source))
    except OSError as error:
        if error.args and error.args[0] == 2:
            return None
        raise


def save(data):
    data = validate(data)
    with open(PATH + '.tmp', 'w') as target:
        json.dump(data, target)
    os.rename(PATH + '.tmp', PATH)


# Joint limits are saved from the web app's limit editor, in their own file so
# calibration and limits can be restored independently.
LIMITS_PATH = 'joint_limits.json'


def validate_limits(data, hard):
    if not isinstance(data, (list, tuple)) or len(data) != 3:
        raise ValueError('Joint limits need hip, upper leg and lower leg pairs')
    result = []
    for pair, (hard_low, hard_high) in zip(data, hard):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError('Each joint limit needs a minimum and maximum')
        low, high = pair
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in pair):
            raise ValueError('Joint limits must be finite numbers')
        if not hard_low <= low < high <= hard_high:
            raise ValueError('Joint limits must be ordered and within %g..%g' % (hard_low, hard_high))
        result.append((low, high))
    return tuple(result)


def load_limits(hard):
    try:
        with open(LIMITS_PATH) as source:
            return validate_limits(json.load(source), hard)
    except OSError as error:
        if error.args and error.args[0] == 2:
            return None
        raise


def save_limits(data, hard):
    data = validate_limits(data, hard)
    with open(LIMITS_PATH + '.tmp', 'w') as target:
        json.dump([list(pair) for pair in data], target)
    os.rename(LIMITS_PATH + '.tmp', LIMITS_PATH)
    return data
