"""Run in Thonny after uploading the four training files to the board root."""
import os
import machine
for name in ('main.py', 'FUNCTIONS.py', 'calibration_store.py', 'motion_bridge.py'):
    os.stat(name)
import motion_bridge
controller = motion_bridge.Controller()
print('Training bridge ready:', controller.state(True)['protocol'])
print('Close Thonny after restart, then connect from the Motion Trainer.')
machine.reset()
