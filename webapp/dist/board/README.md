# Hexapod Motion Studio

A separate, local movement-authoring app for the Servo 2040 hexapod.
Run `Start Motion Studio.cmd` on Windows or `Start Motion Studio.command` on
macOS. Python 3.8+ is required on the computer. The launcher serves `webapp/dist`
on loopback port 5180 and opens Edge or Chrome when installed.

## What you can teach

Six draft templates: forward, backward, sideways left, sideways right, turn
left and turn right. “Clawer” is provisionally treated as turning. A template
contains 20 target poses: home, lift/place/lower each leg individually, and
return the supported stance to home. These are starting points for tuning,
not a guarantee of real-world travel or balance.

Choose a timeline pose to inspect all six feet's XYZ and three corrected servo
angles per leg. Edit positions, transition time (100–10,000 ms), pose name and
movement name. Duplicate/remove poses, import/export JSON, and export CSV with
all per-pose targets. Programs contain up to 128 poses.

The bundled React preview smoothly animates connected hip, knee and foot
segments between target poses without commanding hardware. Click Preview to
start, pause or resume; scrub the preview timeline, choose 0.25×–2× speed,
switch between angled and top views, or show foot paths. Editing a target
also animates the model to the new pose. Preview speed only affects the
animation. It is not a physics simulation. Enable at home commands the physical robot;
subsequent transitions interpolate XYZ at up to 50 updates/second. The board
preflights every interpolation sample before starting a transition. The app
also rejects neighbouring feet closer than 30 mm and feet inside the body
outline above 20 mm Z, using the preview's approximate hip positions (not a
measured frame); legs, knees and the body underside are not checked. None of
the checks detect poor balance or incorrect servo calibration. The first activation has no known physical starting angle.

## Coordinate convention and direction assumptions

Coordinates are leg-local millimetres, using the original IK lengths:
47, 88.46, 148.8 mm. Home is `(0, 175, 70)` in the provided configuration. Lower Z
lifts a foot in this model. Positive Z points down from the hip reference.
The training workspace is X, Y and Z -284..284 mm (full leg reach); a foot
within 1 mm of the hip axis (X and Y near 0) is rejected. Servo output limits
are -180..180 degrees per joint after applying your calibration. Change them
only in `JOINT_LIMITS` in `FUNCTIONS.py` (hip, upper leg, lower leg) and the
workspace in `LIMITS` in `motion_bridge.py`: the app adopts the board's values
on connect and warns if the board files are too old to report them. The
bridge reads back every servo value and disables all outputs if the driver
did not apply it.

The draft generator uses the source's leg angles `30, 0, -30, 150, 180, -150`
for L1/L2/L3/R1/R2/R3. For forward f and lateral-left l displacement:

    local_x = stride * (f * cos(theta) - l * sin(theta))
    local_y = 175 + stride * (f * sin(theta) + l * cos(theta))

Turning uses equal signed local-X displacements, following the source's turning
convention. The code does not establish the assembled robot's physical front,
handedness, or exact mounting transforms. Verify each direction experimentally
at low stride before treating its label as actual robot displacement.

The pose diagram projects commanded targets; it does not estimate the body’s
location in the room. Capture robot pose records the last command reported by
the board, not encoder or sensor measurements. This is teach-and-repeat pose
training, not autonomous learning or feedback-controlled navigation.

## Board installation

Back up the prior `main.py`. Save the four files listed in START HERE.txt to the
board root, retaining `calibration_offsets.json` if present. Run
`check_and_restart.py` once from Thonny. Close Thonny and reconnect USB normally.
The new `main.py` boots into the training bridge with all outputs off. No mode
flag is required. Use this app only with its matching training bridge.

Only the new main startup is required to replace calibration mode; to restore
it, copy the previous calibration main and supporting files back. Do not flash
new firmware or erase the filesystem for this task.

## USB protocol and stopping

Newline-delimited JSON over USB CDC, protocol `hexapod-motion-v1`:
`hello`, `status`, `ping`, `arm`, `pose`, `pause`, `off`. The browser sends
`pose` with all six XYZ arrays and an integer `duration_ms`, then polls status
until complete before sending another pose. Requests are acknowledged by ID.
Queued movement requests have cancellation guards; pause/off cancel the player
before their commands are sent. The firmware remains responsive to commands
during interpolation. The app plays a sequence only once.

A 3-second communication timeout releases outputs. Pause holds the most recent
commanded pose. Disable releases torque. A browser closing, sleeping or losing
USB can trigger the timeout. Keep the body supported throughout teaching.

## Persistence

The browser stores up to 20 named programs in localStorage. Saving an existing
name replaces that browser copy. JSON is the portable backup; CSV is for
inspection. Browser storage can be cleared and is specific to its origin.
Programs are streamed during playback and are not saved as autonomous board
programs. Saved calibration offsets are loaded from the existing board JSON;
importing a movement does not overwrite that calibration.

## Verification

Run board tests with `python3 -m unittest discover -s tests -p 'test_*.py' -v`
from this folder. Run model/transport/player tests with `npm test` in webapp.
No Python/Node dependency installation is required for these tests.
Physical balance, real-world direction and browser/Windows USB operation
remain unverified in this environment.

## Ready-to-open position files

Example Movements contains six JSON programs and matching CSV tables.
They use the source-code default offsets, not measurements from your robot.
Import a JSON in the app; once connected, current board calibration is used
for angle checks and fresh exports. CSV files can be opened in Excel.

## React animation development

React and ReactDOM are bundled locally in `webapp/dist/scene.bundle.js`.
Normal use requires no Node installation or internet download. This animation
update requires no changes to an already installed training bridge. Restart
the Mac launcher and refresh the page with Command–Shift–R after updating.

The React component and preview clock are in `webapp/src`. To rebuild after
editing them, run `npm ci` then `npm run build` inside `webapp` using Node.js.
Run `npm test` for animation, controller, model and transport tests. The
animated geometry uses the source leg lengths and schematic body mounts; it
does not measure the assembled robot.

## Stand and Sit

Choose Stand or Sit / crouch in the movement picker for an offline animated
transition that can be edited, saved or exported. Defaults are `(0,175,70)`
for Stand and `(0,175,30)` for Sit. The legacy folded sit at Z=-100 is now
within the allowed range if you set it.

Change Stand height, Sit height and Transition in Stand / Sit settings.
Heights are -100–150 mm (Z down from the hips, so a higher Stand pushes the
feet further down). Each leg has a Stand trim and a Sit trim (±40 mm); each
leg's Sit must stay lower than its Stand. Transition time is
100–10,000 ms. Settings persist in this browser, apply immediately to the
physical posture buttons, and apply to templates when you rebuild them.
Changing settings never sends a robot command. Current calibration limits
are checked; the full movement path is checked before physical playback.

Connect and Enable at home before using physical Stand/Sit. Enable at home
still uses the board's fixed home position. Stand/Sit interpolate from the
current commanded position and hold torque afterward. Disable motors
releases outputs; Pause holds. Physical clearance and balance still require
verification. No board update is needed.

## Target and board-reported poses

The Selected target pose editor controls the selected frame in your sequence.
The separate Board-reported pose table shows all six legs' last reported XYZ,
without changing your edits. The animation label identifies selected-target,
sequence-preview, or board-reported-command mode. On disconnect, the reported
table clears and a remaining command animation is marked disconnected.
Board-reported values are commands, not sensor measurements.

## Undo and draft recovery

Undo and Redo above the timeline retain up to 30 edits in this session,
including template replacement, import and loading a named movement. These
controls never move the robot and are locked during physical playback.
A new edit clears redo history. Undo history does not survive page reload.

Every completed edit saves the current draft automatically in this browser.
Reloading recovers the last draft with its saved calibration for offline
inspection; connecting loads the board's current calibration. Draft recovery
sends no commands. Named saves remain separate. The draft status reports
storage failures; keep JSON exports as portable backups. Posture settings
are saved separately and are not part of sequence Undo/Redo.

## Tuning on the robot (panel 06)

- **Jog a foot / touch-down / teach.** With the robot enabled, step one foot
  in X, Y or Z (1–10 mm, 0.5 s per step; arrow keys work when no field has
  focus). Lower a foot until it just touches the ground, then *Save foot
  height as Stand trim*. To teach a pose, jog the feet and *Add robot pose to
  sequence*. Hobby servos cannot report their position, so poses are taught
  by jogging rather than by moving the leg by hand.
- **Joint limits.** Step one servo (±1/±5°, at most 10° per command) toward its
  end stop, back off, and use the angle as that joint's min or max. *Save
  limits to board* stores them in `joint_limits.json` on the board (needs the
  current `calibration_store.py`); they apply to all six legs and every pose.
  After testing a servo, use Enable at home before moving the legs.
- **Release one leg.** Turns off one leg's three servos to free it while the
  others hold. Use Enable at home before moving again.

## Playback options

Repeat (1–99, or until stopped), gait speed (0.25–4×) and *Ease in/out* sit
under the robot buttons. Speed divides every transition time, kept within
100–10,000 ms. Easing keeps the same path but slows the start and end of each
transition, on the board and in the preview. A repeat also checks the move
from the last pose back to the first.

## Joining movements

Tick *Join new templates to the end of the current sequence* before choosing a
movement, or pick a saved movement and *Join to end*. A template's first,
assumed starting pose is dropped when joining. Check the transition into the
joined part before playing it. Up to 128 poses.

## Balance estimate

The preview draws the support area of the lowest feet and the body centre
(middle of the hip mounts), and labels the margin. It is an estimate: mass
distribution, battery position and dynamics are not modelled.

## Simulated robot

*Simulate robot* connects the app to a copy of the board bridge running in the
page (`dist/simulator.js`). It accepts the same commands with the same checks
and timing, so you can test playback, jogging and limits without USB. It does
not model servos, load, balance or the communication timeout.

## Leg calibration tab

The *Leg calibration* tab replaces the separate Web Leg Calibration app and
uses the same connection and board files (`motion_bridge.py` now includes the
calibration commands from `web_bridge.py`). Open it with the tab, or go to
`http://localhost:5180/#calibrate`.

1. Select a leg on the map. This disables every servo.
2. Adjust each joint with −/+ (1°) or its slider (up to 15° per change). With
   the outputs off this only changes the preview.
3. *Enable leg & apply* holds that leg alone at the home reference pose
   `(0, 175, 70)`; the other legs stay off. Further adjustments move only the
   joint being changed. The 3-second communication timeout still applies.
4. *Save to robot* writes `calibration_offsets.json`; *Download offsets* saves
   a copy. Servo voltage and current are shown when the board reports them.

Offsets change for the whole app straight away, so poses are checked against
the new calibration. After calibrating, use Enable at home before moving in
Motion studio. Joint angles are checked against the board's joint limits
rather than the old fixed ±90°.

## Speed and transition time

Each pose's transition can take 100–10,000 ms (shorter is faster). *All
transitions · Apply to every pose* under the sequence sets them all at once
and can be undone; Speed (0.25–4×) scales them at playback without editing
the sequence. Moves are interpolated every 20 ms, capped at 200 steps, so a
10 s transition updates every 50 ms; the board checks every one of those steps
before moving. Under about 300 ms the servos may not physically keep up, and
the app warns when playing such transitions.

## Battery voltage

The header shows the servo power voltage and current measured by the Servo
2040 on every board reply (Motion studio and Leg calibration). This is the
servo supply rail: it is the battery voltage only if the battery feeds the
servos directly, not through a regulator. Set *Low battery below … V* next to
the playback options to get a warning (off when empty). Boards whose firmware
lacks the sensor library show "not reported".

## Step size

Walking templates accept stride 5–150 mm, foot lift 5–60 mm and body height
30–150 mm. The crawl moves the body by one stride per cycle, so stride is the
distance travelled per 20 poses. Every pose and path is still checked: with
the default calibration the largest reachable strides are about 130–140 mm
forward/backward and 80–95 mm sideways; beyond that the app names the leg that
cannot reach. Increase stride gradually on the robot: long steps need more
grip and servo torque, and the body can tip while one foot is lifted.

## Gaits

Choose the gait under the preview before building a walking template:

| Gait | Legs moving | Feet down | Body travel per cycle | Notes |
|---|---|---|---|---|
| 1 leg · crawl | 1 | 5 | 1 stride | Slowest, stop-and-go |
| 3 legs · tripod | 3 | 3 | 2 strides | Lift, swing, lower in separate poses |
| Insect · tripod | 3 | 3 | 2 strides | Continuous, fastest |
| Insect · ripple | 2 | 4 | 1.5 strides | Continuous, waves back to front |
| Insect · wave | 1 | 5 | 1.2 strides | Continuous, most stable |

The insect gaits walk like a real insect: each foot swings forward on a raised
arc and then pushes back in a straight line on the ground, and the legs'
rhythms are staggered from back to front (L3 → L2 → L1 on each side). They
step in from home three legs at a time, walk the chosen number of **Cycles**
(1–8; 128 poses maximum), then step back to home. For smooth walking use a
short Transition (150–250 ms per pose) and leave *Ease in/out* off, since
easing slows every pose's start and end. The app now polls the board every
20 ms near the end of each pose so the next pose follows without a pause.

## Hip rotation

- **Hip swing °** (template settings): sizes each step as a rotation of the
  foot around its hip instead of a stride in mm. 0 uses the stride. Turns and
  the middle legs walking forward rotate by exactly this angle; with the
  default calibration about 64° works forward/backward, 46° for turns and 32°
  sideways before a leg cannot reach or feet collide.
- **Hip − / Hip +** (panel 06): rotate the selected foot around its hip by
  1–10°, keeping its reach and height. Keys `[` and `]` do the same.

## Live walk (inverse kinematics on the board)

*Live walk* under the robot buttons makes the board itself compute every
foot's path and run inverse kinematics every 20 ms, so the robot walks
continuously like an insect instead of stepping between poses. Each foot swings
forward on a smooth raised arc, then pushes straight back on the ground; the
legs' rhythms are staggered back to front (tripod, ripple or wave).

- Press a direction to start (forward, back, sideways, turn on the spot, or
  curve left/right). Press another to steer: the walk turns toward it over
  about one cycle, so planted feet never jump. ■ (or X) slows down over one
  cycle and stops exactly in the stance it started from.
- Settings in the Live walk panel:
  - **Step height** (0–60 mm): how high each foot lifts on its arc.
  - **Speed** (0.1–2.5 steps per second): each leg steps once per cycle.
  - **Stride angle** (2–60°): how far each hip swings per step; about 3 mm of
    step per degree at the home reach.
  The panel shows the resulting step length and walking speed. All three can
  be changed while walking: the board checks the new values from the stance
  the walk started in, then blends into them over about one step (speed
  changes never jump the legs to another point of the cycle). Gait and cycles
  (0 = until stopped) are set before starting. It starts from whatever stance
  the robot holds (Stand trims too) and speeds up over the first cycle.
- The app checks the whole walk (reach, joint limits, collisions) before
  starting, and the board checks again. Pause holds at once; Disable motors and
  the 3-second watchdog release the servos as usual. Keys: W A S D, Q / E, X.
- *Preview walk* animates the same foot paths offline.

## Smooth sequences

- Boards with the current `motion_bridge.py` queue the next pose and start it
  the moment the current one ends, so sequences play without a pause between
  poses (with 150 ms or longer poses on a normal USB connection).
- *Smoothness* (insect gaits) multiplies the poses per cycle by 2 or 3 and
  divides each pose's time to match, so foot arcs are rounder at the same
  walking speed.
