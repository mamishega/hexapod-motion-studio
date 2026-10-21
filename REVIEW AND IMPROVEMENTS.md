# Implemented improvements

- Separate Selected target pose and read-only Board-reported pose tables. Animation source labels distinguish the selected target, sequence preview and reported commands. Reports clear on disconnect.
- Undo/Redo for up to 30 sequence edits, including template replacements, imports and loading saved programs. Automatic browser draft recovery with stored calibration, and visible storage-failure messages. Undo history is session-only; JSON remains the portable backup.
- Persisted browser settings for Stand/Sit heights (30–110 mm, Sit below Stand) and transition duration (500–4000 ms). Physical buttons and newly built posture templates share these settings. Calibration and path validation remain enforced. Enable at home retains the board home pose.

No board files changed. These features do not add sensor feedback, contact physics or balance prediction. Browser appearance and physical robot operation remain unverified in this environment.
