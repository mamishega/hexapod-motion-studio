# Motion Studio bug fixes

- A file import could finish after physical playback started and replace the timeline used by playback callbacks. The app now rechecks command/playback state after reading the file and rejects the import if busy.
- Cancelling the player immediately unlocked Play while Pause/Disable was still awaiting acknowledgement. A pending-stop counter now locks playback, editing and preview until all stop requests finish, including the error-recovery pause.
- Pausing froze the animated model between board reports. The scene now uses the acknowledged held position after Pause/Disable; Enable also updates it to the acknowledged home position.
- A pose reply reporting disabled outputs could bypass the player's check when no transition was in progress. The player now checks output state on the initial reply as well as every poll.

Comments beside the fixes explain the bugs and changes. Verification: 34 JavaScript tests and 8 simulated board tests passed. Physical robot operation and browser rendering were not verified in this review. Board files are unchanged.

- A delayed USB write rejection could reject a newer request after the original command was acknowledged. Write errors now match their original request ID before clearing pending state.
- Immediate USB write exceptions left a stale pending request and timeout. They now use the same cleanup path as asynchronous errors. Both regressions were reproduced with failing tests before the fix.

- Disconnect previously left Stand/Sit/Play enabled while waiting for the off reply. Controls now stay locked through shutdown, and the connection is captured before awaiting replies.
- Events and errors from an old port could change or disconnect a new session. Connection callbacks and background status polling now verify the originating connection before updating state. Two regression tests failed before these fixes and pass afterward.
