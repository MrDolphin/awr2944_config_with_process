# Project State

## Active, hardware-independent work

- Add a front-end control only after the disabled-by-default encoder mode has
  been exercised with real GPIO and an explicit operator opt-in.
- Preserve the static-capture policy: the server waits for a new radar frame
  after `capture_ready`, then releases the reverse command.

## Hardware acceptance items

- Measure encoder counts per final platform revolution.
- Confirm motor direction polarity and safe cable-managed sweep limits.
- Measure endpoint settling time and field-of-view overlap.
- Verify actual GPIO wiring, power, and radar data capture.

## Last verified baseline

- Commit `8785554`: encoder sweep planning module, 5 unit tests.
- Commit `3afced6`: canonical worktree and background-loop rules.
- Full test suite: 27 passed, 1 optional browser test skipped.
- GPIO adapter simulation: 2 tests passed; no real GPIO was created or tested.
- Encoder session simulation: 3 tests passed; it exposes measured angle and
  `capture_ready` before applying a reverse command.
- Encoder server mode: disabled by default, requires explicit opt-in and a
  calibrated `counts_per_rev`; full automated suite: 34 passed, 1 skipped.
