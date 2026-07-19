# Project State

## Active, hardware-independent work

- Integrate the tested encoder sweep plan and GPIO adapter with the radar server
  as a disabled-by-default scan mode; expose only measured scan angle to the
  radar pipeline.
- Preserve the static-capture policy: point-cloud frames are accepted only after
  the scan plan reports that an endpoint has settled.

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
