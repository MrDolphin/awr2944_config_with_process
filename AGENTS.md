# AWR2944 Project Operating Rules

## Canonical development source

- This worktree (`awr2944_config_and_process_with_trace_codex`) is the only
  location for code edits, tests, commits, pushes, and Pull Request updates.
- `D:\hp-laptop\USV\awr2944_config_and_process_with_trace` is a legacy
  hardware-reference tree. Read it only to recover proven wiring or test logic;
  copy a selected change deliberately and test it in this worktree.
- Before any commit, verify the current branch and `git status --short`.

## Hardware-aware development

- Hardware is not currently connected. Do not claim GPIO, serial, radar, motor,
  encoder, or deployment behaviour has been verified without a recorded test.
- Keep hardware adapters disabled by default. Put motion policy and point-cloud
  metadata logic behind pure, testable modules first.
- Treat pin polarity, counts per output revolution, safe end angles, settling
  time, and radar field-of-view overlap as acceptance items for real hardware.

## Development loop

1. Choose one small, actionable outcome with explicit acceptance tests.
2. Search the code graph before broad text search; use legacy files only when
   looking for non-indexed wiring or hardware reference material.
3. Write or update the failing test before production code.
4. Implement the smallest change, then run focused and full tests.
5. Commit and push only a verified, coherent phase. Add a PR comment only for a
   material milestone, using rendered Markdown rather than escaped newlines.
6. If no safe, hardware-independent task remains, record the blocking acceptance
   item and exit instead of repeating broad analysis.

## Background runs

- Start with the worktree, tests, PR/CI status, and open acceptance items.
- Exit quickly when there is no actionable task. Do not create subagents or do
  broad research solely to fill a background run.
