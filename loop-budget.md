# Background Development Budget

## Daily caps

| Pattern | Daily cap | Maximum per run | Max sub-agent spawns/run |
| --- | ---: | ---: | ---: |
| `hardware-independent-development` | 60,000 tokens | 20,000 tokens | 0 |
| `pr-ci-check` | 12,000 tokens | 4,000 tokens | 0 |
| `hardware-blocked-review` | 8,000 tokens | 3,000 tokens | 0 |

## Kill switches

- `loop-pause-all`: false
- Enter report-only mode at 80% of a pattern's daily cap.
- Stop at 100% of a pattern's daily cap.
- Never start a new code change without one explicit acceptance test or a
  recorded documentation/operational outcome.

## Alerts This Period

- 2026-07-19 14:55 +08:00: 39,000 estimated tokens already spent on
  `hardware-independent-development`; defer the larger radar-server integration
  until a fresh budget period to avoid crossing the 80% report-only threshold.
