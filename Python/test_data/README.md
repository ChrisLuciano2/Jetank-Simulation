# test_data — recorded telemetry for offline regression scenarios

Two live-drive recordings from Unity, replayed by `test_gap_logic.py`
scenarios G and K. Each row is one controller tick:

    t, distances, motor_l, motor_r, state, target_deg, search_side

`distances` is the semicolon-joined 13-ray scan, nearest-first by ray
index (left → right across the 120° fan).

| File | Encodes |
|------|---------|
| `run2_wave_bug.csv` | The "waving" bug: gap hysteresis snapped the committed target back to the containing gap's centre whenever the scan momentarily read fully-free, oscillating left/right of a 1×1 cube five times in 30 s. |
| `run6_pivot_exit_bug.csv` | A 3-tick "corridor is clear" blip during PIVOT (a corner swinging past the ray fan and back) that fooled the old 2-tick debounce into exiting early and driving back into the same corner. |

Record a fresh one with:

```bash
py -3.8 test_gap_navigation.py 30 myrun.csv
```

Replay it offline (no Unity needed) with:

```bash
py -3.8 test_gap_navigation.py --replay myrun.csv
```

Only add a recording here if a test scenario actually loads it — these
are regression fixtures, not a general run archive.
