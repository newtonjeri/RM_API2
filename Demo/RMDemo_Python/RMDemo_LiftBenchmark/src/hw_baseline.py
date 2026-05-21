"""
hw_baseline.py
--------------
Shared hardware baseline store for the LiftBenchmark test suite.

Each test writes its measured values here so later tests can derive
calibrated thresholds instead of using conservative hardcoded estimates.

File location: hw_baseline.json  (same directory as this module)

Schema
------
    mean_lat_us      float   API mean per-call latency (µs)        ← test 2
    min_lat_us       float   API min per-call latency (µs)         ← test 2
    max_lat_us       float   API max per-call latency (µs)         ← test 2
    cmd_rate_hz      float   Achieved non-blocking command rate     ← test 2
    udp_interval_ms  float   UDP packet mean interval (ms)         ← test 3
    udp_sigma_ms     float   UDP packet interval std dev (ms)      ← test 3
    jitter_mean_ms   float   Command timing jitter mean (ms)       ← test 3
    jitter_max_ms    float   Command timing jitter max (ms)        ← test 3
    connect_time_ms  float   Time to establish robot connection     ← test 1
    speed_map        dict    speed% → effective mm/s               ← test 6

Usage
-----
    import hw_baseline

    # Read a value (with default fallback if baseline not yet populated):
    thresh_us = hw_baseline.latency_threshold_us(multiplier=2.0)

    # Get UDP interval window (mean ± n_sigma):
    lo, hi = hw_baseline.udp_interval_window_ms(n_sigma=3.0)

    # Write measured values after a test run:
    hw_baseline.update({
        "mean_lat_us": 7055.7,
        "udp_interval_ms": 10.0,
        "udp_sigma_ms": 0.47,
    })
"""

import json
import pathlib
from typing import Any

_BASELINE_FILE = pathlib.Path(__file__).parent / "hw_baseline.json"

# Conservative defaults used when no prior run has populated the baseline.
# These match the first-run estimates that were previously hardcoded in each test.
_DEFAULTS: dict = {
    "mean_lat_us":     15000.0,   # µs — test 2 populates this
    "min_lat_us":       4000.0,
    "max_lat_us":      40000.0,
    "cmd_rate_hz":      100.0,
    "udp_interval_ms":   10.0,   # ms — test 3 populates this
    "udp_sigma_ms":       1.0,
    "jitter_mean_ms":     1.0,
    "jitter_max_ms":     10.0,
    "connect_time_ms":  500.0,
    "speed_map":          {},
}


# ─── I/O helpers ─────────────────────────────────────────────────────────────

def load() -> dict:
    """Return the full baseline dict; missing keys are filled from _DEFAULTS."""
    if _BASELINE_FILE.exists():
        with _BASELINE_FILE.open() as f:
            data = json.load(f)
        merged = dict(_DEFAULTS)
        merged.update(data)
        return merged
    return dict(_DEFAULTS)


def update(values: dict) -> None:
    """Merge *values* into the baseline file (creates file if absent)."""
    current = load()
    current.update(values)
    with _BASELINE_FILE.open("w") as f:
        json.dump(current, f, indent=2)
    _report_written(values)


def get(key: str, default: Any = None) -> Any:
    """Return a single value from the baseline, or *default* if absent."""
    return load().get(key, default)


def _report_written(values: dict) -> None:
    keys = ", ".join(values.keys())
    print(f"  [BASELINE] Saved → {_BASELINE_FILE.name}  ({keys})")


# ─── Derived threshold helpers ────────────────────────────────────────────────

def latency_threshold_us(multiplier: float = 2.0) -> float:
    """
    Return  mean_lat_us × multiplier  as an API-latency assertion threshold.

    With multiplier=2.0 and a measured mean of 7 ms this gives 14 ms — tighter
    than the 15 ms hardcoded estimate used before first calibration.

    Falls back to the _DEFAULTS value (15 ms) when no measurement exists.
    """
    return load()["mean_lat_us"] * multiplier


def udp_interval_window_ms(n_sigma: float = 3.0) -> tuple:
    """
    Return (lo_ms, hi_ms) UDP interval assertion window as mean ± n_sigma·σ.

    With measured mean=10.0 ms, σ=0.47 ms, n_sigma=3:
        window = [10.0 − 1.41, 10.0 + 1.41] = [8.59, 11.41] ms
    tighter than the hardcoded [8, 12] ms estimate.

    Falls back to [10 − n_sigma, 10 + n_sigma] when no measurement exists.
    """
    b = load()
    mean  = b["udp_interval_ms"]
    sigma = b["udp_sigma_ms"]
    return mean - n_sigma * sigma, mean + n_sigma * sigma


def nonblocking_call_limit_ms(multiplier: float = 3.0) -> float:
    """
    Return  mean_lat_us × multiplier / 1000  as a non-blocking call wall-time
    limit in **milliseconds**.

    Replaces the hardcoded 20 ms limit in test 5 (blocking_vs_nonblocking).
    With measured mean=7 ms and multiplier=3: limit = 21 ms.
    """
    return load()["mean_lat_us"] * multiplier / 1000.0


def api_overhead_s() -> float:
    """
    Return  mean_lat_us / 1_000_000  (seconds) as the expected SDK call
    overhead to subtract from wall-clock timing in test 6 (speed_param).
    """
    return load()["mean_lat_us"] / 1_000_000.0
