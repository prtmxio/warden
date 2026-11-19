"""
Single-row sparkline and bar helpers for telemetry history.

sparkline  — block-char sparkline over a list of floats.
bar        — horizontal fill bar for a single normalised value.
stat_tile  — compact label+value+sparkline row for a stat.
"""

from __future__ import annotations

_BLOCKS = "▁▂▃▄▅▆▇█"
_N = len(_BLOCKS)


def sparkline(
    values: list[float],
    width: int = 20,
    *,
    lo: float | None = None,
    hi: float | None = None,
    color: str = "#ff7a18",
) -> str:
    """Map values to block chars ▁▂▃▄▅▆▇█ scaled to [lo, hi].

    Returns a single-line Rich-markup string of `width` chars.
    If values is empty returns a dim '─' × width placeholder.
    """
    if not values:
        return f"[dim]{'─' * width}[/dim]"

    # Sample or pad to exactly `width` points
    n = len(values)
    if n >= width:
        # Subsample: pick evenly spaced indices
        step = n / width
        sampled = [values[min(n - 1, int(i * step))] for i in range(width)]
    else:
        # Stretch: repeat last value to fill
        sampled = list(values) + [values[-1]] * (width - n)

    v_lo = lo if lo is not None else min(sampled)
    v_hi = hi if hi is not None else max(sampled)
    span = v_hi - v_lo

    chars: list[str] = []
    for v in sampled:
        if span == 0:
            level = _N // 2
        else:
            norm = max(0.0, min(1.0, (v - v_lo) / span))
            level = min(_N - 1, int(norm * _N))
        chars.append(_BLOCKS[level])

    return f"[{color}]{''.join(chars)}[/{color}]"


def bar(
    value: float,
    width: int = 20,
    *,
    lo: float = 0.0,
    hi: float = 1.0,
    color: str = "#ff7a18",
    bg_color: str = "#0a0a0c",
) -> str:
    """Horizontal fill bar for a single value in [lo, hi].

    Returns a single-line Rich-markup string of `width` chars using '█' and '░'.
    """
    span = hi - lo
    if span == 0:
        filled = width // 2
    else:
        norm = max(0.0, min(1.0, (value - lo) / span))
        filled = int(norm * width)

    empty = width - filled
    return (
        f"[{color}]{'█' * filled}[/{color}]"
        f"[{bg_color}]{'░' * empty}[/{bg_color}]"
    )


def stat_tile(
    label: str,
    value: float,
    history: list[float],
    unit: str = "",
    width: int = 20,
    *,
    color: str = "#ff7a18",
    label_color: str = "#888888",
) -> str:
    """Compact one-line stat: label  value unit  sparkline.

    Returns a single Rich-markup string.
    """
    value_str = f"{value:>7.2f}{unit}"
    spark = sparkline(history, width, color=color)
    return (
        f"[{label_color}]{label:<8}[/{label_color}]"
        f"[{color}]{value_str}[/{color}]"
        f"  {spark}"
    )


if __name__ == "__main__":
    import math
    import re

    def strip(s: str) -> str:
        return re.sub(r"\[[^\]]+\]", "", s)

    # Sine wave
    wave = [math.sin(i * 0.4) for i in range(60)]

    print("--- sparkline (60 sine samples → width 40) ---")
    print(strip(sparkline(wave, width=40)))
    print()

    print("--- sparkline empty → placeholder ---")
    print(strip(sparkline([], width=20)))
    print()

    print("--- bar at 0%, 50%, 100% ---")
    for v in [0.0, 0.5, 1.0]:
        print(f"  {v:.0%}  {strip(bar(v, width=20))}")
    print()

    print("--- stat_tile ---")
    tof_history = [300 - i * 2 for i in range(30)]
    print(strip(stat_tile("TOF", 240.0, tof_history, unit="mm", width=24)))
    print()

    print("--- sparkline fewer points than width (stretch) ---")
    short = [1.0, 3.0, 2.0, 5.0]
    print(strip(sparkline(short, width=20)))
    print()
