"""
Pixel-art robot sprite and compass glyphs.

The sprite is 9 chars wide × 7 lines tall (block/half-block chars).
8 heading variants cover N/NE/E/SE/S/SW/W/NW (45-degree buckets).
Orange accent pixel marks the direction indicator.
"""

from __future__ import annotations

import math

_ACCENT = "#ff7a18"
_BASE = "#c8c8d0"

_ARROWS = ["↑", "↗", "→", "↘", "↓", "↙", "←", "↖"]

_HEADING_LABELS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

# Each sprite is a tuple of 7 lines.  Accent positions are marked with 'A',
# base body chars are literal block characters, spaces are spaces.
# Lines are stored as (base_line, accent_positions) where accent_positions is
# a set of column indices that should be colored with the orange accent.

_SPRITE_BODY: list[str] = [
    " ▄███▄ ",
    "█▌   ▐█",
    "█▌ ● ▐█",
    " █████ ",
    "  █ █  ",
    " █   █ ",
    "▀█   █▀",
]

# Direction indicator rows per heading bucket (0=N … 7=NW)
# Each entry: (row_index_in_sprite, col_index, char)
_DIRECTION_MARKS: list[tuple[int, int, str]] = [
    (0, 3, "▲"),  # N  — top center
    (0, 5, "▲"),  # NE — top right
    (3, 6, "▶"),  # E  — mid right
    (5, 5, "▼"),  # SE — bottom right
    (5, 3, "▼"),  # S  — bottom center
    (5, 1, "▼"),  # SW — bottom left
    (3, 0, "◀"),  # W  — mid left
    (0, 1, "▲"),  # NW — top left
]


def _theta_to_bucket(theta_rad: float) -> int:
    """Map radians (0=East, counter-clockwise) to 0-7 heading bucket (0=N)."""
    # Convert math convention (0=East, CCW) to compass (0=North, CW)
    compass_deg = (90.0 - math.degrees(theta_rad)) % 360.0
    bucket = int((compass_deg + 22.5) / 45.0) % 8
    return bucket


def robot_sprite(theta_rad: float, *, color: str = "#ff7a18") -> str:
    """Return a 7-line Rich markup string for the robot sprite with directional accent."""
    bucket = _theta_to_bucket(theta_rad)
    acc_row, acc_col, acc_char = _DIRECTION_MARKS[bucket]

    lines: list[str] = []
    for row_idx, line in enumerate(_SPRITE_BODY):
        if row_idx == acc_row:
            chars = list(line)
            # Pad if needed
            while len(chars) <= acc_col:
                chars.append(" ")
            chars[acc_col] = acc_char
            before = "".join(chars[:acc_col])
            after = "".join(chars[acc_col + 1:])
            rendered = (
                f"[{_BASE}]{before}[/{_BASE}]"
                f"[{color}]{acc_char}[/{color}]"
                f"[{_BASE}]{after}[/{_BASE}]"
            )
        else:
            rendered = f"[{_BASE}]{line}[/{_BASE}]"
        lines.append(rendered)

    return "\n".join(lines)


def heading_glyph(theta_rad: float) -> str:
    """Return one of ↑ ↗ → ↘ ↓ ↙ ← ↖ based on θ (radians, 0=East, CCW+)."""
    bucket = _theta_to_bucket(theta_rad)
    return _ARROWS[bucket]


if __name__ == "__main__":
    import math
    import re

    def strip(s: str) -> str:
        return re.sub(r"\[[^\]]+\]", "", s)

    headings = [
        ("N",  math.pi / 2),
        ("NE", math.pi / 4),
        ("E",  0.0),
        ("SE", -math.pi / 4),
        ("S",  -math.pi / 2),
        ("SW", -3 * math.pi / 4),
        ("W",  math.pi),
        ("NW", 3 * math.pi / 4),
    ]

    for label, theta in headings:
        glyph = heading_glyph(theta)
        print(f"--- {label} ({glyph}) ---")
        print(strip(robot_sprite(theta)))
        print()
