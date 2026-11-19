"""
5×7 dot-matrix numeral renderer.

Each glyph is a list of 7 strings, each exactly 5 chars wide using '█' and ' '.
Digits are separated by a 1-column gap. The result is a multi-line Rich-markup
string where every line is one row of the dot-matrix display.
"""

from __future__ import annotations

_GLYPHS: dict[str, list[str]] = {
    "0": [
        " ███ ",
        "█   █",
        "█  ██",
        "█ █ █",
        "██  █",
        "█   █",
        " ███ ",
    ],
    "1": [
        "  █  ",
        " ██  ",
        "  █  ",
        "  █  ",
        "  █  ",
        "  █  ",
        " ███ ",
    ],
    "2": [
        " ███ ",
        "█   █",
        "    █",
        "  ██ ",
        " █   ",
        "█    ",
        "█████",
    ],
    "3": [
        " ███ ",
        "█   █",
        "    █",
        "  ██ ",
        "    █",
        "█   █",
        " ███ ",
    ],
    "4": [
        "   █ ",
        "  ██ ",
        " █ █ ",
        "█  █ ",
        "█████",
        "   █ ",
        "   █ ",
    ],
    "5": [
        "█████",
        "█    ",
        "████ ",
        "    █",
        "    █",
        "█   █",
        " ███ ",
    ],
    "6": [
        " ███ ",
        "█    ",
        "█    ",
        "████ ",
        "█   █",
        "█   █",
        " ███ ",
    ],
    "7": [
        "█████",
        "    █",
        "   █ ",
        "  █  ",
        " █   ",
        " █   ",
        " █   ",
    ],
    "8": [
        " ███ ",
        "█   █",
        "█   █",
        " ███ ",
        "█   █",
        "█   █",
        " ███ ",
    ],
    "9": [
        " ███ ",
        "█   █",
        "█   █",
        " ████",
        "    █",
        "    █",
        " ███ ",
    ],
    ".": [
        "  ",
        "  ",
        "  ",
        "  ",
        "  ",
        " █",
        " █",
    ],
    "-": [
        "   ",
        "   ",
        "   ",
        "███",
        "   ",
        "   ",
        "   ",
    ],
    "+": [
        "   ",
        " █ ",
        " █ ",
        "███",
        " █ ",
        " █ ",
        "   ",
    ],
    " ": [
        "   ",
        "   ",
        "   ",
        "   ",
        "   ",
        "   ",
        "   ",
    ],
}

_GAP = " "


def render_number(value: str | int | float, *, color: str = "#ff7a18") -> str:
    """Return a Rich-markup string of dot-matrix numerals for `value`.

    Each digit is 5 cols wide, separated by a 1-col gap.  The result has 7
    lines, one per dot-matrix row, all wrapped in a single color tag pair.
    """
    text = str(value)
    rows: list[list[str]] = [[] for _ in range(7)]

    for i, ch in enumerate(text):
        glyph = _GLYPHS.get(ch, _GLYPHS[" "])
        for row_idx, row_str in enumerate(glyph):
            if i > 0:
                rows[row_idx].append(_GAP)
            rows[row_idx].append(row_str)

    lines = ["".join(r) for r in rows]
    body = "\n".join(lines)
    return f"[{color}]{body}[/{color}]"


if __name__ == "__main__":
    import sys

    samples = ["0123456789", "-3.14", "+42", "100.0"]
    for s in samples:
        print(f"--- {s!r} ---")
        markup = render_number(s)
        # Strip Rich tags for plain terminal output so the demo works without Rich.
        import re
        plain = re.sub(r"\[[^\]]+\]", "", markup)
        print(plain)
        print()
