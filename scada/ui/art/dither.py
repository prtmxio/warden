"""
Bayer 4×4 ordered dither → ASCII art.

bayer_dither  — convert a BGR numpy frame to a dithered Rich-markup string.
diagonal_accent — static decorative diagonal stripe, no image input.
"""

from __future__ import annotations

import numpy as np

_BAYER4: np.ndarray = np.array(
    [
        [ 0,  8,  2, 10],
        [12,  4, 14,  6],
        [ 3, 11,  1,  9],
        [15,  7, 13,  5],
    ],
    dtype=np.float32,
) / 16.0  # normalised 0..1


def bayer_dither(
    image_bgr: np.ndarray,
    cols: int = 60,
    rows: int = 20,
    ramp: str = " .:-=+*#%@",
    fg_color: str = "#ff7a18",
    bg_color: str = "#0a0a0c",
    tint_orange: bool = True,
) -> str:
    """Downsample image_bgr to cols×rows, Bayer-dither, map to ramp chars.

    Returns a multi-line Rich-markup string (rows lines of cols chars).
    When tint_orange is True the brightest cells use fg_color, dim cells use bg_color.
    """
    h, w = image_bgr.shape[:2]

    # Downsample: compute mean of each cell block
    cell_h = max(1, h // rows)
    cell_w = max(1, w // cols)

    gray_cells = np.zeros((rows, cols), dtype=np.float32)
    for r in range(rows):
        for c in range(cols):
            block = image_bgr[
                r * cell_h : min((r + 1) * cell_h, h),
                c * cell_w : min((c + 1) * cell_w, w),
            ]
            if block.size == 0:
                gray_cells[r, c] = 0.0
            else:
                # Luminance (BT.601)
                b = block[:, :, 0].mean() / 255.0
                g = block[:, :, 1].mean() / 255.0
                rv = block[:, :, 2].mean() / 255.0
                gray_cells[r, c] = 0.114 * b + 0.587 * g + 0.299 * rv

    n = len(ramp)
    output_lines: list[str] = []

    for r in range(rows):
        line_parts: list[str] = []
        prev_color: str | None = None
        segment: list[str] = []

        for c in range(cols):
            lum = gray_cells[r, c]
            threshold = _BAYER4[r % 4, c % 4]
            dithered = lum + threshold * (1.0 / n)
            level = min(n - 1, int(dithered * n))
            ch = ramp[level]

            if tint_orange:
                t = level / max(1, n - 1)
                cell_color = fg_color if t > 0.4 else bg_color
            else:
                cell_color = fg_color

            if cell_color != prev_color:
                if segment:
                    line_parts.append(f"[{prev_color}]{''.join(segment)}[/{prev_color}]")
                segment = [ch]
                prev_color = cell_color
            else:
                segment.append(ch)

        if segment and prev_color:
            line_parts.append(f"[{prev_color}]{''.join(segment)}[/{prev_color}]")

        output_lines.append("".join(line_parts))

    return "\n".join(output_lines)


def diagonal_accent(cols: int = 40, rows: int = 12) -> str:
    """Static decorative diagonal stripe pattern using Bayer thresholds directly.

    The gradient runs from top-left (bright orange) to bottom-right (near-black).
    Returns a Rich-markup multi-line string.
    """
    ramp = " .:-=+*#%@"
    n = len(ramp)
    fg = "#ff7a18"
    bg = "#0a0a0c"

    output_lines: list[str] = []
    for r in range(rows):
        line_parts: list[str] = []
        prev_color: str | None = None
        segment: list[str] = []

        for c in range(cols):
            # Diagonal gradient: lum high at top-left, low at bottom-right
            lum = 1.0 - (r / max(1, rows - 1) * 0.5 + c / max(1, cols - 1) * 0.5)
            threshold = _BAYER4[r % 4, c % 4]
            dithered = lum + threshold * (1.0 / n)
            level = min(n - 1, max(0, int(dithered * n)))
            ch = ramp[level]

            t = level / max(1, n - 1)
            cell_color = fg if t > 0.35 else bg

            if cell_color != prev_color:
                if segment:
                    line_parts.append(f"[{prev_color}]{''.join(segment)}[/{prev_color}]")
                segment = [ch]
                prev_color = cell_color
            else:
                segment.append(ch)

        if segment and prev_color:
            line_parts.append(f"[{prev_color}]{''.join(segment)}[/{prev_color}]")

        output_lines.append("".join(line_parts))

    return "\n".join(output_lines)


if __name__ == "__main__":
    import re

    def strip(s: str) -> str:
        return re.sub(r"\[[^\]]+\]", "", s)

    print("--- diagonal_accent (40×12) ---")
    print(strip(diagonal_accent(40, 12)))
    print()

    print("--- bayer_dither on a synthetic gradient image (60×20) ---")
    # Create a 120×40 synthetic BGR gradient (white top-left, black bottom-right)
    h, w = 40, 120
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for r in range(h):
        for c in range(w):
            v = int(255 * (1.0 - r / h * 0.5 - c / w * 0.5))
            img[r, c] = [v, v, v]

    result = bayer_dither(img, cols=60, rows=20)
    print(strip(result))
    print()
