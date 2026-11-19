from __future__ import annotations

import math
import sys

from textual.widget import Widget
from textual.widgets import Static
from textual.app import ComposeResult

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from ui.art.sprites import robot_sprite
from tui.theme import BOX_COLORS, DIM, ORANGE, BORDER, PANEL

try:
    from arena_config import BOX_POSITIONS, ARENA_SIZE_CM
    _ARENA_M = ARENA_SIZE_CM / 100.0
except ImportError:
    BOX_POSITIONS = {10: (0.40, 1.70), 11: (0.85, 1.70), 12: (1.35, 1.70), 13: (1.60, 1.70)}
    _ARENA_M = 2.0


class ArenaSprite(Widget):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._x = 0.75
        self._y = 0.75
        self._theta = 0.0
        self._box_states: dict[int, str] = {}
        self._cols = 40
        self._rows = 20

    def compose(self) -> ComposeResult:
        yield Static("", id="arena-frame")

    def on_resize(self, event) -> None:
        self._cols = max(16, event.size.width - 4)
        self._rows = max(8,  event.size.height - 2)
        self._redraw()

    def on_mount(self) -> None:
        self._redraw()

    def update_pose(self, x: float, y: float, theta: float) -> None:
        self._x = x
        self._y = y
        self._theta = theta
        self._redraw()

    def update_box_state(self, box_id: int, status: str) -> None:
        self._box_states[box_id] = status
        self._redraw()

    def _redraw(self) -> None:
        lines = _render_arena(
            self._cols, self._rows,
            self._x, self._y, self._theta,
            self._box_states,
        )
        self.query_one("#arena-frame", Static).update("\n".join(lines))


def _render_arena(
    cols: int,
    rows: int,
    bot_x: float,
    bot_y: float,
    theta: float,
    box_states: dict[int, str],
) -> list[str]:
    grid = [[" " for _ in range(cols)] for _ in range(rows)]

    def to_col(x: float) -> int:
        return int(x / _ARENA_M * (cols - 2)) + 1

    def to_row(y: float) -> int:
        return rows - 2 - int(y / _ARENA_M * (rows - 2))

    for c in range(cols):
        grid[0][c] = "─"
        grid[rows - 1][c] = "─"
    for r in range(rows):
        grid[r][0] = "│"
        grid[r][cols - 1] = "│"
    grid[0][0] = "┌"
    grid[0][cols - 1] = "┐"
    grid[rows - 1][0] = "└"
    grid[rows - 1][cols - 1] = "┘"

    for box_id, (bx, by) in BOX_POSITIONS.items():
        bc = to_col(bx)
        br = to_row(by)
        status = box_states.get(box_id, "UNKNOWN")
        _, fg = BOX_COLORS.get(status, BOX_COLORS["UNKNOWN"])
        label = str(box_id)
        for di, ch in enumerate(label):
            cc = bc + di
            if 1 <= br < rows - 1 and 1 <= cc < cols - 1:
                grid[br][cc] = f"\x1b[1m{ch}\x1b[0m"

    bc = to_col(bot_x)
    br = to_row(bot_y)
    from ui.art.sprites import heading_glyph
    arrow = heading_glyph(theta)
    if 1 <= br < rows - 1 and 1 <= bc < cols - 1:
        grid[br][bc] = arrow

    plain_lines: list[str] = []
    for row_idx, row in enumerate(grid):
        line = ""
        for col_idx, cell in enumerate(row):
            if row_idx == 0 or row_idx == rows - 1 or col_idx == 0 or col_idx == cols - 1:
                line += f"[{DIM}]{cell}[/{DIM}]"
            else:
                line += cell
        plain_lines.append(line)

    return plain_lines
