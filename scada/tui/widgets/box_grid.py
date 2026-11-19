from __future__ import annotations

import sys

from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from tui.theme import BOX_COLORS, DIM

try:
    from arena_config import BOX_ARUCO_IDS
except ImportError:
    BOX_ARUCO_IDS = [10, 11, 12, 13]


class BoxGrid(Widget):
    def compose(self) -> ComposeResult:
        yield Static(
            f"[{DIM}]BOXES[/{DIM}]",
            id="box-grid-label",
        )
        with Widget(id="box-chip-row"):
            for bid in BOX_ARUCO_IDS:
                yield Static(
                    self._chip_text(bid, "UNKNOWN"),
                    id=f"box-chip-{bid}",
                    classes="box-chip",
                )

    def update_box(self, box_id: int, status: str) -> None:
        try:
            chip = self.query_one(f"#box-chip-{box_id}", Static)
            chip.update(self._chip_text(box_id, status))
        except Exception:
            pass

    @staticmethod
    def _chip_text(box_id: int, status: str) -> str:
        bg, fg = BOX_COLORS.get(status, BOX_COLORS["UNKNOWN"])
        label = status[:4] if len(status) > 4 else status
        return f"[{fg} on {bg}] {box_id} {label} [/{fg} on {bg}]"
