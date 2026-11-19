from __future__ import annotations

import sys

from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from ui.art.dotmatrix import render_number
from tui.theme import STATE_COLORS, DIM, WHITE, ORANGE


class _Tile(Widget):
    DEFAULT_CSS = """
    _Tile {
        width: 1fr;
        height: 9;
        padding: 0 1;
        margin: 0 1 0 0;
    }
    """

    def __init__(self, label: str, tile_id: str, **kwargs) -> None:
        super().__init__(id=tile_id, **kwargs)
        self._label = label
        self._value_id = f"{tile_id}-val"
        self._label_id = f"{tile_id}-lbl"

    def compose(self) -> ComposeResult:
        yield Static(self._label, id=self._label_id, classes="tile-label")
        yield Static(render_number("---"), id=self._value_id)

    def set_text(self, text: str, color: str = ORANGE) -> None:
        self.query_one(f"#{self._value_id}", Static).update(render_number(text, color=color))


class StatTiles(Widget):
    def compose(self) -> ComposeResult:
        with Widget(id="tile-row"):
            yield _Tile("STATE",  "tile-state")
            yield _Tile("TOF mm", "tile-tof")
            yield _Tile("X m",    "tile-x")
            yield _Tile("Y m",    "tile-y")
            yield _Tile("THETA",  "tile-theta")
            yield _Tile("OK",     "tile-ok")
            yield _Tile("FAULT",  "tile-fault")

    def update_state(self, state: str) -> None:
        color = STATE_COLORS.get(state, WHITE)
        abbrev = state[:6] if len(state) > 6 else state
        self.query_one("#tile-state-val", Static).update(
            render_number(abbrev, color=color)
        )

    def update_tof(self, mm: float) -> None:
        self.query_one("#tile-tof-val", Static).update(
            render_number(f"{mm:.0f}", color=ORANGE)
        )

    def update_pose(self, x: float, y: float, theta_deg: float) -> None:
        self.query_one("#tile-x-val", Static).update(
            render_number(f"{x:.2f}", color=ORANGE)
        )
        self.query_one("#tile-y-val", Static).update(
            render_number(f"{y:.2f}", color=ORANGE)
        )
        self.query_one("#tile-theta-val", Static).update(
            render_number(f"{theta_deg:.1f}", color=ORANGE)
        )

    def update_counts(self, ok: int, fault: int) -> None:
        ok_color    = "#aaffaa" if ok > 0    else DIM
        fault_color = "#ffaaaa" if fault > 0 else DIM
        self.query_one("#tile-ok-val",    Static).update(render_number(str(ok),    color=ok_color))
        self.query_one("#tile-fault-val", Static).update(render_number(str(fault), color=fault_color))
