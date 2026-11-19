from __future__ import annotations

import math

from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static

from tui.theme import DIM, ORANGE


_HINTS = (
    "[s] start  [x] stop  [r/q/k] decide  [m] queue  [Q] quit"
)


class StatusBar(Widget):
    def compose(self) -> ComposeResult:
        yield Static(_HINTS, id="sb-hints")
        yield Static("", id="sb-telem")

    def update_telem(
        self,
        x: float,
        y: float,
        theta_rad: float,
        tof_mm: float,
        link: bool = True,
    ) -> None:
        deg = math.degrees(theta_rad)
        link_ch = f"[{ORANGE}]●[/{ORANGE}]" if link else f"[{DIM}]○[/{DIM}]"
        telem = (
            f"[{DIM}]"
            f"θ={deg:+.1f}°  x={x:.2f}m  y={y:.2f}m  tof={tof_mm:.0f}mm  link={link_ch}"
            f"[/{DIM}]"
        )
        self.query_one("#sb-telem", Static).update(telem)
