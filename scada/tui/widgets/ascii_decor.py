from __future__ import annotations

import sys

from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Static

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from ui.art.dither import diagonal_accent


class AsciiDecor(Widget):
    def compose(self) -> ComposeResult:
        yield Static(diagonal_accent(40, 6), id="decor-frame")

    def on_resize(self, event) -> None:
        cols = max(8, event.size.width - 4)
        rows = max(2, event.size.height - 2)
        self.query_one("#decor-frame", Static).update(diagonal_accent(cols, rows))
