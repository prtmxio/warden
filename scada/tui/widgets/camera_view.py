from __future__ import annotations

from textual.widget import Widget
from textual.widgets import Static
from textual.app import ComposeResult

from tui.theme import DIM, BORDER


_SIGNAL_LOST = (
    f"[{DIM}]"
    "                    \n"
    "   SIGNAL LOST      \n"
    "   no frame         \n"
    "                    \n"
    f"[/{DIM}]"
)


class CameraView(Widget):
    def __init__(self, label: str = "CAM", **kwargs) -> None:
        super().__init__(**kwargs)
        self._label = label
        self._cols = 40
        self._rows = 12

    def compose(self) -> ComposeResult:
        yield Static(f"[dim]-- {self._label} --[/dim]", id="cam-label")
        yield Static(_SIGNAL_LOST, id="cam-frame")

    def on_resize(self, event) -> None:
        self._cols = max(10, event.size.width - 4)
        self._rows = max(4,  event.size.height - 3)

    def update_frame(self, ascii_str: str) -> None:
        self.query_one("#cam-frame", Static).update(ascii_str)

    def show_lost(self) -> None:
        self.query_one("#cam-frame", Static).update(_SIGNAL_LOST)
