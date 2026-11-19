from __future__ import annotations

from textual.app import ComposeResult
from textual.reactive import reactive
from textual.timer import Timer
from textual.widget import Widget
from textual.widgets import Static


class TitleBar(Widget):
    _blink: reactive[bool] = reactive(False)

    def on_mount(self) -> None:
        self._timer: Timer = self.set_interval(0.6, self._toggle_blink)

    def _toggle_blink(self) -> None:
        self._blink = not self._blink

    def watch__blink(self, value: bool) -> None:
        cursor = "_" if value else " "
        self.query_one("#title-text", Static).update(
            f"[bold]WARDEN // SCADA {cursor}[/bold]"
        )

    def compose(self) -> ComposeResult:
        yield Static("WARDEN // SCADA _", id="title-text")
