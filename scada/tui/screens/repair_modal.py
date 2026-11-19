from __future__ import annotations

from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Button, Label, Static

if TYPE_CHECKING:
    from tui.app import WardenApp


class RepairModal(ModalScreen[str]):
    def __init__(self, box_id: int, app_ref: "WardenApp") -> None:
        super().__init__()
        self._box_id = box_id
        self._app_ref = app_ref

    def compose(self) -> ComposeResult:
        with Static(id="repair-container"):
            yield Label(
                f"FAULT detected at Box {self._box_id}",
                id="repair-title",
            )
            with Static(id="repair-btn-row"):
                yield Button("Repair Now [r]", id="repair",  variant="warning")
                yield Button("Queue     [q]", id="queue",   variant="default")
                yield Button("Skip      [k]", id="skip",    variant="default")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        btn = event.button.id
        if btn == "repair":
            self._app_ref.ros_node.publish_decision("REPAIR_NOW")
            self.dismiss("REPAIR_NOW")
        elif btn == "queue":
            self._app_ref.ros_node.publish_decision("QUEUE")
            self.dismiss("QUEUE")
        elif btn == "skip":
            self._app_ref.ros_node.publish_decision("SKIP")
            self.dismiss("SKIP")
