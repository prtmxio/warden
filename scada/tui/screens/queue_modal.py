from __future__ import annotations

from typing import TYPE_CHECKING

from textual.app import ComposeResult
from textual.screen import ModalScreen
from textual.widgets import Button, Label, ListView, ListItem, Static

if TYPE_CHECKING:
    from tui.app import WardenApp


class _BoxItem(ListItem):
    def __init__(self, box_id: int) -> None:
        super().__init__(Label(f"Box {box_id}"))
        self.box_id = box_id
        self._selected = False

    def toggle(self) -> None:
        self._selected = not self._selected
        label_text = f"[>] Box {self.box_id}" if self._selected else f"    Box {self.box_id}"
        self.query_one(Label).update(label_text)


class QueueModal(ModalScreen[str]):
    def __init__(self, queued_ids: list[int], app_ref: "WardenApp") -> None:
        super().__init__()
        self._queued_ids = list(queued_ids)
        self._app_ref = app_ref
        self._order: list[int] = []

    def compose(self) -> ComposeResult:
        with Static(id="queue-container"):
            yield Label(
                "Select repair order (click to toggle, top=first):",
                id="queue-title",
            )
            with ListView(id="queue-list"):
                for bid in self._queued_ids:
                    yield _BoxItem(bid)
            with Static(id="queue-btn-row"):
                yield Button("Confirm order", id="confirm", variant="warning")
                yield Button("Cancel (none)", id="cancel",  variant="default")

    def on_list_item__child_clicked(self, event) -> None:
        item = event.item
        if isinstance(item, _BoxItem):
            item.toggle()
            if item._selected:
                if item.box_id not in self._order:
                    self._order.append(item.box_id)
            else:
                self._order = [b for b in self._order if b != item.box_id]

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "confirm":
            if self._order:
                payload = "REPAIR_ORDER:" + ",".join(str(b) for b in self._order)
            else:
                payload = "REPAIR_ORDER:NONE"
            self._app_ref.ros_node.publish_decision(payload)
            self.dismiss(payload)
        else:
            self._app_ref.ros_node.publish_decision("REPAIR_ORDER:NONE")
            self.dismiss("REPAIR_ORDER:NONE")
