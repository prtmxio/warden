from __future__ import annotations

import math
import sys
from typing import Any

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical

from tui import theme
from tui.widgets.title_bar    import TitleBar
from tui.widgets.stat_tiles   import StatTiles
from tui.widgets.camera_view  import CameraView
from tui.widgets.arena_sprite import ArenaSprite
from tui.widgets.box_grid     import BoxGrid
from tui.widgets.status_bar   import StatusBar
from tui.widgets.ascii_decor  import AsciiDecor


class WardenApp(App):
    CSS = theme.CSS

    BINDINGS = [
        Binding("s",     "start",        "Start run",    show=True),
        Binding("x",     "stop_bot",     "Stop bot",     show=True),
        Binding("r",     "repair_now",   "Repair now",   show=True),
        Binding("q",     "queue_fault",  "Queue fault",  show=True),
        Binding("k",     "skip_fault",   "Skip fault",   show=True),
        Binding("m",     "manage_queue", "Manage queue", show=True),
        Binding("Q",     "quit",         "Quit",         show=True),
    ]

    def __init__(self, ros_node: Any = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self._state = "SETUP"
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._tof_mm = 0.0
        self._box_states: dict[int, str] = {}
        self._modal_open = False
        self._queued_ids: list[int] = []

    def compose(self) -> ComposeResult:
        yield TitleBar()
        yield StatTiles(id="stat-tiles")
        with Horizontal():
            with Vertical(id="left-col"):
                yield CameraView(label="OVERHEAD", id="overhead-cam")
                yield CameraView(label="BOT CAM",  id="bot-cam")
            with Vertical(id="right-col"):
                yield ArenaSprite(id="arena")
                yield BoxGrid(id="box-grid")
                yield AsciiDecor(id="ascii-decor")
        yield StatusBar(id="status-bar")

    # ── ROS callbacks (called via call_from_thread) ───────────────────────────

    def on_fsm_state(self, state: str) -> None:
        self._state = state
        try:
            self.query_one("#stat-tiles", StatTiles).update_state(state)
        except Exception:
            pass
        if state != "INSPECTING":
            try:
                self.query_one("#bot-cam", CameraView).show_lost()
            except Exception:
                pass

    def on_inspection_result(self, payload: str) -> None:
        parts = payload.split(":", 1)
        if len(parts) != 2:
            return
        status_key, box_id_str = parts[0], parts[1]
        try:
            box_id = int(box_id_str)
        except ValueError:
            return

        status_map = {
            "FAULT":    "FAULT",
            "OK":       "OK",
            "QUEUED":   "QUEUED",
            "SKIPPED":  "SKIPPED",
            "REPAIRED": "REPAIRED",
        }
        status = status_map.get(status_key, "UNKNOWN")
        self._box_states[box_id] = status

        try:
            self.query_one("#box-grid", BoxGrid).update_box(box_id, status)
        except Exception:
            pass
        try:
            self.query_one("#arena", ArenaSprite).update_box_state(box_id, status)
        except Exception:
            pass

        ok_count    = sum(1 for s in self._box_states.values() if s == "OK")
        fault_count = sum(1 for s in self._box_states.values() if s in ("FAULT", "QUEUED"))
        try:
            self.query_one("#stat-tiles", StatTiles).update_counts(ok_count, fault_count)
        except Exception:
            pass

        if status == "FAULT" and self._state == "AWAITING_HUMAN":
            if status_key == "FAULT":
                self._queued_ids = [
                    bid for bid, st in self._box_states.items() if st == "QUEUED"
                ]
            if not self._modal_open:
                self._open_repair_modal(box_id)

    def on_pose(self, x: float, y: float, theta: float) -> None:
        self._x = x
        self._y = y
        self._theta = theta
        try:
            self.query_one("#stat-tiles", StatTiles).update_pose(x, y, math.degrees(theta))
        except Exception:
            pass
        try:
            self.query_one("#arena", ArenaSprite).update_pose(x, y, theta)
        except Exception:
            pass
        try:
            self.query_one("#status-bar", StatusBar).update_telem(
                x, y, theta, self._tof_mm, link=True
            )
        except Exception:
            pass

    def on_tof(self, mm: float) -> None:
        self._tof_mm = mm
        try:
            self.query_one("#stat-tiles", StatTiles).update_tof(mm)
        except Exception:
            pass
        try:
            self.query_one("#status-bar", StatusBar).update_telem(
                self._x, self._y, self._theta, mm, link=True
            )
        except Exception:
            pass

    def on_overhead_frame(self, ascii_str: str) -> None:
        try:
            self.query_one("#overhead-cam", CameraView).update_frame(ascii_str)
        except Exception:
            pass

    def on_botcam_frame(self, ascii_str: str) -> None:
        try:
            self.query_one("#bot-cam", CameraView).update_frame(ascii_str)
        except Exception:
            pass

    # ── key action handlers ───────────────────────────────────────────────────

    def action_start(self) -> None:
        if self._state == "SETUP" and self.ros_node:
            self.ros_node.publish_decision("START")
        else:
            self.notify("START only valid in SETUP state", severity="warning")

    def action_stop_bot(self) -> None:
        if self.ros_node:
            self.ros_node.publish_stop()
        self.notify("STOP sent")

    def action_repair_now(self) -> None:
        if self._state == "AWAITING_HUMAN" and self.ros_node:
            self.ros_node.publish_decision("REPAIR_NOW")
        else:
            self.notify("Not in AWAITING_HUMAN state", severity="warning")

    def action_queue_fault(self) -> None:
        if self._state == "AWAITING_HUMAN" and self.ros_node:
            self.ros_node.publish_decision("QUEUE")
        else:
            self.notify("Not in AWAITING_HUMAN state", severity="warning")

    def action_skip_fault(self) -> None:
        if self._state == "AWAITING_HUMAN" and self.ros_node:
            self.ros_node.publish_decision("SKIP")
        else:
            self.notify("Not in AWAITING_HUMAN state", severity="warning")

    def action_manage_queue(self) -> None:
        if not self._queued_ids:
            self.notify("No queued faults", severity="information")
            return
        if not self._modal_open:
            self._open_queue_modal()

    def action_quit(self) -> None:
        self.exit()

    # ── modal helpers ─────────────────────────────────────────────────────────

    def _open_repair_modal(self, box_id: int) -> None:
        from tui.screens.repair_modal import RepairModal
        self._modal_open = True

        def _on_dismiss(result: str | None) -> None:
            self._modal_open = False

        self.push_screen(RepairModal(box_id, self), _on_dismiss)

    def _open_queue_modal(self) -> None:
        from tui.screens.queue_modal import QueueModal
        self._modal_open = True

        def _on_dismiss(result: str | None) -> None:
            self._modal_open = False

        self.push_screen(QueueModal(self._queued_ids, self), _on_dismiss)
