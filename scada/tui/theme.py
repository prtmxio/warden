from __future__ import annotations

BG     = "#0a0a0c"
PANEL  = "#101014"
BORDER = "#2a2a30"
ORANGE = "#ff7a18"
DIM    = "#8a8a8a"
WHITE  = "#e8e8ec"

STATE_COLORS: dict[str, str] = {
    "SETUP":            "#888888",
    "IDLE":             "#cccccc",
    "NAVIGATING":       "#4488ff",
    "ALIGNING":         "#ffcc00",
    "APPROACH_SCAN":    "#ffcc00",
    "APPROACH_DOCK":    "#ffcc00",
    "DOCKING":          "#ffcc00",
    "INSPECTING":       "#ffcc00",
    "AWAITING_HUMAN":   "#ff8800",
    "REVIEWING_QUEUE":  "#0dcaf0",
    "REPAIRING":        "#ff8800",
    "RETURNING_HOME":   "#4488ff",
    "MISSION_COMPLETE": "#00cc88",
}

BOX_COLORS: dict[str, tuple[str, str]] = {
    "UNKNOWN":  ("#444444", "#aaaaaa"),
    "OK":       ("#116611", "#aaffaa"),
    "FAULT":    ("#881111", "#ffaaaa"),
    "QUEUED":   ("#664400", "#ffcc88"),
    "REPAIRED": ("#006666", "#aaffff"),
    "SKIPPED":  ("#2a0a0a", "#ff8888"),
}

CSS = f"""
Screen {{
    background: {BG};
    color: {WHITE};
}}

TitleBar {{
    height: 1;
    background: {PANEL};
    color: {ORANGE};
    padding: 0 1;
}}

StatTiles {{
    height: 9;
    background: {BG};
    padding: 0 1;
}}

#tile-row {{
    height: 9;
    layout: horizontal;
}}

.tile {{
    width: 1fr;
    height: 9;
    background: {PANEL};
    border: tall {BORDER};
    padding: 0 1;
    margin: 0 1 0 0;
}}

.tile-label {{
    color: {DIM};
    text-style: none;
    height: 1;
    content-align: center middle;
}}

CameraView {{
    background: {PANEL};
    border: tall {BORDER};
    padding: 0 1;
    height: 1fr;
}}

ArenaSprite {{
    background: {PANEL};
    border: tall {BORDER};
    padding: 0 1;
    height: 1fr;
}}

BoxGrid {{
    height: 5;
    background: {PANEL};
    border: tall {BORDER};
    padding: 0 1;
}}

#box-chip-row {{
    layout: horizontal;
    height: 3;
}}

.box-chip {{
    width: 1fr;
    height: 3;
    border: tall {BORDER};
    content-align: center middle;
    margin: 0 1 0 0;
}}

AsciiDecor {{
    height: 8;
    background: {PANEL};
    border: tall {BORDER};
    padding: 0 1;
}}

StatusBar {{
    height: 1;
    background: {PANEL};
    color: {DIM};
    padding: 0 1;
}}

#left-col {{
    width: 1fr;
}}

#right-col {{
    width: 1fr;
}}

RepairModal {{
    align: center middle;
}}

#repair-container {{
    width: 50;
    height: 15;
    background: {PANEL};
    border: tall {ORANGE};
    padding: 1 2;
}}

#repair-title {{
    color: {ORANGE};
    height: 2;
    content-align: center middle;
    text-style: bold;
}}

#repair-btn-row {{
    layout: horizontal;
    height: 3;
    margin: 1 0 0 0;
}}

#repair-btn-row Button {{
    width: 1fr;
    margin: 0 1 0 0;
    background: {BORDER};
    color: {WHITE};
    border: tall {BORDER};
}}

#repair-btn-row Button:hover {{
    background: {ORANGE};
    color: {BG};
}}

QueueModal {{
    align: center middle;
}}

#queue-container {{
    width: 55;
    height: 24;
    background: {PANEL};
    border: tall {ORANGE};
    padding: 1 2;
}}

#queue-title {{
    color: {ORANGE};
    height: 2;
    content-align: center middle;
    text-style: bold;
}}

#queue-list {{
    height: 10;
    border: tall {BORDER};
    background: {BG};
}}

#queue-btn-row {{
    layout: horizontal;
    height: 3;
    margin: 1 0 0 0;
}}

#queue-btn-row Button {{
    width: 1fr;
    margin: 0 1 0 0;
    background: {BORDER};
    color: {WHITE};
    border: tall {BORDER};
}}

#queue-btn-row Button:hover {{
    background: {ORANGE};
    color: {BG};
}}
"""
