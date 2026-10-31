"""TRACKGUIDE-1: the analyser's "Track guide" tab -- the Classic track guide's corners against the driver's lap.

One row per corner the guide describes (Lights Out Racing: entrance speed, slowest speed, time through the corner, from
the guide author's own replay lap), beside the same numbers from the chosen lap; the selected row's advice (braking
point, gears, line) below. See trackguide.py for where the
numbers come from and how the corners are placed on the lap.
"""
import html

from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (QAbstractItemView, QHeaderView, QLabel, QSplitter, QTableWidget, QTableWidgetItem,
                             QTextBrowser, QVBoxLayout, QWidget)
from PyQt6.QtCore import Qt

import analyser as A
import trackguide as G

HEADS = ["Section", "Corner", "Entry km/h\nguide / you", "Slowest km/h\nguide / you", "Time s\nguide / you", "From the line"]

def _f(x, fmt="{:.0f}"):
    return "—" if x is None else fmt.format(x)

def guide_rows(rep, lap):
    """(rows, source text) for `lap` of `rep`; rows None when the track has no guide (source then says why)."""
    trk = rep.h.get("track", "")
    secs = G.current_sections(trk, rep.laplen, rep.h.get("sections", []))
    entries, src = G.load(trk, [n for _s, n in secs])
    if entries is None:
        return None, src
    text, _p = G._text(trk, "lor")
    G.place(entries, lap, secs, rep.laplen, G.guide_lap(text))
    return G.compare(entries, lap), src

class TrackGuide(QWidget):
    def __init__(self, rep):
        super().__init__()
        self.rep = rep; self.rows = []
        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0)
        self.head = QLabel(""); self.head.setWordWrap(True); v.addWidget(self.head)
        sp = QSplitter(Qt.Orientation.Vertical); v.addWidget(sp, 1)
        self.table = QTableWidget(0, len(HEADS)); self.table.setHorizontalHeaderLabels(HEADS)
        self.table.verticalHeader().setVisible(False); self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._show)
        sp.addWidget(self.table)
        self.text = QTextBrowser(); self.text.setObjectName("guide"); sp.addWidget(self.text)
        sp.setSizes([480, 260])
        self.lap = None

    def set_lap(self, lap):
        """Show the guide against `lap` (the player's first ticked lap, else their best)."""
        if lap is None or lap is self.lap:
            return
        self.lap = lap
        try:
            rows, src = guide_rows(self.rep, lap)
        except Exception as e:
            rows, src = None, f"the track guide could not be read ({e})"
        self.rows = rows or []
        if rows is None:
            self.head.setText(f"No track guide: {src}."); self.table.setRowCount(0); self.text.setPlainText(""); return
        self.head.setText(f"<b>Track guide</b> -- against {html.escape(lap.driver)}, lap {lap.num} "
                          f"({A.fmt_time(lap.time) if lap.time is not None else A._untimed(lap)}). The guide's numbers are "
                          "a fast lap's; the corners are placed on your lap by the track's sections. "
                          "Pick a row for the guide's advice.")
        self.table.setRowCount(len(self.rows))
        for r, e in enumerate(self.rows):
            ent = f"{_f(e['entry'])} / {_f(e['you_entry'])}"
            mn = f"{_f(e['through'])} / {_f(e['you_min'])}"
            tm = f"{_f(e['negotiate'], '{:.2f}')} / {_f(e['you_time'], '{:.2f}')}"
            where = "—" if e.get("s_arrive") is None else f"{max(0.0, e['s_arrive']):.0f}–{e['s_exit']:.0f} m"
            for c, val in enumerate((e["section"] or "start", e["title"], ent, mn, tm, where)):
                it = QTableWidgetItem(val)
                if c == 4 and e["negotiate"] and e["you_time"] and e["you_time"] > 1.3 * e["negotiate"]:
                    it.setForeground(QColor("#e57373"))
                self.table.setItem(r, c, it)
        self.table.resizeColumnsToContents()

        self.text.setHtml("<p>Pick a corner above for the guide's advice.</p>")

    def _show(self):
        r = self.table.currentRow()
        if not (0 <= r < len(self.rows)):
            return
        e = self.rows[r]
        self.text.setHtml(f"<h4>{html.escape(e['section'] or 'start')} — {html.escape(e['title'])}</h4>"
                          f"<p>{html.escape(e['advice'])}</p>")
