"""WGTD-1 (h): Classic Replay Analyser's GRAPHICAL race reports for julia racer recordings (.jmr).

Classic Replay Analyser ("Classic Replay Analyser.txt", Reports):
  * Race History Graph -- "the development of the race for all drivers in one single graph. All drivers are compared to
    a virtual driver whose laptimes are consistent and equal to the winner's average laptime."
  * Graphical Race Report -- "Race and LapByLap reports combined and shown as a picture": here the lap chart drawn as
    position-per-lap lines, the classic graphical lap chart.

Both are computed from the same race distance per car as the text reports (analyser._cum), so the pictures and the
tables cannot disagree. RaceGraphs is a QWidget (tab "Race graphs" in the analyser) with a selector and PNG export.
"""
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QComboBox, QFileDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

CAR_COLOURS = ["#e8443a", "#2f7fd8", "#3aa655", "#d89a2f", "#8e5bd0", "#2fb7b7", "#c75b9b", "#7a7a7a"]

def crossings(rep, cums=None):
    """For every car: the times at which it completed lap 1, 2, ... -- from the SAME source as the text lap chart
    (analyser.lap_chart: the recording's timed laps, plus the lap-counter crossing for a car's first lap), so the picture
    and the table cannot disagree. (A distance-based crossing disagreed: a respawn moves the car back along the road.)"""
    by = [dict() for _ in range(rep.ncar)]
    for lp in rep.laps:
        by[lp.car][lp.num] = lp.t1
    for c in range(rep.ncar):
        car = rep.cars[c]
        for k in range(1, rep.n):
            if car["lap"][k] == car["lap"][k - 1] + 1:
                n = int(car["lap"][k])
                if n >= 1 and n not in by[c]:
                    by[c][n] = rep.t[k]
    out = []
    for c in range(rep.ncar):
        ts = []; n = 1
        while n in by[c]:
            ts.append(by[c][n]); n += 1
        out.append(ts)
    return out

def lap_durations(rep):
    """Per car, the duration of laps 1, 2, ... (the recording's lap times; consecutive from lap 1). Race TIME, not the
    clock: a recording that spans a restart (before REPLAY-4) has a gap of minutes between two laps' completions."""
    by = [dict() for _ in range(rep.ncar)]
    for lp in rep.laps:
        by[lp.car][lp.num] = lp.time
    out = []
    for c in range(rep.ncar):
        ds = []; n = 1
        while n in by[c]:
            ds.append(by[c][n]); n += 1
        out.append(ds)
    return out

def race_history(rep, cums=None):
    """Classic's race history: per car, (lap k, its race time after k laps − k × the winner's average lap) -- positive =
    behind the virtual driver who laps at the winner's average. The winner: the most laps, then the least race time."""
    dur = lap_durations(rep)
    if not any(dur):
        return None, []
    win = min(range(rep.ncar), key=lambda c: (-len(dur[c]), sum(dur[c]) if dur[c] else 1e18))
    avg = sum(dur[win]) / len(dur[win])
    hist = []
    for c in range(rep.ncar):
        h = []; t = 0.0
        for k, d in enumerate(dur[c], 1):
            t += d; h.append((k, t - k * avg))
        hist.append(h)
    return win, hist

def lap_positions(rep, cums):
    """Per car, its running position at each lap it completed (ranked by when the cars completed that lap; a car not
    yet there ranks behind)."""
    cr = crossings(rep)
    nl = max((len(x) for x in cr), default=0)
    pos = [[] for _ in range(rep.ncar)]
    for k in range(nl):
        done = sorted((cr[c][k], c) for c in range(rep.ncar) if len(cr[c]) > k)
        for p, (_t, c) in enumerate(done, 1):
            pos[c].append((k + 1, p))
    start = sorted(range(rep.ncar), key=lambda c: -(cums[c][0] if cums[c] else -1e18))
    for p, c in enumerate(start, 1):
        pos[c].insert(0, (0, p))
    return pos

class RaceGraphs(QWidget):
    MODES = ("Race history (vs the winner's average lap)", "Lap chart (position per lap)")

    def __init__(self, rep, cum_fn, parent=None):
        super().__init__(parent)
        self.rep = rep
        self.cums = [cum_fn(rep, c) for c in range(rep.ncar)]
        self.win, self.hist = race_history(rep, self.cums)
        self.pos = lap_positions(rep, self.cums)
        v = QVBoxLayout(self); v.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        self.mode = QComboBox(); self.mode.addItems(self.MODES); self.mode.currentIndexChanged.connect(self.update)
        exp = QPushButton("Save picture…"); exp.clicked.connect(self._save)
        row.addWidget(QLabel("Graph:")); row.addWidget(self.mode); row.addStretch(1); row.addWidget(exp)
        v.addLayout(row); v.addStretch(1)

    def _save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save race graph", "race_graph.png", "PNG (*.png)")
        if fn:
            self.grab().save(fn)

    def paintEvent(self, _ev):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#ffffff"))
        r = QRectF(self.rect()).adjusted(84, 72, -170, -44)
        if r.width() < 50 or r.height() < 50:
            return
        if self.mode.currentIndex() == 0:
            self._draw_history(p, r)
        else:
            self._draw_laps(p, r)
        self._legend(p, QRectF(r.right() + 14, r.top(), 150, r.height()))

    def _axes(self, p, r, x0, x1, y0, y1, xlab, ylab, yinv=False, yticks=None):
        p.setPen(QPen(QColor("#888888"), 1)); p.drawRect(r)
        f = QFont(); f.setPointSize(8); p.setFont(f)
        X = lambda x: r.left() + (x - x0) / max(x1 - x0, 1e-9) * r.width()
        Y = (lambda y: r.top() + (y - y0) / max(y1 - y0, 1e-9) * r.height()) if yinv else \
            (lambda y: r.bottom() - (y - y0) / max(y1 - y0, 1e-9) * r.height())
        for k in range(int(x0), int(x1) + 1):
            p.setPen(QPen(QColor("#e4e4e4"), 1)); p.drawLine(QPointF(X(k), r.top()), QPointF(X(k), r.bottom()))
            p.setPen(QColor("#444444")); p.drawText(QPointF(X(k) - 4, r.bottom() + 14), str(k))
        for y in (yticks if yticks is not None else []):
            p.setPen(QPen(QColor("#e4e4e4"), 1)); p.drawLine(QPointF(r.left(), Y(y)), QPointF(r.right(), Y(y)))
            p.setPen(QColor("#444444")); p.drawText(QPointF(r.left() - 34, Y(y) + 4), str(y))
        p.setPen(QColor("#222222"))
        p.drawText(QPointF(r.center().x() - 10, r.bottom() + 34), xlab)
        p.save(); p.translate(r.left() - 50, r.center().y() + 40); p.rotate(-90); p.drawText(QPointF(0, 0), ylab); p.restore()
        return X, Y

    def _draw_history(self, p, r):
        p.setPen(QColor("#222222"))
        f = QFont(); f.setBold(True); p.setFont(f)
        if not self.hist or not any(self.hist):
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "No completed laps in this recording."); return
        p.drawText(QPointF(r.left(), r.top() - 14), "Race history: time behind a driver lapping at the winner's ("
                   + self.rep.names[self.win] + ") average lap -- up = ahead")
        pts = [pt for h in self.hist for pt in h]
        nl = max(k for k, _g in pts)
        gmax = max(g for _k, g in pts); gmin = min(min(g for _k, g in pts), 0.0)
        span = max(gmax - gmin, 1.0)
        step = next(s for s in (1, 2, 5, 10, 20, 30, 60, 120, 300) if span / s <= 10)
        ticks = list(range(int(gmin // step) * step, int(gmax // step + 2) * step, step))
        X, Y = self._axes(p, r, 0, nl, ticks[0], ticks[-1], "lap", "seconds behind", yinv=True, yticks=ticks)
        p.setPen(QPen(QColor("#999999"), 1, Qt.PenStyle.DashLine)); p.drawLine(QPointF(r.left(), Y(0)), QPointF(r.right(), Y(0)))
        for c, h in enumerate(self.hist):
            if not h:
                continue
            col = QColor(CAR_COLOURS[c % len(CAR_COLOURS)])
            p.setPen(QPen(col, 2.4 if c == 0 else 1.6))
            prev = QPointF(X(0), Y(0))
            for k, g in h:
                q = QPointF(X(k), Y(g)); p.drawLine(prev, q); p.drawEllipse(q, 2.5, 2.5); prev = q

    def _draw_laps(self, p, r):
        p.setPen(QColor("#222222"))
        f = QFont(); f.setBold(True); p.setFont(f)
        nl = max((k for ps in self.pos for k, _p in ps), default=0)
        if nl == 0:
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "No completed laps in this recording."); return
        p.drawText(QPointF(r.left(), r.top() - 14), "Lap chart: running position at the end of each lap (lap 0 = the grid)")
        X, Y = self._axes(p, r, 0, nl, 1, self.rep.ncar, "lap", "position", yinv=True, yticks=list(range(1, self.rep.ncar + 1)))
        for c, ps in enumerate(self.pos):
            col = QColor(CAR_COLOURS[c % len(CAR_COLOURS)])
            p.setPen(QPen(col, 3.0 if c == 0 else 2.0))
            for (k0, p0), (k1, p1) in zip(ps, ps[1:]):
                p.drawLine(QPointF(X(k0), Y(p0)), QPointF(X(k1), Y(p1)))
            for k, q in ps:
                p.drawEllipse(QPointF(X(k), Y(q)), 3, 3)

    def _legend(self, p, r):
        f = QFont(); f.setPointSize(9); p.setFont(f)
        for c in range(self.rep.ncar):
            y = r.top() + 8 + 20 * c
            col = QColor(CAR_COLOURS[c % len(CAR_COLOURS)])
            p.setPen(QPen(col, 3)); p.drawLine(QPointF(r.left(), y), QPointF(r.left() + 22, y))
            p.setPen(QColor("#222222")); p.drawText(QPointF(r.left() + 28, y + 4), self.rep.names[c])
