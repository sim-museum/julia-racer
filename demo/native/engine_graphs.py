

import json
import math
import os

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QSizePolicy, QWidget

HERE = os.path.dirname(os.path.abspath(__file__))
BLUE = QColor(40, 70, 200); GREEN = QColor(20, 150, 40); RED = QColor(220, 30, 30); GRID = QColor(200, 200, 205)

def load_curves(path=os.path.join(HERE, "engine_curves.json")):
    with open(path) as f:
        return json.load(f)

def car_curve(C, car):
    """(rpm, torque N·m, power kW) of `car`'s full-throttle curve."""
    k = C["cars"].get(car, C["cars"]["Lotus"])
    rpm = [r * k["kR"] for r in C["rpm"]]
    tq = [t * k["kP"] / k["kR"] for t in C["lotus_torque_nm"]]
    kw = [t * r * 2 * math.pi / 60 / 1000 for t, r in zip(tq, rpm)]
    return rpm, tq, kw

def band(rpm, y, frac=0.9):
    """The rpm range over which y is within `frac` of its peak (the contiguous range round the peak)."""
    i = max(range(len(y)), key=lambda j: y[j]); lim = frac * y[i]
    a = i
    while a > 0 and y[a - 1] >= lim:
        a -= 1
    b = i
    while b < len(y) - 1 and y[b + 1] >= lim:
        b += 1
    return rpm[a], rpm[b]

def kmh(rpm, gear, final, kR, rpm_r, redline=9500.0):
    """Road speed [km/h] at the car's rpm in a gear: the garage's wheel radius (rpm_r = 9,500 x r), the final x kR."""
    r = rpm_r / redline
    return rpm * r * 2 * math.pi / 60 / (gear * final * kR) * 3.6

def summary(C, car, gears, final, rpm_r):
    """Everything the two panels draw, as numbers (the launcher test reads these)."""
    rpm, tq, kw = car_curve(C, car)
    kR = C["cars"].get(car, C["cars"]["Lotus"])["kR"]
    tb = band(rpm, tq); pb = band(rpm, kw)
    red = C["redline_rpm"] * kR
    bands = [(kmh(tb[0], g, final, kR, rpm_r), kmh(tb[1], g, final, kR, rpm_r),
              kmh(pb[0], g, final, kR, rpm_r), kmh(pb[1], g, final, kR, rpm_r)) for g in gears]
    return dict(rpm=rpm, tq=tq, kw=kw, peak_tq=max(tq), peak_tq_rpm=rpm[tq.index(max(tq))], peak_kw=max(kw),
                peak_kw_rpm=rpm[kw.index(max(kw))], torque_band=tb, power_band=pb, gear_bands=bands,
                top_kmh=kmh(red, min(gears), final, kR, rpm_r) if gears else 0.0, redline=red)

class EngineGraphs(QWidget):
    """The two Classic-style panels; update(car, gears, final, rpm_r) redraws."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(430)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        try:
            self.C = load_curves()
        except (OSError, ValueError):
            self.C = None
        self.S = None; self.car = "Lotus"

    def update_inputs(self, car, gears, final, rpm_r):
        self.car = car
        if self.C is not None and gears and final > 0:
            self.S = summary(self.C, car, list(gears), final, rpm_r)
        self.update()

    def _axes(self, p, R, xmax, xstep, ymax, ystep, xlabel, title, xfmt=lambda x: "%d" % x):
        p.setPen(QPen(GRID, 1))
        x = 0.0
        while x <= xmax + 1e-6:
            X = R.left() + R.width() * x / xmax
            p.drawLine(QPointF(X, R.top()), QPointF(X, R.bottom()))
            p.setPen(QPen(Qt.GlobalColor.black)); p.drawText(QRectF(X - 30, R.bottom() + 2, 60, 16), Qt.AlignmentFlag.AlignHCenter, xfmt(x))
            p.setPen(QPen(GRID, 1)); x += xstep
        if ystep:
            y = 0.0
            while y <= ymax + 1e-6:
                Y = R.bottom() - R.height() * y / ymax
                p.drawLine(QPointF(R.left(), Y), QPointF(R.right(), Y))
                p.setPen(QPen(Qt.GlobalColor.black)); p.drawText(QRectF(R.left() - 46, Y - 8, 40, 16), Qt.AlignmentFlag.AlignRight, "%d" % y)
                p.setPen(QPen(GRID, 1)); y += ystep
        p.setPen(QPen(Qt.GlobalColor.black, 1)); p.drawRect(R)
        p.setPen(QColor(150, 20, 20)); f = p.font(); f.setBold(True); p.setFont(f)
        p.drawText(QRectF(R.left(), R.top() - 20, R.width(), 18), Qt.AlignmentFlag.AlignLeft, title)
        f.setBold(False); p.setFont(f); p.setPen(Qt.GlobalColor.black)
        p.drawText(QRectF(R.left(), R.bottom() + 16, R.width(), 16), Qt.AlignmentFlag.AlignHCenter, xlabel)

    def paintEvent(self, ev):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(250, 250, 252))
        p.setFont(QFont("sans", 9))
        if self.S is None:
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "engine curve not available (engine_curves.json)")
            return
        S = self.S; W = self.width(); H = self.height()
        R1 = QRectF(56, 26, W - 76, H * 0.46 - 54)
        R2 = QRectF(56, H * 0.46 + 26, W - 76, H * 0.54 - 64)

        xmax = 1000 * math.ceil((S["redline"] + 500) / 1000); ymax = 50 * math.ceil(max(S["peak_tq"], S["peak_kw"]) * 1.1 / 50)
        self._axes(p, R1, xmax, 2000 if xmax > 9000 else 1000, ymax, 50, "rpm",
                   "%s: torque (N·m, blue) and power (kW, green) versus rpm -- peaks %.0f N·m at %.0f, %.0f kW (%.0f PS) at %.0f"
                   % (self.car, S["peak_tq"], S["peak_tq_rpm"], S["peak_kw"], S["peak_kw"] * 1.3596, S["peak_kw_rpm"]))
        for ys, col in ((S["tq"], BLUE), (S["kw"], GREEN)):
            p.setPen(QPen(col, 2))
            pts = [QPointF(R1.left() + R1.width() * r / xmax, R1.bottom() - R1.height() * y / ymax) for r, y in zip(S["rpm"], ys) if r <= xmax]
            for a, b in zip(pts, pts[1:]):
                p.drawLine(a, b)

        n = len(S["gear_bands"]); vmax = 20 * math.ceil(max(S["top_kmh"], 1.0) * 1.08 / 20)
        self._axes(p, R2, vmax, 20 if vmax <= 240 else 40, 1, 0, "km/h",
                   "90 %% torque (blue) and power (green) bands versus speed; red: the redline in top gear, %.0f km/h" % S["top_kmh"])
        rowh = R2.height() / (n + 1)
        for i, (t0, t1, p0, p1) in enumerate(S["gear_bands"]):
            y = R2.top() + rowh * (i + 1)
            p.setPen(Qt.GlobalColor.black); p.drawText(QRectF(R2.left() - 30, y - 8, 24, 16), Qt.AlignmentFlag.AlignRight, str(i + 1))
            X = lambda v: R2.left() + R2.width() * min(v, vmax) / vmax
            p.setPen(QPen(BLUE, 3)); p.drawLine(QPointF(X(t0), y - 3), QPointF(X(t1), y - 3))
            p.setPen(QPen(GREEN, 3)); p.drawLine(QPointF(X(p0), y + 3), QPointF(X(p1), y + 3))
        p.setPen(QPen(RED, 2)); xr = R2.left() + R2.width() * S["top_kmh"] / vmax
        p.drawLine(QPointF(xr, R2.top()), QPointF(xr, R2.bottom()))
