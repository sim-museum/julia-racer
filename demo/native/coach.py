"""REPLAY-2 S4 -- an optional Claude Code analysis of a replay (PO 2026-10-06: "Also add an optional claude code analysis
of the replay and how you can improve").

`build_summary` turns a replay into a compact, numbers-first text: your laps, the reference lap you are compared with,
and corner by corner (found from the speed trace) the minimum speeds, where braking and the throttle start, and the
time won or lost. `CoachDialog` shows exactly that text, and only when you press Send runs the local Claude Code CLI
(`claude -p`, no tools, no saved session) with it and shows the answer. Without the CLI the button explains why it is
unavailable; nothing is sent anywhere unless you ask.
"""
import math
import os
import re
import shutil

from PyQt6.QtCore import QProcess, QTimer
from PyQt6.QtWidgets import QDialog, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QTextBrowser, QVBoxLayout

import analyser as A

TIMEOUT_S = 240
SETUPS = {"default": "reference default", "ww103": "WW103 fast loose (Wolfgang Wagner)"}

def claude_cli():
    return shutil.which("claude")

def _section(rep, d):
    name = ""
    for s, n in rep.h.get("sections", []):
        if s <= d:
            name = n
    return name

def corners(ref, min_drop=15.0, gap=150.0):
    """Corners from the reference lap's speed trace: local minima at least `min_drop` km/h below the peak in the
    400 m before them, merged when closer than `gap` m. Returns grid indices of the minima."""
    v = ref.ch["kmh"]; n = len(v); w = int(400 / A.GRID_M); out = []
    for k in range(2, n - 2):
        if v[k] <= v[k - 1] and v[k] <= v[k + 1] and v[k] < min(v[k - 2], v[k + 2]) + 1e-6:
            peak = max(v[max(0, k - w):k + 1])
            if peak - v[k] >= min_drop:
                if out and (k - out[-1]) * A.GRID_M < gap:
                    if v[k] < v[out[-1]]:
                        out[-1] = k
                else:
                    out.append(k)
    return out

def _brake_start(lp, k):
    """Where the slowing for the corner at grid index k starts: the brake pedal if the lap has one, else the speed peak."""
    w = int(400 / A.GRID_M); lo = max(0, k - w)
    if "brake" in lp.ch:
        for j in range(lo, k + 1):
            if lp.ch["brake"][j] > 0.2:
                return j
    seg = lp.ch["kmh"][lo:k + 1]
    if not seg:
        return None
    j = lo + max(range(len(seg)), key=lambda i: seg[i])
    return None if j <= lo + 2 else j

def _throttle_on(lp, k):
    if "throttle" not in lp.ch:
        return None
    for j in range(k, min(len(lp.dist), k + int(300 / A.GRID_M))):
        if lp.ch["throttle"][j] > 0.5:
            return j
    return None

def pick_laps(rep):
    """Your best lap, and what to compare it with: the fastest lap by anyone else, or your own second-best."""
    mine = sorted((lp for lp in rep.laps if lp.car == 0), key=lambda lp: lp.time)
    if not mine:
        return None, None
    others = sorted((lp for lp in rep.laps if lp.car != 0), key=lambda lp: lp.time)
    return mine[0], (others[0] if others else (mine[1] if len(mine) > 1 else None))

CARS = {"lotus": "Lotus 49", "eagle": "Eagle T1G", "ferrari": "Ferrari 312", "brabham": "Brabham BT24",
        "cooper": "Cooper T86", "honda": "Honda RA300", "brm": "BRM P115"}

def car_name(rep):
    """The car the driver raced: the recording names the player "You" (the Lotus) or "You (Cooper)" (PO 2026-10-28:
    "claude analysis says I'm in the lotus 49 rather than the cooper")."""
    names = rep.h.get("names") or ["You"]
    m = re.match(r"You \((\w+)\)", str(names[0]))
    return CARS.get(m.group(1).lower(), m.group(1)) if m else CARS["lotus"]

def build_summary(rep):
    you, ref = pick_laps(rep)
    track = A._track_name(rep)
    L = [f"Julia Racer replay analysis -- {track}, lap {rep.laplen:.0f} m, {car_name(rep)} "
         f"(setup: {SETUPS.get(rep.h.get('carsetup', 'default'), rep.h.get('carsetup', '?'))}, "
         f"gearbox: {'automatic' if rep.h.get('gearbox', 'auto') == 'auto' else 'manual'})."]
    mine = [lp for lp in rep.laps if lp.car == 0]
    if not mine:
        L.append("The driver completed no timed lap in this recording (a race's lap 1 is timed from the green flag, every "
                 "other lap from one crossing of the line to the next).")
        return "\n".join(L)
    ts = [lp.time for lp in mine]
    avg = sum(ts) / len(ts); sd = (sum((t - avg) ** 2 for t in ts) / len(ts)) ** 0.5
    L.append(f"Your timed laps: {', '.join(A.fmt_time(t) + ('' if lp.clean else ' (off track)') for t, lp in zip(ts, mine))}. "
             f"Best {A.fmt_time(min(ts))}, average {A.fmt_time(avg)}, spread (sigma) {sd:.2f} s.")
    if ref is None:
        L.append("No other lap to compare with.")
        return "\n".join(L)
    L.append(f"Compared lap: {ref.driver}, lap {ref.num}, {A.fmt_time(ref.time)} "
             f"({'faster' if ref.time < you.time else 'slower'} by {abs(ref.time - you.time):.2f} s).")
    sy = A.sector_times(you, rep.laplen); sr = A.sector_times(ref, rep.laplen)
    L.append("Quarter-lap sectors (you vs compared, s): " +
             "; ".join(f"S{i + 1} {a:.2f} vs {b:.2f} ({a - b:+.2f})" for i, (a, b) in enumerate(zip(sy, sr))))
    dl = A.delta(ref, you)
    L.append("Corner by corner (distance from the line; speeds km/h; braking/throttle points in m; time = what you "
             "lost (+) or gained (-) over the stretch belonging to that corner -- from halfway after the previous corner to "
             "halfway to the next, so the corner times add up to the whole lap's difference):")
    n = len(you.dist); cs = corners(ref)
    bounds = [0] + [(p + q) // 2 for p, q in zip(cs, cs[1:])] + [n - 1]
    for i, k in enumerate(cs):
        a, b = bounds[i], bounds[i + 1]
        kb_y, kb_r = _brake_start(you, k), _brake_start(ref, k)
        ky = min(range(a, b + 1), key=lambda j: you.ch["kmh"][j])
        thr = _throttle_on(you, ky)
        name = f"{_section(rep, you.dist[k])} ({you.dist[k]:.0f} m)" if _section(rep, you.dist[k]) else f"corner at {you.dist[k]:.0f} m"
        fmt = lambda lp, j: f"{lp.dist[j]:.0f}" if j is not None else "n/a"
        L.append(f"- {name}: min speed {you.ch['kmh'][ky]:.0f} vs {ref.ch['kmh'][k]:.0f}; "
                 f"slowing starts {fmt(you, kb_y)} vs {fmt(ref, kb_r)}; "
                 + (f"gear {int(round(you.ch['gear'][ky]))}; " if "gear" in you.ch else "")
                 + (f"full-ish throttle from {you.dist[thr]:.0f}; " if thr is not None else "")
                 + f"time {dl[b] - dl[a]:+.2f} s.")
    worst = sorted(((dl[bounds[i + 1]] - dl[bounds[i]], k) for i, k in enumerate(cs)), reverse=True)
    worst = [x for x in worst if x[0] > 0.05][:3]
    if worst:
        L.append("Biggest losses: " + ", ".join(f"{_section(rep, you.dist[k]) or f'{you.dist[k]:.0f} m'} ({t:+.2f} s)" for t, k in worst))
    L += guide_block(rep, you)
    return "\n".join(L)

def guide_block(rep, you, n_advice=3, advice_chars=700):
    """TRACKGUIDE-1 (PO 2026-10-09: "for the julia racer post-race analysis, use Classic track guides"): the track guide's
    corners against your best lap, and the guide's own advice for the corners where you are slowest against it."""
    try:
        import guidetab
        rows, src = guidetab.guide_rows(rep, you)
    except Exception:
        return []
    if not rows:
        return []
    out = ["Track guide (a fast lap -- compare shape, not absolute speed). Per corner: "
           "guide entry/slowest km/h and time through it vs yours:"]
    f = lambda x, p=0: "n/a" if x is None else f"{x:.{p}f}"
    for e in rows:
        out.append(f"- {e['section'] or 'start'} / {e['title']}: entry {f(e['entry'])} vs {f(e['you_entry'])}, slowest "
                   f"{f(e['through'])} vs {f(e['you_min'])}, time {f(e['negotiate'], 2)} vs {f(e['you_time'], 2)} s")
    ratio = [(e['you_time'] / e['negotiate'], e) for e in rows if e.get('you_time') and e.get('negotiate')]
    for _r, e in sorted(ratio, key=lambda x: -x[0])[:n_advice]:
        out.append(f"Guide advice for {e['section'] or 'start'} / {e['title']} (your time {_r:.1f}x the guide's): "
                   + e['advice'][:advice_chars] + ("…" if len(e['advice']) > advice_chars else ""))
    return out

PROMPT = ("You are a racing driving coach for Julia Racer: a 1967 Grand Prix car (the summary names which: about 400 hp, no "
          "wings, narrow hard tyres) on "
          "the classic game circuits, with physics fitted to reference telemetry. Below is a numeric summary of one of "
          "the driver's sessions compared with a faster or reference lap. Give the three changes that would gain the most "
          "time, each tied to named corners and to the numbers (braking point, minimum speed, throttle application, "
          "line), and say what to try on the next lap. Then one sentence on consistency. Be concrete and brief "
          "(under 300 words), use Markdown, and do not invent data that is not in the summary. Where the summary quotes the "
          "track guide, use its advice for those corners (call it \"the track guide\").\n\n")

class CoachDialog(QDialog):
    """Shows what would be sent; on Send, runs `claude -p` (no tools, no saved session) and shows the answer."""

    def __init__(self, rep, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Coaching analysis (Claude)")
        self.resize(820, 720)
        v = QVBoxLayout(self)
        self.summary = build_summary(rep)
        info = QLabel("This summary of your session is what will be sent to Claude (through the Claude Code CLI on this "
                      "PC) when you press <b>Send</b>. Nothing is sent before that.")
        info.setWordWrap(True); v.addWidget(info)
        self.sent = QPlainTextEdit(self.summary); self.sent.setReadOnly(True); self.sent.setMaximumHeight(220)
        v.addWidget(self.sent)
        self.answer = QTextBrowser(); self.answer.setObjectName("guide"); v.addWidget(self.answer, 1)
        row = QHBoxLayout(); v.addLayout(row)
        self.status = QLabel(""); self.status.setObjectName("hint"); row.addWidget(self.status, 1)
        self.send_b = QPushButton("Send to Claude"); self.send_b.setObjectName("primary")
        self.send_b.clicked.connect(self.send); row.addWidget(self.send_b)
        close = QPushButton("Close"); close.clicked.connect(self.reject); row.addWidget(close)
        self.proc = None; self._out = b""
        self.timer = QTimer(self); self.timer.setSingleShot(True); self.timer.timeout.connect(self._timeout)
        if claude_cli() is None:
            self.send_b.setEnabled(False)
            self.status.setText("The Claude Code CLI (`claude`) is not installed on this PC, so this analysis is unavailable.")

    def command(self):
        return claude_cli(), ["-p", "--tools", "", "--no-session-persistence", "--output-format", "text"]

    def send(self):
        prog, args = self.command()
        if prog is None:
            return
        self.send_b.setEnabled(False); self.status.setText("Asking Claude… (usually under a minute)")
        self.answer.setPlainText("")
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        self.proc.readyReadStandardOutput.connect(lambda: setattr(self, "_out", self._out + bytes(self.proc.readAllStandardOutput())))
        self.proc.finished.connect(self._done)
        self.proc.start(prog, args)
        self.proc.write((PROMPT + self.summary + "\n").encode("utf-8")); self.proc.closeWriteChannel()
        self.timer.start(TIMEOUT_S * 1000)

    def _timeout(self):
        if self.proc is not None and self.proc.state() != QProcess.ProcessState.NotRunning:
            self.proc.kill(); self.status.setText(f"No answer within {TIMEOUT_S} s -- stopped.")
            self.send_b.setEnabled(True)

    def _done(self, code, _status):
        self.timer.stop()
        text = self._out.decode("utf-8", errors="replace").strip(); self._out = b""
        if code == 0 and text:
            self.answer.setMarkdown(text); self.status.setText("Done.")
        else:
            err = bytes(self.proc.readAllStandardError()).decode("utf-8", errors="replace").strip()
            self.answer.setPlainText(f"Claude did not answer (exit {code}). {err[-600:]}")
            self.status.setText("")
        self.send_b.setEnabled(True)
