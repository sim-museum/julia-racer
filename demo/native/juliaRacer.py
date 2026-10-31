#!/usr/bin/env python3
"""juliaMotor — PyQt6 front-end: launch the Classic Lotus 49 driving sim and calibrate
your controller.

Two devices are supported the same way: the Logitech Extreme 3D Pro joystick (as
before) and a Thrustmaster TX wheel with clutch/brake/accelerator pedals.  Both are
mapped by the same wizard — you hold each control and the GUI captures which axis (or
button) it is and its travel endpoints.

WHY A JULIA HELPER READS THE STICK: the game reads the controller through GLFW, and
`joystick.conf` records axis indices in GLFW's ordering.  Reading the device in Python
(pygame/evdev) would number the axes differently and produce a config that maps the
wrong axes in-game.  So this GUI streams the live state from `joyserver.jl` (GLFW) and
calibrates against exactly the indices the game uses.  The file it writes,
`joystick.conf`, is the same format `calibrate.jl`/`JoyCfg` already read.

Run:  python3 juliaRacer.py (from demo/native/, or anywhere — paths are resolved)
"""
import json
import os
import re
import shutil
import socket
import sys
import time

from PyQt6.QtCore import QProcess, QProcessEnvironment, QSettings, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QKeySequence, QPalette
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QFrame, QGridLayout, QGroupBox,
    QButtonGroup, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QProgressBar, QPushButton, QRadioButton,
    QPlainTextEdit, QSizePolicy, QSpinBox, QTabWidget, QVBoxLayout, QWidget,
)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from garage_tab import GarageTab, named_setups, BASE_NAMES, garage_dir, TRACK_TITLES, setup_tags, track_in_name
import garage as gm
import classic_setup
from guides_tab import GuidesTab
UI_DIR = os.path.join(HERE, "assets", "ui")
GUIDE = os.path.join(HERE, "docs", "HOW_TO_RACE.md")
CONF = os.path.join(HERE, "joystick.conf")
PROFILE_DIR = os.path.join(HERE, "joystick_profiles")
BUILTIN_DIR = os.path.join(HERE, "controller_profiles")

TRACK_KEYS = ["zandvoort", "skidpad", "nurburgring", "watglen", "monza", "spa"]

REF_LAP = {"zandvoort": 86.848, "nurburgring": 501.931, "watglen": 66.912,
           "monza": 90.202, "spa": 200.342, "skidpad": 30.0}

def timing_env(qenv):
    """LOADHANG-1 (2026-10-07): every launch logs its load stages with elapsed seconds, so a slow load can be read from
    last_sim_run.log afterwards (the Spa "hang" had no stamps and took a sprint to attribute). Costs nothing."""
    if not qenv.contains("JM_TIMING"):
        qenv.insert("JM_TIMING", "1")

def _read_track_times(fname):
    """Read a '<track>\\t<seconds>' file → {track: seconds}. Empty if absent."""
    out = {}
    try:
        with open(os.path.join(HERE, fname)) as f:
            for ln in f:
                sp = ln.strip().split("\t")
                if len(sp) == 2:
                    try:
                        out[sp[0]] = float(sp[1])
                    except ValueError:
                        pass
    except OSError:
        pass
    return out

def human_bests():
    """Per-track best lap (s) the sim banks whenever you improve."""
    return _read_track_times("human_best.txt")

def human_recents():
    """B (PO): per-track MOST-RECENT race AVERAGE lap (s) — the sim overwrites it each race."""
    return _read_track_times("human_recent.txt")

def preset_ai_pct(track_key):
    """B (PO): the AI-speed % that paces the fastest AI car at the driver's MOST RECENT race
    AVERAGE on this track (so the field matches how you actually race, not a one-off hot lap):
    % = Classicrank_ref / your_recent_average · 100.  Falls back to your best lap, then 50 % if
    you've never raced/lapped here.  Clamped to the spinbox range [30, 200].
    PO 2026-09-05 set the DEFAULT to 60% (was 85%). That default applies when there is no pace for
    the track; once you HAVE raced here the personalised value still wins, because that is what the
    preset was asked for in the first place."""
    ref = REF_LAP.get(track_key)

    def plausible(t):
        return t and t > 0 and (not ref or t >= 0.5 * ref)
    base = next((t for t in (human_recents().get(track_key), human_bests().get(track_key))
                 if plausible(t)), None)
    if not base or not ref:
        return 60
    return max(30, min(200, round(ref / base * 100)))

def find_julia():
    j = shutil.which("julia")
    if j:
        return j
    cand = os.path.expanduser("~/.juliaup/bin/julia")
    return cand if os.path.exists(cand) else "julia"

CONTROLS = [
    ("Throttle / brake", "W / S  (or your pedals)"),
    ("Steer", "A / D  (or your wheel)"),
    ("Shift up / down", "E / Q  (or the paddles)"),
    ("Clutch", "C  (or the clutch pedal)"),
    ("Automatic / manual gearbox", "G"),
    ("Change view (cockpit, chase, ...)", "V"),
    ("Hide / show the top band (full-screen 3D)", "F"),
    ("Restart the race", "R"),
    ("Mute the engine", "M"),
    ("Leave the session", "Esc"),
]

def show_controls(parent):
    """GUI-1: the key list as a table in a dialog (it was a run-on sentence under the Launch button)."""
    d = QDialog(parent); d.setWindowTitle("Controls"); v = QVBoxLayout(d)
    g = QGridLayout(); v.addLayout(g)
    for i, (what, keys) in enumerate(CONTROLS):
        g.addWidget(QLabel(what), i, 0); g.addWidget(QLabel(f"<b>{keys}</b>"), i, 1)
    note = QLabel("A calibrated wheel and pedals work directly (Controller tab). The first start compiles and loads "
                  "the game: the window can take a few minutes to appear.")
    note.setWordWrap(True); note.setObjectName("hint"); v.addWidget(note)
    ok = QPushButton("Close"); ok.clicked.connect(d.accept); v.addWidget(ok, 0, Qt.AlignmentFlag.AlignRight)
    d.exec()

def guide_dialog(parent):
    """DOC-RACE-1 (PO 2026-10-06: "create documentation about how to race"): docs/HOW_TO_RACE.md in a reader."""
    from PyQt6.QtWidgets import QTextBrowser
    d = QDialog(parent); d.setWindowTitle("How to race"); d.resize(820, 760); v = QVBoxLayout(d)
    t = QTextBrowser(); t.setOpenExternalLinks(True); t.setObjectName("guide")
    try:
        with open(GUIDE, encoding="utf-8") as f:
            t.setMarkdown(f.read())
    except OSError as e:
        t.setPlainText(f"The guide could not be opened ({e}).")
    v.addWidget(t)
    ok = QPushButton("Close"); ok.clicked.connect(d.accept); v.addWidget(ok, 0, Qt.AlignmentFlag.AlignRight)
    return d

WETNESS_CHOICES = (("Dry", "1"), ("Wet (reference \"very wet\")", "6"), ("Very wet (reference \"extremely wet\")", "7"))

_CLASSIC_TIPS = {}

def classic_tip(path):
    """SETUPPICK-1 S2: a Classic item's tooltip -- its numbers and what the translation changes to fit reference's garage.
    SETUPFILTER-1: every track's files are listed now (~180 for the Lotus), so each is worked out once per file version."""
    try:
        key = (path, os.path.getmtime(path))
    except OSError:
        key = (path, None)
    if key not in _CLASSIC_TIPS:
        r = classic_import(path, write=False)
        notes = r[2] if r else ["could not be read"]
        e = classic_setup.shipped_classic_entry(path)
        try:
            tip = e["tip"] if e is not None else classic_setup.classic_setup_summary(path)
        except Exception:
            tip = ""
        tip += ("\nAdjusted to reference's garage:\n  " + "\n  ".join(notes)) if notes else "\nImports exactly (nothing outside reference's ranges)."
        _CLASSIC_TIPS[key] = tip
    return _CLASSIC_TIPS[key]

def classic_import(path, write=True):
    """SETUPPICK-1: a Classic setup translated onto the reference default's knobs (classic_setup.translate, SETUPIO-1's rules),
    cached as <garage>/classic/<md5>.toml (a subfolder: not one of the saved setups) -> (toml path, name, notes) or None."""
    import hashlib
    e = classic_setup.shipped_classic_entry(path)
    if e is not None:
        return (path, "Classic " + e["stem"], e["notes"])
    try:
        with open(path, "rb") as fh:
            b = fh.read()
        d = os.path.join(garage_dir(), "classic")
        os.makedirs(d, exist_ok=True)
        out = os.path.join(d, hashlib.md5(b).hexdigest() + ".toml")
        g = classic_setup.decode_kj1(b)
        med = gm.load_toml().get("classic", {}).get("arb_median")
        K, notes = classic_setup.translate(g, gm.sessions()["default"]["knobs"], arb_median=tuple(med) if med else None)
        if write and not os.path.exists(out):
            gm.write_garage_file(out, K, base="default")
        return (out, "Classic " + os.path.splitext(os.path.basename(path))[0], notes)
    except Exception:
        return None

PLAYER_CARS = (("Lotus 49 (Clark)", "lotus"), ("Eagle T1G (Gurney)", "eagle"), ("Ferrari 312 (Amon)", "ferrari"),
               ("Brabham BT24 (Brabham)", "brabham"), ("Cooper T86 (Bonnier)", "cooper"), ("Honda RA300 (Surtees)", "honda"),
               ("BRM P115 (Stewart)", "brm"))

def segnames_env(qenv, on):
    """TRACKSEG-3: the track-section names are on by default; off = the sim's JM_SEGNAME_SECS=0."""
    if not on:
        qenv.insert("JM_SEGNAME_SECS", "0")

HUD_FLAGS = (
    ("hud/delta", "JM_HUD_DELTA", "Delta-to-last-lap bar (red = slower, green = faster)",
     "From your second lap: a bar in the middle of the top band. It grows left in red while this lap is slower than\n"
     "your last one at the same point of the track, right in green while it is faster. Not shown on the first lap."),
    ("hud/timing", "JM_HUD_TIMING", "Sector times and lap count",
     "Each sector's time and its gap to your best sector this session, the lap number, your race position and\n"
     "your optimal lap (the sum of your best sectors). Monza and Watkins Glen use reference's own sectors."),
    ("hud/trackmap", "JM_HUD_MAP", "Track map with every car (races)",
     "In a race: the circuit with a numbered disc per car at its race position; yours is the larger one with a\n"
     "white ring."),
)

def hud_env(qenv, settings):
    """HUDDELTA-1 / HUDTIMING-1 / HUDMAP-1: a readout switched off = its JM_HUD_* = 0 (the sim's default is on)."""
    for key, var, _, _ in HUD_FLAGS:
        if str(settings.value(key, "true")) != "true":
            qenv.insert(var, "0")

def spectators_env(qenv, on):
    """SPECTATORS-1: Classic's standing spectators, only where they stand well off the road (the sim's JM_SAFE_SPECTATORS=1).
    Off by default: the PO's 2026-08-27 order removes loose people, and stands until the PO turns this on."""
    if on:
        qenv.insert("JM_SAFE_SPECTATORS", "1")

class Ctrl:
    def __init__(self, axis=0, a=0.0, b=1.0):
        self.axis, self.a, self.b = int(axis), float(a), float(b)

    def norm(self, axes):
        if self.axis < 1 or self.axis > len(axes):
            return 0.0
        d = self.b - self.a
        return 0.0 if abs(d) < 1e-6 else (axes[self.axis - 1] - self.a) / d

class JoyMap:
    def __init__(self):

        self.steer = Ctrl(1, -1.0, 1.0)
        self.throttle = Ctrl(2, 0.0, -1.0)
        self.brake = Ctrl(2, 0.0, 1.0)
        self.clutch = Ctrl(0, 0.0, 1.0)
        self.up_btn, self.dn_btn, self.clutch_btn = 1, 2, 3
        self.fullscreen_btn = 0
        self.deadzone = 0.06
        self.wheel_half_deg = 0.0

    def apply(self, axes, btns):
        clamp = lambda v, lo, hi: max(lo, min(hi, v))
        s = clamp(1.0 - 2.0 * self.steer.norm(axes), -1.0, 1.0)
        if abs(s) < self.deadzone:
            s = 0.0
        thr = clamp(self.throttle.norm(axes), 0.0, 1.0)
        if thr < self.deadzone:
            thr = 0.0
        brk = clamp(self.brake.norm(axes), 0.0, 1.0)
        if brk < self.deadzone:
            brk = 0.0
        btn = lambda i: 1 <= i <= len(btns) and btns[i - 1] != 0
        if self.clutch.axis >= 1:
            clu = clamp(self.clutch.norm(axes), 0.0, 1.0)
        else:
            clu = 1.0 if btn(self.clutch_btn) else 0.0
        return s, thr, brk, clu, btn(self.up_btn), btn(self.dn_btn)

    def save(self, path, meta=None):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("# zand_racer joystick config — generated by juliaRacer.py\n")
            for k, v in (meta or {}).items():
                f.write(f"{k} {v}\n")
            for nm, c in (("steer", self.steer), ("throttle", self.throttle),
                          ("brake", self.brake), ("clutch", self.clutch)):
                f.write(f"{nm}.axis {c.axis}\n{nm}.a {c.a}\n{nm}.b {c.b}\n")
            f.write(f"up_btn {self.up_btn}\ndn_btn {self.dn_btn}\n")
            f.write(f"clutch_btn {self.clutch_btn}\ndeadzone {self.deadzone}\n")
            if self.fullscreen_btn > 0:
                f.write(f"fullscreen_btn {self.fullscreen_btn}\n")
            if self.wheel_half_deg > 0:
                f.write(f"wheel_half_deg {self.wheel_half_deg}\n")

    @classmethod
    def load(cls, path):
        m = cls()
        if not os.path.isfile(path):
            return m
        d = {}
        for ln in open(path):
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            p = ln.split()
            if len(p) >= 2:
                try:
                    d[p[0]] = float(p[1])
                except ValueError:
                    pass
        ctrl = lambda nm, df: Ctrl(round(d.get(nm + ".axis", df.axis)),
                                   d.get(nm + ".a", df.a), d.get(nm + ".b", df.b))
        m.steer = ctrl("steer", m.steer)
        m.throttle = ctrl("throttle", m.throttle)
        m.brake = ctrl("brake", m.brake)
        m.clutch = ctrl("clutch", m.clutch)
        m.up_btn = round(d.get("up_btn", m.up_btn))
        m.dn_btn = round(d.get("dn_btn", m.dn_btn))
        m.clutch_btn = round(d.get("clutch_btn", m.clutch_btn))
        m.fullscreen_btn = round(d.get("fullscreen_btn", 0))
        m.deadzone = d.get("deadzone", m.deadzone)
        m.wheel_half_deg = d.get("wheel_half_deg", 0.0)
        return m

def load_meta(path):
    """CTRLCAL-1: the text keys of a conf/profile file (label, match, mode, device), as JoyCfg.loadmeta reads them."""
    d = {}
    if not os.path.isfile(path):
        return d
    for ln in open(path):
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        p = ln.split(None, 1)
        v = p[1].strip() if len(p) > 1 else ""
        try:
            float(v)
        except ValueError:
            d[p[0]] = v
    return d

def device_file(name):
    """Where a device's own calibration is kept (JoyCfg.device_file: non-alphanumerics -> '_')."""
    return os.path.join(PROFILE_DIR, "".join(ch if ch.isalnum() else "_" for ch in name) + ".conf")

def builtin_profiles():
    """[(label, match, path)] of the shipped profiles (Thrustmaster TX, Logitech Extreme 3D Pro), by label."""
    out = []
    if os.path.isdir(BUILTIN_DIR):
        for f in sorted(os.listdir(BUILTIN_DIR)):
            if f.endswith(".conf"):
                p = os.path.join(BUILTIN_DIR, f); m = load_meta(p)
                out.append((m.get("label", f), m.get("match", ""), p))
    return sorted(out)

def saved_profiles():
    """Every profile the driver can pick: [(display label, path)] -- built-ins first, then each device calibration the
    wizard saved (joystick_profiles/)."""
    out = [(lab + "  (built-in)", p) for lab, _, p in builtin_profiles()]
    if os.path.isdir(PROFILE_DIR):
        for f in sorted(os.listdir(PROFILE_DIR)):
            if f.endswith(".conf"):
                p = os.path.join(PROFILE_DIR, f); m = load_meta(p)
                out.append((m.get("label", m.get("device", f[:-5].replace("_", " "))) + "  (your calibration)", p))
    return out

def profile_for(name):
    """(label, path) of the built-in profile whose every `match` word is in the device name, or None (JoyCfg.profile_for)."""
    n = name.lower()
    for lab, mt, p in builtin_profiles():
        w = mt.lower().split()
        if w and all(x in n for x in w):
            return lab, p
    return None

def _mapkey(m):
    c = lambda x: (x.axis, x.a, x.b)
    return (c(m.steer), c(m.throttle), c(m.brake), c(m.clutch), m.up_btn, m.dn_btn, m.clutch_btn, m.deadzone,
            m.wheel_half_deg)

def legacy_owner(conf):
    """The per-device file holding the same map as `conf` (where a pre-CTRLCAL-1 conf was made), or None."""
    if not os.path.isdir(PROFILE_DIR):
        return None
    k = _mapkey(JoyMap.load(conf))
    for f in sorted(os.listdir(PROFILE_DIR)):
        p = os.path.join(PROFILE_DIR, f)
        if f.endswith(".conf") and _mapkey(JoyMap.load(p)) == k:
            return p
    return None

def resolve(conf, name):
    """What the sim will use for device `name` -> (JoyMap, source text). Mirrors JoyCfg.resolve exactly (gated)."""
    meta = load_meta(conf); mode = meta.get("mode", "")
    if os.path.isfile(conf) and mode == "profile":
        return JoyMap.load(conf), f'saved profile "{meta.get("label", "?")}"' + (f' on "{name}"' if name else "")
    if os.path.isfile(conf) and mode != "autodetect":

        owner = legacy_owner(conf)
        if not name or owner is None or owner == device_file(name):
            return JoyMap.load(conf), "joystick.conf"
    dev = meta.get("device", "")
    if os.path.isfile(conf) and (not name or dev == name):
        return JoyMap.load(conf), f'autodetect: calibration of "{dev or "?"}"'
    if name and os.path.isfile(device_file(name)):
        return JoyMap.load(device_file(name)), f'autodetect: saved calibration of "{name}"'
    pr = profile_for(name)
    if pr:
        return JoyMap.load(pr[1]), f'autodetected {pr[0]} ("{name}")'
    x3d = [p for lab, _, p in builtin_profiles() if "extreme 3d" in lab.lower()]
    return (JoyMap.load(x3d[0]) if x3d else JoyMap(),
            "X3D default (no controller found)" if not name else
            f'X3D default -- UNKNOWN controller "{name}": calibrate it in the launcher')

class JoyReader(QProcess):
    updated = pyqtSignal()
    status = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.present = False
        self.name = ""
        self.axes = []
        self.buttons = []
        self._buf = b""
        self.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        self.readyReadStandardOutput.connect(self._read)

    def start_reader(self, slot=1):
        if self.state() != QProcess.ProcessState.NotRunning:
            return
        self.setWorkingDirectory(HERE)
        self.start(find_julia(), ["--project=.", "joyserver.jl", str(slot)])
        self.status.emit("starting joystick reader (GLFW)… first start compiles, ~10–20 s")

    def stop_reader(self):
        if self.state() != QProcess.ProcessState.NotRunning:
            self.kill()
            self.waitForFinished(1500)

    def _read(self):
        self._buf += bytes(self.readAllStandardOutput())
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"{"):
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            self.present = d.get("present", False)
            self.name = d.get("name", "")
            self.axes = [float(x) for x in d.get("axes", [])]
            self.buttons = [int(x) for x in d.get("buttons", [])]
            self.updated.emit()

class Bar(QProgressBar):
    def __init__(self):
        super().__init__()
        self.setRange(-1000, 1000)
        self.setValue(0)
        self.setTextVisible(True)
        self.setFormat("%v")
        self.setFixedHeight(18)

    def set_norm(self, frac, raw=None):
        self.setValue(int(max(-1.0, min(1.0, frac)) * 1000))
        self.setFormat(f"{raw:.2f}" if raw is not None else f"{frac:.2f}")

CENTRED_LO = 1.0 - 2 ** -0.5

class Sweep:
    """CTRLCAL-1 S2: calibration the way Classic (classic.exe FUN_004c2990/2d90/2ab0) and reference do it. `start` snapshots every
    axis at rest (Classic: centre = current value, min = max = it); `update`, every frame, grows each axis's min/max while the
    driver moves every control through its full travel. Each axis then classifies itself: rest within 29.3-70.7 % of its
    travel = CENTRED (steering, a stick's push/pull); nearer an end = a PEDAL resting there. An axis that moved less than
    10 % of a ±1 range is inactive (Classic: range < 0.2 x centre reads 0). 2 % of the travel at each end saturates."""
    MIN_RANGE = 0.2
    END = 0.02

    def __init__(self):
        self.centre, self.lo, self.hi = [], [], []

    def start(self, axes):
        self.centre = list(axes); self.lo = list(axes); self.hi = list(axes)

    def update(self, axes):
        if len(axes) != len(self.centre):
            return
        for i, v in enumerate(axes):
            if v < self.lo[i]:
                self.lo[i] = v
            if v > self.hi[i]:
                self.hi[i] = v

    def range(self, i):
        return self.hi[i] - self.lo[i]

    def kind(self, i):
        """'centred', 'pedal' or None (not moved enough)."""
        r = self.range(i)
        if r < self.MIN_RANGE:
            return None
        p = (self.centre[i] - self.lo[i]) / r
        if CENTRED_LO <= p <= 1.0 - CENTRED_LO:
            return "centred"

        c = self.centre[i]
        if abs(c) <= 0.25 and self.lo[i] < c - 0.1 and self.hi[i] > c + 0.1:
            return "centred"
        return "pedal"

    def travel(self, i, v):
        """How far value v is from rest toward the far end on its side, 0..1 (signed side via `side`)."""
        c = self.centre[i]
        end = self.hi[i] if v >= c else self.lo[i]
        return 0.0 if abs(end - c) < 1e-9 else (v - c) / (end - c)

    def rest_end(self, i):
        """A pedal's resting END: the swept end nearer its rest snapshot."""
        c = self.centre[i]
        return self.lo[i] if (c - self.lo[i]) < (self.hi[i] - c) else self.hi[i]

    def pull(self, i, v):
        """How far a control is from rest, 0..1. PO 2026-10-18 (4): "it says it's waiting for axis 3, even though I already
        moved the accelerator" -- a pedal was measured from its rest SNAPSHOT toward the swept end on that side, so a
        snapshot a little short of the end (0.9 against a true rest of 1.0) read a released pedal as 100 % pressed: the
        clutch then won the accelerator step, or (with the arming wait) held it for good. A pedal is measured from its
        resting END over its whole swept travel, so the snapshot's offset drops out; a centred axis as before."""
        if self.kind(i) == "pedal":
            r = self.range(i)
            return 0.0 if r < 1e-9 else abs(v - self.rest_end(i)) / r
        return abs(self.travel(i, v))

    def pedal_ctrl(self, i, v):
        """A Ctrl for a control pressed toward value v on axis i: from rest (0) to the swept end on v's side (1), with
        Classic's 2 % saturation at both ends. For a pedal the rest is an end; for a centred axis (a stick's push or pull)
        it is the centre."""
        c = self.centre[i]
        if self.kind(i) == "pedal":
            rest = self.rest_end(i)
            far = self.hi[i] if rest == self.lo[i] else self.lo[i]
        else:
            rest = c
            far = self.hi[i] if v >= c else self.lo[i]
        d = far - rest
        return rest + self.END * d, far - self.END * d

    def steer_ctrl(self, i, left_value):
        """(a, b) for steering on centred axis i: symmetric about the rest centre (so centre = straight ahead), a on the
        side the driver turned LEFT, half-width = the smaller swept half less 2 %."""
        c = self.centre[i]
        h = min(c - self.lo[i], self.hi[i] - c) * (1.0 - self.END)
        return (c - h, c + h) if left_value < c else (c + h, c - h)

STEPS = [
    ("idle", ""),
    ("rest", "Step 1 — CENTRE the wheel/stick and RELEASE every pedal, then click Next."),
    ("sweep", "Step 2 — Move EVERY control through its FULL travel: turn the wheel lock to lock (stick: every way), press "
              "each pedal to the floor and let it back, run any slider end to end. Then click Next."),
    ("steer", "Step 3 — Turn the wheel (or push the stick) to the LEFT: a quarter turn is plenty."),
    ("wheel90", "Step 4 — WHEEL: turn it exactly 90° LEFT (a quarter turn: the top of the rim at 9 o'clock) and click "
                "Capture. Joystick, or a wheel the game already knows: Skip."),
    ("throttle", "Step 5 — Press the ACCELERATOR fully (stick: push forward), then release."),
    ("brake", "Step 6 — Press the BRAKE fully (stick: pull back), then release."),
    ("clutch", "Step 7 — Press the CLUTCH pedal fully (or run the clutch slider), then release — "
               "or click 'Use button' / 'No clutch'."),
    ("clutch_btn", "Step 7b — Press the button you want to use as CLUTCH."),
    ("up", "Step 8 — Press the SHIFT-UP paddle/button."),
    ("dn", "Step 9 — Press the SHIFT-DOWN paddle/button."),
    ("fs", "Step 10 — Press the button that will switch FULL SCREEN on and off during a race — or Skip."),
    ("done", "Done — review the live preview below, then Save."),
]
STEP = {k: i for i, (k, _) in enumerate(STEPS)}
PRESS = 0.6
RELEASED = 0.2

STEER_PRESS = 0.15

PEDAL_PRESS = 0.25
REST_STILL = 0.05
ARM_FRAMES = 20

class CalibrateTab(QWidget):
    def __init__(self, joy: JoyReader, on_saved):
        super().__init__()
        self.joy = joy
        self.on_saved = on_saved
        self.map = JoyMap.load(CONF)
        self.work = JoyMap.load(CONF)
        self.rest = []
        self.sweep = Sweep()
        self._used = set()
        self.step = 0
        self._btn_armed = True
        self._btn_held = set()
        self._sax = 0
        self.axis_bars = []
        self.btn_lamps = []
        self._build()
        self.joy.updated.connect(self.refresh)
        self.joy.status.connect(lambda s: self.statusl.setText(s))

    def _build(self):
        root = QVBoxLayout(self)

        self.devl = QLabel("No controller — plug it in (it must be the first joystick).")
        self.devl.setStyleSheet("font-weight:bold")
        root.addWidget(self.devl)
        self.statusl = QLabel("")
        self.statusl.setStyleSheet("color:#888")
        root.addWidget(self.statusl)

        cal = QGroupBox("Calibration")
        cg = QGridLayout(cal)
        self.auto_r = QRadioButton("Autodetect")
        self.prof_r = QRadioButton("Saved profile:")
        self.mode_g = QButtonGroup(self)
        self.mode_g.addButton(self.auto_r); self.mode_g.addButton(self.prof_r)
        autol = QLabel("The game finds the controller you plug in and uses its own calibration -- as Classic and reference "
                       "do. A Thrustmaster TX or a Logitech Extreme 3D Pro works without one; any other controller: "
                       "run the wizard below once.")
        autol.setWordWrap(True); autol.setStyleSheet("color:#888")
        self.profile = QComboBox()
        self.use_b = QPushButton("Use this profile")
        self.use_b.clicked.connect(self.use_profile)
        cg.addWidget(self.auto_r, 0, 0); cg.addWidget(autol, 0, 1, 1, 2)
        cg.addWidget(self.prof_r, 1, 0); cg.addWidget(self.profile, 1, 1); cg.addWidget(self.use_b, 1, 2)
        self.usingl = QLabel("")
        self.usingl.setStyleSheet("font-weight:bold")
        self.usingl.setWordWrap(True)
        cg.addWidget(self.usingl, 2, 0, 1, 3)
        cg.setColumnStretch(1, 1)
        root.addWidget(cal)
        self._fill_profiles()
        meta = load_meta(CONF)
        (self.prof_r if meta.get("mode") == "profile" else self.auto_r).setChecked(True)
        if meta.get("mode") == "profile":
            for i in range(self.profile.count()):
                if self.profile.itemText(i).startswith(meta.get("label", "\0") + "  ("):
                    self.profile.setCurrentIndex(i)
        self.profile.setEnabled(self.prof_r.isChecked()); self.use_b.setEnabled(self.prof_r.isChecked())
        self.auto_r.toggled.connect(self._mode_toggled)
        self._show_using()

        mid = QHBoxLayout()
        root.addLayout(mid, 1)

        live = QGroupBox("Live input  (raw values from GLFW)")
        self.live_v = QVBoxLayout(live)
        self.axes_box = QVBoxLayout()
        self.live_v.addLayout(self.axes_box)
        self.live_v.addWidget(QLabel("Buttons:"))
        self.btn_grid = QGridLayout()
        self.live_v.addLayout(self.btn_grid)
        self.live_v.addStretch(1)
        mid.addWidget(live, 1)

        wiz = QGroupBox("Calibration wizard")
        wv = QVBoxLayout(wiz)
        self.instr = QLabel("Click Start to map the controller now plugged in.")
        self.instr.setWordWrap(True)
        self.instr.setMinimumHeight(60)
        wv.addWidget(self.instr)

        brow = QHBoxLayout()
        self.start_b = QPushButton("Start")
        self.start_b.clicked.connect(self.start_wizard)
        self.cap_b = QPushButton("Capture")
        self.cap_b.clicked.connect(self.capture)
        self.cap_b.setEnabled(False)
        self.alt_b = QPushButton("Use button")
        self.alt_b.clicked.connect(self.clutch_use_button)
        self.alt_b.hide()
        self.skip_b = QPushButton("No clutch")
        self.skip_b.clicked.connect(self.skip)
        self.skip_b.hide()
        for x in (self.start_b, self.cap_b, self.alt_b, self.skip_b):
            brow.addWidget(x)
        brow.addStretch(1)
        wv.addLayout(brow)

        self.capl = QLabel("")
        self.capl.setStyleSheet("color:#2a7")
        self.capl.setWordWrap(True)
        wv.addWidget(self.capl)
        wv.addStretch(1)
        mid.addWidget(wiz, 1)

        prev = QGroupBox("Mapped output  (what the game will see)")
        pg = QGridLayout(prev)
        self.pv = {}
        for i, nm in enumerate(("steer", "throttle", "brake", "clutch")):
            pg.addWidget(QLabel(nm.capitalize()), i, 0)
            bar = Bar(); self.pv[nm] = bar; pg.addWidget(bar, i, 1)
        self.shiftl = QLabel("shift: · ·")
        pg.addWidget(self.shiftl, 4, 0, 1, 2)
        root.addWidget(prev)

        srow = QHBoxLayout()
        self.save_b = QPushButton("Save → joystick.conf")
        self.save_b.clicked.connect(self.save)
        srow.addWidget(self.save_b)
        self.summl = QLabel(self._summary())
        self.summl.setStyleSheet("color:#888")
        srow.addWidget(self.summl, 1)
        root.addLayout(srow)

    def refresh(self):
        j = self.joy
        if not j.present:
            self.devl.setText("No controller detected on joystick #1 — plug it in.")
            return
        self.devl.setText(f"● {j.name}   ({len(j.axes)} axes, {len(j.buttons)} buttons)")
        if j.name != getattr(self, "_using_name", None):
            self._show_using()

        if len(self.axis_bars) != len(j.axes):
            while self.axes_box.count():
                it = self.axes_box.takeAt(0)
                if it.widget():
                    it.widget().deleteLater()
            self.axis_bars = []; self.axis_labels = []
            for i in range(len(j.axes)):
                row = QHBoxLayout()
                lab = QLabel(f"axis {i + 1}"); lab.setMinimumWidth(110); row.addWidget(lab)
                bar = Bar(); row.addWidget(bar, 1)
                self.axis_bars.append(bar); self.axis_labels.append(lab)
                w = QWidget(); w.setLayout(row)
                self.axes_box.addWidget(w)
        if len(self.btn_lamps) != len(j.buttons):
            while self.btn_grid.count():
                it = self.btn_grid.takeAt(0)
                if it.widget():
                    it.widget().deleteLater()
            self.btn_lamps = []
            for i in range(len(j.buttons)):
                lab = QLabel(str(i + 1))
                lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
                lab.setFixedSize(24, 20)
                lab.setFrameShape(QFrame.Shape.Box)
                self.btn_lamps.append(lab)
                self.btn_grid.addWidget(lab, i // 8, i % 8)
        for i, v in enumerate(j.axes):
            self.axis_bars[i].set_norm(v, v)
        for i, v in enumerate(j.buttons):
            self.btn_lamps[i].setStyleSheet(
                "background:#2a7;color:white" if v else "background:none")

        s, thr, brk, clu, up, dn = self.work.apply(j.axes, j.buttons)
        self.pv["steer"].set_norm(s, s)
        self.pv["throttle"].set_norm(thr, thr)
        self.pv["brake"].set_norm(brk, brk)
        self.pv["clutch"].set_norm(clu, clu)
        self.shiftl.setText(f"shift:  up {'▼' if up else '·'}   down {'▼' if dn else '·'}")

        key = STEPS[self.step][0]
        key not in ("idle", "done") and self._trace({"step": key, "axes": [round(x, 4) for x in j.axes],
                                                     "buttons": [i + 1 for i, v in enumerate(j.buttons) if v]})
        if key not in ("idle", "rest") and self.sweep.centre:
            self.sweep.update(j.axes)
            for i, lab in enumerate(getattr(self, "axis_labels", [])):
                k = self.sweep.kind(i) if i < len(self.sweep.centre) else None
                lab.setText(f"axis {i + 1}" + (f" · {k}" if k else ""))
        if key in ("up", "dn", "clutch_btn", "fs"):
            self._auto_button()
        elif key in ("steer", "throttle", "brake", "clutch"):
            self._auto_axis()

    def _fill_profiles(self):
        cur = self.profile.currentText()
        self.profile.clear()
        self._profiles = saved_profiles()
        for lab, _ in self._profiles:
            self.profile.addItem(lab)
        i = self.profile.findText(cur)
        if i >= 0:
            self.profile.setCurrentIndex(i)

    def _show_using(self):
        """Say what the game will use for the controller plugged in now (JoyCfg.resolve, mirrored by resolve())."""
        m, src = resolve(CONF, self.joy.name if self.joy.present else "")
        self._using_name = self.joy.name if self.joy.present else ""
        if self.step == 0:
            self.work = m
        self.usingl.setText("The game will use: " + src)
        if hasattr(self, "summl"):
            self.summl.setText(self._summary())

    def _mode_toggled(self, auto):
        self.profile.setEnabled(not auto); self.use_b.setEnabled(not auto)
        if auto:
            self.set_autodetect()

    def set_autodetect(self):
        """Autodetect: joystick.conf keeps the calibration it holds (if any) but becomes per-device -- the game uses it
        only on the device it was made on; any other device gets its own saved calibration or a built-in profile."""
        meta = load_meta(CONF)
        if os.path.isfile(CONF) and meta.get("mode") != "profile":
            m = JoyMap.load(CONF)
            meta.pop("mode", None); meta.pop("label", None)
            m.save(CONF, {"mode": "autodetect", **({"device": meta["device"]} if "device" in meta else {})})
        else:
            with open(CONF, "w") as f:
                f.write("# zand_racer joystick config — generated by juliaRacer.py\nmode autodetect\n")
        self._show_using()
        self.on_saved()

    def use_profile(self):
        """Saved profile: the chosen profile becomes joystick.conf and is used whatever device is plugged in."""
        i = self.profile.currentIndex()
        if i < 0 or i >= len(self._profiles):
            return
        lab, path = self._profiles[i]
        JoyMap.load(path).save(CONF, {"mode": "profile", "label": lab.split("  (")[0]})
        self.capl.setText(f"Profile \"{lab}\" is in use.")
        self._show_using()
        self.on_saved()

    def start_wizard(self):
        if not self.joy.present:
            QMessageBox.warning(self, "No controller",
                                "Plug in the controller (as joystick #1) and try again.")
            return
        self.work = JoyMap()
        self.sweep = Sweep(); self._used = set()

        try:
            self._tracef = open(os.path.join(HERE, "joystick_wizard_trace.jsonl"), "w")
            self._trace({"event": "start", "device": self.joy.name})
        except OSError:
            self._tracef = None
        self._goto("rest")

    def _goto(self, key):
        self.step = STEP[key]
        self._show_step()

    def _show_step(self):
        key, txt = STEPS[self.step]
        self.instr.setText(txt)
        self.cap_b.setText("Capture" if key == "wheel90" else "Next")
        self.cap_b.setVisible(key in ("rest", "sweep", "wheel90"))
        self.cap_b.setEnabled(key in ("rest", "sweep", "wheel90"))
        self.alt_b.setVisible(key == "clutch")
        self.skip_b.setVisible(key in ("clutch", "wheel90", "fs"))
        self.skip_b.setText("Skip" if key in ("wheel90", "fs") else "No clutch")
        self.start_b.setText("Restart" if key != "idle" else "Start")

        self._btn_armed = key not in ("up", "dn", "clutch_btn", "fs")
        self._btn_held = set()
        self._ax_armed = False; self._still = 0
        if key == "done":
            self.cap_b.setEnabled(False)
            self.capl.setText("Calibration captured. Review the preview, then Save.")
        self.summl.setText(self._summary())

    def capture(self):
        key = STEPS[self.step][0]
        ax = self.joy.axes
        if not ax:
            return
        if key == "rest":
            self.rest = list(ax)
            self.sweep.start(ax)
            self.capl.setText("Rest captured. Now the sweep.")
            self._goto("sweep")
        elif key == "sweep":
            act = [i for i in range(len(ax)) if self.sweep.kind(i)]
            if not act:
                self.capl.setText("No control has moved yet — move each one through its full travel.")
                return
            self.capl.setText("Swept: " + ", ".join(f"axis {i + 1} {self.sweep.kind(i)}" for i in act))
            self._goto("steer")
        elif key == "wheel90":
            i = self.work.steer.axis - 1
            d = abs(ax[i] - self.sweep.centre[i]) if 0 <= i < len(ax) else 0.0
            if d < 0.02:
                self.capl.setText("The wheel hasn't turned — turn it 90° left, or Skip.")
                return
            self.work.wheel_half_deg = round(90.0 / d, 1)
            self.capl.setText(f"Wheel range {2 * self.work.wheel_half_deg:.0f}° (lock to lock)")
            self._goto("throttle")

    def skip(self):
        if STEPS[self.step][0] == "fs":
            self.work.fullscreen_btn = 0
            self._goto("done")
        elif STEPS[self.step][0] == "wheel90":
            self.work.wheel_half_deg = 0.0
            self._goto("throttle")
        else:
            self.clutch_skip()

    def clutch_use_button(self):
        self.work.clutch = Ctrl(0, 0.0, 1.0)
        self._goto("clutch_btn")

    def clutch_skip(self):
        self.work.clutch = Ctrl(0, 0.0, 1.0)
        self.work.clutch_btn = 0
        self._goto("up")

    def _auto_axis(self):
        """Steer / throttle / brake / clutch: the control the driver uses now is found by its movement (the axis that
        moved most, past PRESS of its swept travel, on a side not assigned yet); the ranges come from the sweep."""
        key = STEPS[self.step][0]
        ax = self.joy.axes; sw = self.sweep
        if len(ax) != len(sw.centre):
            return
        tr = [sw.pull(i, v) if sw.kind(i) else 0.0 for i, v in enumerate(ax)]
        if not self._ax_armed:

            if key == "steer":
                watch = [i for i in range(len(ax)) if sw.kind(i) == "centred"]
            else:
                watch = [i for i in range(len(ax)) if i != self.work.steer.axis - 1]
            busy = [i for i in watch if tr[i] >= (REST_STILL if key == "steer" else RELEASED)]
            if not busy:
                self._still = getattr(self, "_still", 0) + 1
                if self._still >= ARM_FRAMES:
                    self._ax_armed = True; self._still = 0
                    self._trace({"event": "armed", "step": key})
                    self.capl.setText("Ready -- " + ("turn the wheel LEFT." if key == "steer" else f"press the {key.upper()}."))
            else:
                self._still = 0
                i = max(busy, key=lambda k: tr[k])
                self.capl.setText(f"Waiting for axis {i + 1} to come back to rest ({tr[i]:.0%} of its travel) -- "
                                  + ("centre the wheel." if key == "steer" else "release it."))
            return

        side = lambda i: ((1 if ax[i] >= sw.rest_end(i) else -1) if sw.kind(i) == "pedal" else (1 if ax[i] >= sw.centre[i] else -1))
        if key == "steer":
            cand = [i for i in range(len(ax)) if sw.kind(i) == "centred"]
            if not cand:
                self.capl.setText("No axis was swept as a wheel/stick (centred). Click Restart and, at step 2, turn the "
                                  "wheel all the way LEFT and all the way RIGHT.")
                return
        else:
            cand = [i for i in range(len(ax)) if i != self.work.steer.axis - 1 and (i, side(i)) not in self._used]
        need = STEER_PRESS if key == "steer" else PEDAL_PRESS
        if key == "steer":
            far = max(cand, key=lambda k: tr[k], default=None)
            if far is not None and tr[far] > 0.02:
                self.capl.setText(f"Turning: axis {far + 1} at {min(1.0, tr[far] / need):.0%} -- keep turning LEFT")
        cand = [i for i in cand if tr[i] >= need]
        if not cand:
            return
        i = max(cand, key=lambda k: tr[k])
        self._used.add((i, side(i)))
        self._trace({"event": "assign", "step": key, "axis": i + 1, "side": side(i), "tr": [round(x, 3) for x in tr]})
        if key == "steer":
            a, b = sw.steer_ctrl(i, ax[i])
            self.work.steer = Ctrl(i + 1, a, b)
            self.capl.setText(f"Steer = axis {i + 1} ({sw.kind(i)}), centre {sw.centre[i]:.2f}")
            self._goto("wheel90")
            return
        c = Ctrl(i + 1, *sw.pedal_ctrl(i, ax[i]))
        setattr(self.work, key, c)
        if key == "clutch":
            self.work.clutch_btn = 0
        self.capl.setText(f"{key.capitalize()} = axis {i + 1} ({sw.kind(i)}): {c.a:.2f} → {c.b:.2f}")
        self._goto({"throttle": "brake", "brake": "clutch", "clutch": "up"}[key])

    def _auto_button(self):
        bs = self.joy.buttons
        if not self._btn_armed:
            self._btn_held = {i for i, v in enumerate(bs) if v}
            self._btn_armed = True
            return
        self._btn_held -= {i for i, v in enumerate(bs) if not v}
        for i, v in enumerate(bs):
            if v and i not in self._btn_held:
                self._trace({"event": "button", "step": STEPS[self.step][0], "button": i + 1, "held": sorted(self._btn_held)})
                key = STEPS[self.step][0]
                if key == "clutch_btn":
                    self.work.clutch_btn = i + 1
                    self.capl.setText(f"Clutch button = {i + 1}")
                    self._goto("up")
                elif key == "up":
                    self.work.up_btn = i + 1
                    self.capl.setText(f"Shift-up button = {i + 1}")
                    self._goto("dn")
                elif key == "dn":
                    self.work.dn_btn = i + 1
                    self.capl.setText(f"Shift-down button = {i + 1}")
                    self._goto("fs")
                elif key == "fs":
                    self.work.fullscreen_btn = i + 1
                    self.capl.setText(f"Full-screen button = {i + 1}")
                    self._goto("done")
                return

    def _trace(self, d):
        f = getattr(self, "_tracef", None)
        if f is None:
            return
        try:
            d = dict(d); d.setdefault("t", round(time.monotonic(), 3)); f.write(json.dumps(d) + "\n")
            d.get("event") in ("assign", "button", "armed", "start", "save") and f.flush()
        except (OSError, ValueError):
            pass

    def _summary(self):
        m = self.work
        c = (f"steer ax{m.steer.axis}  thr ax{m.throttle.axis}  brk ax{m.brake.axis}  "
             f"clutch {'ax'+str(m.clutch.axis) if m.clutch.axis else ('btn'+str(m.clutch_btn) if m.clutch_btn else 'none')}  "
             f"up b{m.up_btn} dn b{m.dn_btn}" + (f"  full screen b{m.fullscreen_btn}" if m.fullscreen_btn else ""))
        return c

    def save(self):

        name = self.joy.name if self.joy.present else ""
        meta = {"mode": "autodetect"}
        if name:
            meta["device"] = name
            meta["label"] = name + " " + time.strftime("%Y-%m-%d")
            self.work.save(device_file(name), meta)
        self.work.save(CONF, meta)
        self._trace({"event": "save", "summary": self._summary()})
        self.map = JoyMap.load(CONF)
        self.auto_r.blockSignals(True); self.auto_r.setChecked(True); self.auto_r.blockSignals(False)
        self.profile.setEnabled(False); self.use_b.setEnabled(False)
        self._fill_profiles()
        self.capl.setText(f"Saved → {CONF}")
        self._show_using()
        QMessageBox.information(self, "Saved",
                                f"Wrote {CONF}.\nThe game will use it on next launch.")
        self.on_saved()

class DriveTab(QWidget):
    def __init__(self, joy: JoyReader, on_result=None):
        super().__init__()
        self.joy = joy
        self.proc = None
        self.on_result = on_result
        root = QVBoxLayout(self)

        self._settings = QSettings("juliaRacer", "launcher")
        st = self._settings
        cols = QHBoxLayout(); root.addLayout(cols)

        sess = QGroupBox("Session"); form = QGridLayout(sess); cols.addWidget(sess, 3)
        form.addWidget(QLabel("Track:"), 0, 0)
        self.track = QComboBox()
        self.track.addItems(["Zandvoort", "Skidpad (test area)", "Nürburgring", "Watkins Glen", "Monza", "Spa"])
        self.track.setCurrentIndex(min(max(int(st.value("session/track", 0)), 0), 5))
        form.addWidget(self.track, 0, 1, 1, 2)
        form.addWidget(QLabel("Session:"), 1, 0)
        self.mode = QComboBox()
        self.mode.addItems(["Practice", "Race"])
        self.mode.setToolTip("Practice: you alone on the track. Race: a standing start against the AI field.")
        self.mode.setCurrentIndex(min(max(int(st.value("session/mode", 0)), 0), 1))
        form.addWidget(self.mode, 1, 1, 1, 2)

        self.laps_l = QLabel("Laps:")
        form.addWidget(self.laps_l, 2, 0)
        self.laps = QSpinBox(); self.laps.setRange(1, 99); self.laps.setValue(int(st.value("session/laps", 3)))
        form.addWidget(self.laps, 2, 1)
        self.ai_l = QLabel("Opponents:")
        form.addWidget(self.ai_l, 3, 0)
        self.ai = QSpinBox(); self.ai.setRange(0, 6)

        _ai = int(st.value("session/ai", 6))
        if str(st.value("session/ai_v", "1")) != "2":
            _ai = 6 if _ai == 5 else _ai
            st.setValue("session/ai", _ai); st.setValue("session/ai_v", "2")
        self.ai.setValue(_ai)
        self.ai.setToolTip("Number of AI cars on the grid (up to 6: Eagle, Ferrari, Brabham, Cooper, Honda, BRM).")
        form.addWidget(self.ai, 3, 1)
        self.ai_pct_l = QLabel("Opponent pace:")
        form.addWidget(self.ai_pct_l, 4, 0)
        self.ai_pct = QSpinBox(); self.ai_pct.setRange(30, 200); self.ai_pct.setValue(60)
        self.ai_pct.setSuffix(" %")
        self.ai_pct.setToolTip("Field pace as a % of the track's Classicrank reference lap time: 100% = the "
                               "fastest AI car hits the Classicrank time for this circuit. Auto-preset when you "
                               "pick a track to Classicrank/your-best-lap·100 (so the fastest AI matches your "
                               "best lap); 50% if you've no recorded lap yet. Edit it freely.")
        form.addWidget(self.ai_pct, 4, 1)
        self.ai_pct_note = QLabel("")
        self.ai_pct_note.setObjectName("hint")
        self.ai_pct_note.setWordWrap(True)
        form.addWidget(self.ai_pct_note, 5, 1, 1, 2)
        self.qual = QCheckBox("Qualifying session first (your hot lap sets the grid)")
        self.qual.setChecked(str(st.value("session/qual", "false")) == "true")
        self.qual.setToolTip("Off (default): start on the grid, floor it to launch the field. On: a practice/qualifying session sets your grid slot first (press T to end it).")
        form.addWidget(self.qual, 6, 0, 1, 3)

        form.addWidget(QLabel("Track:"), 7, 0)
        self.wet = QComboBox()
        for label, lvl in WETNESS_CHOICES:
            self.wet.addItem(label, lvl)
        i = self.wet.findData(str(st.value("session/wetness", "1")))
        self.wet.setCurrentIndex(i if i >= 0 else 0)
        self.wet.setToolTip("Dry, or wet as in reference's rain at Monza (measured from your 25 October wet laps): the tyres\n"
                            "grip less, and less the faster you go -- about 85 % of dry at 70 km/h, half at 200 km/h.\n"
                            "The AI drives slower to match. The look stays dry for now. Not on the skidpad.")
        self.wet.currentIndexChanged.connect(lambda _: st.setValue("session/wetness", self.wet.currentData()))
        form.addWidget(self.wet, 7, 1, 1, 2)
        self.track.currentIndexChanged.connect(lambda i: self.wet.setEnabled(i != 1))
        self.wet.setEnabled(self.track.currentIndex() != 1)
        form.setColumnStretch(1, 1)
        form.setRowStretch(8, 1)

        car = QGroupBox("Car"); cform = QGridLayout(car); cols.addWidget(car, 2)

        cform.addWidget(QLabel("Drive:"), 0, 0)
        self.playercar = QComboBox()
        for label, key in PLAYER_CARS:
            self.playercar.addItem(label, key)
        i = self.playercar.findData(str(st.value("car/player", "lotus")))
        self.playercar.setCurrentIndex(i if i >= 0 else 0)
        self.playercar.setToolTip("The car you drive -- its body, wheels, cockpit and physics. The AI field is the six others.\n"
                                  "Each car is the reference-fitted Lotus 49 model with its own mass, wheelbase, tracks, engine\n"
                                  "power and rev band, and gearing (the real 1967 cars' published figures, doc/CARSPECS_1967.md).")
        self.playercar.currentIndexChanged.connect(lambda _: st.setValue("car/player", self.playercar.currentData()))
        self.playercar.currentIndexChanged.connect(lambda _: self.reload_setups() if hasattr(self, "carsetup") else None)
        cform.addWidget(self.playercar, 0, 1)

        cform.addWidget(QLabel("Setup:"), 1, 0)
        self.carsetup = QComboBox()

        self.reload_setups(select=st.value("car/setup_choice", None), fallback=int(st.value("car/setup", 0)))
        self.carsetup.setToolTip("reference default: the Lotus 49 setup Julia has always raced.\n"
                                 "WW103 fast loose: Wolfgang Wagner's Classic Watkins Glen setup, driven in reference "
                                 "(session 261005) -- soft front springs, stiff rear bar, 35° drive ramp (the diff "
                                 "locks under power), front toe-out, long gearing. Loose on power, controllable at speed.\n"
                                 "Classic: …  every Classic setup on this PC for this car and track (~/sgl/THU/INSTALL), "
                                 "translated to reference's knobs as the Garage's Classic import does (hover one for its "
                                 "numbers).")
        self.carsetup.currentIndexChanged.connect(self._remember_setup)
        cform.addWidget(self.carsetup, 1, 1)
        cform.addWidget(QLabel("Gearbox:"), 2, 0)
        self.gearbox = QComboBox()
        self.gearbox.addItems(["Automatic", "Manual (clutch + shift)"])
        self.gearbox.setToolTip("Automatic shifts up/down by speed and needs no clutch; Manual = work the clutch "
                                "(C) and shift (E/Q or the paddles). G toggles in-game.")
        self.gearbox.setCurrentIndex(min(max(int(st.value("car/gearbox", 0)), 0), 1))
        cform.addWidget(self.gearbox, 2, 1)

        cform.addWidget(QLabel("Fuel:"), 3, 0)
        self.fuel = QComboBox()
        self.fuel.addItems(["Race distance + 2 laps", "Full tank (179.5 L)"])
        self.fuel.setToolTip("Race distance + 2 laps: enough for the race's laps (practice: 10) plus two, as the "
                             "reference burn measured on the Lotus 49.\nFull tank: 179.5 L, about 100 kg more than a "
                             "short race -- the car is heavier, softer and slower until it burns off.")
        self.fuel.setCurrentIndex(min(max(int(st.value("car/fuel", 0)), 0), 1))
        self.fuel.currentIndexChanged.connect(lambda i: QSettings("juliaRacer", "launcher").setValue("car/fuel", i))
        cform.addWidget(self.fuel, 3, 1)
        cform.setColumnStretch(1, 1)
        cform.setRowStretch(3, 1)

        self.mute = QCheckBox("Mute the engine sound")
        self.mute.setToolTip("JM_NOSOUND")

        self.noffb = QCheckBox("Turn force feedback off")
        self.noffb.setToolTip("JM_NOFFB")
        self.ibt = QCheckBox("Record reference-format telemetry (.ibt) to data/juliaracer/")
        self.replay = QCheckBox("Record a replay of every session (all cars) to data/juliaracer/")
        self.replay.setToolTip("Saves a .jmr recording of every car each session -- watch it from the Replays tab.")
        self.d2 = QCheckBox("Simplified 2-D physics (no jumps, lighter on slow PCs)")
        self.d2.setToolTip("JM_2D -- the full 3-D model is the default")
        for w, key, dflt in ((self.mute, "pref/mute", "false"), (self.noffb, "pref/noffb", "false"),
                             (self.ibt, "pref/ibt", "true"), (self.replay, "pref/replay", "true"),
                             (self.d2, "pref/d2", "false")):
            w.setChecked(str(st.value(key, dflt)) == "true")
            w.toggled.connect(lambda on, k=key: QSettings("juliaRacer", "launcher").setValue(k, "true" if on else "false"))
        self._gfx_group = self._build_gfx_group()

        root.addWidget(self._build_net_group())

        self.mode.currentIndexChanged.connect(self._mode_changed)
        self._mode_changed(self.mode.currentIndex())

        self.track.currentIndexChanged.connect(self._track_changed)
        self.track.currentIndexChanged.connect(lambda _: self.reload_setups())
        self._track_changed(self.track.currentIndex())

        brow = QHBoxLayout()
        self.launch_b = QPushButton("Start")
        self.launch_b.setObjectName("primary")
        self.launch_b.setMinimumHeight(44); self.launch_b.setMinimumWidth(220)
        self.launch_b.setDefault(True)
        self.launch_b.clicked.connect(self.launch)
        self.stop_b = QPushButton("Stop")
        self.stop_b.setMinimumHeight(44)
        self.stop_b.setToolTip("End the session (Esc in the game window does the same)")
        self.stop_b.clicked.connect(self.stop)
        self.stop_b.setEnabled(False)
        self.keys_b = QPushButton("Controls…")
        self.keys_b.setToolTip("The keyboard and wheel controls")
        self.keys_b.clicked.connect(lambda: show_controls(self))
        self.log_b = QPushButton("Show log")
        self.log_b.setCheckable(True)
        self.log_b.toggled.connect(self._toggle_log)
        brow.addWidget(self.launch_b)
        brow.addWidget(self.stop_b)
        brow.addStretch(1)
        brow.addWidget(self.keys_b)
        brow.addWidget(self.log_b)
        root.addLayout(brow)
        self._mode_label()

        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setTextVisible(True)
        self.progress.setMinimumHeight(22)
        root.addWidget(self.progress)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setObjectName("log")
        self.log.setVisible(False)
        root.addWidget(self.log, 1)
        self._spacer = QWidget(); self._spacer.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        root.addWidget(self._spacer, 1)

    def settings_page(self):
        """GUI-1: the rarely changed preferences, on their own tab (Main adds it)."""
        page = QWidget(); v = QVBoxLayout(page)
        v.addWidget(self._gfx_group)
        snd = QGroupBox("Sound and force feedback"); sv = QVBoxLayout(snd)
        sv.addWidget(self.mute); sv.addWidget(self.noffb); v.addWidget(snd)
        rec = QGroupBox("Recording"); rv = QVBoxLayout(rec)
        rv.addWidget(self.replay); rv.addWidget(self.ibt); v.addWidget(rec)
        adv = QGroupBox("Advanced"); av = QVBoxLayout(adv)
        av.addWidget(self.d2); v.addWidget(adv)
        v.addStretch(1)
        return page

    def _toggle_log(self, on):
        self.log.setVisible(on); self._spacer.setVisible(not on)
        self.log_b.setText("Hide log" if on else "Show log")

    def _mode_label(self):
        race = self.mode.currentIndex() == 1
        self.launch_b.setText("Start race" if race else "Start practice")

    def _status(self, msg):
        w = self.window()
        if isinstance(w, QMainWindow) and w.statusBar() is not None:
            w.statusBar().showMessage(msg)

    PRECOMPILE_EXPECTED = 318

    PRECOMPILE_CURVE = [0.011, 0.026, 0.053, 0.113, 0.192, 0.379, 0.660, 0.941, 0.954, 1.0]

    @classmethod
    def _precompile_work_fraction(cls, n):
        """Fraction of total compile WORK done after n packages (piecewise-linear on the curve)."""
        x = max(0.0, min(1.0, n / float(cls.PRECOMPILE_EXPECTED))) * 10.0
        i = int(x)
        if i >= 10:
            return 1.0
        lo = cls.PRECOMPILE_CURVE[i - 1] if i > 0 else 0.0
        return lo + (cls.PRECOMPILE_CURVE[i] - lo) * (x - i)

    LOAD_STAGES = [
        ("loading Classic", 20, "loading track…"),
        ("extracting geometry", 45, "extracting geometry…"),
        ("loading textures", 62, "decoding textures…"),
        ("placed", 85, "placing scenery…"),
        ("Drive:", 100, "ready — window opening"),
    ]

    NET_PORT_DEFAULT = 47700
    LOBBY_ASK = b"JRLOBBY?"

    def _build_gfx_group(self):
        if not hasattr(self, "_settings"):
            self._settings = QSettings("juliaRacer", "launcher")
        g = QGroupBox("Graphics")
        lay = QGridLayout(g)
        lay.addWidget(QLabel("Resolution:"), 0, 0)
        self.gfx_res = QComboBox()
        native = None
        scr = QApplication.primaryScreen()
        if scr is not None:
            sz = scr.size(); dpr = scr.devicePixelRatio()
            native = (int(round(sz.width() * dpr)), int(round(sz.height() * dpr)))
        self._res_values = []
        if native:
            self._res_values.append("native")
            self.gfx_res.addItem(f"Native ({native[0]} x {native[1]})")
        for w, h in ((3840, 2160), (2560, 1440), (1920, 1080), (1600, 900), (1440, 810)):
            if native and (w, h) == native:
                continue
            self._res_values.append(f"{w}x{h}")
            self.gfx_res.addItem(f"{w} x {h}" + ("  (old default)" if (w, h) == (1440, 810) else ""))
        saved = str(self._settings.value("gfx/res", self._res_values[0]))
        self.gfx_res.setCurrentIndex(self._res_values.index(saved) if saved in self._res_values else 0)
        lay.addWidget(self.gfx_res, 0, 1)
        self.gfx_full = QCheckBox("Full screen")
        self.gfx_full.setChecked(str(self._settings.value("gfx/full", "true")) == "true")
        self.gfx_full.setToolTip("Full screen on the primary monitor at the chosen resolution (Native = the monitor's own mode).")
        lay.addWidget(self.gfx_full, 0, 2)
        lay.addWidget(QLabel("Anti-aliasing:"), 1, 0)
        self.gfx_msaa = QComboBox(); self._msaa_values = ["8", "4", "2", "0"]
        self.gfx_msaa.addItems(["8x MSAA (smoothest)", "4x MSAA", "2x MSAA", "Off (fastest)"])
        sm = str(self._settings.value("gfx/msaa", "8"))
        self.gfx_msaa.setCurrentIndex(self._msaa_values.index(sm) if sm in self._msaa_values else 0)
        lay.addWidget(self.gfx_msaa, 1, 1)

        self.gfx_segnames = QCheckBox("Show track section names (e.g. \"Front Straight\")")
        self.gfx_segnames.setChecked(str(self._settings.value("hud/segnames", "true")) == "true")
        self.gfx_segnames.setToolTip("Shows each section's name above the 3-D view for a few seconds as you enter it.")
        lay.addWidget(self.gfx_segnames, 2, 0, 1, 3)
        self.gfx_hud = {}
        for row, (key, var, text, tip) in enumerate(HUD_FLAGS, start=3):
            cb = QCheckBox(text); cb.setChecked(str(self._settings.value(key, "true")) == "true"); cb.setToolTip(tip)
            lay.addWidget(cb, row, 0, 1, 3); self.gfx_hud[key] = cb

        self.gfx_spect = QCheckBox("Spectators, where they stand well off the road (as in Classic)")
        self.gfx_spect.setChecked(str(self._settings.value("scene/spectators", "false")) == "true")
        self.gfx_spect.setToolTip("Classic's standing spectators, kept only where they are more than 4 m beyond the racing\n"
                                  "surface and on the ground. Off: none at all (the rule since 27 August).")
        lay.addWidget(self.gfx_spect, 3 + len(HUD_FLAGS), 0, 1, 3)
        return g

    def _gfx_env(self, qenv):
        res = self._res_values[self.gfx_res.currentIndex()]
        full = self.gfx_full.isChecked(); msaa = self._msaa_values[self.gfx_msaa.currentIndex()]
        self._settings.setValue("gfx/res", res); self._settings.setValue("gfx/full", "true" if full else "false")
        self._settings.setValue("gfx/msaa", msaa)
        qenv.insert("JM_RES", res); qenv.insert("JM_MSAA", msaa)
        if full:
            qenv.insert("JM_FULLSCREEN", "1")
        segnames = self.gfx_segnames.isChecked()
        self._settings.setValue("hud/segnames", "true" if segnames else "false")
        segnames_env(qenv, segnames)
        for key, cb in self.gfx_hud.items():
            self._settings.setValue(key, "true" if cb.isChecked() else "false")
        hud_env(qenv, self._settings)
        spect = self.gfx_spect.isChecked()
        self._settings.setValue("scene/spectators", "true" if spect else "false")
        spectators_env(qenv, spect)

    def _build_net_group(self):
        if not hasattr(self, "_settings"):
            self._settings = QSettings("juliaRacer", "launcher")
        g = QGroupBox("Multiplayer (LAN)")
        lay = QGridLayout(g)
        lay.addWidget(QLabel("Players:"), 0, 0)
        self.net_mode = QComboBox()
        self.net_mode.addItems(["Single player", "Host a LAN race", "Join a LAN race"])
        self.net_mode.setToolTip("Host: this PC runs the AI field and sends it to the other player.\n"
                                 "Join: connect to a host on your network; its AI cars are drawn here.")
        lay.addWidget(self.net_mode, 0, 1, 1, 2)
        self.net_host_l = QLabel("Host address:")
        lay.addWidget(self.net_host_l, 1, 0)
        self.net_host = QLineEdit(str(self._settings.value("net/host", "")))
        self.net_host.setPlaceholderText("e.g. 192.168.1.20 (shown on the host's launcher)")
        lay.addWidget(self.net_host, 1, 1)
        self.net_fetch = QPushButton("Get host's settings")
        self.net_fetch.setToolTip("Ask the host's launcher for its track, mode, laps and AI cars and use them here.")
        self.net_fetch.clicked.connect(self._lobby_fetch)
        lay.addWidget(self.net_fetch, 1, 2)
        self.net_port_l = QLabel("Port (UDP):")
        lay.addWidget(self.net_port_l, 2, 0)
        self.net_port = QSpinBox(); self.net_port.setRange(1024, 65534)
        self.net_port.setValue(int(self._settings.value("net/port", self.NET_PORT_DEFAULT)))
        self.net_port.setToolTip("The race uses this UDP port and the launcher lobby the next one; "
                                 "allow both through a firewall on the host.")
        lay.addWidget(self.net_port, 2, 1)

        self.net_pw_l = QLabel("Password:")
        lay.addWidget(self.net_pw_l, 3, 0)
        self.net_pw = QLineEdit("")
        self.net_pw.setEchoMode(QLineEdit.EchoMode.Password)
        self.net_pw.setPlaceholderText("optional -- blank = open race")
        self.net_pw.setToolTip("Host: racers must type this password to join; leave it blank for an open race.\n"
                               "Join: the host's password, if the race has one.")
        lay.addWidget(self.net_pw, 3, 1)
        self.net_info = QLabel("")
        self.net_info.setWordWrap(True)
        self.net_info.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self.net_info, 4, 0, 1, 3)
        self._lobby_sock = None
        self._lobby_timer = QTimer(self)
        self._lobby_timer.timeout.connect(self._lobby_poll)
        self.net_mode.setCurrentIndex(int(self._settings.value("net/mode", 0)))
        self.net_mode.currentIndexChanged.connect(self._net_changed)
        self.net_port.valueChanged.connect(lambda _v: self._net_changed(self.net_mode.currentIndex()))
        self._net_changed(self.net_mode.currentIndex())
        return g

    @staticmethod
    def _lan_addresses():
        """This PC's LAN address(es) for the host to read out. A UDP connect sends nothing; it only
        asks the kernel which interface would route, which is the address a peer must use."""
        out = []
        for probe in ("192.168.255.255", "10.255.255.255", "172.31.255.255"):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as t:
                    t.connect((probe, 9)); a = t.getsockname()[0]
                if a and not a.startswith("127.") and a not in out:
                    out.append(a)
            except OSError:
                pass
        return out

    def _lobby_close(self):
        self._lobby_timer.stop()
        if self._lobby_sock is not None:
            self._lobby_sock.close(); self._lobby_sock = None

    def _net_changed(self, idx):
        join, host = idx == 2, idx == 1
        for w in (self.net_host_l, self.net_host, self.net_fetch):
            w.setVisible(join)
        for w in (self.net_port_l, self.net_port, self.net_info,
                  self.net_pw_l, self.net_pw):
            w.setVisible(idx != 0)
        self._lobby_close()
        port = self.net_port.value()
        if host:
            ips = self._lan_addresses()
            txt = ("Tell the other player to choose <b>Join a LAN race</b> with address <b>"
                   + (" or ".join(ips) if ips else "this PC's LAN IP") + f"</b> and port <b>{port}</b>. ")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("", port + 1)); s.setblocking(False)
                self._lobby_sock = s; self._lobby_timer.start(200)
                txt += "Their launcher can copy your track and race settings with <i>Get host's settings</i>."
            except OSError as e:
                txt += f"<span style='color:#c80'>(lobby port {port+1} unavailable: {e}; set the same track, mode, laps and AI cars on both PCs by hand)</span>"
            txt += " Launch first: the host's AI field starts with the host."
            self.net_info.setText(txt)
        elif join:
            self.net_info.setText("Use the <b>same track, mode, laps and AI cars</b> as the host "
                                  "(<i>Get host's settings</i> does that). The host steps the AI field; "
                                  "its cars are drawn here.")
        else:
            self.net_info.setText("")

    def _lobby_state(self):
        return {"jr": 1, "track": self.track.currentIndex(), "mode": self.mode.currentIndex(),
                "laps": self.laps.value(), "ai": self.ai.value(), "ai_pct": self.ai_pct.value(),
                "port": self.net_port.value()}

    def _lobby_poll(self):
        """Host side: answer every lobby query waiting on the socket (non-blocking, 5 Hz)."""
        s = self._lobby_sock
        while s is not None:
            try:
                data, addr = s.recvfrom(256)
            except (BlockingIOError, OSError):
                return
            if data.strip() == self.LOBBY_ASK:
                try:
                    s.sendto(json.dumps(self._lobby_state()).encode(), addr)
                    self.log.appendPlainText(f"[lobby] sent race settings to {addr[0]}")
                except OSError:
                    pass

    def _lobby_apply(self, st):
        """Join side: adopt the host's settings. Returns a one-line summary."""
        names = [self.track.itemText(i) for i in range(self.track.count())]
        self.mode.setCurrentIndex(int(st["mode"]))
        self.track.setCurrentIndex(int(st["track"]))
        self.laps.setValue(int(st["laps"])); self.ai.setValue(int(st["ai"]))
        self.ai_pct.setValue(int(st["ai_pct"]))
        return (f"{names[int(st['track'])]}, {self.mode.itemText(int(st['mode']))}"
                + (f", {st['laps']} laps, {st['ai']} AI" if int(st["mode"]) == 1 else ""))

    def _lobby_fetch(self):
        host = self.net_host.text().strip()
        if not host:
            self.net_info.setText("<span style='color:#c80'>Enter the host's address first.</span>"); return
        port = self.net_port.value() + 1
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.settimeout(0.6)
                for _ in range(3):
                    try:
                        s.sendto(self.LOBBY_ASK, (host, port))
                        data, _a = s.recvfrom(1024)
                        st = json.loads(data.decode())
                        if st.get("jr") == 1:
                            break
                    except socket.timeout:
                        st = None
                else:
                    st = None
        except (OSError, ValueError) as e:
            self.net_info.setText(f"<span style='color:#c80'>Could not reach {host}: {e}</span>"); return
        if not st:
            self.net_info.setText(f"<span style='color:#c80'>No answer from {host}:{port}. Is its launcher "
                                  "open with <b>Host a LAN race</b> selected, and is the port allowed?</span>"); return
        if int(st.get("port", self.net_port.value())) != self.net_port.value():
            self.net_port.setValue(int(st["port"]))
        self.net_info.setText("Host's settings applied: <b>" + self._lobby_apply(st) + "</b>. Launch when ready.")

    def _net_env(self, qenv):
        """Set JM_NET* for the sim (launch() has already refused a Join with no address)."""
        idx = self.net_mode.currentIndex()
        self._settings.setValue("net/mode", idx)
        self._settings.setValue("net/port", self.net_port.value())
        self._settings.setValue("net/host", self.net_host.text().strip())
        if idx == 0:
            return
        qenv.insert("JM_NET_PORT", str(self.net_port.value()))
        if self.net_pw.text().strip():
            qenv.insert("JM_NET_PASSWORD", self.net_pw.text().strip())
        if idx == 1:
            qenv.insert("JM_NET", "host")
        else:
            qenv.insert("JM_NET", "join")
            qenv.insert("JM_NET_HOST", self.net_host.text().strip())

    def _mode_changed(self, idx):
        """Race (idx 1) needs opponents and can't run on the skidpad; Practice is a lone car
        (no Laps / AI cars / AI speed — those rows are hidden)."""
        is_race = (idx == 1)
        for w in (self.laps_l, self.laps, self.ai_l, self.ai, self.ai_pct_l, self.ai_pct, self.ai_pct_note, self.qual):
            w.setVisible(is_race)
        if hasattr(self, "launch_b"):
            self._mode_label()
        skid = self.track.model().item(1)
        if skid is not None:
            skid.setEnabled(not is_race)
        if is_race:
            if self.track.currentIndex() == 1:
                self.track.setCurrentIndex(0)
            if self.ai.value() == 0:
                self.ai.setValue(6)

    def _track_changed(self, idx):
        """B: pre-set AI-speed % to Classicrank / your-most-recent-race-average · 100 (best lap, then 50%, as fallback)."""
        key = TRACK_KEYS[idx] if 0 <= idx < len(TRACK_KEYS) else "zandvoort"
        self.ai_pct.setValue(preset_ai_pct(key))

        ref = REF_LAP.get(key)
        ok = lambda t: bool(t) and t > 0 and (not ref or t >= 0.5 * ref)
        recent = human_recents().get(key); recent = recent if ok(recent) else None
        best = human_bests().get(key);     best = best if ok(best) else None
        if recent:
            self.ai_pct_note.setText(f"≈ your recent avg {self._fmt(recent)} vs Classicrank {self._fmt(REF_LAP.get(key,0))}")
        elif best:
            self.ai_pct_note.setText(f"≈ your best {self._fmt(best)} vs Classicrank {self._fmt(REF_LAP.get(key,0))} (race for an avg)")
        else:
            self.ai_pct_note.setText("no recorded lap yet → 50%")

    @staticmethod
    def _fmt(sec):
        m, s = divmod(sec, 60)
        return f"{int(m)}:{s:06.3f}"

    def reload_setups(self, select=None, fallback=0):
        """Fill the Setup list: the two base setups, then the saved ones ('<name> (on <base>)'). select: a choice key
        ('default', 'ww103', 'saved:<name>') or a saved setup's bare name; else keep the current choice."""
        cur = select if select is not None else (self.carsetup.currentData() if self.carsetup.count() else None)
        if isinstance(cur, str) and cur not in ("default", "ww103") and not cur.startswith("saved:"):
            cur = "saved:" + cur
        self.carsetup.blockSignals(True)
        self.carsetup.clear()

        car = (self.playercar.currentData() if hasattr(self, "playercar") else None) or "lotus"
        ti = self.track.currentIndex() if hasattr(self, "track") else -1
        tk = TRACK_KEYS[ti] if 0 <= ti < len(TRACK_KEYS) else None
        self.carsetup.addItem("reference default" if car == "lotus" else "Default setup", "default")
        if car == "lotus":
            self.carsetup.addItem("WW103 fast loose (Wolfgang Wagner, Watkins Glen)", "ww103")
        saved = [(n, setup_tags(p, n)[1], b) for n, p, b in named_setups(car)]
        for name, t, base in sorted(saved, key=lambda e: (e[1] != tk, e[0].lower())):
            say = "" if t is None or track_in_name(name) is not None else f", {TRACK_TITLES.get(t, t)}"
            self.carsetup.addItem(f"{name}  (on {BASE_NAMES[base]}{say})", "saved:" + name)

        gs = classic_setup.find_classic_setups_all(car, first=tk)
        if gs:
            self.carsetup.insertSeparator(self.carsetup.count())
            for stem, path, mod, gt in gs:
                self.carsetup.addItem(f"Classic: {stem}  [{mod}, {TRACK_TITLES.get(gt, gt)}]", "classic:" + path)
                self.carsetup.setItemData(self.carsetup.count() - 1, f"{path}\n{classic_tip(path)}", Qt.ItemDataRole.ToolTipRole)
        k = self.carsetup.findData(cur) if cur is not None else -1
        if k < 0:
            k = max(self.carsetup.findData("ww103" if int(fallback) == 1 else "default"), 0)
        self.carsetup.setCurrentIndex(k)
        self.carsetup.blockSignals(False)

        if (select is not None or isinstance(cur, str) and cur.startswith("classic:")) and \
                self.carsetup.currentData() != getattr(self, "_setup_shown", None):
            self._remember_setup()
            self.carsetup.currentIndexChanged.emit(self.carsetup.currentIndex())
        self._setup_shown = self.carsetup.currentData()

    def _remember_setup(self, _i=None):
        st = QSettings("juliaRacer", "launcher")
        st.setValue("car/setup_choice", self.carsetup.currentData())
        st.setValue("car/setup", 1 if self.setup_choice()[0] == "ww103" else 0)
        self._setup_shown = self.carsetup.currentData()

    def setup_choice(self):
        """-> (base session key, saved setup path or None, its name or None)"""
        key = self.carsetup.currentData() or "default"
        if key.startswith("classic:"):
            r = classic_import(key[len("classic:"):])
            return ("default", r[0], r[1]) if r else ("default", None, None)
        if key.startswith("saved:"):
            name = key[len("saved:"):]
            for n, path, base in named_setups():
                if n == name:
                    return (base, path, n)
            return ("default", None, None)
        return (key, None, None)
    def launch(self):
        if self.warm_alive():
            self.end_warm()
        if self.proc and self.proc.state() != QProcess.ProcessState.NotRunning:
            return
        if self.net_mode.currentIndex() == 2 and not self.net_host.text().strip():
            QMessageBox.warning(self, "Join a LAN race", "Enter the host's address (shown on the host's launcher).")
            return

        st = self._settings
        st.setValue("session/track", self.track.currentIndex()); st.setValue("session/mode", self.mode.currentIndex())
        st.setValue("session/laps", self.laps.value()); st.setValue("session/ai", self.ai.value())
        st.setValue("session/qual", "true" if self.qual.isChecked() else "false")
        st.setValue("car/gearbox", self.gearbox.currentIndex())
        self._status(f"Starting {self.mode.currentText().lower()} at {self.track.currentText()}… "
                     "(the game window opens when loading finishes)")

        self.joy.stop_reader()
        qenv = QProcessEnvironment.systemEnvironment()
        qenv.insert("TRACK", ["zandvoort", "skidpad", "nurburgring",
                              "watglen", "monza", "spa"][self.track.currentIndex()])
        is_race = (self.mode.currentIndex() == 1)
        qenv.insert("JM_MODE", "race" if is_race else "practice")
        qenv.insert("JM_LAPS", str(self.laps.value()))
        qenv.insert("JM_AI", str(self.ai.value() if is_race else 0))
        qenv.insert("JM_AI_PCT", str(self.ai_pct.value()))
        qenv.insert("JM_PLAYER_CAR", self.playercar.currentData() or "lotus")
        wet = self.wet.currentData() or "1"
        wet != "1" and qenv.insert("JM_WETNESS", wet)
        qenv.insert("ZAND_SHIFT", "auto" if self.gearbox.currentIndex() == 0 else "manual")
        qenv.insert("JM_CARSETUP", self.setup_choice()[0])
        gf = self.garage_file() if getattr(self, "garage_file", None) else None
        if gf:
            qenv.insert("JM_GARAGE", gf)
        qenv.insert("JM_FUEL", ["event", "full"][self.fuel.currentIndex()])
        timing_env(qenv)
        if self.mute.isChecked():
            qenv.insert("JM_NOSOUND", "1")
        if self.noffb.isChecked():
            qenv.insert("JM_NOFFB", "1")
        if not self.ibt.isChecked():
            qenv.insert("JM_NOIBT", "1")
        if not self.replay.isChecked():
            qenv.insert("JM_NOREPLAY", "1")
        if self.qual.isChecked():
            qenv.insert("JM_QUAL", "1")
        if self.d2.isChecked():
            qenv.insert("JM_2D", "1")
        self._net_env(qenv)
        self._gfx_env(qenv)
        if self.net_mode.currentIndex() == 0:
            qenv.insert("JM_WARM_HOLD", "1")
        self._warm = False; self._session_closed = False; self._warm_declined = None

        self._result_path = os.path.join(HERE, "last_race_result.txt")
        try:
            os.remove(self._result_path)
        except OSError:
            pass
        self.proc = QProcess(self)
        self.proc.setProcessEnvironment(qenv)
        self.proc.setWorkingDirectory(HERE)
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._log)
        self.proc.finished.connect(self._done)

        try:
            self._sim_log = open(os.path.join(HERE, "last_sim_run.log"), "w",
                                 encoding="utf-8", errors="replace")
        except OSError:
            self._sim_log = None

        jlargs = ["-t", "2", "--gcthreads=3,1", "--project=."]
        sysimg = os.path.join(HERE, "jlracer.so")
        fast = os.path.exists(sysimg)
        if fast:
            jlargs += ["-J", sysimg]
        jlargs.append("drive_native_mtk.jl")
        self.proc.start(find_julia(), jlargs)
        self.log.clear()
        self.log.appendPlainText(
            "launching drive_native_mtk.jl … " +
            ("(sysimage: faster start)\n" if fast else
             "(first load ~3–4 min — run build_sysimage.jl once to speed this up)\n"))
        self.launch_b.setEnabled(False)
        self.stop_b.setEnabled(True)
        self._launched_at = time.time()
        self._stage = 0
        self.progress.setRange(0, 0)
        self.progress.setFormat("compiling…")
        self.progress.setVisible(True)

    def stop(self):
        if self.proc:
            self._stopping = True
            self.proc.terminate()
            if not self.proc.waitForFinished(2000):
                self.proc.kill()

    on_warm_hold = None; on_warm_replay = None; on_warm_exit = None

    def warm_alive(self):
        return bool(self.proc) and self.proc.state() != QProcess.ProcessState.NotRunning and getattr(self, "_warm", False)

    def send_warm(self, line):
        self._warm = False
        self.proc.write((line + "\n").encode())

    def end_warm(self):
        if not self.warm_alive():
            return
        self._stopping = True
        self.proc.write(b"quit\n")
        if not self.proc.waitForFinished(5000):
            self.proc.kill()
            self.proc.waitForFinished(2000)

    def _log(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        self.log.appendPlainText(text.rstrip())
        f = getattr(self, "_sim_log", None)
        if f is not None:
            try: f.write(text); f.flush()
            except (OSError, ValueError): pass
        if "[warm] holding" in text:
            self._warm = True
            if not getattr(self, "_session_closed", False):
                self._session_closed = True
                self._session_over(False)
            if self.on_warm_hold:
                self.on_warm_hold()
            return
        m = re.search(r"\[warm\] declined: ([^\n]*)", text)
        if m:
            self._warm_declined = m.group(1)
        if "[instant] replaying" in text and self.on_warm_replay:
            self.on_warm_replay()

        done = len(re.findall(r"✓\s", text))
        if done or "Precompiling packages" in text:
            if getattr(self, "_pc_t0", None) is None:
                self._pc_t0 = time.monotonic()
                self._pc_n = 0
            self._pc_n += done
            n = self._pc_n
            total = max(self.PRECOMPILE_EXPECTED, n)
            frac = self._precompile_work_fraction(n)
            self.progress.setRange(0, 1000)
            self.progress.setValue(min(int(frac * 1000), 999))
            elapsed = time.monotonic() - self._pc_t0

            if frac > 0.02 and elapsed > 20:
                remain = elapsed * (1.0 / frac - 1.0)
                mins = int(remain // 60)
                eta = f"about {mins} min left" if mins >= 1 else "under a minute left"
            else:
                eta = "estimating…"
            mm = int(elapsed // 60)
            self.progress.setFormat(
                f"compiling Julia packages — {n}/{total} — {mm} min elapsed, {eta}"
                f"   (one-off: later launches start immediately)")
            self.progress.setVisible(True)
            return

        if getattr(self, "_pc_t0", None) is not None and getattr(self, "_stage", 0) == 0:

            self.progress.setRange(0, 100)
            self.progress.setValue(0)
            self._pc_t0 = None

        m = re.findall(r"\[texcache\] first run on this track: preparing textures, (\d+)", text)
        if m:
            self._status(f"Loading: preparing this track's textures for the first time ({m[-1]} so far) -- "
                         "a one-off, later launches skip it")
        for marker, pct, label in self.LOAD_STAGES:
            if marker in text and pct > getattr(self, "_stage", 0):
                if self.progress.maximum() == 0:
                    self.progress.setRange(0, 100)
                self._stage = pct
                self.progress.setValue(pct)
                self.progress.setFormat(f"{label}  %p%")
                self._status("Loading: " + label)

    def _keep_run_log(self):
        try:
            trk = ["zandvoort", "skidpad", "nurburgring", "watglen", "monza", "spa"][self.track.currentIndex()]
        except (IndexError, AttributeError):
            trk = None
        keep_run_log(os.path.join(HERE, "last_sim_run.log"), getattr(self, "_launched_at", None), trk)

    def _done(self):
        self._warm = False
        if getattr(self, "_session_closed", False):
            self._session_closed = False
            self._stopping = False
            self.log.appendPlainText("\n— game exited —")
            self.launch_b.setEnabled(True)
            self.stop_b.setEnabled(False)
            if self.on_warm_exit:
                self.on_warm_exit(getattr(self, "_warm_declined", None))
            self._warm_declined = None
            return
        failed = (self.proc.exitStatus() != QProcess.ExitStatus.NormalExit or self.proc.exitCode() != 0) \
            and not getattr(self, "_stopping", False)
        self._session_over(failed)

    def _session_over(self, failed):
        self.log.appendPlainText("\n— session over —" if getattr(self, "_warm", False) else "\n— game exited —")

        w = self.window()
        if w is not None:
            if w.isMinimized():
                w.showNormal()
            w.raise_()
            w.activateWindow()
        self.launch_b.setEnabled(True)
        self.stop_b.setEnabled(False)
        self.progress.setVisible(False)

        self._stopping = False
        rep = self.on_session_end(getattr(self, "_launched_at", None)) if getattr(self, "on_session_end", None) else None
        if failed:
            self.log_b.setChecked(True)
            self._status("The game stopped with an error -- the log below has the details (also last_sim_run.log)."
                         + ("  Its replay was saved (Replays tab)." if rep else ""))
        else:
            self._status("Session ended." + ("  Its replay is ready on the Replays tab." if rep else "") + "  Ready.")
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setFormat("")
        f = getattr(self, "_sim_log", None)
        if f is not None:
            try:
                f.write("\n[juliaRacer] session held for replays (the process stays)\n" if getattr(self, "_warm", False) else
                        "\n[juliaRacer] sim exited: code=%r status=%r\n" % (self.proc.exitCode(), self.proc.exitStatus()))
                f.close()
            except (OSError, ValueError, RuntimeError): pass
            self._sim_log = None
            self._keep_run_log()

        self._track_changed(self.track.currentIndex())

        if self.on_result:
            self.on_result(getattr(self, "_result_path", os.path.join(HERE, "last_race_result.txt")))

def _run_log_dir():
    return os.path.join(os.path.dirname(os.path.dirname(HERE)), "data", "juliaracer", "logs")

def keep_run_log(src, launched_at, track, keep=40):
    """TD261013-1 (PO 2026-10-13 test drive): last_sim_run.log is rewritten by every launch, so of an evening's six runs
    only the last one's log survived -- the Spa run whose wheels came off left nothing to read. Keep a copy of each run's
    log next to the replays (data/juliaracer/logs/<track> <date time>.log), the newest `keep` of them."""
    import shutil, time as _t
    try:
        d = _run_log_dir(); os.makedirs(d, exist_ok=True)
        stamp = _t.strftime("%Y-%m-%d %H-%M-%S", _t.localtime(launched_at or _t.time()))
        dst = os.path.join(d, "%s %s.log" % (track or "session", stamp))
        shutil.copyfile(src, dst)
        logs = sorted((os.path.join(d, n) for n in os.listdir(d) if n.endswith(".log")), key=os.path.getmtime)
        for old in logs[:-keep]:
            os.remove(old)
        return dst
    except OSError:
        return None

class ResultTab(QWidget):
    """PO: the race result lives in a TAB (not a modal that duplicates 'choose track'). Two inner tabs —
    Classification (order + winner total / gap) and Your laps — populated from last_race_result.txt."""
    def __init__(self, on_again):
        super().__init__()
        self._on_again = on_again
        v = QVBoxLayout(self)
        self.head = QLabel("<i>No race yet — start one from the Race tab.</i>")
        self.head.setTextFormat(Qt.TextFormat.RichText)
        v.addWidget(self.head)
        self.tabs = QTabWidget()
        self.cls = QLabel(); self.cls.setTextFormat(Qt.TextFormat.RichText)
        cw = QWidget(); cv = QVBoxLayout(cw); cv.addWidget(self.cls); cv.addStretch(1)
        self.tabs.addTab(cw, "Classification")
        self.laps = QLabel(); self.laps.setTextFormat(Qt.TextFormat.RichText)
        lw = QWidget(); lv = QVBoxLayout(lw); lv.addWidget(self.laps); lv.addStretch(1)
        self.tabs.addTab(lw, "Your laps")
        v.addWidget(self.tabs)
        brow = QHBoxLayout()
        again = QPushButton("Race again")
        again.clicked.connect(lambda: self._on_again())
        brow.addWidget(again); brow.addStretch(1)
        v.addLayout(brow)

    def load(self, path):
        """Render last_race_result.txt into the tab. Returns True if a result was found."""
        if not os.path.exists(path):
            return False
        try:
            rows = [ln.rstrip("\n").split("\t") for ln in open(path) if ln.strip()]
        except OSError:
            return False
        d = {r[0]: r[1] for r in rows if len(r) >= 2 and not r[0].startswith("P")}
        grid = [(r[1], r[2] if len(r) >= 3 else "") for r in rows if r[0].startswith("P") and len(r) >= 2]
        best_any = next((r[1:] for r in rows if r[0] == "best_any"), ["-", "-"])
        your_laps = d.get("you_laps", "").split(",") if d.get("you_laps") else []
        self.head.setText(
            f"<b>{d.get('track','?').title()} — {d.get('laps','?')} laps</b><br>"
            f"You finished <b>P{d.get('you_pos','?')}</b> of {d.get('field','?')} "
            f"(started P{d.get('you_start','?')})")

        crows = ["<table cellspacing=0 cellpadding=4 width='100%'>",
                 "<tr style='color:#888'><th align=left>Pos</th><th align=left>Car</th>"
                 "<th align=right>Total&nbsp;/&nbsp;gap</th></tr>"]
        for i, (name, gap) in enumerate(grid, 1):
            you = (name == "You")
            sty = " style='background:#ffe98a;color:#000'" if you else ""
            nm = f"<b>{name}</b>" if you else name
            crows.append(f"<tr{sty}><td>P{i}</td><td>{nm}</td><td align=right>{gap}</td></tr>")
        crows.append("</table>")
        fast = (f"<br><b>Fastest lap:</b> {best_any[0]}"
                + (f" &nbsp;({best_any[1]})" if len(best_any) > 1 else ""))
        self.cls.setText("".join(crows) + fast)

        if your_laps:
            lrows = ["<table cellspacing=0 cellpadding=4 width='100%'>",
                     "<tr style='color:#888'><th align=left>Lap</th><th align=right>Time</th></tr>"]
            best_str = d.get("you_best", ""); seen_best = False
            for i, lt in enumerate(your_laps, 1):
                mark = ""
                if lt == best_str and not seen_best:
                    mark = " &nbsp;<span style='color:#6c6'>(best)</span>"; seen_best = True
                lrows.append(f"<tr><td>Lap {i}</td><td align=right>{lt}{mark}</td></tr>")
            lrows.append("</table>")
        else:
            lrows = ["<i>No completed laps recorded.</i>"]
        self.laps.setText("".join(lrows)
                          + f"<br><b>Your best:</b> {d.get('you_best','-')} &nbsp;·&nbsp; "
                            f"<b>Total:</b> {d.get('you_total','-')}")
        return True

class ReplayTab(QWidget):
    """E18: pick a saved .jmr race recording and watch it back (cockpit/chase + VCR keys)."""
    def __init__(self, joy):
        super().__init__()
        self.joy = joy
        self.proc = None
        self.dir = os.path.join(os.path.dirname(os.path.dirname(HERE)), "data", "juliaracer")
        v = QVBoxLayout(self)
        v.addWidget(QLabel("Every session is recorded. Pick one and watch it back (newest first):"))
        self.combo = QComboBox()
        v.addWidget(self.combo)
        self.latest_l = QLabel(""); self.latest_l.setObjectName("hint")
        v.addWidget(self.latest_l)
        row = QHBoxLayout()
        self.refresh_b = QPushButton("Refresh")
        self.watch_b = QPushButton("Watch replay")
        self.watch_b.setObjectName("primary")
        self.analyse_b = QPushButton("Analyse…")
        self.analyse_b.setToolTip("Laps, racing lines on the track map, telemetry graphs, time difference and split times")
        self.stop_b = QPushButton("Stop")
        self.stop_b.setEnabled(False)
        row.addWidget(self.watch_b)
        row.addWidget(self.analyse_b)
        row.addWidget(self.stop_b)
        row.addStretch(1)
        row.addWidget(self.refresh_b)
        v.addLayout(row)
        v.addWidget(QLabel("In the replay:  SPACE play/pause · ←/→ scrub · PgUp/PgDn ±10 s · Home/End start/end · ↑/↓ speed · Esc quit"))

        v.addWidget(QLabel("The race you just drove: press I in the race window once it is over (flag or wreck) to watch it at once."))

        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setTextVisible(True)
        self.progress.setMinimumHeight(22)
        v.addWidget(self.progress)
        v.addWidget(QLabel("Cinematics:  V = switch ANGLE (cockpit · chase · TV/distant · F10 rear · nose · RR-suspension)   ·   C = switch CAR (cycle the field)"))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        v.addWidget(self.log)
        self.refresh_b.clicked.connect(self.refresh)
        self.watch_b.clicked.connect(self.watch)
        self.analyse_b.clicked.connect(self.analyse)
        self.combo.currentIndexChanged.connect(self._can_analyse)
        self.stop_b.clicked.connect(self.stop)
        self.refresh()

    TRACK_NAMES = {"zandvoort": "Zandvoort", "skidpad": "Skidpad", "nurburgring": "Nürburgring",
                   "watglen": "Watkins Glen", "monza": "Monza", "spa": "Spa"}

    @classmethod
    def label(cls, name):
        """REPLAY-2: 'replay_watglen 5ai 2026-10-05 08-54-36.jmr' -> 'Watkins Glen · 5 AI · 2026-10-05 08:54'."""
        m = re.match(r"replay_(\w+) (\d+)ai (\d{4}-\d\d-\d\d) (\d\d)-(\d\d)", name)
        if not m:
            return name
        n = int(m.group(2))
        return (f"{cls.TRACK_NAMES.get(m.group(1), m.group(1))}  ·  {'solo' if n == 0 else f'{n} AI'}  ·  "
                f"{m.group(3)} {m.group(4)}:{m.group(5)}")

    def refresh(self):
        """REPLAY-2: newest first by time (a name sort grouped them by track), readable labels, the file as data."""
        self.combo.clear()
        try:
            files = [f for f in os.listdir(self.dir) if f.endswith(".jmr")]
            files.sort(key=lambda f: os.path.getmtime(os.path.join(self.dir, f)), reverse=True)
        except OSError:
            files = []
        for f in files:
            self.combo.addItem(self.label(f), f)
        self.latest_l.setText("" if files else "No recordings yet -- every session you drive is saved here.")
        return files

    def _current_jrt(self):
        name = self.combo.currentData() or self.combo.currentText()
        if not name:
            return None
        p = os.path.join(self.dir, name[:-4] + ".jrt")
        return p if os.path.exists(p) else None

    def _can_analyse(self, *_):
        ok = self._current_jrt() is not None
        self.analyse_b.setEnabled(ok)
        self.analyse_b.setToolTip("Laps, racing lines on the track map, telemetry graphs, time difference and split times"
                                  if ok else "This recording predates the analysis data (recorded before 2026-10-07).")

    def analyse(self):
        p = self._current_jrt()
        if p is None:
            return
        import analyser
        try:
            self._an = analyser.AnalyserWindow(p, self)
        except Exception as e:
            QMessageBox.warning(self, "Analyse", f"Could not read {os.path.basename(p)}: {e}")
            return
        self._an.show()

    def show_latest(self, since=None):
        """REPLAY-2: after a session, select its recording (the newest file, if it was written after `since`)."""
        files = self.refresh()
        if files and (since is None or os.path.getmtime(os.path.join(self.dir, files[0])) >= since - 1):
            self.combo.setCurrentIndex(0)
            self.latest_l.setText("The session you just finished: " + self.label(files[0]))
            return files[0]
        return None

    def watch(self, _checked=False, fresh=False):
        if self.proc and self.proc.state() != QProcess.ProcessState.NotRunning:
            return
        name = self.combo.currentData() or self.combo.currentText()
        if not name:
            return
        d = getattr(self, "drive", None)
        if d is not None and d.warm_alive():
            if not fresh:
                self._warm_name = name
                d.on_warm_replay = self._warm_started; d.on_warm_hold = self._warm_ended; d.on_warm_exit = self._warm_exit
                self.log.clear()
                self.log.appendPlainText(f"replaying {name} in the last race's process (its track is loaded) …")
                self.watch_b.setEnabled(False)
                d.send_warm("replay " + os.path.join(self.dir, name))
                return
            d.end_warm()
        m = re.match(r"replay_(\w+) (\d+)ai", name)
        if not m:
            self.log.appendPlainText("could not read track / car count from the filename")
            return
        track, nai = m.group(1), m.group(2)
        self.joy.stop_reader()
        qenv = QProcessEnvironment.systemEnvironment()
        qenv.insert("TRACK", track)
        qenv.insert("JM_MODE", "race")
        qenv.insert("JM_AI", nai)
        qenv.insert("JM_REPLAY", os.path.join(self.dir, name))
        qenv.insert("JM_VIEW", "0")
        timing_env(qenv)
        segnames_env(qenv, str(QSettings("juliaRacer", "launcher").value("hud/segnames", "true")) == "true")
        hud_env(qenv, QSettings("juliaRacer", "launcher"))
        spectators_env(qenv, str(QSettings("juliaRacer", "launcher").value("scene/spectators", "false")) == "true")
        self.proc = QProcess(self)
        self.proc.setProcessEnvironment(qenv)
        self.proc.setWorkingDirectory(HERE)
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._log)
        self.proc.finished.connect(self._done)
        self._stage = 0
        self._t0 = time.monotonic()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setFormat("starting Julia…  (a replay loads the whole track, like a race: 3–4 min)")
        self.progress.setVisible(True)
        jlargs = ["-t", "2", "--gcthreads=3,1", "--project=."]
        sysimg = os.path.join(HERE, "jlracer.so")
        if os.path.exists(sysimg):
            jlargs += ["-J", sysimg]
        jlargs.append("drive_native_mtk.jl")
        self.proc.start(find_julia(), jlargs)
        self.log.clear()
        self.log.appendPlainText(f"replaying {name} … (loading)")
        self.watch_b.setEnabled(False)
        self.stop_b.setEnabled(True)

    def stop(self):
        if self.proc:
            self.proc.terminate()
            if not self.proc.waitForFinished(2000):
                self.proc.kill()

    LOAD_STAGES = [
        ("loading Classic", 15, "loading track…"),
        ("extracting geometry", 35, "extracting geometry…"),
        ("loading textures", 50, "decoding textures…"),
        ("placed", 70, "placing scenery…"),
        ("AI car models begin", 80, "loading the cars…"),
        ("physics build", 90, "building physics…"),
        ("REPLAY:", 100, "ready — replay window opening"),
    ]

    def _log(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        self.log.appendPlainText(text.rstrip())
        m = re.findall(r"\[texcache\] first run on this track: preparing textures, (\d+)", text)
        if m:
            mm = int((time.monotonic() - self._t0) // 60)
            self.progress.setFormat(f"preparing this track's textures for the first time ({m[-1]} so far, one-off)"
                                    f"   ({mm} min elapsed)")
        for marker, pct, label in self.LOAD_STAGES:
            if marker in text and pct > self._stage:
                self._stage = pct
                mm = int((time.monotonic() - self._t0) // 60)
                self.progress.setValue(pct)
                self.progress.setFormat(f"{label}  %p%   ({mm} min elapsed)")

    def _warm_started(self):
        self.log.appendPlainText("replay window open (no reload)")

    def _warm_ended(self):
        self._warm_unhook()
        self.log.appendPlainText("\n— replay ended —")
        self.watch_b.setEnabled(True)

    def _warm_exit(self, why):
        self._warm_unhook()
        self.watch_b.setEnabled(True)
        if why:
            self.log.appendPlainText(f"the last race's process cannot play it ({why}) -- loading the track")
            self.watch(fresh=True)

    def _warm_unhook(self):
        d = getattr(self, "drive", None)
        if d is not None:
            d.on_warm_replay = d.on_warm_hold = d.on_warm_exit = None

    def _done(self):
        self.log.appendPlainText("\n— replay ended —")
        self.progress.setVisible(False)
        self.progress.setValue(0)
        self.watch_b.setEnabled(True)
        self.stop_b.setEnabled(False)

class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Julia Racer — Lotus 49")
        self.resize(980, 700)
        self.joy = JoyReader(self)
        tabs = QTabWidget()
        self.tabs = tabs
        self.cal = CalibrateTab(self.joy, self._reload)
        self.result = ResultTab(self._race_again)
        self.drive = DriveTab(self.joy, on_result=self._show_result_tab)
        self.replay = ReplayTab(self.joy)
        self.drive.on_session_end = self.replay.show_latest
        self.replay.drive = self.drive

        tabs.addTab(self.drive, "Race")

        self.garage = GarageTab(self.drive.setup_choice)
        self.garage.on_setups_changed = lambda name: self.drive.reload_setups(select="saved:" + name)
        self.garage.get_track = lambda: TRACK_KEYS[max(0, self.drive.track.currentIndex())]
        self.drive.carsetup.currentIndexChanged.connect(lambda _i: self.garage.load_session(keep_saved=True))
        self.drive.garage_file = self.garage.garage_file_for_launch
        self.garage.set_car(self.drive.playercar.currentData() or "lotus")
        self.drive.playercar.currentIndexChanged.connect(lambda _i: self.garage.set_car(self.drive.playercar.currentData() or "lotus"))
        tabs.addTab(self.garage, "Garage")
        tabs.addTab(self.result, "Results")
        tabs.addTab(self.replay, "Replays")
        self.guides = GuidesTab(lambda: TRACK_KEYS[max(0, self.drive.track.currentIndex())])
        self.drive.track.currentIndexChanged.connect(lambda _i: self.guides.fill())
        tabs.addTab(self.guides, "Guides")
        tabs.addTab(self.drive.settings_page(), "Settings")
        tabs.addTab(self.cal, "Controller")
        tabs.currentChanged.connect(self._tab)
        corner = QWidget(); cl = QHBoxLayout(corner); cl.setContentsMargins(0, 0, 6, 0)
        self.text_l = QLabel("Text"); cl.addWidget(self.text_l)
        self.smaller_b = QPushButton("A−"); self.smaller_b.setToolTip("Smaller text (Ctrl+−)")
        self.bigger_b = QPushButton("A+"); self.bigger_b.setToolTip("Bigger text (Ctrl++)")
        for b, d in ((self.smaller_b, -1), (self.bigger_b, +1)):
            b.clicked.connect(lambda _c=False, d=d: self._text_size(d)); cl.addWidget(b)
        tabs.setCornerWidget(corner, Qt.Corner.TopRightCorner)
        self.setCentralWidget(tabs)
        self._build_menus()
        self.statusBar().showMessage("Ready")
        self.joy.start_reader(1)

    def _build_menus(self):
        mb = self.menuBar()
        game = mb.addMenu("&Game")
        a = game.addAction("&Start"); a.setShortcut("Ctrl+Return"); a.triggered.connect(self._start)
        a = game.addAction("S&top"); a.triggered.connect(self.drive.stop)
        game.addSeparator()
        a = game.addAction("&Quit"); a.setShortcut("Ctrl+Q"); a.triggered.connect(self.close)
        view = mb.addMenu("&View")
        for txt, keys, d in (("&Bigger text", ("Ctrl++", "Ctrl+="), +1), ("&Smaller text", ("Ctrl+-",), -1),
                             ("&Normal text size", ("Ctrl+0",), 0)):
            a = view.addAction(txt); a.setShortcuts([QKeySequence(k) for k in keys])
            a.triggered.connect(lambda _c=False, d=d: self._text_size(d))
        hlp = mb.addMenu("&Help")
        a = hlp.addAction("&How to race…"); a.triggered.connect(lambda: guide_dialog(self).exec())
        a = hlp.addAction("&Controls…"); a.setShortcut("F1"); a.triggered.connect(lambda: show_controls(self))
        a = hlp.addAction("&About Julia Racer"); a.triggered.connect(self._about)

    def _text_size(self, d):
        k = step_text_scale(QApplication.instance(), d)
        self.statusBar().showMessage(f"Text size {k:.0%}", 3000)

    def _start(self):
        self.tabs.setCurrentWidget(self.drive)
        self.drive.launch()

    def _about(self):
        QMessageBox.about(self, "About Julia Racer",
                          "<b>Julia Racer</b><br>The 1967 Lotus 49 on Classic's circuits, with physics fitted to reference "
                          "telemetry (ModelingToolkit) and an AI field that drives like Classic's.")

    def _show_result_tab(self, path):
        """PO: after a race, populate the Race Result tab and switch to it (no modal popup)."""
        if self.result.load(path):
            self.tabs.setCurrentWidget(self.result)

    def _race_again(self):
        self.tabs.setCurrentWidget(self.drive)
        self.drive.launch()

    def _tab(self, i):

        if self.tabs.widget(i) is self.cal and self.joy.state() == QProcess.ProcessState.NotRunning:
            self.joy.start_reader(1)

    def _reload(self):
        self.cal.map = JoyMap.load(CONF)

    def closeEvent(self, e):
        self.joy.stop_reader()
        if self.drive.proc:
            self.drive.stop()
        super().closeEvent(e)

THEME = """
QMainWindow, QDialog { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0e1a24, stop:1 #1a2834); }
QWidget { color: #e8eef3; font-family: Arial; font-size: 11pt; }
QTabWidget::pane { border: 1px solid #2c3e50; border-radius: 6px; background: rgba(10, 18, 26, 120); top: -1px; }
QTabBar::tab { background: #1b2733; color: #b9c7d3; padding: 8px 18px; border: 1px solid #2c3e50;
               border-bottom: none; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px; }
QTabBar::tab:selected { background: #24384a; color: #ffffff; font-weight: bold; border-bottom: 3px solid #3fa86a; }
QTabBar::tab:hover:!selected { background: #22313f; }
QGroupBox { background: rgba(255, 255, 255, 10); border: 1px solid #2f6f4f; border-radius: 6px;
            margin-top: 14px; padding: 12px 10px 10px 10px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #7fd1ae; }
QLabel { background: transparent; }
QLabel#hint { color: #8aa0b2; font-size: 9pt; font-weight: normal; }
QPushButton { background: #2a3a4a; color: #dde6ee; border: 1px solid #3e5468; border-radius: 4px; padding: 6px 14px; }
QPushButton:hover { background: #34495c; }
QPushButton:pressed { background: #22303d; }
QPushButton:checked { background: #24384a; border-color: #3fa86a; }
QPushButton:disabled { background: #1c2731; color: #5f6f7e; border-color: #2a3742; }
QPushButton#primary { background: #2e7d4f; border: 1px solid #3fa86a; color: #ffffff; font-size: 13pt; font-weight: bold; }
QPushButton#primary:hover { background: #379160; }
QPushButton#primary:pressed { background: #276b43; }
QPushButton#primary:disabled { background: #1f3a2b; color: #6f8f7d; border-color: #2a4a38; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit { background: #0f1922; color: #e8eef3; border: 1px solid #34495e; border-radius: 3px;
                                 padding: 4px 6px; selection-background-color: #2e7d4f; }
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover { border-color: #3fa86a; }
QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QLineEdit:disabled { color: #5f6f7e; border-color: #26333f; }
QComboBox QAbstractItemView { background: #0f1922; color: #e8eef3; border: 1px solid #34495e;
                              selection-background-color: #2e7d4f; selection-color: #ffffff; }
QSpinBox, QDoubleSpinBox { padding-right: 22px; }
QSpinBox::up-button, QSpinBox::down-button, QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { subcontrol-origin: border; width: 20px; background: #1f2c38;
                                             border-left: 1px solid #34495e; }
QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-position: top right; border-top-right-radius: 3px; }
QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-position: bottom right; border-bottom-right-radius: 3px; }
QSpinBox::up-button:hover, QSpinBox::down-button:hover, QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background: #2e7d4f; }
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url(@UI@/arrow_up.png); width: 10px; height: 6px; }
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url(@UI@/arrow_down.png); width: 10px; height: 6px; }
QCheckBox { spacing: 8px; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #4a6276; border-radius: 3px; background: #0f1922; }
QCheckBox::indicator:checked { background: #2e7d4f; border-color: #3fa86a; image: url(@UI@/check.png); }
QCheckBox::indicator:hover { border-color: #3fa86a; }
QProgressBar { background: #0f1922; border: 1px solid #34495e; border-radius: 4px; color: #e8eef3; text-align: center; }
QProgressBar::chunk { background: #2e7d4f; border-radius: 3px; }
QScrollArea, QScrollArea > QWidget > QWidget { background: transparent; }
QListWidget, QListView { background: #0f1922; color: #e8eef3; border: 1px solid #2c3e50; border-radius: 4px; }
QListWidget::item { padding: 6px 8px; }
QListWidget::item:selected { background: #2e7d4f; color: #ffffff; }
QTextBrowser#guide { background: #101a23; color: #e8eef3; border: 1px solid #2c3e50; border-radius: 4px;
                     font-family: Arial; font-size: 12pt; padding: 10px; }
QTableWidget, QTableView { background: #0f1922; alternate-background-color: #131f2a; color: #e8eef3;
                           gridline-color: #26333f; border: 1px solid #2c3e50; border-radius: 4px; }
QHeaderView::section { background: #1b2733; color: #b9c7d3; border: none; border-right: 1px solid #26333f;
                       border-bottom: 1px solid #26333f; padding: 4px 6px; font-weight: bold; }
QTableCornerButton::section { background: #1b2733; border: none; }
QSplitter::handle { background: #2c3e50; }
QSplitter::handle:hover { background: #3fa86a; }
QPlainTextEdit, QTextEdit { background: #0b1218; color: #b8c7d3; border: 1px solid #2c3e50; border-radius: 4px;
                            font-family: monospace; font-size: 9pt; }
QMenuBar { background: #0b1218; color: #dde6ee; }
QMenuBar::item:selected { background: #24384a; }
QMenu { background: #16222d; color: #e8eef3; border: 1px solid #34495e; }
QMenu::item:selected { background: #2e7d4f; }
QStatusBar { background: #0b1218; color: #5fd38d; font-weight: bold; }
QToolTip { background: #16222d; color: #e8eef3; border: 1px solid #3fa86a; }
QScrollBar:vertical { background: #0f1922; width: 12px; }
QScrollBar::handle:vertical { background: #34495e; border-radius: 5px; min-height: 24px; }
"""

TEXT_SCALES = (0.8, 0.9, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0, 2.5)

def text_scale():
    try:
        v = float(QSettings("juliaRacer", "launcher").value("ui/text_scale", 1.0))
    except (TypeError, ValueError):
        v = 1.0
    return min(TEXT_SCALES, key=lambda x: abs(x - v))

def scaled_pt(qss, k):
    return re.sub(r"(\d+(?:\.\d+)?)pt\b", lambda m: f"{float(m.group(1)) * k:.1f}pt", qss)

def apply_theme(app, scale=None):
    k = text_scale() if scale is None else scale
    if os.environ.get("JR_THEME", "") != "classic":
        app.setStyle("Fusion")
        app.setStyleSheet(scaled_pt(THEME.replace("@UI@", UI_DIR.replace(os.sep, "/")), k))
    else:
        if not hasattr(app, "_jr_base_pt"):
            app._jr_base_pt = app.font().pointSizeF()
        f = app.font(); f.setPointSizeF(app._jr_base_pt * k); app.setFont(f)

def step_text_scale(app, d):
    """d = +1 bigger, -1 smaller, 0 back to normal; applies and remembers the new scale, returns it."""
    i = TEXT_SCALES.index(text_scale())
    k = 1.0 if d == 0 else TEXT_SCALES[max(0, min(len(TEXT_SCALES) - 1, i + d))]
    QSettings("juliaRacer", "launcher").setValue("ui/text_scale", k)
    apply_theme(app, k)
    return k

def main():
    app = QApplication(sys.argv)
    apply_theme(app)
    w = Main()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
