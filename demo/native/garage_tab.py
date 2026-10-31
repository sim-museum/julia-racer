"""SETUPMODEL-1: the launcher's GARAGE tab -- reference's Lotus 49 setup page in julia.

PO 2026-10-15: "julia's Setup tab then adjusts the car via reference's own list of settings"; the garage's derived numbers
(corner weights, ride heights, shock deflections, cross weight, static camber/toe, gear speeds) follow the knobs the way
reference's do. Pages, names, steps and limits are reference's (JuliaMotorMTK/src/garage_lotus49.toml); a dagger marks a
knob whose limits the PO's capture has not shown yet (provisional). The numbers are the chosen session's own plus the
garage model's change (garage.py, the mirror of JuliaMotorMTK/src/garage.jl): "Reset" is the session to the digit.

On a launch with a modified garage, the launcher writes the knobs to a garage file and hands it to the sim (JM_GARAGE);
the sim re-solves them with the Julia model before the car is built.
"""
import copy
import glob
import os

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout,
                             QInputDialog, QLabel, QMessageBox, QPushButton, QScrollArea, QTabWidget, QVBoxLayout,
                             QWidget)

import garage as gm
import classic_setup
import reference_sheet

CORNERS = ("LEFT FRONT", "RIGHT FRONT", "LEFT REAR", "RIGHT REAR")

TT_CORNERS = (("LF", "LEFT FRONT"), ("RF", "RIGHT FRONT"), ("LR", "LEFT REAR"), ("RR", "RIGHT REAR"))
def tyre_hints(imo, work=60.0):
    """Tuning hints for one tyre's (inner, middle, outer) carcass temperatures [C]."""
    i, m, o = imo
    out = []
    if max(imo) < work - 5:
        out.append("cold: not up to temperature (less grip) -- drive harder before judging the spread")
        return out
    edges = (i + o) / 2
    if m > edges + 4:
        out.append("middle hot: too much pressure -- lower it")
    elif m < edges - 4:
        out.append("edges hot: too little pressure -- raise it")
    if i - o > 10:
        out.append("inner much hotter: too much negative camber -- reduce it")
    elif o - i > 3:
        out.append("outer hotter: too little negative camber -- add some")
    return out or ["even: no change suggested"]
def tt_colour(T):
    return "#4a78ff" if T < 50 else "#2fbf4a" if T < 90 else "#e6c020" if T < 115 else "#ff3a1a"
SESSION_KEYS = ("default", "ww103")
BASE_NAMES = {"default": "reference default", "ww103": "WW103 fast loose"}
PROVISIONAL_TIP = ("† Provisional range: the PO's reference capture (261017) shows this knob's values but not yet its "
                   "limits; SETUPMODEL-1 Part C will settle them.")

class KnobSpin(QDoubleSpinBox):
    """reference's knob: steps on the knob's own lattice (e.g. 1/64 inch) at full precision, shows the garage's digits."""
    def __init__(self, show_decimals, lattice=None):
        super().__init__()
        self.show_decimals = show_decimals
        self.lattice = lattice
        self.setDecimals(6)

    def textFromValue(self, v):
        return f"{v:.{self.show_decimals}f}"

    def valueFromText(self, text):
        t = text.replace(self.suffix(), "").replace(self.prefix(), "").strip()
        try:
            return float(t)
        except ValueError:
            return self.value()

    def stepBy(self, steps):
        v = self.value() + steps * self.singleStep()
        if self.lattice:
            v = round(v / self.lattice) * self.lattice
        self.setValue(min(max(v, self.minimum()), self.maximum()))

def garage_dir():
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config")
    d = os.path.join(base, "juliaRacer", "garage")
    os.makedirs(d, exist_ok=True)
    return d

_NAME_CARS = (("lotus", ("lotus", "lot")), ("honda", ("honda",)), ("brm", ("brm",)), ("ferrari", ("ferrari", "ferr")),
              ("eagle", ("eagle",)), ("brabham", ("brabham", "brab")), ("cooper", ("cooper",)))
_NAME_TRACKS = (("monza", ("monza", "mza")), ("zandvoort", ("zandvoort", "zandvort", "zand")), ("nurburgring", ("nurburg", "ring")),
                ("watglen", ("watglen", "glen")), ("spa", ("spa",)), ("skidpad", ("skidpad",)))

TRACK_TITLES = {"zandvoort": "Zandvoort", "skidpad": "Skidpad", "nurburgring": "Nürburgring", "watglen": "Watkins Glen",
                "monza": "Monza", "spa": "Spa"}

def track_in_name(text):
    """SETUPFILTER-1: the track a name or path speaks of (a TRACK_KEYS key) or None."""
    toks = [t for t in "".join(c if c.isalnum() else " " for c in str(text).lower()).split()]
    return next((t for t, ks in _NAME_TRACKS if any(x == k or x.startswith(k) for x in toks for k in ks)), None)

def setup_tags(path, name):
    """SETUPFILTER-1: (car, track) a saved setup was made for; track None = any."""
    try:
        with open(path, "rb") as f:
            d = gm.tomllib.load(f)
    except Exception:
        d = {}
    toks = [t for t in "".join(c if c.isalnum() else " " for c in name.lower()).split()]
    hit = lambda keys: any(t == k or t.startswith(k) for t in toks for k in keys)
    car = d.get("car") or next((c for c, ks in _NAME_CARS if hit(ks)), "lotus")
    track = d.get("track") or next((t for t, ks in _NAME_TRACKS if hit(ks)), None)
    return car, track

def named_setups(car=None, track=None):
    """SETUPLIST-1 (PO 2026-10-20: an imported setup must be choosable): every saved garage setup -- Save as, or an
    import -- as (name, path, base). The current_*.toml files are the per-choice working copies, not setups.
    SETUPFILTER-1 (PO 2026-10-28, revised the same day: "offer all setups available for the car the user chose,
    regardless of track. The setup has to match the car. Whether it matches the track is for the user to decide"): with
    car, only that car's setups; with track too, only that track's (not used by the launcher)."""
    out = []
    for fn in sorted(glob.glob(os.path.join(garage_dir(), "*.toml")), key=lambda f: os.path.basename(f).lower()):
        name = os.path.splitext(os.path.basename(fn))[0]
        if name.startswith("current_"):
            continue
        try:
            base, _ = gm.read_garage_file(fn)
        except Exception:
            continue
        if base in SESSION_KEYS:
            if car is not None or track is not None:
                c, t = setup_tags(fn, name)
                if (car is not None and c != car) or (track is not None and t is not None and t != track):
                    continue
            out.append((name, fn, base))
    return out

def safe_name(name):
    return "".join(c if c.isalnum() or c in "-_ " else "_" for c in name.strip())

def spec(key):
    return next(k for k in gm.KNOB_SPECS if k["key"] == key)

def same_knobs(a, b):
    for f in gm.KNOB_FIELDS:
        x, y = a[f], b[f]
        if isinstance(x, (list, tuple)):
            if len(x) != len(y) or any(abs(float(p) - float(q)) > 1e-6 for p, q in zip(x, y)):
                return False
        elif isinstance(x, str) or isinstance(y, str):
            if str(x) != str(y):
                return False
        elif abs(float(x) - float(y)) > 1e-6:
            return False
    return True

class GarageTab(QWidget):
    def __init__(self, get_choice, parent=None):
        super().__init__(parent)

        self._choice = get_choice
        self.on_setups_changed = None
        self.get_track = None
        self._sessions = gm.sessions()
        self._busy = False
        self.widgets = {}
        self.derived_labels = {}
        self._shown = {}
        self._build()
        self.load_session(keep_saved=True)

    def choice(self):
        c = self._choice()
        if isinstance(c, int):
            return (SESSION_KEYS[max(0, min(c, len(SESSION_KEYS) - 1))], None, None)
        return c

    @property
    def session_key(self):
        return self.choice()[0]

    @property
    def session(self):
        return self._sessions[self.session_key]

    def current_path(self):
        base, _, name = self.choice()

        return os.path.join(garage_dir(), f"current_{base}.toml" if name is None else f"current_{base}@{safe_name(name)}.toml")

    def is_modified(self):
        return not same_knobs(self.K, self.session["knobs"])

    def load_session(self, keep_saved=False):
        """Start from the chosen session; keep_saved: resume the last garage made on it (if any)."""
        self.K = copy.deepcopy(self.session["knobs"])
        _, named, name = self.choice()
        if named is not None:
            try:
                _, self.K = gm.read_garage_file(named, self.K)
            except Exception:
                pass
        p = self.current_path()
        if keep_saved and os.path.exists(p):
            try:
                base, K = gm.read_garage_file(p, self.K)
                if base == self.session_key:
                    self.K = K
            except Exception:
                pass
        pre = f"Setup: <b>{name}</b> on the base " if name is not None else "Base setup: "
        self.base_lbl.setText(f"{pre}<b>{BASE_NAMES[self.session_key]}"
                              f"</b> &nbsp;<span style='color:gray'>({self.session['source']})</span>")
        self.refresh()

    def save_current(self):
        gm.write_garage_file(self.current_path(), self.K, base=self.session_key)

    def garage_file_for_launch(self):
        """Path for JM_GARAGE, or None when the garage is off or untouched (the sim then uses the session as is)."""
        if not self.use.isChecked() or not self.is_modified():
            return None
        self.save_current()
        return self.current_path()

    def _spin(self, key, axle=None, decimals=1, display=False):
        s = spec(key)
        lattice = s.get("step") if s.get("unit") == "mm" and abs(s.get("step", 0) - 0.396875) < 1e-9 else None
        w = KnobSpin(decimals, lattice)
        suf = s.get("unit", "")
        w.setSuffix(f" {suf}" if suf else "")
        if axle is not None and "min_front" in s:
            lo, hi = (s["min_front"], s["max_front"]) if axle == 0 else (s["min_rear"], s["max_rear"])
        else:
            lo, hi = s.get("min", -1e9), s.get("max", 1e9)
        if display:
            lo, hi = lo - 3.0, hi + 3.0
        w.setRange(lo, hi)
        w.setSingleStep(s.get("step", 1.0))
        w.setKeyboardTracking(False)
        if not s.get("known", True):
            w.setToolTip(PROVISIONAL_TIP)
        return w

    def _combo(self, key):
        s = spec(key)
        w = QComboBox()
        w.addItems(s["options"])
        if not s.get("known", True):
            w.setToolTip(PROVISIONAL_TIP)
        return w

    def _label(self, key, text=None):
        s = spec(key)
        t = (text or s["label"]) + ("†" if not s.get("known", True) else "") + ":"
        l = QLabel(t)
        if not s.get("known", True):
            l.setToolTip(PROVISIONAL_TIP)
        return l

    def _build(self):
        top = QVBoxLayout(self)
        head = QHBoxLayout()
        self.base_lbl = QLabel()
        head.addWidget(self.base_lbl, 1)
        self.use = QCheckBox("Use this garage for the next race")
        self.use.setChecked(True)
        self.use.setToolTip("Off: race the base setup exactly as the session has it, whatever is set here.")
        head.addWidget(self.use)
        top.addLayout(head)
        btns = QHBoxLayout()
        self.reset_b = QPushButton("Reset to the session")
        self.reset_b.setToolTip("Every knob back to the base session's value (the .ibt's own numbers).")
        self.reset_b.clicked.connect(self.reset)
        self.save_b = QPushButton("Save as…")
        self.save_b.clicked.connect(self.save_as)
        self.load_b = QPushButton("Load…")
        self.load_b.clicked.connect(self.load_from)
        self.classic_b = QPushButton("Import Classic setup…")
        self.classic_b.setToolTip("CLASSICSETUP-1: read a Classic Lotus 49 setup (.kj1) and carry it to reference's knobs -- pressures, springs\n"
                              "(Classic wheel rate -> reference spring rate), dampers and bars by range position, camber, toe, bias,\n"
                              "fuel, diff and the nearest reference gearing. What reference's ranges cannot take is reported.")
        self.classic_b.clicked.connect(self.import_classic)

        self.classicx_b = QPushButton("Export Classic setup…")
        self.classicx_b.setToolTip("SETUPIO-1: write this garage as a Classic Lotus 49 setup (.kj1, Classic's own file) -- pressures,\n"
                               "springs (reference spring rate -> Classic wheel rate), dampers and bars by range position, camber,\n"
                               "toe, bias, fuel, diff (ramps snap to Classic's 30/45/60/85) and Classic's nearest gear teeth.")
        self.classicx_b.clicked.connect(self.export_classic)
        self.sheet_b = QPushButton("Import reference sheet…")
        self.sheet_b.setToolTip("SETUPIO-1: read an reference Lotus 49 setup sheet (.htm, the garage's Print/Export): every knob,\n"
                                "camber and front toe as reference displays them; differences in derived numbers are reported.")
        self.sheet_b.clicked.connect(self.import_sheet)
        self.sheetx_b = QPushButton("Export reference sheet…")
        self.sheetx_b.setToolTip("SETUPIO-1: write this garage as an reference-layout setup sheet (.htm) with julia's derived numbers.")
        self.sheetx_b.clicked.connect(self.export_sheet)
        self.guide_b = QPushButton("Setup guide…")
        self.guide_b.setToolTip("What each knob does, how to work through a setup, and a table of fixes by symptom.")
        self.guide_b.clicked.connect(self.show_guide)
        for b in (self.reset_b, self.save_b, self.load_b, self.classic_b, self.classicx_b, self.sheet_b, self.sheetx_b, self.guide_b):
            btns.addWidget(b)
        btns.addStretch(1)
        self.status = QLabel()
        btns.addWidget(self.status)
        top.addLayout(btns)

        pages = QTabWidget()
        self.pages = pages
        top.addWidget(pages, 1)
        for page, name in ((self._tires_page(), "TIRES"), (self._chassis_page(), "CHASSIS"),
                           (self._drivetrain_page(), "DRIVETRAIN"), (self._temps_page(), "TYRE TEMPS")):
            sc = QScrollArea()
            sc.setWidgetResizable(True)
            sc.setWidget(page)
            pages.addTab(sc, name)
        note = QLabel("Derived numbers (in monospace) are reference's for the base session plus the garage model's change for the "
                      "knobs you turn. † = range provisional.")
        note.setStyleSheet("color: #b9c7d3")
        note.setWordWrap(True)
        top.addWidget(note)

    def show_guide(self):
        """DOCGUIDES-2: the one setup guide (docs/guides_text/setup.md) in its own window."""
        import guides
        from PyQt6.QtWidgets import QDialog, QTextBrowser
        p = guides.setup_doc()
        dlg = QDialog(self)
        dlg.setWindowTitle("Setup guide")
        dlg.resize(900, 760)
        lay = QVBoxLayout(dlg)
        t = QTextBrowser()
        t.setObjectName("guide")
        if p:
            guides.show_markdown(t, p)
        else:
            t.setPlainText("The setup guide is missing (docs/guides_text/setup.md).")
        lay.addWidget(t)
        self._guide_dlg = dlg
        dlg.show()
        return dlg

    def _derived(self, key):
        l = QLabel("—")
        l.setStyleSheet("color: #ffffff; font-family: monospace")
        self.derived_labels[key] = l
        return l

    def _tires_page(self):
        w = QWidget()
        g = QGridLayout(w)
        for i, name in enumerate(CORNERS):
            box = QGroupBox(name)
            bg = QGridLayout(box)
            bg.addWidget(self._label("pressure"), 0, 0)
            sp = self._spin("pressure", decimals=0)
            sp.valueChanged.connect(lambda v, i=i: self._set_corner("pressure", i, v))
            self.widgets[("pressure", i)] = sp
            bg.addWidget(sp, 0, 1)
            g.addWidget(box, i // 2, i % 2)
        g.setRowStretch(2, 1)
        return w

    def _temps_page(self):
        w = QWidget()
        v = QVBoxLayout(w)
        self.tt_head = QLabel("")
        self.tt_head.setWordWrap(True)
        v.addWidget(self.tt_head)
        g = QGridLayout()
        self.tt_cells = {}
        for k, (key, name) in enumerate(TT_CORNERS):
            box = QGroupBox(name)
            bg = QGridLayout(box)
            for j, zn in enumerate(("inner", "middle", "outer")):
                bg.addWidget(QLabel(zn), 0, j)
                lab = QLabel("--")
                lab.setStyleSheet("font: bold 15pt monospace")
                bg.addWidget(lab, 1, j)
                self.tt_cells[(key, j)] = lab
            hint = QLabel("")
            hint.setWordWrap(True)
            bg.addWidget(hint, 2, 0, 1, 3)
            self.tt_cells[(key, "hint")] = hint
            g.addWidget(box, k // 2, k % 2)
        v.addLayout(g)
        b = QPushButton("Refresh")
        b.clicked.connect(self.refresh_temps)
        v.addWidget(b)
        v.addStretch(1)
        self.refresh_temps()
        return w

    def showEvent(self, ev):
        self.refresh_temps()
        super().showEvent(ev)

    def refresh_temps(self):
        """Show the last run's tyres (last_tyres.json), or say there is none yet."""
        import json
        f = os.path.join(garage_dir(), "last_tyres.json")
        try:
            d = json.load(open(f))
        except (OSError, ValueError):
            self.tt_head.setText("No run yet: the tyre temperatures appear here when the car comes to rest after a run, "
                                 "or when a session ends (as a crew reads them in the pits).")
            for (key, j), lab in self.tt_cells.items():
                lab.setText("--" if j != "hint" else "")
            return None
        self.tt_head.setText("Last run: %s, %s, %s (%s, %.0f s into the session, %.1f km on these tyres). Carcass temperatures, C."
                             % (d.get("car", "?").capitalize(), d.get("track", "?"), d.get("time", "?"), d.get("why", ""),
                                d.get("session_s", 0), d.get("km", 0)))
        for key, _ in TT_CORNERS:
            t = d.get("tyres", {}).get(key)
            if not t:
                continue
            imo = t["carcass_imo"]
            for j in range(3):
                lab = self.tt_cells[(key, j)]
                lab.setText("%.0f" % imo[j])
                lab.setStyleSheet("font: bold 15pt monospace; color: %s" % tt_colour(imo[j]))
            hints = tyre_hints(imo)
            tr = t.get("tread_imo")
            if tr:
                hints = hints + ["tread left %s %%" % " / ".join("%.1f" % x for x in tr)]
            self.tt_cells[(key, "hint")].setText("\n".join(hints))
        return d

    def _chassis_page(self):
        w = QWidget()
        g = QGridLayout(w)
        front = QGroupBox("FRONT")
        fg = QGridLayout(front)
        r = 0
        for key in ("farb_arm", "farb_dia"):
            fg.addWidget(self._label(key), r, 0)
            cb = self._combo(key)
            cb.currentIndexChanged.connect(lambda idx, key=key: self._set_option(key, idx))
            self.widgets[(key, None)] = cb
            fg.addWidget(cb, r, 1)
            r += 1
        fg.addWidget(self._label("brake_bias"), r, 0)
        sp = self._spin("brake_bias", decimals=2)
        sp.valueChanged.connect(lambda v: self._set_scalar("brake_bias", v))
        self.widgets[("brake_bias", None)] = sp
        fg.addWidget(sp, r, 1)
        r += 1
        fg.addWidget(self._label("toe_front"), r, 0)
        sp = self._spin("toe_front", decimals=1, display=True)
        sp.valueChanged.connect(lambda v: self._set_display("toe_front", None, v))
        self.widgets[("toe_front", None)] = sp
        fg.addWidget(sp, r, 1)
        r += 1
        fg.addWidget(self._label("steering_ratio"), r, 0)
        cb = self._combo("steering_ratio")
        cb.currentIndexChanged.connect(lambda idx: self._set_option("steering_ratio", idx))
        self.widgets[("steering_ratio", None)] = cb
        fg.addWidget(cb, r, 1)
        r += 1
        fg.addWidget(QLabel("Cross weight:"), r, 0)
        fg.addWidget(self._derived("cross"), r, 1)
        g.addWidget(front, 0, 0, 1, 2)

        for i, name in enumerate(CORNERS):
            axle = 0 if i < 2 else 1
            box = QGroupBox(name)
            bg = QGridLayout(box)
            rr = 0
            for dk, dl in (("cw", "Corner weight"), ("rh", "Ride height"), ("shock", "Shock deflection")):
                bg.addWidget(QLabel(dl + ":"), rr, 0)
                bg.addWidget(self._derived((dk, i)), rr, 1)
                rr += 1
            for key, dec in (("perch", 1), ("spring", 0), ("packer", 1), ("bump", 0), ("rebound", 0)):
                bg.addWidget(self._label(key), rr, 0)
                sp = self._spin(key, axle=axle, decimals=dec)
                if key in ("bump", "rebound"):
                    sp.setPrefix("+")
                sp.valueChanged.connect(lambda v, key=key, i=i: self._set_corner(key, i, v))
                self.widgets[(key, i)] = sp
                bg.addWidget(sp, rr, 1)
                rr += 1
            bg.addWidget(self._label("camber"), rr, 0)
            sp = self._spin("camber", decimals=1, display=True)
            sp.valueChanged.connect(lambda v, i=i: self._set_display("camber", i, v))
            self.widgets[("camber", i)] = sp
            bg.addWidget(sp, rr, 1)
            g.addWidget(box, 1 + i // 2, i % 2)

        rear = QGroupBox("REAR")
        rg = QGridLayout(rear)
        rg.addWidget(self._label("fuel"), 0, 0)
        sp = self._spin("fuel", decimals=1)
        sp.valueChanged.connect(lambda v: self._set_scalar("fuel", v))
        self.widgets[("fuel", None)] = sp
        rg.addWidget(sp, 0, 1)
        for r, key in enumerate(("rarb_dia", "rarb_arm"), start=1):
            rg.addWidget(self._label(key), r, 0)
            cb = self._combo(key)
            cb.currentIndexChanged.connect(lambda idx, key=key: self._set_option(key, idx))
            self.widgets[(key, None)] = cb
            rg.addWidget(cb, r, 1)
        rg.addWidget(self._label("toe_rear"), 3, 0)
        sp = self._spin("toe_rear", decimals=1)
        sp.valueChanged.connect(lambda v: self._set_scalar("toe_rear", v))
        self.widgets[("toe_rear", None)] = sp
        rg.addWidget(sp, 3, 1)
        g.addWidget(rear, 3, 0, 1, 2)
        return w

    def _drivetrain_page(self):
        w = QWidget()
        g = QGridLayout(w)
        tr = QGroupBox("Transmission")
        tg = QGridLayout(tr)
        tg.addWidget(self._label("final_drive"), 0, 0)
        cb = self._combo("final_drive")
        cb.currentIndexChanged.connect(lambda idx: self._set_option("final_drive", idx))
        self.widgets[("final_drive", None)] = cb
        tg.addWidget(cb, 0, 1)
        for n in range(1, 6):
            key = f"gear{n}"
            tg.addWidget(self._label(key), n, 0)
            cb = self._combo(key)
            cb.currentIndexChanged.connect(lambda idx, n=n: self._set_gear(n, idx))
            self.widgets[(key, None)] = cb
            tg.addWidget(cb, n, 1)
            tg.addWidget(self._derived(("kmh", n - 1)), n, 2)
        g.addWidget(tr, 0, 0)
        df = QGroupBox("Differential")
        dg = QGridLayout(df)
        for r, (key, dec) in enumerate((("diff_plates", 0), ("diff_preload", 0), ("diff_drive", 0), ("diff_coast", 0))):
            dg.addWidget(self._label(key), r, 0)
            sp = self._spin(key, decimals=dec)
            sp.valueChanged.connect(lambda v, key=key: self._set_scalar(key, v))
            self.widgets[(key, None)] = sp
            dg.addWidget(sp, r, 1)
        g.addWidget(df, 1, 0)

        from engine_graphs import EngineGraphs
        self.graphs = EngineGraphs()
        g.addWidget(self.graphs, 0, 1, 3, 1)
        g.setColumnStretch(1, 1)
        g.setRowStretch(2, 1)
        return w

    def set_car(self, car):
        """The car you drive (its engine for the graphs): a '67 chassis name, any case."""
        names = {"lotus": "Lotus", "eagle": "Eagle", "ferrari": "Ferrari", "brabham": "Brabham", "cooper": "Cooper",
                 "honda": "Honda", "brm": "BRM"}
        self.car = names.get(str(car).lower(), "Lotus")
        self.car_key = str(car).lower() if str(car).lower() in names else "lotus"
        self._update_graphs()

    def _update_graphs(self):
        if getattr(self, "graphs", None) is not None and getattr(self, "K", None) is not None:
            self.graphs.update_inputs(getattr(self, "car", "Lotus"), self.K["gears"], self.K["final_drive"], gm.PARAMS["rpm_r"])

    def _edit(self, fn):
        if self._busy:
            return
        fn()
        self.save_current()
        self.refresh()

    def _set_corner(self, key, i, v):
        def f():
            vals = list(self.K[key])
            v2 = int(round(v)) if key in ("bump", "rebound") else float(v)
            sv = self.session["knobs"][key][i]
            if not isinstance(v2, int) and abs(v2 - sv) < 0.03:
                v2 = float(sv)
            vals[i] = v2
            self.K[key] = vals
        self._edit(f)

    def _set_scalar(self, key, v):
        self._edit(lambda: self.K.__setitem__(key, int(round(v)) if key == "diff_plates" else float(v)))

    def _set_option(self, key, idx):
        s = spec(key)

        def f():
            if "values" in s:
                self.K[key] = float(s["values"][idx])
            else:
                self.K[key] = s["options"][idx]
        self._edit(f)

    def _set_gear(self, n, idx):
        def f():
            gears = list(self.K["gears"])
            gears[n - 1] = float(spec(f"gear{n}")["values"][idx])
            self.K["gears"] = gears
        self._edit(f)

    def _set_display(self, key, i, v):
        """Camber and toe show reference's DISPLAYED static value; a click moves the knob by the same step."""
        def f():
            dv = float(v) - self._shown[(key, i)]
            if key == "camber":
                vals = list(self.K["camber"])
                vals[i] += dv
                self.K["camber"] = vals
            else:
                self.K["toe_front"] += dv
        self._edit(f)

    def reset(self):
        self.K = copy.deepcopy(self.session["knobs"])
        _, named, _ = self.choice()
        if named is not None:
            try:
                _, self.K = gm.read_garage_file(named, self.K)
            except Exception:
                pass
        self.save_current()
        self.refresh()

    def save_as(self):
        name, ok = QInputDialog.getText(self, "Save garage setup", "Name:")
        if not ok or not name.strip():
            return
        self.save_named(name)

    def save_named(self, name, unique=False, car=None, track=None):
        """Write the garage as a saved setup and make it the Race tab's choice (SETUPLIST-1). unique: never overwrite a
        different saved setup of that name -- an import takes 'name 2', 'name 3', ... instead. SETUPFILTER-1: the file
        names its car (default: the one you drive) and track (default: the Race tab's), and its name says the track
        ('<name> - Monza' unless the name already says one)."""
        car = car or getattr(self, "car_key", "lotus")
        track = track or (self.get_track() if self.get_track else None)
        if track and track_in_name(name) is None:
            name = f"{name.strip()} - {TRACK_TITLES.get(track, track)}"
        stem = safe_name(name) or "setup"
        fn = os.path.join(garage_dir(), stem + ".toml")
        if unique:
            n = 2
            while os.path.exists(fn):
                try:
                    b, K = gm.read_garage_file(fn)
                    if b == self.session_key and same_knobs({**self.session["knobs"], **K}, self.K):
                        break
                except Exception:
                    pass
                fn = os.path.join(garage_dir(), f"{stem} {n}.toml"); n += 1
        gm.write_garage_file(fn, self.K, base=self.session_key, car=car, track=track)
        name = os.path.splitext(os.path.basename(fn))[0]
        wc = os.path.join(garage_dir(), f"current_{self.session_key}@{safe_name(name)}.toml")
        if os.path.exists(wc):
            os.remove(wc)
        self.status.setText(f"saved {os.path.basename(fn)}")
        if self.on_setups_changed is not None:
            self.on_setups_changed(name)
        return name

    def load_from(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Load garage setup", garage_dir(), "Garage setups (*.toml)")
        if not fn:
            return
        try:
            base, K = gm.read_garage_file(fn, self.session["knobs"])
        except Exception as e:
            QMessageBox.warning(self, "Garage", f"Could not read {fn}: {e}")
            return
        if base != self.session_key:
            QMessageBox.information(self, "Garage", f"That setup was made on the '{base}' base; it is applied to "
                                                    f"'{self.session_key}' as saved.")
        self.K = K
        self.save_current()
        self.refresh()

    def import_classic_file(self, fn):
        """Translate a .kj1 onto the current base session's knobs; returns the notes (also used by the test)."""
        with open(fn, "rb") as fh:
            g = classic_setup.decode_kj1(fh.read())
        med = gm.load_toml().get("classic", {}).get("arb_median")
        K, notes = classic_setup.translate(g, self.session["knobs"], arb_median=tuple(med) if med else None)
        self.K = K
        self.refresh()

        name = self.save_named(os.path.splitext(os.path.basename(fn))[0], unique=True, car=classic_setup.car_of_file(fn),
                               track=track_in_name(os.path.basename(os.path.dirname(fn))))
        self.status.setText(f"imported {os.path.basename(fn)} as '{name}'")
        return notes

    def export_classic_file(self, fn):
        """Write the garage as a Classic .kj1; returns the notes."""
        med = gm.load_toml().get("classic", {}).get("arb_median")
        g, notes = classic_setup.export(self.K, self.session, arb_median=tuple(med) if med else None)
        with open(fn, "wb") as fh:
            fh.write(classic_setup.encode_kj1(g))
        self.status.setText(f"exported {os.path.basename(fn)}")
        return notes

    def export_classic(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Export as a Classic Lotus 49 setup", os.path.expanduser("~/julia_garage.kj1"),
                                            "Classic Lotus 49 setups (*.kj1)")
        if not fn:
            return
        fn = fn if fn.lower().endswith(".kj1") else fn + ".kj1"
        try:
            notes = self.export_classic_file(fn)
        except Exception as e:
            QMessageBox.warning(self, "Export Classic setup", f"Could not write {fn}: {e}")
            return
        QMessageBox.information(self, "Exported " + os.path.basename(fn),
                                "The garage is now a Classic Lotus 49 setup (copy it to Classic's players/<you>/setups/<track>/).\n\n" + "\n".join(notes))

    def import_sheet_file(self, fn):
        """Read an reference setup sheet into the garage; returns the notes."""
        with open(fn, encoding="latin-1") as fh:
            d = reference_sheet.parse(fh.read())
        K, notes = reference_sheet.to_knobs(d, self.session)
        self.K = K
        self.refresh()

        name = self.save_named(os.path.splitext(os.path.basename(fn))[0], unique=True, car="lotus",
                               track=track_in_name(d.get("track", "")))
        self.status.setText(f"imported {os.path.basename(fn)} as '{name}'")
        return notes

    def import_sheet(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Import an reference Lotus 49 setup sheet", os.path.expanduser("~"),
                                            "reference setup sheets (*.htm *.html *.txt);;All files (*)")
        if not fn:
            return
        try:
            notes = self.import_sheet_file(fn)
        except Exception as e:
            QMessageBox.warning(self, "Import reference sheet", f"Could not read {fn}: {e}")
            return
        QMessageBox.information(self, "Imported " + os.path.basename(fn),
                                "The sheet is now the garage (on the " + self.session_key + " base), saved as a setup you can choose on the Race tab.\n\n" +
                                ("\n".join(notes) if notes else "Every knob read; julia's derived numbers agree with the sheet's."))

    def export_sheet_file(self, fn, name=None):
        with open(fn, "w", encoding="latin-1", errors="replace") as fh:
            fh.write(reference_sheet.from_knobs(self.K, self.session, name=name or os.path.splitext(os.path.basename(fn))[0]))
        self.status.setText(f"exported {os.path.basename(fn)}")

    def export_sheet(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Export as an reference setup sheet", os.path.expanduser("~/julia_garage.htm"),
                                            "Setup sheets (*.htm)")
        if not fn:
            return
        fn = fn if fn.lower().endswith((".htm", ".html")) else fn + ".htm"
        try:
            self.export_sheet_file(fn)
        except Exception as e:
            QMessageBox.warning(self, "Export reference sheet", f"Could not write {fn}: {e}")

    def import_classic(self):
        start = os.path.expanduser("~")
        fn, _ = QFileDialog.getOpenFileName(self, "Import a Classic Lotus 49 setup", start, "Classic Lotus 49 setups (*.kj1 *.lo1);;All files (*)")
        if not fn:
            return
        try:
            notes = self.import_classic_file(fn)
        except Exception as e:
            QMessageBox.warning(self, "Import Classic setup", f"Could not read {fn}: {e}")
            return
        QMessageBox.information(self, "Imported " + os.path.basename(fn),
                                "The Classic setup is now the garage (on the " + self.session_key + " base), saved as a setup you can choose on the Race tab.\n\n" + "\n".join(notes))

    def refresh(self):
        self._update_graphs()
        self.D = gm.derived(self.K, self.session)
        D = self.D
        self._busy = True
        try:
            for (key, i), w in self.widgets.items():
                if key.startswith("gear"):
                    n = int(key[4:])
                    vals = spec(key)["values"]
                    g = self.K["gears"][n - 1]
                    j = min(range(len(vals)), key=lambda k: abs(vals[k] - g))
                    w.setCurrentIndex(j)
                elif isinstance(w, QComboBox):
                    s = spec(key)
                    cur = self.K[key]
                    if "values" in s:
                        j = min(range(len(s["values"])), key=lambda k: abs(s["values"][k] - float(cur)))
                    else:
                        j = s["options"].index(str(cur)) if str(cur) in s["options"] else 0
                    w.setCurrentIndex(j)
                elif key == "camber":
                    w.setValue(D["camber"][i]); self._shown[(key, i)] = D["camber"][i]
                elif key == "toe_front":
                    w.setValue(D["toe_front"]); self._shown[(key, i)] = D["toe_front"]
                elif i is None:
                    w.setValue(float(self.K[key]))
                else:
                    w.setValue(float(self.K[key][i]))
        finally:
            self._busy = False
        for i in range(4):
            self.derived_labels[("cw", i)].setText(f"{D['cw'][i]:.0f} N")
            loose = D["shock"][i] > D["shock_max"][i] - 0.05
            self.derived_labels[("rh", i)].setText(f"{D['rh'][i]:.1f} mm")
            self.derived_labels[("shock", i)].setText(f"{D['shock'][i]:.1f} mm of {D['shock_max'][i]:.1f} mm"
                                                      + ("  (bottomed)" if loose else ""))
        self.derived_labels["cross"].setText(f"{D['cross']:.1f}%")
        for n, v in enumerate(D["gears_kmh"]):
            self.derived_labels[("kmh", n)].setText(f"{v:.1f} Km/h")
        mod = self.is_modified()
        self.status.setText(("MODIFIED from the session" if mod else "= the session") +
                            ("" if D["ok"] else "  (solve did not converge)"))
        self.status.setStyleSheet("color: #ffb347" if mod else "color: #b9c7d3")
