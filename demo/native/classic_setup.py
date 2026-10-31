"""CLASSICSETUP-1: import a Classic Lotus 49 setup (.kj1) into julia's garage (reference's knobs).

PO 2026-10-17: "add functionality to import a Classic lotus 49 setup and translate it into a julia setup".

The .kj1 layout is classic.exe's own serialiser ('PGTS' v3; JuliaMotorMTK/tools/classicsetup_decode.jl has the decomp notes).
The translation follows WWSETUP-1's rules (PRODUCT_BACKLOG.md "WWSETUP-1"), the ones the PO's WW103 was carried to
reference with:
  * PHYSICAL where both games expose the same quantity -- tyre pressure (psi -> kPa), camber (deg), toe (in -> mm),
    brake bias (%), fuel (US gal -> L), diff ramp angles (deg) and clutch plates, the overall gearing (each gear's
    ratio x final, matched to reference's ratio lists), and the springs: Classic's WHEEL rate through the garage's measured
    spring-to-wheel ratios (front 0.81, rear 0.669) to an reference SPRING rate;
  * RANGE POSITION where the units differ -- damper clicks (Classic 1..5 -> reference 0..12) and the anti-roll bars (Classic's bar
    rate against the median of the Lotus setups on disk -> the reference bar whose garage rate stands the same way against
    the default car's).
Everything is clamped to reference's ranges (garage_lotus49.toml), and each clamp is reported, so a Classic setup outside
reference's limits says what it lost.
"""
import math
import struct

import garage as gm

LBIN = 0.175127
MRK = (0.809881, 0.668786)

CLASSIC_ARB_MEDIAN = (None, None)

def decode_kj1(data: bytes):
    if data[:4] != b"PGTS" or len(data) < 0x104:
        raise ValueError("not a Classic setup file (no 'PGTS' header)")
    f = lambda o, n: list(struct.unpack_from("<%df" % n, data, o))
    i = lambda o, n: list(struct.unpack_from("<%di" % n, data, o))
    g = f(0x0C, 9)
    return dict(gears=g[2:8], final=g[8], ramp=f(0x54, 2), psi=f(0x5C, 4), rate=f(0x6C, 4), bump=i(0x7C, 4),
                rebound=i(0x8C, 4), camber=f(0x9C, 4), rubber=f(0xCC, 4), toe=f(0xDC, 2), arb=f(0xE4, 2),
                ride=[min(max(x, 2.5), 5.0) for x in f(0xEC, 2)], clutches=i(0xF4, 1)[0], fuel_gal=f(0xF8, 1)[0],
                steer=f(0xFC, 1)[0], bias=i(0x100, 1)[0])

def _spec(key):
    return next(k for k in gm.KNOB_SPECS if k["key"] == key)

def _clamp_step(v, lo, hi, step, base=0.0):
    c = min(max(v, lo), hi)
    return base + round((c - base) / step) * step, (c != v)

def translate(classic, base_knobs, arb_median=None):
    """Classic setup (decode_kj1) -> (garage knobs, notes). `base_knobs` supplies every knob Classic has no equivalent for."""
    K = {k: (list(v) if isinstance(v, list) else v) for k, v in base_knobs.items()}
    notes = []

    def note_clamp(name, clamped, want, got, unit=""):
        if clamped:
            notes.append(f"{name}: Classic's {want:g}{unit} is outside reference's range -> {got:g}{unit}")

    sp = _spec("pressure")
    pr = []
    for i, psi in enumerate(classic["psi"]):
        v, c = _clamp_step(psi * 6.895, sp["min"], sp["max"], sp["step"], sp["min"])
        note_clamp(f"pressure {('LF', 'RF', 'LR', 'RR')[i]}", c, psi * 6.895, v, " kPa")
        pr.append(round(v, 1))
    K["pressure"] = pr

    ss = _spec("spring")
    springs = []
    for i, r in enumerate(classic["rate"]):
        a = 0 if i < 2 else 1
        want = r * LBIN / MRK[a] ** 2
        lo, hi = (ss["min_front"], ss["max_front"]) if a == 0 else (ss["min_rear"], ss["max_rear"])
        v, c = _clamp_step(want, lo, hi, ss["step"])
        note_clamp(f"spring {('LF', 'RF', 'LR', 'RR')[i]}", c, round(want, 1), v, " N/mm")
        springs.append(float(v))
    K["spring"] = springs

    K["bump"] = [int(round((min(max(c, 1), 5) - 1) / 4 * 12)) for c in classic["bump"]]
    K["rebound"] = [int(round((min(max(c, 1), 5) - 1) / 4 * 12)) for c in classic["rebound"]]

    cs = _spec("camber")
    K["camber"] = [round(min(max(c, cs["min"]), cs["max"]), 1) for c in classic["camber"]]
    for key, v in (("toe_front", classic["toe"][0] * 25.4), ("toe_rear", classic["toe"][1] * 25.4)):
        s = _spec(key)
        got, c = _clamp_step(v, s["min"], s["max"], s["step"])
        note_clamp(key.replace("_", " "), c, round(v, 1), got, " mm")
        K[key] = float(got)

    sb = _spec("brake_bias")
    K["brake_bias"], c = _clamp_step(float(classic["bias"]), sb["min"], sb["max"], sb["step"])
    note_clamp("brake bias", c, float(classic["bias"]), K["brake_bias"], " %")
    sf = _spec("fuel")
    fuel = classic["fuel_gal"] * 3.785
    K["fuel"], c = _clamp_step(fuel, sf["min"], sf["max"], sf["step"])
    K["fuel"] = round(K["fuel"], 1)
    note_clamp("fuel", c, round(fuel, 1), K["fuel"], " L")

    for key, v in (("diff_drive", classic["ramp"][0]), ("diff_coast", classic["ramp"][1])):
        s = _spec(key)
        got, c = _clamp_step(v, s["min"], s["max"], s["step"], s["min"])
        note_clamp(key.replace("_", " "), c, v, got, " deg")
        K[key] = float(got)
    s = _spec("diff_plates")
    K["diff_plates"], c = _clamp_step(classic["clutches"], s["min"], s["max"], 1)
    K["diff_plates"] = int(K["diff_plates"])
    note_clamp("clutch plates", c, classic["clutches"], K["diff_plates"])

    finals = _spec("final_drive")["values"]
    glists = [_spec(f"gear{n}")["values"] for n in range(1, 6)]
    want = [g * classic["final"] for g in classic["gears"][:5]]
    best = None
    for fd in finals:
        pick = [min(gl, key=lambda r: abs(math.log(r * fd / w))) for gl, w in zip(glists, want)]
        err = sum(abs(math.log(r * fd / w)) for r, w in zip(pick, want))
        if best is None or err < best[0]:
            best = (err, fd, pick)
    K["final_drive"], K["gears"] = float(best[1]), [float(r) for r in best[2]]
    kmh_g = [gm.PARAMS["rpm_r"] * 2 * math.pi / 60 / w * 3.6 for w in want]
    kmh_i = gm.gear_speeds_kmh(K)
    notes.append("gears (top speed, Classic -> reference's nearest ratios): " +
                 ", ".join(f"{a:.0f}->{b:.0f}" for a, b in zip(kmh_g, kmh_i)) + " km/h")

    med = arb_median or CLASSIC_ARB_MEDIAN
    if med[0] and med[1]:
        for axle, (dk, ak) in enumerate((("farb_dia", "farb_arm"), ("rarb_dia", "rarb_arm"))):
            rel = classic["arb"][axle] / med[axle]
            ref = gm.arb_rate(gm.PARAMS, axle, *gm.ARB_REF[axle][:1], str(gm.ARB_REF[axle][1]))
            target = ref * rel
            opts = [(d, a) for d in _spec(dk)["values"] for a in _spec(ak)["options"]]
            d, a = min(opts, key=lambda o: abs(gm.arb_rate(gm.PARAMS, axle, o[0], o[1]) - target))
            K[dk], K[ak] = float(d), a
            notes.append(f"{('front', 'rear')[axle]} bar: Classic {classic['arb'][axle]:g} lb/in = {rel:.2f}x the Classic median -> "
                         f"{d:g}\" arms {a} ({gm.arb_rate(gm.PARAMS, axle, d, a) / ref:.2f}x the default's)")

    st = min(_spec("steering_ratio")["values"], key=lambda v: abs(v - classic["steer"]))
    if abs(st - classic["steer"]) <= 0.5:
        K["steering_ratio"] = float(st)
    else:
        notes.append(f"steering: Classic {classic['steer']:g}:1 -- kept the base's {K['steering_ratio']:g}:1 (reference's Lotus "
                     f"offers 8:1 to 16:1, its steps in between not yet known)")

    target = gm.solve_static(base_knobs)["rh"]
    ps = _spec("perch")
    for _ in range(12):
        rh = gm.solve_static(K)["rh"]
        perch = list(K["perch"])
        for i in range(4):
            lo, hi = (ps["min_front"], ps["max_front"]) if i < 2 else (ps["min_rear"], ps["max_rear"])
            perch[i] = min(max(perch[i] + (rh[i] - target[i]) / 0.8, lo), hi)
        K["perch"] = perch
    K["perch"] = [round(p / ps["step"]) * ps["step"] for p in K["perch"]]
    rh = gm.solve_static(K)["rh"]
    notes.append("perches re-set to keep the base car's ride heights: " + "/".join(f"{p:.1f}" for p in K["perch"]) +
                 " mm (ride heights " + "/".join(f"{x:.1f}" for x in rh) + " mm)")
    notes.append("not carried: Classic's ride height and bump rubbers (measured differently)")
    return K, notes

CLASSIC_TEETH = ((13, 46), (14, 46), (14, 45), (12, 35), (13, 35), (16, 42), (15, 38), (14, 34), (14, 33), (14, 32),
             (16, 42), (15, 38), (14, 34), (17, 41), (14, 33), (14, 32), (15, 33), (16, 31), (15, 32), (15, 31),
             (16, 32), (16, 30), (17, 31), (17, 30), (17, 29), (14, 34), (17, 41), (14, 33), (14, 32), (15, 33),
             (15, 32), (15, 31), (16, 32), (16, 31), (20, 38), (16, 30), (17, 31), (17, 30), (17, 29), (20, 34),
             (18, 30), (18, 29), (18, 28), (19, 29), (21, 32), (19, 28), (18, 26), (19, 27), (24, 34), (20, 28),
             (21, 29), (24, 33), (25, 34), (20, 27), (24, 32), (25, 33), (19, 25), (23, 30), (21, 27), (25, 32),
             (26, 33), (19, 24), (23, 29), (25, 31), (21, 26), (26, 32), (19, 23), (24, 29), (27, 32), (22, 26),
             (25, 29), (19, 22), (27, 31), (28, 32), (22, 25), (25, 28), (27, 30), (28, 31), (29, 32), (22, 24),
             (24, 26), (25, 27), (23, 24), (24, 24), (25, 24), (26, 24), (7, 31), (8, 31), (9, 31), (10, 31))
CLASSIC_GEAR_IDX = range(0, 86)
CLASSIC_FINAL_IDX = range(86, 90)
CLASSIC_RAMPS = (30.0, 45.0, 60.0, 85.0)
teeth_ratio = lambda k: CLASSIC_TEETH[k][1] / CLASSIC_TEETH[k][0]

def encode_kj1(g: dict) -> bytes:
    """A Classic 'PGTS' v3 setup file from a dict with decode_kj1's fields plus gear_idx (9) -- gears = the 9 ratio slots."""
    p = bytearray()
    p += struct.pack("<9f", *g["ratios"]) + struct.pack("<9i", *g["gear_idx"])
    p += struct.pack("<2f", *g["ramp"]) + struct.pack("<4f", *g["psi"]) + struct.pack("<4f", *g["rate"])
    p += struct.pack("<4i", *g["bump"]) + struct.pack("<4i", *g["rebound"]) + struct.pack("<4f", *g["camber"])
    p += struct.pack("<4f", 800.0, 800.0, 800.0, 800.0) + struct.pack("<4f", 7.0, 7.0, 7.0, 7.0)
    p += struct.pack("<4f", *g["rubber"]) + struct.pack("<2f", *g["toe"]) + struct.pack("<2f", *g["arb"])
    p += struct.pack("<2f", *g["ride"]) + struct.pack("<i", g["clutches"]) + struct.pack("<f", g["fuel_gal"])
    p += struct.pack("<f", g["steer"]) + struct.pack("<i", g["bias"]) + bytes([g.get("flag", 0)])
    assert len(p) == 249
    b = b"PGTS" + struct.pack("<ii", 3, len(p)) + bytes(p)
    return b + b" " * ((-len(b)) % 4)

def export(K, session, arb_median=None):
    """Garage knobs K (on `session`) -> (Classic setup dict for encode_kj1, notes). The inverse of translate(): physical
    where both games share the quantity, range position where the units differ, Classic's own tooth table for the gearing."""
    notes = []
    D = gm.derived(K, session)
    r = lambda x, q: round(x / q) * q

    want = [g * K["final_drive"] for g in K["gears"]]
    best = None
    for fi in CLASSIC_FINAL_IDX:
        fd = teeth_ratio(fi)
        pick = [min(CLASSIC_GEAR_IDX, key=lambda k: abs(math.log(teeth_ratio(k) * fd / w))) for w in want]
        err = sum(abs(math.log(teeth_ratio(k) * fd / w)) for k, w in zip(pick, want))
        if best is None or err < best[0]:
            best = (err, fi, pick)
    _, fi, pick = best
    sixth = pick[-1] if teeth_ratio(pick[-1]) <= 1.0 else 83
    idx = [4, 0] + pick + [sixth, fi]
    ratios = [-teeth_ratio(4), 0.0] + [teeth_ratio(k) for k in pick] + [teeth_ratio(sixth), teeth_ratio(fi)]
    kmh = lambda ov: gm.PARAMS["rpm_r"] * 2 * math.pi / 60 / ov * 3.6
    notes.append("gears (top speed, reference -> Classic's nearest teeth): " +
                 ", ".join(f"{kmh(w):.0f}->{kmh(teeth_ratio(k) * teeth_ratio(fi)):.0f}" for k, w in zip(pick, want)) +
                 f" km/h, final {CLASSIC_TEETH[fi][1]}/{CLASSIC_TEETH[fi][0]}")

    ramp = [min(CLASSIC_RAMPS, key=lambda a: abs(a - float(K[k]))) for k in ("diff_drive", "diff_coast")]
    for k, a in zip(("diff_drive", "diff_coast"), ramp):
        abs(a - float(K[k])) > 0.1 and notes.append(f"{k.replace('_', ' ')}: reference {K[k]:g} deg -> Classic {a:g} (Classic offers 30/45/60/85)")
    clutches = int(min(max(int(K["diff_plates"]), 1), 6))
    notes.append(f"diff preload {K['diff_preload']:g} Nm: Classic has none")

    psi = [min(max(r(p / 6.895, 0.25), 10.0), 30.0) for p in K["pressure"]]
    rate = [min(max(round(s * MRK[0 if i < 2 else 1] ** 2 / LBIN), 50 if i < 2 else 70), 125 if i < 2 else 160)
            for i, s in enumerate(K["spring"])]
    for i, s in enumerate(K["spring"]):
        want_r = s * MRK[0 if i < 2 else 1] ** 2 / LBIN
        abs(want_r - rate[i]) > 1 and notes.append(f"wheel rate {('LF', 'RF', 'LR', 'RR')[i]}: {want_r:.0f} lb/in is outside Classic's "
                                                   f"range -> {rate[i]}")
    damp = lambda c: int(min(max(round(1 + c / 12 * 4), 1), 5))
    camber = [round(min(max(c, -3.0), 3.0), 2) for c in D["camber"]]
    toe = [round(D["toe_front"] / 25.4, 3), round(K["toe_rear"] / 25.4, 3)]

    med = arb_median or CLASSIC_ARB_MEDIAN
    arb = [150.0, 150.0]
    if med[0] and med[1]:
        for axle, (dk, ak) in enumerate((("farb_dia", "farb_arm"), ("rarb_dia", "rarb_arm"))):
            ref = gm.arb_rate(gm.PARAMS, axle, *gm.ARB_REF[axle][:1], str(gm.ARB_REF[axle][1]))
            rel = gm.arb_rate(gm.PARAMS, axle, float(K[dk]), K[ak]) / ref
            arb[axle] = float(min(max(r(rel * med[axle], 5.0), 40.0), 300.0))
            notes.append(f"{('front', 'rear')[axle]} bar: {K[dk]:g}\" arms {K[ak]} = {rel:.2f}x the default's -> Classic {arb[axle]:g} lb/in")

    ride = [min(max(round((D["rh"][0] + D["rh"][1]) / 2 / 25.4, 2), 2.5), 5.0),
            min(max(round((D["rh"][2] + D["rh"][3]) / 2 / 25.4, 2), 2.5), 5.0)]
    notes.append(f"static ride {ride[0]:g}/{ride[1]:g} in from reference's ride heights (Classic measures its own way); bump rubbers 1.0 in")
    bias = int(round(float(K["brake_bias"])))
    abs(bias - float(K["brake_bias"])) > 1e-6 and notes.append(f"brake bias {K['brake_bias']:g} % -> {bias} (Classic: whole percent)")
    g = dict(ratios=ratios, gear_idx=idx, ramp=ramp, psi=psi, rate=[float(x) for x in rate],
             bump=[damp(c) for c in K["bump"]], rebound=[damp(c) for c in K["rebound"]], camber=camber,
             rubber=[1.0] * 4, toe=toe, arb=arb, ride=ride, clutches=clutches,
             fuel_gal=round(float(K["fuel"]) / 3.785, 2), steer=float(K["steering_ratio"]), bias=bias, flag=0)
    return g, notes

CLASSIC_SETUP_EXT = {"lotus": ".kj1", "eagle": ".kg1", "ferrari": ".ki1", "brabham": ".ka1", "cooper": ".ke1",
                 "brm": ".kc1", "honda": ".kn1"}
CLASSIC_TRACK_DIR = {"zandvoort": "zandvort", "nurburgring": "nurburg", "watglen": "watglen", "monza": "monza", "spa": "spa"}

def car_of_file(path):
    """SETUPFILTER-1: the car a Classic setup file is for, by its extension (.kj1 = lotus, ...); None if not a Classic setup."""
    import os
    e = os.path.splitext(path)[1].lower()
    return next((c for c, x in CLASSIC_SETUP_EXT.items() if x == e), None)

def find_classic_setups_all(car, first=None, root=None):
    """SETUPFILTER-1: every Classic setup for car on every track -> [(label, path, mod, track)], the track `first` first.
    Without a Classic install's setups (classic_setup_root), the translated ones this repository ships (shipped_classic_setups)."""
    order = sorted(CLASSIC_TRACK_DIR, key=lambda t: (t != first, list(CLASSIC_TRACK_DIR).index(t)))
    import os
    if root is None and not os.path.isdir(classic_setup_root()):
        sh = [e for e in shipped_classic_setups() if e["car"] == car]
        return [(e["stem"], e["path"], e["mod"], t) for t in order for e in sh if e["track"] == t]
    return [(s, p, m, t) for t in order for (s, p, m) in find_classic_setups(car, t, root)]

_SHIPPED = None
def shipped_classic_setups():
    """The Classic setups translated ahead into julia racer garage files (data/classic_setups/index.json: stem, mod, track, car,
    toml, tip, notes), each entry with its file's full "path"; [] when there are none."""
    global _SHIPPED
    if _SHIPPED is None:
        import json, os
        d = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "classic_setups"))
        try:
            with open(os.path.join(d, "index.json"), encoding="utf-8") as fh:
                _SHIPPED = [dict(e, path=os.path.join(d, e["toml"])) for e in json.load(fh)]
        except (OSError, ValueError):
            _SHIPPED = []
    return _SHIPPED

def shipped_classic_entry(path):
    """The shipped translated setup at `path`, or None."""
    return next((e for e in shipped_classic_setups() if e["path"] == path), None)

def classic_setup_root():
    """The Classic setups (INSTALL/<mod>__Driver/setups/<track>): JM_CLASSIC_SETUPS_ROOT, else this repository's classic/INSTALL
    (install.sh), else the serious-games-lab layout this folder may sit in: THU/INSTALL beside the THU/WP that holds Classic
    (classicroot.jl's fallback; a dev tree's THU/WP is a link to the install, so INSTALL is found beside its target)."""
    import os
    if os.environ.get("JM_CLASSIC_SETUPS_ROOT"):
        return os.environ["JM_CLASSIC_SETUPS_ROOT"]
    here = os.path.dirname(os.path.abspath(__file__))
    own = os.path.normpath(os.path.join(here, "..", "..", "classic", "INSTALL"))
    if os.path.isdir(own):
        return own
    wp = os.path.join(here, "..", "..", "..", "..", "WP")
    rel = os.path.normpath(os.path.join(os.path.realpath(wp), "..", "INSTALL"))
    return rel if os.path.isdir(rel) else own

def find_classic_setups(car, track, root=None):
    """-> [(label, path, mod)] for car (a PLAYER_CARS key) on track (a TRACK_KEYS key), duplicates (same bytes) once,
    sorted by name; [] when Classic or the track has none (the skidpad)."""
    import glob, hashlib, os
    ext = CLASSIC_SETUP_EXT.get(car); tdir = CLASSIC_TRACK_DIR.get(track)
    root = root or classic_setup_root()
    if not ext or not tdir or not os.path.isdir(root):
        return []
    dirs = []
    for pat in ("*/setups", "*/setups/setups", "old/*/setups"):
        for s in sorted(glob.glob(os.path.join(root, pat))):
            for d in os.listdir(s):
                if d.lower() == tdir and os.path.isdir(os.path.join(s, d)):
                    dirs.append(os.path.join(s, d))
    seen, out = set(), []
    for d in dirs:
        rel = os.path.relpath(d, root).split(os.sep)
        mod = "/".join(p for p in rel if p not in ("setups", rel[-1])).replace("__Driver", "")
        for fn in sorted(os.listdir(d)):
            if not fn.lower().endswith(ext):
                continue
            p = os.path.join(d, fn)
            try:
                with open(p, "rb") as fh:
                    b = fh.read()
                decode_kj1(b)
            except Exception:
                continue
            h = hashlib.md5(b).hexdigest()
            if h in seen:
                continue
            seen.add(h)
            out.append((os.path.splitext(fn)[0], p, mod))
    out.sort(key=lambda e: (e[0].lower(), e[2]))
    return out

def classic_setup_summary(path):
    """One line for the list's tooltip: overall top gear, final drive, pressures, wheel rates, bars, fuel."""
    with open(path, "rb") as fh:
        g = decode_kj1(fh.read())
    return (f"gears {'/'.join(f'{r:.2f}' for r in g['gears'] if r > 0)} x final {g['final']:.3f}; psi {'/'.join(f'{p:g}' for p in g['psi'])}; "
            f"wheel rates {'/'.join(f'{r:g}' for r in g['rate'])} lb/in; bars {g['arb'][0]:g}/{g['arb'][1]:g} lb/in; "
            f"fuel {g['fuel_gal']:g} gal")
