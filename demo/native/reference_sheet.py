"""SETUPIO-1: reference Lotus 49 setup SHEETS in and out of julia's garage.

PO 2026-10-19: "implement in julia racer Classic setup import/export and reference sheet import/export". An reference setup sheet
is the garage's own export (Garage -> Print/Export, an .htm): every knob and every derived number of the three garage
screens in plain text, unlike the encrypted .sto (memory jr-garage-261017). Layout (the PO's 31 sheets of 261018):
  LEFT FRONT/LEFT REAR/RIGHT FRONT/RIGHT REAR (tyres): Cold pressure, Last hot pressure, Last temps, Tread remaining
  FRONT: ARB arms, ARB diameter, Brake bias, Toe-in, Steering ratio, Cross weight
  LEFT FRONT .. RIGHT REAR (chassis): Corner weight, Ride height, Shock deflection "x of y", Spring perch offset,
      Spring rate, Packer, Bump stiffness "+n clicks", Rebound stiffness, Camber
  REAR: Fuel level, ARB diameter, ARB arms, Toe-in
  Transmission: Final drive, First..Fifth Gear "ratio  speed Km/h";  Differential: Clutch plates, Preload, Drive/Coast
      ramp angle
Values carry their unit (an imperial reference writes psi, in, lbs/in, gal, ft-lbs), so the parser converts by unit.
Camber and front toe on a sheet are what reference DISPLAYS (the garage's static solve), as julia's Garage tab shows them;
import solves the knob that displays that value.
"""
import html
import math
import re

import garage as gm

CORNERS = ("LEFT FRONT", "RIGHT FRONT", "LEFT REAR", "RIGHT REAR")
_HEAD = re.compile(r"^(LEFT FRONT|LEFT REAR|RIGHT FRONT|RIGHT REAR|FRONT|REAR|Transmission|Differential):$")

def parse(text):
    """Sheet text (.htm) -> {"name", "track", ("TIRES"|"CHASSIS"|"FRONT"|..., label): value string}."""
    t = re.sub(r"<br\s*/?>", "\n", text, flags=re.I)
    t = re.sub(r"<[^>]+>", " ", t)
    t = html.unescape(t).split(" Notes:")[0].split("\nNotes:")[0]
    d, sec, seen, key = {}, None, set(), None
    for raw in t.split("\n"):
        ln = re.sub(r"[ \t]+", " ", raw).strip()
        if not ln:
            continue
        m = re.match(r"(\S+) setup: (.*)", ln)
        if m:
            d["car"] = m.group(1); d["name"] = m.group(2).strip(); continue
        m = re.match(r"track: (.*)", ln)
        if m:
            d["track"] = m.group(1).strip(); continue
        m = _HEAD.match(ln)
        if m:
            s = m.group(1)
            sec = ("CHASSIS " + s) if (s in seen and s in CORNERS) else (("TIRES " + s) if s in CORNERS else s)
            seen.add(s); key = None; continue
        if sec is None:
            continue
        k, sep, v = ln.partition(":")
        if sep and not re.fullmatch(r"[-+\d.]+ ?\S*", k):
            key = (sec, k.strip().lower()); d[key] = v.strip()
        elif key:
            d[key] += " " + ln
    return d

def _num(s):
    m = re.search(r"[-+]?\d+(?:\.\d+)?", s or "")
    return float(m.group()) if m else float("nan")

def _unit(s):
    m = re.search(r"[-+]?\d+(?:\.\d+)?\s*([A-Za-z%\"/:-]+(?:/[A-Za-z]+)?)", s or "")
    return m.group(1).lower() if m else ""

def _si(s, kind):
    """A sheet value with its unit -> julia's garage unit (kPa, mm, N/mm, L, Nm, deg)."""
    v, u = _num(s), _unit(s)
    conv = {"pressure": {"psi": 6.894757}, "length": {"in": 25.4, '"': 25.4, "cm": 10.0},
            "spring": {"lbs/in": 0.175127, "lb/in": 0.175127}, "fuel": {"gal": 3.785412}, "torque": {"ft-lbs": 1.355818}}
    return v * conv.get(kind, {}).get(u, 1.0)

def to_knobs(d, session, base_knobs=None):
    """Parsed sheet -> (garage knobs K, notes). Knobs a sheet lacks keep the base session's value."""
    K = {k: (list(v) if isinstance(v, list) else v) for k, v in (base_knobs or session["knobs"]).items()}
    notes = []
    g = lambda sec, lab: d.get((sec, lab))
    for i, c in enumerate(CORNERS):
        if g("TIRES " + c, "cold pressure"):
            K["pressure"][i] = round(_si(g("TIRES " + c, "cold pressure"), "pressure"), 1)
        ch = "CHASSIS " + c
        for key, lab, kind in (("perch", "spring perch offset", "length"), ("spring", "spring rate", "spring"),
                               ("packer", "packer", "length")):
            g(ch, lab) and K[key].__setitem__(i, round(_si(g(ch, lab), kind), 3))
        for key, lab in (("bump", "bump stiffness"), ("rebound", "rebound stiffness")):
            g(ch, lab) and K[key].__setitem__(i, int(round(_num(g(ch, lab)))))
    for sec, dk, ak in (("FRONT", "farb_dia", "farb_arm"), ("REAR", "rarb_dia", "rarb_arm")):
        if g(sec, "arb diameter"):
            s = g(sec, "arb diameter"); K[dk] = 0.0 if "disc" in s.lower() else _num(s)
        if g(sec, "arb arms"):
            K[ak] = g(sec, "arb arms").strip().lower()
    g("FRONT", "brake bias") and K.__setitem__("brake_bias", _num(g("FRONT", "brake bias")))
    g("REAR", "toe-in") and K.__setitem__("toe_rear", round(_si(g("REAR", "toe-in"), "length"), 2))
    g("REAR", "fuel level") and K.__setitem__("fuel", round(_si(g("REAR", "fuel level"), "fuel"), 1))
    g("FRONT", "steering ratio") and K.__setitem__("steering_ratio", _num(g("FRONT", "steering ratio")))
    if g("Transmission", "final drive"):
        K["final_drive"] = _num(g("Transmission", "final drive"))
        gears = []
        for n in ("first", "second", "third", "fourth", "fifth"):
            s = g("Transmission", n + " gear")
            gears.append(_num(s) if s else K["gears"][len(gears)])
        K["gears"] = gears
    for key, lab, kind in (("diff_plates", "clutch plates", None), ("diff_preload", "preload", "torque"),
                           ("diff_drive", "drive ramp angle", None), ("diff_coast", "coast ramp angle", None)):
        s = g("Differential", lab)
        if s:
            v = _si(s, kind) if kind else _num(s)
            K[key] = int(round(v)) if key == "diff_plates" else round(v, 1)

    want_c = [(_num(g("CHASSIS " + c, "camber")) if g("CHASSIS " + c, "camber") else None) for c in CORNERS]
    want_t = _si(g("FRONT", "toe-in"), "length") if g("FRONT", "toe-in") else None
    for _ in range(4):
        D = gm.derived(K, session)
        K["camber"] = [K["camber"][i] + (w - D["camber"][i]) if w is not None else K["camber"][i] for i, w in enumerate(want_c)]
        if want_t is not None:
            K["toe_front"] += want_t - D["toe_front"]

    D = gm.derived(K, session)
    for i, c in enumerate(CORNERS):
        s = g("CHASSIS " + c, "ride height")
        if s and abs(_si(s, "length") - D["rh"][i]) > 1.5:
            notes.append(f"ride height {c.lower()}: sheet {_si(s, 'length'):.1f} mm, julia's garage {D['rh'][i]:.1f} mm")
    for key in gm.KNOB_FIELDS:
        spec = next((k for k in gm.KNOB_SPECS if k["key"] == key), None)
        if spec and "min" in spec and not isinstance(K[key], list):
            tol = spec.get("step", 0.0)/2
            if not (spec["min"] - tol <= float(K[key]) <= spec["max"] + tol):
                notes.append(f"{key}: {K[key]:g} outside {spec['min']}..{spec['max']}")
            K[key] = min(max(float(K[key]), spec["min"]), spec["max"]) if key not in ("diff_plates",) else K[key]
    return K, notes

def _fmt(v, d):
    return f"{v:.{d}f}"

def from_knobs(K, session, name="julia garage", track=""):
    """Garage knobs -> an reference-layout setup sheet (.htm) with julia's derived numbers (corner weights, ride heights,
    shock deflections, displayed camber/toe, gear speeds). Readable by parse() and laid out as reference's own export."""
    D = gm.derived(K, session)
    e = html.escape
    out = ["<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0 Transitional//EN\">", "<html>", "<head>",
           "<title>julia racer Car Setup</title>", "<meta name=\"GENERATOR\" content=\"julia racer (SETUPIO-1)\">", "</head>",
           "<body>", f"<H2 align=\"center\">julia racer garage<br>lotus49 setup: {e(name)}<br>track: {e(track)}</H2><br><br>"]
    for i, c in (0, "LEFT FRONT"), (2, "LEFT REAR"), (1, "RIGHT FRONT"), (3, "RIGHT REAR"):
        out.append(f"<H2><U>{c}:</U></H2>")
        out.append(f"Cold pressure:<U>{K['pressure'][i]:.0f} kPa</U><br><br>")
    arm = lambda a: str(a)
    dia = lambda v: "Disconnected" if float(v) <= 0 else f"{float(v):g}\""
    out.append("<H2><U>FRONT:</U></H2>")
    out.append(f"ARB arms:<U>{arm(K['farb_arm'])}</U><br>ARB diameter:<U>{dia(K['farb_dia'])}</U><br>"
               f"Brake bias:<U>{float(K['brake_bias']):.2f}%</U><br>Toe-in:<U>{D['toe_front']:+.1f} mm</U><br>"
               f"Steering ratio:<U>{float(K['steering_ratio']):g}:1</U><br>Cross weight:<U>{D['cross']:.1f}%</U><br><br>")
    for i, c in (0, "LEFT FRONT"), (2, "LEFT REAR"), (1, "RIGHT FRONT"), (3, "RIGHT REAR"):
        out.append(f"<H2><U>{c}:</U></H2>")
        out.append(f"Corner weight:<U>{D['cw'][i]:.0f} N</U><br>Ride height:<U>{D['rh'][i]:.1f} mm</U><br>"
                   f"Shock deflection:<U>{D['shock'][i]:.1f} mm</U><U>of {D['shock_max'][i]:.1f} mm</U><br>"
                   f"Spring perch offset:<U>{K['perch'][i]:.1f} mm</U><br>Spring rate:<U>{K['spring'][i]:.0f} N/mm</U><br>"
                   f"Packer:<U>{K['packer'][i]:.1f} mm</U><br>Bump stiffness:<U>{int(K['bump'][i]):+d} clicks</U><br>"
                   f"Rebound stiffness:<U>{int(K['rebound'][i]):+d} clicks</U><br>Camber:<U>{D['camber'][i]:+.1f} deg</U><br><br>")
    out.append("<H2><U>REAR:</U></H2>")
    out.append(f"Fuel level:<U>{float(K['fuel']):.1f} L</U><br>ARB diameter:<U>{dia(K['rarb_dia'])}</U><br>"
               f"ARB arms:<U>{arm(K['rarb_arm'])}</U><br>Toe-in:<U>{float(K['toe_rear']):+.0f} mm</U><br><br>")
    out.append("<H2><U>Transmission:</U></H2>")
    gl = f"Final drive:<U>{float(K['final_drive']):.2f}</U><br>"
    for n, gr, v in zip(("First", "Second", "Third", "Fourth", "Fifth"), K["gears"], D["gears_kmh"]):
        gl += f"{n} Gear:<U>{gr:g}</U><U>{v:.1f} Km/h</U><br>"
    out.append(gl + "<br>")
    out.append("<H2><U>Differential:</U></H2>")
    out.append(f"Clutch plates:<U>{int(K['diff_plates'])}</U><br>Preload:<U>{float(K['diff_preload']):.0f} Nm</U><br>"
               f"Drive ramp angle:<U>{float(K['diff_drive']):.0f} deg</U><br>Coast ramp angle:<U>{float(K['diff_coast']):.0f} deg</U><br><br>")
    out.append("<H2><U>Notes:</U></H2>Written by julia racer's Garage tab: the knobs as reference's garage sets them; corner "
               "weights, ride heights, shock deflections, camber, front toe, cross weight and gear speeds are julia's garage "
               "model of reference's (the base session's numbers plus the model's change).<br></body></html>")
    return "\n".join(out)
