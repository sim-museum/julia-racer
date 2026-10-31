"""SETUPMODEL-1: the launcher's copy of the Lotus 49 garage solve (JuliaMotorMTK/src/garage.jl).

reference's garage turns setup KNOBS (perch offsets, springs, packers, bars, fuel, camber, toe, pressures) into DERIVED
numbers (corner weights, ride heights, shock deflections, static camber and toe, cross weight). The physics lives in
Julia; this mirror exists only so the Garage tab can show the derived numbers live as a knob turns. Both read
JuliaMotorMTK/src/garage_lotus49.toml (constants + reference's knob list), and tools/garage_smoke.jl fails if the two
solvers disagree.

What the tab shows (and the sim installs) is the SESSION's own numbers -- reference's, from the .ibt -- plus the model's
change from the session's knobs to the current ones: an untouched garage is the session to the digit.
"""
import copy
import math
import os
import tomllib

G = 9.80665
HERE = os.path.dirname(os.path.abspath(__file__))
MTK_SRC = os.path.normpath(os.path.join(HERE, "..", "..", "JuliaMotorMTK", "src"))
GARAGE_TOML = os.path.join(MTK_SRC, "garage_lotus49.toml")
SESSIONS_TOML = os.path.join(MTK_SRC, "garage_sessions.toml")

KNOB_FIELDS = ("perch", "spring", "packer", "camber", "pressure", "bump", "rebound", "toe_front", "toe_rear",
               "farb_dia", "farb_arm", "rarb_dia", "rarb_arm", "fuel", "brake_bias", "steering_ratio", "final_drive",
               "gears", "diff_plates", "diff_preload", "diff_drive", "diff_coast")
SHOCK_MAX = (104.6, 113.0)
ARB_REF = ((0.375, 2), (0.6875, 5))

def load_toml(path=GARAGE_TOML):
    with open(path, "rb") as f:
        return tomllib.load(f)

_CFG = load_toml()
PARAMS = _CFG["params"]
KNOB_SPECS = _CFG["knob"]

def sessions():
    """{"default": {"knobs": {...}, "derived": {...}, "source": ...}, "ww103": {...}}"""
    return load_toml(SESSIONS_TOML)

def armhole(a):
    s = str(a).strip().lower()
    return 1 if s == "soft" else 7 if s == "firm" else int(s)

def arb_rate(P, axle, dia, arm):
    if dia <= 0:
        return 0.0
    dref, href = ARB_REF[axle]
    return P["karb"][axle] * (dia / dref) ** 4 * max(0.05, 1 + P["arm_beta"][axle] * (armhole(arm) - href)) ** 2

def _axle(i):
    return 0 if i < 2 else 1

def _side(i):
    return 1.0 if i % 2 == 0 else -1.0

def _other(i):
    return i + 1 if i % 2 == 0 else i - 1

def corner_force(P, K, i, z):
    a = _axle(i)
    MR, MRq = P["MR"][a], P["MRq"][a]
    s = P["s0"][a] + MR * z + 0.5 * MRq * (MR * z) ** 2
    mr = MR + MRq * MR ** 2 * z
    x = (P["s0"][a] - K["perch"][i] + P["L"][a]) + P["MRk"][a] * z
    Fs = K["spring"][i] * max(x, 0.0)
    smax = SHOCK_MAX[a] - K["packer"][i]
    gap = P["bump_gap"][a]
    e = s - (smax - gap)
    Fb = P["kbump"][a] * e * e / (2 * gap) if e > 0 else 0.0
    return P["unsprung"][a] + P["MRk"][a] * Fs + mr * Fb, s, smax

def tyre_rate(P, a, p):
    return P["kt"][a] * (1 + P["kt_p"] * (p - 152.0))

def _solve_lin(A, b):
    """Gaussian elimination with partial pivoting (7x7)."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-14:
            raise ZeroDivisionError("singular")
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x

def solve_static(K, P=PARAMS):
    """Mirror of Garage.solve_static: -> dict(cw, rh, shock, shock_max, camber, cross, toe_front, ok)."""
    Wt = P["W0"] + K["fuel"] * P["fuel_kgL"] * G
    Wf = P["W0"] * P["front0"] + K["fuel"] * P["fuel_kgL"] * G * P["fuel_front"]
    Wr = Wt - Wf
    kbar = (arb_rate(P, 0, K["farb_dia"], K["farb_arm"]), arb_rate(P, 1, K["rarb_dia"], K["rarb_arm"]))
    tr = (1.0, P["trackratio"])

    def loads(z):
        return [corner_force(P, K, i, z[i])[0] + kbar[_axle(i)] * (z[i] - z[_other(i)]) for i in range(4)]

    def resid(v):
        hf, hr, ro = v[0], v[1], v[2]
        z = v[3:7]
        W = loads(z)
        r = [0.0] * 7
        for i in range(4):
            a = _axle(i)
            rh = (hf if a == 0 else hr) + _side(i) * ro * tr[a] / 2
            dg = K["camber"][i] - P["cam_ref"][a]
            r[i] = rh - (P["R0"][a] + P["rp"][a] * (K["pressure"][i] - 152.0)
                         + P.get("cam_jack", 0.0) * dg + P.get("cam_jack2", 0.0) * dg * dg - z[i]
                         - W[i] / tyre_rate(P, a, K["pressure"][i]))
        r[4] = (W[0] + W[1]) - Wf
        r[5] = (W[2] + W[3]) - Wr
        rdeg = ro / 1500 * 180 / math.pi
        cam = [K["camber"][i] - P["cam_gain"][_axle(i)] * z[i] + _side(i) * P["cam_roll"][_axle(i)] * rdeg
               for i in range(4)]
        r[6] = sum(W[i] * _side(i) * (P["track_f"] * tr[_axle(i)] / 2 + P["scrub"][_axle(i)] * z[i]
                                      + P["cam_patch"] * (cam[i] - P["cam_ref"][_axle(i)])) for i in range(4))
        return r

    v = [P["R0"][0] - 20.0, P["R0"][1] - 20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ok = False
    for _ in range(60):
        r = resid(v)
        J = [[0.0] * 7 for _ in range(7)]
        h = 1e-6
        for j in range(7):
            vp = v[:]
            vp[j] += h
            rp = resid(vp)
            for i in range(7):
                J[i][j] = (rp[i] - r[i]) / h
        try:
            dv = _solve_lin(J, r)
        except ZeroDivisionError:
            break
        v = [v[k] - dv[k] for k in range(7)]
        if max(abs(d) for d in dv) < 1e-9:
            ok = True
            break
    hf, hr, ro = v[0], v[1], v[2]
    z = v[3:7]
    cf = [corner_force(P, K, i, z[i]) for i in range(4)]
    W = [cf[i][0] + kbar[_axle(i)] * (z[i] - z[_other(i)]) for i in range(4)]
    hfm = (1 - P["rh_mix"][0]) * hf + P["rh_mix"][0] * hr
    hrm = (1 - P["rh_mix"][1]) * hr + P["rh_mix"][1] * hf
    rh = [(hfm if _axle(i) == 0 else hrm) + _side(i) * ro * (1.0 if _axle(i) == 0 else P["trackratio"]) / 2
          for i in range(4)]
    rolldeg = ro / 1500 * 180 / math.pi
    cam = [K["camber"][i] - P["cam_gain"][_axle(i)] * z[i] + _side(i) * P["cam_roll"][_axle(i)] * rolldeg
           for i in range(4)]
    Ws = [W[i] - (1 - P["bar_shown"]) * kbar[_axle(i)] * (z[i] - z[_other(i)]) for i in range(4)]
    return dict(cw=Ws, rh=rh, shock=[c[1] for c in cf], shock_max=[c[2] for c in cf], camber=cam,
                cross=100 * (Ws[1] + Ws[2]) / sum(Ws), toe_front=K["toe_front"] + P["toe_gain"] * (z[0] + z[1]),
                ok=ok)

def gear_speeds_kmh(K, P=PARAMS):
    return [P["rpm_r"] * 2 * math.pi / 60 / (g * K["final_drive"]) * 3.6 for g in K["gears"]]

def derived(K, session, P=PARAMS):
    """What the garage shows for knobs K: the session's own numbers + the model's change from its knobs."""
    D = solve_static(K, P)
    Ds = solve_static(session["knobs"], P)
    sd = session["derived"]
    cw = [sd["cw"][i] + D["cw"][i] - Ds["cw"][i] for i in range(4)]
    return dict(cw=cw,
                rh=[sd["rh"][i] + D["rh"][i] - Ds["rh"][i] for i in range(4)],
                shock=[sd["shock"][i] + D["shock"][i] - Ds["shock"][i] for i in range(4)],
                shock_max=D["shock_max"],
                camber=[sd["camber"][i] + D["camber"][i] - Ds["camber"][i] for i in range(4)],
                toe_front=sd["toe_front"] + D["toe_front"] - Ds["toe_front"],
                cross=100 * (cw[1] + cw[2]) / sum(cw),
                gears_kmh=gear_speeds_kmh(K, P), ok=D["ok"])

def write_garage_file(path, K, base="default", car=None, track=None):
    """The file the sim reads (JM_GARAGE): TOML, base session + one table of knobs (same layout as garage.jl's).
    SETUPFILTER-1: a saved setup also names the car and track it was made for (the sim ignores both)."""
    def val(v):
        if isinstance(v, str):
            return '"' + v.replace('"', '\\"') + '"'
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (list, tuple)):
            return "[" + ", ".join(val(x) for x in v) + "]"
        if isinstance(v, int):
            return str(v)
        return repr(float(v))
    lines = ['base = ' + val(base)] + [f"{k} = {val(v)}" for k, v in (("car", car), ("track", track)) if v] + ["", "[knobs]"]
    for f in KNOB_FIELDS:
        lines.append(f"{f} = {val(K[f])}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return path

def read_garage_file(path, base_knobs=None):
    with open(path, "rb") as f:
        d = tomllib.load(f)
    K = copy.deepcopy(base_knobs) if base_knobs else {}
    K.update(d.get("knobs", {}))
    return d.get("base", "default"), K
