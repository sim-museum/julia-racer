

module JoyCfg

export Ctrl, JoyMap, defaultmap, loadmap, savemap, apply, x3dmap, txmap, profile_for, resolve, loadmeta, builtin_profiles, device_file, wheel_half_range_deg, steer_physical, shift_ok

struct Ctrl
    axis::Int
    a::Float64
    b::Float64
end

struct JoyMap
    steer::Ctrl
    throttle::Ctrl
    brake::Ctrl
    clutch::Ctrl
    up_btn::Int
    dn_btn::Int
    clutch_btn::Int
    deadzone::Float64
    wheel_half_deg::Float64
end
JoyMap(s, t, b, c, u, d, cb, dz) = JoyMap(s, t, b, c, u, d, cb, dz, 0.0)

"Default = the original hardcoded mapping: steer=axis1, throttle/brake=axis2 (push/
pull), buttons 1/2/3 = up/down/clutch.  `apply` then reduces to the old formulas."
defaultmap() = JoyMap(Ctrl(1, -1.0, 1.0), Ctrl(2, 0.0, -1.0), Ctrl(2, 0.0, 1.0),
                      Ctrl(0, 0.0, 1.0), 1, 2, 3, 0.06)

const BUILTIN_DIR = joinpath(@__DIR__, "controller_profiles")
const USER_DIR    = joinpath(@__DIR__, "joystick_profiles")

"""The text keys of a profile/conf file (`label`, `match`, `mode`, `device`, `profile`): the rest of the line."""
function loadmeta(path)
    d = Dict{String,String}()
    isfile(path) || return d
    for ln in eachline(path)
        s = strip(ln); (isempty(s) || startswith(s, "#")) && continue
        k, v = (p = split(s; limit = 2); length(p) == 2 ? (String(p[1]), String(strip(p[2]))) : (String(p[1]), ""))
        tryparse(Float64, v) === nothing && (d[k] = v)
    end
    d
end

"""Built-in profiles: (label, match words, path), sorted by label."""
function builtin_profiles(dir = BUILTIN_DIR)
    isdir(dir) || return Tuple{String,String,String}[]
    out = [(get(m, "label", f), get(m, "match", ""), p) for f in readdir(dir) if endswith(f, ".conf")
           for p in (joinpath(dir, f),) for m in (loadmeta(p),)]
    sort(out)
end

builtin(file) = loadmap(joinpath(BUILTIN_DIR, file))
"The Logitech Extreme 3D Pro: steer = roll, throttle/brake = push/pull on axis 2, clutch = slider (axis 4)."
x3dmap() = builtin("logitech_extreme_3d_pro.conf")
"The Thrustmaster TX: wheel axis 1, throttle 2, clutch 3, brake 4; pedals rest at +1.0; right paddle (2) = up."
txmap() = builtin("thrustmaster_tx.conf")

"""Built-in profile for a GLFW joystick NAME: (label, JoyMap) when every word of a profile's `match` occurs in the
name (case-blind), or nothing if the device is unknown."""
function profile_for(name::AbstractString)
    n = lowercase(name)
    for (lab, mt, p) in builtin_profiles()
        w = split(lowercase(mt))
        !isempty(w) && all(x -> occursin(x, n), w) && return (lab, loadmap(p))
    end
    nothing
end

"The file name a device's own calibration is kept under (the launcher's rule: non-alphanumerics -> '_')."
device_file(name::AbstractString, dir = USER_DIR) = joinpath(dir, map(c -> isletter(c) || isdigit(c) ? c : '_', name) * ".conf")

samemap(a::JoyMap, b::JoyMap) = all(getfield(a, f) == getfield(b, f) for f in fieldnames(JoyMap))

"The per-device file holding the same map as `conf` (the device a pre-CTRLCAL-1 conf was made on), or nothing."
function legacy_owner(conf, userdir)
    isdir(userdir) || return nothing
    m = loadmap(conf)
    for f in sort(readdir(userdir; join = true))
        endswith(f, ".conf") && samemap(loadmap(f), m) && return f
    end
    nothing
end

"""Resolve the live map: (JoyMap, source) from the conf path and the connected device's name (see the mode notes)."""
function resolve(conf::AbstractString, name::AbstractString; userdir = USER_DIR)
    meta = loadmeta(conf); mode = get(meta, "mode", "")
    if isfile(conf) && mode == "profile"
        return (loadmap(conf), "saved profile \"" * get(meta, "label", "?") * "\"" *
                (isempty(name) ? "" : " on \"" * name * "\""))
    end
    if isfile(conf) && mode != "autodetect"

        owner = legacy_owner(conf, userdir)
        (isempty(name) || owner === nothing || owner == device_file(name, userdir)) && return (loadmap(conf), "joystick.conf")
    end
    dev = get(meta, "device", "")
    isfile(conf) && (isempty(name) || dev == name) &&
        return (loadmap(conf), "autodetect: calibration of \"" * (isempty(dev) ? "?" : dev) * "\"")
    if !isempty(name)
        f = device_file(name, userdir)
        isfile(f) && return (loadmap(f), "autodetect: saved calibration of \"" * name * "\"")
    end
    pr = profile_for(name)
    pr === nothing || return (pr[2], "autodetected " * pr[1] * " (\"" * name * "\")")
    (x3dmap(), isempty(name) ? "X3D default (no controller found)" :
               "X3D default -- UNKNOWN controller \"" * name * "\": calibrate it in the launcher")
end

"""Half the wheel's rotation range in degrees (raw ±1), from the kernel's per-device `range` attribute
(hid-tmff2 et al.), matched on the GLFW name; else `measured` (the calibration's 90° step) if given; else 450 (= 900°)
for a Thrustmaster TX; 0 when the device is not a wheel (no physical steering)."""
function wheel_half_range_deg(name::AbstractString; measured::Real = 0.0)
    isempty(name) && return 0.0
    try
        for d in readdir("/sys/bus/hid/devices"; join = true)
            f = joinpath(d, "range"); isfile(f) || continue
            u = read(joinpath(d, "uevent"), String)
            m = match(r"HID_NAME=(.*)", u)
            (m !== nothing && strip(m.captures[1]) == strip(name)) || continue
            r = tryparse(Float64, strip(read(f, String)))
            r !== nothing && r > 0 && return r/2
        end
    catch
    end
    measured > 0 && return Float64(measured)
    n = lowercase(name)
    occursin("thrustmaster", n) && occursin("tx", n) && return 450.0
    0.0
end

"""Physical steering output in [-1,1] (fraction of the sim's full road-wheel lock) for a wheel:
centre and sign from the map's calibration, gain = (half range °/ steering ratio) / max road angle °."""
function steer_physical(m::JoyMap, js, gain::Float64; lim::Float64 = 1.0)
    c = m.steer; (c.axis < 1 || js === nothing || c.axis > length(js)) && return 0.0
    centre = 0.5*(c.a + c.b); sgn = c.b >= c.a ? 1.0 : -1.0

    clamp(-sgn*(Float64(js[c.axis]) - centre)*gain, -lim, lim)
end

"""MANUAL shift gate (PO 2026-10-03: "let me blip the clutch rather than stomping on it"; a downshift
"roughly matching revs without using the clutch" must go through). Accepted if the clutch is in now
(>= 0.4), or was blipped past 0.25 within the last 0.35 s, or the engine is within ±15 % of the rpm the
new gear needs at this road speed (`tgt_rpm`; a rev-matched clutchless shift)."""
shift_ok(clu, secs_since_blip, rpm, tgt_rpm) =
    clu >= 0.4 || secs_since_blip <= 0.35 || (tgt_rpm > 1500 && abs(rpm - tgt_rpm) <= 0.15*tgt_rpm)

@inline function _norm(c::Ctrl, js)
    (c.axis < 1 || js === nothing || c.axis > length(js)) && return 0.0
    d = c.b - c.a
    abs(d) < 1e-6 ? 0.0 : (Float64(js[c.axis]) - c.a) / d
end

"""Map raw GLFW axes/buttons → (steer∈[-1,1], throttle∈[0,1], brake∈[0,1],
clutch∈[0,1], up::Bool, dn::Bool) per a JoyMap."""
function apply(m::JoyMap, js, bs)
    s = clamp(1.0 - 2.0 * _norm(m.steer, js), -1.0, 1.0)

    dz = (v = tryparse(Float64, get(ENV,"JM_DEADZONE","")); v === nothing ? m.deadzone : v)
    thr = clamp(_norm(m.throttle, js), 0.0, 1.0); thr < dz && (thr = 0.0)
    brk = clamp(_norm(m.brake,    js), 0.0, 1.0); brk < dz && (brk = 0.0)
    btn(i) = (bs !== nothing && i >= 1 && length(bs) >= i && bs[i] != 0)
    clu = m.clutch.axis >= 1 ? clamp(_norm(m.clutch, js), 0.0, 1.0) : (btn(m.clutch_btn) ? 1.0 : 0.0)
    (s, thr, brk, clu, btn(m.up_btn), btn(m.dn_btn))
end

function savemap(path, m::JoyMap)
    open(path, "w") do io
        println(io, "# zand_racer joystick config — generated by calibrate.jl")
        for (nm, c) in (("steer",m.steer),("throttle",m.throttle),("brake",m.brake),("clutch",m.clutch))
            println(io, "$nm.axis $(c.axis)"); println(io, "$nm.a $(c.a)"); println(io, "$nm.b $(c.b)")
        end
        println(io, "up_btn $(m.up_btn)"); println(io, "dn_btn $(m.dn_btn)")
        println(io, "clutch_btn $(m.clutch_btn)"); println(io, "deadzone $(m.deadzone)")
        m.wheel_half_deg > 0 && println(io, "wheel_half_deg $(m.wheel_half_deg)")
    end
end

"""Load a JoyMap from `path`; any missing key falls back to the default."""
function loadmap(path)
    isfile(path) || return defaultmap()
    d = Dict{String,Float64}()
    for ln in eachline(path)
        s = strip(ln); (isempty(s) || startswith(s, "#")) && continue
        parts = split(s); length(parts) >= 2 || continue
        v = tryparse(Float64, parts[2]); v !== nothing && (d[parts[1]] = v)
    end
    dm = defaultmap()
    ctrl(nm, def) = Ctrl(round(Int, get(d, "$nm.axis", def.axis)),
                         get(d, "$nm.a", def.a), get(d, "$nm.b", def.b))
    JoyMap(ctrl("steer",dm.steer), ctrl("throttle",dm.throttle), ctrl("brake",dm.brake),
           ctrl("clutch",dm.clutch),
           round(Int, get(d,"up_btn",dm.up_btn)), round(Int, get(d,"dn_btn",dm.dn_btn)),
           round(Int, get(d,"clutch_btn",dm.clutch_btn)), get(d,"deadzone",dm.deadzone), get(d, "wheel_half_deg", 0.0))
end

end
