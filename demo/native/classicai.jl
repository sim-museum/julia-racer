

module CLASSICAI
using Random

const TICK = 1/36
const T2 = 36.0^2

"One Classic line (.lp), resampled for interpolation. Speeds in m/s, lateral velocity in m/s at line speed."
struct Line
    v::Vector{Float64}
    dv::Vector{Float64}
    d::Vector{Float64}
    flags::Vector{UInt32}
end
const REC = 3.0

function read_line(path)
    b = read(path)
    n = Int(reinterpret(UInt32, b[5:8])[1])
    f = reinterpret(Float32, b[9:8+20n]); u = reinterpret(UInt32, b[9:8+20n])
    Line(Float64.(f[1:5:end]) .* 36.0, Float64.(f[2:5:end]) .* 36.0, Float64.(f[3:5:end]), u[5:5:end])
end
@inline function _li(L::Line, s, f::Function)
    n = length(L.d); x = mod(s, n*REC)/REC; i = floor(Int, x); u = x - i
    a = mod(i, n) + 1; b = mod(i + 1, n) + 1
    f(L, a)*(1 - u) + f(L, b)*u
end
dlat(L::Line, s)  = _li(L, s, (l, i) -> l.d[i])
dlatv(L::Line, s) = _li(L, s, (l, i) -> l.dv[i])
lspeed(L::Line, s) = _li(L, s, (l, i) -> l.v[i])
"d(lateral velocity)/ds -- the line's lateral-acceleration feed-forward per unit speed (Classic 0x44a950)"
function dlatv_ds(L::Line, s)
    n = length(L.d); i = mod(round(Int, mod(s, n*REC)/REC), n) + 1; j = mod(i, n) + 1
    (L.dv[j] - L.dv[i]) / REC
end
flags(L::Line, s) = (n = length(L.d); L.flags[mod(round(Int, mod(s, n*REC)/REC), n) + 1])

"""Classic's own track frame: the .trk section chain, each section a constant-curvature ARC (length, start heading,
heading change), walked exactly -- position and heading are continuous by construction and the curvature is
piecewise constant, as in Classic (0x497d70 reads the section radius). Nothing here is a fitted or interpolated curve:
an interpolating spline through unevenly spaced samples of these arcs loops at short sections (radius 0.0 m at
Watkins Glen s = 1523 and 2858), and that loop is what threw the first draft's cars sideways (AICLASSIC-2, 2026-10-04).
World = similarity transform (a, b, tx, tz) of Classic's plane: (a*x - b*y + tx, b*x + a*y + tz); `sgn` maps +dlat
onto the left normal."""
struct Ref
    S0::Vector{Float64}; L::Vector{Float64}; th0::Vector{Float64}; kap::Vector{Float64}
    x0::Vector{Float64}; y0::Vector{Float64}
    gx::Float64; gy::Float64
    lap::Float64
    a::Float64; b::Float64; tx::Float64; tz::Float64
    sgn::Float64
    lx::Vector{Float64}; lz::Vector{Float64}; ls::Vector{Float64}
end
"Read the .trk section chain: lengths, start headings and the start point the sim's centreline uses."
function trk_sections(b::Vector{UInt8})
    u32(o) = UInt32(b[o+1]) | UInt32(b[o+2])<<8 | UInt32(b[o+3])<<16 | UInt32(b[o+4])<<24
    i32(o) = reinterpret(Int32, u32(o)); TRK = 19685.03937
    traces = Int(u32(12)); sections = Int(u32(16)); wallsize = Int(u32(20))
    secbase = 28 + 64 + sections*4 + 32*traces*sections + wallsize
    altbase = 28 + 64 + sections*4
    toff = [i32(28+t*4) for t in 0:15]; ctr = argmin(abs.(toff[1:traces])) - 1
    x = i32(altbase + ctr*32 + 24)/TRK; y = i32(altbase + ctr*32 + 28)/TRK
    L = [i32(secbase + s*52 + 8)/TRK for s in 0:sections-1]
    th = [i32(secbase + s*52 + 12) * 2pi / 2.0^32 for s in 0:sections-1]
    (L, th, x, y)
end
function Ref(L::Vector{Float64}, th::Vector{Float64}, x::Float64, y::Float64;
             a = 1.0, b = 0.0, tx = 0.0, tz = 0.0, sgn = 1.0)
    n = length(L); wrap(d) = d > pi ? d-2pi : d < -pi ? d+2pi : d
    S0 = zeros(n); kap = zeros(n); x0 = zeros(n); y0 = zeros(n)
    for k in 1:n
        k > 1 && (S0[k] = S0[k-1] + L[k-1])
        dth = wrap(th[mod1(k+1, n)] - th[k]); kap[k] = dth / max(L[k], 1e-9)
        x0[k] = x; y0[k] = y
        if abs(dth) < 1e-9
            x += L[k]*cos(th[k]); y += L[k]*sin(th[k])
        else
            R = L[k]/dth; x += R*(sin(th[k]+dth) - sin(th[k])); y += R*(cos(th[k]) - cos(th[k]+dth))
        end
    end
    lap = S0[end] + L[end]
    r = Ref(S0, L, th, kap, x0, y0, x0[1] - x, y0[1] - y, lap, Float64(a), Float64(b), Float64(tx), Float64(tz),
            Float64(sgn), Float64[], Float64[], Float64[])
    for s in 0:1.0:lap-1e-6
        w = world(r, s, 0.0); push!(r.lx, w[1]); push!(r.lz, w[2]); push!(r.ls, s)
    end
    r
end
"the same chain, placed in the world by `ref`'s transform fitted to points `Q` that correspond to `P` (Classic plane)"
function fit_transform(r::Ref, P::Vector{NTuple{2,Float64}}, Q::Vector{NTuple{2,Float64}}; sgn = r.sgn)
    n = min(length(P), length(Q))
    px = sum(p[1] for p in P[1:n])/n; py = sum(p[2] for p in P[1:n])/n
    qx = sum(q[1] for q in Q[1:n])/n; qz = sum(q[2] for q in Q[1:n])/n
    sxx = sum((P[k][1]-px)*(Q[k][1]-qx) + (P[k][2]-py)*(Q[k][2]-qz) for k in 1:n)
    sxy = sum((P[k][1]-px)*(Q[k][2]-qz) - (P[k][2]-py)*(Q[k][1]-qx) for k in 1:n)
    spp = sum((P[k][1]-px)^2 + (P[k][2]-py)^2 for k in 1:n)
    a = sxx/spp; b = sxy/spp
    Ref(r.L, r.th0, r.x0[1], r.y0[1]; a = a, b = b, tx = qx - (a*px - b*py), tz = qz - (b*px + a*py), sgn = sgn)
end
@inline function _sec(r::Ref, s)
    sm = mod(s, r.lap)
    k = clamp(searchsortedlast(r.S0, sm), 1, length(r.L))
    (k, sm - r.S0[k], sm)
end
"(x, y, heading) in Classic's plane at dlong s on dlat 0, closure removed"
function plane(r::Ref, s)
    k, u, sm = _sec(r, s); t0 = r.th0[k]; kp = r.kap[k]
    if abs(kp*u) < 1e-9
        x = r.x0[k] + u*cos(t0); y = r.y0[k] + u*sin(t0)
    else
        R = 1/kp; x = r.x0[k] + R*(sin(t0 + kp*u) - sin(t0)); y = r.y0[k] + R*(cos(t0) - cos(t0 + kp*u))
    end
    f = sm / r.lap
    (x + r.gx*f, y + r.gy*f, t0 + kp*u)
end
"world (x, z) of Classic (dlong, dlat), and the line's heading there"
function world(r::Ref, s, d)
    x, y, th = plane(r, s)
    x += -sin(th)*r.sgn*d; y += cos(th)*r.sgn*d
    (r.a*x - r.b*y + r.tx, r.b*x + r.a*y + r.tz, th + atan(r.b, r.a))
end
heading(r::Ref, s) = plane(r, s)[3] + atan(r.b, r.a)
"curvature toward +dlat (1/m of Classic dlong): the section's, constant along it"
curv(r::Ref, s) = r.sgn * r.kap[_sec(r, s)[1]]
"locate world (x, z): (dlong, dlat, sample index). `hint` = last sample index (0: search everything)"
function locate(r::Ref, x, z, hint::Int = 0)
    m = length(r.lx); best = Inf; bk = 1; bu = 0.0
    rng = hint == 0 ? (1:m) : ((hint - 80):(hint + 80))
    @inbounds for kk in rng
        k = mod1(kk, m); j = k == m ? 1 : k + 1
        ax = r.lx[k]; az = r.lz[k]; vx = r.lx[j] - ax; vz = r.lz[j] - az; L2 = vx*vx + vz*vz
        u = L2 > 0 ? clamp(((x - ax)*vx + (z - az)*vz)/L2, 0.0, 1.0) : 0.0
        dx = x - (ax + u*vx); dz = z - (az + u*vz); d2 = dx*dx + dz*dz
        d2 < best && (best = d2; bk = k; bu = u)
    end
    hint != 0 && best > 30.0^2 && return locate(r, x, z, 0)
    s = mod(r.ls[bk] + bu*1.0, r.lap)

    for _ in 1:2
        wx, wz, th = world(r, s, 0.0)
        s = mod(s + ((x - wx)*cos(th) + (z - wz)*sin(th)) / hypot(r.a, r.b), r.lap)
    end
    wx, wz, th = world(r, s, 0.0)
    d = r.sgn*(-(x - wx)*sin(th) + (z - wz)*cos(th)) / hypot(r.a, r.b)
    (s, d, bk)
end

"classic_ai.ini's fuzzy-line parameter set (SI units)"
struct Fuzzy
    k1::Float64; k2::Float64; k1c::Float64; k2c::Float64
    vsw::Float64
    T::Float64; Tc::Float64
    sep::Float64; sepc::Float64
    st::Float64; lt::Float64
    avoid::Float64
end
function Fuzzy(ini, sec, d)
    g(k, x) = get(get(ini, sec, Dict{String,Float64}()), k, x)
    Fuzzy(g("dlat_accel_k1", d[1])*T2, g("dlat_accel_k2", d[2])*36, g("cornering_dlat_accel_k1", d[3])*T2,
          g("cornering_dlat_accel_k2", d[4])*36, g("switch_to_cornering_dlat_velocity", 0.10)*36,
          g("dlat_trans_time", d[5]), g("cornering_dlat_trans_time", d[6]),
          g("desired_dlong_sep", d[7]), g("cornering_desired_dlong_sep", d[8]),
          g("short_term_lookahead", 17.0), g("long_term_lookahead", 108.0), g("avoid_time_coeff", d[9]))
end
Base.@kwdef struct Params
    follow::Fuzzy; basic::Fuzzy; abrupt::Fuzzy
    traction::Float64 = 0.011154*T2
    brake_eff::Float64 = 0.80
    max_accel::Float64 = 0.006806*T2
    latk::Float64 = 0.0125*36
    auto_blocker::Float64 = 0.85
    passee_sep::Float64 = 0.35
    squeezed::Float64 = 0.85
    dlat_sep::Float64 = 2.70
    sw_close::Float64 = 0.022*36
    sw_sep::Float64 = 10.0
    pass_radius::Float64 = 400.0
    yaw_k1::Float64 = 0.12*T2; yaw_k2::Float64 = 1.0*36
    alt_k1::Float64 = 0.07*T2; alt_k2::Float64 = 0.32*36
    ride::Float64 = 0.43152
    pitch_k::Float64 = 2.0/T2
    roll_k::Float64 = 3.0/T2
    slip_k::Float64 = 0.1667*T2
    sep_coeff::Float64 = 1.0
    adj::Float64 = 1.0
    vcap::Float64 = 2.41*36
    start_hiatus::Float64 = 18.0
end
"Params from classic_ai.ini (parsed: section => key => value) and the track.ini numbers."
function params(ini; sep_coeff = 1.0, adj = 1.0, vcap = 2.41*36)
    g(sec, k, x) = get(get(ini, sec, Dict{String,Float64}()), k, x)
    Params(follow = Fuzzy(ini, "follow_line", (0.0045, 0.1202, 0.0086, 0.1432, 4.0, 3.0, 14.0, 13.5, 0.10)),
           basic  = Fuzzy(ini, "basic_line_transition", (0.0056, 0.1254, 0.0092, 0.1524, 4.0, 3.0, 13.5, 12.5, 0.0875)),
           abrupt = Fuzzy(ini, "abrupt_line_transition", (0.0087, 0.1224, 0.0101, 0.1274, 3.0, 2.0, 13.0, 12.0, 0.06)),
           traction = g("GP", "nominal_traction_circle", 0.011154)*T2,
           brake_eff = g("GP", "braking_efficiency_coeff", 0.80),
           max_accel = g("GP", "nominal_max_accel", 0.006806)*T2,
           latk = g("physics", "max_lat_acc_from_speed", 0.0125)*36,
           auto_blocker = g("behavior", "auto_blocker_line_speed_pct", 0.85),
           passee_sep = g("behavior", "passee_dlong_sep_coeff", 0.35),
           squeezed = g("behavior", "being_squeezed_speed_coeff", 0.85),
           dlat_sep = g("behavior", "min_dlat_sep_front", 2.70),
           sw_close = g("behavior", "straightaway_pass_closing_velocity", 0.022)*36,
           sw_sep = g("behavior", "straightaway_pass_dlong_sep", 10.0),
           pass_radius = g("behavior", "min_cornering_outside_pass_radius", 400.0),
           yaw_k1 = g("physics", "yaw_accel_k1", 0.12)*T2, yaw_k2 = g("physics", "yaw_accel_k2", 1.0)*36,
           alt_k1 = g("physics", "alt_accel_k1", 0.07)*T2, alt_k2 = g("physics", "alt_accel_k2", 0.32)*36,
           pitch_k = g("physics", "pitch_accel_coeff", 2.0)/T2, roll_k = g("physics", "roll_accel_coeff", 3.0)/T2,
           slip_k = g("physics", "inverse_slipcurve_k", 0.1667)*T2,
           start_hiatus = g("behavior", "base_race_start_hiatus", 18.0),
           sep_coeff = sep_coeff, adj = adj, vcap = vcap)
end

const RACE, MINR, MAXR, PASS1, PASS2 = 0, 1, 2, 3, 4
const PIT_FREE = Base.Ref(get(ENV, "JM_AI_PITCLAMP", "0") == "0")
const ROLLFIX = Base.Ref(get(ENV, "JM_AI_ROLLFIX", "1") != "0")

const ROLL_MAX = parse(Float64, get(ENV, "JM_AI_ROLLMAX", "0.30"))
const ROLLDIAG = get(ENV, "JM_AI_ROLLDIAG", "0") != "0"; const ROLLDIAG_N = Base.Ref(0)
const PIT = 5
const CAR_LEN = 4.0; const CAR_W = 1.75
const PLAYER_HIT_V = parse(Float64, get(ENV, "JM_CLASSICAI_HITV", "5.0"))
const EDGE = 0.30
const GLASS = 0.10
struct Track
    lines::Dict{Int,Line}
    ref::Ref
    lap::Float64
    P::Params
    height::Function

    dsh::Vector{Float64}
    dstep::Float64
end
Track(lines, ref, lap, P, height) = Track(lines, ref, lap, P, height, Float64[], 3.0)
@inline function dshift(T::Track, sa)
    n = length(T.dsh); n == 0 && return 0.0
    t = mod(sa, T.ref.lap)/T.dstep; k = floor(Int, t); u = t - k
    a = T.dsh[mod(k, n) + 1]; b = T.dsh[mod(k + 1, n) + 1]
    a + (b - a)*u
end
L(T::Track, k) = T.lines[k]

_k(T::Track) = T.ref.lap / T.lap
tworld(T::Track, s, d) = (sa = s*_k(T); world(T.ref, sa, d + dshift(T, sa)))
theading(T::Track, s) = heading(T.ref, s*_k(T))
tcurv(T::Track, s) = curv(T.ref, s*_k(T)) * _k(T)
"locate world (x, z) in Classic's frame: (dlong, dlat, hint)"
tlocate(T::Track, x, z, hint::Int = 0) = ((s, d, h) = locate(T.ref, x, z, hint); (s/_k(T), d - dshift(T, s), h))

const KEEP = Base.Ref(NTuple{4,Float64}[])
function corridor(T::Track, s)
    lo = dlat(L(T, MINR), s); hi = dlat(L(T, MAXR), s)
    for (a, b, l, h) in KEEP[]
        len = mod(b - a, T.lap); x = mod(s - a, T.lap)
        x <= len || continue
        w = clamp(min(x, len - x)/40.0, 0.0, 1.0)
        lo2 = lo + w*(max(lo, l) - lo); hi2 = hi + w*(min(hi, h) - hi)
        hi2 - lo2 >= 2.0 && (lo = lo2; hi = hi2)
    end
    (lo, hi)
end

mutable struct Car
    id::Int
    s::Float64; v::Float64
    d::Float64; dv::Float64; da::Float64
    lap::Int
    line::Int; offset::Float64
    mode::Int
    w::Float64; dw::Float64
    d0::Float64; dv0::Float64
    fromline::Int
    hold::Int
    passctr::Int
    passctr2::Int
    passee::Int
    passee_t::Int
    squeezed::Bool
    alim::Float64
    along::Float64; atire::Float64
    yaw::Float64; yawr::Float64
    h::Float64; hv::Float64
    yroad::Float64; yroadv::Float64
    pace::Float64
    react::Int
    pose::NTuple{6,Float64}
    prev::NTuple{6,Float64}
    hint::Int
    contact::Int
    why::Symbol
end
function Car(T::Track, id, s, d; v = 0.0, pace = 1.0)
    lo, hi = corridor(T, s); d = clamp(d, min(lo + 0.2, 0.5*(lo + hi)), max(hi - 0.2, 0.5*(lo + hi)))
    c = Car(id, mod(s, T.lap), v, d, 0.0, 0.0, 0, RACE, d - dlat(L(T, RACE), s), 0, 1.0, 0.0, d, 0.0, RACE,
            0, 0, 0, 0, 0, false, 99.0, 0.0, 0.0, 0.0, 0.0, T.P.ride - 9.80665/T.P.alt_k1, 0.0, 0.0, 0.0, pace, 0,
            ntuple(_ -> 0.0, 6), ntuple(_ -> 0.0, 6), 0, 9999, :none)
    x, z, th = tworld(T, c.s, c.d); c.yaw = th + (T.ref.sgn < 0 ? 0.0 : 0.0)
    c.yroad = T.height(c.s, c.d, x, z)
    c.pose = _pose(T, c); c.prev = c.pose
    c
end

"the goal (dlat, dlat velocity, lateral feed-forward) of the line object the car is in -- Classic 0x498080/0x445300"
function goal(T::Track, c::Car, F::Fuzzy)
    s = c.s; v = c.v
    Ln = L(T, c.line)
    vl = max(lspeed(Ln, s), 1.0)
    g  = dlat(Ln, s) + c.offset
    gv = dlatv(Ln, s) * v / vl
    ff = dlatv_ds(Ln, s) * v
    if c.mode != 0
        Lo = L(T, c.fromline)
        g  = (1 - c.w)*c.d0 + c.w*g
        gv = ((1 - c.w)*c.dv0 + c.w*gv) * (c.mode == 2 ? 0.1 : 0.2)
        ff = (1 - c.w)*dlatv_ds(Lo, s)*v + c.w*ff
    end
    (c.line == PIT || (c.mode != 0 && c.fromline == PIT)) && return (g, gv, ff)
    lo, hi = corridor(T, s)
    la, ha = corridor(T, s + 0.5*v)
    if lo - GLASS <= c.d <= hi + GLASS
        lo = max(lo, min(la, 0.5*(lo + hi))); hi = min(hi, max(ha, 0.5*(lo + hi)))
        g = clamp(g, min(lo + EDGE, 0.5*(lo + hi)), max(hi - EDGE, 0.5*(lo + hi)))
        w = get(WALL, c.id, nothing)
        w === nothing || (g = clamp(g, w[1], max(w[1], w[2])))
    elseif (c.d > hi && ff > 0) || (c.d < lo && ff < 0)
        ff *= 0.33
    end
    (g, gv, ff)
end

"WGTD-1 S9: move onto Classic's pit line (offset 0) through an ordinary line transition"
function join_pit!(T::Track, c::Car)
    haskey(T.lines, PIT) || return false
    c.line == PIT && return false
    F = T.P.basic; _, gv, _ = goal(T, c, T.P.follow)
    c.why = :pit; c.fromline = c.line; c.d0 = c.d; c.dv0 = gv
    c.line = PIT; c.offset = 0.0
    c.mode = 1; c.dw = 1/max(abs(c.dv) < F.vsw ? F.T : F.Tc, 1.0); c.w = c.dw
    true
end

"join the nearest of RACE/PASS1/PASS2 at dlat `want` through a line transition (Classic 0x4477c0 + 0x4451d0)"
function join!(T::Track, c::Car, want; at = c.s, abrupt = false, why = :join)
    c.why = why
    best = RACE; bo = want - dlat(L(T, RACE), at)
    for k in (PASS1, PASS2)
        o = want - dlat(L(T, k), at); abs(o) < abs(bo) - 1e-6 && (best = k; bo = o)
    end
    (c.line == best && abs(c.offset - bo) < 0.3) && return false
    F = abrupt ? T.P.abrupt : T.P.basic
    _, gv, _ = goal(T, c, T.P.follow)
    c.fromline = c.line
    c.d0 = c.d; c.dv0 = gv
    c.line = best; c.offset = bo
    Tt = (abs(c.dv) < F.vsw ? F.T : F.Tc)
    c.mode = abrupt ? 2 : 1; c.dw = 1/max(Tt, 1.0); c.w = c.dw
    true
end

"spring/damper with a Heun step, Classic 0x496860 (SI, dt = one tick)"
@inline function spring(x, g, v, gv, k1, k2, dt)
    a1 = k1*(g - x) + k2*(gv - v)
    xp = x + v*dt + 0.5*a1*dt^2; vp = v + a1*dt
    a2 = k1*(g - xp) + k2*(gv - vp)
    0.5*(a1 + a2)
end

"""What car `c` sees of the others `τ` ticks ahead (Classic 0x445c10/0x4481d0/0x447f50): one entry per car within
98 m ahead -- its projected lateral interval and the acceleration that would keep the desired separation behind
it -- and, for cars overlapping me along the track (alongside), their lateral interval as plain occupancy.
`others` holds (s, d, dv, v, line, offset, is_player, id, longitudinal accel). Entry: (lo, hi, a_allow, projected gap, idx, kind),
kind 1 = ahead (a blocker), 2 = alongside (occupies lateral space only)."""
function look(T::Track, c::Car, others, τ, F::Fuzzy)
    P = T.P; out = NTuple{7,Float64}[]
    t = τ*TICK
    for (k, o) in enumerate(others)
        gap = mod(o[1] - c.s + T.lap/2, T.lap) - T.lap/2

        gapt = gap + (o[4] - c.v)*τ*TICK
        (-CAR_LEN < gap <= 98.27 || (gap <= -CAR_LEN && gapt > -CAR_LEN)) || continue
        isp = o[7] > 0.5
        pd = if t >= 1.0 && !isp
            dlat(L(T, Int(o[5])), o[1] + o[4]*t) + o[6]
        elseif t >= 1.0 && !WALLS_ON[]
            dlat(L(T, RACE), o[1] + o[4]*t) + (o[2] - dlat(L(T, RACE), o[1]))
        elseif t >= 1.0
            o[2] + o[3]*1.0
        else
            o[2] + o[3]*t
        end

        w = (o[4] < 0.6204*36 ? max(1.9035, o[4]/(0.6204*36)*P.dlat_sep) : P.dlat_sep) + (isp ? 0.33 : 0.0)
        a = min(pd, 0.5*(pd + o[2])); b = max(pd, 0.5*(pd + o[2]))
        if gap <= CAR_LEN
            push!(out, (min(a, o[2]) - w, max(b, o[2]) + w, -99.0, gap, Float64(k), 2.0, o[1] + o[4]*t))
            continue
        end
        sep = F.sep*P.sep_coeff
        braking = o[9] < -3.0
        (c.passee != 0 && Int(o[8]) == c.passee && !braking) && (sep = max(6.45, sep*P.passee_sep))
        dvc = c.v - o[4] + (braking ? -o[9]*0.5 : 0.0)
        Tr = max((36.0 + 25.2292*(2.85384 - c.v/36))/36, 0.5)
        gp = gap + (o[4] - c.v)*t
        aal = 2*(gap - sep - max(dvc, 0.0)*(Tr + t))/t^2
        push!(out, (a - w, b + w, aal, gp, Float64(k), 1.0, o[1] + o[4]*t))
    end
    out
end
"blocker AHEAD at lateral position x in map `m`: (a_allow, projected gap, idx) of the most restrictive, or nothing"
function blocked_at(m, x)
    r = nothing
    for e in m
        (e[6] == 1.0 && e[1] <= x <= e[2]) || continue
        (r === nothing || e[3] < r[1]) && (r = (e[3], e[4], e[5]))
    end
    r
end
"is lateral position x occupied by a car alongside?"
occupied(m, x) = any(e -> e[6] == 2.0 && e[1] <= x <= e[2], m)
"""scan from `from` toward `to` in 0.33 m bins for the first position that does not restrict car `c`
(Classic 0x4461a0); `need` = the acceleration the car wants, `lo/hi` the corridor"""
function free_toward(m, from, to, need, reach, lo, hi)
    st = to >= from ? 0.33 : -0.33
    n = max(1, ceil(Int, abs(to - from)/0.33))
    for k in 0:n
        x = clamp(from + k*st, lo, hi)
        occupied(m, x) && continue
        b = blocked_at(m, x)
        (b === nothing || b[1] >= need || b[2] >= reach) && return x
    end
    nothing
end

"accel the car would like at speed v (engine-limited; Classic 0x497940 'max accel')"
a_engine(P::Params, v) = min(P.max_accel, 536.0/max(v, 1.0)) - 0.42/560*v^2

"no section tighter than min_cornering_outside_pass_radius within `dist` metres ahead"
function straight_ahead(T::Track, s, dist)
    for x in 0:10.0:dist
        abs(tcurv(T, s + x)) > 1/T.P.pass_radius && return false
    end
    true
end
"""pull out past blocker entry `bi`: the nearest free position beside its interval, inside the corridor; join the
nearest authored line there and hold it (0x446ef0 + 0x4477c0 + 0x447a60). true if a move was made."""
function try_pass!(T::Track, c::Car, m, bi, need, lo, hi, t, rng, tpass, sb = c.s + c.v*t, vb = 0.0)
    k = findfirst(e -> Int(e[5]) == bi && e[6] == 1.0, m); k === nothing && return false
    lo2, hi2 = corridor(T, sb); lo = max(lo, lo2); hi = min(hi, hi2)
    e = m[k]; cand = Float64[]
    for x in (e[1] - 0.2, e[2] + 0.2)
        lo + 0.3 <= x <= hi - 0.3 || continue
        occupied(m, x) && continue
        side_busy(c, x) && continue
        bb = blocked_at(m, x); (bb === nothing || bb[1] >= need) && push!(cand, x)
    end

    gb = mod(sb - c.s, T.lap); at = c.s + clamp(gb, 0.0, c.v*t)
    isempty(cand) && return false
    join!(T, c, cand[argmin(abs.(cand .- c.d))]; at = at, why = :pass)

    tclear = (mod(sb - c.s, T.lap) + 2CAR_LEN) / max(c.v - vb, 1.0) * 36
    c.hold = clamp(round(Int, 1.5*max(tpass, tclear)), 36, 216) + rand(rng, 0:35)
    c.passctr = 0
    true
end

"the line join! would pick for `want` at `at`, followed with its offset from s0 to s1, stays inside the corridor"
function path_inside(T::Track, want, at, s0, s1)
    best = RACE; bo = want - dlat(L(T, RACE), at)
    for k in (PASS1, PASS2)
        o = want - dlat(L(T, k), at); abs(o) < abs(bo) - 1e-6 && (best = k; bo = o)
    end
    for x in s0:3.0:s1
        lo, hi = corridor(T, x); y = dlat(L(T, best), x) + bo
        (lo + 0.2 <= y <= hi - 0.2) || return false
    end
    true
end

"""where car `c` will be across the track `t` seconds ahead (Classic 0x44c920): beyond a second, along its own line +
offset at its projected dlong; nearer, its current lateral plus its rate -- clamped into the corridor there"""
function own_proj(T::Track, c::Car, t)
    sp = c.s + c.v*t
    lo, hi = corridor(T, sp)
    if t >= 1.0 || c.mode != 0
        clamp(dlat(L(T, c.line), sp) + c.offset, lo, hi)
    else
        g, gv, _ = goal(T, c, T.P.follow); clamp(g + gv*t, lo, hi)
    end
end

"""the most restrictive car ahead that my own path meets: for each map entry, my lateral where I reach its projected
station (along my line + offset when that is a second or more away), tested against its interval"""
function blocked_on_path(T::Track, c::Car, m, t)
    r = nothing
    for e in m
        e[6] == 1.0 || continue
        ahead = mod(e[7] - c.s, T.lap)
        tt = min(t, ahead/max(c.v, 1.0))
        pd = own_proj(T, c, tt)
        e[1] <= pd <= e[2] || continue
        (r === nothing || e[3] < r[1]) && (r = (e[3], e[4], e[5]))
    end
    r
end

"""Classic's avoidance (0x4468f0) on one look-ahead map: :acted (moved to pass), :blocked (stay in line behind the
blocker -- the following law holds the car), :free (nothing in my lane). `map` 1 = short term, 2 = long term."""
function avoid!(T::Track, c::Car, others, m, τ, map, rng, scale)
    P = T.P; F = P.follow; t = τ*TICK
    lo, hi = wallclip(c, corridor(T, c.s)...)
    need = a_engine(P, c.v)

    reach = max(c.v*t, F.sep*P.sep_coeff + 6.0)
    b = blocked_on_path(T, c, m, t)
    (b !== nothing && b[1] < need && b[2] < reach) || return :free
    o = others[Int(b[3])]
    faster = c.v > o[4] || o[4] < 2.0
    (o[4] >= 5.0 && !isempty(YZ) && inyellow(T, c.s)) && return :blocked
    ratio = o[4] / max(min(lspeed(L(T, RACE), o[1])*P.adj, P.vcap)*scale, 1.0)
    slowB = ratio < P.auto_blocker || o[4] < 5.0
    gap = mod(o[1] - c.s + T.lap/2, T.lap) - T.lap/2
    vfree = min(lspeed(L(T, c.line), c.s)*P.adj, P.vcap)*c.pace*scale
    if map == 1 && vfree > o[4] + 0.5 && o[8] > 0.5
        c.passee = Int(o[8]); c.passee_t = 0
    end

    if map == 2 && c.passee != 0 && Int(o[8]) == c.passee && straight_ahead(T, c.s, 150.0) &&
       (c.alim < 0.995*need || c.v - o[4] > P.sw_close) && gap < (2.0 - min(ratio, 1.0))*P.sw_sep + 2.0
        try_pass!(T, c, m, Int(b[3]), need, lo, hi, t, rng, 1.5*36.0, o[1], o[4]) && return :acted
    end
    ctr = map == 1 ? c.passctr : c.passctr2
    if !faster || !slowB
        ctr = max(ctr - 1, 0)
    else
        ctr += 1

        tpass = o[4] < 5.0 ? 0.0 : max(gap, 1.0)/max(c.v - o[4], 0.3) * 36 * F.avoid * 4
        if ctr > tpass && try_pass!(T, c, m, Int(b[3]), need, lo, hi, t, rng, tpass, o[1], o[4])
            c.passctr = 0; c.passctr2 = 0; return :acted
        end
    end
    map == 1 ? (c.passctr = ctr) : (c.passctr2 = ctr)
    :blocked
end

"""Classic's BASIC RACING think step (0x442f60) for one car. `others` as in `look`; returns nothing.
`rng` gives the random part of the hold time."""
function think!(T::Track, c::Car, others, rng, scale = 1.0)
    P = T.P; F = P.follow
    c.hold > 0 && (c.hold -= 1)
    c.squeezed = false

    (PIT_FREE[] && c.line == PIT) && return nothing
    τs = F.st; t = τs*TICK
    m = look(T, c, others, τs, F)
    lo, hi = wallclip(c, corridor(T, c.s)...)
    need = a_engine(P, c.v)
    reach = max(c.v*t, F.sep*P.sep_coeff + 6.0)
    r = avoid!(T, c, others, m, τs, 1, rng, scale)
    r == :acted && return nothing
    if r == :free
        c.passctr = max(c.passctr - 1, 0)
        c.passee_t += 1; c.passee_t > 108 && (c.passee = 0)
        pd = own_proj(T, c, t)

        for (k, o) in enumerate(others)
            gap = mod(o[1] - c.s + T.lap/2, T.lap) - T.lap/2
            abs(gap) < 5.5 || continue
            od = o[2] + o[3]*t
            sep = abs(pd - od)
            sep < P.dlat_sep - 0.6 || continue
            side = c.d >= o[2] ? 1.0 : -1.0
            lo2, hi2 = corridor(T, c.s + c.v*t)
            want = clamp(od + side*(P.dlat_sep - 0.3), lo + 0.3, hi - 0.3)
            if abs(want - od) >= P.dlat_sep - 0.9
                join!(T, c, want; at = c.s, abrupt = (o[3] - c.dv)*side > 0.05*36, why = :side)
                c.hold = max(c.hold, round(Int, τs))
            else
                c.squeezed = true
            end
            return nothing
        end
    end

    if !(lo - GLASS <= c.d <= hi + GLASS) && c.mode == 0
        join!(T, c, clamp(c.d, lo + 0.5, hi - 0.5); why = :corridor); return nothing
    end

    if c.hold <= 0 && (c.line != RACE || abs(c.offset) > 1e-3) && c.mode == 0
        target = dlat(L(T, RACE), c.s + c.v*t)
        lo2, hi2 = corridor(T, c.s + c.v*t)
        x = free_toward(m, target, c.d, need, reach, max(lo, lo2), min(hi, hi2))
        (x !== nothing && !side_busy(c, x)) && join!(T, c, x; at = c.s + c.v*t, why = :approach)
    end

    if c.v > 0.6204*36 && c.mode == 0
        avoid!(T, c, others, look(T, c, others, F.lt, F), F.lt, 2, rng, scale)
    end
    nothing
end

"""one 36 Hz tick of car `c`'s motion (Classic 0x496930): longitudinal (line speed, following law, traction
circle), lateral (spring/damper on the goal, clamped by grip and the corridor), yaw, height."""
function advance!(T::Track, c::Car, m_follow, scale, vcap_rel)
    P = T.P; dt = TICK

    Ln = L(T, c.line)
    vt = min(lspeed(Ln, c.s)*P.adj, P.vcap) * c.pace * scale
    c.squeezed && (vt *= P.squeezed)
    (!isempty(YZ) && inyellow(T, c.s)) && (vt *= YELLOW_K[])
    vt = min(vt, vcap_rel)
    aw = (vt - c.v)/dt
    alimf = m_follow === nothing ? 99.0 : m_follow
    c.alim += (alimf - c.alim)*0.3
    aw = min(aw, c.alim)
    amax = a_engine(P, c.v); abrk = P.brake_eff*P.traction
    lat_used = abs(c.atire)
    rem = sqrt(max(P.traction^2 - lat_used^2, 0.0))
    aw = clamp(aw, -min(abrk, max(rem, 0.3*abrk)), min(amax, rem))
    c.react > 0 && (c.react -= 1; aw = 0.0)
    c.along = aw
    c.v = max(c.v + aw*dt, 0.0)

    F = c.mode == 2 ? P.abrupt : c.mode == 1 ? P.basic : P.follow
    corner = abs(c.dv) >= F.vsw
    k1 = corner ? F.k1c : F.k1; k2 = corner ? F.k2c : F.k2
    g, gv, ff = goal(T, c, F)
    a = spring(c.d, g, c.dv, gv, k1, k2, dt) + ff
    cen = tcurv(T, c.s) * c.v^2
    tyre = a + cen
    lim = min(max(P.latk*c.v, abs(cen)), P.traction)
    tyre = clamp(tyre, -lim, lim)
    c.atire = tyre
    c.da = tyre - cen
    c.dv += c.da*dt
    dprev = c.d
    c.d  += c.dv*dt
    w = get(WALL, c.id, nothing)
    if w !== nothing
        (c.dv > 0 && c.d > w[2] && dprev <= w[2] + 0.05) && (c.d = max(dprev, w[2]); c.dv = 0.0)
        (c.dv < 0 && c.d < w[1] && dprev >= w[1] - 0.05) && (c.d = min(dprev, w[1]); c.dv = 0.0)
    end

    if c.mode != 0
        c.w += c.dw
        c.w > 1.0 && (c.mode = 0; c.w = 1.0)
    end

    s0 = c.s
    c.s += c.v*dt
    if c.s >= T.lap; c.s -= T.lap; c.lap += 1; end
    lo, hi = corridor(T, c.s)

    onpit = PIT_FREE[] && (c.line == PIT || (c.mode != 0 && c.fromline == PIT))
    if !onpit && (c.d < lo || c.d > hi)
        if haskey(SOFTEDGE, c.id)
            Δ = clamp(clamp(c.d, lo, hi) - c.d, -SOFT_V*dt, SOFT_V*dt)
            c.d += Δ; c.dv = Δ/dt; c.da = 0.0
        else
            Δ = clamp(c.d, lo, hi) - c.d
            c.d += Δ; c.dv += Δ/dt; c.da += Δ/dt^2
        end
    end
    haskey(SOFTEDGE, c.id) && ((lo <= c.d <= hi) || (SOFTEDGE[c.id] -= 1) <= 0) && delete!(SOFTEDGE, c.id)

    th = theading(T, c.s) + (T.ref.sgn > 0 ? 1 : -1)*atan(c.dv, max(c.v, 0.5)) * (c.v > 0.5 ? 1 : 0)
    slip = (T.ref.sgn > 0 ? 1 : -1) * tyre / P.slip_k
    yg = th + slip
    e = atan(sin(yg - c.yaw), cos(yg - c.yaw))
    yrg = tcurv(T, c.s)*c.v*(T.ref.sgn > 0 ? 1 : -1)
    c.yawr += (P.yaw_k1*e + P.yaw_k2*(yrg - c.yawr))*dt
    c.yaw += c.yawr*dt

    x, z, _ = tworld(T, c.s, c.d)
    yr = T.height(c.s, c.d, x, z)
    yrv = (yr - c.yroad)/dt
    if !isfinite(yr); yr = c.yroad; yrv = 0.0; end
    jump = abs(yr - c.yroad) > 2.0
    yra = jump ? 0.0 : (yrv - c.yroadv)/dt
    c.yroad = yr; c.yroadv = jump ? 0.0 : yrv
    gext = -9.80665 - clamp(yra, -60.0, 60.0)
    ah = (c.h <= P.ride ? P.alt_k1*(P.ride - c.h) - P.alt_k2*c.hv : 0.0) + gext
    c.hv += ah*dt; c.h += c.hv*dt
    c.h < 0.0 && (c.h = 0.0; c.hv = max(c.hv, 0.0))
    c.prev = c.pose
    c.pose = _pose(T, c)
    nothing
end
function _pose(T::Track, c::Car)
    P = T.P
    x, z, _ = tworld(T, c.s, c.d)
    heq = P.ride - 9.80665/P.alt_k1
    y = c.yroad + (c.h - heq)

    h1 = T.height(c.s + 2.0, c.d, tworld(T, c.s + 2.0, c.d)[1], tworld(T, c.s + 2.0, c.d)[2])
    h0 = T.height(c.s - 2.0, c.d, tworld(T, c.s - 2.0, c.d)[1], tworld(T, c.s - 2.0, c.d)[2])
    hl = T.height(c.s, c.d + 0.8, tworld(T, c.s, c.d + 0.8)[1], tworld(T, c.s, c.d + 0.8)[2])
    hr = T.height(c.s, c.d - 0.8, tworld(T, c.s, c.d - 0.8)[1], tworld(T, c.s, c.d - 0.8)[2])
    pr = isfinite(h1) && isfinite(h0) ? atan(h1 - h0, 4.0) : 0.0
    rr = isfinite(hl) && isfinite(hr) ? atan(hl - hr, 1.6) : 0.0

    if ROLLFIX[]
        hc = c.yroad
        sl = isfinite(hl) && isfinite(hc) ? atan(hl - hc, 0.8) : 0.0
        sr = isfinite(hr) && isfinite(hc) ? atan(hc - hr, 0.8) : 0.0
        rr = sign(sl) != sign(sr) ? 0.0 : (abs(sl) < abs(sr) ? sl : sr)
        (ROLLDIAG && abs(rr) > ROLL_MAX && (ROLLDIAG_N[] += 1) % 20 == 1) &&
            println("  [airoll] s ", round(c.s, digits = 0), " d ", round(c.d, digits = 1), " camber ", round(rad2deg(rr), digits = 1),
                    " deg (one-sided ", round(rad2deg(sl), digits = 1), " / ", round(rad2deg(sr), digits = 1), "), drawn at most ", round(rad2deg(ROLL_MAX), digits = 1))
        rr = clamp(rr, -ROLL_MAX, ROLL_MAX)
    end
    pitch = pr - P.pitch_k*c.along
    roll  = T.ref.sgn*rr + P.roll_k*c.atire
    (x, isfinite(y) ? y : c.yroad, z, c.yaw, pitch, roll)
end

"""The speed factor that makes the race line's lateral demand fit the grip: Classic's line speeds were authored for
Classic's own AI grip, which is not the julia Lotus's. p98 of |centreline curvature x v^2 + line lateral accel| at
line speed must not exceed `frac` of the traction circle. 1.0 when it already fits."""
function grip_scale(T::Track; frac = 0.95, q = 0.98)
    Ln = L(T, RACE); dem = Float64[]
    for s in 0:REC:T.lap-REC
        v = min(lspeed(Ln, s)*T.P.adj, T.P.vcap)
        push!(dem, abs(tcurv(T, s)*v^2 + dlatv_ds(Ln, s)*v))
    end
    sort!(dem); d = dem[clamp(round(Int, q*length(dem)), 1, length(dem))]
    min(1.0, sqrt(frac*T.P.traction/max(d, 1e-6)))
end

const GONE = Set{Int}()

const SOFTEDGE = Dict{Int,Int}()
const SOFT_V = 4.0
const YZ = Float64[]
const YELLOW_BEFORE = 300.0; const YELLOW_PAST = 40.0
const YELLOW_K = Base.RefValue(0.55)
function inyellow(T, s)
    for z in YZ
        g = mod(s - z + T.lap/2, T.lap) - T.lap/2
        -YELLOW_BEFORE <= g <= YELLOW_PAST && return true
    end
    false
end

const WALLS_ON = Base.RefValue(get(ENV, "JM_AI_WALLS", "1") != "0")
const WALL_CLEAR = 2.4
const SIDE_TICKS = 36
const WALL = Dict{Int,NTuple{2,Float64}}()
const SIDE = Dict{Int,NTuple{2,Int}}()
const TICKN = Base.RefValue(0)
function walls!(T, c, others)
    lo, hi = corridor(T, c.s)
    WALLS_ON[] || return (WALL[c.id] = (lo, hi))
    (sl, sr) = get(SIDE, c.id, (0, 0))
    for o in others
        gap = mod(o[1] - c.s + T.lap/2, T.lap) - T.lap/2
        abs(gap) < CAR_LEN + 0.5 || continue
        sep = o[2] - c.d
        0.5CAR_W <= abs(sep) <= 2.75CAR_W || continue
        if sep > 0
            hi = min(hi, o[2] - WALL_CLEAR); sl = TICKN[] + SIDE_TICKS
        else
            lo = max(lo, o[2] + WALL_CLEAR); sr = TICKN[] + SIDE_TICKS
        end
    end
    lo > hi && (m = 0.5*(lo + hi); lo = m; hi = m)
    SIDE[c.id] = (sl, sr)
    WALL[c.id] = (lo, hi)
end
"a move from `c`'s lateral to x would go toward a side remembered occupied"
side_busy(c, x) = (sd = get(SIDE, c.id, (0, 0)); (x > c.d + 0.5 && sd[1] > TICKN[]) || (x < c.d - 0.5 && sd[2] > TICKN[]))
"the corridor (lo, hi) narrowed by car `c`'s walls"
wallclip(c, lo, hi) = (w = get(WALL, c.id, nothing); w === nothing ? (lo, hi) : (max(lo, min(w[1], hi)), min(hi, max(w[2], lo))))

mutable struct Field
    T::Track
    cars::Vector{Car}
    acc::Float64
    rng::Any
    stats::Dict{Symbol,Int}
end
Field(T::Track, cars; seed = 1967) = Field(T, cars, 0.0, MersenneTwister(seed), Dict{Symbol,Int}())

"""Advance the field by `dt` seconds of wall time (whole 36 Hz ticks; the remainder carries) and return the
drawn poses interpolated between the last two ticks. `player` = (dlong, dlat, dlat rate, speed) or nothing.
Returns (poses, player_hit)."""
function step!(f::Field, dt; player = nothing, scale = 1.0, vrel = Inf)
    f.acc += dt
    hit = false
    while f.acc >= TICK
        f.acc -= TICK
        hit |= tick!(f, player, scale, vrel)
    end
    α = f.acc / TICK
    poses = NTuple{6,Float64}[]
    for c in f.cars
        p = c.prev; q = c.pose
        dyaw = atan(sin(q[4] - p[4]), cos(q[4] - p[4]))
        push!(poses, (p[1] + α*(q[1]-p[1]), p[2] + α*(q[2]-p[2]), p[3] + α*(q[3]-p[3]), p[4] + α*dyaw,
                      p[5] + α*(q[5]-p[5]), p[6] + α*(q[6]-p[6])))
    end
    (poses, hit)
end
function tick!(f::Field, player, scale, vrel)
    T = f.T; P = T.P; n = length(f.cars)
    hit = false

    view = NTuple{9,Float64}[(c.s, c.d, c.dv, c.v, Float64(c.line), c.offset, 0.0, Float64(c.id), c.along) for c in f.cars if !(c.id in GONE)]
    player !== nothing && push!(view, (player[1], player[2], player[3], player[4], 0.0, 0.0, 1.0, 0.0, length(player) >= 5 ? player[5] : 0.0))
    TICKN[] += 1
    for (i, c) in enumerate(f.cars)
        c.id in GONE && continue
        others = [v for v in view if !(v[7] < 0.5 && Int(v[8]) == c.id)]
        walls!(T, c, others)
        think!(T, c, others, f.rng, scale)

        F = P.follow; t = F.st*TICK
        m = look(T, c, others, F.st, F)
        b = blocked_at(m, own_proj(T, c, t))
        al = b === nothing ? 99.0 : b[1]
        advance!(T, c, al, scale, vrel)
    end

    for a in 1:n-1, b in a+1:n
        ca = f.cars[a]; cb = f.cars[b]
        (ca.id in GONE || cb.id in GONE) && continue
        ds = mod(ca.s - cb.s + T.lap/2, T.lap) - T.lap/2
        (abs(ds) < CAR_LEN && abs(ca.d - cb.d) < CAR_W) || continue
        f.stats[:contact] = get(f.stats, :contact, 0) + 1
        sgn = ca.d >= cb.d ? 1.0 : -1.0
        cl = (cb.dv - ca.dv)*sgn
        cl > -0.5 && (ca.dv += sgn*(cl + 0.5)/2; cb.dv -= sgn*(cl + 0.5)/2)

        if abs(ds) > CAR_LEN/2
            if ds > 0; cb.v = min(cb.v, ca.v); else; ca.v = min(ca.v, cb.v); end
        end
        ca.contact = 0; cb.contact = 0
    end
    if player !== nothing
        for c in f.cars
            c.id in GONE && continue
            ds = mod(c.s - player[1] + T.lap/2, T.lap) - T.lap/2
            (abs(ds) < CAR_LEN && abs(c.d - player[2]) < CAR_W) || continue
            hit = true; f.stats[:player_contact] = get(f.stats, :player_contact, 0) + 1
            sg = c.d >= player[2] ? 1.0 : -1.0
            cl = (player[3] - c.dv)*sg; cl > 0 && (c.dv += sg*cl)

            ds < -CAR_LEN/2 && c.v - player[4] < PLAYER_HIT_V && (c.v = min(c.v, player[4]))
        end
    end
    for c in f.cars; c.contact += 1; end
    hit
end

end
