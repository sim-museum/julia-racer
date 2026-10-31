

module RaceAI

struct AILine
    x::Vector{Float64}; z::Vector{Float64}; y::Vector{Float64}
    s::Vector{Float64}
    θ::Vector{Float64}
    κ::Vector{Float64}
    rl::Vector{Float64}
    total::Float64
end

wrapπ(a) = a > π ? a - 2π : a < -π ? a + 2π : a

const VMAX = Ref(something(tryparse(Float64, get(ENV, "JM_AI_VMAX", "")), 87.8))
"""Set the AI top-speed anchor [m/s]. Rejects implausible values: the fallback must win over a
   bad derivation rather than produce a car that does 900 km/h or 40 km/h."""
function set_vmax_from_drivetrain!(v::Real)
    if !(isfinite(v) && 40.0 <= v <= 130.0)
        @warn "AI vmax anchor $(v) m/s outside the plausible 40-130 range -- keeping $(VMAX[])"
        return VMAX[]
    end
    VMAX[] = float(v)
end
const AMAX = Ref(something(tryparse(Float64, get(ENV, "JM_AI_AMAX", "")), 13.67))
"""Set the AI grip anchor from measured telemetry. `g` is metres/s^2.
   Ignores implausible values: a raw .ibt PEAK is a kerb strike (measured 55.7 m/s2 = 5.68 g on a
   clean lap), so a caller passing a peak instead of a percentile must not silently produce a car
   that corners at five g."""
function set_anchor_from_ibt!(g::Real)
    if !(isfinite(g) && 5.0 <= g <= 25.0)
        @warn "AI grip anchor $(g) m/s2 is outside the plausible 5-25 range -- keeping $(AMAX[])"
        return AMAX[]
    end
    AMAX[] = float(g)
end
const VT_BRAKE = Ref(get(ENV, "JM_VT_BRAKE", "1") != "0")

const BRAKE_A  = Ref(something(tryparse(Float64, get(ENV, "JM_BRAKE_A", "30.0")), 30.0))
const SOFT_BAND = Ref(get(ENV, "JM_SOFT_BAND", "1") != "0")
const POSE_LERP = Ref(get(ENV, "JM_POSE_LERP", "0") != "0")

const SOFT_KNEE = Ref(something(tryparse(Float64, get(ENV, "JM_SOFT_KNEE", "0.7")), 0.7))

"Build an AI racing line from centreline points `pts` (each (x,z) in physics frame)
and a `groundz(x,z)->y` elevation function.  Resamples to ~`spacing` m so tight corners are
well-represented (a coarse line is a polygon that the AI chord across = corner-cutting) and
the per-point curvature is accurate."
function build_line(pts, groundz; spacing = 3.0, halfwidth = 3.0)

    m = length(pts)
    cum = zeros(m+1); for i in 1:m; cum[i+1] = cum[i] + hypot(pts[i%m+1][1]-pts[i][1], pts[i%m+1][2]-pts[i][2]); end
    total = cum[m+1]; nfine = max(m, round(Int, total/spacing))
    fine = Vector{Tuple{Float64,Float64}}(undef, nfine)
    for k in 1:nfine
        d = (k-1)/nfine * total
        j = clamp(searchsortedlast(cum, d), 1, m); f = (d - cum[j]) / max(cum[j+1]-cum[j], 1e-9)
        a = pts[j]; b = pts[j%m+1]; fine[k] = (a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f)
    end
    pts = fine
    n = length(pts)
    x = Float64[p[1] for p in pts]; z = Float64[p[2] for p in pts]
    s = zeros(n); for i in 2:n; s[i] = s[i-1] + hypot(x[i]-x[i-1], z[i]-z[i-1]); end
    θ = [atan(z[i%n+1]-z[i], x[i%n+1]-x[i]) for i in 1:n]
    κ0 = zeros(n)
    for i in 1:n
        j = i % n + 1; ds = max(j == 1 ? (s[end]-s[i]) : (s[j]-s[i]), 0.5)
        κ0[i] = abs(wrapπ(θ[j]-θ[i])) / ds
    end

    rx = copy(x); rz = copy(z)
    nx = Float64[-sin(θ[i]) for i in 1:n]; nz = Float64[cos(θ[i]) for i in 1:n]

    hwκ = [halfwidth * (1.0 - 0.5*clamp((κ0[i]-0.018)/0.045, 0.0, 1.0)) for i in 1:n]
    hw  = similar(hwκ); for i in 1:n; a=0.0; for d in -3:3; a += hwκ[mod(i-1+d,n)+1]; end; hw[i]=a/7; end

    let hs = get(ENV, "JM_HW_SCALE", "1")
        f = something(tryparse(Float64, hs), 1.0)
        if f != 1.0
            hw .*= f
            @info "AI corridor: JM_HW_SCALE=$f applied (halfwidth band scaled)"
        end
    end
    pinned = falses(n)
    for _ in 1:600
        for i in 1:n
            p = mod(i-2, n)+1; q = i % n + 1
            mx = 0.5*(rx[p]+rx[q]); mz = 0.5*(rz[p]+rz[q])
            rx[i] += 0.25*(mx-rx[i]); rz[i] += 0.25*(mz-rz[i])
            offraw = (rx[i]-x[i])*nx[i] + (rz[i]-z[i])*nz[i]

            off = if SOFT_BAND[]
                a = SOFT_KNEE[] * hw[i]
                if abs(offraw) <= a
                    offraw
                else
                    sign(offraw) * (a + (hw[i] - a) * tanh((abs(offraw) - a) / max(hw[i] - a, 1e-6)))
                end
            else
                clamp(offraw, -hw[i], hw[i])
            end

            pinned[i] = (offraw != off)
            rx[i] = x[i] + off*nx[i]; rz[i] = z[i] + off*nz[i]
        end
    end
    rlg = Float64[(rx[i]-x[i])*nx[i] + (rz[i]-z[i])*nz[i] for i in 1:n]

    κg = zeros(n)
    for i in 1:n; j=i%n+1; ds=max(hypot(rx[j]-rx[i], rz[j]-rz[i]),0.5); κg[i]=abs(wrapπ(atan(rz[j]-rz[i],rx[j]-rx[i]) - atan(rz[i]-rz[mod(i-2,n)+1], rx[i]-rx[mod(i-2,n)+1])))/ds; end
    vp = [sqrt(8.0/max(κg[i], 1e-3)) for i in 1:n]
    W  = max(1, round(Int, 22/spacing))
    sh = zeros(n)
    for i in 1:n
        κg[i] > 1/130 || continue
        trend = vp[mod(i-1+W,n)+1] - vp[mod(i-1-W,n)+1]
        sh[i] = clamp(trend*0.18, -2.5, 2.5)
    end
    shs = zeros(n)
    for i in 1:n; a=0.0; for d in -W:W; a += sh[mod(i-1+d,n)+1]; end; shs[i]=a/(2W+1); end
    di(arr, p) = (p=mod(p-1,n)+1; lo=floor(Int,p); f=p-lo; lo=mod(lo-1,n)+1; hi=mod(lo,n)+1; arr[lo]*(1-f)+arr[hi]*f)
    rl = Float64[di(rlg, i - shs[i]/spacing) for i in 1:n]
    rx = Float64[x[i] + rl[i]*nx[i] for i in 1:n]; rz = Float64[z[i] + rl[i]*nz[i] for i in 1:n]

    θr = [atan(rz[i%n+1]-rz[i], rx[i%n+1]-rx[i]) for i in 1:n]
    κr = zeros(n)
    for i in 1:n
        j = i % n + 1; ds = max(hypot(rx[j]-rx[i], rz[j]-rz[i]), 0.5)
        κr[i] = abs(wrapπ(θr[j]-θr[i])) / ds
    end
    κ = zeros(n)
    if get(ENV, "JM_KAPPA_ARC", "0") == "1"

        W = parse(Float64, get(ENV, "JM_KAPPA_ARC_M", "9.0")) / 2
        arc(i, j) = begin
            d = 0.0; k = i
            while k != j; kn = k % n + 1; d += hypot(rx[kn]-rx[k], rz[kn]-rz[k]); k = kn; end
            d
        end
        step_to(i, dist, dir) = begin
            d = 0.0; k = i
            while d < dist
                kn = dir > 0 ? (k % n + 1) : (mod(k - 2, n) + 1)
                d += hypot(rx[kn]-rx[k], rz[kn]-rz[k]); k = kn
                k == i && break
            end
            (k, d)
        end
        for i in 1:n
            (ib, db) = step_to(i, W, -1)
            (ifw, df) = step_to(i, W, +1)
            span = db + df
            κ[i] = span > 0.5 ? abs(wrapπ(θr[ifw] - θr[ib])) / span : κr[i]
        end
    else
        for i in 1:n
            a = 0.0; for d in -2:2; a += κr[mod(i-1+d, n)+1]; end; κ[i] = a/5
        end
    end

    if get(ENV, "JM_CLAMP_DIAG", "0") == "1"
        npin = count(pinned); frac = npin / n
        top = sortperm(κ, rev=true)[1:min(20, n)]
        hit = count(i -> pinned[i], top)
        println("  [clampdiag] nodes=", n, "  pinned by the road-width clamp=", npin,
                " (", round(100*frac, digits=1), "% of the lap)")
        println("  [clampdiag] of the top-20 |kappa| nodes, ", hit, " are pinned",
                "  (chance would be ", round(20*frac, digits=1), ")")
        println("  [clampdiag] VERDICT: ", hit > 10 ? "CLAMP IS IMPLICATED -- pursue it" :
                                           hit <= 20*frac + 2 ? "NO BETTER THAN CHANCE -- candidate is DEAD" :
                                                                "weak/ambiguous -- do not build on this")
        flush(stdout)
    end
    y = Float64[(h = groundz(x[i], z[i]); isfinite(h) ? h : 0.0) for i in 1:n]

    AILine(x, z, y, s, θ, κ, rl, s[end] + hypot(x[1]-x[end], z[1]-z[end]))
end

function curv_at(line::AILine, s)
    i, f = _locate(line, s); j = i % length(line.κ) + 1
    line.κ[i]*(1-f) + line.κ[j]*f
end

"Racing-line lateral offset (m, left +) at arc-length `s`."
function racelane(line::AILine, s)
    i, f = _locate(line, s); j = i % length(line.rl) + 1

    if POSE_LERP[]
        line.rl[i]*(1-f) + line.rl[j]*f
    else
        nn = length(line.rl); h = mod(i-2, nn) + 1; k = j % nn + 1
        t = f; t2 = t*t; t3 = t2*t
        p0 = line.rl[h]; p1 = line.rl[i]; p2 = line.rl[j]; p3 = line.rl[k]
        0.5*((2p1) + (-p0 + p2)*t + (2p0 - 5p1 + 4p2 - p3)*t2 + (-p0 + 3p1 - 3p2 + p3)*t3)
    end
end

mutable struct AICar; s::Float64; v::Float64; lap::Int; lane::Float64; tlane::Float64; spin::Float64; follow::Float64
    pace::Float64
    mishap::Float64
    vlane::Float64
end
AICar(s, v, lap, lane) = AICar(s, v, lap, lane, lane, 0.0, 0.0, 1.0, 0.0, 0.0)

const CLASSICAI_TICK = 36.0
include(joinpath(@__DIR__, "classicroot.jl"))
"Search order for Classic's AI parameter file. JM_CLASSIC_AI_INI overrides everything."
function classic_ai_paths()
    out = String[]
    haskey(ENV, "JM_CLASSIC_AI_INI") && push!(out, ENV["JM_CLASSIC_AI_INI"])
    for r in (classic_root(),)
        push!(out, joinpath(r, "classic_ai.ini"))
        push!(out, joinpath(r, "repository", "CLASSICPSInstaller", "Teamfix", "Studio", "classic_ai.ini"))
    end
    out
end
"""Parse `classic_ai.ini` into section => key => value. Tolerant by necessity: sections are written
`[ name ]`, values are tab-separated from their `;` comment, and at least one shipped line
(`cornering_dlat_accel_k1`) is missing its `;` altogether, so the value is the first token after the
`=` that parses as a number."""
function parse_classic_ai(path)
    out = Dict{String,Dict{String,Float64}}(); sec = ""
    for ln in eachline(path)
        t = strip(ln)
        (isempty(t) || startswith(t, ";")) && continue
        if startswith(t, "[")
            sec = strip(strip(t, ['[', ']'])); get!(out, sec, Dict{String,Float64}()); continue
        end
        i = findfirst('=', t); i === nothing && continue
        k = strip(t[1:i-1]); isempty(k) && continue
        startswith(k, ";") && continue
        v = nothing
        for tok in split(replace(t[i+1:end], ';' => ' '))
            v = tryparse(Float64, tok); v === nothing || break
        end
        v === nothing && continue
        get!(out, sec, Dict{String,Float64}())[k] = v
    end
    out
end
const CLASSIC_AI = Ref{Union{Nothing,Dict{String,Dict{String,Float64}}}}(nothing)
const CLASSIC_AI_SRC = Ref("")
function classic_ai()
    CLASSIC_AI[] === nothing || return CLASSIC_AI[]
    for p in classic_ai_paths()
        if isfile(p)
            try
                d = parse_classic_ai(p)

                mg = get(get(d, "magic", Dict{String,Float64}()), "parameter_table_magic_number", 0.0)
                if mg != 1234.0
                    println("  [classicai] ", p, ": magic number ", mg, " != 1234 -- ignoring this file")
                    continue
                end
                CLASSIC_AI[] = d; CLASSIC_AI_SRC[] = p
                return d
            catch e
                println("  [classicai] ", p, ": ", sprint(showerror, e))
            end
        end
    end
    CLASSIC_AI[] = Dict{String,Dict{String,Float64}}(); CLASSIC_AI_SRC[] = ""
    CLASSIC_AI[]
end
"One Classic parameter, or `dflt` when the file is absent or lacks it."
function classic_par(sec, key, dflt)
    d = classic_ai(); sd = get(d, sec, nothing); sd === nothing && return dflt
    get(sd, key, dflt)
end

struct DlatGains
    k1::Float64; k2::Float64
    k1c::Float64; k2c::Float64
    vsw::Float64
end
function dlat_gains(sec, k1, k2, k1c, k2c)
    t2 = CLASSICAI_TICK^2
    DlatGains(classic_par(sec, "dlat_accel_k1", k1) * t2,
              classic_par(sec, "dlat_accel_k2", k2) * CLASSICAI_TICK,
              classic_par(sec, "cornering_dlat_accel_k1", k1c) * t2,
              classic_par(sec, "cornering_dlat_accel_k2", k2c) * CLASSICAI_TICK,
              classic_par(sec, "switch_to_cornering_dlat_velocity", 0.10) * CLASSICAI_TICK)
end

const DLAT_FOLLOW  = Ref{Union{Nothing,DlatGains}}(nothing)
const DLAT_BASIC   = Ref{Union{Nothing,DlatGains}}(nothing)
const DLAT_ABRUPT  = Ref{Union{Nothing,DlatGains}}(nothing)
dlat_follow() = (DLAT_FOLLOW[] === nothing && (DLAT_FOLLOW[] = dlat_gains("follow_line", 0.0045, 0.1202, 0.0086, 0.1432)); DLAT_FOLLOW[])
dlat_basic()  = (DLAT_BASIC[]  === nothing && (DLAT_BASIC[]  = dlat_gains("basic_line_transition", 0.0056, 0.1254, 0.0092, 0.1524)); DLAT_BASIC[])
dlat_abrupt() = (DLAT_ABRUPT[] === nothing && (DLAT_ABRUPT[] = dlat_gains("abrupt_line_transition", 0.0087, 0.1224, 0.0101, 0.1274)); DLAT_ABRUPT[])

const DLAT_AMAX_K = Ref(-1.0)
dlat_amax_k() = (DLAT_AMAX_K[] < 0.0 && (DLAT_AMAX_K[] = classic_par("physics", "max_lat_acc_from_speed", 0.0125) * CLASSICAI_TICK); DLAT_AMAX_K[])

const DLAT_CLASSIC = Ref(-1)
@inline function dlat_classic()
    DLAT_CLASSIC[] < 0 && (DLAT_CLASSIC[] = get(ENV, "JM_AI_DLAT_CLASSIC", "1") != "0" ? 1 : 0)
    DLAT_CLASSIC[] == 1
end
dlat_classic!(on::Bool) = (DLAT_CLASSIC[] = on ? 1 : 0; on)

@inline dlat_mode(car) = car.tlane == 0.0 ? dlat_follow() : dlat_basic()
"""Classic's lateral law: a spring/damper on the dlat ACCELERATION goal, with the cornering gain pair
above `vsw` of lateral speed and the acceleration capped by `max_lat_acc_from_speed * speed`."""
function dlat_step!(car::AICar, tgt, dt, lim, g::DlatGains = dlat_mode(car), band = nothing)
    cornering = abs(car.vlane) >= g.vsw
    k1 = cornering ? g.k1c : g.k1
    k2 = cornering ? g.k2c : g.k2
    a  = -k1*(car.lane - tgt) - k2*car.vlane
    amax = dlat_amax_k() * max(car.v, 1.0)
    a = clamp(a, -amax, amax)
    car.vlane += a*dt
    car.lane  += car.vlane*dt

    lo, hi = band === nothing ? (-lim, lim) : band
    if car.lane < lo || car.lane > hi
        car.lane = clamp(car.lane, lo, hi); car.vlane = 0.0
    end
    nothing
end

const LANE_V = 2.4
const LANE_K = 3.0
const LANE_A = Ref(-1.0)
@inline function lane_accel()
    LANE_A[] < 0.0 && (LANE_A[] = something(tryparse(Float64, get(ENV, "JM_AI_LANE_ACCEL", "8.0")), 8.0))
    LANE_A[]
end
lane_accel!(a::Real) = (LANE_A[] = float(a); a)
"""Move `car.lane` toward `tgt` with bounded lateral speed AND bounded lateral acceleration, clamped
to ±`lim`. Returns nothing; updates `car.lane` and `car.vlane`."""
function lane_step!(car::AICar, tgt, dt, lim, band = nothing)
    dlat_classic() && return dlat_step!(car, tgt, dt, lim, dlat_mode(car), band)
    a = lane_accel()
    if a <= 0.0
        car.lane += clamp(tgt - car.lane, -LANE_V*dt, LANE_V*dt)
        car.lane  = clamp(car.lane, -lim, lim)
        car.vlane = 0.0
        return nothing
    end
    e = tgt - car.lane

    vdes = clamp(sign(e) * min(LANE_V, LANE_K*abs(e), sqrt(2.0*a*abs(e))), -LANE_V, LANE_V)
    car.vlane += clamp(vdes - car.vlane, -a*dt, a*dt)
    car.lane += car.vlane*dt
    if car.lane < -lim || car.lane > lim
        car.lane = clamp(car.lane, -lim, lim); car.vlane = 0.0
    end
    nothing
end

const RAIL     = 2.4

const BAND_MARGIN = parse(Float64, get(ENV, "JM_AI_BAND_MARGIN", "0.85"))
const LANE_MAX = 3.8

const CLEAR_MARGIN = parse(Float64, get(ENV, "JM_AI_CLEAR_MARGIN", "0.6"))
const AVOID_ROOM = get(ENV, "JM_AI_AVOID_ROOM", "1") != "0"

const AVOID_CORNER = get(ENV, "JM_AI_AVOID_CORNER", "1") != "0"

const PASS_RADIUS = parse(Float64, get(ENV, "JM_AI_PASS_RADIUS", "75.0"))
const AVOIDSTAT = Ref(0)

const SPIN_LO  = parse(Float64, get(ENV, "JM_AI_SPIN_LO", "1.0"))
const SPIN_HI  = parse(Float64, get(ENV, "JM_AI_SPIN_HI", "3.0"))
const CAR_LEN  = 4.2
const CAR_WID  = 1.7

const LANE_BIAS = parse(Float64, get(ENV, "JM_AI_LANE_BIAS", "0.7"))
lanebias(i, n) = n <= 1 ? 0.0 : LANE_BIAS * (2.0*((0.41*i) % 1.0) - 1.0)

const GAPCTL        = get(ENV, "JM_AI_GAPCTL", "1") != "0"
const CLASSIC_DESIRED_SEP = 14.0

const CLASSIC_CLOSE_TAU   = 4.0
const ENGAGE_MARGIN   = parse(Float64, get(ENV, "JM_AI_ENGAGE_MARGIN", "1.0"))
const AI_MASS = 560.0; const AI_PMAX = 300_000.0
const AI_FMAX = 6800.0; const AI_DRAG = 0.42; const AI_BRAKE = 16.0
function advance_speed(v, vtarget, dt)
    if v < vtarget
        Fdrive = min(AI_FMAX, AI_PMAX / max(v, 8.0))
        a = (Fdrive - AI_DRAG*v*v) / AI_MASS
        min(v + a*dt, vtarget)
    else
        max(v - AI_BRAKE*dt, vtarget)
    end
end

"Grid of `n` AI cars staggered ~9 m apart behind arc-length `start_s`, alternating lanes."
init_cars(line::AILine, n; start_s = 0.0) =
    [AICar(mod(start_s - 9.0*i, line.total), 25.0, 0, iseven(i) ? 2.4 : -2.4, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0) for i in 1:n]

function _locate(line::AILine, sq)
    sq = mod(sq, line.total)
    n = length(line.s)
    i = clamp(searchsortedlast(line.s, sq), 1, n)
    send = i == n ? line.total : line.s[i+1]
    f = (sq - line.s[i]) / max(send - line.s[i], 1e-6)
    i, f
end

"World pose (x, y, z, heading) at arc-length `s` with lateral `lane` (left +)."
function pose_at(line::AILine, s, lane)
    i, f = _locate(line, s); j = i % length(line.x) + 1

    if POSE_LERP[]
        x = line.x[i]*(1-f) + line.x[j]*f
        z = line.z[i]*(1-f) + line.z[j]*f
        y = line.y[i]*(1-f) + line.y[j]*f
    else
        nn = length(line.x); h = mod(i-2, nn) + 1; k = j % nn + 1
        t = f; t2 = t*t; t3 = t2*t
        cr(p0, p1, p2, p3) = 0.5*((2p1) + (-p0 + p2)*t + (2p0 - 5p1 + 4p2 - p3)*t2 + (-p0 + 3p1 - 3p2 + p3)*t3)
        x = cr(line.x[h], line.x[i], line.x[j], line.x[k])
        z = cr(line.z[h], line.z[i], line.z[j], line.z[k])
        y = cr(line.y[h], line.y[i], line.y[j], line.y[k])
    end

    n = length(line.x)

    tanspan = parse(Int, get(ENV, "JM_TANGENT_SPAN", "3"))
    θ = if get(ENV, "JM_SEGMENT_TANGENT", "0") != "0" || tanspan <= 0
        line.θ[i] + f*wrapπ(line.θ[j]-line.θ[i])
    elseif get(ENV, "JM_CATMULL_TANGENT", "0") != "0"
        h = mod(i-2, n) + 1
        k = j % n + 1

        t = f
        dx = 0.5*((-line.x[h] + line.x[j]) +
                  2t*(2line.x[h] - 5line.x[i] + 4line.x[j] - line.x[k]) +
                  3t*t*(-line.x[h] + 3line.x[i] - 3line.x[j] + line.x[k]))
        dz = 0.5*((-line.z[h] + line.z[j]) +
                  2t*(2line.z[h] - 5line.z[i] + 4line.z[j] - line.z[k]) +
                  3t*t*(-line.z[h] + 3line.z[i] - 3line.z[j] + line.z[k]))
        (dx == 0.0 && dz == 0.0) ? line.θ[i] : atan(dz, dx)
    else

        a = mod(i - 1 - tanspan, n) + 1
        b = mod(i - 1 + tanspan, n) + 1
        a2 = mod(j - 1 - tanspan, n) + 1
        b2 = mod(j - 1 + tanspan, n) + 1
        θa = atan(line.z[b] - line.z[a], line.x[b] - line.x[a])
        θb = atan(line.z[b2] - line.z[a2], line.x[b2] - line.x[a2])
        θa + f*wrapπ(θb - θa)
    end

    (x + lane*(-sin(θ)), y, z + lane*cos(θ), θ)
end

"""    reground(p, height) -> pose

Return pose `p` with its height taken from the terrain AT THE POSE'S OWN (x, z), rather than from
the racing line's centreline. `height(x, z)` must return `(y, ok)`; when `ok` is false the pose is
returned UNCHANGED, because off the terrain there is no better answer than the line's.

This exists because `pose_at` offsets a car laterally without re-sampling the ground under it
(E104(a): AI cars measured up to 0.30 m off the terrain, median 0.09 m, while the player measured
0.00 m — the PO's "every car floats 20–40 cm above the road"). Every wheel inherits the error,
since wheels are drawn at `<car origin> + [wx, r, wz]`.

It lives HERE, in the module, rather than inline in `drive_native_mtk.jl`, so it can be tested:
that file launches the sim and cannot be loaded by a gate (E103-S2).
Accepts a 4-tuple or a 6-tuple (the draw path carries pitch/roll too).
"""
function reground(p, height)
    h = height(p[1], p[3])
    h[2] || return p
    length(p) >= 6 ? (p[1], h[1], p[3], p[4], p[5], p[6]) : (p[1], h[1], p[3], p[4])
end

@inline _crp(p0, p1, p2, p3, t) =
    0.5 * (2p1 + (-p0 + p2)*t + (2p0 - 5p1 + 4p2 - p3)*t^2 + (-p0 + 3p1 - 3p2 + p3)*t^3)
@inline _crv(p0, p1, p2, p3, t) =
    0.5 * ((-p0 + p2) + 2t*(2p0 - 5p1 + 4p2 - p3) + 3t*t*(-p0 + 3p1 - 3p2 + p3))
@inline _cra(p0, p1, p2, p3, t) =
    0.5 * (2*(2p0 - 5p1 + 4p2 - p3) + 6t*(-p0 + 3p1 - 3p2 + p3))
const PROJ_NODE = Ref(false); const PROJ_NODE_SEEN = Ref(false)
@inline function proj_node()
    if !PROJ_NODE_SEEN[]
        PROJ_NODE[] = get(ENV, "JM_AI_PROJ_NODE", "0") != "0"; PROJ_NODE_SEEN[] = true
    end
    PROJ_NODE[]
end
proj_node!(on::Bool) = (PROJ_NODE[] = on; PROJ_NODE_SEEN[] = true; on)

"""Project a world point onto the line → (arc-length s, signed lateral offset, left +).

`hint` = an arc length near the answer (last frame's s). With one, the nearest node is found in a
±`HINTW`-node window instead of a full sweep -- O(1) instead of O(n), and it keeps the search on the
car's own stretch of track where the circuit passes close to itself. A hint whose window best sits
at the window EDGE is treated as stale and the full sweep runs, so a teleported/respawned car still
projects correctly."""
const HINTW = 12
function project(line::AILine, x, z; hint = nothing)
    n = length(line.x)
    bi = 0; bd = Inf; edge = false
    if hint !== nothing && n > 2HINTW + 2
        i0 = clamp(searchsortedlast(line.s, mod(float(hint), line.total)), 1, n)
        @inbounds for d in -HINTW:HINTW
            i = mod(i0 - 1 + d, n) + 1
            dd = (line.x[i]-x)^2 + (line.z[i]-z)^2
            dd < bd && (bd = dd; bi = i; edge = abs(d) == HINTW)
        end
    end
    if bi == 0 || edge
        bd = Inf
        @inbounds for i in 1:n
            d = (line.x[i]-x)^2 + (line.z[i]-z)^2
            d < bd && (bd = d; bi = i)
        end
    end
    if proj_node()
        lat = (x-line.x[bi])*(-sin(line.θ[bi])) + (z-line.z[bi])*cos(line.θ[bi])
        return (line.s[bi], lat)
    end

    prv = mod(bi - 2, n) + 1
    seg, t = let a = line.x[prv], b = line.x[bi], az = line.z[prv], bz = line.z[bi]
        l2 = (b-a)^2 + (bz-az)^2
        tb = l2 < 1e-9 ? 1.0 : clamp(((x-a)*(b-a) + (z-az)*(bz-az))/l2, 0.0, 1.0)
        tb > 0.999 ? (bi, 0.0) : (prv, tb)
    end
    @inbounds for _ in 1:3
        h = mod(seg-2, n)+1; j = mod(seg, n)+1; k = mod(seg+1, n)+1
        ax, bx, cx, dx = line.x[h], line.x[seg], line.x[j], line.x[k]
        az, bz, cz, dz = line.z[h], line.z[seg], line.z[j], line.z[k]
        ex = _crp(ax,bx,cx,dx,t) - x; ez = _crp(az,bz,cz,dz,t) - z
        v1x = _crv(ax,bx,cx,dx,t); v1z = _crv(az,bz,cz,dz,t)
        a2x = _cra(ax,bx,cx,dx,t); a2z = _cra(az,bz,cz,dz,t)
        g1 = ex*v1x + ez*v1z
        g2 = v1x*v1x + v1z*v1z + ex*a2x + ez*a2z

        g2 <= 1e-9 && break
        st = clamp(-g1/g2, -0.5, 0.5)
        t += st
        if t < 0.0
            seg = mod(seg-2, n)+1; t += 1.0
        elseif t > 1.0
            seg = mod(seg, n)+1; t -= 1.0
        end
        abs(st) < 1e-6 && break
    end
    t = clamp(t, 0.0, 1.0)
    h = mod(seg-2, n)+1; j = mod(seg, n)+1; k = mod(seg+1, n)+1
    fx = _crp(line.x[h], line.x[seg], line.x[j], line.x[k], t)
    fz = _crp(line.z[h], line.z[seg], line.z[j], line.z[k], t)
    tx = _crv(line.x[h], line.x[seg], line.x[j], line.x[k], t)
    tz = _crv(line.z[h], line.z[seg], line.z[j], line.z[k], t)
    tl = hypot(tx, tz)
    nx, nz = tl < 1e-9 ? (-sin(line.θ[seg]), cos(line.θ[seg])) : (-tz/tl, tx/tl)
    lat = (x - fx)*nx + (z - fz)*nz
    s2 = j == 1 ? line.total : line.s[j]
    (mod(line.s[seg] + t*(s2 - line.s[seg]), line.total), lat)
end

"Curvature-limited target speed at arc-length `s`, moving at `v` (m/s).  Scans a BRAKING
HORIZON ahead (∝ speed) for the tightest corner, so the car slows BEFORE a tight bend
instead of arriving too hot and washing out."

const CLASSICV = Ref{Union{Nothing,Vector{Float64}}}(nothing)
set_classic_speeds!(v) = (CLASSICV[] = v === nothing ? nothing : Float64.(v); nothing)

const CLASSICLAT = Ref{Union{Nothing,NTuple{3,Vector{Float64}}}}(nothing)
set_classic_lateral!(race, p1, p2) = (CLASSICLAT[] = race === nothing ? nothing : (Float64.(race), Float64.(p1), Float64.(p2)); nothing)
@inline _gidx(line::AILine, s, n) = mod(floor(Int, mod(s, line.total) / 3.0), n) + 1

const CLASSICLAT_STEP = Ref(get(ENV, "JM_CLASSICLAT_STEP", "0") != "0")
@inline function _gfrac(line::AILine, s, n)
    u = mod(s, line.total) / (line.total / n); i = floor(Int, u)
    (mod(i, n) + 1, u - i)
end
@inline function _gcr(v::Vector{Float64}, s, line::AILine)
    n = length(v)
    CLASSICLAT_STEP[] && return v[_gidx(line, s, n)]
    (i, t) = _gfrac(line, s, n)
    p0 = v[mod1(i - 1, n)]; p1 = v[i]; p2 = v[mod1(i + 1, n)]; p3 = v[mod1(i + 2, n)]
    0.5*((2p1) + (-p0 + p2)*t + (2p0 - 5p1 + 4p2 - p3)*t^2 + (-p0 + 3p1 - 3p2 + p3)*t^3)
end
"Classic race-line lateral at s, or `nothing` when no table is loaded."
classic_racelane(line::AILine, s) = (g = CLASSICLAT[]; g === nothing ? nothing : _gcr(g[1], s, line))
"Classic rail offset from the race line at s for side +1 (left, pass1) / -1 (right, pass2); nothing without a table."
classic_rail(line::AILine, s, side) = (g = CLASSICLAT[]; g === nothing ? nothing :
    _gcr(side > 0 ? g[2] : g[3], s, line) - _gcr(g[1], s, line))

const CLASSICBAND = Ref{Union{Nothing,NTuple{2,Vector{Float64}}}}(nothing)
function set_classic_band!(lo, hi)
    if lo === nothing || hi === nothing
        CLASSICBAND[] = nothing; return nothing
    end
    a = Float64.(lo); b = Float64.(hi)

    CLASSICBAND[] = ([min(a[i], b[i]) for i in eachindex(a)], [max(a[i], b[i]) for i in eachindex(a)])
    nothing
end
const CLASSICBAND_ON = Ref(-1)
@inline function classicband_on()
    CLASSICBAND_ON[] < 0 && (CLASSICBAND_ON[] = get(ENV, "JM_AI_CLASSICBAND", "1") != "0" ? 1 : 0)
    CLASSICBAND_ON[] == 1
end
classicband_on!(on::Bool) = (CLASSICBAND_ON[] = on ? 1 : 0; on)
"""Classic's own lateral bounds at `s`, as `(lo, hi)`, or `nothing` when no corridor is loaded.
`margin` is kept clear of each wall so a car sits inside the corridor rather than on its edge."""
function classic_band(line::AILine, s; margin = 0.0)
    (classicband_on() && CLASSICBAND[] !== nothing) || return nothing
    g = CLASSICBAND[]; n = length(g[1])
    if CLASSICLAT_STEP[]
        i = _gidx(line, s, n); a = g[1][i]; b = g[2][i]
    else
        (i, t) = _gfrac(line, s, n); j = mod1(i + 1, n)
        a = g[1][i] + (g[1][j] - g[1][i])*t; b = g[2][i] + (g[2][j] - g[2][i])*t
    end
    lo = a + margin; hi = b - margin
    lo <= hi ? (lo, hi) : ((a + b)/2, (a + b)/2)
end

function _vtarget(line::AILine, s, v; amax, vmax, vmin, scale)
    g = CLASSICV[]
    if g !== nothing

        n = length(g); rec = line.total / n
        i0 = mod(floor(Int, mod(s, line.total) / rec), n) + 1
        vt = g[i0]
        horizon = max(v*2.2, 30.0); off = rec
        while off <= horizon
            vt = min(vt, g[mod(i0 - 1 + round(Int, off/rec), n) + 1]); off += rec
        end
        return clamp(vt*scale, vmin, vmax*scale)
    end
    κ0 = max(line.κ[_locate(line, s)[1]], 1e-4)
    horizon = max(v*2.2, 30.0)
    if VT_BRAKE[]

        vt = sqrt(amax/κ0)
        off = 5.0
        while off <= horizon
            κd = max(line.κ[_locate(line, s + off)[1]], 1e-4)
            vc = sqrt(amax/κd)
            vt = min(vt, sqrt(vc*vc + 2.0*BRAKE_A[]*off))
            off += 6.0
        end
        return clamp(vt*scale, vmin, vmax*scale)
    end
    κ = κ0
    off = 5.0
    while off <= horizon
        κ = max(κ, line.κ[_locate(line, s + off)[1]]); off += 6.0
    end
    clamp(sqrt(amax/κ)*scale, vmin, vmax*scale)
end

"Advance one AI car by `dt` on the race line; returns its world pose.  `scale` paces it
(see `natural_laptime`).  Single-car (no racecraft) — used for pace calibration."
function step!(car::AICar, line::AILine, dt; amax = AMAX[], vmax = VMAX[], vmin = 12.0, scale = 1.0)
    vt = _vtarget(line, car.s, car.v; amax, vmax, vmin, scale)
    car.v += clamp(vt - car.v, -30.0*dt, 9.0*dt)
    prev = mod(car.s, line.total); car.s += car.v*dt
    mod(car.s, line.total) < prev && (car.lap += 1)
    pose_at(line, car.s, car.lane)
end

"""Advance the AI field with Classic-style racecraft + simple collision physics.  Returns
`(poses, player_hit)`: each car's world pose (heading carries a collision yaw `spin`), and
whether an AI made contact with the human (so the app can give the player a bump).  `rel`
caps every AI to `rel × player_speed` so the field never runs away from the human."""

const PLAYER_OVL_LOG = Ref(false)
const PLAYER_OVL = NTuple{3,Float64}[]
mutable struct AIStat; engage::Int; release::Int; match::Int; qsnap::Int; sidepush::Int; mishap::Int; hardyield::Int; end
const AISTAT = AIStat(0, 0, 0, 0, 0, 0, 0)

const YIELD_RATE = Ref(something(tryparse(Float64, get(ENV, "JM_AI_YIELD_RATE", "6.0")), 6.0))
aistat_reset!() = (AISTAT.engage = AISTAT.release = AISTAT.match = AISTAT.qsnap = AISTAT.sidepush = AISTAT.mishap = AISTAT.hardyield = 0; nothing)

"""Free-running speed profile: one car alone on the line for a lap, sampled every `ds` metres.
Returns (s_samples, v_samples). This is what a car does with nobody ahead -- the baseline any
'fall back' must be measured against, because a car braking for Ascari is not falling back."""
function free_speed_profile(line::AILine; scale = 1.0, dt = 1/60, ds = 5.0, amax = AMAX[], vmax = VMAX[], vmin = 12.0)
    car = AICar(0.0, 25.0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    n = max(2, round(Int, line.total / ds)); vs = fill(NaN, n)
    for _ in 1:2
        while car.lap < 1
            step!(car, line, dt; amax, vmax, vmin, scale)
            k = clamp(floor(Int, mod(car.s, line.total) / ds) + 1, 1, n); vs[k] = car.v
        end
        car.lap = 0
    end
    for k in 1:n; isnan(vs[k]) && (vs[k] = vs[k == 1 ? n : k-1]); end
    ((0:n-1) .* ds, vs)
end

function step_field!(cars::Vector{AICar}, line::AILine, dt;
                     scale = 1.0, player = nothing, rel = Inf,
                     amax = 11.0, vmax = 74.0, vmin = 12.0)
    total = line.total
    blocker(s_i, skip) = begin
        bg = Inf; res = nothing
        for (k,c) in enumerate(cars)
            k == skip && continue
            g = mod(c.s - s_i, total); (0.0 < g < bg) && (bg = g; res = (g, c.lane, c.v))
        end
        if player !== nothing
            g = mod(player[1] - s_i, total); (0.0 < g < bg) && (bg = g; res = (g, player[2], player[3]))
        end
        res
    end

    for (i, car) in enumerate(cars)
        vt = _vtarget(line, car.s, car.v; amax, vmax, vmin, scale) * car.pace
        if car.mishap > 0.0
            car.mishap -= dt; vt *= 0.22
        elseif rand() < 8.0e-6
            car.mishap = 2.4; AISTAT.mishap += 1
        end
        b = blocker(car.s, i)
        gap   = b === nothing ? Inf : b[1]
        blane = b === nothing ? 0.0 : b[2]
        bv    = b === nothing ? Inf : b[3]
        if GAPCTL

            bg = Inf; bp = nothing
            for (k, c) in enumerate(cars)
                k == i && continue
                g = mod(c.s - car.s, total)
                (0.0 < g < bg && abs(c.lane - car.lane) < CAR_WID + 0.6) && (bg = g; bp = (g, c.lane, c.v))
            end
            if player !== nothing
                g = mod(player[1] - car.s, total)

                (0.0 < g < bg && abs(player[2] - car.lane) < CAR_WID + 0.6) && (bg = g; bp = (g, player[2], player[3]))
            end
            if bp !== nothing

                vt = max(min(vt, bp[3] + (bp[1] - CLASSIC_DESIRED_SEP) / CLASSIC_CLOSE_TAU), 0.0)
            end
        end
        if false
        end

        if car.tlane == 0.0

            if gap < car.v*1.0 + CLASSIC_DESIRED_SEP + ENGAGE_MARGIN && abs(car.lane - blane) < 2.2

                rl0  = something(classic_racelane(line, car.s), racelane(line, car.s))
                devL = clamp(rl0 + RAIL, -LANE_MAX, LANE_MAX) - rl0
                devR = rl0 - clamp(rl0 - RAIL, -LANE_MAX, LANE_MAX)
                away = blane >= 0.0 ? -RAIL : RAIL
                car.tlane = if !AVOID_ROOM || abs(devL - devR) < 0.15
                    away
                elseif devL > devR
                    RAIL
                else
                    -RAIL
                end
                AISTAT.engage += 1
            end
        else
            (gap > car.v*1.7 + 30.0) && (car.tlane = 0.0; AISTAT.release += 1)
            (gap < car.v*0.6 + CAR_LEN && abs(car.lane - blane) < 1.6) && (vt = min(vt, b[3]); AISTAT.match += 1)
        end
        isfinite(rel) && player !== nothing && (vt = min(vt, max(player[3]*rel, 6.0)))
        gl = classic_racelane(line, car.s)

        bnd = classic_band(line, car.s; margin = BAND_MARGIN)
        if gl === nothing
            tgt = racelane(line, car.s) + lanebias(i, length(cars)) + car.tlane
            if bnd === nothing
                tgt = clamp(tgt, -LANE_MAX, LANE_MAX)
                lane_step!(car, tgt, dt, LANE_MAX)
            else
                tgt = clamp(tgt, bnd[1], bnd[2])
                lane_step!(car, tgt, dt, max(abs(bnd[1]), abs(bnd[2])), bnd)
            end
        else

            dev = car.tlane == 0.0 ? 0.0 : something(classic_rail(line, car.s, car.tlane > 0 ? 1 : -1), car.tlane)
            tgt = gl + lanebias(i, length(cars)) + dev
            if bnd === nothing
                lim = max(LANE_MAX, abs(gl) + LANE_MAX)
                lane_step!(car, tgt, dt, lim)
            else
                tgt = clamp(tgt, bnd[1], bnd[2])
                lane_step!(car, tgt, dt, max(abs(bnd[1]), abs(bnd[2])), bnd)
            end
        end
        car.v     = advance_speed(car.v, vt, dt)
        car.spin *= exp(-dt/0.45)
        prev = mod(car.s, total); car.s += car.v*dt
        mod(car.s, total) < prev && (car.lap += 1)
    end

    n = length(cars)
    for a in 1:n, b in 1:n
        a == b && continue
        Δs = mod(cars[a].s - cars[b].s, total)
        Δs > total/2 && continue
        dl = cars[a].lane - cars[b].lane
        Δs < CAR_LEN || continue
        if abs(dl) < CAR_WID*0.7
            AISTAT.qsnap += 1
            cars[b].s = cars[a].s - CAR_LEN
            cars[b].v = min(cars[b].v, cars[a].v)
        elseif abs(dl) < CAR_WID

            push = (CAR_WID - abs(dl)) * 0.5; d = dl >= 0 ? 1.0 : -1.0; AISTAT.sidepush += 1
            cars[a].lane = clamp(cars[a].lane + d*push, -LANE_MAX, LANE_MAX)
            cars[b].lane = clamp(cars[b].lane - d*push, -LANE_MAX, LANE_MAX)
            cars[b].s = cars[a].s - CAR_LEN
            cars[b].v = min(cars[b].v, cars[a].v)
        end
    end

    player_hit = false
    if player !== nothing
        for c in cars
            Δs = mod(c.s - player[1] + total/2, total) - total/2

            if abs(Δs) < CAR_LEN && abs(c.lane - player[2]) < CAR_WID + CLEAR_MARGIN

                wasOverlap = abs(c.lane - player[2]) < CAR_WID

                (wasOverlap && PLAYER_OVL_LOG[]) && push!(PLAYER_OVL, (CAR_WID - abs(c.lane - player[2]), CAR_LEN - abs(Δs), Δs))
                d = (c.lane - player[2]) >= 0 ? 1.0 : -1.0

                step = min(1.3, YIELD_RATE[]*dt)
                step > 0.2 && (AISTAT.hardyield += 1)
                c.lane = clamp(c.lane + d*step, -LANE_MAX, LANE_MAX)
                c.spin += 0.22*d*(step/1.3); c.v *= 0.9

                wasOverlap && (player_hit = true)
            end
        end
    end
    poses = NTuple{4,Float64}[]
    for c in cars
        p = pose_at(line, c.s, c.lane); push!(poses, (p[1], p[2], p[3], p[4] + c.spin))
    end
    (poses, player_hit)
end

"""Brain-only racecraft for the HYBRID-PHYSICS AI: decide each car's target rail (`tlane`,
with the same pass-hysteresis as the kinematic field) and target speed, WITHOUT moving the
car — the JM physics model + the controller do the motion.  Each `AICar` must already have
its `s`, `lane`, `v` updated from its physics car (project it onto the line).  Returns the
per-car target speed `vt`."""
function plan!(cars::Vector{AICar}, line::AILine; player = nothing, scale = 1.0, rel = Inf,
               amax = 11.0, vmax = 74.0, vmin = 12.0, dt = 1/60)
    total = line.total
    blocker(s_i, skip) = begin
        bg = Inf; res = nothing
        for (k,c) in enumerate(cars)
            k == skip && continue
            g = mod(c.s - s_i, total); (0.0 < g < bg) && (bg = g; res = (g, c.lane, c.v))
        end
        if player !== nothing
            g = mod(player[1] - s_i, total); (0.0 < g < bg) && (bg = g; res = (g, player[2], player[3]))
        end
        res
    end

    maxκ(s, dist) = begin κ = 1e-4; off = 0.0; while off <= dist; κ = max(κ, line.κ[_locate(line, s+off)[1]]); off += 6.0; end; κ end
    vts = Float64[]
    for (i, car) in enumerate(cars)

        gaffe = (rand() < 0.0015) ? 1.10 : 1.0
        if car.mishap > 0.0; car.mishap -= dt; elseif rand() < 8.0e-6; car.mishap = 2.4; end
        mish  = car.mishap > 0.0 ? 0.22 : 1.0
        vt = _vtarget(line, car.s, car.v; amax, vmax, vmin, scale) * car.pace * gaffe * mish
        b = blocker(car.s, i); gap = b === nothing ? Inf : b[1]; blane = b === nothing ? 0.0 : b[2]; bv = b === nothing ? Inf : b[3]
        dlane  = b === nothing ? Inf : abs(car.lane - blane)
        tail   = car.v*0.9 + 10.0
        zone   = max(car.v*2.0, 45.0)

        straight = maxκ(car.s, zone) < 1/PASS_RADIUS
        overlap  = gap < CAR_LEN*0.7
        patience = 1.2 + 1.8*((0.37*i) % 1.0)
        edge   = bv + 1.0 < vt
        if car.tlane == 0.0
            if gap < tail && dlane < 2.4
                car.follow += dt

                if car.follow > patience && edge && (straight || overlap)
                    car.tlane = blane >= 0.0 ? -RAIL : RAIL
                    car.follow = 0.0
                elseif gap < car.v*0.6 + CAR_LEN
                    vt = min(vt, bv)
                end
            else
                car.follow = max(0.0, car.follow - dt)
            end
        else
            if gap > car.v*1.7 + 30.0
                car.tlane = 0.0
            elseif !straight && !overlap

                imminent = gap < CAR_LEN*1.6 && dlane < CAR_WID
                if AVOID_CORNER && imminent
                    side = blane >= 0.0 ? -1.0 : 1.0
                    car.tlane = side * min(RAIL, CAR_WID*0.9)
                    AVOIDSTAT[] += 1
                    if get(ENV, "JM_TRACE_AVOID", "") != "" && (AVOIDSTAT[] <= 3 || AVOIDSTAT[] % 200 == 0)
                        println("  [avoid] corner sidestep #", AVOIDSTAT[], "  gap=", round(gap, digits=2),
                                " dlane=", round(dlane, digits=2), " -> tlane=", round(car.tlane, digits=2))
                    end
                else
                    car.tlane = 0.0
                end
                vt = min(vt, bv)
            elseif gap < car.v*0.6 + CAR_LEN && dlane < 1.6
                vt = min(vt, bv)
            end
        end
        isfinite(rel) && player !== nothing && (vt = min(vt, max(player[3]*rel, 6.0)))
        push!(vts, vt)
    end
    vts
end

"""Steering/throttle/brake for one hybrid-physics AI to track the RACING LINE (plus a pass
`dev`iation, e.g. ±RAIL to pull out alongside) at speed `tv` (short-look-ahead pursuit +
cross-track + yaw-damp, corner throttle-cut).  `cs`/`clane` = the car's current arc-length/
lateral (from projection), `cθ`/`cv`/`r` its heading/speed/yaw-rate.  Returns (thr, brk, steer)."""
function controller(line::AILine, cs, clane, dev, tv, cx, cz, cθ, cv, r; power = 1.0)
    la = clamp(6.0 + cv*0.35, 6.0, 20.0)

    κloc = max(curv_at(line, cs), curv_at(line, cs + 0.5*la), 1e-4)
    la = clamp(min(la, 0.32/κloc), 4.0, 20.0)
    tlane = clamp(racelane(line, cs + la) + dev, -LANE_MAX, LANE_MAX)
    here  = clamp(racelane(line, cs)      + dev, -LANE_MAX, LANE_MAX)
    lp = pose_at(line, cs + la, tlane)
    herr = wrapπ(atan(lp[3]-cz, lp[1]-cx) - cθ)

    slide = clamp((abs(r) - SPIN_LO) / (SPIN_HI - SPIN_LO), 0.0, 1.0)
    hgain = 3.0 * (1.0 - 0.45*slide)
    yawd  = 0.22 + 0.55*slide
    steer = clamp(hgain*herr - 0.24*(clane - here) - yawd*r, -1.0, 1.0)

    thr = (cv < tv ? clamp((tv-cv)*0.25, 0, 1) * clamp(1.4 - 2.2*abs(steer), cv < 3.0 ? 0.35 : 0.0, 1.0) : 0.0) * power * (1.0 - 0.75*slide)
    brk = cv > tv + 1.0 ? clamp((cv-tv)*0.25, 0, 1) : 0.0

    (brk < 0.05 && abs(steer) > 0.2 && cv > tv*0.92) && (brk = clamp(0.10*abs(steer), 0.0, 0.18))
    (thr, brk, steer)
end

"""Grid order from a qualifying session.  `player_time` is the human's best qual lap
(`Inf` if they set none → starts last); `ai_times` the AI reference qual times.  Returns
the entrant ids sorted pole-first, where id 0 = the player and id i = AI car i.  The
position of id 0 in the result is the player's grid slot (1 = pole)."""
function grid_order(player_time, ai_times)
    entr = Tuple{Int,Float64}[(0, player_time)]
    for i in eachindex(ai_times); push!(entr, (i, Float64(ai_times[i]))); end
    sort!(entr, by = x -> x[2])
    [e[1] for e in entr]
end

"""Simulate one AI car around a full lap at the given `scale` and return the lap
time (s).  Used to calibrate the pace: knowing the natural lap time at scale 1.0,
the app picks the scale that makes a clean lap hit the Classic reference laptime ×
(100/pct).  Robust to a non-closing line (caps at ~2× the straight-line estimate)."""
function natural_laptime(line::AILine; scale = 1.0, dt = 1/60,
                        amax = AMAX[], vmax = VMAX[], vmin = 12.0)

    car = AICar(0.0, 25.0, 0, 0.0)
    t = 0.0; tmax = 4.0 * line.total / 10.0 + 30.0
    while car.lap < 1 && t < tmax
        step!(car, line, dt; scale = scale, amax = amax, vmax = vmax, vmin = vmin)
        t += dt
    end
    t
end

end
