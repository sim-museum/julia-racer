

const TT_GRIP   = get(ENV, "JM_TYRETEMP", "1") != "0"
const TT_AIR    = 25.6
const TT_TRACK  = 39.8
const TT_WORK   = parse(Float64, get(ENV, "JM_TYRETEMP_WORK", "45"))
const TT_COLD   = parse(Float64, get(ENV, "JM_TYRETEMP_COLD", "0.036"))
const TT_CEXP   = parse(Float64, get(ENV, "JM_TYRETEMP_CEXP", "1.0"))
const TT_HOT    = 100.0
const TT_T0     = let e = get(ENV, "JM_TYRE_T0", "")
    e == "warm" ? max(TT_WORK, 60.0) : something(tryparse(Float64, e), TT_TRACK)
end

const TT_S0 = 0.004
const TT_S1 = parse(Float64, get(ENV, "JM_TT_S1", "0.035"))
const TT_SN = parse(Float64, get(ENV, "JM_TT_SN", "3"))
const TT_GS = parse(Float64, get(ENV, "JM_TT_GS", "30"))

const TT_FRONT = parse(Float64, get(ENV, "JM_TT_FRONT", "2.5"))
const TT_KIN  = 0.020
const TT_KOUT = 0.005

const TT_WEAR_KF = parse(Float64, get(ENV, "JM_TT_WEAR_KF", "1.54e-4"))
const TT_WEAR_KR = parse(Float64, get(ENV, "JM_TT_WEAR_KR", "5.2e-5"))
const TT_WEAR = fill(1.0, 4, 3)
const TT_DIST = Ref(0.0)
const TT_SURF = fill(TT_T0, 4, 3)
const TT_CARC = fill(TT_T0, 4, 3)
const TT_MU   = fill(1.0, 4)
const TT_ON   = Ref(true)

const REPLAY_TT_CH = [string(q, "_", w, "_", z) for q in ("ts", "tc") for w in ("FL", "FR", "RL", "RR") for z in ("i", "m", "o")]
append!(REPLAY_TELE_P, REPLAY_TT_CH)
const REP_TT_LIVE = Ref(false)
const REPLAY_TTDIAG = get(ENV, "JM_REPLAY_TTDIAG", "0") != "0"
"Push the 24 zone temperatures onto a replay's channel buffer (REPLAY_TT_CH's order)."
tt_record!(buf) = (for A in (TT_SURF, TT_CARC), w in 1:4, z in 1:3; push!(buf, Float32(A[w, z])); end; nothing)
const TT_MG4  = 617.0*9.80665/4
const AD_TT_EXP = parse(Float64, get(ENV, "JM_AD_TT_EXP", "2.0"))

tt_reset!(T0 = TT_T0) = (fill!(TT_SURF, T0); fill!(TT_CARC, T0); fill!(TT_MU, 1.0); fill!(TT_WEAR, 1.0); TT_DIST[] = 0.0; nothing)

"Grip factor for a mean carcass temperature (1.0 warm; cold and overheated less)."
function tt_mu(Tc::Float64)
    TT_GRIP || return 1.0
    c = clamp((TT_WORK - Tc)/(TT_WORK - TT_TRACK), 0.0, 1.0)
    h = max(0.0, Tc - TT_HOT)/50
    clamp(1.0 - TT_COLD*c^TT_CEXP - 0.05*h, 0.6, 1.0)
end

"""One step of the four tyres: `tc` = cs.tc (Fx, Fy, grip limit per wheel, mg/4 units), `v` the car speed [m/s],
`wet` > 1 for a wet track (water cools: your very-wet race ran its surfaces at 28-30 C)."""
function tt_step!(tc, v::Float64, dt::Float64, wet::Int)
    (dt > 0 && isfinite(v)) || return
    dt = min(dt, 0.1); av = abs(v)
    ch = DriveRT3D.CHASSIS[]
    cool = wet > 1 ? 3.0 : 1.0
    tamb = wet > 1 ? TT_AIR : TT_TRACK
    for i in 1:4
        fx, fy, lim = tc[i]
        F = hypot(fx, fy) * TT_MG4; L = max(lim * TT_MG4, 1.0); u = clamp(F/L, 0.0, 1.2)
        P = F * av * (TT_S0 + TT_S1*u^TT_SN) / 1000
        wet > 1 && (P *= 0.6)
        i <= 2 && (P *= TT_FRONT)

        a = 0.30*clamp(fy*TT_MG4/L, -1.0, 1.0)
        left = i == 1 || i == 3
        g = (ch.camber === nothing ? 0.0 : ch.camber[i]); b = 0.20*clamp(-g/deg2rad(3.0), -1.0, 1.0)
        bin = left ? b : -b
        p = (hasproperty(ch, :pressure) && ch.pressure !== nothing) ? ch.pressure[i] : 152.0
        cm = 0.25*clamp((p - 152.0)/30.0, -1.0, 1.0)
        wL = max(0.05, 1/3 - a/2 - bin/2 - cm/2); wR = max(0.05, 1/3 + a/2 + bin/2 - cm/2); wM = max(0.05, 1/3 + cm)
        s = wL + wM + wR; w = (wL/s, wM/s, wR/s)
        mc = 0.0
        i == 1 && (TT_DIST[] += av*dt/1000)
        for z in 1:3
            TT_WEAR[i, z] = max(0.0, TT_WEAR[i, z] - (i <= 2 ? TT_WEAR_KF : TT_WEAR_KR)*3*w[z]*P*dt)
            Tc = TT_CARC[i, z]
            Ts_eq = Tc - 0.25*min(av/30, 1.0)*(Tc - TT_AIR)*cool + TT_GS*3*w[z]*P/cool
            TT_SURF[i, z] += (Ts_eq - TT_SURF[i, z]) * min(1.0, dt/0.8)
            TT_CARC[i, z] += dt*(TT_KIN*(TT_SURF[i, z] - Tc) - TT_KOUT*cool*(1 + av/30)*(Tc - tamb))
            mc += TT_CARC[i, z]/3
        end
        TT_MU[i] = tt_mu(mc)
    end
    nothing
end

"(inner, middle, outer) of wheel i from the L/M/R zones (left wheels: L is outer)."
tt_imo(A, i) = (i == 1 || i == 3) ? (A[i, 3], A[i, 2], A[i, 1]) : (A[i, 1], A[i, 2], A[i, 3])

const TT_DIAG = parse(Float64, get(ENV, "JM_TYRETEMP_DIAG", "0"))
const TT_DIAG_T = Ref(0.0)
function tt_diag(t::Float64)
    (TT_DIAG > 0 && t >= TT_DIAG_T[] + TT_DIAG) || return
    TT_DIAG_T[] = t
    r(x) = round(Int, x)
    println("  [tyretemp] t ", round(t; digits = 1), "  carcass ", join(("$(n) $(join(r.(TT_CARC[i, :]), "/"))" for (i, n) in enumerate(("FL", "FR", "RL", "RR"))), "  "),
            "  surface max ", r(maximum(TT_SURF)), " mean ", r(sum(TT_SURF)/12), "  grip ", join(round.(TT_MU; digits = 3), "/"),
            "  wear % ", join((round(100*(1 - sum(TT_WEAR[i, :])/3); digits = 3) for i in 1:4), "/"), " over ", round(TT_DIST[]; digits = 2), " km")
end
println("  [tyretemp] start ", TT_T0, " C, full grip from ", TT_WORK, " C carcass, cold penalty ", TT_GRIP ? TT_COLD : 0.0,
        TT_GRIP ? "" : " (JM_TYRETEMP=0: isothermal)")

const TT_FILE = let f = get(ENV, "JM_TYRETEMP_FILE", "")
    !isempty(f) ? f : SMOKE ? "" :
        joinpath(get(ENV, "XDG_CONFIG_HOME", joinpath(homedir(), ".config")), "juliaRacer", "garage", "last_tyres.json")
end
const TT_RUN = Ref(0.0); const TT_REST = Ref(0.0); const TT_SAVED = Ref(false)
function tt_save(t::Float64, why::String)
    isempty(TT_FILE) && return
    try
        mkpath(dirname(TT_FILE))
        io = IOBuffer(); f1(x) = string(round(x; digits = 1))
        print(io, "{\"track\": \"", TRACKSEL, "\", \"car\": \"", PLAYER_CAR, "\", \"time\": \"", Libc.strftime("%Y-%m-%d %H:%M", time()),
              "\", \"session_s\": ", f1(t), ", \"km\": ", f1(TT_DIST[]), ", \"why\": \"", why, "\", \"tyres\": {")
        for (i, n) in enumerate(("LF", "RF", "LR", "RR"))
            c = tt_imo(TT_CARC, i); sf = tt_imo(TT_SURF, i)
            wr = tt_imo(TT_WEAR, i)
            print(io, i > 1 ? ", " : "", "\"", n, "\": {\"carcass_imo\": [", join(f1.(c), ", "), "], \"surface_imo\": [", join(f1.(sf), ", "),
                  "], \"tread_imo\": [", join((string(round(100x; digits = 2)) for x in wr), ", "), "], \"grip\": ", round(TT_MU[i]; digits = 3), "}")
        end
        print(io, "}}\n")
        tmp = TT_FILE * ".tmp"; write(tmp, take!(io)); mv(tmp, TT_FILE; force = true)
        println("  [tyretemp] readout (", why, ") -> ", TT_FILE)
    catch e
        println("  [tyretemp] readout not written: ", e)
    end
end
"Per frame: a car that comes to rest after a run gets its readout."
function tt_rest_step!(v::Float64, t::Float64, dt::Float64)
    if abs(v) > 0.5
        TT_RUN[] += dt; TT_REST[] = 0.0; TT_SAVED[] = false
    else
        TT_REST[] += dt
        (TT_REST[] >= 2.0 && TT_RUN[] >= 20.0 && !TT_SAVED[]) && (tt_save(t, "stopped"); TT_SAVED[] = true; TT_RUN[] = 0.0)
    end
end

const MAN_THR = (0.01, 0.06, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.94, 1.0)
const MAN_RPM = (2250.0, 3750.0, 5250.0, 6750.0, 8250.0, 9500.0)
const MAN_TAB = [0.403 0.271 0.257 0.243 0.244 0.252;
                 0.400 0.283 0.269 0.252 0.252 0.252;
                 0.465 0.369 0.342 0.315 0.307 0.307;
                 0.577 0.497 0.464 0.430 0.407 0.407;
                 0.687 0.594 0.550 0.523 0.510 0.485;
                 0.777 0.688 0.644 0.615 0.605 0.595;
                 0.847 0.769 0.731 0.702 0.702 0.685;
                 0.901 0.837 0.812 0.795 0.783 0.775;
                 0.941 0.905 0.882 0.872 0.870 0.867;
                 0.977 0.944 0.931 0.924 0.920 0.917;
                 0.981 0.981 0.971 0.960 0.976 0.976;
                 0.992 0.992 0.992 0.992 0.992 0.992]
@inline function _bracket(K, x)
    x <= K[1] && return (1, 1, 0.0)
    x >= K[end] && return (length(K), length(K), 0.0)
    i = findlast(k -> k <= x, K); (i, i + 1, (x - K[i])/(K[i+1] - K[i]))
end
"reference's manifold pressure [bar] at a pedal (0..1) and rpm."
function manifold_bar(thr::Real, rpm::Real)
    (i0, i1, a) = _bracket(MAN_THR, Float64(thr)); (j0, j1, b) = _bracket(MAN_RPM, Float64(rpm))
    v0 = MAN_TAB[i0, j0] + b*(MAN_TAB[i0, j1] - MAN_TAB[i0, j0]); v1 = MAN_TAB[i1, j0] + b*(MAN_TAB[i1, j1] - MAN_TAB[i1, j0])
    v0 + a*(v1 - v0)
end
