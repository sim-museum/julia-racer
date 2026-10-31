

using GLFW, ModernGL, LinearAlgebra, Dates, Serialization

const _T0 = time()

const _TCOMP = get(ENV,"JM_TIMING","0") == "2" && (Base.cumulative_compile_timing(true); true)
tstamp(lbl) = get(ENV,"JM_TIMING","0") != "0" && (println("[t+", round(time()-_T0, digits=1), "s", _TCOMP ? " c" * string(round(Base.cumulative_compile_time_ns()[1]/1e9, digits=1)) : "", "] ", lbl); flush(stdout); true)
using JRTrack

if get(ENV, "JM_PHYS_INCLUDE", "0") != "0"
    include(normpath(joinpath(@__DIR__,"..","..","JuliaMotorMTK","src","drive_rt.jl"))); using .DriveRT
    include(normpath(joinpath(@__DIR__,"..","..","JuliaMotorMTK","src","drive_rt3d.jl"))); using .DriveRT3D
else
    using JRPhysics
    using JRPhysics.DriveRT, JRPhysics.DriveRT3D
end
include(joinpath(@__DIR__,"people_filter.jl")); using .PeopleFilter
include(joinpath(@__DIR__,"classic_lp.jl")); using .CLASSICLP
include(joinpath(@__DIR__,"susp_pose.jl")); using .SuspPose
include(joinpath(@__DIR__,"wreck_seal.jl")); using .WreckSeal
include(joinpath(@__DIR__, "wreck_body.jl"))

include(normpath(joinpath(@__DIR__,"..","..","JuliaMotorMTK","src","ibt.jl"))); using .IBT

if get(ENV, "JM_RENDER_INCLUDE", "0") != "0"
    include("render.jl"); using .Render
    include("classictrack.jl"); using .CLASSICTrack
    include("classicwall.jl"); using .CLASSICWall
    include(joinpath(@__DIR__, "roadcurve.jl")); using .RoadCurve
else
    using JRRender
    using JRRender: Render, CLASSICTrack, CLASSICWall, RoadCurve
    using .Render, .CLASSICTrack, .CLASSICWall, .RoadCurve
end
"""E107-S7: refit polyline A (Classic's aligned .trk centreline) onto B (our re-centred line, same indexing) by least
squares: rotation + uniform scale + translation. The result is Classic's reference line where its .lp records and its
.trk walls land on the drawn road (E107-S8 default)."""
function rigid_refit(A, B)
    n = min(length(A), length(B))
    ax = sum(p[1] for p in A[1:n])/n; az = sum(p[2] for p in A[1:n])/n
    bx = sum(p[1] for p in B[1:n])/n; bz = sum(p[2] for p in B[1:n])/n
    sxx = sum((A[k][1]-ax)*(B[k][1]-bx) + (A[k][2]-az)*(B[k][2]-bz) for k in 1:n)
    sxy = sum((A[k][1]-ax)*(B[k][2]-bz) - (A[k][2]-az)*(B[k][1]-bx) for k in 1:n)
    saa = sum((A[k][1]-ax)^2 + (A[k][2]-az)^2 for k in 1:n)
    th = atan(sxy, sxx); sc = hypot(sxx, sxy)/saa
    [(bx + sc*(cos(th)*(p[1]-ax) - sin(th)*(p[2]-az)), bz + sc*(sin(th)*(p[1]-ax) + cos(th)*(p[2]-az))) for p in A]
end
include(joinpath(@__DIR__, "classicroot.jl"))
include("audio.jl"); using .EngineAudio
include("joycfg.jl"); using .JoyCfg
include("ffb.jl"); using .FFB
include("ai.jl"); using .RaceAI
include("classicai.jl"); using .CLASSICAI

const SMOKE_NOJOY = haskey(ENV, "JM_SMOKE") && get(ENV, "JM_SMOKE_JOY", "0") == "0"
const FFB_ON     = !haskey(ENV, "JM_NOFFB") && !SMOKE_NOJOY
const FFB_GAIN   = parse(Float64, get(ENV, "JM_FFB_GAIN", "1.3"))
const FFB_SIGN   = parse(Float64, get(ENV, "JM_FFB_SIGN", "-1.0"))
const FFB_ATRAIL = parse(Float64, get(ENV, "JM_FFB_TRAIL", "0.18"))
const FFB_TFLOOR = 0.40
const FFB_AF     = 1.314
const FFB_DELTA  = 0.30
const FFB_SQ     = parse(Float64, get(ENV, "JM_FFB_SQ",  "0.03"))
const FFB_LP     = parse(Float64, get(ENV, "JM_FFB_LP",  haskey(ENV, "JM_FFB_LEGACY") ? "0.05" : "0.015"))
const FFB_SPRING = parse(Float64, get(ENV, "JM_FFB_SPRING", "0.55"))

const FFB_PHYS    = !haskey(ENV, "JM_FFB_LEGACY")
const FFB_TREF    = parse(Float64, get(ENV, "JM_FFB_TREF", "8.0"))
const FFB_DESPIKE = parse(Float64, get(ENV, "JM_FFB_DESPIKE", FFB_PHYS ? "0.015" : "0.05"))

const FFB_IRMODE  = FFB_PHYS && get(ENV, "JM_FFB_MODE", "reference") != "legacy"
const FFB_MAXNM   = parse(Float64, get(ENV, "JM_FFB_MAXNM", "28.33"))
const FFB_SPRING_IR = parse(Float64, get(ENV, "JM_FFB_SPRING_IR", "0.0"))
const _JOYCONF = joinpath(@__DIR__, "joystick.conf")

const JOYNAME = try
    GLFW.Init(); (!SMOKE_NOJOY && GLFW.JoystickPresent(GLFW.JOYSTICK_1)) ? String(GLFW.GetJoystickName(GLFW.JOYSTICK_1)) : ""
catch
    ""
end
const JOYMAP, JOYSRC = JoyCfg.resolve(_JOYCONF, JOYNAME)
println("  controller: ", JOYSRC)

const WHEEL_HALF_DEG = JoyCfg.wheel_half_range_deg(JOYNAME; measured = JOYMAP.wheel_half_deg)
const STEER_GAIN = Ref(0.0)

const STEER_LIM = Ref(1.0)
const STEER_RATIO = Ref(10.0)

if JOYMAP.clutch.axis < 1
    @warn """clutch is NOT on an axis (clutch.axis=$(JOYMAP.clutch.axis)) — the PO's standing
             requirement is a SLIDER you can ride. Check $(_JOYCONF); the X3D default is axis 4."""
else
    println("  clutch: axis ", JOYMAP.clutch.axis, " (ridable slider)  <- ", JOYSRC)
end

function choose_track()
    haskey(ENV, "TRACK")     && return lowercase(ENV["TRACK"])
    haskey(ENV, "JM_SMOKE")  && return "zandvoort"
    isa(stdin, Base.TTY)     || return "zandvoort"
    println("""

      ╔════════════════════════════════════════════════╗
      ║          juliaMotor — choose your track         ║
      ╠════════════════════════════════════════════════╣
      ║   1) Zandvoort     Classic 1967 circuit (default)   ║
      ║   2) Skidpad       flat centripetal pad,        ║
      ║                    Ø10–200 m circles (10 m step) ║
      ║   3) Nürburgring   Classic Nordschleife 1967,       ║
      ║                    22.7 km, full elevation      ║
      ╚════════════════════════════════════════════════╝""")
    print("\n  Track [1/2/3] (Enter = 1): "); flush(stdout)
    s = strip(readline())
    return s == "2" ? "skidpad" : s == "3" ? "nurburgring" : "zandvoort"
end

const TRACKKEYS = ("zandvoort", "skidpad", "nurburgring", "monza", "watglen", "spa")
const TRACKSEL = let t = choose_track()
    if !(t in TRACKKEYS)
        println("!! unknown TRACK=\"", t, "\" -- known tracks: ", join(TRACKKEYS, ", "))
        println("!! refusing to run: an unknown name would silently load Zandvoort under that label.")
        flush(stdout); exit(2)
    end
    t
end
const SKIDPAD  = TRACKSEL == "skidpad"
const NURB     = TRACKSEL == "nurburgring"

const RINGBANNER_DROP = get(ENV, "JM_RINGBANNER", "0") == "0"
const RINGBANNER_TEX  = Set(lowercase.(split(get(ENV, "JM_RINGBANNER_TEX", "pitrim"), ",")))
ringbanner_zone(found, lapdist, lateral) = NURB && RINGBANNER_DROP && found && 1300.0 < lapdist < 1800.0 && lateral < -2.0
const MONZA    = TRACKSEL == "monza"
const WATGLEN  = TRACKSEL == "watglen"
const ZANDV    = TRACKSEL == "zandvoort"
const SPA      = TRACKSEL == "spa"

const MZ_ROAD_B = parse(Float64, get(ENV, "JM_MONZA_ROAD_B", "0.51"))
const MZ_ROAD_A = parse(Float64, get(ENV, "JM_MONZA_ROAD_A", "0.12"))
const MZ_DARK_B = parse(Float64, get(ENV, "JM_MONZA_DARK_B", "1.05"))
const MZ_DARK_A = parse(Float64, get(ENV, "JM_MONZA_DARK_A", "0.55"))
const MZ_BANK_B = parse(Float64, get(ENV, "JM_MONZA_BANK_B", "0.55"))
const MZ_BANK_A = parse(Float64, get(ENV, "JM_MONZA_BANK_A", "0.22"))
const MZ_OTHER_B = parse(Float64, get(ENV, "JM_MONZA_OTHER_B", "0.72"))
const MZ_OTHER_A = parse(Float64, get(ENV, "JM_MONZA_OTHER_A", "0.34"))
println("  → track: ", uppercasefirst(TRACKSEL))

const HAT_EXCLUDE = get(ENV,"JM_HAT_KEEP_BRIDGE","0") != "0" ? Set{String}() :
                    Set{String}(["br_under", "villone"])
const HAT_EXCLUDE_PRED = nothing
const ROAD_PRED = MONZA ? (lt -> occursin("asp", lt) || startswith(lt,"groove") || startswith(lt,"kerb")) : nothing

const ROAD_TEX = lt -> occursin("asp", lt) || startswith(lt,"groove") || startswith(lt,"curb") ||
                       startswith(lt,"kerb") || lt in ("sline","sgrid","start") ||

                       startswith(lt,"atog") || startswith(lt,"a_l_g") || lt == "concrete" ||

                       startswith(lt,"borcem") || lt == "asfa" || lt == "griline"

const MODE      = lowercase(get(ENV, "JM_MODE", "practice"))
const RACE_LAPS = max(1, tryparse(Int, get(ENV, "JM_LAPS", "3")) |> x -> x === nothing ? 3 : x)
const AI_COOLDOWN = get(ENV, "JM_AI_COOLDOWN", "1") != "0"
const AI_COOL_DIAG = get(ENV, "JM_AI_COOLDIAG", "0") != "0"

const AI_COOL_PACE = parse(Float64, get(ENV, "JM_AI_COOLPACE", "0.9"))
const PITPARK = get(ENV, "JM_PITPARK", "1") != "0"
const AI_COOL_STOPPED = Set{Int}()
const AI_COOL = Dict{Int,Tuple{Symbol,Float64,Float64}}()
const AI_ROADSHIFT_CHECK = get(ENV, "JM_AI_ROADSHIFT_CHECK", "1") != "0"
const AI_PITLINE = get(ENV, "JM_AI_PITLINE", "1") != "0"
const PIT_IN = Ref(NaN); const PIT_BOX = Ref(NaN); const PIT_END = Ref(NaN)
const PRACTICE_SEC = 60.0 * (tryparse(Float64, get(ENV, "JM_PRACTICE_MIN", "15")) |> x -> x === nothing ? 15.0 : x)

const N_AI      = let rf = get(ENV, "JM_REPLAY", "")
    n = if !isempty(rf) && isfile(rf)
        try; clamp(deserialize(rf).ncar - 1, 0, 6); catch; 6; end
    else
        clamp(tryparse(Int, get(ENV, "JM_AI", "0")) |> x -> x === nothing ? 0 : x, 0, 6)
    end

    netm = lowercase(get(ENV, "JM_NET", ""))
    if netm == "join" && n > 0 && !haskey(ENV, "JM_NET_AI")
        println("  ⚠ networked session (client): AI field disabled ($n → 0) -- the HOST steps the ",
                "field and sends its poses; they are drawn here as remote cars (MP-5). ",
                "JM_NET_AI=1 steps a local field instead (diverging fields).")
        n = 0
    elseif netm == "host" && n > 0
        println("  ☑ networked session (host): host-authoritative AI -- the $n-car field is stepped ",
                "here and its poses are sent to the client every packet interval (MP-5).")
    end
    n
end

const N_AI_REQ  = clamp(tryparse(Int, get(ENV, "JM_AI", "0")) |> x -> x === nothing ? 0 : x, 0, 6)
const IS_RACE   = MODE == "race"
const IS_TRAIN  = MODE == "training"

const AI_PCT    = clamp(tryparse(Float64, get(ENV, "JM_AI_PCT", "60")) |> x -> x === nothing ? 100.0 : x, 30.0, 200.0)

const AI_REL    = haskey(ENV, "JM_AI_REL") ? clamp(parse(Float64, ENV["JM_AI_REL"]), 1.0, 4.0) : Inf

const AI_AMAX   = clamp(parse(Float64, get(ENV, "JM_AI_AMAX", "8.0")), 4.0, 14.0)

const AI_HEADSTART = max(0.0, parse(Float64, get(ENV, "JM_AI_HEADSTART", "0")))

const COUNTDOWN = max(0.0, parse(Float64, get(ENV, "JM_COUNTDOWN", "5")))

const ARM_KEY = max(0.0, parse(Float64, get(ENV, "JM_ARM_KEY", "0")))
const START_ARM = (get(ENV, "JM_NO_START_ARM", "0") == "0" && get(ENV, "JM_AUTODRIVE", "0") == "0") || ARM_KEY > 0
const ARM_AFTER = max(0.0, parse(Float64, get(ENV, "JM_ARM_AFTER", "0")))

const CTRACE = get(ENV, "JM_CTRACE", "1") != "0"

onroad_at(x::Float64, z::Float64) = JRTrack.hat3d(TERRAIN0, x, z; ref = Inf)[3]
const CTRACE_UNTIL = Ref(0.0)

const CTRACE_RACE = parse(Float64, get(ENV, "JM_CTRACE_RACE", "Inf"))

const CTRACE_PROF = get(ENV, "JM_CTRACE_PROF", "")
!isempty(CTRACE_PROF) && @eval Main using Profile
function ctrace_start!(why, secs = 2.0)
    CTRACE || return nothing
    CTRACE_UNTIL[] == 0.0 && ccall(:jl_force_trace_compile_timing_enable, Cvoid, ())
    (!isempty(CTRACE_PROF) && CTRACE_UNTIL[] == 0.0 && (why == CTRACE_PROF || why == "ready")) &&
        Base.invokelatest(() -> (Main.Profile.clear(); Main.Profile.start_timer()))
    CTRACE_UNTIL[] = max(CTRACE_UNTIL[], time() + secs)
    println(stderr, "  [ctrace] ", why, ": tracing compiles for ", secs, " s"); flush(stderr)
    nothing
end
ctrace_tick!() = (CTRACE_UNTIL[] > 0.0 && time() >= CTRACE_UNTIL[] &&
                  (ccall(:jl_force_trace_compile_timing_disable, Cvoid, ()); CTRACE_UNTIL[] = 0.0;
                   !isempty(CTRACE_PROF) && Base.invokelatest(() -> (Main.Profile.stop_timer();
                       Main.Profile.print(stderr; format = :tree, C = false, mincount = 3, maxdepth = 60, noisefloor = 1.0)))); nothing)

const POLE = get(ENV, "JM_POLE", "0") != "0"

const AI_PHYSICS = haskey(ENV, "JM_AI_PHYSICS")

const CLASSICAI_ON = !AI_PHYSICS && get(ENV, "JM_AI_MODEL", "classic") == "classic"

const AI_POWER  = clamp(tryparse(Float64, get(ENV, "JM_AI_POWER", "0.90")) |> x -> x === nothing ? 0.90 : x, 0.4, 1.0)
const CONTACT_D = parse(Float64, get(ENV, "JM_CONTACT_D", "2.1"))

"""
    prog_delta(ps, prev, total; teleport=false) -> Float64

Arc-length to add to the player's accumulated lap progress this frame.

PO 2026-09-04 (Watkins Glen): "my first lap was listed at 1.19 seconds ... maybe because I hit r
to get back on the road at one point". R (respawn) and SHIFT+R (recover) TELEPORT the car BACKWARD
along the centreline. The unwrap here assumes every step is a small FORWARD one, so a backward jump
of more than half a lap has `+= total` applied and comes out as a large FORWARD step -- crediting
up to 40% of a lap in one frame, which banks an instant lap and corrupts the lap COUNT (and with it
`cs.laps >= RACE_LAPS`, the race-over announcement, and the replay written at exit).

A teleport is not motion, so it contributes NOTHING. Extracted as a function so a gate can exercise
the real rule instead of a copy of it.
"""
function prog_delta(ps::Real, prev::Real, total::Real; teleport::Bool = false)
    teleport && return 0.0
    ds = ps - prev
    ds < -total/2 && (ds += total)
    ds >  total/2 && (ds -= total)
    return abs(ds) < total*0.4 ? float(ds) : 0.0
end

const CAR_HALF_L = parse(Float64, get(ENV, "JM_CAR_HALF_L", "2.0"))
const CAR_HALF_W = parse(Float64, get(ENV, "JM_CAR_HALF_W", "0.9"))
const CONTACT_CIRCLE = haskey(ENV, "JM_CONTACT_D")

"""Ellipse radius of a car heading `θ` measured along the unit direction (ux,uz)."""
carrad(θ, ux, uz) = hypot(CAR_HALF_L * (cos(θ)*ux + sin(θ)*uz),
                          CAR_HALF_W * (-sin(θ)*ux + cos(θ)*uz))

"""Contact threshold between two cars separated by (dx,dz), |d|, heading θa and θb."""
contact_d(dx, dz, d, θa, θb) =
    CONTACT_CIRCLE ? CONTACT_D : (carrad(θa, dx/d, dz/d) + carrad(θb, dx/d, dz/d))

include("carcontact.jl")

const AI_LOOSE = get(ENV, "JM_AI_LOOSE", "1") != "0"
const LOOSE_N = 2
const LOOSE_CARS = Any[]
const LOOSE_OF = Dict{Int,Int}()
const LOOSE_T0 = zeros(LOOSE_N)
const LOOSE_DIAG = get(ENV, "JM_LOOSE_DIAG", "0") != "0"
const LOOSE_HITS = zeros(Int, LOOSE_N)
const LOOSE_REC = falses(LOOSE_N)
const LOOSE_DR = 0.6; const LOOSE_DV = 4.0; const LOOSE_UP = 2.0
const LOOSE_BRAKE = parse(Float64, get(ENV, "JM_LOOSE_BRAKE", "1.0"))
const LOOSE_STAT = Ref((n = 0, maxdr = 0.0, maxup = 0.0))
const LOOSE_DROP = (f = split(get(ENV, "JM_LOOSE_DROP", ""), ":"); length(f) == 4 && haskey(ENV, "JM_SMOKE") ? Tuple(parse.(Float64, f)) : nothing)
const LOOSE_DROP_DONE = Ref(false)
const RAM_AI = (r = get(ENV, "JM_RAM_AI", ""); isempty(r) ? nothing : Tuple(parse.(Float64, split(r, ":"))))
const RAM_STATE = Ref((t0 = NaN, done = false))
const RAM_HIT = Ref(NaN)
const RAM_SOLO = get(ENV, "JM_RAM_SOLO", "0") != "0"; const RAM_SOLO_DONE = Ref(false)
"""CARCOLLIDE-2 S4: AI car k (pose p, heading aθ, world velocity (avx,avz)) hit by the player car `pc0`. Hands it to a
free pool car when the hit is hard, or applies the impulse to its pool car when it is loose already. true = handled here
(the caller skips the field's own gentle answer)."""
function loose_hit!(k, p, aθ, avx, avz, pc0, pvx, pvz, pr, t)
    (AI_LOOSE && !isempty(LOOSE_CARS)) || return false
    sl = get(LOOSE_OF, k, 0)

    if haskey(AI_WRECK, k)
        b = AI_WRECK[k].b
        ib = car_contact_impulse(p[1], p[3], aθ, b.v[1], -b.v[3], (b.R*b.ω)[2], pc0.x, pc0.z, pvx, pvz; θb = pc0.θ, rb = pr)
        ib === nothing || WreckBody.kick!(b, WreckBody.V3(ib[1], ib[5], -ib[2]), Float64(ib[3]))
        return true
    end
    if AI_WHEELOFF_ON
        (wvx, wvz) = sl > 0 ? loose_wvel(sl) : (Float64(avx), Float64(avz))
        if !isnan(ai_tear_wheel!(k, p, wvx, wvz, Float64(pc0.x), Float64(pc0.z), Float64(pvx), Float64(pvz), Float64(t)))
            ib = car_contact_impulse(p[1], p[3], aθ, wvx, wvz, sl > 0 ? loose_yawrate(sl) : 0.0, pc0.x, pc0.z, pvx, pvz; θb = pc0.θ, rb = pr)
            ib === nothing && (ib = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            sl > 0 && (delete!(LOOSE_OF, k); LOOSE_BODY[sl] = nothing)
            ai_wreck_start!(k, p, wvx + ib[1], wvz + ib[2], Float64(ib[3]), Float64(ib[5]), Float64(ib[6]), Float64(ib[7]))
            return true
        end
    end
    if sl > 0
        pc = LOOSE_CARS[sl]
        (vx, vz) = loose_wvel(sl)
        ib = car_contact_impulse(pc.x, pc.z, pc.θ, vx, vz, loose_yawrate(sl), pc0.x, pc0.z, pvx, pvz; θb = pc0.θ, rb = pr)
        ib === nothing && return true
        b = LOOSE_BODY[sl]
        b === nothing ? DriveRT3D.bump3d!(pc, ib[1], ib[2], ib[3], ib[5], ib[6], ib[7]) :
                        WreckBody.kick!(b, WreckBody.V3(ib[1], ib[5], -ib[2]), Float64(ib[3]))
        LOOSE_HITS[sl] += 1
        return true
    end
    ib = car_contact_impulse(p[1], p[3], aθ, avx, avz, 0.0, pc0.x, pc0.z, pvx, pvz; θb = pc0.θ, rb = pr)
    ib === nothing && return false
    (abs(ib[3]) > LOOSE_DR || ib[5] > LOOSE_UP || hypot(ib[1], ib[2]) > LOOSE_DV) || return false
    used = Set(values(LOOSE_OF)); sl = findfirst(i -> !(i in used), eachindex(LOOSE_CARS))
    sl === nothing && return false
    pc = LOOSE_CARS[sl]

    DriveRT3D.pin3d!(pc, p[1], p[3], aθ, p[2])
    pc.s_vel(pc.integ, [hypot(avx, avz), 0.0]); pc.v = hypot(avx, avz)
    DriveRT3D.bump3d!(pc, ib[1], ib[2], ib[3], ib[5], ib[6], ib[7])
    LOOSE_OF[k] = sl; LOOSE_T0[sl] = t; LOOSE_REC[sl] = false; LOOSE_HITS[sl] = 0; LOOSE_BODY[sl] = nothing
    st = LOOSE_STAT[]; LOOSE_STAT[] = (n = st.n + 1, maxdr = max(st.maxdr, abs(ib[3])), maxup = max(st.maxup, ib[5]))
    println("  [loose] t=", round(t, digits = 2), " AI ", k, " knocked loose: dv ", round(hypot(ib[1], ib[2]), digits = 2),
            " m/s, yaw kick ", round(ib[3], digits = 2), " rad/s, launch ", round(ib[5], digits = 2), " m/s"); flush(stdout)
    true
end

const CARHIT_DAMAGE = get(ENV, "JM_CARHIT_DAMAGE", "1") != "0"
const CARHIT_N = Ref(0); const CARHIT_MAX = Ref(0.0)
function carhit_damage!(cs, pdvx, pdvz, j)
    (CARHIT_DAMAGE && CAR3D && !WRECKED[]) || return nothing
    closing = j/((1 + CAR_E)*CAR_M/2)
    closing > 0.0 || return nothing
    cθ = cos(cs.θ); sθ = sin(cs.θ)
    fxb = pdvx*cθ + pdvz*sθ; fyb = -pdvx*sθ + pdvz*cθ
    DriveRT3D.damage_impact!(fxb, fyb, closing)
    CARHIT_N[] += 1; CARHIT_MAX[] = max(CARHIT_MAX[], closing)
    closing > DriveRT3D.DMG_ONSET_MS && println("  [carhit] t=", round(cs.t, digits = 2), " closing ", round(closing, digits = 2),
                                               " m/s  damage ", round.(DriveRT3D.DAMAGE, digits = 3), " engine ", round(DriveRT3D.ENGINE_DAMAGE[], digits = 3))
    nothing
end

const SPIN_LIM  = parse(Float64, get(ENV, "JM_SPIN_LIM", "1.4"))

const DRAFT_LEN = 26.0; const DRAFT_LAT = 2.8; const TOW_MAX = 4.0

const DRAFT_DRAG_CUT = clamp(parse(Float64, get(ENV, "JM_DRAFT_CUT", "0.45")), 0.0, 0.8)
function draft_tow(fx, fz, fθ, fv, leads)
    fv < 26.0 && return 0.0
    hx = cos(fθ); hz = sin(fθ); best = 0.0
    for L in leads
        dx = L[1]-fx; dz = L[2]-fz; ahead = dx*hx + dz*hz
        (ahead < 2.0 || ahead > DRAFT_LEN) && continue
        abs(-dx*hz + dz*hx) > DRAFT_LAT && continue
        abs(RaceAI.wrapπ(L[3]-fθ)) > 0.5 && continue
        best = max(best, TOW_MAX*(1.0 - ahead/DRAFT_LEN))
    end
    best
end

const CARHALF = 1.4

const OBJ_PR = Tuple(parse.(Float64, split(get(ENV, "JM_OBJ_PR", "-1,0"), ",")))

const INVIS_SOLIDS_DIAG = get(ENV, "JM_INVIS_SOLIDS", "0") != "0"
const INVIS_SOLID_ROWS = Tuple{Float64,Float64,String,Float64}[]
const RAIL_ROLL = parse(Float64, get(ENV, "JM_RAIL_ROLL", "1"))
objrollsign(nm) = (OBJ_PR[2] == 0 && startswith(lowercase(nm), "armco")) ? RAIL_ROLL : OBJ_PR[2]
const OBJ_CPU = Dict{String,Any}()
const SolidT    = Tuple{Float64,Float64,Float64,Symbol}
const SolidBoxT = Union{Nothing,NTuple{3,Float64}}
global SOLIDS::Vector{SolidT}      = SolidT[]
global SOLIDBOX::Vector{SolidBoxT} = SolidBoxT[]
"signed gap from (px,pz) to solid k's boundary and its outward normal (box if it has one, else disc)"
@inline function solid_gap(px, pz, k)
    (ox, oz, r, _) = SOLIDS[k]
    b = k <= length(SOLIDBOX) ? SOLIDBOX[k] : nothing
    b === nothing ? disc_gap(px, pz, ox, oz, r) : box_gap(px, pz, ox, oz, b[1], b[2], b[3])
end

const CARW  = parse(Float64, get(ENV, "JM_CARW",  "0.95"))
const CARLF = parse(Float64, get(ENV, "JM_CARLF", "1.05"))
const CAR_CIRCLE = get(ENV, "JM_CAR_CIRCLE", "0") != "0"
@inline function car_gap(x, z, θ, k)
    if CAR_CIRCLE
        (g, nx, nz) = solid_gap(x, z, k); return (g - CARHALF, nx, nz)
    end
    cθ = cos(θ); sθ = sin(θ)
    (gf, nfx, nfz) = solid_gap(x + CARLF*cθ, z + CARLF*sθ, k)
    (gr, nrx, nrz) = solid_gap(x - CARLF*cθ, z - CARLF*sθ, k)
    gf <= gr ? (gf - CARW, nfx, nfz) : (gr - CARW, nrx, nrz)
end
function solid_hit(x, z, θ, v)
    v < 1.2 && return nothing
    @inbounds for k in eachindex(SOLIDS)
        (gap, nx, nz) = car_gap(x, z, θ, k)
        gap >= 0.0 && continue
        (ox, oz) = SOLIDS[k]; dx = x - ox; dz = z - oz
        vn = v*cos(θ)*nx + v*sin(θ)*nz
        vn >= -0.3 && continue
        across = -dx*sin(θ) + dz*cos(θ)
        along  =  dx*cos(θ) + dz*sin(θ)
        e = 0.45; j = (1+e)*(-vn)
        lift  = clamp(abs(across)/CARHALF * (-vn) * 0.45, 0.0, 6.0)
        droll = clamp(sign(across)*lift*0.8, -5.0, 5.0)
        dpitch = clamp(sign(along)*lift*0.7, -4.0, 4.0)
        dr    = clamp(-sign(across)*(-vn)*0.04, -1.0, 1.0)
        return (j*nx, j*nz, dr, lift, droll, dpitch)
    end
    nothing
end

solidkind(nm) = (startswith(nm,"haie")||startswith(nm,"bush")||startswith(nm,"shrub")||startswith(nm,"hedge")||
                 startswith(nm,"hay")||startswith(nm,"straw")) ? :soft : :wall

const WVX = Ref(0.0); const WVZ = Ref(0.0)
const PRVX = Ref(NaN); const PRVZ = Ref(NaN)

function update_world_velocity!(cs, x, z, dt)
    if CAR3D
        (WVX[], WVZ[]) = DriveRT3D.world_velocity(cs)
    elseif isfinite(PRVX[]) && dt > 1e-6
        WVX[] = (x - PRVX[])/dt; WVZ[] = (z - PRVZ[])/dt
    else
        WVX[] = 0.0; WVZ[] = 0.0
    end
    PRVX[] = x; PRVZ[] = z
    (WVX[], WVZ[])
end
function solid_contact(x, z, θ, v, dt)
    Fx = 0.0; Fy = 0.0; Mz = 0.0; peak = 0.0
    hardpk = 0.0
    closing = 0.0
    @inbounds for k in eachindex(SOLIDS)
        kind = SOLIDS[k][4]
        (gap, nx, nz) = car_gap(x, z, θ, k)
        gap >= 0.0 && continue
        vn = WVX[]*nx + WVZ[]*nz

        (fx, fy, mz) = DriveRT3D.contact_force(-gap, nx, nz, vn, θ; kind = kind, dt = dt)
        Fx += fx; Fy += fy; Mz += mz; peak = max(peak, hypot(fx, fy))
        kind === :soft || (hardpk = max(hardpk, hypot(fx, fy)))

        kind === :soft || (closing = max(closing, -vn))
    end

    if Fx != 0.0 || Fy != 0.0
        cθ = cos(θ); sθ = sin(θ)
        vbx =  WVX[]*cθ + WVZ[]*sθ; vby = -WVX[]*sθ + WVZ[]*cθ
        (Fx, Fy, sc) = DriveRT3D.cap_total_contact(Fx, Fy, vbx, vby; dt = dt)
        Mz *= sc
    end
    (Fx, Fy, Mz, peak, hardpk, closing)
end

const REF_LAP = Dict("zandvoort"=>86.848, "nurburgring"=>501.931, "watglen"=>66.912,
                     "monza"=>90.202, "spa"=>200.342, "skidpad"=>30.0)
const AI_REFLAP = (v = tryparse(Float64, get(ENV, "JM_AI_REFLAP", ""));
                   v === nothing ? get(REF_LAP, TRACKSEL, 90.0) : v)
println("  → mode: ", uppercasefirst(MODE),
        IS_RACE ? "  ($RACE_LAPS laps" * (N_AI>0 ? ", $N_AI AI cars)" : ")") : "")

const FUEL_LPK    = clamp(tryparse(Float64, get(ENV,"JM_FUEL_LPK","0.55")) |> x-> x===nothing ? 0.55 : x, 0.05, 5.0)
const FUEL_MARGIN = max(0, tryparse(Int, get(ENV,"JM_FUEL_MARGIN","10")) |> x-> x===nothing ? 10 : x)

const FUEL_MODE = lowercase(strip(get(ENV, "JM_FUEL", "event")))
const FUEL_PLAN_LPK = 0.40
const FUEL_SETUP_L = Ref(NaN)
const FUEL_START_L = Ref(NaN)
function fuel_start_litres(laplen_m)
    FUEL_MODE == "full" && return DriveRT3D.FUEL_MAX_L
    x = tryparse(Float64, FUEL_MODE); x !== nothing && return clamp(x, 1.0, DriveRT3D.FUEL_MAX_L)
    laps = (IS_RACE ? RACE_LAPS : 10) + 2
    clamp(laps*FUEL_PLAN_LPK*laplen_m/1000, 1.0, DriveRT3D.FUEL_MAX_L)
end

const IBTREC0 = !haskey(ENV, "JM_NOIBT")

include(joinpath(@__DIR__, "step_guard.jl")); using .StepGuard
include(joinpath(@__DIR__, "solid_geom.jl")); using .SolidGeom

const REPLAY_FILE_R = Ref(get(ENV, "JM_REPLAY", ""))

const REPLAY_TELE_P  = ["lap", "lapdist", "speed", "throttle", "brake", "steer", "clutch", "gear", "rpm", "lateral", "ontrack", "race",

                        [string(q, "_", w) for w in ("FL", "FR", "RL", "RR") for q in ("fx", "fy", "gx", "gy", "xi")]...]
const REPLAY_TYRE_CH = [string(q, "_", w) for w in ("FL", "FR", "RL", "RR") for q in ("fx", "fy", "gx", "gy", "xi")]
const REPLAY_TELE_AI = ["s", "speed", "lap", "lane"]

append!(REPLAY_TELE_P, ["pitch", "roll", "body"]); append!(REPLAY_TELE_AI, ["pitch", "roll"])

push!(REPLAY_TELE_P, "prog")
const REPLAY_ROLL = get(ENV, "JM_REPLAY_ROLL", "1") != "0"

const REPCAM_HEAD = Ref(NaN)
const REPCAM_TILT = get(ENV, "JM_REPCAM_TILT", "1") != "0"
const REPCAM_EARTH = get(ENV, "JM_REPCAM_EARTH", "1") != "0"
const REP_BODY = Ref((NaN, NaN))
const REPLAY_BODYDIAG = get(ENV, "JM_REPLAY_BODYDIAG", "0") != "0"
const REP_DIAGN = Ref(0)
_jstr(x::AbstractString) = "\"" * replace(x, "\\" => "\\\\", "\"" => "\\\"") * "\""
_jarr(v) = "[" * join((x isa AbstractString ? _jstr(x) : string(x) for x in v), ",") * "]"
"""REPLAY-2: write the session's replay -- the .jmr (Julia, played back by the sim) and the .jrt (one JSON header line,
then float32 frames: poses (1 + 4 ncar) followed by tele (12 + 4 (ncar-1)) per frame), for the launcher's analyser.
Each write replaces the files whole (temp + rename), so an autosave never leaves a torn file."""
function write_replay(out, buf, tele, ncar, names, line; final = true)
    try
        mkpath(dirname(out))
        np = 1 + 4*ncar; nt = length(REPLAY_TELE_P) + length(REPLAY_TELE_AI)*(ncar - 1)
        nf = min(length(buf) ÷ np, length(tele) ÷ nt)
        tmp = out * ".tmp"
        serialize(tmp, (track=TRACKSEL, ncar=ncar, names=names, fps=15, nframes=nf, data=buf[1:nf*np], version=2,
                        tele=tele[1:nf*nt], tele_p=REPLAY_TELE_P, tele_ai=REPLAY_TELE_AI, laplen=Float64(LAPLEN),
                        wheels=replay_wheels_upto(REPLAY_WHEELBUF, nf)))
        mv(tmp, out; force = true)
        jrt = replace(out, r"\.jmr$" => ".jrt")

        ref = line === nothing ? Tuple{Float64,Float64}[] :
              [(round(line.x[i], digits=1), round(line.z[i], digits=1)) for i in 1:max(1, length(line.x) ÷ 1500):length(line.x)]
        hdr = "{\"format\":\"jrt\",\"version\":1,\"track\":" * _jstr(TRACKSEL) * ",\"laplen\":" * string(round(LAPLEN, digits=1)) *
              ",\"line_total\":" * string(line === nothing ? 0.0 : round(line.total, digits=1)) * ",\"fps\":15,\"ncar\":" * string(ncar) *
              ",\"nframes\":" * string(nf) * ",\"names\":" * _jarr(names) * ",\"pose\":[\"t\",\"x\",\"y\",\"z\",\"heading\"]" *
              ",\"tele_player\":" * _jarr(REPLAY_TELE_P) * ",\"tele_ai\":" * _jarr(REPLAY_TELE_AI) * ",\"final\":" * string(final) *

              ",\"sections\":[" * join(("[$(round(sec[1], digits=1)),$(_jstr(sec[2]))]" for sec in SECTIONS), ",") * "]" *
              ",\"carsetup\":" * _jstr(get(ENV, "JM_CARSETUP", "default")) * ",\"gearbox\":" * _jstr(get(ENV, "ZAND_SHIFT", "auto")) *
              ",\"refline\":[" * join(("[$(p[1]),$(p[2])]" for p in ref), ",") * "]}"
        open(jrt * ".tmp", "w") do io
            write(io, hdr, "\n")
            for f in 0:nf-1
                write(io, view(buf, f*np+1:(f+1)*np)); write(io, view(tele, f*nt+1:(f+1)*nt))
            end
        end
        mv(jrt * ".tmp", jrt; force = true)
        final && println("  wrote replay: ", out, "  (", nf, " frames, ", filesize(out)÷1024, " KB) + ", basename(jrt))
    catch e
        println("  replay export failed: ", e)
    end
end

const IBTDIR = let repo = normpath(joinpath(@__DIR__,"..","..","data","reference")),
                   golds = (joinpath(homedir(), "gold standard", "julia racer"), "/home/admin/gold standard/julia racer"),
                   hasibt(g) = isdir(g) && !isempty(filter(f->endswith(lowercase(f),".ibt"), readdir(g)))
    d = get(ENV, "JM_IBTDIR", "")
    !isempty(d) ? d : something(findfirst(hasibt, golds) |> i -> i === nothing ? nothing : golds[i], repo)
end
println("  reference reference (.ibt template, session gearbox/mass): ", IBTDIR)

function ibt_anchors()
    f = joinpath(IBTDIR, "ai_anchors.toml"); d = Dict{String,String}()
    isfile(f) || return d
    for l in eachline(f)
        m = match(r"^\s*(\w+)\s*=\s*\"?([^\"#]*?)\"?\s*(#.*)?$", l); m === nothing || (d[m[1]] = strip(m[2]))
    end
    d
end
function jm_anchor_from_ibt()
    a = ibt_anchors()
    if haskey(a, "lat_accel_p99")
        RaceAI.set_anchor_from_ibt!(parse(Float64, a["lat_accel_p99"]))
        println("  → AI grip anchor ", round(RaceAI.AMAX[], digits=2), " m/s² (", round(RaceAI.AMAX[]/9.81, digits=2), " g) from ",
                get(a, "lat_accel_source", "?"), " (LatAccel p99 of ", get(a, "lat_accel_samples", "?"), " samples, ai_anchors.toml)")
        return
    end

    files = try
        all = filter(f -> endswith(lowercase(f), ".ibt"), readdir(IBTDIR; join = true))
        skid = filter(f -> occursin("skidpad", lowercase(basename(f))), all)
        vcat(sort(skid; by = filesize, rev = true),
             sort(filter(!in(Set(skid)), all); by = filesize, rev = true))
    catch e
        @warn "AI grip anchor: cannot read $IBTDIR ($e) -- keeping fallback $(RaceAI.AMAX[])"; return
    end
    isempty(files) && (@warn "AI grip anchor: no .ibt in $IBTDIR -- keeping fallback $(RaceAI.AMAX[])"; return)
    for f in files
        try
            v = filter(isfinite, abs.(IBT.channel(IBT.ibt_open(f), "LatAccel")))
            length(v) < 500 && continue
            u = sort(v); p99 = u[clamp(ceil(Int, 0.99*length(u)), 1, length(u))]
            got = RaceAI.set_anchor_from_ibt!(p99)
            println("  → AI grip anchor ", round(RaceAI.AMAX[], digits=2), " m/s² (",
                    round(RaceAI.AMAX[]/9.81, digits=2), " g) from ", basename(f),
                    " (LatAccel p99 of ", length(v), " samples)")
            return
        catch e
            @warn "AI grip anchor: $(basename(f)) unusable ($e)"
        end
    end
    @warn "AI grip anchor: no usable .ibt LatAccel -- keeping fallback $(RaceAI.AMAX[])"
end
jm_anchor_from_ibt()

function jm_vmax_from_ibt()
    a = ibt_anchors()
    if haskey(a, "rpm_limit")
        rpmlim = parse(Float64, a["rpm_limit"]); rw = DriveRT3D.RW_R
        RaceAI.set_vmax_from_drivetrain!(rw * rpmlim * 2π/60 / (DriveRT3D.GEARS[end] * DriveRT3D.FINAL[]))
        println("  → AI vmax anchor ", round(RaceAI.VMAX[], digits=1), " m/s (", round(Int, 3.6*RaceAI.VMAX[]), " km/h) from rev limit ",
                round(Int, rpmlim), " rpm in ", get(a, "rpm_limit_source", "?"), "  [RW_R=", rw, " m] (ai_anchors.toml)")
        return
    end
    files = try
        sort(filter(f -> endswith(lowercase(f), ".ibt"), readdir(IBTDIR; join = true)); by = filesize, rev = true)
    catch e
        @warn "AI vmax anchor: cannot read $IBTDIR ($e) -- keeping fallback $(RaceAI.VMAX[])"; return
    end
    isempty(files) && (@warn "AI vmax anchor: no .ibt in $IBTDIR -- keeping fallback $(RaceAI.VMAX[])"; return)
    rpmlim = 0.0; src = ""
    for f in files
        try
            r = filter(isfinite, IBT.channel(IBT.ibt_open(f), "RPM"))
            length(r) < 500 && continue
            m = maximum(r)
            if m > rpmlim; rpmlim = m; src = basename(f); end
        catch e
            @warn "AI vmax anchor: $(basename(f)) unusable ($e)"
        end
    end
    if rpmlim <= 0
        @warn "AI vmax anchor: no usable RPM channel -- keeping fallback $(RaceAI.VMAX[])"
        return
    end
    rw = DriveRT3D.RW_R
    v  = rw * rpmlim * 2π/60 / (DriveRT3D.GEARS[end] * DriveRT3D.FINAL[])
    RaceAI.set_vmax_from_drivetrain!(v)
    println("  → AI vmax anchor ", round(RaceAI.VMAX[], digits=1), " m/s (",
            round(Int, 3.6*RaceAI.VMAX[]), " km/h) from rev limit ", round(Int, rpmlim),
            " rpm in ", src, "  [RW_R=", rw, " m]")
end
jm_vmax_from_ibt()

const IBTNAME = get(Dict("nurburgring" => "nurburgring nordschleife",
                         "zandvort"    => "zandvoort"),
                    TRACKSEL, TRACKSEL)

include(normpath(joinpath(@__DIR__,"..","..","JuliaMotorMTK","src","setup.jl"))); using .Setup
include(normpath(joinpath(@__DIR__,"..","..","JuliaMotorMTK","src","garage.jl"))); using .Garage

const GARAGE_NOTE = Ref("")
const GARAGE_PP = Ref{Any}(nothing)
function session_params(fn::AbstractString)
    pp = Setup.setup_params(IBT.session_yaml(IBT.ibt_open(fn)))
    Ks = Garage.calibrate_session(pp)

    K0 = Garage.julia_setup(Ks, isww103(pp) ? "ww103" : "default")
    g = get(ENV, "JM_GARAGE", "")
    if isempty(g) || !isfile(g)
        own = Garage.knob_diff(K0, Ks)
        isempty(own) && return pp
        GARAGE_NOTE[] = "garage: julia's setup -- " * join(own, ", ") * " changed from the session  <- garage_lotus49.toml [julia_setup]"
        pg = Garage.apply_garage(pp, K0; Ks)
        GARAGE_PP[] = pg
        return pg
    end
    _, K = Garage.read_garage_file(g, K0)
    changed = Garage.knob_diff(K, Ks)
    GARAGE_NOTE[] = isempty(changed) ? "garage: unchanged from the session" :
                    "garage: " * join(changed, ", ") * " changed from the session  <- " * basename(g)
    pg = Garage.apply_garage(pp, K; Ks)
    isempty(changed) || (GARAGE_PP[] = pg)
    pg
end

"The session YAML only (header: sessionInfoLen @16, sessionInfoOffset @20) -- no need to read a 40 MB file to see its setup."
function _ibt_yaml(path)
    open(path) do io
        h = read(io, 112); length(h) < 112 && return ""
        len = Int(reinterpret(Int32, h[17:20])[1]); off = Int(reinterpret(Int32, h[21:24])[1])
        seek(io, off); String(read(io, len))
    end
end
function _official_ring(dir)
    best = ""; bestv = ""
    for (r, _, fs) in (isdir(dir) ? walkdir(dir) : ()), fn in fs
        (startswith(lowercase(fn), "lotus49_nurburgring") && endswith(lowercase(fn), ".ibt")) || continue
        y = try _ibt_yaml(joinpath(r, fn)) catch; continue end
        m = match(r"DriverSetupName: (.*)", y); md = match(r"DriverSetupIsModified: (\d)", y); cv = match(r"DriverCarVersion: (\S+)", y)
        (m !== nothing && occursin("rung_official", m.captures[1]) && !occursin("skidpad", m.captures[1]) &&
         md !== nothing && md.captures[1] == "0") || continue
        v = cv === nothing ? "" : cv.captures[1]
        (v > bestv || (v == bestv && fn > basename(best))) && (best = joinpath(r, fn); bestv = v)
    end
    best
end
const IBTTMPL_DEFAULT = let want = (SKIDPAD ? "lotus49_skidpad" : "lotus49_nurburgring"),
                    cands = isdir(IBTDIR) ? sort(filter(f -> startswith(lowercase(f), want) &&
                                                             endswith(lowercase(f), ".ibt"),
                                                        readdir(IBTDIR))) : String[]
    off = (SKIDPAD || lowercase(get(ENV, "JM_DEFAULT_TEMPLATE", "")) == "june") ? "" : _official_ring(IBTDIR)
    !isempty(off) ? off : isempty(cands) ? joinpath(IBTDIR, want * " (none found).ibt") : joinpath(IBTDIR, cands[1])
end

const CARSETUPS = (
    (key = "default", label = "reference default",  note = "the reference Lotus 49 setup Julia has always raced"),
    (key = "ww103",   label = "WW103 fast loose", note = "Wolfgang Wagner's Classic Watkins Glen setup via reference: soft front, stiff rear bar, 35° drive ramp, toe-out"),
)
function choose_carsetup()
    haskey(ENV, "JM_CARSETUP") && return lowercase(strip(ENV["JM_CARSETUP"]))
    (haskey(ENV, "JM_SMOKE") || !isa(stdin, Base.TTY)) && return "default"
    println("""

      ╔════════════════════════════════════════════════╗
      ║          juliaMotor — car setup                 ║
      ╚════════════════════════════════════════════════╝""")
    for (i, c) in enumerate(CARSETUPS); println("   ", i, ") ", rpad(c.label, 18), c.note); end
    print("\n  Setup [1/2] (Enter = 1): "); flush(stdout)
    s = strip(readline())
    s == "2" ? "ww103" : "default"
end
const CARSETUP = let k = choose_carsetup()
    if !any(c -> c.key == k, CARSETUPS)
        println("!! unknown JM_CARSETUP=\"", k, "\" -- known: ", join((c.key for c in CARSETUPS), ", "))
        flush(stdout); exit(2)
    end
    k
end
isww103(sp) = sp.diff_drive_ramp == 35 && sp.diff_coast_ramp == 85 && sp.spring_rate_Npmm[:LF] == 18
const IBTTMPL = if CARSETUP == "ww103"
    want = SKIDPAD ? "lotus49_skidpad" : "lotus49_nurburgring"
    cands = String[]
    for (r, _, fs) in (isdir(IBTDIR) ? walkdir(IBTDIR) : ()), fn in sort(fs)
        (startswith(lowercase(fn), want) && endswith(lowercase(fn), ".ibt")) || continue
        f = joinpath(r, fn)
        ok = try isww103(Setup.setup_params(IBT.session_yaml(IBT.ibt_open(f)))) catch; false end
        ok && push!(cands, f)
    end
    if isempty(cands)

        println("!! WW103: no ", want, " session with the WW103 setup under ", IBTDIR,
                " (expected its 261005 folder) -- driving the reference DEFAULT setup instead")
        IBTTMPL_DEFAULT
    else
        first(cands)
    end
else
    IBTTMPL_DEFAULT
end
println("  car setup: ", CARSETUP == "ww103" && IBTTMPL != IBTTMPL_DEFAULT ? "WW103 fast loose" : "reference default",
        "   <- ", basename(IBTTMPL))

let
    if isfile(IBTTMPL)
        try
            pp = session_params(IBTTMPL)
            PPSRC = basename(IBTTMPL) * (startswith(GARAGE_NOTE[], "garage: unchanged") || isempty(GARAGE_NOTE[]) ? "" : " + garage")
            isempty(GARAGE_NOTE[]) || println("  ", GARAGE_NOTE[], "\n  garage:  ", Garage.describe_derived_pp(pp))
            DriveRT3D.set_transmission!(pp.gear_ratios, pp.final_drive; source = PPSRC)
            isfinite(pp.shift_rpm) && DriveRT3D.set_shift_rpm!(pp.shift_rpm; source = PPSRC)

            DriveRT3D.set_corner_weights!(pp.corner_weight_N; source = PPSRC)
            FUEL_SETUP_L[] = isfinite(pp.fuel_L) ? pp.fuel_L : 43.2

            let rh = pp.ride_height_mm
                if all(k -> haskey(rh, k) && isfinite(rh[k]) && rh[k] > 0, (:LF,:RF,:LR,:RR))
                    DriveRT3D.set_ride_height!(rh[:LF]/1000, rh[:RF]/1000, rh[:LR]/1000, rh[:RR]/1000;
                                               source = PPSRC)
                end
            end
            let sp = pp.spring_rate_Npmm
                if all(k -> haskey(sp, k) && isfinite(sp[k]) && sp[k] > 0, (:LF,:RF,:LR,:RR))
                    DriveRT3D.set_suspension!(DriveRT3D.wheel_rate(sp[:LF]), DriveRT3D.wheel_rate(sp[:RF]),
                                              DriveRT3D.wheel_rate(sp[:LR]; rear = true), DriveRT3D.wheel_rate(sp[:RR]; rear = true);
                                              source = PPSRC)
                end
            end

            DriveRT3D.set_chassis!(DriveRT3D.chassis_from_setup(pp; source = PPSRC))
        catch e
            @warn "E100: could not read the gearbox from $(basename(IBTTMPL)); the built-in \
                   fallback is the SKIDPAD setup and is wrong for a circuit" e
        end
    else
        @warn "E100: no ibt at $IBTTMPL -- gearbox falls back to built-in constants (SKIDPAD setup)"
    end

    println("  gearbox: ", DriveRT3D.GEARS, "  final ", DriveRT3D.FINAL[],
            "   <- ", DriveRT3D.transmission_source())
    println("  AUTO up-shift: ", round(Int, DriveRT3D.SHIFT_RPM[]), " rpm (shift light)   <- ", DriveRT3D.SHIFT_SRC[])

    let ratio = (isfile(IBTTMPL) ? (try session_params(IBTTMPL).steering_ratio catch; NaN end) : NaN),
        src = isfinite(ratio) ? basename(IBTTMPL) : "built-in 10:1 (Lotus 49 setup) -- NOT from an ibt"
        isfinite(ratio) && ratio > 1 || (ratio = 10.0)
        STEER_RATIO[] = ratio
        if WHEEL_HALF_DEG > 0
            STEER_GAIN[] = WHEEL_HALF_DEG / ratio / rad2deg(DriveRT3D.MAXSTEER)
            STEER_LIM[] = max(1.0, min(WHEEL_HALF_DEG, DriveRT3D.WHEEL_LOCK_DEG) / ratio / rad2deg(DriveRT3D.MAXSTEER))
            println("  steering: wheel ±", round(Int, WHEEL_HALF_DEG), "° / ratio ", ratio, ":1 -> full lock (",
                    round(rad2deg(DriveRT3D.MAXSTEER)*STEER_LIM[], digits = 1), "° road) at ±", round(Int, rad2deg(DriveRT3D.MAXSTEER)*STEER_LIM[]*ratio),
                    "° of wheel (reference's lock ", round(Int, DriveRT3D.WHEEL_LOCK_DEG), "°)   <- ", src)
        else
            println("  steering: endpoint calibration (not a wheel with a known rotation range)")
        end
    end
    println("  mass:    ", round(DriveRT3D.MASS[], digits=1), " kg  front ",
            round(100*DriveRT3D.FRONT_FRAC[], digits=1), "%")

    println("  springs: ", join((round(Int, k) for k in DriveRT3D.KS[]), " / "),
            " N/m (FL/FR/RL/RR)   <- ", DriveRT3D.KS_SRC[])
    println("  ride ht: ", join((round(1000*h, digits=1) for h in DriveRT3D.RIDE_H[]), " / "),
            " mm (FL/FR/RL/RR)   <- ", DriveRT3D.RIDE_H_SRC[])
    println("  chassis: ", DriveRT3D.describe_chassis())
end

const NETMODE  = lowercase(get(ENV, "JM_NET", ""))
const NET_HOST = get(ENV, "JM_NET_HOST", "127.0.0.1")
const NET_PORT = parse(Int, get(ENV, "JM_NET_PORT", "47700"))
const NET_ID   = parse(Int, get(ENV, "JM_NET_ID", NETMODE == "host" ? "1" : "2"))
const NET_HZ   = parse(Float64, get(ENV, "JM_NET_HZ", "10"))
const NET_DIAG = parse(Int, get(ENV, "JM_NET_DIAG", "0"))

const FIXED_DT    = parse(Float64, get(ENV, "JM_FIXED_DT", "0"))
const WHEELOFF_RULE = get(ENV, "JM_WHEELOFF_RULE", "1") != "0"

const INSTANT_ON = get(ENV, "JM_INSTANT_REPLAY", "1") != "0" && isempty(NETMODE) && !haskey(ENV, "JM_NOREPLAY")
const INSTANT_TEST = get(ENV, "JM_INSTANT_TEST", "0") != "0"
const INSTANT_REQ = Ref(false)

const WARM_HOLD = get(ENV, "JM_WARM_HOLD", "0") == "1" && isempty(NETMODE)
function warm_refuse(p, names)
    isfile(p) || return "no such file"
    jrt = replace(p, r"\.jmr$" => ".jrt")
    isfile(jrt) || return "no .jrt beside it"
    hdr = try readline(jrt) catch; "" end
    m = match(r"\"track\":\"([^\"]*)\"", hdr); (m === nothing || m[1] != TRACKSEL) && return "another track"
    m = match(r"\"names\":\[([^\]]*)\]", hdr); m === nothing && return "no car names"
    fn = [strip(x, ['"', ' ']) for x in split(m[1], ",")]
    fn == [string(x) for x in names] || return "another field ($(length(fn)) cars: $(join(fn, ", ")))"
    nothing
end

function warm_hide!()
    GLFW.GetWindowMonitor(win).handle != C_NULL && GLFW.SetWindowMonitor(win, GLFW.Monitor(C_NULL), 40, 40, W, H, GLFW.DONT_CARE)
    GLFW.HideWindow(win); GLFW.PollEvents()
end

const WARM_LINES = Ref{Union{Nothing,Channel{String}}}(nothing)
function warm_lines()
    WARM_LINES[] === nothing || return WARM_LINES[]
    lines = Channel{String}(4); WARM_LINES[] = lines
    @async (while true; l = try readline(stdin) catch; "" end; put!(lines, l); (eof(stdin) || isempty(strip(l))) && break; end)
    lines
end
function warm_hold(names)
    warm_hide!()
    lines = warm_lines()
    while true
        println("  [warm] holding ", TRACKSEL, " (", length(names), " cars) -- waiting for a replay"); flush(stdout)
        while !isready(lines); GLFW.PollEvents(); sleep(0.05); end
        l = strip(take!(lines))
        (isempty(l) || l == "quit") && return nothing
        startswith(l, "replay ") || continue
        p = String(strip(l[8:end])); why = warm_refuse(p, names)
        why === nothing && (GLFW.ShowWindow(win); go_fullscreen!(); return p)
        println("  [warm] declined: ", why); flush(stdout)
        return nothing
    end
end
const INSTANT_AT_END = Ref(false)
const WHEELOFF_MSG = Ref(0.0)

const MONZA_TREEBANK_DROP = get(ENV, "JM_MONZA_TREEBANK_DROP", "0") != "0"
const SAFE_SPECT = get(ENV, "JM_SAFE_SPECTATORS", "0") != "0"
const SPECT_MARGIN = parse(Float64, get(ENV, "JM_SPECT_MARGIN", "4.0"))
const RECOVER_AT  = Float64[parse(Float64, x) for x in split(get(ENV, "JM_RECOVER_AT", ""), ",") if !isempty(strip(x))]
const RECOVER_I   = Ref(1)
const WRECK_AT    = parse(Float64, get(ENV, "JM_WRECK_AT", "0"))
const WRECK_AT_DONE = Ref(false)

const WHEELOFF_AT = let f = split(get(ENV, "JM_WHEELOFF_AT", ""), ":"); length(f) == 2 ? (parse(Float64, f[1]), String(f[2])) : nothing end
const WHEELOFF_AT_DONE = Ref(false)
const WRECK_ROLL  = parse(Float64, get(ENV, "JM_WRECK_ROLL", "0"))
const CARHIT_TRACE = get(ENV, "JM_CARHIT_TRACE", "") == "1"
const WRECK_ROLL_DUE = Ref(false)
import Random
const AI_SEED = something(tryparse(Int, get(ENV, "JM_AI_SEED", "")), haskey(ENV, "JM_SMOKE") ? 1967 : rand(1:typemax(Int32)))
const AI_RNG  = Ref{Any}(nothing)
const RESTART_AT  = Float64[parse(Float64, x) for x in split(get(ENV, "JM_RESTART_AT", ""), ",") if !isempty(strip(x))]
const RESTART_I   = Ref(1)

const LASTZ_AT = (f = split(get(ENV, "JM_LASTZ_AT", ""), ":"); length(f) == 2 ? (parse(Float64, f[1]), parse(Float64, f[2])) : nothing)
const LASTZ_SET = Ref(false)

const AI_OFFROAD_DIAG = get(ENV, "JM_AI_OFFROAD", "0") != "0"
const AI_OFF = [0, 0, 0, 0]; const AI_OFFBIN = Dict{Int,Int}(); const AI_EDGEBIN = Dict{Int,Int}()

const AI_OFF_POS = tryparse(Float64, get(ENV, "JM_AI_OFFROAD_POS", ""))
const JOLT_MS = parse(Float64, get(ENV, "JM_JOLT_MS", "3.0"))
const JOLT_PRE = Ref((0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0))
const JOLT_N = Ref(0)
haskey(ENV, "JM_SOFT_DVMAX") && (JRPhysics.DriveRT3D.SOFT_DVMAX_OVERRIDE[] = parse(Float64, ENV["JM_SOFT_DVMAX"]))
const AI_NOWHEELS = get(ENV, "JM_NO_AI_WHEELS", "0") != "0"
const DRIVECHECK  = get(ENV, "JM_DRIVECHECK", "0") != "0"
const VDIAG = get(ENV, "JM_VDIAG", "0") != "0"; const VDIAG_T = Ref(-1.0)
const DC_AIR_MAX  = parse(Float64, get(ENV, "JM_DC_AIR",   "0.75"))
const DC_VZ_MAX   = parse(Float64, get(ENV, "JM_DC_VZ",    "6.0"))
const DC_STUCK_V  = parse(Float64, get(ENV, "JM_DC_STUCKV","2.0"))
const DC_STUCK_S  = parse(Float64, get(ENV, "JM_DC_STUCKS","8.0"))
const DC_SETTLE_S = parse(Float64, get(ENV, "JM_DC_SETTLE","1.0"))
mutable struct DriveCheck
    airmax::Float64; air_s::Float64; air_bad::Int
    vzmax::Float64;  vz_s::Float64;  vz_bad::Int
    stuck::Float64;  stuckmax::Float64; stuck_s::Float64
    lastz::Float64;  lastt::Float64;  maxs::Float64
    inair::Bool;     events::Vector{Tuple{Float64,Float64,Float64,Float64,Float64}}

    off_bad::Int;    off_s::Float64;  off_y::Float64;  offevents::Vector{Tuple{Float64,Float64,Float64}}
    ghmin::Float64
end
const DC = Ref(DriveCheck(0.0,0.0,0, 0.0,0.0,0, 0.0,0.0,0.0, 0.0,0.0,0.0, false, Tuple{Float64,Float64,Float64,Float64,Float64}[], 0, 0.0, 0.0, Tuple{Float64,Float64,Float64}[], Inf))
const AUTODRIVE   = get(ENV, "JM_AUTODRIVE", "0") != "0"
const AUTODRIVE_V = parse(Float64, get(ENV, "JM_AUTODRIVE_V", "45"))

const AUTODRIVE_LAT = parse(Float64, get(ENV, "JM_AUTODRIVE_LAT", "0"))
const AUTODRIVE_ROAD = get(ENV, "JM_AUTODRIVE_ROAD", "1") != "0"
const AD_OFF = Ref{Union{Nothing,Tuple{Float64,Vector{Float64}}}}(nothing)
const ADLINE = Ref{Any}(nothing)

const AUTODRIVE_RECOVER = parse(Float64, get(ENV, "JM_AUTODRIVE_RECOVER", "0"))
const AD_SLOW_T = Ref(0.0)

const AUTODRIVE_TRAFFIC = get(ENV, "JM_AUTODRIVE_TRAFFIC", "0") != "0"
const AD_TRAFFIC = Ref(NTuple{3,Float64}[])
const AUTODRIVE_PROF_PACE = parse(Float64, get(ENV, "JM_AUTODRIVE_PROFILE", "0"))
const AUTODRIVE_PROF = Ref{Any}(nothing)
const GROUND_LAT_SMOOTH = get(ENV, "JM_GROUND_LAT_SMOOTH", "0") != "0"
const AUTODRIVE_DIAG = parse(Int, get(ENV, "JM_AUTODRIVE_DIAG", "0"))

const AUTODRIVE_MANUAL = get(ENV, "JM_AUTODRIVE_MANUAL", "0") != "0"
const ADM = Ref((hold = 0, up = false, dn = false))

const SKIDRAMP  = max(1e-3, parse(Float64, get(ENV, "JM_SKIDPAD_RAMP", "4.0")))
const SKIDDRIVE = let s = get(ENV, "JM_SKIDPAD_DRIVE", "")
    if isempty(s)
        nothing
    else
        segs = NamedTuple{(:steer, :throttle, :dur), Tuple{Float64, Float64, Float64}}[]
        for part in split(s, ';', keepempty = false)
            f = parse.(Float64, strip.(split(part, ',')))
            length(f) >= 2 || error("JM_SKIDPAD_DRIVE segment needs \"<steer>,<throttle>[,<secs>]\", got \"$part\"")
            push!(segs, (steer = clamp(f[1], -1.0, 1.0), throttle = clamp(f[2], 0.0, 1.0),
                         dur = length(f) >= 3 ? max(f[3], 0.0) : Inf))
        end
        segs
    end
end
const AI_LAPDIAG = get(ENV, "JM_AI_LAPDIAG", "0") != "0"

const NETPOSES = Ref(Tuple{Int,NTuple{6,Float64}}[])

const NETVEL = Ref(Dict{Int,NTuple{2,Float64}}())
const NET_HITS = Ref(0)
const NET_DMIN_AT = Ref{Any}(nothing)
const NET_PENNOW = Ref(0.0)
const NET_PENMAX = Ref(0.0)
const NET_DMIN = Ref(Inf)
const NET_COLLIDE = get(ENV, "JM_NET_COLLIDE", "1") != "0"

const NET_ERR  = get(ENV, "JM_NET_ERR", "0") != "0"
const NET_PREV = Ref(Dict{UInt8,NamedTuple}())
const NET_ERRS = Float64[]
const NET_DTS  = Float64[]

include(joinpath(@__DIR__,"netplay.jl")); using .NetPlay

include(joinpath(@__DIR__,"serious_games_week.jl")); using .SeriousGamesWeek
const NETLINK = if NETMODE == "host"
        println("  net:     HOSTING on udp/", NET_PORT, " as car ", NET_ID); flush(stdout)
        l = NetPlay.netopen(port = NET_PORT)
        matchmaker_announce(NET_PORT, get(ENV, "JM_NET_TITLE", "Julia Racer at " * uppercasefirst(TRACKSEL));
                        name = get(ENV, "SGW_NAME", get(ENV, "USER", "")), max_players = Int(AI_ID0) - 1)
        l
    elseif NETMODE == "join"
        jhost, jport = NET_HOST, NET_PORT
        if get(ENV, "JM_NET_HOST", "matchmaker") == "matchmaker"
            races = matchmaker_races()
            if isempty(races)
                jhost = "127.0.0.1"
                println("  matchmaker:  ", matchmaker_configured() ? "no races listed" : "no matchmaker configured",
                        "; joining ", jhost, ":", jport)
            else
                jhost, jport = races[1][1], races[1][2]
                println("  matchmaker:  joining \"", races[1][4], "\" (", length(races), " race(s) listed)")
            end
        end
        println("  net:     JOINING ", jhost, ":", jport, " as car ", NET_ID); flush(stdout)
        NetPlay.netopen(port = jport + NET_ID, peer = (jhost, jport))
    else
        nothing
    end

include(joinpath(@__DIR__,"setup_tab.jl")); using .SetupTab
const SETUP = SetupTab.session_setup(DriveRT3D.KS[], DriveRT3D.RIDE_H[],
                                     DriveRT3D.FINAL[], DriveRT3D.MASS[])
let spec = get(ENV, "JM_SETUP", "")
    SetupTab.apply_spec!(SETUP, spec)

    if !haskey(ENV, "JM_SETUP") && !haskey(ENV, "JM_SMOKE") &&
       !haskey(ENV, "JM_NO_SETUP_MENU") && isa(stdin, Base.TTY)
        SetupTab.setup_menu!(SETUP)
    end
    if !SetupTab.is_default(SETUP)

        DriveRT3D.set_suspension!(SETUP.ks...; source = "player setup (from $(basename(IBTTMPL)))")
        DriveRT3D.set_ride_height!(SETUP.rh...; source = "player setup (from $(basename(IBTTMPL)))")
        DriveRT3D.set_mass!(SETUP.mass, DriveRT3D.FRONT_FRAC[]; source = "player setup")
        DriveRT3D.FINAL[] = SETUP.final
    end

    print("  setup:   ")
    println(SetupTab.is_default(SETUP) ? "session default (unmodified)" :
            "MODIFIED by the player -- the setup menu's R, or JM_SETUP=reset, restores the session")
    SetupTab.is_default(SETUP) || print(SetupTab.describe(SETUP))
end

const IBTREC = IBTREC0 && isfile(IBTTMPL)
if IBTREC0 && !IBTREC
    println("  (.ibt telemetry off: the reference template is not in this checkout — ",
            basename(IBTTMPL), ". The plain-text telemetry below is unaffected.)")
end

const SEG7 = Dict('0'=>(1,1,1,1,1,1,0),'1'=>(0,1,1,0,0,0,0),'2'=>(1,1,0,1,1,0,1),
                  '3'=>(1,1,1,1,0,0,1),'4'=>(0,1,1,0,0,1,1),'5'=>(1,0,1,1,0,1,1),
                  '6'=>(1,0,1,1,1,1,1),'7'=>(1,1,1,0,0,0,0),'8'=>(1,1,1,1,1,1,1),'9'=>(1,1,1,1,0,1,1))
function skidpad_parts()
    up = (0f0,1f0,0f0)
    push3!(v,x,y,z,c) = append!(v, Float32[x,y,z, up[1],up[2],up[3], c[1],c[2],c[3], 0f0,0f0])

    quad!(v,x0,z0,x1,z1,y,c) = (push3!(v,x0,y,z0,c);push3!(v,x1,y,z1,c);push3!(v,x1,y,z0,c);
                                push3!(v,x0,y,z0,c);push3!(v,x0,y,z1,c);push3!(v,x1,y,z1,c))

    function digit!(v, ch, x0, z0, s, th, c)
        S = get(SEG7, ch, (0,0,0,0,0,0,0)); W=0.62f0
        seg = Dict(1=>(th,W-th, 1-th,1f0), 2=>(W-th,W, 0.5f0,1-th), 3=>(W-th,W, th,0.5f0),
                   4=>(th,W-th, 0f0,th),   5=>(0f0,th, th,0.5f0),   6=>(0f0,th, 0.5f0,1-th),
                   7=>(th,W-th, 0.5f0-th/2,0.5f0+th/2))
        for k in 1:7
            S[k]==1 || continue; (u0,u1,v0,v1)=seg[k]
            quad!(v, x0+u0*s, z0+v0*s, x0+u1*s, z0+v1*s, 0.03f0, c)
        end
    end

    function label!(v, n::Int, cx, cz, s, c)
        ds = string(n); nd=length(ds); adv=0.78f0*s; total=(nd-1)*adv + 0.62f0*s
        x0 = cx - total/2
        for ch in ds; digit!(v, ch, x0, cz - s/2, s, 0.13f0, c); x0 += adv; end
    end

    FSEG = Dict(1=>(0.13f0,0.49f0,0.87f0,1f0), 2=>(0.49f0,0.62f0,0.5f0,0.87f0), 3=>(0.49f0,0.62f0,0.13f0,0.5f0),
                4=>(0.13f0,0.49f0,0f0,0.13f0), 5=>(0f0,0.13f0,0.13f0,0.5f0), 6=>(0f0,0.13f0,0.5f0,0.87f0),
                7=>(0.13f0,0.49f0,0.435f0,0.565f0))
    function flabel!(v, n::Int, cx, cz, s, c)
        r=sqrt(cx^2+cz^2); rx=cx/r; rz=cz/r; tx=-rz; tz=rx
        wpt(uu,vv) = (cx + uu*s*tx + vv*s*rx, cz + uu*s*tz + vv*s*rz)
        ds=string(n); adv=0.78f0; total=Float32((length(ds)-1)*adv+0.62f0); u0=-total/2
        for ch in ds
            S=get(SEG7,ch,(0,0,0,0,0,0,0))
            for k in 1:7
                S[k]==1 || continue; (a0,a1,b0,b1)=FSEG[k]
                (x00,z00)=wpt(u0+a0,b0); (x10,z10)=wpt(u0+a1,b0); (x11,z11)=wpt(u0+a1,b1); (x01,z01)=wpt(u0+a0,b1)
                push3!(v,x00,0.04f0,z00,c); push3!(v,x10,0.04f0,z10,c); push3!(v,x11,0.04f0,z11,c)
                push3!(v,x00,0.04f0,z00,c); push3!(v,x11,0.04f0,z11,c); push3!(v,x01,0.04f0,z01,c)
            end
            u0+=adv
        end
    end
    parts = Render.TrackPart[]

    g = Float32[]; asph=(0.62f0,0.63f0,0.64f0); S=160f0
    for (ax,az,bx,bz,cx,cz) in ((-S,-S, S,-S, S,S), (-S,-S, S,S, -S,S))
        push3!(g,ax,0f0,az,asph); push3!(g,cx,0f0,cz,asph); push3!(g,bx,0f0,bz,asph)
    end
    push!(parts, Render.TrackPart(g, "", asph))

    labels = Float32[]; lcol = (1f0,0.95f0,0.45f0)
    for d in 10:10:200
        r = Float32(d/2); ring=Float32[]; w=(d%50==0 ? 0.30f0 : 0.16f0); y=0.02f0
        col = d%50==0 ? (1f0,0.92f0,0.35f0) : (0.92f0,0.93f0,0.96f0)
        seg = max(72, round(Int, r*3.5))
        for i in 0:seg-1
            a0=2f0*Float32(pi)*i/seg; a1=2f0*Float32(pi)*(i+1)/seg
            xi0=(r-w)*cos(a0); zi0=(r-w)*sin(a0); xo0=(r+w)*cos(a0); zo0=(r+w)*sin(a0)
            xi1=(r-w)*cos(a1); zi1=(r-w)*sin(a1); xo1=(r+w)*cos(a1); zo1=(r+w)*sin(a1)
            push3!(ring,xi0,y,zi0,col); push3!(ring,xo1,y,zo1,col); push3!(ring,xo0,y,zo0,col)
            push3!(ring,xi0,y,zi0,col); push3!(ring,xi1,y,zi1,col); push3!(ring,xo1,y,zo1,col)
        end
        push!(parts, Render.TrackPart(ring, "", col))

        if d % 50 == 0
            ls = clamp(Float32(d)*0.06f0, 2.0f0, 5.0f0)
            flabel!(labels, d, 0f0,  r, ls, lcol); flabel!(labels, d, 0f0, -r, ls, lcol)
            flabel!(labels, d,  r, 0f0, ls, lcol); flabel!(labels, d, -r, 0f0, ls, lcol)
        end
    end
    push!(parts, Render.TrackPart(labels, "", lcol))

    cones = Float32[]; ocol = (0.95f0, 0.42f0, 0.10f0)
    function cone!(v, cx, cz, h, rad, c)
        n = 6
        for i in 0:n-1
            a0 = 2f0*Float32(pi)*i/n; a1 = 2f0*Float32(pi)*(i+1)/n
            push3!(v, cx+rad*cos(a0), 0f0, cz+rad*sin(a0), c)
            push3!(v, cx+rad*cos(a1), 0f0, cz+rad*sin(a1), c)
            push3!(v, cx, h, cz, c)
        end
    end
    for k in 0:5
        ang = Float32(k)*Float32(pi)/3; cone!(cones, 2.6f0*cos(ang), 2.6f0*sin(ang), 0.62f0, 0.22f0, ocol)
    end
    cone!(cones, 0f0, 0f0, 0.7f0, 0.26f0, ocol)
    push!(parts, Render.TrackPart(cones, "", ocol))

    parts
end

align_centreline(cl, hat) = CLASSICTrack.align_centreline(cl, hat)

function recentre_on_road(cl, hat; reach = 14.0, step = 0.5, frac = 1.0, cap = 4.0, passes = 1,
                          shift = nothing)
    n = length(cl)
    on(x, z) = JRTrack.hat3d(hat, x, z; ref = Inf)[3]
    for pass in 1:passes
        nrm(i) = (q = cl[mod(i, n)+1]; r = cl[mod(i-2, n)+1]; tx = q[1]-r[1]; ty = q[2]-r[2];
                  tl = hypot(tx, ty); tl < 1e-6 ? (0.0, 0.0) : (-ty/tl, tx/tl))
        raw = zeros(n)
        for i in 1:n
            p = cl[i]; (nx, ny) = nrm(i)
            (nx == 0.0 && ny == 0.0) && continue
            if on(p[1], p[2])
                hi = 0.0; while hi < reach && on(p[1]+nx*(hi+step), p[2]+ny*(hi+step)); hi += step; end
                lo = 0.0; while lo < reach && on(p[1]-nx*(lo+step), p[2]-ny*(lo+step)); lo += step; end
                raw[i] = clamp(frac*(hi - lo)/2, -cap, cap)
            else

                hit = 0.0
                for d in step:step:reach
                    if     on(p[1]+nx*d, p[2]+ny*d); hit =  d; break
                    elseif on(p[1]-nx*d, p[2]-ny*d); hit = -d; break
                    end
                end
                raw[i] = clamp(hit, -cap, cap)
            end
        end
        sm = zeros(n); W = 3
        for i in 1:n; a = 0.0; for d in -W:W; a += raw[mod(i-1+d, n)+1]; end; sm[i] = a/(2W+1); end

        if get(ENV, "JM_SHIFT_CLAMP", "0") == "1"
            sd = parse(Float64, get(ENV, "JM_SHIFT_SECONDDIFF", "0.05"))
            for _ in 1:8
                worst = 0.0
                for i in 1:n
                    mid = (sm[mod(i-2, n)+1] + sm[mod(i, n)+1]) / 2
                    d   = sm[i] - mid
                    if abs(d) > sd
                        worst = max(worst, abs(d) - sd)
                        sm[i] = mid + sign(d)*sd
                    end
                end
                worst < 1e-4 && break
            end
        end
        cl = [(p = cl[i]; (nx, ny) = nrm(i); (p[1]+nx*sm[i], p[2]+ny*sm[i])) for i in 1:n]
        shift === nothing || (shift .+= sm)
        println("centreline re-centred on the visible road (pass ", pass, "/", passes, "): max shift ",
                round(maximum(abs, sm), digits=1), " m, mean ", round(sum(abs, sm)/n, digits=2),
                " m, at-start ", round(sm[1], digits=2), " m")
        maximum(abs, sm) < 0.3 && break
    end
    cl
end

function dedup_scenery(tris)
    isempty(tris) && return tris
    seen = Set{NTuple{4,Int}}(); keep = eltype(tris)[]
    for i in sort(collect(eachindex(tris)); by = j -> isempty(tris[j].tex) ? 1 : 0)
        tr = tris[i]; a,b,c = tr.p[1], tr.p[2], tr.p[3]
        cx=(a[1]+b[1]+c[1])/3; cy=(a[2]+b[2]+c[2])/3; cz=(a[3]+b[3]+c[3])/3
        ux=b[1]-a[1];uy=b[2]-a[2];uz=b[3]-a[3]; vx=c[1]-a[1];vy=c[2]-a[2];vz=c[3]-a[3]
        ar=0.5*sqrt((uy*vz-uz*vy)^2+(uz*vx-ux*vz)^2+(ux*vy-uy*vx)^2)
        key=(round(Int,cx/0.08),round(Int,cy/0.08),round(Int,cz/0.08),round(Int,ar/0.05))
        key in seen && continue
        push!(seen,key); push!(keep,tr)
    end
    keep
end

function veil_opaque_span(w, uv, tex, ribbon; lat=5.5, hlo=0.3, hhi=3.0, n=16)
    lo = Inf; hi = -Inf
    for i in 0:n, j in 0:(n-i)
        a = i/n; b = j/n; c = 1 - a - b
        x = a*w[1][1] + b*w[2][1] + c*w[3][1]; y = a*w[1][2] + b*w[2][2] + c*w[3][2]
        z = a*w[1][3] + b*w[2][3] + c*w[3][3]
        hr = JRTrack.hat(ribbon, x, y); hr.found || continue
        (abs(hr.lateral) < lat && hlo <= z - hr.height <= hhi) || continue
        if tex !== nothing
            tw, th, rgba = tex
            u = a*uv[1][1] + b*uv[2][1] + c*uv[3][1]; v = a*uv[1][2] + b*uv[2][2] + c*uv[3][2]
            px = mod(floor(Int, u*tw), tw); py = mod(floor(Int, v*th), th)
            rgba[4*(py*tw + px) + 4] < 0x0a && continue
        end
        lo = min(lo, hr.lateral); hi = max(hi, hr.lateral)
    end
    lo <= hi ? (lo, hi) : nothing
end

const BRIDGE_TEX = ("bridge", "br_under")

const SCEN_PRSTAT = Ref(0)

const VEILX = String[lowercase(x) for x in split(get(ENV, "JM_VEILX", "bkbill"), ",") if !isempty(x)]
function classic_scenery(ztrk, datpack, ribbon)
    pls = Render.CLASSIC3DO.classic_placements(ztrk)

    let fl = Set(lowercase.(filter(!isempty, split(get(ENV, "JM_YAWFLIP", ""), ","))))
        isempty(fl) || (pls = [(lowercase(String(nm)) in fl ? (nm, (t[1], t[2], t[3], t[4] + pi, t[5], t[6], t[7])) : (nm, t)) for (nm, t) in pls])
    end

    _veilhide = get(ENV, "JM_CLASSICVIS_VEIL", "1") != "0" ? ["half01","half02","half03","half04","half05","half06","half07","half08","half09","half1s",
                                                          "adena-r2","bergw-r1","breid-ra","wehr-r1b", VEILX...] : String[]
    let hd = Set(vcat(lowercase.(filter(!isempty, split(get(ENV, "JM_PLACE_HIDE", ""), ","))), _veilhide))
        if !isempty(hd)
            for (nm, t) in pls
                (lowercase(String(nm)) in hd && !(lowercase(String(nm)) in _veilhide)) && println("  [place_hide] ", nm, " ", map(x -> round(Float64(x), digits = 3), t))
            end
            pls = [(nm, t) for (nm, t) in pls if !(lowercase(String(nm)) in hd)]
        end
    end

    scen_yaw = deg2rad(parse(Float64, get(ENV, "JM_SCENERY_YAW_ADD", "0")))
    SCEN_EULER_INT = get(ENV, "JM_SCEN_EULER", "ext") == "int"
    SCEN_PR = parse.(Float64, split(get(ENV, "JM_SCEN_PR", "1,1"), ","))
    function placemat(t)
        d=(t[1],t[2],t[3]); m=(t[4]+scen_yaw,t[5],t[6]); s = t[7] <= 0 ? 1.0 : t[7]

        m = (m[1], SCEN_PR[1]*m[2], SCEN_PR[2]*m[3])
        ca,sa=cos(m[1]),sin(m[1]); cb,sb=cos(m[2]),sin(m[2]); cc,sc=cos(m[3]),sin(m[3])
        Ryaw=[ca -sa 0; sa ca 0; 0 0 1.0]; Rpit=[cb 0 sb; 0 1.0 0; -sb 0 cb]; Rrol=[1.0 0 0; 0 cc -sc; 0 sc cc]
        R=(SCEN_EULER_INT ? Ryaw*Rpit*Rrol : Rrol*Rpit*Ryaw).*s
        SCEN_PRSTAT[] += (abs(m[2]) > 1e-4 || abs(m[3]) > 1e-4)
        [R[1,1] R[1,2] R[1,3] d[1]; R[2,1] R[2,2] R[2,3] d[2]; R[3,1] R[3,2] R[3,3] d[3]; 0 0 0 1.0]
    end
    cache=Dict{String,Any}(); tmp=tempdir()
    getmesh(nm)=get!(cache, lowercase(nm)) do
        v=get(datpack, lowercase(nm*".3do"), nothing); v===nothing && return nothing
        tp=joinpath(tmp,"jm_nb_"*lowercase(nm)*".3do"); isfile(tp)||write(tp,v)

        m = try
                Render.CLASSIC3DO.parse_3do(tp)
            catch e
                if get(ENV,"JM_MESHERR","") != ""
                    println("   [mesherr] ", rpad(nm,16), " bytes=", rpad(length(v),9),
                            typeof(e), " :: ", first(split(sprint(showerror, e), "\n")))
                    flush(stdout)
                end
                nothing
            end
        if m === nothing
            nothing
        else
            d = dedup_scenery(m.tris)
            if (d === nothing || isempty(d)) && get(ENV,"JM_MESHERR","") != ""
                println("   [mesherr] ", rpad(nm,16), " parsed OK but EMPTY after dedup (",
                        length(m.tris), " raw tris)"); flush(stdout)
            end
            d
        end
    end

    spritecache=Dict{String,Bool}()
    issprite(nm,mesh)=get!(spritecache,nm) do
        lo3=Inf;hi3=-Inf;loh=Inf;hih=-Inf
        for tr in mesh, p in tr.p
            lo3=min(lo3,p[3]);hi3=max(hi3,p[3]);loh=min(loh,p[1],p[2]);hih=max(hih,p[1],p[2])
        end
        (hi3-lo3) < 0.5 && (hih-loh) < 6.0
    end
    hat=Render.CLASSIC3DO.Tri[]; groups=Dict{String,Vector{Float32}}(); nskip=0
    ndrop_edge = Ref(0); ndrop_road = Ref(0); nkeep_t = Ref(0)
    scene_at = Tuple{String,Float64,Float64,Int,Float64,Float64}[]
    scene_drop = Tuple{String,Float64,Float64,String}[]
    scene_tridrop = Dict{Tuple{String,String},Int}()
    snote_at = get(ENV, "JM_SCENE_AT", ""); snote_win = parse(Float64, get(ENV, "JM_SCENE_WIN", "250"))

    scene_note(nm, rule, h) = (snote_at != "" && h.found && abs(h.lapdist - parse(Float64, snote_at)) < snote_win) &&
        (scene_tridrop[(nm, rule)] = get(scene_tridrop, (nm, rule), 0) + 1)

    local n_offered=0; local n_treesrb=0; local n_nomesh=0; local n_kept=0
    sprites = NamedTuple[]
    n_people_dropped = Ref(0)
    n_veil = Ref(0)
    VEIL_DROP = get(ENV, "JM_VEIL_KEEP", "0") == "0"
    VEIL_EXTENT = get(ENV, "JM_VEIL_EXTENT", "0") != "0"
    n_veil_clear = Ref(0)
    veil_tex = Ref{Any}(nothing); veil_rgba = Dict{String,Any}()
    scene_names=Dict{String,Int}()
    scene_excl = Set(lowercase.(split(get(ENV, "JM_SCENE_EXCLUDE", ""), ",", keepempty = false)))
    for (nm,t) in pls
        n_offered += 1
        scene_names[nm] = get(scene_names,nm,0) + 1
        lowercase(nm) in scene_excl && continue
        if startswith(nm,"treesrb"); n_treesrb += 1; continue; end

        if PeopleFilter.is_loose_person(lowercase(nm)) && get(ENV, "JM_KEEP_CROWDROWS", "0") == "0"
            n_people_dropped[] += 1; continue
        end
        mesh=getmesh(nm)
        if (mesh===nothing || isempty(mesh))
            n_nomesh += 1

            tp = joinpath(tmp, "jm_nb_"*lowercase(nm)*".3do")
            if isfile(tp)
                try
                    hh, ww, strs, aax = Render.billboard_stub(tp)
                    M = placemat(t)
                    sc = t[7] <= 0 ? 1.0 : t[7]

                    if PeopleFilter.is_loose_person(lowercase(nm)) && get(ENV, "JM_KEEP_CROWDROWS", "0") == "0"
                        n_people_dropped[] += 1; continue
                    end
                    push!(sprites, (name=nm, x=Float32(M[1,4]), y=Float32(M[2,4]), z=Float32(M[3,4]),
                                    h=Float32(hh*sc), w=Float32(ww*sc), texs=strs,
                                    yaw=Float32(t[4]), aax=Float32(aax)))
                catch
                end
            end

            if get(ENV,"JM_SCENE_AT","") != ""
                let M2 = placemat(t), hr2 = JRTrack.hat(ribbon, Float64(M2[1,4]), Float64(M2[2,4]))
                    if hr2.found && abs(hr2.lapdist - parse(Float64, ENV["JM_SCENE_AT"])) < parse(Float64, get(ENV,"JM_SCENE_WIN","250"))
                        push!(scene_drop, (nm, hr2.lapdist, hr2.lateral, isfile(tp) ? "no-mesh -> billboard" : "no-mesh, UNRESOLVED (vanishes)"))
                    end
                end
            end
            continue
        end
        n_kept += 1
        if issprite(nm,mesh)
            nskip+=1

            if get(ENV,"JM_SPRITESKIP","") != ""
                lo3=Inf;hi3=-Inf;loh=Inf;hih=-Inf; cx=0.0; cy=0.0; np=0
                for tr in mesh, q in tr.p
                    lo3=min(lo3,q[3]);hi3=max(hi3,q[3]);loh=min(loh,q[1],q[2]);hih=max(hih,q[1],q[2])
                end
                M=placemat(t)
                px = M[1,4]; py = M[2,4]
                hr = JRTrack.hat(ribbon, Float32(px), Float32(py))

                bbs = "no .3do"
                tpq = joinpath(tmp, "jm_nb_"*lowercase(nm)*".3do")
                if isfile(tpq)
                    try
                        bh, bw, bstrs, _ = Render.billboard_stub(tpq)
                        bbs = string("bb h=", round(bh,digits=2), " w=", round(bw,digits=2),
                                     " tex=", isempty(bstrs) ? "NONE" : join(bstrs,","))
                    catch e
                        bbs = "billboard_stub THREW: " * string(e)
                    end
                end
                println("   [spriteskip] ", rpad(nm,16),
                        "h=", rpad(round(hi3-lo3,digits=2),7),
                        "w=", rpad(round(hih-loh,digits=2),7),
                        hr.found ? string("lapdist=", rpad(round(hr.lapdist,digits=0),9),
                                          "lat=", rpad(round(hr.lateral,digits=1),7)) : "off-ribbon  ",
                        bbs)
            end
            continue
        end
        M=placemat(t)

        if get(ENV,"JM_SCENE_AT","") != ""
            _hr = JRTrack.hat(ribbon, Float64(M[1,4]), Float64(M[2,4]))
            if _hr.found && abs(_hr.lapdist - parse(Float64, ENV["JM_SCENE_AT"])) < parse(Float64, get(ENV,"JM_SCENE_WIN","250"))

                let zlo = Inf, zhi = -Inf
                    for tr in mesh, q in tr.p
                        wz = Float64(M[3,1]*q[1]+M[3,2]*q[2]+M[3,3]*q[3]+M[3,4]); zlo = min(zlo, wz); zhi = max(zhi, wz)
                    end
                    push!(scene_at, (nm, _hr.lapdist, _hr.lateral, length(mesh),
                                     round(zlo - _hr.height, digits=1), round(zhi - _hr.height, digits=1)))
                end
            end
        end
        ap(q)=(Float32(M[1,1]*q[1]+M[1,2]*q[2]+M[1,3]*q[3]+M[1,4]),
               Float32(M[2,1]*q[1]+M[2,2]*q[2]+M[2,3]*q[3]+M[2,4]),
               Float32(M[3,1]*q[1]+M[3,2]*q[2]+M[3,3]*q[3]+M[3,4]))
        rn(n)=(Float32(M[1,1]*n[1]+M[1,2]*n[2]+M[1,3]*n[3]),
               Float32(M[2,1]*n[1]+M[2,2]*n[2]+M[2,3]*n[3]),
               Float32(M[3,1]*n[1]+M[3,2]*n[2]+M[3,3]*n[3]))
        if get(ENV,"JM_SCENEDROP","") != "" && occursin(lowercase(ENV["JM_SCENEDROP"]), lowercase(nm))
            M0 = placemat(t)
            hr0 = JRTrack.hat(ribbon, Float64(M0[1,4]), Float64(M0[2,4]))
            println("   [scenedrop] ", nm, ": mesh has ", length(mesh), " tris; placed at ",
                    hr0.found ? string("lapdist ", round(Int,hr0.lapdist), " lat ", round(hr0.lateral,digits=1)) : "OFF-RIBBON",
                    "  origin z=", round(M0[3,4],digits=1))
        end

        local ocx=0f0; local ocy=0f0; local ocz=0f0; local objext=1f0
        begin
            xs=Float32[]; ys=Float32[]; zs=Float32[]
            for tr in mesh, q in tr.p
                pw = ap(q); push!(xs,pw[1]); push!(ys,pw[2]); push!(zs,pw[3])
            end
            if !isempty(xs)
                sort!(xs); sort!(ys); sort!(zs)
                md(v) = v[cld(length(v),2)]
                ocx, ocy, ocz = md(xs), md(ys), md(zs)
                q95(v) = v[max(1,min(length(v), ceil(Int,0.95*length(v))))]
                q05(v) = v[max(1,min(length(v), ceil(Int,0.05*length(v))))]
                objext = max(q95(xs)-q05(xs), q95(ys)-q05(ys), q95(zs)-q05(zs), 1f0) / 2
            end
        end
        for tr in mesh
            w=(ap(tr.p[1]),ap(tr.p[2]),ap(tr.p[3])); nn=(rn(tr.n[1]),rn(tr.n[2]),rn(tr.n[3]))

            e1=hypot(w[2][1]-w[1][1],w[2][2]-w[1][2],w[2][3]-w[1][3])
            e2=hypot(w[3][1]-w[2][1],w[3][2]-w[2][2],w[3][3]-w[2][3])
            e3=hypot(w[1][1]-w[3][1],w[1][2]-w[3][2],w[1][3]-w[3][3])
            emax=max(e1,e2,e3); emin=min(e1,e2,e3)

            _junk = if get(ENV,"JM_EDGE_ABS","0") != "0"
                (emax > 150f0 || (emax > 70f0 && emax > 10f0*emin))
            else

                far = max(abs(w[1][1]-ocx), abs(w[2][1]-ocx), abs(w[3][1]-ocx),
                          abs(w[1][2]-ocy), abs(w[2][2]-ocy), abs(w[3][2]-ocy),
                          abs(w[1][3]-ocz), abs(w[2][3]-ocz), abs(w[3][3]-ocz))
                (far > 3f0*objext + 20f0) || emax > 400f0
            end
            if _junk
                if get(ENV,"JM_SCENEDROP","") != "" && occursin(lowercase(ENV["JM_SCENEDROP"]), lowercase(nm))
                    ndrop_edge[] = ndrop_edge[] + 1
                end
                if get(ENV,"JM_SCENE_AT","") != "" && (let cgx=(w[1][1]+w[2][1]+w[3][1])/3; cgy=(w[1][2]+w[2][2]+w[3][2])/3; cgz=(w[1][3]+w[2][3]+w[3][3])/3; hrE = JRTrack.hat(ribbon, cgx, cgy); hrE.found && abs(hrE.lapdist - parse(Float64, ENV["JM_SCENE_AT"])) < parse(Float64, get(ENV,"JM_SCENE_WIN","250")) end)
                    scene_tridrop[(nm * "/" * tr.tex, "stretched-edge")] = get(scene_tridrop, (nm * "/" * tr.tex, "stretched-edge"), 0) + 1
                end
                continue
            end

            if get(ENV, "JM_FACEDUMP", "") != "" && lowercase(nm) == lowercase(ENV["JM_FACEDUMP"])
                let hv = [JRTrack.hat(ribbon, w[k][1], w[k][2]) for k in 1:3]
                    if all(h -> h.found, hv)
                        ux=w[2][1]-w[1][1]; uy=w[2][2]-w[1][2]; uz=w[2][3]-w[1][3]; vx=w[3][1]-w[1][1]; vy=w[3][2]-w[1][2]; vz=w[3][3]-w[1][3]
                        nzz=ux*vy-uy*vx; nll=sqrt((uy*vz-uz*vy)^2+(uz*vx-ux*vz)^2+nzz^2)
                        println("   [facedump] ", nm, " ", rpad(tr.tex, 12), " s ", round(Int, minimum(h.lapdist for h in hv)), "..",
                                round(Int, maximum(h.lapdist for h in hv)), " lat ", round(minimum(h.lateral for h in hv), digits=1), "..",
                                round(maximum(h.lateral for h in hv), digits=1), " z-road ",
                                round(minimum(w[k][3] - hv[k].height for k in 1:3), digits=1), "..",
                                round(maximum(w[k][3] - hv[k].height for k in 1:3), digits=1), " nup ", round(nzz/max(nll,1f-9), digits=2),
                                " col ", round.(tr.col, digits=2))
                    end
                end
            end

            cgx=(w[1][1]+w[2][1]+w[3][1])/3; cgy=(w[1][2]+w[2][2]+w[3][2])/3; cgz=(w[1][3]+w[2][3]+w[3][3])/3
            hr = JRTrack.hat(ribbon, cgx, cgy)

            if (hr.found && abs(hr.lateral) < 5.0 && abs(cgz - hr.height) < 3.0) && !(lowercase(tr.tex) in BRIDGE_TEX)
                if get(ENV,"JM_SCENEDROP","") != "" && occursin(lowercase(ENV["JM_SCENEDROP"]), lowercase(nm))
                    ndrop_road[] = ndrop_road[] + 1
                end
                if get(ENV,"JM_SCENE_AT","") != "" && abs(hr.lapdist - parse(Float64, ENV["JM_SCENE_AT"])) < parse(Float64, get(ENV,"JM_SCENE_WIN","250"))
                    scene_tridrop[(nm * "/" * tr.tex, "road-corridor")] = get(scene_tridrop, (nm * "/" * tr.tex, "road-corridor"), 0) + 1
                end
                continue
            end

            ux=w[2][1]-w[1][1]; uy=w[2][2]-w[1][2]; uz=w[2][3]-w[1][3]
            vx=w[3][1]-w[1][1]; vy=w[3][2]-w[1][2]; vz=w[3][3]-w[1][3]
            nz=ux*vy-uy*vx; nl=sqrt((uy*vz-uz*vy)^2+(uz*vx-ux*vz)^2+nz^2)
            (lowercase(tr.tex) in RINGBANNER_TEX && ringbanner_zone(hr.found, hr.lapdist, hr.lateral)) && continue
            (nl > 1f-6 && abs(nz)/nl > 0.4f0) && push!(hat, Render.CLASSIC3DO.Tri(w, nn, tr.uv, tr.tex, tr.col, tr.flat, tr.ptype))

            if VEIL_DROP && nl > 1f-6 && abs(nz)/nl <= 0.4f0 && hr.found
                hv = (JRTrack.hat(ribbon, w[1][1], w[1][2]), JRTrack.hat(ribbon, w[2][1], w[2][2]),
                      JRTrack.hat(ribbon, w[3][1], w[3][2]))
                if all(h -> h.found, hv)
                    llo = minimum(h.lateral for h in hv); lhi = maximum(h.lateral for h in hv)
                    zlo = min(w[1][3], w[2][3], w[3][3]) - hr.height
                    zhi = max(w[1][3], w[2][3], w[3][3]) - hr.height
                    if lhi > -5.5 && llo < 5.5 && zlo < 3.0 && zhi > 1.5

                        if !VEIL_EXTENT
                            veil_tex[] === nothing && (veil_tex[] = Render.classic_texture_index(dirname(ztrk)))
                            tx = get!(() -> Render.tex_rgba(veil_tex[], tr.tex), veil_rgba, tr.tex)

                            sp = veil_opaque_span(w, tr.uv, tx, ribbon)
                            clear = sp === nothing || !(sp[1] < 0.0 < sp[2])
                            clear && (n_veil_clear[] += 1)
                        else
                            clear = false
                        end
                        if !clear
                            n_veil[] += 1
                            scene_note(nm * "/" * tr.tex, "veil", hr)
                            continue
                        end
                    end
                end
            end

            if nl > 1f-6 && nz/nl < -0.5f0
                hrz = JRTrack.hat(ribbon, cgx, cgy)
                if hrz.found && 5.0 < abs(hrz.lateral) < 30.0 && cgz > hrz.height + 3.0
                    scene_note(nm * "/" * tr.tex, "underside", hrz)
                    continue
                end
            end
            v=get!(groups, tr.tex, Float32[])
            for i in 1:3
                q=w[i]; n=nn[i]; uv=tr.uv[i]
                append!(v, Float32[q[1],q[3],-q[2], n[1],n[3],-n[2], tr.col[1],tr.col[2],tr.col[3], uv[1],uv[2]])
            end
        end
    end
    nskip > 0 && print("(skipped ", nskip, " flat sprite stubs) ")
    n_veil[] > 0 && print("(dropped ", n_veil[], " road-spanning veil faces, E109-S5/S6; kept ", n_veil_clear[], " with a transparent road part) ")
    if get(ENV,"JM_SCENEDIAG","") != ""
        println()
        println("== JM_SCENEDIAG scenery placements offered to the loader ==")
        println("   offered              ", n_offered)
        println("   dropped: treesrb*    ", n_treesrb)
        println("   dropped: NO MESH     ", n_nomesh, "   <-- placement exists but its object could not be loaded")
        println("   dropped: LOOSE PEOPLE ", n_people_dropped[], "   <-- TRACKGOLD-1 S3: lines of people / marshals as sprites (JM_KEEP_CROWDROWS=1 keeps)")
        println("   dropped: sprite stub ", nskip)
        println("   reached the renderer ", n_kept - nskip)
        println("   distinct object names: ", length(scene_names))

        let keys_ci = Dict(lowercase(k) => k for k in keys(datpack))
            nlower = count(k -> k == lowercase(k), keys(datpack))
            println("   -- archive keys: ", length(datpack), " total, ", nlower, " already lowercase, ",
                    length(datpack)-nlower, " mixed/upper --")
            found_ci = 0; total_missing = 0
            for (nm,c) in scene_names
                (getmesh(nm)===nothing || isempty(getmesh(nm))) || continue
                total_missing += c
                haskey(keys_ci, lowercase(nm*".3do")) && (found_ci += c)
            end
            println("   -- of ", total_missing, " failed placements, ", found_ci,
                    " WOULD resolve with a case-insensitive archive lookup --")
        end

        if get(ENV,"JM_SCENEFIND","") != ""
            pat = Regex(get(ENV,"JM_SCENEFIND",""), "i")
            hits = sort([(nm,c) for (nm,c) in scene_names if occursin(pat, nm)], by=x->-x[2])
            println("   -- PLACED names matching /", get(ENV,"JM_SCENEFIND",""), "/i: ", length(hits), " --")
            for (nm,c) in hits[1:min(end,20)]
                mm = getmesh(nm)
                st = mm === nothing ? "NO MESH" : (isempty(mm) ? "EMPTY (billboard)" : string(length(mm), " tris"))
                println("      ", rpad(nm,16), rpad(c,6), "placements   ", st)
            end
            isempty(hits) && println("      (none placed)")
        end
        miss = sort([(nm,c) for (nm,c) in scene_names if getmesh(nm)===nothing || isempty(getmesh(nm))], by=x->-x[2])
        if !isempty(miss)
            println("   -- most-placed names with NO MESH (top 15) --")
            for (nm,c) in miss[1:min(end,15)]; println("      ", rpad(nm,18), c, " placements"); end
        end
        flush(stdout)
    end
    if get(ENV,"JM_SCENE_AT","") != ""
        sort!(scene_at, by=x->abs(x[3]))
        println("== JM_SCENE_AT ", ENV["JM_SCENE_AT"], " ±250 m: ", length(scene_at), " scenery objects rendered ==")
        println("   name            lapdist   lateral   tris")
        for r in scene_at[1:(haskey(ENV,"JM_SCENE_WIN") ? end : min(end,22))]
            println("   ", rpad(r[1],15), rpad(round(Int,r[2]),10), rpad(round(r[3],digits=1),10), rpad(r[4],7),
                    "  z-road: ", r[5], "..", r[6], " m")
        end
        sort!(scene_drop, by=x->abs(x[3]))
        println("== JM_SCENE_AT: ", length(scene_drop), " placements in the window that did NOT reach the renderer ==")
        println("   name            lapdist   lateral   why")
        for r in scene_drop[1:(haskey(ENV,"JM_SCENE_WIN") ? end : min(end,40))]
            println("   ", rpad(r[1],15), rpad(round(Int,r[2]),10), rpad(round(r[3],digits=1),10), r[4])
        end

        let want = parse(Float64, ENV["JM_SCENE_AT"]), win = parse(Float64, get(ENV,"JM_SCENE_WIN","250")), rows = Any[]
            for sp in sprites
                hr = JRTrack.hat(ribbon, Float64(sp.x), Float64(sp.y))
                (hr.found && abs(hr.lapdist - want) < win && abs(hr.lateral) < 60.0) || continue
                push!(rows, (sp.name, round(hr.lapdist), round(hr.lateral, digits=1), round(sp.h, digits=1), round(sp.w, digits=1), round(Float64(sp.z) - hr.height, digits=1), sp.texs))
            end
            sort!(rows, by = r -> -r[4])
            println("== JM_SCENE_AT billboards in the window: ", length(rows), " (tallest first) ==")
            println("   name            lapdist  lateral  h      w      z-road  tex")
            for r in rows[1:min(end, 30)]
                println("   ", rpad(r[1], 15), rpad(r[2], 9), rpad(r[3], 9), rpad(r[4], 7), rpad(r[5], 7), rpad(r[6], 8), r[7])
            end
        end

        let want = parse(Float64, ENV["JM_SCENE_AT"]), win = parse(Float64, get(ENV,"JM_SCENE_WIN","250"))

            rendered = Any[]
            for (tex, v) in groups
                for k in 1:33:length(v)-32
                    pts = ntuple(i -> (Float32(v[k+11*(i-1)]), Float32(-v[k+11*(i-1)+2]), Float32(v[k+11*(i-1)+1])), 3)
                    uvs = ntuple(i -> (Float32(v[k+11*(i-1)+9]), Float32(v[k+11*(i-1)+10])), 3)
                    push!(rendered, (p = pts, tex = tex, uv = uvs))
                end
            end
            for (label, tris) in (("RENDERED scenery groups", rendered), ("terrain-candidate scenery", hat), ("track .3do", TRACKMESH0.tris))
                acc = Dict{String,Vector{Float64}}(); lats = Dict{String,Vector{Float64}}(); uvr = Dict{String,Vector{Float64}}()
                for t in tris
                    cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
                    cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
                    cz = (Float64(t.p[1][3])+Float64(t.p[2][3])+Float64(t.p[3][3]))/3
                    hr = JRTrack.hat(ribbon, cx, cy)
                    (hr.found && abs(hr.lapdist - want) < win && abs(hr.lateral) < 40.0) || continue
                    lt = lowercase(t.tex); lt = lt == "" ? "<none>" : lt
                    push!(get!(acc, lt, Float64[]), cz); push!(get!(lats, lt, Float64[]), hr.lateral)
                    if hasproperty(t, :uv)
                        for q in t.uv; push!(get!(uvr, lt, Float64[]), Float64(q[1]), Float64(q[2])); end
                        if lt in split(lowercase(get(ENV, "JM_SCENE_TEX", "\0")), ",")
                            println("      [", lt, "] ", join([string("(", round(Float64(t.p[i][1]), digits=1), ",", round(Float64(t.p[i][2]), digits=1), ",", round(Float64(t.p[i][3]), digits=1), ") lat=", (hv = JRTrack.hat(ribbon, Float64(t.p[i][1]), Float64(t.p[i][2])); hv.found ? round(hv.lateral, digits=1) : NaN), " uv=(", round(Float64(t.uv[i][1]), digits=2), ",", round(Float64(t.uv[i][2]), digits=2), ")") for i in 1:3], "  "))
                        end
                    end
                end
                println("== JM_SCENE_AT mesh in the window, ", label, ": ", length(acc), " textures ==")
                println("   texture          tris   z_min    z_mean   z_max    lat range")
                for (lt, zs) in sort(collect(acc), by = x -> -length(x[2]))[1:min(end, 24)]
                    ls = lats[lt]
                    println("   ", rpad(lt, 16), rpad(length(zs), 7), rpad(round(minimum(zs), digits=1), 9),
                            rpad(round(sum(zs)/length(zs), digits=1), 9), rpad(round(maximum(zs), digits=1), 9),
                            string(round(minimum(ls), digits=1), "..", round(maximum(ls), digits=1)),
                            haskey(uvr, lt) ? string("   uv u ", round(minimum(uvr[lt][1:2:end]), digits=2), "..", round(maximum(uvr[lt][1:2:end]), digits=2),
                                                     " v ", round(minimum(uvr[lt][2:2:end]), digits=2), "..", round(maximum(uvr[lt][2:2:end]), digits=2)) : "")
                end
            end
        end
        println("== JM_SCENE_AT: triangles dropped by the per-triangle rules inside the window, per object ==")
        for (k, v) in sort(collect(scene_tridrop), by = x -> -x[2])[1:min(end, 30)]
            println("   ", rpad(k[1], 15), rpad(k[2], 16), v)
        end
        flush(stdout)
    end
    if get(ENV,"JM_SCENEDROP","") != ""
        println("   [scenedrop] ", ENV["JM_SCENEDROP"], ": dropped ", ndrop_edge[],
                " by the stretched-edge rule, ", ndrop_road[], " by the road-corridor rule")
        flush(stdout)
    end
    (hat, [Render.TrackPart(v, tex, (0.5f0,0.5f0,0.5f0)) for (tex,v) in groups], sprites)
end

const MODEL = nothing

const CLASSICBASE = joinpath(classic_root(), "tracks")

const CLASSICNAME = get(Dict("nurburgring"=>"nurburg", "zandvoort"=>"zandvort",
                         "watglen"=>"watglen", "monza"=>"monza", "spa"=>"spa67"),
                    TRACKSEL, "zandvort")
const ZD   = joinpath(CLASSICBASE, CLASSICNAME)

find_ci(dir, name) = (isdir(dir) || return joinpath(dir, name);
                      m = filter(f -> lowercase(f) == lowercase(name), readdir(dir));
                      isempty(m) ? joinpath(dir, name) : joinpath(dir, m[1]))
const TRACKDAT = (p = find_ci(ZD, CLASSICNAME*".dat"); isfile(p) ? Render.CLASSICDat.parse_dat(p) : Dict{String,Vector{UInt8}}())
const TMPTRK = mktempdir()
"Path to a track file (`base`+`ext`, e.g. \".3do\"/\".trk\"): loose on disk if present, else extracted from the track .dat."
function track_file(base, ext)
    p = find_ci(ZD, base*ext); isfile(p) && return p
    v = get(TRACKDAT, lowercase(base*ext), nothing)
    v === nothing && return p
    q = joinpath(TMPTRK, base*ext); write(q, v); q
end
const ZTRK = track_file(CLASSICNAME, ".3do")

const _MAIN_LOWER = (get(ENV, "JM_PRELOWER", "1") != "0" && Threads.nthreads() >= 2) ? Threads.@spawn(begin
        _p = @__FILE__; _ex = Meta.parseall(read(_p, String); filename = _p)
        _i = findfirst(x -> x isa Expr && x.head == :const && x.args[1] isa Expr && x.args[1].args[1] === :_MAIN_Q, _ex.args)
        _q = _ex.args[_i].args[1].args[2]
        _f = _q.args[1].args[findfirst(x -> x isa Expr && x.head == :function, _q.args[1].args)]
        Meta.lower(Main, _f)
    end) : nothing

if SKIDPAD
    print("building skidpad... "); flush(stdout)
    const TRACK = skidpad_parts()
    println("flat pad + 20 measurement circles, diameters 10-200 m")

    const TRKALT = nothing
    const TRK_CAL = (on = false, sign = 1.0, off = 0.0, sscale = 1.0)
    const TRK_ROAD_LAT = 5.4
    const TRK_BLEND = 1.0
    const TRK_CORR_ON = false
    trk_road(s, lat) = 0.0
    trk_conf(s, lat) = 0.0
    ground_road(x, z) = (0.0, true)
else
    tstamp("track parse begins"); print("loading Classic ", CLASSICNAME, "… "); flush(stdout)
    const TRACKMESH0 = Render.CLASSIC3DO.parse_3do(ZTRK)

    const HAT_NEARBAND = get(ENV, "JM_HAT_NEARBAND", "1") != "0"
    const HAT_BANDDROP_N = Ref(0)
    hat_banddrop(t) = HAT_NEARBAND && !isempty(Render.CLASSIC3DO.BANDTAG) &&
        (tg = get(Render.CLASSIC3DO.BANDTAG, Render.CLASSIC3DO.bandkey(t.p...), Int32(0)); tg != 0 && (tg & 1) == 0) && (HAT_BANDDROP_N[] += 1; true)

    const TERRAIN0 = CLASSICTrack.build_hat(TRACKMESH0; exclude=HAT_EXCLUDE, exclude_pred=HAT_EXCLUDE_PRED, drop_overpass=MONZA, road_pred=ROAD_PRED, drop_tri=hat_banddrop)
    println("  [hatband] physics ground: ", HAT_BANDDROP_N[], " mid/far-band road triangles left out (JM_HAT_NEARBAND)"); flush(stdout)

    const ROADHAT = let rp = ROAD_TEX
        nroad = count(t -> rp(lowercase(t.tex)), TRACKMESH0.tris)

        rh_env = get(ENV, "JM_ROADHAT", "")
        if (rh_env == "" ? (WATGLEN || NURB || MONZA || SPA || ZANDV) : rh_env != "0") && nroad >= 200
            h = CLASSICTrack.build_hat(TRACKMESH0; exclude=HAT_EXCLUDE, exclude_pred=(lt -> !rp(lt)), drop_overpass=MONZA, road_pred=ROAD_PRED)
            println("  road-only HAT: ", nroad, " road tris (align/recentre oracle)"); h
        else
            println("  road-only HAT: off for this track (", nroad, " road tris recognised) — full-terrain oracle as before")
            TERRAIN0
        end
    end

    const RECENTRE_SHIFT = zeros(length(CLASSICTrack.trk_centreline(track_file(CLASSICNAME, ".trk"))))

    const ALIGNED0 = align_centreline(CLASSICTrack.trk_centreline(track_file(CLASSICNAME, ".trk")), ROADHAT)

    function smooth_recentre(a0, a1, shiftv)
        get(ENV, "JM_RECENTRE_SMOOTH", "1") == "0" && return a1
        n = min(length(a0), length(a1)); n < 16 && return a1
        dx = [a1[k][1] - a0[k][1] for k in 1:n]; dz = [a1[k][2] - a0[k][2] for k in 1:n]
        med(v) = [sort([v[mod1(k + d, n)] for d in -3:3])[4] for k in 1:n]
        gw = [exp(-0.5*(d/1.5)^2) for d in -4:4]; gw ./= sum(gw)
        gau(v) = [sum(gw[d + 5]*v[mod1(k + d, n)] for d in -4:4) for k in 1:n]

        let thr = parse(Float64, get(ENV, "JM_RECENTRE_OUTLIER", "1.0")), W = 15,
            onr(x, z) = JRTrack.hat3d(ROADHAT, x, z; ref = Inf)[3]
            if thr > 0
                medw(v, k) = sort([v[mod1(k + d, n)] for d in -W:W])[W + 1]
                tx = [medw(dx, k) for k in 1:n]; tz = [medw(dz, k) for k in 1:n]
                bad = [hypot(dx[k] - tx[k], dz[k] - tz[k]) > thr for k in 1:n]
                if any(bad) && !all(bad)
                    for k0 in 1:n
                        (bad[k0] && !bad[mod1(k0 - 1, n)]) || continue
                        run = Int[]; k = k0; while bad[mod1(k, n)] && length(run) <= n; push!(run, mod1(k, n)); k += 1; end
                        ok = length(run) <= 20 && all(run) do q
                            j = mod1(q + 1, n); ux = a0[j][1] - a0[q][1]; uz = a0[j][2] - a0[q][2]; ul = max(hypot(ux, uz), 1e-9)
                            nx = -uz/ul; nz = ux/ul; cx = a0[q][1] + tx[q]; cz = a0[q][2] + tz[q]
                            onr(cx, cz) && onr(cx + 2.5nx, cz + 2.5nz) && onr(cx - 2.5nx, cz - 2.5nz)
                        end
                        ok || foreach(q -> bad[q] = false, run)
                    end
                end
                if any(bad) && !all(bad)
                    for k in 1:n
                        bad[k] || continue
                        i = k; while bad[mod1(i, n)]; i -= 1; end
                        j = k; while bad[mod1(j, n)]; j += 1; end
                        u = (k - i)/(j - i); i = mod1(i, n); j = mod1(j, n)
                        dx[k] = dx[i] + (dx[j] - dx[i])*u; dz[k] = dz[i] + (dz[j] - dz[i])*u
                    end
                    runs = Int[]; for k in 1:n; bad[k] && !bad[mod1(k - 1, n)] && push!(runs, k); end
                    acc = zeros(n); for k in 2:n; acc[k] = acc[k-1] + hypot(a0[k][1] - a0[k-1][1], a0[k][2] - a0[k-1][2]); end
                    println("  re-centre junction pulls removed (AIJIT-3): ", count(bad), " node(s) in ", length(runs),
                            " run(s) at s ", join((round(Int, acc[k]) for k in runs), ", "), " (JM_RECENTRE_OUTLIER=0 disables)")
                end
            end
        end
        sx = gau(med(dx)); sz = gau(med(dz))
        out = [(a0[k][1] + sx[k], a0[k][2] + sz[k]) for k in 1:n]
        worst = maximum(hypot(out[k][1] - a1[k][1], out[k][2] - a1[k][2]) for k in 1:n)
        for k in 1:min(n, length(shiftv))
            j = mod1(k + 1, n); tx = a0[j][1] - a0[k][1]; tz = a0[j][2] - a0[k][2]; tl = max(hypot(tx, tz), 1e-9)
            shiftv[k] = sx[k]*(-tz/tl) + sz[k]*(tx/tl)
        end
        println("  re-centre shift smoothed (AIJIT-1): largest node correction ", round(worst, digits = 2), " m")
        out
    end
    const ALIGNED  = let a = ALIGNED0

        haskey(ENV, "JM_NO_RECENTRE") ? a :
            haskey(ENV,"JM_NORECENTRE") ? (println("  re-centring SKIPPED (JM_NORECENTRE) — E84-S4 A/B"); a) :
            smooth_recentre(a, recentre_on_road(a, ROADHAT; passes = (ROADHAT === TERRAIN0 ? 1 : 4), shift = RECENTRE_SHIFT), RECENTRE_SHIFT)
    end

    if get(ENV, "JM_SHIFT_PROFILE", "0") != "0"
        let n = length(RECENTRE_SHIFT), nb = 20
            println("SHIFT_PROFILE track=", TRACKSEL, " nodes=", n, " mean|shift|=", round(sum(abs, RECENTRE_SHIFT)/n, digits=2),
                    " max|shift|=", round(maximum(abs, RECENTRE_SHIFT), digits=2))
            for b in 0:nb-1
                r = (b*n÷nb + 1):((b+1)*n÷nb); v = RECENTRE_SHIFT[r]
                println("  bin ", lpad(b, 2), "  mean ", lpad(round(sum(v)/length(v), digits=2), 6), "  max|.| ",
                        lpad(round(maximum(abs, v), digits=2), 5))
            end
        end

        let A = ALIGNED0, B = ALIGNED, n = min(length(ALIGNED0), length(ALIGNED))
            ax = sum(p[1] for p in A[1:n])/n; az = sum(p[2] for p in A[1:n])/n
            bx = sum(p[1] for p in B[1:n])/n; bz = sum(p[2] for p in B[1:n])/n
            sxx = 0.0; sxy = 0.0; saa = 0.0
            for k in 1:n
                u = A[k][1]-ax; v = A[k][2]-az; p = B[k][1]-bx; q = B[k][2]-bz
                sxx += u*p + v*q; sxy += u*q - v*p; saa += u^2 + v^2
            end
            th = atan(sxy, sxx); sc = hypot(sxx, sxy)/saa
            res = [hypot(bx + sc*(cos(th)*(A[k][1]-ax) - sin(th)*(A[k][2]-az)) - B[k][1],
                         bz + sc*(sin(th)*(A[k][1]-ax) + cos(th)*(A[k][2]-az)) - B[k][2]) for k in 1:n]
            r = sort(res)
            println("SHIFT_RIGID track=", TRACKSEL, " rot=", round(rad2deg(th), digits=4), " deg scale=", round(sc, digits=6),
                    " trans=(", round(bx-ax, digits=2), ",", round(bz-az, digits=2), ")  residual p50=", round(r[n÷2], digits=2),
                    " p90=", round(r[9n÷10], digits=2), " max=", round(r[end], digits=2), "  (before: mean|shift| ",
                    round(sum(abs, RECENTRE_SHIFT)/length(RECENTRE_SHIFT), digits=2), ")")
        end
        flush(stdout); exit(0)
    end
    const RIBBON0  = CLASSICTrack.build_surface(ALIGNED, TERRAIN0)

    SECTRI = Render.CLASSIC3DO.Tri[]; SECPARTS = Render.TrackPart[]; RINGSPRITES = NamedTuple[]
    if NURB && isfile(joinpath(ZD, "nurburg.dat"))
        print("scenery… "); flush(stdout)
        dp = Render.CLASSICDat.parse_dat(joinpath(ZD, "nurburg.dat"))
        SECTRI, SECPARTS, RINGSPRITES = classic_scenery(ZTRK, dp, RIBBON0)
        println("  [scenery] placement matrices with pitch/roll: ", SCEN_PRSTAT[], "  (JM_SCEN_EULER=", get(ENV, "JM_SCEN_EULER", "ext"), ", JM_SCEN_PR=", get(ENV, "JM_SCEN_PR", "1,1"), ")")
        print(length(SECPARTS), " groups / ", length(SECTRI), " tris / ",
              length(RINGSPRITES), " sprites… ")
    end

    function drop_overhang(m)
        get(ENV, "JM_KEEP_OVERHANG", "0") != "0" && return m
        keep = trues(length(m.tris)); ndrop = 0; names = Set{String}()
        for (k, t) in enumerate(m.tris)
            lt = lowercase(t.tex); startswith(lt, "br") && continue
            cx = sum(q[1] for q in t.p)/3; cy = sum(q[2] for q in t.p)/3; cz = sum(q[3] for q in t.p)/3
            n1 = (t.p[2][1]-t.p[1][1], t.p[2][2]-t.p[1][2], t.p[2][3]-t.p[1][3]); n2 = (t.p[3][1]-t.p[1][1], t.p[3][2]-t.p[1][2], t.p[3][3]-t.p[1][3])
            nx = n1[2]*n2[3]-n1[3]*n2[2]; ny = n1[3]*n2[1]-n1[1]*n2[3]; nz = n1[1]*n2[2]-n1[2]*n2[1]; nn = sqrt(nx^2+ny^2+nz^2)
            (nn > 40.0 && abs(nz)/nn > 0.5) || continue
            hr = JRTrack.hat(RIBBON0, cx, cy)
            (hr.found && abs(hr.lateral) < 4.0 && 2.5 < cz - hr.height < 6.0) || continue
            keep[k] = false; ndrop += 1; push!(names, t.tex)
        end
        ndrop == 0 && return m
        println("  [overhang] OVERHANG-1: dropped ", ndrop, " polygon(s) hanging over the tarmac: ", join(names, " "), " (JM_KEEP_OVERHANG=1 keeps)")
        Render.CLASSIC3DO.Mesh3DO(m.tris[keep], m.textures, m.groups[keep])
    end
    const ROADCURVE_ON = get(ENV, "JM_ROADCURVE", "1") != "0"

    const ROAD_SMOOTHN = get(ENV, "JM_ROAD_SMOOTHN", "0") != "0"
    function smooth_road_normals(m)
        ROAD_SMOOTHN || return m
        q(v) = (round(Int, v[1]*50), round(Int, v[2]*50), round(Int, v[3]*50))
        fn = Dict{Int,NTuple{3,Float64}}(); acc = Dict{NTuple{3,Int},Vector{NTuple{3,Float64}}}()
        for (i, t) in enumerate(m.tris)
            ROAD_TEX(lowercase(t.tex)) || continue
            a = t.p[1]; b = t.p[2]; c = t.p[3]
            u = (b[1]-a[1], b[2]-a[2], b[3]-a[3]); w = (c[1]-a[1], c[2]-a[2], c[3]-a[3])
            n = (u[2]*w[3]-u[3]*w[2], u[3]*w[1]-u[1]*w[3], u[1]*w[2]-u[2]*w[1])
            n0 = t.n[1]; (n[1]*n0[1] + n[2]*n0[2] + n[3]*n0[3] < 0) && (n = (-n[1], -n[2], -n[3]))
            fn[i] = n
            for v in t.p; push!(get!(acc, q(v), NTuple{3,Float64}[]), n); end
        end
        nrm(v) = (l = sqrt(v[1]^2 + v[2]^2 + v[3]^2); l < 1e-12 ? (0.0, 0.0, 1.0) : (v[1]/l, v[2]/l, v[3]/l))
        cc = cosd(30.0); nsm = 0
        tris = similar(m.tris)
        for (i, t) in enumerate(m.tris)
            if !haskey(fn, i)
                tris[i] = t; continue
            end
            f = nrm(fn[i])
            ns = ntuple(3) do k
                s_ = (0.0, 0.0, 0.0)
                for g in acc[q(t.p[k])]
                    gn = nrm(g); (gn[1]*f[1] + gn[2]*f[2] + gn[3]*f[3] >= cc) && (s_ = (s_[1]+g[1], s_[2]+g[2], s_[3]+g[3]))
                end
                r = nrm(s_); (Float32(r[1]), Float32(r[2]), Float32(r[3]))
            end
            nsm += 1
            tris[i] = Render.CLASSIC3DO.Tri(t.p, ns, t.uv, t.tex, t.col, t.flat, t.ptype)
        end
        println("  [roadshade] smooth normals on ", nsm, " road tris (", length(acc), " welded vertices; JM_ROAD_SMOOTHN=0 reverts)")
        Render.CLASSIC3DO.Mesh3DO(tris, m.textures, m.groups)
    end
    const TRACKMESH0C = if ROADCURVE_ON
        let P = [(p[1], p[3]) for p in RIBBON0.pos], HS = [p[2] for p in RIBBON0.pos], q = RIBBON0.perp[1], tol = parse(Float64, get(ENV, "JM_ROADCURVE_TOL", "0.05")), sig = parse(Float64, get(ENV, "JM_ROADCURVE_SIG", "2.0"))
            length(P) > 2 && hypot(P[end][1]-P[1][1], P[end][2]-P[1][2]) < 0.5 && (pop!(P); pop!(HS))
            post(path, m) = abspath(path) == abspath(ZTRK) ? smooth_road_normals(RoadCurve.curve_mesh(drop_overhang(m), P, (q[1], q[3]); tol=tol, sig=sig, heights=HS)[1]) : m
            Render.CLASSIC3DO.POSTPROC[] = post
            mc, st = RoadCurve.curve_mesh(drop_overhang(TRACKMESH0), P, (q[1], q[3]); tol=tol, sig=sig, heights=HS)
            println("  [roadcurve] ON: ", st.tris_in, " -> ", st.tris_out, " tris (", st.curved, " polygons rounded; ",
                    st.mapped, "/", st.verts, " vertices in the track frame, ", st.overhead, " left alone as overhead; tol ", tol, " m; JM_ROADCURVE=0 disables)")
            mc
        end
    else
        Render.CLASSIC3DO.POSTPROC[] = (path, m) -> abspath(path) == abspath(ZTRK) ? smooth_road_normals(drop_overhang(m)) : m
        drop_overhang(TRACKMESH0)
    end

    if haskey(ENV, "JM_MESHDUMP")
        let a = split(ENV["JM_MESHDUMP"], ","), s0 = parse(Float64, a[1]), R = parse(Float64, a[2]),
            fn = length(a) > 2 ? a[3] : "/tmp/jm_meshdump.txt", P = RIBBON0.pos, acc = 0.0, c = P[1]
            for k in 2:length(P)
                acc += hypot(P[k][1] - P[k-1][1], P[k][3] - P[k-1][3]); acc >= s0 && (c = P[k]; break)
            end
            open(fn, "w") do io
                for (tag, m) in (("raw", TRACKMESH0), ("cur", TRACKMESH0C)), t in m.tris
                    any(v -> hypot(v[1] - c[1], v[2] - c[3]) < R, t.p) || continue
                    println(io, tag, " ", isempty(t.tex) ? "-" : t.tex, " ", join((string(round(v[1], digits = 3), " ", round(v[2], digits = 3)) for v in t.p), " "))
                end
                for q in P; hypot(q[1] - c[1], q[3] - c[3]) < R && println(io, "rib ", q[1], " ", q[3]); end
            end
            println("  [meshdump] s ", s0, " centre (", round(c[1], digits = 1), ", ", round(c[3], digits = 1), ") -> ", fn)
        end
    end
    const TRACKMESH = isempty(SECTRI) ? TRACKMESH0C :
        Render.CLASSIC3DO.Mesh3DO([TRACKMESH0C.tris; SECTRI], TRACKMESH0C.textures,
                              [TRACKMESH0C.groups; fill(0, length(SECTRI))])
    const TERRAIN  = (isempty(SECTRI) && !ROADCURVE_ON) ? TERRAIN0 : CLASSICTrack.build_hat(TRACKMESH; exclude=HAT_EXCLUDE, exclude_pred=HAT_EXCLUDE_PRED, drop_overpass=MONZA, road_pred=ROAD_PRED, drop_tri=hat_banddrop)
    const TRKSURF  = CLASSICTrack.build_surface(ALIGNED, TERRAIN)

    const TRKSURF_G = let σ = parse(Float64, get(ENV, "JM_GROUND_RIBBON_SIGMA", "10.0")), P = collect(ALIGNED), n = length(P)
        if σ <= 0 || n < 16
            TRKSURF
        else
            ds = sum(hypot(P[mod1(i+1, n)][1] - P[i][1], P[mod1(i+1, n)][2] - P[i][2]) for i in 1:n) / n
            si = max(σ / ds, 0.5); W = ceil(Int, 3si); w = [exp(-(q/si)^2/2) for q in -W:W]; w ./= sum(w)
            Q = [(sum(w[q+W+1]*P[mod1(i+q, n)][1] for q in -W:W), sum(w[q+W+1]*P[mod1(i+q, n)][2] for q in -W:W)) for i in 1:n]
            CLASSICTrack.build_surface(eltype(P) <: Tuple ? convert(Vector{eltype(P)}, Q) : Q, TERRAIN)
        end
    end
    const TRKSURF_G_K = TRKSURF.lap_length / TRKSURF_G.lap_length

    const TRKSURF_G_D = let n = length(TRKSURF_G.lapdist), L = TRKSURF.lap_length
        if TRKSURF_G === TRKSURF || get(ENV, "JM_GROUND_RIBBON_REMAP", "1") == "0"
            Float64[]
        else

            d = map(1:n) do i
                p = TRKSURF_G.pos[i]; h = JRTrack.hat(TRKSURF, p[1], p[3])
                h.found ? mod(h.lapdist - TRKSURF_G_K*TRKSURF_G.lapdist[i] + L/2, L) - L/2 : NaN
            end
            ok = findall(isfinite, d)
            for i in 1:n
                isfinite(d[i]) && continue
                j = ok[argmin([min(abs(k - i), n - abs(k - i)) for k in ok])]; d[i] = d[j]
            end
            σ = parse(Float64, get(ENV, "JM_GROUND_RIBBON_SIGMA", "10.0")); si = max(σ / (L / n), 0.5)
            W = ceil(Int, 3si); w = [exp(-(q/si)^2/2) for q in -W:W]; w ./= sum(w)
            ds = [sum(w[q+W+1]*d[mod1(i+q, n)] for q in -W:W) for i in 1:n]
            println("  [ground ribbon] smoothed-ribbon lapdist drift: max |d| ", round(maximum(abs, ds), digits = 2), " m, ",
                    n - length(ok), " node(s) off the ribbon (now mapped back; JM_GROUND_RIBBON_REMAP=0 = before)")
            ds
        end
    end
    function ground_ribbon_ld(ldg)
        isempty(TRKSURF_G_D) && return ldg * TRKSURF_G_K
        L = TRKSURF_G.lapdist; n = length(L); i = clamp(searchsortedlast(L, ldg), 1, n)
        e = i == n ? TRKSURF_G.lap_length : L[i+1]; f = clamp((ldg - L[i]) / max(e - L[i], 1e-9), 0.0, 1.0)
        ldg * TRKSURF_G_K + TRKSURF_G_D[i]*(1 - f) + TRKSURF_G_D[mod1(i+1, n)]*f
    end

    const LAPLEN = TRKSURF.lap_length

    const TRKALT = (get(ENV, "JM_TRK_SURFACE", "1") != "0") ? CLASSICTrack.trk_altitude(track_file(CLASSICNAME, ".trk")) : nothing
    const TRK_CAL = let ta = TRKALT
        if ta === nothing || abs(ta.total - LAPLEN) > 0.02 * LAPLEN
            ta === nothing || println("  [trksurf] OFF: .trk lap ", round(ta.total, digits=1), " m vs ribbon ", round(LAPLEN, digits=1), " m")
            (on = false, sign = 1.0, off = 0.0, sscale = 1.0)
        else
            n = length(TRKSURF.pos)
            res = Dict(1.0 => Float64[], -1.0 => Float64[]); off0 = Float64[]
            for i in 1:6:n
                p = TRKSURF.pos[i]; q = TRKSURF.perp[i]; s = TRKSURF.lapdist[i]
                h0 = JRTrack.hat3d(TERRAIN, p[1], p[3]; ref=Inf)
                h0[3] && push!(off0, Float64(h0[1]) - CLASSICTrack.trk_height(ta, s, 0.0))
                for lat in (-4.0, 4.0)
                    hm = JRTrack.hat3d(TERRAIN, p[1] + lat*q[1], p[3] + lat*q[3]; ref=Inf)
                    hm[3] || continue
                    for sg in (1.0, -1.0)
                        push!(res[sg], abs(Float64(hm[1]) - CLASSICTrack.trk_height(ta, s, sg*lat) - (isempty(off0) ? 0.0 : off0[end])))
                    end
                end
            end
            med(v) = isempty(v) ? 0.0 : (sort!(v); v[div(length(v)+1, 2)])
            off = med(copy(off0)); sg = med(copy(res[1.0])) <= med(copy(res[-1.0])) ? 1.0 : -1.0
            r = sort!(abs.(off0 .- off))
            println("  [trksurf] ON: lateral sign ", sg, " (residual +", round(med(copy(res[1.0])), digits=3), " / -", round(med(copy(res[-1.0])), digits=3),
                    " m), vertical offset ", round(off, digits=3), " m, centreline |mesh-trk| p50 ", round(r[div(length(r)+1, 2)], digits=3),
                    " p90 ", round(r[max(1, round(Int, 0.9*length(r)))], digits=3), " m over ", length(off0), " samples")
            (on = true, sign = sg, off = off, sscale = 1.0)
        end
    end
    const TRK_ROAD_LAT = parse(Float64, get(ENV, "JM_TRK_ROAD_LAT", "5.4"))
    const TRK_BLEND = 1.0

    const TRK_CORR_ON = TRK_CAL.on && get(ENV, "JM_TRK_CORR", "1") != "0"
    const TRK_SSCALE = TRK_CORR_ON ? TRKALT.total / LAPLEN : TRK_CAL.sscale
    const TRK_CORR_LAT = (-5.0, -2.5, 0.0, 2.5, 5.0)
    const TRK_CORR_LAYER = get(ENV, "JM_TRK_CORR_LAYER", "1") != "0"

    function _trk_corr_table()
        if !TRK_CORR_ON
            (ds = 1.0, t = zeros(1, 5), cf = ones(1, 5))
        else
            n = max(8, round(Int, LAPLEN / 2.0)); ds = LAPLEN / n
            _ld = TRKSURF.lapdist; _np = length(TRKSURF.pos)
            raw = fill(NaN, n, 5); nlayer = 0
            for i in 1:n
                s = (i - 1) * ds
                k = clamp(searchsortedlast(_ld, s), 1, _np - 1); fr = (s - _ld[k]) / max(_ld[k+1] - _ld[k], 1e-6)
                p = TRKSURF.pos[k]; q = TRKSURF.pos[k+1]; pp = TRKSURF.perp[k]
                x = p[1] + (q[1]-p[1])*fr; z = p[3] + (q[3]-p[3])*fr
                for (j, lat) in enumerate(TRK_CORR_LAT)
                    hs = CLASSICTrack.trk_height(TRKALT, s * TRK_SSCALE, TRK_CAL.sign * lat) + TRK_CAL.off
                    h = JRTrack.hat3d(TERRAIN, x + lat*pp[1], z + lat*pp[3]; ref=hs + 3.0)
                    h[3] || (h = JRTrack.hat3d(TERRAIN, x + lat*pp[1], z + lat*pp[3]; ref=Inf))

                    if TRK_CORR_LAYER && h[3]
                        h2 = JRTrack.hat3d(TERRAIN, x + lat*pp[1], z + lat*pp[3]; ref=Float64(h[1]) - 0.55)
                        (h2[3] && h[1] - h2[1] < 0.6 && abs(h2[1] - hs) < abs(h[1] - hs) - 0.05) && (h = h2; nlayer += 1)
                    end
                    h[3] && (raw[i, j] = Float64(h[1]) - hs)
                end
            end

            nrej = 0
            sig = parse(Float64, get(ENV, "JM_TRK_CORR_SIG", "8.0")); kw = ceil(Int, 3sig / ds)
            w = [exp(-0.5 * (m*ds/sig)^2) for m in -kw:kw]
            t = zeros(n, 5)
            for j in 1:5, i in 1:n
                a = 0.0; b = 0.0
                for m in -kw:kw
                    v = raw[mod1(i + m, n), j]; isnan(v) && continue
                    a += w[m+kw+1] * v; b += w[m+kw+1]
                end
                t[i, j] = b > 0 ? a / b : 0.0
            end

            kc = round(Int, 10.0 / ds); cf = ones(n, 5)
            for j in 1:5, i in 1:n
                mx = 0.0
                for m in -kc:kc
                    v = raw[mod1(i + m, n), j]; isnan(v) || (mx = max(mx, abs(v - t[mod1(i + m, n), j])))
                end
                cf[i, j] = clamp(1.0 - (mx - 0.3) / 0.5, 0.0, 1.0)
            end
            nlow = count(<(0.99), cf)
            fin = filter(!isnan, raw); res = sort!(abs.(filter(!isnan, raw .- t)))
            println("  [trksurf] SINK-1 correction: sscale ", round(TRK_SSCALE, digits=5), ", mesh-spline raw min ",
                    round(minimum(fin), digits=3), " max ", round(maximum(fin), digits=3), " m -> residual p50 ",
                    round(res[div(length(res)+1, 2)], digits=3), " p99 ", round(res[max(1, round(Int, 0.99*length(res)))], digits=3),
                    " m (sigma ", sig, " m; spline confidence < 1 on ", nlow, " of ", 5n, " samples -> mesh there; JM_TRK_CORR=0 disables); ",
                    nlayer, " samples on the lower of two road layers (JM_TRK_CORR_LAYER=0 = the top)")
            (ds = ds, t = t, cf = cf)
        end
    end
    const TRK_CORR = _trk_corr_table()

    function trk_corr(s, lat)
        TRK_CORR_ON || return 0.0
        t = TRK_CORR.t; n = size(t, 1)
        u = mod(s, LAPLEN) / TRK_CORR.ds; i = floor(Int, u); f = u - i
        l = clamp(lat, TRK_CORR_LAT[1], TRK_CORR_LAT[end])
        j = clamp(floor(Int, (l - TRK_CORR_LAT[1]) / 2.5) + 1, 1, 4); g = (l - TRK_CORR_LAT[j]) / 2.5
        cr(jj) = (p0 = t[mod1(i, n), jj]; p1 = t[mod1(i+1, n), jj]; p2 = t[mod1(i+2, n), jj]; p3 = t[mod1(i+3, n), jj];
                  0.5 * (2p1 + (-p0 + p2)*f + (2p0 - 5p1 + 4p2 - p3)*f^2 + (-p0 + 3p1 - 3p2 + p3)*f^3))

        q0 = cr(clamp(j - 1, 1, 5)); q1 = cr(j); q2 = cr(j + 1); q3 = cr(clamp(j + 2, 1, 5))
        0.5 * (2q1 + (-q0 + q2)*g + (2q0 - 5q1 + 4q2 - q3)*g^2 + (-q0 + 3q1 - 3q2 + q3)*g^3)
    end
    function trk_conf(s, lat)
        TRK_CORR_ON || return 1.0
        c = TRK_CORR.cf; n = size(c, 1)
        u = mod(s, LAPLEN) / TRK_CORR.ds; i = floor(Int, u); f = u - i
        l = clamp(lat, TRK_CORR_LAT[1], TRK_CORR_LAT[end])
        j = clamp(floor(Int, (l - TRK_CORR_LAT[1]) / 2.5) + 1, 1, 4); g = (l - TRK_CORR_LAT[j]) / 2.5
        a0 = c[mod1(i+1, n), j]*(1-f) + c[mod1(i+2, n), j]*f; a1 = c[mod1(i+1, n), j+1]*(1-f) + c[mod1(i+2, n), j+1]*f
        a0*(1-g) + a1*g
    end

    smoothblend(u) = (c = clamp(u, 0.0, 1.0); c*c*(3.0 - 2.0*c))

    trk_road(s, lat) = CLASSICTrack.trk_height(TRKALT, s * TRK_SSCALE, TRK_CAL.sign * lat) + TRK_CAL.off + trk_corr(s, lat)

    for spec in filter(!isempty, split(get(ENV, "JM_GROUND_PROBE", ""), ";"))
        let (gs, gl) = parse.(Float64, split(spec, ":")), k = clamp(searchsortedlast(TRKSURF.lapdist, gs), 1, length(TRKSURF.pos) - 1)
            p = TRKSURF.pos[k]; q = TRKSURF.perp[k]; x = p[1] + gl*q[1]; z = p[3] + gl*q[3]
            hp = trk_road(gs, gl); hr = JRTrack.hat3d(ROADHAT, x, z; ref = hp + 3.0); ht = JRTrack.hat3d(TERRAIN, x, z; ref = hp + 3.0)
            sp = CLASSICTrack.trk_height(TRKALT, gs * TRK_SSCALE, TRK_CAL.sign * gl) + TRK_CAL.off
            println("  [groundprobe] s ", gs, " lat ", gl, ": physics ", round(hp, digits = 3), " (spline ", round(sp, digits = 3), " + corr ",
                    round(trk_corr(gs, gl), digits = 3), ")  drawn road ", hr[3] ? round(hr[1], digits = 3) : "-", "  TERRAIN ",
                    ht[3] ? round(ht[1], digits = 3) : "-")

            for t in TRACKMESH.tris
                a, b, c = t.p
                d = (b[2]-c[2])*(a[1]-c[1]) + (c[1]-b[1])*(a[2]-c[2]); abs(d) < 1e-9 && continue
                l1 = ((b[2]-c[2])*(x-c[1]) + (c[1]-b[1])*(z-c[2]))/d; l2 = ((c[2]-a[2])*(x-c[1]) + (a[1]-c[1])*(z-c[2]))/d; l3 = 1 - l1 - l2
                (l1 >= 0 && l2 >= 0 && l3 >= 0) || continue
                y = l1*a[3] + l2*b[3] + l3*c[3]; abs(y - hp) < 2.0 || continue
                tg = isempty(Render.CLASSIC3DO.BANDTAG) ? -1 : get(Render.CLASSIC3DO.BANDTAG, Render.CLASSIC3DO.bandkey(t.p...), Int32(0))
                println("      tri y ", round(y, digits = 3), "  tex ", t.tex, "  band ", tg, "  road ", ROAD_TEX(lowercase(t.tex)), "  dropped ", hat_banddrop(t))
            end
        end
    end

    const MESH_SMOOTH = parse(Float64, get(ENV, "JM_MESH_SMOOTH", "1.0"))

    const _GG = get(ENV, "JM_GROUND_ABOVE_OK", "0") == "1" ? "" : get(ENV, "JM_GROUND_GUARDS", "SL")
    const STENCIL_GUARD = Ref('S' in _GG)
    const LEDGE_GUARD = Ref('L' in _GG)

    function mesh_ground(x, z, ref = Inf; fallback = true)
        h0 = JRTrack.hat3d(TERRAIN, x, z; ref=ref)
        if !h0[3] && ref != Inf && fallback
            h0 = JRTrack.hat3d(TERRAIN, x, z; ref=Inf)
        end
        (h0[3] && MESH_SMOOTH > 0.0) || return (Float64(h0[1]), h0[3])
        acc = Float64(h0[1]); n = 1.0
        for (dx, dz) in ((MESH_SMOOTH, 0.0), (-MESH_SMOOTH, 0.0), (0.0, MESH_SMOOTH), (0.0, -MESH_SMOOTH))
            h = JRTrack.hat3d(TERRAIN, x + dx, z + dz; ref=ref)

            h[3] || (STENCIL_GUARD[] ? continue : return (Float64(h0[1]), true))

            wn = STENCIL_GUARD[] ? clamp((0.30 - abs(Float64(h[1]) - Float64(h0[1]))) / 0.20, 0.0, 1.0) : 1.0
            acc += wn*Float64(h[1]); n += wn
        end
        (acc/n, true)
    end
    function ground_road(x, z)
        if TRK_CAL.on && !SKIDPAD
            hr = JRTrack.hat(TRKSURF, x, z)
            if hr.found
                al = abs(hr.lateral)
                if al < TRK_ROAD_LAT + TRK_BLEND
                    ht = trk_road(hr.lapdist, hr.lateral)
                    w = smoothblend((TRK_ROAD_LAT + TRK_BLEND - al) / TRK_BLEND) * trk_conf(hr.lapdist, hr.lateral)

                    w >= 0.999 && return (ht, true)
                    hm, ok = mesh_ground(x, z, ht + 3.0)

                    if ok && LEDGE_GUARD[] && hm - ht > 0.3

                        g = clamp((hm - ht - 0.3) / 0.3, 0.0, 1.0)
                        hm2, ok2 = mesh_ground(x, z, ht + 0.3; fallback = false)
                        (ok2 && hm2 > ht - 1.5) && (hm = (1 - g)*hm + g*hm2)
                    end
                    return (ok ? w*ht + (1-w)*hm : ht, true)
                end
            end
        end
        mesh_ground(x, z)
    end

    if haskey(ENV, "JM_ROADBLOCK")
        rb_lat = (v = tryparse(Float64, get(ENV, "JM_ROADBLOCK", "")); v === nothing || v <= 0 ? 5.5 : v)
        rb_h   = parse(Float64, get(ENV, "JM_ROADBLOCK_H", "1.5"))

        rb_low = parse(Float64, get(ENV, "JM_ROADBLOCK_LOW", "3.0"))

        rb_bin = parse(Float64, get(ENV, "JM_ROADBLOCK_BIN", "5.0"))
        spans = Dict{Int,Vector{Tuple{Float64,Float64,String}}}()
        nv = Ref(0)
        for t in TRACKMESH.tris

            n1 = (t.p[2][1]-t.p[1][1], t.p[2][2]-t.p[1][2], t.p[2][3]-t.p[1][3])
            n2 = (t.p[3][1]-t.p[1][1], t.p[3][2]-t.p[1][2], t.p[3][3]-t.p[1][3])
            nx = n1[2]*n2[3]-n1[3]*n2[2]; ny = n1[3]*n2[1]-n1[1]*n2[3]; nz = n1[1]*n2[2]-n1[2]*n2[1]
            nn = sqrt(nx^2 + ny^2 + nz^2)
            (nn > 1e-6 && abs(nz)/nn < 0.35) || continue
            nv[] += 1
            lats = Float64[]; hs = Float64[]; sds = Float64[]
            ok = true
            for q in t.p
                hr = JRTrack.hat(RIBBON0, Float64(q[1]), Float64(q[2]))
                hr.found || (ok = false; break)
                push!(lats, hr.lateral); push!(hs, Float64(q[3]) - hr.height); push!(sds, hr.lapdist)
            end
            ok || continue
            (maximum(hs) >= rb_h && minimum(hs) <= rb_low) || continue
            lo, hi = minimum(lats), maximum(lats)
            (hi < -rb_lat || lo > rb_lat) && continue
            push!(get!(spans, floor(Int, sds[1]/rb_bin), Tuple{Float64,Float64,String}[]),
                  (max(lo, -rb_lat), min(hi, rb_lat), lowercase(t.tex)))
        end

        nsp = Ref(0)
        if isdefined(Main, :RINGSPRITES) && RINGSPRITES !== nothing
            for sp in RINGSPRITES

                hr = JRTrack.hat(RIBBON0, Float64(sp.x), -Float64(sp.z))
                hr.found || continue
                (Float64(sp.y) + Float64(sp.h) - hr.height >= rb_h &&
                 Float64(sp.y) - hr.height <= rb_low) || continue
                half = Float64(sp.w)/2
                lo = hr.lateral - half; hi = hr.lateral + half
                (hi < -rb_lat || lo > rb_lat) && continue
                nsp[] += 1
                push!(get!(spans, floor(Int, hr.lapdist/rb_bin), Tuple{Float64,Float64,String}[]),
                      (max(lo, -rb_lat), min(hi, rb_lat), "sprite:" * lowercase(String(sp.name))))
            end
        end

        nsc = Ref(0); ntri_sec = Ref(0); normdots = Float64[]
        rb_texel = get(ENV, "JM_ROADBLOCK_TEXEL", "1") != "0"; rb_tex = Ref{Any}(nothing); rb_rgba = Dict{String,Any}()
        if isdefined(Main, :SECPARTS) && SECPARTS !== nothing
            for prt in SECPARTS
                local v = prt.verts; n = length(v) ÷ 11
                for q in 0:(n ÷ 3 - 1)
                    lats = Float64[]; hs = Float64[]; sds = Float64[]; ok = true
                    for c in 0:2
                        k = 11*(3q + c)
                        vx = Float64(v[k+1]); vy = Float64(v[k+2]); vz = Float64(v[k+3])
                        hr = JRTrack.hat(RIBBON0, vx, -vz)
                        hr.found || (ok = false; break)
                        push!(lats, hr.lateral); push!(hs, vy - hr.height); push!(sds, hr.lapdist)
                    end
                    ok || continue
                    ntri_sec[] += 1
                    (maximum(hs) >= rb_h && minimum(hs) <= rb_low) || continue
                    lo, hi = minimum(lats), maximum(lats)
                    (hi < -rb_lat || lo > rb_lat) && continue

                    if rb_texel
                        rb_tex[] === nothing && (rb_tex[] = Render.classic_texture_index(ZD))
                        tx = get!(() -> Render.tex_rgba(rb_tex[], prt.tex), rb_rgba, prt.tex)
                        k0 = 11*(3q); k1 = k0 + 11; k2 = k1 + 11
                        wh = ((v[k0+1], -v[k0+3], v[k0+2]), (v[k1+1], -v[k1+3], v[k1+2]), (v[k2+1], -v[k2+3], v[k2+2]))
                        uvs = ((v[k0+10], v[k0+11]), (v[k1+10], v[k1+11]), (v[k2+10], v[k2+11]))
                        sp = veil_opaque_span(wh, uvs, tx, RIBBON0; lat=rb_lat, hlo=0.3, hhi=rb_h + 1.5)
                        sp === nothing && continue
                        lo, hi = sp
                    end
                    nsc[] += 1

                    let k0 = 11*(3q), k1 = 11*(3q+1), k2 = 11*(3q+2)
                        ax = Float64(v[k1+1]) - Float64(v[k0+1]); ay = Float64(v[k1+2]) - Float64(v[k0+2]); az = Float64(v[k1+3]) - Float64(v[k0+3])
                        bx = Float64(v[k2+1]) - Float64(v[k0+1]); by = Float64(v[k2+2]) - Float64(v[k0+2]); bz = Float64(v[k2+3]) - Float64(v[k0+3])
                        gnx = ay*bz - az*by; gnz = ax*by - ay*bx
                        i = clamp(searchsortedlast(RIBBON0.lapdist, sds[1]), 1, length(RIBBON0.pos))
                        pq = RIBBON0.perp[i]

                        inx = -sign(lats[1]) * pq[1]; inz = -sign(lats[1]) * (-pq[3])
                        d = gnx*inx + gnz*inz
                        nl = hypot(gnx, gnz)
                        nl > 1e-9 && push!(normdots, d/nl)
                    end
                    push!(get!(spans, floor(Int, sds[1]/rb_bin), Tuple{Float64,Float64,String}[]),
                          (max(lo, -rb_lat), min(hi, rb_lat), "drawn:" * lowercase(String(prt.tex))))
                end
            end
        end

        if haskey(ENV, "JM_ROADBLOCK_AT")
            at_s = parse(Float64, ENV["JM_ROADBLOCK_AT"])
            rows = Tuple{Float64,Float64,Float64,Float64,Float64,String}[]
            for (k, v) in spans, (lo, hi, tx) in v
                sv = k*rb_bin
                abs(sv - at_s) <= 25.0 && push!(rows, (sv, lo, hi, hi-lo, 0.0, tx))
            end
            sort!(rows, by = x -> (x[1], x[2]))
            println("  [roadblock] surfaces within 25 m of s=", at_s, ":  ", length(rows), " row(s)")
            for (sv, lo, hi, w, _, tx) in rows[1:min(60, end)]
                println("       s=", lpad(round(Int, sv), 6), " m  lateral ", lpad(round(lo, digits=1), 6),
                        " .. ", rpad(round(hi, digits=1), 6), " (", round(w, digits=1), " m wide)   ", tx)
            end
        end

        if haskey(ENV, "JM_ROADBLOCK_PLACE")
            pat = lowercase(get(ENV, "JM_ROADBLOCK_PLACE", "halftr"))
            npl = Ref(0); onroad = Ref(0)

            texcache = Dict{String,Bool}()
            function uses_tex(nm)
                get!(texcache, lowercase(String(nm))) do
                    v = get(TRACKDAT, lowercase(String(nm)) * ".3do", nothing)
                    v === nothing && return false
                    try
                        q = joinpath(tempdir(), "jm_rb_" * lowercase(String(nm)) * ".3do")
                        isfile(q) || write(q, v)
                        any(t -> occursin(pat, lowercase(t.tex)), Render.CLASSIC3DO.parse_3do(q).tris)
                    catch; false end
                end
            end

            near = haskey(ENV, "JM_ROADBLOCK_PLACE_NEAR") ? parse(Float64, ENV["JM_ROADBLOCK_PLACE_NEAR"]) : NaN
            rows = Tuple{Float64,String,Float64,Float64,Float64,Float64}[]
            println("  [roadblock] placed objects whose MESH uses a texture matching \"", pat, "\"",
                    isnan(near) ? ":" : string(", nearest lapdist ", near, ":"))
            try
                for (nm, t) in Render.CLASSIC3DO.classic_placements(ZTRK)
                    (isempty(pat) || uses_tex(nm)) || continue
                    npl[] += 1

                    hr = JRTrack.hat(RIBBON0, Float64(t[1]), Float64(t[2]))
                    hr.found || continue
                    abs(hr.lateral) <= rb_lat && (onroad[] += 1)

                    rh = let i = clamp(searchsortedlast(RIBBON0.lapdist, hr.lapdist), 1, length(RIBBON0.pos)),
                             j = mod1(i+1, length(RIBBON0.pos))
                        atan(RIBBON0.pos[j][3] - RIBBON0.pos[i][3], RIBBON0.pos[j][1] - RIBBON0.pos[i][1])
                    end
                    if !isnan(near)
                        dd = abs(rem(hr.lapdist - near, RIBBON0.lap_length, RoundNearest))
                        push!(rows, (dd, String(nm) * " p" * string(round(rad2deg(Float64(t[5])), digits=1)) *
                                         " r" * string(round(rad2deg(Float64(t[6])), digits=1)), hr.lapdist, hr.lateral, rad2deg(Float64(t[4])),
                                     rad2deg(rem(Float64(t[4]) - rh, 2pi, RoundNearest))))
                        continue
                    end
                    npl[] <= 24 && println("       ", rpad(String(nm), 10), " origin s=",
                                           lpad(round(Int, hr.lapdist), 6), " lat ",
                                           lpad(round(hr.lateral, digits=1), 6),
                                           "  yaw ", lpad(round(rad2deg(Float64(t[4])), digits=1), 7),
                                           " deg  road ", lpad(round(rad2deg(rh), digits=1), 7),
                                           " deg  diff ", lpad(round(rad2deg(rem(Float64(t[4]) - rh, 2pi, RoundNearest)), digits=1), 7),
                                           abs(hr.lateral) <= rb_lat ? "   <- ORIGIN ON ROAD" : "")
                end
                if !isnan(near)
                    sort!(rows, by = first)
                    for (dd, nm, sv, lt, yw, df) in rows[1:min(30, end)]
                        println("       ", rpad(nm, 10), " s=", lpad(round(Int, sv), 6), " lat ",
                                lpad(round(lt, digits=1), 7), "  yaw ", lpad(round(yw, digits=1), 7),
                                "  yaw-road ", lpad(round(df, digits=1), 7),
                                " deg   (", round(dd, digits=1), " m from s=", round(Int, near), ")")
                    end
                end
                println("       ", npl[], " placement(s) matched; ", onroad[],
                        " have their ORIGIN inside the corridor")
            catch e
                println("       (classic_placements failed: ", sprint(showerror, e), ")")
            end
        end

        function rb_union(v)
            iv = sort([(a, b) for (a, b, _) in v]); out = Tuple{Float64,Float64}[]
            for (a, b) in iv
                (isempty(out) || a > out[end][2] + 1e-9) ? push!(out, (a, b)) :
                    (out[end] = (out[end][1], max(out[end][2], b)))
            end
            (out, sum(b - a for (a, b) in out; init = 0.0))
        end
        curtain = Tuple{Float64,Float64,String}[]; partial = Tuple{Float64,Float64,String}[]
        for (k, v) in spans
            local u, w = rb_union(v)
            covers0 = any(a <= 0.0 <= b for (a, b) in u)
            tex = join(sort(unique(t for (_, _, t) in v)), "+")
            (covers0 ? curtain : partial) === curtain ?
                push!(curtain, (k*rb_bin, w, tex)) : push!(partial, (k*rb_bin, w, tex))
        end
        sort!(curtain, by = first); sort!(partial, by = x -> -x[2])
        println("  [roadblock] track=", TRACKSEL, " corridor |lat| <= ", rb_lat, " m, must reach ", rb_h,
                " m above the road; ", nv[], " near-vertical track-mesh triangles examined, ",
                length(spans), " bins of ", rb_bin, " m touched; ", nsp[], " sprite panel(s) reach the corridor; of ", ntri_sec[],
                " DRAWN scenery triangles near the ribbon, ", nsc[], " reach it")
        println("  [roadblock] CURTAINS -- the union covers the CENTRELINE, so the car must drive through: ",
                length(curtain), " bin(s)")

        let places = Tuple{Float64,Float64,Float64,String}[]
            for (sv, w, tx) in curtain
                if !isempty(places) && sv - places[end][2] <= 2*rb_bin + 1e-9
                    places[end] = (places[end][1], sv, max(places[end][3], w), places[end][4])
                else
                    push!(places, (sv, sv, w, tx))
                end
            end
            println("     ", length(places), " place(s):")
            for (s0, s1, w, tx) in places[1:min(20, end)]
                println("       s=", lpad(round(Int, s0), 7), "..", rpad(round(Int, s1 + rb_bin), 7),
                        " m  widest cover ", lpad(round(w, digits=2), 5), " m of the ", 2*rb_lat,
                        " m corridor   ", tx)
            end
        end
        println("  [roadblock] PARTIAL intrusions (widest first), ", length(partial), " bin(s):")
        for (sv, w, tx) in partial[1:min(10, end)]
            println("       s=", lpad(round(Int, sv), 7), " m  covers ", lpad(round(w, digits=2), 5), " m   ", tx)
        end
        if !isempty(normdots)
            away = count(<(0.0), normdots); toward = count(>(0.0), normdots)
            u = sort(copy(normdots))
            println("  [roadblock] FRONT-FACE direction of the ", length(normdots),
                    " drawn surfaces in the corridor (normal . direction toward the road centreline):")
            println("     points AWAY from the road: ", away, " (", round(100away/length(normdots), digits=1),
                    "%)   TOWARD it: ", toward, " (", round(100toward/length(normdots), digits=1), "%)")
            println("     cos p10 ", round(u[max(1, length(u)÷10)], digits=3), " p50 ",
                    round(u[length(u)÷2 + 1], digits=3), " p90 ", round(u[max(1, 9*length(u)÷10)], digits=3))
            println("     reading: if these point AWAY, Classic culls them from the road side and our winding is",
                    " inverted; if TOWARD, they are meant to be seen and the fault is elsewhere")
        end

        asp = Dict{Int,Vector{Tuple{Float64,String}}}()
        asp_noalpha = get(ENV, "JM_ASPHALT_NOALPHA", "0") != "0"
        asp_pat = lowercase(get(ENV, "JM_ASPHALT_PAT", ""))

        asp_gpat = lowercase(get(ENV, "JM_ASPHALT_GAP", ""))
        gapd = Dict{Int,Vector{Float64}}()

        asp_h0 = parse(Float64, get(ENV, "JM_ASPHALT_HMIN", "0.3")); asp_h1 = parse(Float64, get(ENV, "JM_ASPHALT_HMAX", "3.0"))
        overroad(x, y, z) = ((h, _, f) = JRTrack.hat3d(ROADHAT, x, y; ref = z); f && asp_h0 <= z - h <= asp_h1)
        if isdefined(Main, :SECPARTS) && SECPARTS !== nothing
            rb_tex[] === nothing && (rb_tex[] = Render.classic_texture_index(ZD))
            for prt in SECPARTS
                local v = prt.verts; n = length(v) ÷ 11
                tx = get!(() -> Render.tex_rgba(rb_tex[], prt.tex), rb_rgba, prt.tex)
                for q in 0:(n ÷ 3 - 1)
                    k0 = 11*(3q); k1 = k0 + 11; k2 = k1 + 11
                    wh = ((v[k0+1], -v[k0+3], v[k0+2]), (v[k1+1], -v[k1+3], v[k1+2]), (v[k2+1], -v[k2+3], v[k2+2]))
                    uvs = ((v[k0+10], v[k0+11]), (v[k1+10], v[k1+11]), (v[k2+10], v[k2+11]))
                    best = Inf; bs = NaN; m = 12
                    for i in 0:m, j in 0:(m-i)
                        a = i/m; b = j/m; c = 1 - a - b
                        x = Float64(a*wh[1][1] + b*wh[2][1] + c*wh[3][1]); y = Float64(a*wh[1][2] + b*wh[2][2] + c*wh[3][2])
                        z = Float64(a*wh[1][3] + b*wh[2][3] + c*wh[3][3])
                        isgap = !isempty(asp_gpat) && occursin(asp_gpat, lowercase(String(prt.tex)))
                        (isgap || overroad(x, y, z)) || continue
                        if tx !== nothing && !asp_noalpha
                            tw, th, rgba = tx
                            uu = a*uvs[1][1] + b*uvs[2][1] + c*uvs[3][1]; vv = a*uvs[1][2] + b*uvs[2][2] + c*uvs[3][2]
                            px = mod(floor(Int, uu*tw), tw); py = mod(floor(Int, vv*th), th)
                            rgba[4*(py*tw + px) + 4] < 0x0a && continue
                        end
                        hr = JRTrack.hat(RIBBON0, x, y); hr.found || continue
                        if isgap
                            if abs(hr.lateral) < 15 && 0.3 <= z - hr.height <= 3.0
                                g_ = get!(gapd, floor(Int, hr.lapdist/rb_bin), [Inf, -Inf])
                                hr.lateral >= 0 ? (g_[1] = min(g_[1], hr.lateral)) : (g_[2] = max(g_[2], hr.lateral))
                            end
                            overroad(x, y, z) || continue
                        end
                        abs(hr.lateral) < abs(best) && (best = hr.lateral; bs = hr.lapdist)
                    end
                    isfinite(best) && push!(get!(asp, floor(Int, bs/rb_bin), Tuple{Float64,String}[]),
                                            (best, "drawn:" * lowercase(String(prt.tex))))
                end
            end
        end
        if isdefined(Main, :RINGSPRITES) && RINGSPRITES !== nothing
            for sp in RINGSPRITES
                hr = JRTrack.hat(RIBBON0, Float64(sp.x), -Float64(sp.z)); hr.found || continue
                half = Float64(sp.w)/2; best = Inf
                for f in -1:0.25:1
                    x = Float64(sp.x) + f*half*hr.perp[1]; y = -Float64(sp.z) + f*half*hr.perp[2]
                    (h, _, fd) = JRTrack.hat3d(ROADHAT, x, y; ref = Float64(sp.y) + 0.5)
                    (fd && Float64(sp.y) - h <= asp_h1 && Float64(sp.y) + Float64(sp.h) - h >= asp_h0) || continue
                    l_ = hr.lateral + f*half; abs(l_) < abs(best) && (best = l_)
                end
                isfinite(best) && push!(get!(asp, floor(Int, hr.lapdist/rb_bin), Tuple{Float64,String}[]),
                                        (best, "sprite:" * lowercase(String(sp.name))))
            end
        end

        function asp_edge(sd, sg)
            i = clamp(searchsortedlast(RIBBON0.lapdist, sd), 1, length(RIBBON0.pos))
            p = RIBBON0.pos[i]; q = RIBBON0.perp[i]; e = 0.0
            for d in 0.0:0.05:15.0
                (_, _, f) = JRTrack.hat3d(ROADHAT, p[1] + sg*d*q[1], p[3] + sg*d*q[3]; ref = p[2] + 3.0)
                f ? (e = d) : (d > e + 0.3 && break)
            end
            e
        end
        let rows = [(k*rb_bin, (b = v[argmin(abs.(first.(v)))][1]; b), join(sort(unique(last.(v))), "+")) for (k, v) in asp]
            sort!(rows, by = r -> abs(r[2]))
            bytex = Dict{String,Int}()
            for (k, v) in asp, t in unique(last.(v)); bytex[t] = get(bytex, t, 0) + 1; end
            deep = count(r -> abs(r[2]) < 3.0, rows)
            println("  [roadblock] ASPHALT intrusions (drawn texels 0.3-3 m above a drawn road triangle): ",
                    length(rows), " bin(s), ", deep, " within 3 m of the centreline")
            for (t, c) in sort(collect(bytex), by = x -> -x[2])[1:min(12, end)]
                println("       ", rpad(t, 28), lpad(c, 5), " bins")
            end
            for (sv, d, tx) in rows[1:min(15, end)]
                println("       s=", lpad(round(Int, sv), 7), " m  innermost |lat| ", lpad(round(abs(d), digits=2), 5), " m   ", tx)
            end
            if !isempty(asp_pat)
                pr = [r for r in rows if any(occursin(p_, r[3]) for p_ in split(asp_pat, ","))]
                println("  [roadblock] ASPHALT rows matching '", asp_pat, "': ", length(pr))
                for (sv, d, tx) in pr[1:min(40, end)]
                    e_ = asp_edge(sv + rb_bin/2, sign(d))
                    println("       pat s=", lpad(round(Int, sv), 7), " m  innermost |lat| ", lpad(round(abs(d), digits=2), 5),
                            " m  road edge ", lpad(round(e_, digits=2), 5), " m  -> ", lpad(round(e_ - abs(d), digits=2), 6), " m onto the road   ", tx)
                end
            end
            if !isempty(asp_gpat)

                dl = Tuple{Float64,Float64,Float64,Float64}[]
                for k in sort(collect(keys(gapd))), sd in (1, 2)
                    g_ = gapd[k]; isfinite(g_[sd]) || continue
                    sg = sd == 1 ? 1.0 : -1.0; e_ = asp_edge(k*rb_bin + rb_bin/2, sg)
                    push!(dl, (k*rb_bin, sg, abs(g_[sd]), e_ - abs(g_[sd])))
                end
                if !isempty(dl)
                    ds = sort(last.(dl)); qq(f) = round(ds[clamp(round(Int, f*length(ds)), 1, length(ds))], digits=2)
                    println("  [roadblock] veil inner edge vs drawn road edge ('", asp_gpat, "'): ", length(dl), " bin-sides; depth onto road",
                            " p10 ", qq(0.1), " p50 ", qq(0.5), " p90 ", qq(0.9), " max ", round(ds[end], digits=2),
                            "; ", count(>(0.0), ds), " > 0, ", count(>(0.5), ds), " > 0.5 m")
                    for (sv, sg, il, dp) in sort(dl, by = x -> -x[4])[1:min(12, end)]
                        println("       s=", lpad(round(Int, sv), 6), " side ", sg > 0 ? "+" : "-", "  inner ", lpad(round(il, digits=2), 6),
                                "  depth ", lpad(round(dp, digits=2), 6))
                    end
                end
            end
            println("ASPHALT_RESULT track=", TRACKSEL, " bins=", length(rows), " deep_bins=", deep)
        end
        println("ROADBLOCK_RESULT track=", TRACKSEL, " curtain_bins=", length(curtain),
                " partial_bins=", length(partial))
        flush(stdout); exit(0)
    end

    if get(ENV, "JM_OVERROAD", "0") != "0"
        cnt = Dict{Tuple{String,Int},Vector{Float64}}()
        for t in TRACKMESH.tris
            cx = sum(q[1] for q in t.p)/3; cy = sum(q[2] for q in t.p)/3; cz = sum(q[3] for q in t.p)/3
            hr = JRTrack.hat(TRKSURF, cx, cy)
            (hr.found && abs(hr.lateral) < parse(Float64, get(ENV, "JM_OVERROAD_LAT", "4.0"))) || continue
            dh = cz - hr.height
            dh > 2.5 || continue
            k = (t.tex, round(Int, hr.lapdist / 50) * 50)
            e = get!(cnt, k, [0.0, Inf, -Inf, 0.0]); e[1] += 1; e[2] = min(e[2], dh); e[3] = max(e[3], dh)
            a = 0.5*sqrt(sum(((t.p[2][i]-t.p[1][i])*(t.p[3][j]-t.p[1][j]) - (t.p[2][j]-t.p[1][j])*(t.p[3][i]-t.p[1][i]))^2 for (i,j) in ((1,2),(2,3),(1,3))))
            e[4] += a

            n1 = (t.p[2][1]-t.p[1][1], t.p[2][2]-t.p[1][2], t.p[2][3]-t.p[1][3]); n2 = (t.p[3][1]-t.p[1][1], t.p[3][2]-t.p[1][2], t.p[3][3]-t.p[1][3])
            nz = n1[1]*n2[2]-n1[2]*n2[1]; nn = sqrt((n1[2]*n2[3]-n1[3]*n2[2])^2 + (n1[3]*n2[1]-n1[1]*n2[3])^2 + nz^2)
            nn > 0 && abs(nz)/nn > 0.5 && (length(e) < 5 ? push!(e, a) : (e[5] += a))
        end
        for (k, e) in sort(collect(cnt), by = x -> x[1][2])
            println("  [overroad] s~", lpad(k[2], 6), "  ", rpad(k[1], 10), " tris ", Int(e[1]), "  area ", round(e[4], digits=1), " m2 (horizontal ", round(length(e) >= 5 ? e[5] : 0.0, digits=1), ")  height ", round(e[2], digits=1), "..", round(e[3], digits=1), " m")
        end
        flush(stdout); exit(0)
    end
    const CAR = DriveCar(MODEL, TRKSURF; terrain=TERRAIN)
    println(TERRAIN, "  ", TRKSURF)

    if get(ENV,"JM_LINEONROAD","")!=""

        step = parse(Float64, get(ENV,"JM_LINEONROAD_STEP","250"))
        halfb = parse(Float64, get(ENV,"JM_LINEONROAD_BUCKET","20"))

        near = Dict{Int,Float64}(); cnt = Dict{Int,Int}(); covers = Dict{Int,Bool}()
        for t in TRACKMESH.tris
            ROAD_TEX(lowercase(t.tex)) || continue
            cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
            cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
            hr = JRTrack.hat(TRKSURF, cx, cy)
            hr.found || continue
            b = round(Int, hr.lapdist/step)
            abs(hr.lapdist - b*step) <= halfb || continue
            a = abs(hr.lateral)
            near[b] = min(get(near, b, Inf), a)
            cnt[b] = get(cnt, b, 0) + 1
            if !get(covers, b, false)
                lo = Inf; hi = -Inf
                for vi in 1:3
                    hv = JRTrack.hat(TRKSURF, Float64(t.p[vi][1]), Float64(t.p[vi][2]))
                    hv.found || continue
                    lo = min(lo, hv.lateral); hi = max(hi, hv.lateral)
                end
                (isfinite(lo) && lo <= 0.0 <= hi) && (covers[b] = true)
            end
        end

        total = isempty(near) ? 0 : maximum(keys(near))

        local bad = 0
        local gap = 0
        println("== JM_LINEONROAD: nearest ROAD triangle to the centreline, per ", step, " m ==")
        for b in 0:total
            if !haskey(near, b)
                println("   s=", rpad(b*step,8), "NO ROAD TRIANGLES within ±", halfb, " m  *** GAP ***")
                gap += 1
            elseif !get(covers, b, false) && near[b] > 3.0
                println("   s=", rpad(b*step,8), "no road triangle COVERS the line; nearest centroid ",
                        round(near[b],digits=1), " m off   (", cnt[b], " tris)  *** LINE OFF ROAD ***")
                bad += 1
            end
        end
        println("   --> ", total+1, " buckets: ", gap, " with no road, ", bad,
                " with the line >3 m off the road, ", total+1-gap-bad, " healthy")
        flush(stdout)
    end
    tstamp("geometry extraction begins"); print("extracting geometry… "); flush(stdout)

    const TRACKMAIN0 = Render.extract_classic_car(ZTRK; track=true, mirror=true,
                                              exclude = ((get(ENV, "JM_DRAW_WIREF", "1") == "0" ? ("ltraymap","lshad","wiref_s") : ("ltraymap","lshad"))...,
                                                         (String(x) for x in split(get(ENV, "JM_TRACK_EXCLUDE", ""), ",") if !isempty(x))...))

    railfam(tx) = (lt = lowercase(tx); startswith(lt,"armco") || startswith(lt,"fenc") || startswith(lt,"stfce") ||
                   startswith(lt,"sarmc") || startswith(lt,"yarmc") || startswith(lt,"gd_rail") || startswith(lt,"rail") ||
                   (GROOVE_DEDUP && startswith(lt,"groove")))

    if get(ENV,"JM_RAILDIAG","") != ""
        key(v,t) = begin
            cx=(v[t]+v[t+11]+v[t+22])/3; cy=(v[t+1]+v[t+12]+v[t+23])/3; cz=(v[t+2]+v[t+13]+v[t+24])/3
            ux=v[t+11]-v[t]; uy=v[t+12]-v[t+1]; uz=v[t+13]-v[t+2]
            wx=v[t+22]-v[t]; wy=v[t+23]-v[t+1]; wz=v[t+24]-v[t+2]
            a=0.5*sqrt((uy*wz-uz*wy)^2+(uz*wx-ux*wz)^2+(ux*wy-uy*wx)^2)
            (round(Int,cx*50), round(Int,cy*50), round(Int,cz*50), round(Int,a*100))
        end

        local tot=0; local perpart=0; local gdup=0; local nparts=0
        glob=Set{NTuple{4,Int}}()
        for part in TRACKMAIN0
            railfam(part.tex) || continue
            nparts += 1
            local v = part.verts; seen = Set{NTuple{4,Int}}()
            for t in 1:33:length(v)-32
                tot += 1; k=key(v,t)
                k in seen ? (perpart += 1) : push!(seen,k)
                k in glob  ? (gdup   += 1) : push!(glob,k)
            end
        end
        println("== JM_RAILDIAG rail/fence triangles ==")
        println("   parts=", nparts, "  triangles=", tot)
        println("   duplicates found PER-PART (what the shipped dedup can see): ", perpart,
                tot>0 ? string("  (", round(100perpart/tot,digits=1), "%)") : "")
        println("   duplicates found GLOBALLY (across parts too):               ", gdup,
                tot>0 ? string("  (", round(100gdup/tot,digits=1), "%)") : "")
        flush(stdout)
    end
    GROOVE_DEDUP = get(ENV, "JM_GROOVE_DEDUP", "1") != "0"
    const TRACKMAIN = get(ENV,"JM_RAIL_DEDUP","1") == "0" ? TRACKMAIN0 : begin
        ndrop = Ref(0)
        out = map(TRACKMAIN0) do part
            railfam(part.tex) || return part
            v = part.verts; keep = Float32[]; seen = Set{NTuple{4,Int}}()
            for t in 1:33:length(v)-32
                cx=(v[t]+v[t+11]+v[t+22])/3; cy=(v[t+1]+v[t+12]+v[t+23])/3; cz=(v[t+2]+v[t+13]+v[t+24])/3
                ux=v[t+11]-v[t]; uy=v[t+12]-v[t+1]; uz=v[t+13]-v[t+2]
                wx=v[t+22]-v[t]; wy=v[t+23]-v[t+1]; wz=v[t+24]-v[t+2]
                a=0.5*sqrt((uy*wz-uz*wy)^2+(uz*wx-ux*wz)^2+(ux*wy-uy*wx)^2)

                q = startswith(lowercase(part.tex), "groove") ? (5, 10) : (50, 100)
                k=(round(Int,cx*q[1]), round(Int,cy*q[1]), round(Int,cz*q[1]), round(Int,a*q[2]))
                if k in seen; ndrop[] += 1; else; push!(seen,k); append!(keep, @view v[t:t+32]); end
            end
            Render.TrackPart(keep, part.tex, part.col)
        end
        ndrop[] > 0 && println("  E68 S10: rail/fence dedup dropped ", ndrop[], " coplanar-duplicate tris")
        out
    end

    const ROADTESS = get(ENV, "JM_ROADTESS", "0") != "0"
    function roadtess_parts()
        (ROADHAT === TERRAIN0) && (println("  ROADTESS: no road-only HAT on this track -- off"); return Render.TrackPart[])

        asp = [pt for pt in TRACKMAIN if occursin("asp", lowercase(pt.tex))]
        isempty(asp) && (asp = [pt for pt in TRACKMAIN if ROAD_TEX(lowercase(pt.tex))])
        isempty(asp) && return Render.TrackPart[]
        big = asp[argmax([length(pt.verts) for pt in asp])]
        tex = big.tex
        vv = big.verts; nv = length(vv) ÷ 11
        col = (sum(vv[11k+7] for k in 0:nv-1) / nv, sum(vv[11k+8] for k in 0:nv-1) / nv, sum(vv[11k+9] for k in 0:nv-1) / nv)

        dens = Float64[]
        for k in 0:3:nv-3
            a = 11k; b = 11(k+1)
            dp = hypot(vv[a+1] - vv[b+1], vv[a+2] - vv[b+2], vv[a+3] - vv[b+3]); du = hypot(vv[a+10] - vv[b+10], vv[a+11] - vv[b+11])
            (dp > 0.5 && du > 1e-3) && push!(dens, dp / du)
        end
        tile = parse(Float64, get(ENV, "JM_ROADTESS_TILE", string(isempty(dens) ? 8.0 : sort(dens)[length(dens) ÷ 2 + 1])))
        step = parse(Float64, get(ENV, "JM_ROADTESS_STEP", "0.25")); maxw = 14.0
        println("  ROADTESS: asphalt part ", tex, " colour ", round.(col, digits = 2), " tile ", round(tile, digits = 1), " m (", length(dens), " edges)")
        n = length(TRKSURF.pos)
        edge = Vector{Union{Nothing,NTuple{2,Float64}}}(nothing, n)
        tarmac(x, z) = JRTrack.hat3d(ROADHAT, x, z; ref = Inf)[3]
        for i in 1:n
            p = TRKSURF.pos[i]; q = TRKSURF.perp[i]
            px, pz = Float64(p[1]), Float64(p[3]); qx, qz = Float64(q[1]), Float64(q[3])
            nq = hypot(qx, qz); nq < 1e-6 && continue; qx /= nq; qz /= nq
            tarmac(px, pz) || continue
            lo = 0.0; t = step
            while t <= maxw && tarmac(px - qx*t, pz - qz*t); lo = t; t += step; end
            hi = 0.0; t = step
            while t <= maxw && tarmac(px + qx*t, pz + qz*t); hi = t; t += step; end
            edge[i] = (-lo, hi)
        end
        v = Float32[]; nquad = 0
        hgt(x, z, fb) = (h = JRTrack.hat3d(ROADHAT, x, z; ref = Inf); h[3] ? Float64(h[1]) : fb)

        nrm = (sum(vv[11k+4] for k in 0:nv-1) / nv, sum(vv[11k+5] for k in 0:nv-1) / nv, sum(vv[11k+6] for k in 0:nv-1) / nv)
        let n = hypot(nrm[1], nrm[2], nrm[3]); n > 1f-6 && (nrm = (nrm[1]/n, nrm[2]/n, nrm[3]/n)); end
        if get(ENV, "JM_ROADTESS_DIAG", "") != ""
            println("  ROADTESS S3e: source normal ", round.(Float64.(nrm), digits = 3), " (was hardcoded 0,1,0)")
        end
        corner!(x, y, z, u, vv_) = append!(v, Float32[x, y, -z, nrm[1], nrm[2], nrm[3], col[1], col[2], col[3], u, vv_])
        for i in 1:n
            j = i == n ? 1 : i + 1
            (edge[i] === nothing || edge[j] === nothing) && continue
            si = Float64(TRKSURF.lapdist[i]); sj = j == 1 ? Float64(TRKSURF.lap_length) : Float64(TRKSURF.lapdist[j])
            (sj - si) > 20 && continue
            pts = Tuple{Float64,Float64,Float64,Float64,Float64}[]
            for (k, l, s_) in ((i, edge[i][1], si), (i, edge[i][2], si), (j, edge[j][2], sj), (j, edge[j][1], sj))
                p = TRKSURF.pos[k]; q = TRKSURF.perp[k]; nq = hypot(Float64(q[1]), Float64(q[3]))
                x = Float64(p[1]) + Float64(q[1]) / nq * l; z = Float64(p[3]) + Float64(q[3]) / nq * l
                push!(pts, (x, hgt(x, z, Float64(p[2])) + 0.02, z, l / tile, s_ / tile))
            end
            for (a, b, d) in ((1, 2, 3), (1, 3, 4))
                for k in (a, b, d); corner!(pts[k]...); end
            end
            nquad += 1
        end
        valid = count(!isnothing, edge)
        println("  ROADTESS: ", nquad, " quads from ", valid, "/", n, " waypoints, texture ", tex, ", mean width ",
                round(sum(e[2] - e[1] for e in edge if e !== nothing; init = 0.0) / max(valid, 1), digits = 1), " m")

        if get(ENV, "JM_ROADTESS_DIAG", "") != ""
            println("  ROADTESS S3c: part colour ", round.(Float64.(big.col), digits = 3),
                    " (was hardcoded 1,1,1); vertex colour ", round.(col, digits = 3))
        end
        isempty(v) ? Render.TrackPart[] : [Render.TrackPart(v, tex, big.col)]
    end
    const ROADPARTS = ROADTESS ? roadtess_parts() : Render.TrackPart[]

    _bdtrack = NURB && get(ENV, "JM_BACKDROP", "1") != "0"
    _bdtex(t) = (t = lowercase(t); t == "villone" || startswith(t, "casl"))
    const _TRACKMAIN_BD = _bdtrack ? filter(p -> !_bdtex(p.tex), TRACKMAIN) : TRACKMAIN
    const _SECPARTS_BD  = _bdtrack ? filter(p -> !_bdtex(p.tex), SECPARTS)  : SECPARTS
    _bdtrack && println("  [backdrop] dropped ", length(TRACKMAIN) - length(_TRACKMAIN_BD) + length(SECPARTS) - length(_SECPARTS_BD),
                        " backdrop part(s) from the track scenery (object copies keep the viewing-radius rule)")

    include(joinpath(@__DIR__, "holefill.jl")); using .HoleFill
    const HOLEFILL = get(ENV, "JM_HOLEFILL", "1") == "0" ? (Render.TrackPart[], 0) :
        HoleFill.holefill_parts([_TRACKMAIN_BD; ROADPARTS; _SECPARTS_BD], ROAD_TEX,
                                lt -> occursin(r"gr|grass|dirt|drt|edge|bank|earth|sand|gravel|field|meadow", lt) && !ROAD_TEX(lt);

                                near = parse(Float64, get(ENV, "JM_HOLEFILL_NEAR", "10")))
    println("  [holefill] ", HOLEFILL[2], " ground quad(s) of 2 m in ", length(HOLEFILL[1]), " part(s) where the mesh leaves the ground ",
            "beside the tarmac uncovered (JM_HOLEFILL=0 = off)")
    if haskey(ENV, "JM_HOLEFILL_DIAG")
        let a = parse.(Float64, split(ENV["JM_HOLEFILL_DIAG"], ",")), n = 0
            for p in HOLEFILL[1], k in 1:11:length(p.verts)-10
                hypot(p.verts[k] - a[1], p.verts[k+2] - a[2]) < a[3] || continue
                (n += 1) <= 12 && println("  [holefill@] ", p.tex, " (", round(p.verts[k], digits = 1), ", ", round(p.verts[k+1], digits = 2), ", ",
                                          round(p.verts[k+2], digits = 1), ") col ", round.(p.verts[k+6:k+8], digits = 2), " uv ", round.(p.verts[k+9:k+10], digits = 2))
            end
            println("  [holefill@] ", n, " corners within ", a[3], " m")
            for p in _TRACKMAIN_BD[1:min(3, end)]; println("  [track part] ", p.tex, " first vertex ", round.(p.verts[1:11], digits = 2)); end
        end
    end
    const TRACK = [_TRACKMAIN_BD; ROADPARTS; HOLEFILL[1]; _SECPARTS_BD]
    const SEC_FROM = length(_TRACKMAIN_BD) + length(ROADPARTS) + length(HOLEFILL[1]) + 1

    _cull_tex = Set(lowercase.(filter(!isempty, split(get(ENV, "JM_TRACK_CULL_TEX", ""), ","))))
    const TRACK_RAILCULL = Bool[(railfam(p.tex) && !startswith(lowercase(p.tex), "groove")) || lowercase(p.tex) in _cull_tex for p in TRACK]
    const BOARDCULL_MIN = parse(Float64, get(ENV, "JM_BOARDCULL_MIN", "0.2"))
    const BOARDCULL_SELF = get(ENV, "JM_BOARDCULL_SELF", "0") == "1"

    function _track_boardcull(TRACK, rc0, SEC_FROM)
        rc = falses(length(TRACK))

        if get(ENV, "JM_BOARDCULL", "1") != "0"
            cen(v, k) = ((v[k] + v[k+11] + v[k+22])/3, (v[k+1] + v[k+12] + v[k+23])/3, (v[k+2] + v[k+13] + v[k+24])/3)
            function nrm(v, k)
                ax, ay, az = v[k+11] - v[k], v[k+12] - v[k+1], v[k+13] - v[k+2]; bx, by, bz = v[k+22] - v[k], v[k+23] - v[k+1], v[k+24] - v[k+2]
                nx, ny, nz = ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx; l = sqrt(nx^2 + ny^2 + nz^2)
                l < 1e-9 ? (0.0, 0.0, 0.0, 0.0) : (nx/l, ny/l, nz/l, l/2)
            end
            key(c) = (round(Int, c[1]/0.03), round(Int, c[2]/0.03), round(Int, c[3]/0.03))
            grid = Dict{NTuple{3,Int},Vector{Tuple{Int,NTuple{3,Float64}}}}()
            for (pi, p) in enumerate(TRACK), k in 1:33:length(p.verts)-32
                n = nrm(p.verts, k); n[4] > 0 || continue
                push!(get!(grid, key(cen(p.verts, k)), Tuple{Int,NTuple{3,Float64}}[]), (pi, (n[1], n[2], n[3])))
            end
            nb = 0
            for (pi, p) in enumerate(TRACK)
                (rc0[pi] || pi >= SEC_FROM) && continue
                tot = 0.0; pa = 0.0; ps = 0.0
                for k in 1:33:length(p.verts)-32
                    n = nrm(p.verts, k); n[4] > 0 || continue
                    tot += n[4]; c = key(cen(p.verts, k)); hit = false; hs = false
                    for dx in -1:1, dy in -1:1, dz in -1:1
                        for (qi, m) in get(grid, (c[1]+dx, c[2]+dy, c[3]+dz), ())
                            n[1]*m[1] + n[2]*m[2] + n[3]*m[3] < -0.98 || continue
                            qi != pi ? (hit = true; break) : (hs = true)
                        end
                        hit && break
                    end
                    hit && (pa += n[4]); (hs && !hit) && (ps += n[4])
                end

                BOARDCULL_SELF && (pa += ps)
                (haskey(ENV, "JM_BOARDCULL_DIAG") && tot > 0 && (pa + ps)/tot > 0.02) && println("  [boardcull] ", rpad(p.tex, 14), " back-to-back ", round(pa/tot, digits = 2), " self ", round(ps/tot, digits = 2), "  area ", round(tot, digits = 1), " m2")
                (tot > 0 && pa/tot >= BOARDCULL_MIN) && (rc[pi] = true; nb += 1)
            end
            println("  [boardcull] ", nb, " track parts back-to-back with another part -> single-sided (JM_BOARDCULL=0 = both faces)")

            if haskey(ENV, "JM_ZFIGHT_DIAG")
                zp = Dict{Tuple{String,String},Float64}()
                for (pi, p) in enumerate(TRACK), k in 1:33:length(p.verts)-32
                    n = nrm(p.verts, k); n[4] > 0 || continue; c = key(cen(p.verts, k))
                    for (qi, m) in get(grid, c, ())
                        (qi > pi && n[1]*m[1] + n[2]*m[2] + n[3]*m[3] > 0.98) || continue
                        a, b = sort([p.tex, TRACK[qi].tex]); zp[(a, b)] = get(zp, (a, b), 0.0) + n[4]
                    end
                end
                println("  [zfight] ", length(zp), " part pairs share same-facing coplanar triangles; largest:")
                for (k, v) in sort(collect(zp), by = x -> -x[2])[1:min(end, 15)]; println("     ", rpad(k[1], 14), " + ", rpad(k[2], 14), round(v, digits = 1), " m2"); end
            end
        end
        rc
    end
    const TRACK_BOARDCULL = _track_boardcull(TRACK, TRACK_RAILCULL, SEC_FROM)

    if haskey(ENV, "JM_TRIDUMP_AT")
        let a = split(ENV["JM_TRIDUMP_AT"], ","), tx = lowercase(a[1]), (x0, z0, r0) = parse.(Float64, a[2:4])
            println("  [tri] dump ", tx, " near (", x0, ", ", z0, ") r ", r0, " over ", count(p -> lowercase(p.tex) == tx, TRACK), " parts; track ", sum(length(p.verts) for p in TRACK) ÷ 33, " triangles")
            tx == "__parts__" && for p in sort(TRACK, by = p -> -length(p.verts)); println("  [tri] part ", rpad(p.tex, 14), length(p.verts) ÷ 33); end
            for (pi, p) in enumerate(TRACK)
                lowercase(p.tex) == tx || continue
                v = p.verts
                for k in 1:33:length(v)-32
                    cx = (v[k] + v[k+11] + v[k+22])/3; cz = (v[k+2] + v[k+13] + v[k+24])/3
                    hypot(cx - x0, cz - z0) <= r0 || continue
                    ax, ay, az = v[k+11]-v[k], v[k+12]-v[k+1], v[k+13]-v[k+2]; bx, by, bz = v[k+22]-v[k], v[k+23]-v[k+1], v[k+24]-v[k+2]
                    n = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx); l = sqrt(sum(abs2, n)); l < 1e-9 && continue
                    println("  [tri] part ", pi, " #", (k-1)÷33, " n ", round.(n ./ l, digits = 2), " area ", round(l/2, digits = 1),
                            " v ", join((string(round.((v[j], v[j+1], v[j+2]), digits = 2), " uv ", round.((v[j+9], v[j+10]), digits = 2)) for j in (k, k+11, k+22)), " | "),
                            " vn ", round.((v[k+3], v[k+4], v[k+5]), digits = 2))
                end
            end
        end
        exit(0)
    end
end

if !SKIDPAD && haskey(ENV, "JM_SHARP")
    let A = ALIGNED, n = length(A), sa = zeros(n), rows = Tuple{Float64,Float64}[]
        for k in 2:n; sa[k] = sa[k-1] + hypot(A[k][1]-A[k-1][1], A[k][2]-A[k-1][2]); end
        for k in 1:n
            i = k; j = k
            while sa[k] - sa[mod1(i, n)] < 8.0 && i > k - n ÷ 2; i -= 1; end
            while sa[mod1(j, n)] - sa[k] < 8.0 && j < k + n ÷ 2; j += 1; end
            (i < 1 || j > n) && continue
            a_ = A[i]; b_ = A[k]; c_ = A[j]
            ab = hypot(b_[1]-a_[1], b_[2]-a_[2]); bc = hypot(c_[1]-b_[1], c_[2]-b_[2]); ca = hypot(a_[1]-c_[1], a_[2]-c_[2])
            ar = abs((b_[1]-a_[1])*(c_[2]-a_[2]) - (b_[2]-a_[2])*(c_[1]-a_[1]))/2
            ar > 1e-6 && push!(rows, (sa[k], ab*bc*ca/(4ar)))
        end
        sort!(rows, by = r -> r[2]); picked = Tuple{Float64,Float64}[]
        for r in rows; all(p -> abs(p[1] - r[1]) > 150, picked) && push!(picked, r); length(picked) >= parse(Int, ENV["JM_SHARP"]) && break; end
        println("  [sharp] ", TRACKSEL, " sharpest corners (lapdist, radius m): ", [(round(Int, p[1]), round(p[2], digits = 1)) for p in picked])
    end
end

const CLASSICW_ON   = !SKIDPAD
const CLASSICW_PHYS = CLASSICW_ON && get(ENV, "JM_CLASSICWALL", "1") != "0"
const CLASSICWALLS  = CLASSICW_ON ? CLASSICWall.read_walls(read(track_file(CLASSICNAME, ".trk"))) : nothing
const CLASSICREF    = CLASSICW_ON ? rigid_refit(ALIGNED0, ALIGNED) : NTuple{2,Float64}[]
const CLASSICREF_SUB = CLASSICW_ON ? length(CLASSICREF) ÷ length(CLASSICWALLS.secs) : 1
CLASSICW_ON && length(CLASSICREF) != CLASSICREF_SUB*length(CLASSICWALLS.secs) &&
    error("CLASSICWALL-1: reference line has $(length(CLASSICREF)) points for $(length(CLASSICWALLS.secs)) .trk sections -- not a whole number per section")
const CLASSICW_SGN  = parse(Float64, get(ENV, "JM_CLASSICWALL_SGN", "1"))
const CLASSICW_HMIN = parse(Float64, get(ENV, "JM_CLASSICWALL_HMIN", "0.25"))
const CLASSICW_HINT = Ref(0)
"""Locate world (x, z) on Classic's reference line: (section, fraction, Classic lateral, unit vector of +Classic lateral, d2).
Searches +-60 nodes around the last hit (a circuit that passes close to itself must not jump stretches), falling
back to the whole line when the hint is unset or the nearest node is implausibly far."""
function classicw_locate(x, z; hint::Int = CLASSICW_HINT[], update::Bool = false)
    A = CLASSICREF; n = length(A)
    best = Inf; bk = 1; bu = 0.0
    rng = hint == 0 ? (1:n) : ((hint - 60):(hint + 60))
    @inbounds for kk in rng
        k = mod1(kk, n); j = k == n ? 1 : k + 1
        ax, az = A[k]; tx = A[j][1] - ax; tz = A[j][2] - az; L2 = tx*tx + tz*tz
        u = L2 > 0 ? clamp(((x - ax)*tx + (z - az)*tz)/L2, 0.0, 1.0) : 0.0
        dx = x - (ax + u*tx); dz = z - (az + u*tz); d2 = dx*dx + dz*dz
        d2 < best && (best = d2; bk = k; bu = u)
    end
    hint != 0 && best > 400.0^2 && return classicw_locate(x, z; hint = 0, update = update)
    update && (CLASSICW_HINT[] = bk)

    j = bk == n ? 1 : bk + 1
    tx = A[j][1] - A[bk][1]; tz = A[j][2] - A[bk][2]; tl = max(hypot(tx, tz), 1e-9)
    lx = -tz/tl; lz = tx/tl
    fx = A[bk][1] + bu*tx; fz = A[bk][2] + bu*tz
    lat = CLASSICW_SGN*((x - fx)*lx + (z - fz)*lz)
    sec = (bk - 1) ÷ CLASSICREF_SUB + 1
    f = (((bk - 1) % CLASSICREF_SUB) + bu) / CLASSICREF_SUB
    (sec, f, lat, CLASSICW_SGN*lx, CLASSICW_SGN*lz, best, bk)
end

const CLASSICW_REG = parse(Float64, get(ENV, "JM_CLASSICWALL_REG", "2.0"))
"plan-view grid of near-vertical track-mesh faces at least `hmin` tall: cell => (ax, az, bx, bz)"
function classicw_vertical_grid(tris; C = 4.0, hmin = CLASSICW_HMIN)
    grid = Dict{Tuple{Int,Int},Vector{NTuple{4,Float64}}}()
    for t in tris
        p1, p2, p3 = t.p
        ux, uy, uz = p2[1]-p1[1], p2[2]-p1[2], p2[3]-p1[3]; vx, vy, vz = p3[1]-p1[1], p3[2]-p1[2], p3[3]-p1[3]
        cx, cy, cz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx; nl = sqrt(cx^2 + cy^2 + cz^2)
        (nl < 1e-9 || abs(cz)/nl > 0.3) && continue
        max(p1[3], p2[3], p3[3]) - min(p1[3], p2[3], p3[3]) < hmin && continue
        P = ((p1[1], p1[2]), (p2[1], p2[2]), (p3[1], p3[2]))
        d(a, b) = hypot(a[1]-b[1], a[2]-b[2])
        (a, b) = d(P[1], P[2]) >= max(d(P[1], P[3]), d(P[2], P[3])) ? (P[1], P[2]) :
                 d(P[1], P[3]) >= d(P[2], P[3]) ? (P[1], P[3]) : (P[2], P[3])
        seg = (Float64(a[1]), Float64(a[2]), Float64(b[1]), Float64(b[2]))
        for i in floor(Int, min(a[1], b[1])/C):floor(Int, max(a[1], b[1])/C), j in floor(Int, min(a[2], b[2])/C):floor(Int, max(a[2], b[2])/C)
            push!(get!(grid, (i, j), NTuple{4,Float64}[]), seg)
        end
    end
    grid
end
"base point and +lateral unit vector of Classic's line at (section, fraction)"
function classicw_frame(sec, f)
    A = CLASSICREF; n = length(A)
    t = (sec - 1)*CLASSICREF_SUB + f*CLASSICREF_SUB; k = clamp(floor(Int, t) + 1, 1, n); u = t - (k - 1)
    j = k == n ? 1 : k + 1
    tx = A[j][1] - A[k][1]; tz = A[j][2] - A[k][2]; tl = max(hypot(tx, tz), 1e-9)
    (A[k][1] + u*tx, A[k][2] + u*tz, CLASSICW_SGN*(-tz/tl), CLASSICW_SGN*(tx/tl))
end
"lateral t in [e-w, e+w] nearest e where the line P + n t crosses a drawn vertical face (NaN if none)"
function classicw_hit(grid, px, pz, nx, nz, e, w; C = 4.0)
    best = NaN; bd = Inf; seen = Set{NTuple{4,Float64}}()
    for t in (e - w):C:(e + w + C)
        x = px + nx*min(t, e + w); z = pz + nz*min(t, e + w)
        for i in floor(Int, x/C)-1:floor(Int, x/C)+1, j in floor(Int, z/C)-1:floor(Int, z/C)+1, sg in get(grid, (i, j), NTuple{4,Float64}[])
            sg in seen && continue; push!(seen, sg)
            (ax, az, bx, bz) = sg; ex = bx - ax; ez = bz - az
            den = nx*ez - nz*ex; abs(den) < 1e-9 && continue
            tt = ((ax - px)*ez - (az - pz)*ex)/den
            ss = ((ax - px)*nz - (az - pz)*nx)/den
            (0.0 <= ss <= 1.0 && e - w <= tt <= e + w) || continue
            abs(tt - e) < bd && (bd = abs(tt - e); best = tt)
        end
    end
    best
end
function classicw_register!(W, grid; w = CLASSICW_REG)
    nface = 0; nreg = 0; shifts = Float64[]
    for (si, v) in enumerate(W.secs)
        E = W.E[si]; E0 = copy(E)
        for k in 1:length(v)-1
            (v[k].wall && v[k].height >= CLASSICW_HMIN) || continue
            for kk in (k, k + 1), j in 1:CLASSICWall.NF+1
                f = (j - 1)/CLASSICWall.NF
                (px, pz, nx, nz) = classicw_frame(si, f)
                nface += 1
                t = classicw_hit(grid, px, pz, nx, nz, E0[kk, j], w)
                isnan(t) && continue
                E[kk, j] = t; nreg += 1; push!(shifts, t - E0[kk, j])
            end
        end

        for j in 1:CLASSICWall.NF+1
            all(E[k, j] <= E[k+1, j] + 1e-6 for k in 1:size(E, 1)-1) || (E[:, j] .= E0[:, j])
        end
    end
    (nface, nreg, shifts)
end
const CLASSICW_ROADREG_WIDE = parse(Float64, get(ENV, "JM_CLASSICWALL_ROADREG_WIDE", "3.0"))
if CLASSICW_ON && (get(ENV, "JM_CLASSICWALL_REG", "2.0") != "0" || get(ENV, "JM_CLASSICWALL_CENSUS", "0") != "0")
    let grid = classicw_vertical_grid(TRACKMESH.tris)
        function census(label)
            ds = Float64[]
            for (si, v) in enumerate(CLASSICWALLS.secs), j in 1:CLASSICWall.NF+1, k in 1:length(v)-1
                (v[k].wall && v[k].height >= CLASSICW_HMIN) || continue
                f = (j - 1)/CLASSICWall.NF; (px, pz, nx, nz) = classicw_frame(si, f)
                for kk in (k, k + 1)
                    e = CLASSICWALLS.E[si][kk, j]; t = classicw_hit(grid, px, pz, nx, nz, e, 6.0)
                    push!(ds, isnan(t) ? Inf : abs(t - e))
                end
            end
            u = sort(ds); m = length(u)
            println("  [classicwall census] ", TRACKSEL, " ", label, ": ", m, " wall faces; drawn face within 0.25 m ",
                    round(100*count(<=(0.25), u)/m, digits=1), "%, 1 m ", round(100*count(<=(1.0), u)/m, digits=1),
                    "%, 2 m ", round(100*count(<=(2.0), u)/m, digits=1), "%; none within 6 m ", round(100*count(isinf, u)/m, digits=1), "%")
        end
        CEN = get(ENV, "JM_CLASSICWALL_CENSUS", "0") != "0"
        CEN && census("as placed")

        if ROADHAT !== TERRAIN0 && get(ENV, "JM_CLASSICWALL_ROADREG", "1") != "0"
            NFp = CLASSICWall.NF + 1; ns = length(CLASSICWALLS.secs); shift = fill(NaN, ns*NFp)
            onr(x, z) = JRTrack.hat3d(ROADHAT, x, z; ref = Inf)[3]
            for si in 1:ns, j in 1:NFp
                f = (j - 1)/CLASSICWall.NF; v = CLASSICWALLS.secs[si]
                a0 = NaN; b0 = NaN
                for k in 1:length(v)-1
                    (v[k].typ & 0x7ff) == 1 || continue
                    ea = CLASSICWALLS.E[si][k, j]; eb = CLASSICWALLS.E[si][k+1, j]
                    (isnan(a0) || eb - ea > b0 - a0) && (a0 = ea; b0 = eb)
                end
                isnan(a0) && continue
                c = (a0 + b0)/2; w = b0 - a0
                (px, pz, nx, nz) = classicw_frame(si, f)

                best = NaN; bd = Inf; run0 = NaN
                for t in (c - 12.0):0.25:(c + 12.25)
                    on = t <= c + 12.0 && onr(px + nx*t, pz + nz*t)
                    if on && isnan(run0)
                        run0 = t
                    elseif !on && !isnan(run0)
                        rc = (run0 + t - 0.25)/2; rw = t - 0.25 - run0

                        (rw >= 0.5w && rw <= w + CLASSICW_ROADREG_WIDE && abs(rc - c) < bd) && (bd = abs(rc - c); best = rc)
                        run0 = NaN
                    end
                end
                (!isnan(best) && bd < 6.0) && (shift[(si - 1)*NFp + j] = best - c)
            end

            n = length(shift); sm = zeros(n); got = 0
            for i in 1:n
                win = [shift[mod1(i + d, n)] for d in -6:6 if !isnan(shift[mod1(i + d, n)])]
                isempty(win) && continue
                sm[i] = sort(win)[(length(win) + 1) ÷ 2]; got += 1
            end
            for si in 1:ns, j in 1:NFp
                CLASSICWALLS.E[si][:, j] .+= sm[(si - 1)*NFp + j]
            end
            a_ = sort(abs.(sm))
            println("  CLASSICWALL-1: ", TRACKSEL, " Classic road registered to the drawn road at ", got, " of ", n, " points; |shift| p50 ",
                    round(a_[end ÷ 2 + 1], digits = 2), " m, p90 ", round(a_[max(1, 9n ÷ 10)], digits = 2), " m, max ", round(a_[end], digits = 2), " m")
        end
        if get(ENV, "JM_CLASSICWALL_REG", "2.0") != "0"
            (nf, nr, sh) = classicw_register!(CLASSICWALLS, grid)
            a = sort(abs.(sh))
            println("  CLASSICWALL-1: ", TRACKSEL, " walls registered to the drawn faces: ", nr, " of ", nf, " face points (",
                    round(100*nr/max(nf, 1), digits=1), "%), |shift| p50 ", isempty(a) ? 0 : round(a[end÷2+1], digits=2),
                    " m, p90 ", isempty(a) ? 0 : round(a[max(1, 9*length(a)÷10)], digits=2), " m")
            CEN && census("registered")
        end
        CEN && exit(0)
    end
end

if CLASSICW_ON && ROADHAT !== TERRAIN0 && get(ENV, "JM_CLASSICW_ONROAD", "0") != "0"
    let onr(x, z) = JRTrack.hat3d(ROADHAT, x, z; ref = Inf)[3], sites = Tuple{Float64,Int,Float64,Float64}[], s0 = 0.0, nf = 0
        for (si, v) in enumerate(CLASSICWALLS.secs)
            L = CLASSICWALLS.seclen[si]
            for j in 1:CLASSICWall.NF+1, k in 1:length(v)-1
                (v[k].wall && v[k].height >= CLASSICW_HMIN) || continue
                f = (j - 1)/CLASSICWall.NF; (px, pz, nx, nz) = classicw_frame(si, f)
                for kk in (k, k + 1)
                    e = CLASSICWALLS.E[si][kk, j]; nf += 1
                    (onr(px + nx*(e - 0.5), pz + nz*(e - 0.5)) && onr(px + nx*(e + 0.5), pz + nz*(e + 0.5))) &&
                        push!(sites, (s0 + f*L, si, e, v[k].height))
                end
            end
            s0 += L
        end
        println("  [classicw onroad] ", TRACKSEL, ": ", length(sites), " of ", nf, " wall-face points stand on drawn road")
        for (sl, si, e, h) in sites[1:min(end, 25)]
            println("     s=", round(sl, digits = 1), "  section ", si, "  face at lat ", round(e, digits = 2), "  h ", round(h, digits = 2))
        end
    end
end

if CLASSICW_ON && get(ENV, "JM_CLINE_FREE", "0") != "0"
    let A = ALIGNED, n = length(A), s = 0.0, bad = Tuple{Float64,Float64,Float64,Float64}[], last = -1e9, nchk = 0
        CLASSICW_HINT[] = 0
        for k in 1:n
            k > 1 && (s += hypot(A[k][1] - A[k-1][1], A[k][2] - A[k-1][2]))
            s - last < 2.0 && continue
            last = s; nchk += 1
            (sec, f, lat, _, _, _, _) = classicw_locate(A[k][1], A[k][2]; update = true)
            (lo, hi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, sec, f, lat, lat; hmin = CLASSICW_HMIN)
            (hi - lo < 4.0 || lat < lo || lat > hi) && push!(bad, (s, lat, lo, hi))
        end
        CLASSICW_HINT[] = 0
        println("  [clinefree] ", TRACKSEL, ": ", length(bad), " of ", nchk, " centreline samples with < 4 m free or outside Classic's walls")
        i = 1
        while i <= length(bad)
            j = i; while j < length(bad) && bad[j+1][1] - bad[j][1] < 6.0; j += 1; end
            w = minimum(b[4] - b[3] for b in bad[i:j])
            println("     s ", round(Int, bad[i][1]), "-", round(Int, bad[j][1]), "  (", j - i + 1, " samples)  narrowest free ",
                    round(w, digits = 2), " m  e.g. lat ", round(bad[i][2], digits = 2), " in [", round(bad[i][3], digits = 2), ", ",
                    round(bad[i][4], digits = 2), "]")
            i = j + 1
        end
    end
end

if CLASSICW_ON && get(ENV, "JM_CLASSICW_DUMP", "") != ""
    for si in parse.(Int, split(ENV["JM_CLASSICW_DUMP"], ","))
        local v = CLASSICWALLS.secs[si]; E = CLASSICWALLS.E[si]
        println("  [classicw dump] section ", si, " (", round(CLASSICWALLS.seclen[si], digits = 1), " m)")
        for k in eachindex(v)
            println("     ", lpad(k, 2), " raw ", lpad(round(v[k].l0, digits = 2), 8), " -> ", lpad(round(v[k].l1, digits = 2), 8),
                    "   E f0 ", lpad(round(E[k, 1], digits = 2), 8), "  f.5 ", lpad(round(E[k, 6], digits = 2), 8), "  f1 ", lpad(round(E[k, end], digits = 2), 8),
                    "  0x", string(v[k].typ, base = 16), v[k].wall ? " RAISED h=$(round(v[k].height, digits = 2))" : "")
        end
        println("     free at f=.5 near lat 0: ", round.(CLASSICWall.free_interval(CLASSICWALLS, si, 0.5, 0.0, 0.0; hmin = CLASSICW_HMIN)[1:2], digits = 2),
                "   (hmin ", CLASSICW_HMIN, ")")
    end
end

if CLASSICW_ON && get(ENV, "JM_CLASSICWALL_SHOW", "0") != "0"
    let v = Float32[], col = (1f0, 0f0, 1f0)
        pv(x, y, z) = append!(v, Float32[x, y, -z, 0f0, 1f0, 0f0, col[1], col[2], col[3], 0f0, 0f0])
        gh(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        for (si, w) in enumerate(CLASSICWALLS.secs), k in 1:length(w)-1
            (w[k].wall && w[k].height >= CLASSICW_HMIN) || continue
            hgt = clamp(w[k].height, 0.3, 2.5)
            for kk in (k, k + 1), j in 1:CLASSICWall.NF
                f0 = (j - 1)/CLASSICWall.NF; f1 = j/CLASSICWall.NF
                (p0x, p0z, n0x, n0z) = classicw_frame(si, f0); (p1x, p1z, n1x, n1z) = classicw_frame(si, f1)
                e0 = CLASSICWALLS.E[si][kk, j]; e1 = CLASSICWALLS.E[si][kk, j + 1]
                ax = p0x + n0x*e0; az = p0z + n0z*e0; bx = p1x + n1x*e1; bz = p1z + n1z*e1
                ha = gh(ax, az); hb = gh(bx, bz); (isnan(ha) || isnan(hb)) && continue
                for (q1, q2, q3) in (((ax, ha, az), (bx, hb, bz), (bx, hb + hgt, bz)), ((ax, ha, az), (bx, hb + hgt, bz), (ax, ha + hgt, az)),
                                     ((ax, ha, az), (bx, hb + hgt, bz), (bx, hb, bz)), ((ax, ha, az), (ax, ha + hgt, az), (bx, hb + hgt, bz)))
                    pv(q1...); pv(q2...); pv(q3...)
                end
            end
        end
        push!(TRACK, Render.TrackPart(v, "", col)); push!(TRACK_RAILCULL, false)
        println("  CLASSICWALL-1: JM_CLASSICWALL_SHOW overlay, ", length(v) ÷ 33, " triangles")
    end
end

const SPAFENCE = SPA && CLASSICW_ON ? [let f = split(x, ":"); (parse(Float64, f[1]), parse(Float64, f[2]), f[3] == "L" ? -1.0 : 1.0) end
                                   for x in split(get(ENV, "JM_SPAFENCE", "8170:8440:R"), ";"; keepempty = false)] : Tuple{Float64,Float64,Float64}[]
if !isempty(SPAFENCE)
    let v = Float32[], sacc = cumsum([0.0; CLASSICWALLS.seclen]), H = 1.6, ul = 0.0, nq = 0
        gh(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        function edge(si, f, side)
            w = CLASSICWALLS.secs[si]; bw = -1.0; mid = 0.0
            for k in 1:length(w)-1
                (w[k].typ & 0x7ff) == 1 || continue
                a = CLASSICWall.edge(CLASSICWALLS, si, k, f); b = CLASSICWall.edge(CLASSICWALLS, si, k + 1, f)
                b - a > bw && (bw = b - a; mid = (a + b)/2)
            end
            (lo, hi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, si, f, mid, mid; hmin = CLASSICW_HMIN)
            side > 0 ? hi : lo
        end
        pv(x, y, z, nx, nz, u, t) = append!(v, Float32[x, y, -z, nx, 0f0, -nz, 1f0, 1f0, 1f0, u, t])
        for (s0, s1, side) in SPAFENCE, si in eachindex(CLASSICWALLS.secs)
            (sacc[si+1] < s0 || sacc[si] > s1) && continue
            for j in 1:CLASSICWall.NF
                f0 = (j - 1)/CLASSICWall.NF; f1 = j/CLASSICWall.NF
                (sacc[si] + f0*CLASSICWALLS.seclen[si] < s0 || sacc[si] + f1*CLASSICWALLS.seclen[si] > s1) && continue
                e0 = edge(si, f0, side); e1 = edge(si, f1, side); (isfinite(e0) && isfinite(e1)) || continue
                (p0x, p0z, n0x, n0z) = classicw_frame(si, f0); (p1x, p1z, n1x, n1z) = classicw_frame(si, f1)
                ax = p0x + n0x*e0; az = p0z + n0z*e0; bx = p1x + n1x*e1; bz = p1z + n1z*e1
                ha = gh(ax, az); hb = gh(bx, bz); (isnan(ha) || isnan(hb)) && continue
                L = hypot(bx - ax, bz - az); u0 = ul/3.0; u1 = (ul + L)/3.0; ul += L
                nx = -side*n0x; nz = -side*n0z
                for (q1, q2, q3) in (((ax, ha, az, u0, 1.0), (bx, hb, bz, u1, 1.0), (bx, hb + H, bz, u1, 0.0)),
                                     ((ax, ha, az, u0, 1.0), (bx, hb + H, bz, u1, 0.0), (ax, ha + H, az, u0, 0.0)),
                                     ((ax, ha, az, u0, 1.0), (bx, hb + H, bz, u1, 0.0), (bx, hb, bz, u1, 1.0)),
                                     ((ax, ha, az, u0, 1.0), (ax, ha + H, az, u0, 0.0), (bx, hb + H, bz, u1, 0.0)))
                    pv(q1[1], q1[2], q1[3], nx, nz, q1[4], q1[5]); pv(q2[1], q2[2], q2[3], nx, nz, q2[4], q2[5]); pv(q3[1], q3[2], q3[3], nx, nz, q3[4], q3[5])
                end
                nq += 1
            end
        end
        if nq > 0
            push!(TRACK, Render.TrackPart(v, "wiref_m", (1f0, 1f0, 1f0))); push!(TRACK_RAILCULL, false)
        end
        println("  SPAFENCE-1: a wire fence on Classic's invisible wall -- ", nq, " panels, ", round(Int, ul), " m (JM_SPAFENCE=\"\" removes it)")
    end
end

const CLASSICW_PEN  = parse(Float64, get(ENV, "JM_CLASSICWALL_PEN", "0.3"))
const CLASSICW_PREV = Ref(NaN)
const CLASSICW_LAST = Ref((NaN, NaN))
const CLASSICW_STAT = Ref((hits = 0, clamps = 0, maxpen = 0.0))
const WRECK_NOCLAMP = get(ENV, "JM_WRECK_NOCLAMP", "1") != "0"
const CLASSICW_T = Ref((0.0, 0))
const CLASSICW_NEAR = Ref(true)
"a TELEPORT (respawn, recover, restart, wreck seal, shot placement): the next contact has no 'side it came from'"
classicw_teleported!() = (CLASSICW_PREV[] = NaN; CLASSICW_HINT[] = 0; CLASSICW_LAST[] = (NaN, NaN); OBS_PREV[] = (NaN, NaN); nothing)
"(interval faces, gap to each, normals) for the capsule at (x, z, θ) -- gap < 0 = penetration"
function classicwall_gaps(x, z, θ)
    lp = CLASSICW_LAST[]

    (isnan(lp[1]) || hypot(x - lp[1], z - lp[2]) > 60.0) && (CLASSICW_PREV[] = NaN; CLASSICW_HINT[] = 0)
    (sec, f, lat, gx, gz) = classicw_locate(x, z; update = true)
    CLASSICW_LAST[] = (x, z)
    ref = isnan(CLASSICW_PREV[]) ? lat : CLASSICW_PREV[]
    (lo, hi, hlo, hhi) = CLASSICWall.free_interval(CLASSICWALLS, sec, f, ref, ref; hmin = CLASSICW_HMIN)
    nsec = length(CLASSICWALLS.secs); L = max(CLASSICWALLS.seclen[sec], 1.0)
    tx = CLASSICW_SGN*gz; tz = -CLASSICW_SGN*gx
    ext = CARLF*abs(cos(θ)*tx + sin(θ)*tz) + CARW

    gend = Inf; nend = (0.0, 0.0)
    inside = !(lo <= ref <= hi)
    if inside && f*L < ext + 3.0
        psec = sec == 1 ? nsec : sec - 1
        (plo, phi) = CLASSICWall.free_interval(CLASSICWALLS, psec, 1.0, ref, ref; hmin = CLASSICW_HMIN)
        if plo <= ref <= phi
            lo, hi = plo, phi
            gend = -f*L - ext; nend = (-tx, -tz)
        end
    elseif !inside && (1 - f)*L < ext + 3.0
        qsec = sec == nsec ? 1 : sec + 1
        (qlo, qhi) = CLASSICWall.free_interval(CLASSICWALLS, qsec, 0.0, lat, lat; hmin = CLASSICW_HMIN)
        (qlo <= lat <= qhi) || (gend = (1 - f)*L - ext; nend = (-tx, -tz))
    elseif !inside && f*L < ext + 3.0
        psec = sec == 1 ? nsec : sec - 1
        (plo, phi) = CLASSICWall.free_interval(CLASSICWALLS, psec, 1.0, lat, lat; hmin = CLASSICW_HMIN)
        (plo <= lat <= phi) || (gend = f*L - ext; nend = (tx, tz))
    end

    reach0 = CARLF + CARW
    if (lat - lo) - reach0 > 1.5 && (hi - lat) - reach0 > 1.5 && !isfinite(gend)
        rl = CARLF*abs(cos(θ)*gx + sin(θ)*gz)
        return (lat, lo, hi, (lat - lo) - rl - CARW, (hi - lat) - rl - CARW, gx, gz, (gx, gz), (-gx, -gz), gend, nend)
    end

    δ = 0.02
    (loa, hia) = CLASSICWall.free_interval(CLASSICWALLS, sec, max(f - δ, 0.0), ref, ref; hmin = CLASSICW_HMIN)
    (lob, hib) = CLASSICWall.free_interval(CLASSICWALLS, sec, min(f + δ, 1.0), ref, ref; hmin = CLASSICW_HMIN)
    ds = (min(f + δ, 1.0) - max(f - δ, 0.0))*L
    slo = (isfinite(loa) && isfinite(lob) && abs(lob - lo) < 2.0 && abs(loa - lo) < 2.0) ? clamp((lob - loa)/ds, -3.0, 3.0) : 0.0
    shi = (isfinite(hia) && isfinite(hib) && abs(hib - hi) < 2.0 && abs(hia - hi) < 2.0) ? clamp((hib - hia)/ds, -3.0, 3.0) : 0.0
    klo = sqrt(1 + slo^2); khi = sqrt(1 + shi^2)
    nlo = ((gx - slo*tx)/klo, (gz - slo*tz)/klo)
    nhi = ((-gx + shi*tx)/khi, (-gz + shi*tz)/khi)
    rlo = CARLF*abs(cos(θ)*nlo[1] + sin(θ)*nlo[2]); rhi = CARLF*abs(cos(θ)*nhi[1] + sin(θ)*nhi[2])
    glo = (lat - lo)/klo - rlo - CARW
    ghi = (hi - lat)/khi - rhi - CARW
    (lat, lo, hi, glo, ghi, gx, gz, nlo, nhi, gend, nend)
end
function classicwall_contact(x, z, θ, dt)
    Fx = 0.0; Fy = 0.0; Mz = 0.0; peak = 0.0; closing = 0.0
    (lat, lo, hi, glo, ghi, gx, gz, nlo, nhi, gend, nend) = classicwall_gaps(x, z, θ)
    CLASSICW_NEAR[] = min(glo, ghi, gend) < 2.0
    CLASSICW_NEAR[] || ((lo < lat < hi) && (CLASSICW_PREV[] = lat))
    for (g, (nx, nz)) in ((glo, nlo), (ghi, nhi), (gend, nend))
        g >= 0.0 && continue
        vn = WVX[]*nx + WVZ[]*nz
        (fx, fy, mz) = DriveRT3D.contact_force(-g, nx, nz, vn, θ; kind = :wall, dt = dt)
        Fx += fx; Fy += fy; Mz += mz; peak = max(peak, hypot(fx, fy)); closing = max(closing, -vn)
    end
    (Fx, Fy, Mz, peak, peak, closing)
end
"After the step: never deeper than CLASSICW_PEN into any Classic wall face (sides, and the end faces across the track). Moves
the car back along each face's normal and removes the velocity INTO it, keeping the rest (it stops at the wall and slides
along it; in a closing wedge both faces take the forward speed and it stops)."
function classicwall_clamp!(c)
  local lat, lo, hi
  for it in 1:3
    (lat, lo, hi, glo, ghi, gx, gz, nlo, nhi, gend, nend) = classicwall_gaps(c.x, c.z, c.θ)
    if it == 1 && min(glo, ghi, gend) < 0.0
        CLASSICW_STAT[] = (hits = CLASSICW_STAT[].hits + 1, clamps = CLASSICW_STAT[].clamps, maxpen = max(CLASSICW_STAT[].maxpen, -min(glo, ghi, gend)))
    end
    min(glo, ghi, gend) >= -CLASSICW_PEN - 0.01 && break
    for (g, (nx, nz)) in ((glo, nlo), (ghi, nhi), (gend, nend))
        g < -CLASSICW_PEN || continue
        push = -g - CLASSICW_PEN
        (wx, wz) = DriveRT3D.world_velocity(c)
        vn = wx*nx + wz*nz
        vn < 0.0 && (wx -= vn*nx; wz -= vn*nz)
        cθ = cos(c.θ); sθ = sin(c.θ)
        xn = c.x + push*nx; zn = c.z + push*nz
        c.s_pos(c.integ, [xn, zn]); c.x = xn; c.z = zn
        c.s_vel(c.integ, [wx*cθ + wz*sθ, -wx*sθ + wz*cθ])
        CLASSICW_STAT[] = (hits = CLASSICW_STAT[].hits, clamps = CLASSICW_STAT[].clamps + 1, maxpen = CLASSICW_STAT[].maxpen)
        lat += (nx*gx + nz*gz)*push
    end
  end
    CLASSICW_LAST[] = (c.x, c.z)
    lo < lat < hi && (CLASSICW_PREV[] = lat)
    nothing
end

const AIJIT_ON      = CLASSICW_ON && get(ENV, "JM_AIJIT", "1") != "0"
const AIJIT_TAU_LAT = parse(Float64, get(ENV, "JM_AIJIT_TAU_LAT", "0.25"))
const AIJIT_TAU_YAW = parse(Float64, get(ENV, "JM_AIJIT_TAU_YAW", "0.06"))
const AIJIT_TAU_ATT = parse(Float64, get(ENV, "JM_AIJIT_TAU_ATT", "0.12"))
const AIJIT_TAU_Y   = parse(Float64, get(ENV, "JM_AIJIT_TAU_Y", "0.03"))
mutable struct AISmooth; on::Bool; hint::Int; lat::Float64; y::Float64; pit::Float64; rol::Float64; px::Float64; pz::Float64; yaw::Float64; end

const AIREF = let A = CLASSICREF, n = length(A), out = NTuple{2,Float64}[]
    for i in 1:n, k in 0:7
        u = k/8; h = A[mod1(i - 1, n)]; a = A[i]; b = A[mod1(i + 1, n)]; c = A[mod1(i + 2, n)]
        cr(p0, p1, p2, p3) = 0.5*((2p1) + (-p0 + p2)*u + (2p0 - 5p1 + 4p2 - p3)*u^2 + (-p0 + 3p1 - 3p2 + p3)*u^3)
        push!(out, (cr(h[1], a[1], b[1], c[1]), cr(h[2], a[2], b[2], c[2])))
    end
    out
end
"locate (x, z) on AIREF near `hint` (0 = whole line): (node, fraction, lateral (Classic sign), +lateral unit x, z)"
function airef_locate(x, z, hint::Int)
    A = AIREF; n = length(A); best = Inf; bk = 1; bu = 0.0
    rng = hint == 0 ? (1:n) : ((hint - 200):(hint + 200))
    @inbounds for kk in rng
        k = mod1(kk, n); j = k == n ? 1 : k + 1
        ax, az = A[k]; tx = A[j][1] - ax; tz = A[j][2] - az; L2 = tx*tx + tz*tz
        u = L2 > 0 ? clamp(((x - ax)*tx + (z - az)*tz)/L2, 0.0, 1.0) : 0.0
        dx = x - (ax + u*tx); dz = z - (az + u*tz); d2 = dx*dx + dz*dz
        d2 < best && (best = d2; bk = k; bu = u)
    end
    hint != 0 && best > 400.0^2 && return airef_locate(x, z, 0)
    j = bk == n ? 1 : bk + 1; tx = A[j][1] - A[bk][1]; tz = A[j][2] - A[bk][2]; tl = max(hypot(tx, tz), 1e-9)
    lx = CLASSICW_SGN*(-tz/tl); lz = CLASSICW_SGN*(tx/tl)
    (bk, bu, (x - (A[bk][1] + bu*tx))*lx + (z - (A[bk][2] + bu*tz))*lz, lx, lz)
end
function airef_world(k, u, lat)
    A = AIREF; n = length(A); j = k == n ? 1 : k + 1
    tx = A[j][1] - A[k][1]; tz = A[j][2] - A[k][2]; tl = max(hypot(tx, tz), 1e-9)
    (A[k][1] + u*tx + CLASSICW_SGN*lat*(-tz/tl), A[k][2] + u*tz + CLASSICW_SGN*lat*(tx/tl))
end
const AISM = AISmooth[]
"heading of Classic's line at (section, fraction), from a Catmull-Rom spline through the nodes (continuous: the chords between
nodes turn at every node, which a heading taken from chords would show as a step)"
function classicw_heading_cr(sec, f)
    A = CLASSICREF; n = length(A)
    t = (sec - 1)*CLASSICREF_SUB + f*CLASSICREF_SUB; i = clamp(floor(Int, t) + 1, 1, n); u = t - (i - 1)
    h = mod1(i - 1, n); j = mod1(i + 1, n); k = mod1(i + 2, n)
    dcr(p0, p1, p2, p3) = 0.5*((-p0 + p2) + 2*(2p0 - 5p1 + 4p2 - p3)*u + 3*(-p0 + 3p1 - 3p2 + p3)*u^2)
    atan(dcr(A[h][2], A[i][2], A[j][2], A[k][2]), dcr(A[h][1], A[i][1], A[j][1], A[k][1]))
end
function classicw_world_cr(sec, f, lat)
    A = CLASSICREF; n = length(A)
    t = (sec - 1)*CLASSICREF_SUB + f*CLASSICREF_SUB; i = clamp(floor(Int, t) + 1, 1, n); u = t - (i - 1)
    h = mod1(i - 1, n); j = mod1(i + 1, n); k = mod1(i + 2, n)
    cr(p0, p1, p2, p3) = 0.5*((2p1) + (-p0 + p2)*u + (2p0 - 5p1 + 4p2 - p3)*u^2 + (-p0 + 3p1 - 3p2 + p3)*u^3)
    dcr(p0, p1, p2, p3) = 0.5*((-p0 + p2) + 2*(2p0 - 5p1 + 4p2 - p3)*u + 3*(-p0 + 3p1 - 3p2 + p3)*u^2)
    bx = cr(A[h][1], A[i][1], A[j][1], A[k][1]); bz = cr(A[h][2], A[i][2], A[j][2], A[k][2])
    tx = dcr(A[h][1], A[i][1], A[j][1], A[k][1]); tz = dcr(A[h][2], A[i][2], A[j][2], A[k][2]); tl = max(hypot(tx, tz), 1e-9)
    (bx + CLASSICW_SGN*lat*(-tz/tl), bz + CLASSICW_SGN*lat*(tx/tl))
end
"smooth one AI car's drawn pose (x, y, z, yaw, pitch, roll) the Classic way; `i` = car index, `dt` = frame step"
function ai_smooth(i, p, dt)
    while length(AISM) < i; push!(AISM, AISmooth(false, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)); end
    st = AISM[i]
    (x, y, z, θ, pit, rol) = (Float64(p[1]), Float64(p[2]), Float64(p[3]), Float64(p[4]), Float64(p[5]), Float64(p[6]))
    if st.on && hypot(x - st.px, z - st.pz) > 15.0; st.on = false; end
    (bk, fu, lat) = airef_locate(x, z, st.on ? st.hint : 0)
    if !st.on
        st.on = true; st.hint = bk; st.lat = lat; st.y = y; st.pit = pit; st.rol = rol; st.px = x; st.pz = z; st.yaw = θ
        return (x, y, z, θ, pit, rol)
    end
    st.hint = bk
    a(τ) = 1.0 - exp(-dt/max(τ, 1e-4))
    lat0 = st.lat
    st.lat += (lat - st.lat)*a(AIJIT_TAU_LAT)
    (wx, wz) = airef_world(bk, fu, st.lat)
    st.y += (y - st.y)*a(AIJIT_TAU_Y); st.pit += (pit - st.pit)*a(AIJIT_TAU_ATT); st.rol += (rol - st.rol)*a(AIJIT_TAU_ATT)
    dx = wx - st.px; dz = wz - st.pz; dd = hypot(dx, dz)
    if dd > 1e-2
        th = let A = AIREF, j = mod1(bk + 1, length(A)); atan(A[j][2] - A[bk][2], A[j][1] - A[bk][1]) end
        abs(atan(sin(th - θ), cos(th - θ))) > pi/2 && (th += pi)
        tgt = th + CLASSICW_SGN*atan(st.lat - lat0, dd)*(abs(atan(sin(th - θ), cos(th - θ))) > pi/2 ? -1 : 1)
        abs(atan(sin(tgt - θ), cos(tgt - θ))) > 1.2 && (tgt = θ)
        st.yaw += atan(sin(tgt - st.yaw), cos(tgt - st.yaw))*a(AIJIT_TAU_YAW)
    else
        st.yaw += atan(sin(θ - st.yaw), cos(θ - st.yaw))*a(AIJIT_TAU_YAW)
    end
    st.px = wx; st.pz = wz
    (wx, st.y, wz, st.yaw, st.pit, st.rol)
end
"world point of Classic (section, fraction, lateral)"
function classicw_world(sec, f, lat)
    A = CLASSICREF; n = length(A)
    t = (sec - 1)*CLASSICREF_SUB + f*CLASSICREF_SUB; k = clamp(floor(Int, t) + 1, 1, n); u = t - (k - 1)
    j = k == n ? 1 : k + 1
    tx = A[j][1] - A[k][1]; tz = A[j][2] - A[k][2]; tl = max(hypot(tx, tz), 1e-9)
    (A[k][1] + u*tx + CLASSICW_SGN*lat*(-tz/tl), A[k][2] + u*tz + CLASSICW_SGN*lat*(tx/tl))
end

function boundary_audit()
    println("\n  ═══ E7 BOUNDARY AUDIT — $(uppercasefirst(TRACKSEL)) ═══")
    onhat(x,z) = JRTrack.hat3d(TERRAIN, x, z; ref=Inf)[3]
    n = length(ALIGNED); step = max(1, n ÷ 120); ks = 1:step:n
    CAP = 1500.0
    holes = Int[]; edges = Float64[]; wide = 0
    for k in ks
        x,z = ALIGNED[k]; x2,z2 = ALIGNED[k % n + 1]
        tx,tz = x2-x, z2-z; tl = hypot(tx,tz); tl < 1e-6 && continue
        px,pz = -tz/tl, tx/tl
        onhat(x,z) || push!(holes, k)
        for sgn in (1.0, -1.0)
            d = 0.0; hit = false
            while d < CAP
                d += 1.0
                if !onhat(x + sgn*px*d, z + sgn*pz*d); push!(edges, d); hit = true; break; end
            end
            hit || (wide += 1)
        end
    end

    holelen(k) = begin
        x,z = ALIGNED[k]; x2,z2 = ALIGNED[k % n + 1]
        tx,tz = x2-x, z2-z; tl = hypot(tx,tz); tl < 1e-6 && return 0.0
        tx,tz = tx/tl, tz/tl; fwd = 0.0; bwd = 0.0
        while fwd < 300 && !onhat(x+tx*fwd, z+tz*fwd); fwd += 1.0; end
        while bwd < 300 && !onhat(x-tx*bwd, z-tz*bwd); bwd += 1.0; end
        fwd + bwd
    end
    println("  samples: $(length(ks))   on-line holes: $(length(holes))",
            isempty(holes) ? "" : "  at line-fractions $(round.([h/n for h in holes],digits=2)) lengths $(round.(holelen.(holes),digits=0)) m")
    isempty(edges) || println("  lateral world-edge: min $(round(minimum(edges),digits=1)) m  ",
            "median $(round(sort(edges)[max(1,end÷2)],digits=1)) m  max $(round(maximum(edges),digits=1)) m  ($(length(edges)) probes)")
    println("  sides whose run-off exceeds $(round(Int,CAP)) m (still bounded — finite mesh): $wide")

    println(isempty(holes) ?
        "  ✓ PASS — world is sealed (finite HAT, no on-line holes): the car cannot leave the world." :
        "  ⚠ on-line holes (false containment risk on the racing line) — see fractions above.")
end
if haskey(ENV, "JM_BOUNDARY_TEST") && !SKIDPAD
    boundary_audit(); exit(0)
end

const LOTDIR = joinpath(classic_root(), "cars", "cars67", "lotus")
const CLASSICTEX = Render.classic_texture_index(LOTDIR)
const LOT3DO = joinpath(LOTDIR,"lotus.3do")

const MIRROR_TEX  = ("mirror","lotmirt","lrm","lrimext","lotubase","lotubas2")
const MIRROR_DRAW = Tuple(split(get(ENV, "JM_MIRROR_DRAW", "mirror,lrm"), ","; keepempty = false))
const TUB_GREY = parse(Float32, get(ENV,"JM_TUB_GREY","0.11"))

const HANDS = get(ENV,"JM_HANDS","1") != "0"
const _HAND_EXC = ("ltraymap","lshad","lohand","lotarms","dash7a","windlot")

const _LOTBLACK_EXC = get(ENV,"JM_KEEP_LOTBLACK","0") != "0" ? () : ("lotblack",)
const _EXTRA_EXC = Tuple(split(get(ENV,"JM_EXTRA_EXCLUDE",""), ",", keepempty=false))

const DRIVER_TEX = ("driver5","lotbody","lotsho","knees","neck","lid","arms")

const HELM_OFF = (parse(Float32,get(ENV,"JM_HELM_X","0.14")), parse(Float32,get(ENV,"JM_HELM_Y","0.34")), 0f0)

const HELMET_TEX = get(ENV, "JM_HELMET", "l10helm")
const HELMP = [Render.TrackPart(p.verts, p.tex=="" ? HELMET_TEX : p.tex, p.col)
               for p in Render.extract_classic_car(joinpath(LOTDIR,"helmeg.3do"); maxlat=0.95f0)]

const _GARBAGE_EXC = ("front1","front3")
const CARP_MAXLAT  = parse(Float32, get(ENV,"JM_CARP_MAXLAT","0.85"))

const _CARP_SRC = get(ENV,"JM_BODY_WRAP","1") != "0" ? joinpath(LOTDIR,"lotd.3DO") : LOT3DO

const _COCKPIT_ONLY = ("dash7","dash7a","ldashr")

const _CAR_DEDUP = get(ENV,"JM_CAR_DEDUP","1") != "0" ? :orient : false
const CARP   = Render.extract_classic_car(_CARP_SRC; exclude=(_HAND_EXC...,_LOTBLACK_EXC...,_EXTRA_EXC...,_GARBAGE_EXC...,DRIVER_TEX...,MIRROR_TEX...,Render.STEER_TEX...,"pipe3","plaface","plahelm",_COCKPIT_ONLY...), exclude_groups=Tuple(parse(Int, x) for x in split(get(ENV, "JM_CAR_EXCL_GROUPS", "6600,3560,27288,39792"), ",") if !isempty(strip(x))), cockpit_clean=true, maxlat=CARP_MAXLAT, dedup=_CAR_DEDUP, grey=(TUB_GREY,TUB_GREY+0.01f0,TUB_GREY+0.02f0), visor=true)

if get(ENV,"JM_NOSE_UVFIX","0") != "0"
    let eps = parse(Float64, get(ENV,"JM_NOSE_ZEPS","0.01")), nfix = 0, nparts = 0
        for p in CARP
            v = p.verts; ntri = div(length(v), 33)
            ntri < 4 && continue
            X = Float64[]; Y = Float64[]; Z = Float64[]; U = Float64[]; V = Float64[]
            bad = Int[]
            for t in 0:(ntri-1)
                b = t*33
                p0 = (v[b+1], v[b+2], v[b+3]); p1 = (v[b+12], v[b+13], v[b+14]); p2 = (v[b+23], v[b+24], v[b+25])
                e1 = p1 .- p0; e2 = p2 .- p0
                cr = (e1[2]*e2[3]-e1[3]*e2[2], e1[3]*e2[1]-e1[1]*e2[3], e1[1]*e2[2]-e1[2]*e2[1])
                wa = sqrt(cr[1]^2 + cr[2]^2 + cr[3]^2)/2
                du1 = v[b+21] - v[b+10]; dv1 = v[b+22] - v[b+11]
                du2 = v[b+32] - v[b+10]; dv2 = v[b+33] - v[b+11]
                ua = abs(du1*dv2 - dv1*du2)/2
                if wa > 1e-9 && ua/wa < eps
                    push!(bad, t)
                elseif wa > 1e-9
                    for o in (b, b+11, b+22)
                        push!(X, v[o+1]); push!(Y, v[o+2]); push!(Z, v[o+3])
                        push!(U, v[o+10]); push!(V, v[o+11])
                    end
                end
            end
            (isempty(bad) || length(X) < 8) && continue
            A = hcat(ones(length(X)), X, Y, Z)
            au = A \ U; av = A \ V
            for t in bad
                b = t*33
                for o in (b, b+11, b+22)
                    x = Float64(v[o+1]); y = Float64(v[o+2]); z = Float64(v[o+3])
                    v[o+10] = Float32(au[1] + au[2]*x + au[3]*y + au[4]*z)
                    v[o+11] = Float32(av[1] + av[2]*x + av[3]*y + av[4]*z)
                end
            end
            nfix += length(bad); nparts += 1
        end
        nfix > 0 && println("  [noseuv] REPAIRED ", nfix, " degenerate-UV facets across ", nparts, " part(s) by affine fit from their non-degenerate neighbours (JM_NOSE_UVFIX=0 reverts)")
    end
end

if get(ENV,"JM_NOSE_GRILLEFIX","1") != "0"
    let gb = [parse(Float64,x) for x in split(get(ENV,"JM_GRILLE_UV","0.36,0.61,0.57,0.91"), ",")],
        xsplit = parse(Float32, get(ENV,"JM_NOSE_X","0.6")), nfix = 0, nparts = 0
        for p in CARP
            v = p.verts; ntri = div(length(v), 33)
            ntri < 4 && continue
            X = Float64[]; Y = Float64[]; Z = Float64[]; U = Float64[]; V = Float64[]
            bad = Int[]
            for t in 0:(ntri-1)
                b = t*33
                cx = (v[b+1] + v[b+12] + v[b+23])/3
                cx > xsplit || continue
                cu = (v[b+10] + v[b+21] + v[b+32])/3
                cv = (v[b+11] + v[b+22] + v[b+33])/3
                if gb[1] <= cu <= gb[2] && gb[3] <= cv <= gb[4]
                    push!(bad, t)
                else
                    for o in (b, b+11, b+22)
                        push!(X, v[o+1]); push!(Y, v[o+2]); push!(Z, v[o+3])
                        push!(U, v[o+10]); push!(V, v[o+11])
                    end
                end
            end
            (isempty(bad) || length(X) < 12) && continue
            A = hcat(ones(length(X)), X, Y, Z)
            au = A \ U; av = A \ V
            for t in bad
                b = t*33
                for o in (b, b+11, b+22)
                    x = Float64(v[o+1]); y = Float64(v[o+2]); z = Float64(v[o+3])
                    v[o+10] = Float32(clamp(au[1] + au[2]*x + au[3]*y + au[4]*z, 0f0, 1f0))
                    v[o+11] = Float32(clamp(av[1] + av[2]*x + av[3]*y + av[4]*z, 0f0, 1f0))
                end
            end
            nfix += length(bad); nparts += 1
        end
        nfix > 0 && println("  [noseuv] GRILLE-FIX re-mapped ", nfix, " facet(s) across ", nparts,
                            " part(s) from their non-grille front neighbours (JM_NOSE_GRILLEFIX=0 reverts)")
    end
end
if get(ENV,"JM_NOSE_CENSUS","0") != "0"
    let xsplit = parse(Float32, get(ENV,"JM_NOSE_X","0.6"))
        key = Dict{NTuple{9,Int32},Vector{Tuple{Int,Float32,Float32}}}()
        q(v) = Int32(round(v*2048))
        for (pi,p) in enumerate(CARP)
            v = p.verts
            for t in 0:(div(length(v),33)-1)
                b = t*33
                P = ((v[b+1],v[b+2],v[b+3]), (v[b+12],v[b+13],v[b+14]), (v[b+23],v[b+24],v[b+25]))
                cx = (P[1][1]+P[2][1]+P[3][1])/3

                sp = sort(collect(P))
                k = (q(sp[1][1]),q(sp[1][2]),q(sp[1][3]), q(sp[2][1]),q(sp[2][2]),q(sp[2][3]),
                     q(sp[3][1]),q(sp[3][2]),q(sp[3][3]))
                e1 = P[2] .- P[1]; e2 = P[3] .- P[1]
                nx = e1[2]*e2[3] - e1[3]*e2[2]
                push!(get!(key, k, Tuple{Int,Float32,Float32}[]), (pi, Float32(nx), Float32(cx)))
            end
        end
        nstack = 0; nfront = 0; nopp = 0; nsame = 0; ntris = 0
        for (_,g) in key
            ntris += length(g)
            length(g) < 2 && continue
            nstack += 1
            front = any(x -> x[3] > xsplit, g)
            front && (nfront += 1)
            opp = any(x -> x[2] * g[1][2] < 0, g)
            opp ? (nopp += 1) : (nsame += 1)
        end
        println("  [nose] CARP ", ntris, " tris, ", length(key), " distinct positions")
        println("  [nose] coincident stacks: ", nstack, "  (front of x>", xsplit, ": ", nfront, ")",
                "  opposite-facing: ", nopp, "  same-facing: ", nsame)

        function _slope(a, b)
            n = length(a); n < 2 && return (NaN, NaN)
            ma = sum(a)/n; mb = sum(b)/n
            sab = sum((a .- ma) .* (b .- mb)); saa = sum((a .- ma).^2); sbb = sum((b .- mb).^2)
            (saa == 0 || sbb == 0) ? (NaN, NaN) : (sab/saa, sab/sqrt(saa*sbb))
        end
        println("  [noseuv] per-texture UV census over the shipped CARP (front = centroid x > ", xsplit, ")")
        for p in CARP
            v = p.verts
            ntri = div(length(v), 33)
            ntri < 2 && continue
            ys = Float32[]; zs = Float32[]; us = Float32[]; vs = Float32[]
            nfronttri = 0
            for t in 0:(ntri-1)
                b = t*33
                cx = (v[b+1] + v[b+12] + v[b+23])/3
                cx > xsplit && (nfronttri += 1)
                for o in (b, b+11, b+22)
                    push!(ys, v[o+2]); push!(zs, v[o+3])
                    push!(us, v[o+10]); push!(vs, v[o+11])
                end
            end
            svy, rvy = _slope(ys, vs); suz, ruz = _slope(zs, us)

            dens = Float64[]

            zeps = parse(Float64, get(ENV,"JM_NOSE_ZEPS","0.01"))
            nzero = 0; zero_a = 0.0; tot_a = 0.0
            zxmin = Inf; zxmax = -Inf; zymin = Inf; zymax = -Inf; zzmin = Inf; zzmax = -Inf
            for t in 0:(ntri-1)
                b = t*33
                cx = (v[b+1] + v[b+12] + v[b+23])/3
                cx > xsplit || continue
                p0 = (v[b+1], v[b+2], v[b+3]); p1 = (v[b+12], v[b+13], v[b+14]); p2 = (v[b+23], v[b+24], v[b+25])
                e1 = p1 .- p0; e2 = p2 .- p0
                cr = (e1[2]*e2[3]-e1[3]*e2[2], e1[3]*e2[1]-e1[1]*e2[3], e1[1]*e2[2]-e1[2]*e2[1])
                wa = sqrt(cr[1]^2 + cr[2]^2 + cr[3]^2)/2
                du1 = v[b+21] - v[b+10]; dv1 = v[b+22] - v[b+11]
                du2 = v[b+32] - v[b+10]; dv2 = v[b+33] - v[b+11]
                ua = abs(du1*dv2 - dv1*du2)/2
                wa > 1e-9 && push!(dens, ua/wa)

                if wa > 1e-9
                    tot_a += wa
                    if ua/wa < zeps
                        nzero += 1; zero_a += wa
                        cy = (v[b+2] + v[b+13] + v[b+24])/3
                        cz = (v[b+3] + v[b+14] + v[b+25])/3
                        zxmin = min(zxmin, cx); zxmax = max(zxmax, cx)
                        zymin = min(zymin, cy); zymax = max(zymax, cy)
                        zzmin = min(zzmin, cz); zzmax = max(zzmax, cz)
                    end
                end
            end

            if nfronttri > 0
                fu0=1f9; fu1=-1f9; fv0=1f9; fv1=-1f9
                for t in 0:(ntri-1)
                    b = t*33
                    cx = (v[b+1] + v[b+12] + v[b+23])/3
                    cx > xsplit || continue
                    for o in (b, b+11, b+22)
                        fu0=min(fu0,v[o+10]); fu1=max(fu1,v[o+10])
                        fv0=min(fv0,v[o+11]); fv1=max(fv1,v[o+11])
                    end
                end
                println("    ", rpad(p.tex,12), "  FRONT-facet uv box: u[", round(fu0,digits=3), ",", round(fu1,digits=3),
                        "] v[", round(fv0,digits=3), ",", round(fv1,digits=3), "]  -> atlas px x[",
                        round(Int,fu0*256), ",", round(Int,fu1*256), "] y[", round(Int,fv0*256), ",", round(Int,fv1*256), "]")
            end

            if nfronttri > 0
                gb = [parse(Float64,x) for x in split(get(ENV,"JM_GRILLE_UV","0.36,0.61,0.57,0.91"), ",")]
                ng=0; gx0=1f9; gx1=-1f9; gy0=1f9; gy1=-1f9; gz0=1f9; gz1=-1f9
                for t in 0:(ntri-1)
                    b = t*33
                    cu = (v[b+10] + v[b+21] + v[b+32])/3
                    cv = (v[b+11] + v[b+22] + v[b+33])/3
                    (gb[1] <= cu <= gb[2] && gb[3] <= cv <= gb[4]) || continue
                    ng += 1
                    cx = (v[b+1] + v[b+12] + v[b+23])/3
                    cy = (v[b+2] + v[b+13] + v[b+24])/3
                    cz = (v[b+3] + v[b+14] + v[b+25])/3
                    gx0=min(gx0,cx); gx1=max(gx1,cx); gy0=min(gy0,cy); gy1=max(gy1,cy); gz0=min(gz0,cz); gz1=max(gz1,cz)
                end
                if ng > 0
                    println("    ", rpad(p.tex,12), "  facets sampling the GRILLE box: ", ng, "/", ntri,
                            "  world x[", round(gx0,digits=2), ",", round(gx1,digits=2), "]",
                            " y[", round(gy0,digits=2), ",", round(gy1,digits=2), "]",
                            " z[", round(gz0,digits=2), ",", round(gz1,digits=2), "]")
                end
            end
            dtxt = isempty(dens) ? "  density: (no front tris)" :
                let sd = sort(dens)
                    string("  density uv/m2 min=", round(sd[1], digits=4),
                           " med=", round(sd[cld(length(sd),2)], digits=4),
                           " max=", round(sd[end], digits=4))
                end
            if nzero > 0
                println("    ", rpad(p.tex,12), "  DEGENERATE-UV facets: ", nzero, "/", length(dens),
                        " (", round(100*nzero/max(length(dens),1), digits=1), "% of front tris)",
                        "  world area ", round(100*zero_a/max(tot_a,1e-12), digits=1), "% of the part's front area",
                        "  bbox x[", round(zxmin,digits=3), ",", round(zxmax,digits=3), "]",
                        " y[", round(zymin,digits=3), ",", round(zymax,digits=3), "]",
                        " z[", round(zzmin,digits=3), ",", round(zzmax,digits=3), "]")
            end
            println("    ", rpad(p.tex, 12), " tris=", lpad(ntri, 4),
                    " front=", lpad(nfronttri, 4),
                    "  u[", round(minimum(us), digits=2), ",", round(maximum(us), digits=2), "]",
                    " v[", round(minimum(vs), digits=2), ",", round(maximum(vs), digits=2), "]",
                    "  slope(v on y)=", round(svy, digits=3), " r=", round(rvy, digits=3),
                    dtxt)
        end
    end

    get(ENV,"JM_CENSUS_EXIT","0") != "0" && exit(0)
end
const DRIVERP = Render.extract_classic_car(LOT3DO; only=DRIVER_TEX, maxlat=0.95f0, exclude_groups=Tuple(parse(Int, x) for x in split(get(ENV, "JM_CAR_EXCL_GROUPS", "6600,3560,27288,39792"), ",") if !isempty(strip(x))))
const GAUGEP = Render.extract_classic_car(LOT3DO; only=("dash7a",), maxlat=0.85f0)
const WINDP  = Render.extract_classic_car(LOT3DO; only=("windlot",), maxlat=0.95f0)

const FSUSPP = haskey(ENV,"JM_FSUSP_OLD") ?
    Render.extract_classic_car(LOT3DO; only=("lsusp1",), maxlat=1.3f0, exclude_groups=Tuple(parse(Int, x) for x in split(get(ENV, "JM_CAR_EXCL_GROUPS", "6600,3560,27288,39792"), ",") if !isempty(strip(x)))) :

    Render.extract_classic_car(LOT3DO; only=("lsusp1","frontlot"), maxlat=parse(Float32,get(ENV,"JM_FSUSP_MAXLAT","0.85")),

                           maxedge=parse(Float32,get(ENV,"JM_FSUSP_MAXEDGE","1.5")))

const RSUSPP2 = Render.extract_classic_car(LOT3DO; only=("lshok","lsusp5","lsusp7","lbrdisc"),
                                       maxlat=1.3f0,
                                       maxedge=parse(Float32,get(ENV,"JM_RSUSP2_MAXEDGE","1.5")))
if get(ENV,"JM_RSUSP2_DIAG","") != ""

    println("== JM_RSUSP2_DIAG: the parts RSUSPP2 actually hands to the renderer ==")
    println("   tex           tris   longitudinal x     lateral z          height y           longest edge")
    for pp in RSUSPP2
        local v = pp.verts; n = length(v) ÷ 11
        ex = [Inf32,-Inf32,Inf32,-Inf32,Inf32,-Inf32]
        for i in 1:11:length(v)-10
            ex[1]=min(ex[1],v[i]);   ex[2]=max(ex[2],v[i])
            ex[3]=min(ex[3],v[i+2]); ex[4]=max(ex[4],v[i+2])
            ex[5]=min(ex[5],v[i+1]); ex[6]=max(ex[6],v[i+1])
        end

        le = 0f0
        for t in 1:33:length(v)-32
            a=(v[t],v[t+1],v[t+2]); b=(v[t+11],v[t+12],v[t+13]); c=(v[t+22],v[t+23],v[t+24])
            for (u,w) in ((a,b),(b,c),(c,a))
                le = max(le, sqrt((u[1]-w[1])^2+(u[2]-w[2])^2+(u[3]-w[3])^2))
            end
        end
        println("   ", rpad(pp.tex,14), rpad(n,7),
                rpad(string(round(ex[1],digits=2),"…",round(ex[2],digits=2)),19),
                rpad(string(round(ex[3],digits=2),"…",round(ex[4],digits=2)),19),
                rpad(string(round(ex[5],digits=2),"…",round(ex[6],digits=2)),19),
                round(le,digits=2))
    end
    flush(stdout)
end
const MIRRORP = Render.extract_classic_car(LOT3DO; only=MIRROR_DRAW, maxlat=0.95f0)

const HAND_VFLIP = get(ENV, "JM_HAND_VFLIP", "0") != "0"

const HAND_SWITCH = get(ENV, "JM_HAND_SWITCH", "0")
const GLOVE_LIGHT = (parse.(Float32, split(get(ENV, "JM_GLOVE_LIGHT", "1.6,0.0,1.0"), ","))...,)
const HANDP  = withenv("JM_SWITCH_VARS" => (HAND_SWITCH == "" ? get(ENV, "JM_SWITCH_VARS", "") : "14=" * HAND_SWITCH)) do
    Render.extract_classic_car(LOT3DO; only=("lohand",),  maxlat=0.95f0, vflip = HAND_VFLIP)
end

const _RSEXC = get(ENV,"JM_RS_NOFLAT","0") != "0" ? ("ltraymap","lshad","lsusp2","lsusp7") : ("ltraymap","lshad")

const _RSONLY = get(ENV,"JM_RS_ONLY","")

const SUSP_TRIM = get(ENV,"JM_SUSP_TRIM","1") != "0" ? (split(get(ENV,"JM_SUSP_TRIM_TEX","axlelot"), ",")...,) : false

const RSUSP_MAXLAT = parse(Float32, get(ENV,"JM_RSUSP_MAXLAT", get(ENV,"JM_TRACK_R","0.74")))

const RSUSP_MINCOMP = parse(Int, get(ENV, "JM_RSUSP_MINCOMP", "0"))
const RSUSPP_A = _RSONLY == "" ? Render.extract_classic_car(LOT3DO; include_groups=(27288,), exclude=_RSEXC, maxlat=RSUSP_MAXLAT, trim=SUSP_TRIM, min_component=RSUSP_MINCOMP, min_component_tex=("axlelot",)) :
                                 Render.extract_classic_car(LOT3DO; include_groups=(27288,), only=(_RSONLY,))
const RSUSPP_B = _RSONLY == "" ? Render.extract_classic_car(LOT3DO; include_groups=(39792,), exclude=_RSEXC, maxlat=RSUSP_MAXLAT, trim=SUSP_TRIM, min_component=RSUSP_MINCOMP, min_component_tex=("axlelot",)) :
                                 Render.extract_classic_car(LOT3DO; include_groups=(39792,), only=(_RSONLY,))

const SHIFTER_ON = get(ENV, "JM_SHIFTER", "1") != "0"
const SHIFT_ONTOP = get(ENV, "JM_SHIFT_ONTOP", "1") != "0"
const SHIFT_OVER_ARMS = get(ENV, "JM_SHIFT_OVER_ARMS", "1") != "0"
const TRACK_SHADOW = get(ENV, "JM_TRACK_SHADOW", "0") != "0"
const SHIFT_TEX = SHIFTER_ON ? ("lotshift",) : ()
const CARPIN = get(ENV,"JM_COCKPIT_DRESS","1") != "0" ?
    Render.extract_classic_car(joinpath(LOTDIR,"lotd.3DO"); exclude=(_HAND_EXC...,_LOTBLACK_EXC...,_EXTRA_EXC...,_GARBAGE_EXC...,DRIVER_TEX...,MIRROR_TEX...,Render.STEER_TEX...,"pipe3","plaface","plahelm",SHIFT_TEX...), exclude_groups=Tuple(parse(Int, x) for x in split(get(ENV, "JM_CAR_EXCL_GROUPS", "6600,3560,27288,39792"), ",") if !isempty(strip(x))), cockpit_clean=true, maxlat=parse(Float32,get(ENV,"JM_COCKPIT_MAXLAT","0.30")), dedup=_CAR_DEDUP, grey=(TUB_GREY,TUB_GREY+0.01f0,TUB_GREY+0.02f0), visor=true) :
    Render.TrackPart[]

const SHIFTP = (SHIFTER_ON && get(ENV,"JM_COCKPIT_DRESS","1") != "0") ?

    Render.extract_classic_car(joinpath(LOTDIR,"lotd.3DO"); only=SHIFT_TEX, cockpit_clean=true,
                           maxlat=parse(Float32,get(ENV,"JM_COCKPIT_MAXLAT","0.30")),
                           dedup=get(ENV, "JM_SHIFT_DEDUP", "0") != "0" ? _CAR_DEDUP : false) : Render.TrackPart[]
const SHIFT_LEN = Ref(0f0)
const SHIFT_PIVOT = let ys = Float32[], ps = Vector{Float32}[]
    for p in SHIFTP, i in 0:11:length(p.verts)-11
        push!(ps, Float32[p.verts[i+1], p.verts[i+2], p.verts[i+3]])
    end
    if isempty(ps)
        nothing
    else
        y0 = minimum(q[2] for q in ps); low = [q for q in ps if q[2] < y0 + 0.01f0]
        c = sum(low) / length(low)
        top = ps[argmax([q[2] for q in ps])]
        println("  [shifter] lever foot ", round.(c; digits=3), "  knob top ", round.(top; digits=3), "  (", length(ps) ÷ 3, " tris)")
        SHIFT_LEN[] = top[2] - c[2]
        c
    end
end

const SHIFT_GATE_UV = Tuple(parse.(Float32, split(get(ENV, "JM_SHIFT_GATE_UV", "0.745,0.11"), ",")))
function _uv_points(parts, tex, uv)
    out = Vector{Vector{Float32}}()
    for p in parts
        lowercase(p.tex) == tex || continue
        v = p.verts
        for t in 0:33:length(v)-33
            P = [Float32[v[t+11k+1], v[t+11k+2], v[t+11k+3]] for k in 0:2]
            U = [v[t+11k+10] for k in 0:2]; V = [v[t+11k+11] for k in 0:2]
            d = (U[2]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(V[2]-V[1]); abs(d) < 1f-9 && continue
            b2 = ((uv[1]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(uv[2]-V[1])) / d
            b3 = ((U[2]-U[1])*(uv[2]-V[1]) - (uv[1]-U[1])*(V[2]-V[1])) / d
            (b2 >= -1f-4 && b3 >= -1f-4 && b2 + b3 <= 1 + 1f-4) || continue
            push!(out, P[1] + b2*(P[2]-P[1]) + b3*(P[3]-P[1]))
        end
    end
    out
end
const SHIFT_GATE = SHIFT_PIVOT === nothing ? nothing : let g = _uv_points(CARPIN, "lotinsid", SHIFT_GATE_UV)
    if isempty(g)
        println("  [shifter] gate art not found on the cockpit mesh -- the lever stays where the mesh put it"); nothing
    else
        gp = g[argmin([sum(abs2, q .- SHIFT_PIVOT) for q in g])] .+ Float32[0, parse(Float32, get(ENV, "JM_SHIFT_DY", "0")), 0]
        println("  [shifter] gate at ", round.(gp; digits = 3), " (", length(g), " match(es)); lever moved by ",
                round.(gp .- SHIFT_PIVOT; digits = 3)); gp
    end
end
const SHIFT_MOVE = SHIFT_GATE === nothing ? Render.ident() : Render.translate(SHIFT_GATE .- SHIFT_PIVOT)

const SHIFT_LAT = parse(Float32, get(ENV, "JM_SHIFT_LAT", "12"))
const SHIFT_FA  = parse(Float32, get(ENV, "JM_SHIFT_FA", "14"))
shift_target(g) = g == 0 ? (0f0, 0f0) : g < 0 ? (-1.6f0, -1f0) :
    (g <= 2 ? -1f0 : g <= 4 ? 0f0 : 1f0, isodd(g) ? 1f0 : -1f0)
const SHIFT_TUNE = Ref(false)
const SHIFT_S = 1.8f0
const SHIFT_LEAN = 55f0
const SHIFT_SY = 1.8f0
const SHIFT_XYZ = Float32[0, -0.06, 0]
const COCKPIT_OWNWHEELS = get(ENV, "JM_COCKPIT_OWNWHEELS", "1") != "0"
const TEAM_LEVER_D = Float32.(parse.(Float64, split(get(ENV, "JM_TEAM_LEVER_D", "0,0,0"), ",")))

const TEAM_LEVER_LEFT = Set(lowercase.(split(get(ENV, "JM_TEAM_LEVER_LEFT", "brm"), ",", keepempty = false)))
const TEAM_LEVER_MIRD = Float32.(parse.(Float64, split(get(ENV, "JM_TEAM_LEVER_MIRD", "0.05,0.045,0"), ",")))
const LEVER_TOP = get(ENV, "JM_TEAM_LEVER_TOP", "0") != "0"

const TEAM_LEVER_DT = Dict(lowercase(strip(first(split(e, ":")))) => Float32.(parse.(Float64, split(last(split(e, ":")), ",")))
                           for e in split(get(ENV, "JM_TEAM_LEVER_DT", "cooper:0,0,-0.05"), ";"; keepempty = false))
const TEAM_SLEEVE_RIGID = get(ENV, "JM_TEAM_SLEEVE_RIGID", "0") != "0"

const TEAM_SLEEVE_LOCK = get(ENV, "JM_TEAM_SLEEVE_LOCK", "1") != "0"
const TEAM_CUFF_INTO = parse(Float64, get(ENV, "JM_TEAM_CUFF_INTO", "0.03"))
const TEAM_LEVER_SM = Ref(Render.ident())

const TEAM_SHIFT_HAND = get(ENV, "JM_TEAM_SHIFT_HAND", "0") != "0"
const TEAM_LEVER_LM = Ref{Union{Nothing,Matrix{Float32}}}(nothing)
const TEAM_HANDDIAG = get(ENV, "JM_TEAM_HANDDIAG", "0") != "0"; const TEAM_HANDDIAG_N = Ref(0)

const TEAM_LEVER_GAIN = get(ENV, "JM_TEAM_LEVER_GAIN", "1") != "0"

const TEAM_LEVER_STRAIGHT = get(ENV, "JM_TEAM_LEVER_STRAIGHT", "1") != "0"
team_lever_gain(shlen) = (TEAM_LEVER_GAIN && shlen > 0 && SHIFT_LEN[] > 0) ? clamp(SHIFT_LEN[] / shlen, 1f0, 3f0) : 1f0
const TEAM_LEVER_SIDE = get(ENV, "JM_TEAM_LEVER_SIDE", "1") != "0"
const LEVER_DIAG_G = Ref(-99)
const SHIFT_STATE = Float32[0, 0]

const LEVER_CYCLE = parse(Float64, get(ENV, "JM_LEVER_CYCLE", "0"))
lever_gear(g, t) = LEVER_CYCLE > 0 ? (0, 1, 2, 3, 4, 5, -1)[mod(floor(Int, t / LEVER_CYCLE), 7) + 1] : g
function shift_model(g, dt)
    tl, tf = shift_target(g); l, f = SHIFT_STATE; k = clamp(dt / 0.05f0, 0f0, 1f0)
    if abs(tl - l) > 0.02f0
        f += (0f0 - f) * k; abs(f) < 0.15f0 && (l += (tl - l) * k)
    else
        l = tl; f += (tf - f) * k
    end
    SHIFT_STATE .= (l, f)
    c = SHIFT_PIVOT; inb = c[3] > 0 ? -1f0 : 1f0
    SHIFT_TUNE[] = SHIFT_TUNE[] || haskey(ENV, "JM_SHIFT_XYZ")
    off = SHIFT_TUNE[] ? Float32.(parse.(Float64, split(get(ENV, "JM_SHIFT_XYZ", "0/0/0"), r"[,/ ]+"))) : SHIFT_XYZ
    sc = SHIFT_TUNE[] ? parse(Float32, get(ENV, "JM_SHIFT_S", "1")) : SHIFT_S
    lean = SHIFT_TUNE[] ? parse(Float32, get(ENV, "JM_SHIFT_LEAN", "0")) : SHIFT_LEAN
    sy = SHIFT_TUNE[] ? parse(Float32, get(ENV, "JM_SHIFT_SY", string(sc))) : SHIFT_SY
    Render.translate(off) * SHIFT_MOVE * Render.translate(c) * Render.rotaxis(Float32[0, 0, 1], -deg2rad(SHIFT_FA * f)) *
        Render.rotaxis(Float32[1, 0, 0], inb * deg2rad(SHIFT_LAT * l + lean)) * Render.scalexyz(sc, sy, sc) * Render.translate(-c)
end

if get(ENV,"JM_NOSE_GRILLEFIX","1") != "0" && !isempty(CARPIN)
    let gb = [parse(Float64,x) for x in split(get(ENV,"JM_GRILLE_UV","0.36,0.61,0.57,0.91"), ",")],
        xsplit = parse(Float32, get(ENV,"JM_NOSE_X","0.6")), nfix = 0, nparts = 0
        for p in CARPIN
            v = p.verts; ntri = div(length(v), 33)
            ntri < 4 && continue
            X = Float64[]; Y = Float64[]; Z = Float64[]; U = Float64[]; V = Float64[]
            bad = Int[]
            for t in 0:(ntri-1)
                b = t*33
                cx = (v[b+1] + v[b+12] + v[b+23])/3
                cx > xsplit || continue
                cu = (v[b+10] + v[b+21] + v[b+32])/3
                cv = (v[b+11] + v[b+22] + v[b+33])/3
                if gb[1] <= cu <= gb[2] && gb[3] <= cv <= gb[4]
                    push!(bad, t)
                else
                    for o in (b, b+11, b+22)
                        push!(X, v[o+1]); push!(Y, v[o+2]); push!(Z, v[o+3])
                        push!(U, v[o+10]); push!(V, v[o+11])
                    end
                end
            end
            (isempty(bad) || length(X) < 12) && continue
            A = hcat(ones(length(X)), X, Y, Z)
            au = A \ U; av = A \ V
            for t in bad
                b = t*33
                for o in (b, b+11, b+22)
                    x = Float64(v[o+1]); y = Float64(v[o+2]); z = Float64(v[o+3])
                    v[o+10] = Float32(clamp(au[1] + au[2]*x + au[3]*y + au[4]*z, 0f0, 1f0))
                    v[o+11] = Float32(clamp(av[1] + av[2]*x + av[3]*y + av[4]*z, 0f0, 1f0))
                end
            end
            nfix += length(bad); nparts += 1
        end
        println("  [noseuv] CARPIN grille-fix: ", nfix, " facet(s) across ", nparts, " part(s)")
    end
end

const CAR_UNMIRROR = get(ENV, "JM_CAR_UNMIRROR", "1") != "0"
const CARHAND = CAR_UNMIRROR ? Render.scalexyz(1f0, 1f0, -1f0) : Render.scalexyz(1f0, 1f0, 1f0)
const HSIGN = CAR_UNMIRROR ? -1.0 : 1.0

const _DASH_TEX = ("dash7a","dash7","ldashr")

function _dash_vflip!(parts)
    for p in parts
        lowercase(p.tex) in _DASH_TEX || continue
        v = p.verts
        if get(ENV,"JM_DASH_DIAG","0") != "0"
            for t in 0:(div(length(v),33) - 1)
                b = t*33
                ys = (v[b+2], v[b+13], v[b+24]); vs = (v[b+11], v[b+22], v[b+33])
                println("    [dashdiag] ", p.tex, " tri", t,
                        " y[", round(minimum(ys),digits=3), ",", round(maximum(ys),digits=3), "]",
                        " v[", round(minimum(vs),digits=3), ",", round(maximum(vs),digits=3), "]",
                        " u[", round(min(v[b+10],v[b+21],v[b+32]),digits=3), ",",
                              round(max(v[b+10],v[b+21],v[b+32]),digits=3), "]")
            end
        end
    end
    if get(ENV,"JM_DASH_VFLIP", CAR_UNMIRROR ? "0" : "1") == "0"
        println("  [dashvflip] OFF (JM_DASH_VFLIP=0)"); return parts
    end
    n = 0
    for p in parts
        lowercase(p.tex) in _DASH_TEX || continue
        v = p.verts

        vmn = Inf32; vmx = -Inf32
        @inbounds for i in 11:11:length(v); vmn = min(vmn, v[i]); vmx = max(vmx, v[i]); end
        sum2 = vmn + vmx
        @inbounds for i in 11:11:length(v); v[i] = sum2 - v[i]; n += 1; end

        println("  [dashvflip] ", p.tex, ": ", length(v) / 33, " tris, v[",
                round(vmn,digits=3), ",", round(vmx,digits=3), "] -> mirrored about ",
                round(sum2/2,digits=3))
    end
    println("  [dashvflip] flipped v on ", n, " dash vertices")
    parts
end
const _DASH_VFLIPPED = _dash_vflip!(CARPIN)

function _dash_zmirror!(parts)
    if get(ENV,"JM_DASH_ZMIRROR", CAR_UNMIRROR ? "0" : "1") == "0"
        println("  [dashzmirror] OFF (JM_DASH_ZMIRROR=0)"); return parts
    end
    n = 0
    for p in parts
        lowercase(p.tex) in _DASH_TEX || continue
        v = p.verts
        @inbounds for i in 0:11:length(v)-11; v[i+3] = -v[i+3]; v[i+6] = -v[i+6]; end
        @inbounds for t in 0:33:length(v)-33, k in 1:11
            v[t+11+k], v[t+22+k] = v[t+22+k], v[t+11+k]
        end
        n += length(v) ÷ 33
    end
    println("  [dashzmirror] mirrored ", n, " dash triangles across the centreline")
    parts
end
const _DASH_ZMIRRORED = _dash_zmirror!(CARPIN)

const TACH_UV = (0.825f0, 0.2265f0); const TACH_RU = 0.147f0
const TACH_DEG0 = 113.6; const TACH_DEG_PER_K = 26.4
function _tach_frame(parts)
    for p in parts
        lowercase(p.tex) == "dash7" || continue
        v = p.verts
        for t in 0:33:length(v)-33
            P = [Float32[v[t+11k+1], v[t+11k+2], v[t+11k+3]] for k in 0:2]
            U = [v[t+11k+10] for k in 0:2]; V = [v[t+11k+11] for k in 0:2]
            d = (U[2]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(V[2]-V[1]); abs(d) < 1f-9 && continue

            b2 = ((TACH_UV[1]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(TACH_UV[2]-V[1])) / d
            b3 = ((U[2]-U[1])*(TACH_UV[2]-V[1]) - (TACH_UV[1]-U[1])*(V[2]-V[1])) / d
            (b2 >= -1f-4 && b3 >= -1f-4 && b2 + b3 <= 1 + 1f-4) || continue
            c = P[1] + b2*(P[2]-P[1]) + b3*(P[3]-P[1])

            e1 = P[2]-P[1]; e2 = P[3]-P[1]
            dPdu = ( e1*(V[3]-V[1]) - e2*(V[2]-V[1])) / d
            dPdv = (-e1*(U[3]-U[1]) + e2*(U[2]-U[1])) / d
            right = dPdu * TACH_RU; up = -dPdv * TACH_RU
            nrm = normalize(cross(right, up))

            dot(nrm, Float32[0.46, 0.40, 0] - c) < 0 && (nrm = -nrm)
            return (c = c, right = right, up = up, n = nrm)
        end
    end
    nothing
end
const TACH = _tach_frame(CARPIN)
TACH === nothing ? println("  [tach] dial centre not found -- needles off") :
    println("  [tach] dial centre ", round.(TACH.c; digits=3), "  |right| ", round(norm(TACH.right); digits=4),
            " |up| ", round(norm(TACH.up); digits=4), "  right.up ", round(dot(normalize(TACH.right), normalize(TACH.up)); digits=3))

function _needle_verts(deg, tip, tail, w0, w1, col; lift = 0.002f0)
    th = deg2rad(deg); dir = sin(th)*TACH.right + cos(th)*TACH.up
    side = normalize(cross(dir, TACH.n)) * norm(TACH.right)
    c = TACH.c + lift*TACH.n
    a = c - tail*dir; b = c + tip*dir
    q = (a - w0*side, a + w0*side, b + w1*side, b - w1*side)
    out = Float32[]
    for i in (1,2,3, 1,3,4)
        append!(out, q[i]); append!(out, TACH.n); append!(out, col); append!(out, (0f0, 0f0))
    end
    out
end
const TACH_ON = TACH !== nothing && get(ENV,"JM_TACH_NEEDLES","1") != "0"
const TACH_SGN = TACH === nothing ? 1f0 :
    (dot(Vector{Float32}((Render.rotaxis(TACH.n, 0.1) * Float32[TACH.up..., 0])[1:3]), TACH.right) > 0 ? 1f0 : -1f0)
const TELLTALE = Ref(0.0)
tach_deg(rpm) = TACH_DEG0 + TACH_DEG_PER_K * clamp(rpm, 0.0, 12000.0) / 1000

tach_model(deg) = Render.translate(TACH.c) * Render.rotaxis(TACH.n, TACH_SGN * Float32(deg2rad(deg))) * Render.translate(-TACH.c)

const DIAL_SPECS = (
    (name = "oilpsi", tex = "dash7",  uv = (0.5550f0, 0.3163f0), ru = 0.1120f0,
     scale = ((0.0, -90.0), (20.0, -60.0), (40.0, -20.0), (60.0, 20.0), (80.0, 55.0))),
    (name = "oiltemp", tex = "dash7", uv = (0.5550f0, 0.3163f0), ru = 0.1120f0,
     scale = ((40.0, 235.0), (60.0, 215.0), (80.0, 200.0), (100.0, 180.0), (120.0, 140.0))),
    (name = "water",  tex = "ldashr", uv = (0.1192f0, 0.3131f0), ru = 0.1053f0,
     scale = ((40.0, -140.0), (60.0, -120.0), (80.0, -100.0), (100.0, -40.0), (110.0, 0.0), (120.0, 40.0),
              (130.0, 90.0), (140.0, 135.0))),
    (name = "fuelpsi", tex = "ldashr", uv = (0.4980f0, 0.3201f0), ru = 0.1075f0, scale = ((0.0, -135.0), (120.0, 135.0))),
    (name = "volts",  tex = "ldashr", uv = (0.6942f0, 0.4437f0), ru = 0.0978f0, scale = ((8.0, -60.0), (16.0, 60.0))))

function _dial_frame(parts, tex, uv, ru)
    for p in parts
        lowercase(p.tex) == tex || continue
        v = p.verts
        for t in 0:33:length(v)-33
            P = [Float32[v[t+11k+1], v[t+11k+2], v[t+11k+3]] for k in 0:2]
            U = [v[t+11k+10] for k in 0:2]; V = [v[t+11k+11] for k in 0:2]
            d = (U[2]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(V[2]-V[1]); abs(d) < 1f-9 && continue
            b2 = ((uv[1]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(uv[2]-V[1])) / d
            b3 = ((U[2]-U[1])*(uv[2]-V[1]) - (uv[1]-U[1])*(V[2]-V[1])) / d
            (b2 >= -1f-4 && b3 >= -1f-4 && b2 + b3 <= 1 + 1f-4) || continue
            c = P[1] + b2*(P[2]-P[1]) + b3*(P[3]-P[1])
            e1 = P[2]-P[1]; e2 = P[3]-P[1]
            right = (( e1*(V[3]-V[1]) - e2*(V[2]-V[1])) / d) * ru
            up    = -((-e1*(U[3]-U[1]) + e2*(U[2]-U[1])) / d) * ru
            nrm = normalize(cross(right, up)); dot(nrm, Float32[0.46, 0.40, 0] - c) < 0 && (nrm = -nrm)
            sgn = dot(Vector{Float32}((Render.rotaxis(nrm, 0.1) * Float32[up..., 0])[1:3]), right) > 0 ? 1f0 : -1f0
            return (c = c, right = right, up = up, n = nrm, sgn = sgn)
        end
    end
    nothing
end
const DIALS_ON = get(ENV, "JM_DIALS", "1") != "0"
const DIALS = DIALS_ON ? [(spec = sp, sym = Symbol(sp.name), f = _dial_frame(CARPIN, sp.tex, sp.uv, sp.ru)) for sp in DIAL_SPECS] : []
filter!(d -> d.f !== nothing, DIALS)
println("  [dials] ", isempty(DIALS) ? "none found" : join([d.spec.name for d in DIALS], " "), " (JM_DIALS=0 hides them)")
function dial_deg(scale, x)
    x <= scale[1][1] && return scale[1][2]
    for k in 2:length(scale)
        x <= scale[k][1] && return scale[k-1][2] + (x - scale[k-1][1]) / (scale[k][1] - scale[k-1][1]) * (scale[k][2] - scale[k-1][2])
    end
    scale[end][2]
end
dial_model(f, deg) = Render.translate(f.c) * Render.rotaxis(f.n, f.sgn * Float32(deg2rad(deg))) * Render.translate(-f.c)
function _dial_needle_verts(f, tip, tail, w0, w1, col; lift = 0.002f0)
    dir = f.up; side = normalize(cross(dir, f.n)) * norm(f.right); c = f.c + lift*f.n
    a = c - tail*dir; b = c + tip*dir
    q = (a - w0*side, a + w0*side, b + w1*side, b - w1*side)
    out = Float32[]
    for i in (1,2,3, 1,3,4); append!(out, q[i]); append!(out, f.n); append!(out, col); append!(out, (0f0, 0f0)); end
    out
end

const DIALS_CLASSIC = get(ENV, "JM_DIALS_CLASSIC", "0") == "1"
mutable struct DialVals; oilpsi::Float64; oiltemp::Float64; water::Float64; fuelpsi::Float64; volts::Float64; tw::Float64; end
const DIALV = DIALS_CLASSIC ? DialVals(0.0, 43.0, 43.0, 0.0, 12.2, 0.0) : DialVals(0.0, 77.0, 77.0, 0.0, 12.0, 0.0)
const OILP_RPM = (0.0, 1000.0, 2000.0, 4000.0, 5000.0, 6000.0, 20000.0); const OILP_BAR = (0.0, 1.35, 4.33, 4.34, 5.17, 5.78, 5.78)
"reference's Lotus 49 oil pressure [bar] at an rpm."
oil_bar(rpm) = (r = clamp(Float64(rpm), 0.0, 19999.0); i = findlast(k -> OILP_RPM[k] <= r, 1:length(OILP_RPM));
                OILP_BAR[i] + (r - OILP_RPM[i])/(OILP_RPM[i+1] - OILP_RPM[i])*(OILP_BAR[i+1] - OILP_BAR[i]))
const FUEL_BAR = 3.79
function dials_update!(rpm)
    t = time(); dt = DIALV.tw == 0.0 ? 0.0 : clamp(t - DIALV.tw, 0.0, 0.1); DIALV.tw = t
    run = rpm > 200; load = clamp(rpm / 9000, 0.0, 1.0)
    if DIALS_CLASSIC
        DIALV.oilpsi  = run ? 79.0 * clamp(rpm / 1500, 0.0, 1.0) : 0.0
        DIALV.fuelpsi = run ? 79.0 * clamp(rpm / 1500, 0.0, 1.0) : 0.0
        DIALV.volts   = run ? 11.5 : 11.0
        DIALV.water   += ((run ? 80 + 15 * load : 43.0) - DIALV.water)   * dt / 700
        DIALV.oiltemp += ((run ? 85 + 25 * load : 43.0) - DIALV.oiltemp) * dt / 900
        return DIALV
    end
    DIALV.oilpsi  = oil_bar(rpm) * 14.504
    DIALV.fuelpsi = run ? FUEL_BAR * 14.504 : 0.0
    DIALV.volts   = 12.0
    DIALV.water   += ((run ? 63.2 : 77.0) - DIALV.water)   * dt / 433
    DIALV.oiltemp += ((run ? 58.5 : 77.0) - DIALV.oiltemp) * dt / 400
    DIALV
end
"reference's EngineWarnings bits for julia's .ibt: 0x04 oil pressure low (< 2 bar), 0x08 stalled, 0x20 rev limiter."
engine_warnings(rpm, limit) = (rpm < 200 ? 0x08 : 0) | (oil_bar(rpm) < 2.0 ? 0x04 : 0) | (rpm >= limit - 30 ? 0x20 : 0)

function _drop_box!(parts, x0, x1, y0, z0)
    for (pi, p) in enumerate(parts)
        v = p.verts; n = length(v) ÷ 33
        keep = Float32[]
        for t in 0:n-1
            base = t*33
            cx = (v[base+1] + v[base+12] + v[base+23]) / 3
            cy = (v[base+2] + v[base+13] + v[base+24]) / 3
            cz = (v[base+3] + v[base+14] + v[base+25]) / 3
            inpod = (x0 <= cx <= x1) && (cy >= y0) && (abs(cz) >= z0)
            inpod || append!(keep, @view v[base+1:base+33])
        end
        parts[pi] = Render.TrackPart(keep, p.tex, p.col)
    end
    parts
end

const AXLE_Y = parse(Float32, get(ENV, "JM_AXLE_Y", "0.02"))
const AXLE_R = parse(Float32, get(ENV, "JM_AXLE_R", "0.024"))

const AXLE_STRIP_U = (0.70f0, 0.92f0)
function _rod_verts!(v, p0, p1, r, vrange; segs = 10)
    a = Float32[p1[1]-p0[1], p1[2]-p0[2], p1[3]-p0[3]]; a ./= norm(a)
    ref = abs(a[2]) < 0.9f0 ? Float32[0,1,0] : Float32[1,0,0]
    e1 = normalize(cross(a, ref)); e2 = cross(a, e1)
    for k in 0:segs-1
        a0 = 2f0*Float32(pi)*k/segs; a1 = 2f0*Float32(pi)*(k+1)/segs
        n0 = cos(a0)*e1 + sin(a0)*e2; n1 = cos(a1)*e1 + sin(a1)*e2
        u0 = AXLE_STRIP_U[1] + (AXLE_STRIP_U[2]-AXLE_STRIP_U[1])*k/segs
        u1 = AXLE_STRIP_U[1] + (AXLE_STRIP_U[2]-AXLE_STRIP_U[1])*(k+1)/segs
        for (P, N, U, V) in ((p0 + r*n0, n0, u0, vrange[1]), (p1 + r*n0, n0, u0, vrange[2]), (p1 + r*n1, n1, u1, vrange[2]),
                             (p0 + r*n0, n0, u0, vrange[1]), (p1 + r*n1, n1, u1, vrange[2]), (p0 + r*n1, n1, u1, vrange[1]))
            append!(v, P); append!(v, N); append!(v, (1f0, 1f0, 1f0)); append!(v, (Float32(U), Float32(V)))
        end
    end
    v
end
function _axle_part(zsign, x0::Float32, zhub::Float32)
    z0, z1 = zsign*0.16f0, zsign*zhub
    if get(ENV, "JM_AXLE_OLDUV", "0") == "0"
        return Render.TrackPart(_rod_verts!(Float32[], Float32[x0, AXLE_Y, z0], Float32[x0, AXLE_Y, z1], AXLE_R, (0.84f0, 0.02f0)),
                                "axlelot", (1f0,1f0,1f0))
    end
    segs = 10
    v = Float32[]
    for k in 0:segs-1
        a0 = 2f0*Float32(pi)*k/segs; a1 = 2f0*Float32(pi)*(k+1)/segs

        p00 = (x0 + AXLE_R*cos(a0), AXLE_Y + AXLE_R*sin(a0), z0)
        p01 = (x0 + AXLE_R*cos(a1), AXLE_Y + AXLE_R*sin(a1), z0)
        p10 = (x0 + AXLE_R*cos(a0), AXLE_Y + AXLE_R*sin(a0), z1)
        p11 = (x0 + AXLE_R*cos(a1), AXLE_Y + AXLE_R*sin(a1), z1)
        n0 = (cos(a0), sin(a0), 0f0); n1 = (cos(a1), sin(a1), 0f0)
        uv00 = (Float32(k/segs), 0f0); uv01 = (Float32((k+1)/segs), 0f0)
        uv10 = (Float32(k/segs), 1f0); uv11 = (Float32((k+1)/segs), 1f0)
        for (P,N,UV) in ((p00,n0,uv00),(p10,n0,uv10),(p11,n1,uv11),
                         (p00,n0,uv00),(p11,n1,uv11),(p01,n1,uv01))
            append!(v, Float32[P[1],P[2],P[3], N[1],N[2],N[3], UV[1],UV[2], 1f0,1f0,1f0])
        end
    end
    Render.TrackPart(v, "axlelot", (1f0,1f0,1f0))
end
const PIPE_LIFT = parse(Float32, get(ENV, "JM_PIPE_LIFT", "0.18"))

function _mirror_pipes(parts)
    out = Render.TrackPart[]
    for p in parts
        v = p.verts; n = length(v) ÷ 33
        keep = Float32[]
        for t in 0:n-1
            b = t*33
            cz = (v[b+3] + v[b+14] + v[b+25]) / 3
            cz > 0 || continue
            append!(keep, v[b+1:b+33])

            for k in (2,1,0)
                o = b + k*11
                append!(keep, Float32[v[o+1], v[o+2], -v[o+3],
                                      v[o+4], v[o+5], -v[o+6],
                                      v[o+7], v[o+8], v[o+9], v[o+10], v[o+11]])
            end
        end
        isempty(keep) || push!(out, Render.TrackPart(keep, p.tex, p.col))
    end
    out
end
const _PIPES_RAW = get(ENV, "JM_PIPES", "1") != "0" ?
    Render.extract_classic_car(LOT3DO; only=("pipe3",), maxlat=0.9f0,
                           min_component=6, min_component_tex=("pipe3",)) : Render.TrackPart[]

const PIPE_LAT = parse(Float32, get(ENV, "JM_PIPE_LAT", "0.8"))
function _scale_pipes_lat(parts, sc::Float32)
    sc == 1f0 && return parts
    out = Render.TrackPart[]
    for p in parts
        v = copy(p.verts)
        for i in 3:11:length(v); v[i] *= sc; end
        push!(out, Render.TrackPart(v, p.tex, p.col))
    end
    out
end

const PIPE_TIPLIFT = parse(Float32, get(ENV, "JM_PIPE_TIPLIFT", "0"))
function _shear_pipes(parts, k::Float32)
    k == 0f0 && return parts
    xr = maximum(maximum(p.verts[1:11:end]) for p in parts)
    out = Render.TrackPart[]
    for p in parts
        v = copy(p.verts)
        for i in 1:11:length(v); v[i+1] += k * (xr - v[i]); end
        push!(out, Render.TrackPart(v, p.tex, p.col))
    end
    out
end
const PIPEP  = _shear_pipes(_scale_pipes_lat(get(ENV, "JM_PIPE_MIRROR", "1") != "0" ? _mirror_pipes(_PIPES_RAW) : _PIPES_RAW, PIPE_LAT), PIPE_TIPLIFT)
const ARMP   = Render.extract_classic_car(LOT3DO; only=("lotarms",), maxlat=0.95f0, vflip = HAND_VFLIP)

const GCY = (b = Render.parts_bbox(GAUGEP); Float32((b.ymin + b.ymax)/2))

const GAUGE_DY = parse(Float32, get(ENV,"JM_GAUGE_Y","0.20"))
const GAUGE_DX = parse(Float32, get(ENV,"JM_GAUGE_X","-0.04"))

const GAUGE_S = parse(Float32, get(ENV,"JM_GAUGE_S","1.0"))

const GAUGE_DZ = parse(Float32, get(ENV,"JM_GAUGE_Z","0.10"))

const GAUGE_YFLIP = get(ENV,"JM_GAUGE_YFLIP","1") != "0"
const SW_ROT = deg2rad(parse(Float64, get(ENV,"JM_SW_ROT","0")))
const GAUGEFLIP = Render.translate(Float32[GAUGE_DX,GCY+GAUGE_DY,GAUGE_DZ]) * Render.scalexyz(GAUGE_S, GAUGE_YFLIP ? -GAUGE_S : GAUGE_S, GAUGE_S) * Render.translate(Float32[0,-GCY,0])
const SWPARTS, SWCENTER, SWAXIS = Render.extract_classic_steering(LOT3DO)

const _SW_UVFIX_TEX = ("sterlot",)
function _sw_uvfix!(parts)
    if get(ENV,"JM_SW_UVFIX", CAR_UNMIRROR ? "0" : "1") == "0"
        println("  [swuv] OFF (JM_SW_UVFIX=0)"); return parts
    end
    n = 0
    for p in parts
        lowercase(p.tex) in _SW_UVFIX_TEX || continue
        v = p.verts
        @inbounds for i in 10:11:length(v)-1
            v[i]   = 1f0 - v[i]
            v[i+1] = 1f0 - v[i+1]
            n += 1
        end
        println("  [swuv] ", p.tex, ": rotated UVs 180 deg on ", div(length(v),11), " vertices")
    end
    println("  [swuv] parts fixed: ", n, " vertices")
    parts
end
const _SW_UVFIXED = _sw_uvfix!(SWPARTS)

const MCEN = (b = Render.parts_bbox(MIRRORP); Float32[(b.xmin+b.xmax)/2, (b.ymin+b.ymax)/2, 0f0])

const MIRROR_DY   = parse(Float32, get(ENV,"JM_MIRROR_Y","-0.007"))
const MIRROR_DX   = parse(Float32, get(ENV,"JM_MIRROR_X","0.0"))

const MIRROR_TILT = deg2rad(parse(Float32, get(ENV,"JM_MIRROR_TILT","0")))
const MIRROR_SCALE = parse(Float32, get(ENV,"JM_MIRROR_SCALE","1.0"))
const MIRROR_SPREAD = parse(Float32, get(ENV,"JM_MIRROR_SPREAD","1.05"))

const WIND_ALPHA   = parse(Float32, get(ENV,"JM_WIND_ALPHA","0.55"))

const WIND_B = parse(Float32, get(ENV,"JM_WIND_B","0.45"))
const WIND_A = parse(Float32, get(ENV,"JM_WIND_A","0.45"))

let spread = MIRROR_SPREAD
    for side in (-1, 1)
        zs = Float32[]
        for p in MIRRORP, i in 1:11:length(p.verts)-10
            sign(p.verts[i+2]) == side && push!(zs, p.verts[i+2])
        end
        isempty(zs) && continue
        dz = (sum(zs)/length(zs)) * (spread - 1f0)
        for p in MIRRORP, i in 1:11:length(p.verts)-10
            sign(p.verts[i+2]) == side && (p.verts[i+2] += dz)
        end
    end
end
const MIRRORMAT = Render.translate(Float32[MIRROR_DX,MIRROR_DY,0]) *
                  Render.translate(MCEN) * Render.rotz(MIRROR_TILT) * Render.scalexyz(MIRROR_SCALE,MIRROR_SCALE,MIRROR_SCALE) * Render.translate(-MCEN)
println(length(TRACK), " track parts + ", length(CARP), " Lotus body parts")
const BODY_OFF = Float32[-0.55, 0.30, 0.0]

const SUSP_GAIN = parse(Float32, get(ENV,"JM_SUSP_GAIN","0.9"))

const CAM_TILT_TAU = parse(Float64, get(ENV,"JM_CAM_TILT_TAU","0.05"))

const WTRACK_F = parse(Float32, get(ENV,"JM_TRACK_F","0.78"))
const WTRACK_R = parse(Float32, get(ENV,"JM_TRACK_R","0.74"))

function hat_height_at(sv)
    np = length(TRKSURF.pos); ld = TRKSURF.lapdist
    i = clamp(searchsortedlast(ld, mod(sv, TRKSURF.lap_length)), 1, np)
    send = i == np ? TRKSURF.lap_length : ld[i+1]
    f = (mod(sv, TRKSURF.lap_length) - ld[i]) / max(send - ld[i], 1e-9)
    j = mod1(i+1, np); TRKSURF.pos[i][2] + (TRKSURF.pos[j][2] - TRKSURF.pos[i][2])*f
end
function hat_at_lapdist(sv, lat)
    np = length(TRKSURF.pos); ld = TRKSURF.lapdist
    i = clamp(searchsortedlast(ld, mod(sv, TRKSURF.lap_length)), 1, np)
    send = i == np ? TRKSURF.lap_length : ld[i+1]
    f = (mod(sv, TRKSURF.lap_length) - ld[i]) / max(send - ld[i], 1e-9)
    j = mod1(i+1, np); pa = TRKSURF.pos[i]; pb = TRKSURF.pos[j]; q = TRKSURF.perp[i]
    (pa[1] + (pb[1]-pa[1])*f + lat*q[1], pa[3] + (pb[3]-pa[3])*f + lat*q[3])
end
if get(ENV,"JM_HATPROBE","") != ""

    spec = get(ENV,"JM_HATPROBE","")
    if startswith(spec, "pt:")

        _f = split(spec, ":"); _px = parse(Float64, _f[2]); _pz = parse(Float64, _f[3])
        _h = JRTrack.hat3d(TERRAIN, _px, _pz; ref=Inf); _hr = JRTrack.hat(TRKSURF, _px, _pz)
        println("== JM_HATPROBE pt (", _px, ",", _pz, "): mesh h=", _h[3] ? round(Float64(_h[1]), digits=3) : NaN,
                "  ribbon found=", _hr.found, " lapdist=", round(_hr.lapdist, digits=2), " lateral=", round(_hr.lateral, digits=2), " ribbon h=", round(_hr.height, digits=3),
                "  spline h=", TRK_CAL.on ? round(trk_road(_hr.lapdist, _hr.lateral), digits=3) : NaN,
                "  spline(lat 0)=", TRK_CAL.on ? round(CLASSICTrack.trk_height(TRKALT, _hr.lapdist, 0.0) + TRK_CAL.off, digits=3) : NaN)
        let _n = length(TRKSURF.pos), _d = [ (TRKSURF.pos[i][1]-_px)^2 + (TRKSURF.pos[i][3]-_pz)^2 for i in 1:_n ], _o = sortperm(_d)
            println("   nearest ribbon nodes: ", join(["#$(i) ld=$(round(TRKSURF.lapdist[i],digits=1)) d=$(round(sqrt(_d[i]),digits=2)) pos=($(round(TRKSURF.pos[i][1],digits=1)),$(round(TRKSURF.pos[i][3],digits=1)))" for i in _o[1:4]], "  "))
            println("   ribbon: n=", _n, " lapdist[1]=", round(TRKSURF.lapdist[1],digits=2), " lapdist[end]=", round(TRKSURF.lapdist[end],digits=2), " lap_length=", round(TRKSURF.lap_length,digits=2), " pos[1]=(", round(TRKSURF.pos[1][1],digits=1), ",", round(TRKSURF.pos[1][3],digits=1), ") seg[1..3]=", TRKSURF.seg[1:3])
        end
        for _s in (0.0, 3.0, 3740.0, 3750.0, 3752.0, 3755.0)
            TRK_CAL.on && println("   spline s=", _s, " lat 0 -> ", round(CLASSICTrack.trk_height(TRKALT, _s, 0.0) + TRK_CAL.off, digits=3), "  lat +3 -> ", round(CLASSICTrack.trk_height(TRKALT, _s, 3.0) + TRK_CAL.off, digits=3), "  lat -3 -> ", round(CLASSICTrack.trk_height(TRKALT, _s, -3.0) + TRK_CAL.off, digits=3))
        end
        flush(stdout); haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "cmp:")

        _f = split(spec, ":"); _s0 = parse(Float64, _f[2]); _s1 = parse(Float64, _f[3]); _st = parse(Float64, _f[4])
        println("== JM_HATPROBE cmp s=$_s0..$_s1 step $_st -- road (physics) minus mesh (drawn), lat -5 -3 0 3 5 ==")
        _n = length(TRKSURF.pos); _ld = TRKSURF.lapdist
        for _s in _s0:_st:_s1
            _i = clamp(searchsortedlast(_ld, _s), 1, _n - 1); _fr = (_s - _ld[_i]) / max(_ld[_i+1] - _ld[_i], 1e-6)
            _p = TRKSURF.pos[_i]; _q = TRKSURF.pos[_i+1]; _pp = TRKSURF.perp[_i]
            _x = _p[1] + (_q[1]-_p[1])*_fr; _z = _p[3] + (_q[3]-_p[3])*_fr
            _out = String[]
            for _lat in (-5.0, -3.0, 0.0, 3.0, 5.0)
                _wx = _x + _lat*_pp[1]; _wz = _z + _lat*_pp[3]
                local _h = JRTrack.hat3d(TERRAIN, _wx, _wz; ref=Inf); _r = ground_road(_wx, _wz)
                push!(_out, (_h[3] && _r[2]) ? string(round(_r[1] - Float64(_h[1]), digits=3)) : "NaN")
            end
            _h0 = JRTrack.hat3d(TERRAIN, _x, _z; ref=Inf)
            println("   cmp s=", round(_s, digits=1), " mesh0=", _h0[3] ? round(Float64(_h0[1]), digits=2) : NaN, " d=", join(_out, " "))
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "fold:")

        _f = split(spec, ":"); _s0 = parse(Float64, _f[2])
        _hw = length(_f) >= 4 ? parse(Float64, _f[3]) : 6.0
        _stp = length(_f) >= 5 ? parse(Float64, _f[4]) : 0.25
        println("== JM_HATPROBE fold around s=", _s0, " +-", _hw, " m, step ", _stp, " m")
        for _lat in (-5.0, -4.5, -4.0, -3.5, -3.0, 0.0)
            println("   lat ", _lat, ":")
            _s = _s0 - _hw
            while _s <= _s0 + _hw
                _w = hat_at_lapdist(_s, _lat)
                _rh = hat_height_at(_s)
                _hm = JRTrack.hat3d(TERRAIN, _w[1], _w[2]; ref = _rh + 3.0)
                _hs = TRK_CAL.on ? CLASSICTrack.trk_height(TRKALT, _s * TRK_SSCALE, TRK_CAL.sign * _lat) + TRK_CAL.off : NaN
                _gr = ground_road(_w[1], _w[2])
                println("      s=", rpad(round(_s, digits=2), 8),
                        " mesh ", rpad(_hm[3] ? round(Float64(_hm[1]), digits=3) : NaN, 8),
                        " spline ", rpad(round(_hs + trk_corr(_s, _lat), digits=3), 8),
                        " ground ", rpad(round(_gr[1], digits=3), 8),
                        " conf ", round(trk_conf(_s, _lat), digits=3))
                _s += _stp
            end
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "vcrease:")

        _f = split(spec, ":"); _lats = parse.(Float64, split(_f[2], ",")); _st = parse(Float64, _f[3])

        _w0 = length(_f) >= 5 ? parse(Float64, _f[4]) : 0.0
        _w1 = length(_f) >= 5 ? parse(Float64, _f[5]) : LAPLEN
        println("== JM_HATPROBE vcrease: drawn-mesh curvature, step ", _st, " m, s=", round(_w0, digits=1),
                "..", round(_w1, digits=1), " of lap ", round(LAPLEN, digits=1), " m")
        _allk = Float64[]
        for _lat in _lats
            hs = Float64[]; miss = 0
            _s = _w0
            while _s < _w1
                hr = hat_at_lapdist(_s, _lat)
                if hr === nothing; miss += 1; push!(hs, NaN)
                else

                    _rh = hat_height_at(_s)
                    h = JRTrack.hat3d(TERRAIN, hr[1], hr[2]; ref = _rh + 3.0)
                    h[3] || (h = JRTrack.hat3d(TERRAIN, hr[1], hr[2]; ref = Inf))
                    push!(hs, h[3] ? Float64(h[1]) : NaN); h[3] || (miss += 1)
                end
                _s += _st
            end
            ks = Float64[]; kat = Float64[]
            for i in 2:length(hs)-1
                (isnan(hs[i-1]) || isnan(hs[i]) || isnan(hs[i+1])) && continue
                push!(ks, abs(hs[i-1] - 2hs[i] + hs[i+1]) / _st^2); push!(kat, _w0 + (i-1)*_st)
            end

            if !isempty(ks)
                _o = sortperm(ks; rev = true)
                _pk = Int[]
                for i in _o; all(j -> abs(kat[i] - kat[j]) > 15.0, _pk) && (push!(_pk, i); length(_pk) >= 8 && break); end
                println("      worst creases at lapdist: ",
                        join((string(round(Int, kat[i]), "m(", round(ks[i], digits=3), ", h=",
                                     round(hs[i+1], digits=2), ")") for i in _pk), "  "))
            end
            _hv = [h for h in hs if !isnan(h)]
            if !isempty(_hv)
                _im = argmax([isnan(h) ? -Inf : h for h in hs])
                _wm = hat_at_lapdist(_w0 + (_im-1)*_st, _lat)
                println("      highest point: lapdist ", round(Int, _w0 + (_im-1)*_st), " m, altitude ",
                        round(hs[_im], digits=2), " m, world (", round(_wm[1], digits=1), ", ", round(_wm[2], digits=1), ")")
            end
            append!(_allk, ks)
            _q(v, pp) = (u = sort(copy(v)); isempty(u) ? NaN : u[clamp(ceil(Int, pp*length(u)), 1, length(u))])
            println("   lat ", lpad(_lat, 5), ": ", length(ks), " nodes (", miss, " off-mesh)  |curvature| 1/m: p50 ",
                    rpad(round(_q(ks, 0.5), digits=5), 8), " p99 ", rpad(round(_q(ks, 0.99), digits=5), 8),
                    " max ", round(maximum(ks; init = NaN), digits=5))
        end
        if !isempty(_allk)
            _q2(v, pp) = (u = sort(copy(v)); u[clamp(ceil(Int, pp*length(u)), 1, length(u))])
            println("   ALL: p50 ", round(_q2(_allk, 0.5), digits=5), " p90 ", round(_q2(_allk, 0.9), digits=5),
                    " p99 ", round(_q2(_allk, 0.99), digits=5), " max ", round(maximum(_allk), digits=5))
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "edge:")

        _ns = parse(Int, split(spec, ":")[2])
        if TRKALT === nothing
            println("== JM_HATPROBE edge: no .trk altitude spline on this track")
        else
            _ta = TRKALT

            _cl = CLASSICTrack.trk_centreline(track_file(CLASSICNAME, ".trk"))
            _al = align_centreline(_cl, ROADHAT)
            _nn = length(_al); _sv = zeros(_nn)
            for i in 2:_nn; _sv[i] = _sv[i-1] + hypot(_al[i][1]-_al[i-1][1], _al[i][2]-_al[i-1][2]); end
            _edge = parse(Float64, get(ENV, "JM_EDGE_LAT", string(abs(_ta.lat[div(length(_ta.lat),2)]))))
            _pts = NTuple{2,Float64}[]
            for k in 0:_ns-1
                t = k/_ns * _sv[end]
                i = clamp(searchsortedlast(_sv, t), 1, _nn-1); f = (t - _sv[i])/max(_sv[i+1]-_sv[i], 1e-9)
                local px = _al[i][1] + (_al[i+1][1]-_al[i][1])*f; local pz = _al[i][2] + (_al[i+1][2]-_al[i][2])*f
                tx = _al[i+1][1]-_al[i][1]; tz = _al[i+1][2]-_al[i][2]; tl = max(hypot(tx,tz), 1e-9)
                push!(_pts, (px - tz/tl*_edge, pz + tx/tl*_edge))
            end

            _dev = Float64[]; _nv = Ref(0)
            for prt in TRACKMAIN
                lt = lowercase(String(prt.tex))
                (startswith(lt, "edge") || startswith(lt, "sline") || startswith(lt, "pline")) || continue
                local v = prt.verts; nvert = length(v) ÷ 11
                for q in 0:nvert-1
                    vx = Float64(v[11q+1]); vz = -Float64(v[11q+3])
                    _nv[] += 1
                    d = Inf
                    for (ex, ez) in _pts
                        dd = (vx-ex)^2 + (vz-ez)^2; dd < d && (d = dd)
                    end
                    push!(_dev, sqrt(d))
                end
            end
            if isempty(_dev)
                println("== JM_HATPROBE edge: no road-edge parts found (textures edge*/sline*/pline*)")
            else
                _qq2(v, pp) = (u = sort(copy(v)); u[clamp(ceil(Int, pp*length(u)), 1, length(u))])
                println("== JM_HATPROBE edge: ", _nv[], " drawn road-edge vertices vs the analytic .trk edge at lat ",
                        round(_edge, digits=2), " m")
                println("   distance to the true edge [m]: p50 ", round(_qq2(_dev,0.5), digits=3),
                        " p90 ", round(_qq2(_dev,0.9), digits=3), " p99 ", round(_qq2(_dev,0.99), digits=3),
                        " max ", round(maximum(_dev), digits=3))
            end
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "road:")

        _f = split(spec, ":"); _lat = parse(Float64, _f[2])
        _s0 = parse(Float64, _f[3]); _s1 = parse(Float64, _f[4]); _st = parse(Float64, _f[5])

        let _sp = [hypot(TRKSURF.pos[mod1(i+1,length(TRKSURF.pos))][1]-TRKSURF.pos[i][1],
                         TRKSURF.pos[mod1(i+1,length(TRKSURF.pos))][3]-TRKSURF.pos[i][3]) for i in 1:length(TRKSURF.pos)]
            _o = sortperm(_sp)
            println("== ribbon: ", length(_sp), " segments  spacing min ", round(minimum(_sp), digits=3),
                    " p05 ", round(sort(_sp)[max(1,round(Int,0.05*length(_sp)))], digits=3),
                    " median ", round(sort(_sp)[div(length(_sp),2)], digits=3),
                    " max ", round(maximum(_sp), digits=3), " m")
            println("   five shortest at lapdist: ", join((string(round(TRKSURF.lapdist[i], digits=1), "(", round(_sp[i], digits=2), "m)") for i in _o[1:min(5,end)]), " "))
        end
        println("== JM_HATPROBE road lat=", _lat, " s=", _s0, "..", _s1, " step ", _st,
                "  (LAPLEN=", round(LAPLEN, digits=2), ", ribbon lapdist[end]=", round(TRKSURF.lapdist[end], digits=2),
                ", .trk lap=", round(TRKALT === nothing ? NaN : TRKALT.total, digits=2), ", sscale=", round(TRK_SSCALE, digits=5), ")")
        _prev = Ref(NaN); _prevg = Ref(NaN)
        _npos = length(TRKSURF.pos); _ldp = TRKSURF.lapdist
        for _s in _s0:_st:_s1
            _hsp = TRK_CAL.on ? CLASSICTrack.trk_height(TRKALT, _s * TRK_SSCALE, TRK_CAL.sign * _lat) + TRK_CAL.off : NaN
            _hc = trk_corr(_s, _lat); _hr2 = trk_road(_s, _lat); _cf = trk_conf(_s, _lat)

            _ii = clamp(searchsortedlast(_ldp, _s), 1, _npos)
            _ee = _ii == _npos ? TRKSURF.lap_length : _ldp[_ii+1]
            _ff = clamp((_s - _ldp[_ii]) / max(_ee - _ldp[_ii], 1e-6), 0.0, 1.0)
            _jj = mod1(_ii+1, _npos); _pa = TRKSURF.pos[_ii]; _pb = TRKSURF.pos[_jj]; _pq = TRKSURF.perp[_ii]
            _wx2 = _pa[1] + (_pb[1]-_pa[1])*_ff + _lat*_pq[1]; _wz2 = _pa[3] + (_pb[3]-_pa[3])*_ff + _lat*_pq[3]
            _gm = JRTrack.hat3d(TERRAIN, _wx2, _wz2; ref=Inf)
            _gr = ground_road(_wx2, _wz2)
            _hq = JRTrack.hat(TRKSURF, _wx2, _wz2)

            JRTrack.hat_polyline!(true); _hp = JRTrack.hat(TRKSURF, _wx2, _wz2); JRTrack.hat_polyline!(false)
            println("   s=", rpad(round(_s, digits=2), 8), " spline=", rpad(round(_hsp, digits=3), 7),
                    " corr=", rpad(round(_hc, digits=3), 7), " road(s,lat)=", rpad(round(_hr2, digits=3), 7),
                    " conf=", rpad(round(_cf, digits=3), 6),
                    " | mesh=", rpad(_gm[3] ? round(Float64(_gm[1]), digits=3) : NaN, 8),
                    " driven=", rpad(round(_gr[1], digits=3), 7),
                    " xz=(", round(_wx2, digits=2), ",", round(_wz2, digits=2), ")",
                    " C1(ld=", round(_hq.lapdist, digits=2), ", lat=", round(_hq.lateral, digits=2), ")",
                    " poly(ld=", round(_hp.lapdist, digits=2), ", lat=", round(_hp.lateral, digits=2), ")",
                    " d(driven)=", isnan(_prevg[]) ? "-" : string(round(_gr[1] - _prevg[], digits=4)))
            _prev[] = _hr2; _prevg[] = _gr[1]
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "drv:")

        _f = split(spec, ":"); _latl = parse.(Float64, split(_f[2], ",")); _v = parse(Float64, _f[3])
        _arms = split(get(ENV, "JM_HATPROBE_ARMS", "keep"), ",")
        for _arm in _arms, _lat in _latl
        _arm == "keep" || (STENCIL_GUARD[] = 'S' in _arm; LEDGE_GUARD[] = 'L' in _arm)
        _dt = 1/60; _np = length(TRKSURF.pos); local _ld = TRKSURF.lapdist
        _cr(p0,p1,p2,p3,t) = 0.5*(2p1 + (-p0+p2)*t + (2p0-5p1+4p2-p3)*t^2 + (-p0+3p1-3p2+p3)*t^3)
        _crd(p0,p1,p2,p3,t) = 0.5*((-p0+p2) + 2t*(2p0-5p1+4p2-p3) + 3t*t*(-p0+3p1-3p2+p3))
        _hh = Float64[]; _sl = Float64[]; _miss = Ref(0)
        for _k in 0:floor(Int, LAPLEN/(_v*_dt))
            _s = _k*_v*_dt
            _i = clamp(searchsortedlast(_ld, _s), 1, _np); _e = _i == _np ? LAPLEN : _ld[_i+1]
            _t = clamp((_s - _ld[_i]) / max(_e - _ld[_i], 1e-6), 0.0, 1.0)
            _h4 = mod1(_i-1, _np); _j4 = mod1(_i+1, _np); _k4 = mod1(_i+2, _np)
            _a = TRKSURF.pos[_h4]; _b = TRKSURF.pos[_i]; _c = TRKSURF.pos[_j4]; _d = TRKSURF.pos[_k4]
            _x = _cr(_a[1],_b[1],_c[1],_d[1],_t); _z = _cr(_a[3],_b[3],_c[3],_d[3],_t)
            _tx = _crd(_a[1],_b[1],_c[1],_d[1],_t); _tz = _crd(_a[3],_b[3],_c[3],_d[3],_t)
            _tl = max(hypot(_tx, _tz), 1e-9)
            _px2 = _x + _lat*(-_tz/_tl); _pz2 = _z + _lat*(_tx/_tl)
            _r = ground_road(_px2, _pz2)
            _r[2] ? push!(_hh, _r[1]) : (_miss[] += 1; push!(_hh, isempty(_hh) ? 0.0 : _hh[end]))
            push!(_sl, _s)
        end
        _w = [(_hh[i+1]-_hh[i])/_dt for i in 1:length(_hh)-1]
        _st2 = [abs(_w[i+1]-_w[i]) for i in 1:length(_w)-1]
        _qq(v, p) = (u = sort(copy(v)); u[clamp(ceil(Int, p*length(u)), 1, length(u))])
        _wi = argmax(_st2)
        println("== JM_HATPROBE drv arm=", _arm, " lat=", _lat, " v=", _v, " frame=", get(ENV, "JM_HAT_POLYLINE", "0") == "0" ? "C1 curve" : "POLYLINE (control)",
                " -- vertical-velocity step per 1/60 s frame [m/s]:")
        println("   p50 ", round(_qq(_st2, 0.5), digits=4), "  p90 ", round(_qq(_st2, 0.9), digits=4),
                "  p99 ", round(_qq(_st2, 0.99), digits=4), "  max ", round(maximum(_st2), digits=4),
                "  (worst at s=", round(_sl[_wi], digits=1), " m)  frames ", length(_st2), ", off-road samples ", _miss[])

        let _ord = sortperm(_st2, rev=true), _seen = Float64[]
            for _k in _ord
                any(abs(_sl[_k] - _p2) < 8.0 for _p2 in _seen) && continue
                push!(_seen, _sl[_k]); length(_seen) > 5 && break
                println("     s=", round(_sl[_k], digits=1), "  step ", round(_st2[_k], digits=2),
                        " m/s   h ", round(_hh[_k], digits=3), " -> ", round(_hh[_k+1], digits=3), " -> ", round(_hh[_k+2], digits=3),
                        "   spline confidence ", round(trk_conf(_sl[_k], _lat), digits=3))
            end
        end
        for _sa in parse.(Float64, filter(!isempty, split(get(ENV, "JM_HATPROBE_AT", ""), ",")))
            _ix = [i for i in eachindex(_st2) if abs(_sl[i] - _sa) <= 10.0]
            isempty(_ix) || println("     at s=", _sa, " +-10: max step ", round(maximum(_st2[_ix]), digits=2), " m/s")
        end
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif startswith(spec, "cl:")

        _f = split(spec, ":"); _s0 = parse(Float64, _f[2]); _s1 = parse(Float64, _f[3]); _st = parse(Float64, _f[4])
        println("== JM_HATPROBE centreline s=$_s0..$_s1 step $_st -- physics ground height along the ribbon ==")
        _n = length(TRKSURF.pos); _ld = TRKSURF.lapdist
        _prevh = Ref(NaN); _prevsl = Ref(NaN)
        for _s in _s0:_st:_s1
            _i = clamp(searchsortedlast(_ld, _s), 1, _n - 1); _fr = (_s - _ld[_i]) / max(_ld[_i+1] - _ld[_i], 1e-6)
            _p = TRKSURF.pos[_i]; _q = TRKSURF.pos[_i+1]
            _x = _p[1] + (_q[1]-_p[1])*_fr; _z = _p[3] + (_q[3]-_p[3])*_fr
            local _h = JRTrack.hat3d(TERRAIN, _x, _z; ref=Inf)
            _hh = _h[3] ? Float64(_h[1]) : NaN
            if _h[3] && haskey(ENV, "JM_HATPROBE_TRK") && TRK_CAL.on
                _hh = ground_road(_x, _z)[1]
            end
            if _h[3] && haskey(ENV, "JM_HATPROBE_SMOOTH")
                _hdg = atan(_q[3]-_p[3], _q[1]-_p[1]); _sp = parse(Float64, get(ENV, "JM_GROUND_SMOOTH", "1.0"))
                _acc = _hh; _na = 1
                for _d in (-2.0, -1.0, 1.0, 2.0)
                    _h2 = JRTrack.hat3d(TERRAIN, _x + cos(_hdg)*_d*_sp, _z + sin(_hdg)*_d*_sp; ref=Inf)
                    _h2[3] && (_acc += Float64(_h2[1]); _na += 1)
                end
                _hh = _acc/_na
            end
            _sl = isnan(_prevh[]) ? NaN : (_hh - _prevh[])/_st
            _dsl = (isnan(_sl) || isnan(_prevsl[])) ? NaN : (_sl - _prevsl[])
            println("   s=", lpad(round(_s, digits=1), 8), "  x=", lpad(round(_x, digits=2), 9), " z=", lpad(round(_z, digits=2), 9),
                    "  h=", isnan(_hh) ? "  NO SURFACE" : lpad(round(_hh, digits=3), 8),
                    "  slope=", isnan(_sl) ? "   -" : lpad(round(_sl, digits=4), 8),
                    "  dslope=", isnan(_dsl) ? "   -" : lpad(round(_dsl, digits=4), 8))
            _prevh[] = _hh; _prevsl[] = _sl
        end
        flush(stdout)
        haskey(ENV, "JM_HATPROBE_EXIT") && exit(0)
    elseif occursin(";", spec)
        println("== JM_HATPROBE path -- physics ground height along the car's track ==")
        for tok in split(spec, ";")
            isempty(strip(tok)) && continue
            local pr = split(tok, ",")
            local px = parse(Float64, strip(pr[1]))
            local pz = parse(Float64, strip(pr[2]))
            h = JRTrack.hat3d(TERRAIN, px, pz; ref=Inf)
            println("   (", lpad(round(px,digits=1),9), ",", lpad(round(pz,digits=1),9), ")  ",
                    h[3] ? string("ground ", round(h[1], digits=2)) : "NO SURFACE  <-- HOLE")
        end
    else
        pr = split(spec, ",")
        px = parse(Float64, strip(pr[1])); pz = parse(Float64, strip(pr[2]))
        println("== JM_HATPROBE around (", px, ", ", pz, ") -- physics ground height, 4 m grid ==")
        for dz in -12:4:12
            row = String[]
            for dx in -12:4:12
                h = JRTrack.hat3d(TERRAIN, px+dx, pz+dz; ref=Inf)
                push!(row, h[3] ? string(round(h[1], digits=2)) : "  --  ")
            end
            println("   dz=", lpad(dz,4), "  ", join([lpad(r,8) for r in row]))
        end
    end
    flush(stdout)
end

const WHEELS_TABLE = (( 1.05f0, WTRACK_F,true, 0.31f0,"lotwlf"), ( 1.05f0,-WTRACK_F,true, 0.31f0,"lotwrf"),
                      (-1.15f0, WTRACK_R,false,0.34f0,"lotwlr"), (-1.15f0,-WTRACK_R,false,0.34f0,"lotwrr"))
const WHEELS = let hubs = get(ENV, "JM_WHEELS_TABLE", "0") == "0" ? (try Render.mesh_wheel_hubs(LOT3DO) catch e; @warn "mesh_wheel_hubs (Lotus)" e; nothing end) : nothing
    if hubs === nothing
        println("  [wheels] Lotus: hand table (", join(["($(w[1]),$(w[2]))" for w in WHEELS_TABLE], " "), ")")
        WHEELS_TABLE
    else
        w = Tuple(Render.wheelspec_from_mesh(hubs, collect(WHEELS_TABLE), BODY_OFF[1], BODY_OFF[3]))
        println("  [wheels] Lotus: hubs from lotus.3do  front x=", round(hubs.fx, digits=2), " rear x=", round(hubs.rx, digits=2),
                " half-track ", round(hubs.fy, digits=2), "/", round(hubs.ry, digits=2), " (+BODY_OFF ", BODY_OFF[1], ")  now ",
                join(["($(round(x[1],digits=2)),$(round(x[2],digits=2)))" for x in w], " "))
        w
    end
end

const AXLE_X0, AXLE_ZHUB = let hand = get(ENV,"JM_AXLE_HAND","0") != "0"
    if hand
        (-1.15f0, parse(Float32, get(ENV,"JM_TRACK_R","0.74")))
    else
        (Float32(WHEELS[3][1] - BODY_OFF[1]), Float32(abs(WHEELS[3][2] - BODY_OFF[3])))
    end
end
const AXLEP = get(ENV,"JM_AXLES","1") != "0" ?
    [_axle_part(1f0, AXLE_X0, AXLE_ZHUB), _axle_part(-1f0, AXLE_X0, AXLE_ZHUB)] : Render.TrackPart[]

const FSUSP_SYN = get(ENV,"JM_FSUSP_SYN","1") != "0" ? let w = WHEELS[1]
        xh = Float32(w[1] - BODY_OFF[1]); yh = Float32(w[4] - BODY_OFF[2]); zh = Float32(abs(w[2] - BODY_OFF[3]))
        zi = 0.18f0; zo = zh - 0.07f0; tube = (0.25f0, 0.65f0)
        out = Render.TrackPart[]
        for s in (1f0, -1f0)
            v = Float32[]
            _rod_verts!(v, Float32[xh - 0.05f0, yh + 0.10f0, s*zi], Float32[xh, yh + 0.08f0, s*zo], 0.016f0, tube)
            _rod_verts!(v, Float32[xh + 0.13f0, yh - 0.07f0, s*zi], Float32[xh, yh - 0.08f0, s*zo], 0.011f0, tube)
            _rod_verts!(v, Float32[xh - 0.15f0, yh - 0.07f0, s*zi], Float32[xh, yh - 0.08f0, s*zo], 0.011f0, tube)
            push!(out, Render.TrackPart(v, "axlelot", (1f0,1f0,1f0)))
        end
        println("  [fsusp-syn] front hub x=", round(xh,digits=3), " y=", round(yh,digits=3), " z=+-", round(zh,digits=3),
                "  rods to |z| ", round(zo,digits=3))
        out
    end : Render.TrackPart[]
if !isempty(AXLEP)
    let b = Render.parts_bbox(AXLEP)
        println("  [axle] x0=", round(AXLE_X0,digits=3), " hub z=", round(AXLE_ZHUB,digits=3),
                "  bbox x[", round(b.xmin,digits=3), ",", round(b.xmax,digits=3),
                "] z[", round(b.zmin,digits=3), ",", round(b.zmax,digits=3), "]",
                "  (rear hub raw x=", round(WHEELS[3][1] - BODY_OFF[1], digits=3), ")")
    end
end

if get(ENV,"JM_TINT_PART","") != ""
    let idx = parse(Int, get(ENV,"JM_TINT_PART","0"))
        if 1 <= idx <= length(CARP)

            v = copy(CARP[idx].verts)
            for k in 0:(length(v) ÷ 11 - 1)
                v[11k+9] = 1f0; v[11k+10] = 0f0; v[11k+11] = 1f0
            end
            CARP[idx] = Render.TrackPart(v, CARP[idx].tex, (1f0, 0f0, 1f0))
            println("  [tint] CARP#", idx, " -> magenta on ", length(v) ÷ 11, " vertices (tex=\"",
                    CARP[idx].tex, "\")"); flush(stdout)
        end
    end
end

if get(ENV,"JM_OUTBOARD_CENSUS","0") != "0"
    let half = Float32(WTRACK_R), hubz = Float32(AXLE_Y), rows = Any[]

        for (setname, parts) in (("CARP", CARP), ("CARPIN", CARPIN), ("AXLEP", AXLEP),
                                 ("FSUSPP", FSUSPP), ("DRIVERP", DRIVERP), ("MIRRORP", MIRRORP),
                                 ("WINDP", WINDP), ("GAUGEP", GAUGEP), ("HELMP", HELMP))
            for (i, prt) in enumerate(parts)
                v = prt.verts
                n = length(v) ÷ 11
                n == 0 && continue
                zmax = -Inf32; ymin = Inf32; xlo = Inf32; xhi = -Inf32
                for k in 0:n-1
                    x = v[11k+1]; y = v[11k+2]; z = v[11k+3]
                    zmax = max(zmax, abs(z)); ymin = min(ymin, y)
                    xlo = min(xlo, x); xhi = max(xhi, x)
                end

                if zmax > half + 0.02f0 || ymin < hubz - 0.25f0
                    push!(rows, (zmax - half, setname, i, prt.tex, n ÷ 3, zmax, ymin, xlo, xhi))
                end
            end
        end
        sort!(rows, by = r -> -r[1])
        println("  [outboard] half-track=", round(half,digits=3), " hub y=", round(hubz,digits=3),
                "  parts reaching past them: ", length(rows))
        for r in rows[1:min(end,12)]
            println("    ", r[2], "#", r[3], " tex=", r[4], " tris=", r[5],
                    "  |z|max=", round(r[6],digits=3), " (", round(r[1],digits=3), " outboard)",
                    "  ymin=", round(r[7],digits=3), "  x[", round(r[8],digits=2), ",", round(r[9],digits=2), "]")
        end
        flush(stdout)
        get(ENV,"JM_CENSUS_EXIT","0") != "0" && exit(0)
    end
end

const FULLSCREEN = get(ENV, "JM_FULLSCREEN", "0") != "0"
const W, H = let r = lowercase(strip(get(ENV, "JM_RES", ""))), m = match(r"^(\d{3,5})x(\d{3,5})$", r)
    if m !== nothing
        (parse(Int, m[1]), parse(Int, m[2]))
    elseif FULLSCREEN || r == "native"
        GLFW.Init()
        vm = GLFW.GetVideoMode(GLFW.GetPrimaryMonitor())
        (Int(vm.width), Int(vm.height))
    else
        (1440, 810)
    end
end

const HUDBAND_ON = get(ENV, "JM_HUDBAND", "1") != "0"
const BAND_H = HUDBAND_ON ? round(Int, clamp(parse(Float64, get(ENV, "JM_HUDBAND_FRAC", "0.27"))*H, 150.0, 0.45H)) : 0
const H3 = H - BAND_H

const BAND_HIDDEN = Ref(get(ENV, "JM_BAND_HIDDEN", "0") == "1"); const BAND_PF = Ref(false)
band_shown() = HUDBAND_ON && !BAND_HIDDEN[]
band_px() = band_shown() ? BAND_H : 0
view_h() = H - band_px()
const MSAA = clamp(parse(Int, get(ENV, "JM_MSAA", "8")), 0, 16)
println("  display: ", W, "x", H, FULLSCREEN ? " fullscreen" : " window", ", MSAA ", MSAA)

const OBJ_CULLFACE = get(ENV,"JM_OBJ_CULL","0") != "0"
const STARTBOX_CULL = get(ENV,"JM_STARTBOX_CULL","1") != "0"

const TWINBOARD_CULL = get(ENV, "JM_TWINBOARD_CULL", "1") != "0"

const _LAYOFF = split(get(ENV, "JM_LAYERS_OFF", ""), ",")
OBJBOUND = Tuple{NTuple{3,Float32},Float32}[]
const LAYOFF_OBJ = "obj" in _LAYOFF; const LAYOFF_BB = "bb" in _LAYOFF; const LAYOFF_HZ = "hz" in _LAYOFF
const TWINBOARD = Dict{String,Bool}()
function twinboard_classify!(nm, parts)
    (TWINBOARD_CULL && !haskey(TWINBOARD, nm)) || return
    P = NTuple{3,Float64}[]; Nn = NTuple{3,Float64}[]; Cc = NTuple{3,Float64}[]; Ar = Float64[]; Tx = String[]
    for pp in parts, k in 1:33:length(pp.verts)-32
        v = pp.verts
        a = (Float64(v[k]), Float64(v[k+1]), Float64(v[k+2])); b = (Float64(v[k+11]), Float64(v[k+12]), Float64(v[k+13])); c = (Float64(v[k+22]), Float64(v[k+23]), Float64(v[k+24]))
        e1 = b .- a; e2 = c .- a; n = (e1[2]*e2[3]-e1[3]*e2[2], e1[3]*e2[1]-e1[1]*e2[3], e1[1]*e2[2]-e1[2]*e2[1])
        l = sqrt(sum(n .^ 2)); l > 1e-9 || continue
        push!(Nn, n ./ l); push!(Cc, (a .+ b .+ c) ./ 3); push!(Ar, l/2); push!(Tx, pp.tex)
    end
    n = length(Ar); (2 <= n <= 400) || return
    tot = sum(Ar); paired = falses(n); copl = falses(n)
    for i in 1:n, j in i+1:n
        (Tx[i] == Tx[j] && Ar[i] > 0.05 && Ar[j] > 0.05) || continue
        sum(Nn[i] .* Nn[j]) < -0.99 || continue
        d = Cc[j] .- Cc[i]; sep = sum(d .* Nn[i])

        abs(sep) < 3.0 || continue

        sqrt(max(sum(d .^ 2) - sep^2, 0.0)) < max(2.0, sqrt(max(Ar[i], Ar[j]))) || continue
        paired[i] = paired[j] = true
        abs(sep) <= 0.2 && (copl[i] = copl[j] = true)
    end
    frac = sum(Ar[paired]) / tot
    ctr = (sum(Cc[i][1]*Ar[i] for i in 1:n), sum(Cc[i][2]*Ar[i] for i in 1:n), sum(Cc[i][3]*Ar[i] for i in 1:n)) ./ tot
    outw = sum(Ar[i] * sign(sum(Nn[i] .* (Cc[i] .- ctr))) for i in 1:n if paired[i]; init = 0.0)
    istwin = frac >= 0.4
    get(ENV, "JM_TWINBOARD_DIAG", "0") != "0" && frac > 0 &&
        println("  [twinboard] ", rpad(nm, 12), " tris ", n, "  back-to-back area ", round(frac, digits=2),
                "  outward(CCW) ", round(outw / max(sum(Ar[paired]), 1e-9), digits=2), istwin ? "  -> CULLED" : "")

    istwin && (TWINBOARD[nm] = (sum(Ar[copl]; init = 0.0) > 0.5*sum(Ar[paired])) ? true : outw >= 0)
    nothing
end
const OBJ_FF_CW    = get(ENV,"JM_OBJ_FF","cw") == "cw"

const OBJ_CULL2 = parse(Float32, get(ENV, "JM_OBJ_CULL_D", "2200"))^2
const FRUSTUM_CULL = get(ENV, "JM_FRUSTUM_CULL", "1") != "0"

@inline function infrustum(vp::AbstractMatrix{Float32}, pos, r::Float32)
    x = Float32(pos[1]); y = Float32(pos[2]); z = Float32(pos[3])
    cx = vp[1,1]*x + vp[1,2]*y + vp[1,3]*z + vp[1,4]
    cy = vp[2,1]*x + vp[2,2]*y + vp[2,3]*z + vp[2,4]
    cw = vp[4,1]*x + vp[4,2]*y + vp[4,3]*z + vp[4,4]
    cw < -r && return false

    @inbounds for i in 1:2
        c = i == 1 ? cx : cy
        for sg in (1f0, -1f0)
            ax = vp[4,1] + sg*vp[i,1]; ay = vp[4,2] + sg*vp[i,2]; az = vp[4,3] + sg*vp[i,3]
            cw + sg*c < -r * sqrt(ax*ax + ay*ay + az*az) && return false
        end
    end
    true
end

const BB_LIT    = get(ENV,"JM_BILLBOARD_LIT","0") != "0"
const BB_BRIGHT = parse(Float32, get(ENV,"JM_BB_BRIGHT","1.55"))
const BB_AMB    = parse(Float32, get(ENV,"JM_BB_AMB","0.85"))

const MINPIX_K = Float32(parse(Float64, get(ENV, "JM_MINPIX", "1.5")) / 700.0)

const MINPIX_K_MIR = Float32(parse(Float64, get(ENV, "JM_MINPIX", "1.5")) / 120.0)
const BB_CULL2  = 1300f0^2

const AI_BRIGHT = parse(Float64, get(ENV, "JM_AI_BRIGHT", "1.25"))
const AI_AMB    = parse(Float64, get(ENV, "JM_AI_AMB", "0.78"))
const MIR_OBJ_CULL2 = parse(Float32, get(ENV, "JM_MIRROR_OBJ_CULL_D", "450"))^2
const MIR_BB_CULL2  = parse(Float32, get(ENV, "JM_MIRROR_BB_CULL_D",  "250"))^2
const SMOKE = haskey(ENV, "JM_SMOKE")
const SMOKE_FRAMES = parse(Int, get(ENV, "JM_SMOKE_FRAMES", "40"))

const REC_DIR = let d = get(ENV, "JM_IBT_DIR", SMOKE ? joinpath(tempdir(), "jm_smoke_rec") : joinpath(dirname(dirname(@__DIR__)), "data", "juliaracer"))
    mkpath(d); d
end

struct SmokeShot; s::Float64; view::Int; name::String; env::Vector{Pair{String,String}}; end
const SHOTS = [let f = split(String(spec), ":")
                   SmokeShot(parse(Float64, f[1]), length(f) >= 2 ? parse(Int, f[2]) : 0,
                             length(f) >= 3 ? String(f[3]) : "shot$(i)",
                             length(f) >= 4 ? [String(k) => String(v) for (k, v) in (split(kv, "=") for kv in split(f[4], ",", keepempty = false))] : Pair{String,String}[])
               end for (i, spec) in enumerate(filter(!isempty, split(get(ENV, "JM_SHOTS", ""), ";")))]
const SHOTS_DIR   = get(ENV, "JM_SHOTS_DIR", "/tmp")

const CRASH_SPEC   = get(ENV, "JM_CRASH", "")
const CRASH_FRAMES = parse(Int, get(ENV, "JM_CRASH_FRAMES", "150"))
const CRASH_ON     = haskey(ENV, "JM_SMOKE") && !isempty(CRASH_SPEC)
const CRASHES      = NTuple{4,Float64}[]

struct TimedShot; when::String; view::Int; name::String; end
const SHOTS_AT = [let f = split(String(spec), ":")
                      TimedShot(String(f[1]), length(f) >= 2 ? parse(Int, f[2]) : -1,
                                length(f) >= 3 ? String(f[3]) : "tshot$(i)")
                  end for (i, spec) in enumerate(filter(!isempty, split(get(ENV, "JM_SHOT_AT", ""), ";")))]
const SHOTS_AT_ARMED = fill(-1, length(SHOTS_AT))
const FRAMEDUMP = (v = get(ENV, "JM_FRAMEDUMP", "");
                   isempty(v) ? Int[] : [parse(Int, t) for t in split(v, ":")])
const FRAMEDUMP_DIR = get(ENV, "JM_FRAMEDUMP_DIR", "/home/admin/appimage-build/frames")

const CAR_RANGE = (v = get(ENV, "JM_CAR_RANGE", "");
                   isempty(v) ? nothing : (parse(Int, split(v,":")[1]), parse(Int, split(v,":")[2])))
_in_car_range(i) = CAR_RANGE === nothing || (CAR_RANGE[1] <= i <= CAR_RANGE[2])

const SHOT_SETTLE = parse(Int, get(ENV, "JM_SHOT_SETTLE", "38"))

const WHEELGAP = parse(Int, get(ENV, "JM_WHEELGAP", "0"))
const PACE_ON  = get(ENV, "JM_PACE", "1") != "0"
const DSTAT = get(ENV, "JM_DRAWSTATS", "0") != "0"; const GPUSYNC = get(ENV, "JM_GPUSYNC", "0") != "0"
const DS_LOOP = Ref(0.0); const DS_PREV = Ref(0.0); const DS_SIM = zeros(2); const DS_T0 = Ref(0.0); const DS_SIMACC = zeros(3); const DS_T = zeros(6); const DS_ACC = zeros(5); const DS_N = zeros(Int, 6); const DS_NA = zeros(Int, 6); const DS_CNT = Ref(0)
const ALLOCSITES = get(ENV, "JM_ALLOCSITES", "0") != "0"
const ALLOCPROF = parse(Int, get(ENV, "JM_ALLOCPROF", "0"))
const ALLOC_MARK = zeros(Int, 5); const ALLOC_ACC = zeros(Int, 4); const ALLOC_N = Ref(0)
const PACE_HI  = parse(Float64, get(ENV, "JM_PACE_HI", "19.0")) / 1000
const PACE_LO  = parse(Float64, get(ENV, "JM_PACE_LO", "15.0")) / 1000
const PACE_EMA = Ref(0.0); const PACE_INT = Ref(1); const FPS_DTS = Float64[]
const GC_EVERY = parse(Int, get(ENV, "JM_GC_EVERY", "0")); const GC_MS = zeros(3)
const PACE_WARM = parse(Int, get(ENV, "JM_PACE_WARM", "120"))
get(ENV, "JM_GCLOG", "0") != "0" && GC.enable_logging(true)
const FPSDIAG = parse(Int, get(ENV, "JM_FPSDIAG", "0"))

const STALLLOG = get(ENV, "JM_STALLLOG", "1") != "0"
const STALL_MS = parse(Float64, get(ENV, "JM_STALL_MS", "30"))
STALLLOG && Base.cumulative_compile_timing(true)
const STALL_PREV = UInt64[0, 0, 0]
const STALL_N = Ref(0); const STALL_COMP_MS = Ref(0.0)
const FRAME_FPS = Ref(0.0)
const PERF4_MARK = Ref(0)

@noinline perf4_reveal_marker() = (PERF4_MARK[] += 1; PERF4_MARK[])
const HUD_BV = Float32[]; const HUD_TV = Float32[]; const HUD_ST = Float32[]
const HUD_TY = Float32[]; const HUD_TYT = Float32[]

const GPUQ = parse(Int, get(ENV, "JM_GPUQ", "1"))
const GPUQ_DIAG = parse(Int, get(ENV, "JM_GPUQ_DIAG", "0"))
const GPUQ_FENCE = Ref{Ptr{Cvoid}}(C_NULL); const GPUQ_WAIT = zeros(3)
const FRAMEPROF = parse(Int, get(ENV, "JM_FRAMEPROF", "0"))
const PROF_WORLD = Ref(0.0); const PROF_HUD = Ref(0.0); const PROF_N = Ref(0); const PROF_TOT = Ref(0.0)

const PROF_DEPTH = Ref(0.0); const PROF_TRACK = Ref(0.0); const PROF_OBJ = Ref(0.0); const PROF_BB = Ref(0.0); const PROF_CARS = Ref(0.0)

const BND_PK = Ref(0.0)
const BND_NX = Ref(0.0); const BND_NZ = Ref(0.0)

const BND_NL = Ref(0.0)
const BND_VN = Ref(0.0)
const BND_OFF = Ref(0.0)

const STUCK_SECS = parse(Float64, get(ENV, "JM_STUCK_SECS", "20"))

const TRACE_POS = haskey(ENV, "JM_TRACE_POS")
const POS_LAST  = Ref(-1)
const TRAIL_SECS = parse(Float64, get(ENV, "JM_WRECK_TRAIL", "4.0"))
const TRAIL_HZ   = 10.0

const TRACE_DS  = haskey(ENV, "JM_TRACE_DS")
const DS_RATIO  = Vector{Float64}()
const DS_RAW    = Vector{Float64}()
const DS_PREV   = Ref(-1.0)
function ds_note!(ds, v, dt, total)
    ds > total/2  && (ds -= total)
    ds < -total/2 && (ds += total)
    push!(DS_RAW, ds)
    exp_step = max(v*dt, 1e-6)
    push!(DS_RATIO, abs(ds)/exp_step)
    if length(DS_RATIO) % 500 == 0
        r = sort(DS_RATIO); n = length(r)
        q(p) = r[max(1, min(n, round(Int, p*n)))]
        a = sort(abs.(DS_RAW))
        aq(p) = a[max(1, min(n, round(Int, p*n)))]
        println("[ds] n=$n  |ds| med=$(round(aq(0.5), digits=3)) p99=$(round(aq(0.99), digits=3)) " *
                "max=$(round(a[end], digits=3)) m  |  ratio med=$(round(q(0.5), digits=2)) " *
                "p99=$(round(q(0.99), digits=2)) p999=$(round(q(0.999), digits=2)) " *
                "max=$(round(q(1.0), digits=2))")
        flush(stdout)
    end
    return nothing
end
const TRAIL_BUF  = Vector{NTuple{8,Float64}}()
const TRAIL_LAST = Ref(-1.0e9)
const DMG_EVERY  = parse(Float64, get(ENV, "JM_DMG_EVERY", "1.0"))
const DMG_LAST_T = Ref(-1.0e9)
const DMG_RUN_N  = Ref(0)
const STUCK_T0   = Ref(-1.0)
const WHEEL_REST   = parse(Float64, get(ENV, "JM_WHEEL_REST", "0.35"))
const WRECK_KMH    = parse(Float64, get(ENV, "JM_WRECK_KMH", "50.0"))
const WRECK_MS     = WRECK_KMH/3.6

const WRECK_CLOSE  = parse(Float64, get(ENV, "JM_WRECK_CLOSE", "12.0"))

const WHEEL_TEAR = let f = get(ENV, "JM_WHEEL_TEAR", "10.0,0.25")
    f == "0" ? (Inf, Inf) : (parse(Float64, split(f, ",")[1]), parse(Float64, split(f, ",")[2]))
end

const BACKFLIP_MODE = get(ENV, "JM_BACKFLIP_OLD", "0") == "1" ? 2 : 1
const TEAR_DIAG = get(ENV, "JM_TEARDIAG", "0") == "1"
const WRECKED      = Ref(false)
const WRECK_FROZEN = Ref(false)

const CRASHBODY = get(ENV, "JM_CRASHBODY", "1") != "0"
const WBODY = Ref{Union{Nothing,WreckBody.Body}}(nothing)

const CRASHBODY_DIAG = get(ENV, "JM_CRASHBODY_DIAG", "0") != "0"

const AIRBODY = get(ENV, "JM_AIRBODY", "1") != "0"
const AIRBODY_RH = parse(Float64, get(ENV, "JM_AIRBODY_RH", "0.40"))
const AIRBORNE = Ref(false)

const WB_TELE = Ref((r = 0.0, ax = 0.0, ay = 0.0, az = 9.80665))
const AIR_LAND = Ref(0.0)
const AIR_PEAK = Ref((0.0, 0.0))
const _ATTR_GET = IdDict{Any,Any}()
"""The driving model's roll rate (left side up +), pitch rate (nose up +) and heave rate (up +), for a handover."""
function attrates3d(cs)
    g = get(_ATTR_GET, cs.sys, nothing)
    if g === nothing
        MT = DriveRT3D.ModelingToolkit
        g = _ATTR_GET[cs.sys] = (MT.getsym(cs.sys, cs.sys.pp), MT.getsym(cs.sys, cs.sys.q), MT.getsym(cs.sys, cs.sys.w))
    end
    (Float64(g[1](cs.integ)), Float64(g[2](cs.integ)), Float64(g[3](cs.integ)))
end
const WBODY_PLANE = Ref{WreckBody.Plane}(WreckBody.Plane(0.0))

const LOOSE_BODY = Union{Nothing,WreckBody.Body}[nothing for _ in 1:LOOSE_N]
const LOOSE_BPLANE = [WreckBody.Plane(0.0) for _ in 1:LOOSE_N]
const LOOSE_LAND = zeros(LOOSE_N)
const LOOSE_DOWN = zeros(LOOSE_N)
const LOOSE_PEAK = [(0.0, 0.0) for _ in 1:LOOSE_N]
const LOOSE_UPRIGHT = parse(Float64, get(ENV, "JM_LOOSE_UPRIGHT", "5.0"))
"""World velocity (vx, vz) of pool slot `sl`'s car: its rigid body's while it flies, else its driving model's."""
loose_wvel(sl::Int) = (b = LOOSE_BODY[sl]; b === nothing ? DriveRT3D.world_velocity(LOOSE_CARS[sl]) : (b.v[1], -b.v[3]))
loose_yawrate(sl::Int) = (b = LOOSE_BODY[sl]; b === nothing ? Float64(DriveRT3D.yawrate3d(LOOSE_CARS[sl])) : (b.R * b.ω)[2])
"Wheel `nm`'s drawn rotation: its own spin while the wreck is a rigid body, else the shared driving spin."
wheel_spin_angle(nm, spin) = (WBODY[] === nothing || isempty(nm)) ? spin :
                             (i = findfirst(==(nm), WHEEL_NAMES); i === nothing ? spin : -WBODY[].angle[i])
const WRECK_PX     = Ref(0.0); const WRECK_PZ = Ref(0.0); const WRECK_PT = Ref(0.0)
const WRECK_PH = Ref(NaN); const WRECK_PIN = get(ENV, "JM_WRECK_PIN", "1") != "0"
const WRECK_DAMP   = parse(Float64, get(ENV, "JM_WRECK_DAMP", "6.0"))

const LooseWheel = Tuple{Float64,Float64,Float64,Float64,Float64,Float64,Float64,Float64,String}
const LOOSE_WHEELS = Vector{LooseWheel}()
is_loose(nm) = any(w -> w[9] == nm, LOOSE_WHEELS)
const WHEEL_NAMES  = ("lotwlf","lotwrf","lotwlr","lotwrr")

"""E95: latch the wreck — engine PERMANENTLY disconnected. Wheels are NOT chosen here.
PO 2026-08-29: "detach the wheel that hit the barrier - or both wheels, if two wheels hit before
the car comes to a stop". So which wheels leave is decided per-wheel, per-frame, by whether that
CORNER actually contacts something (see detach_hit_wheels!) — not by picking corners up front."""
function wreck!(v; closing = NaN, bnd_peak = NaN, x = NaN, z = NaN, cause = "")
    WRECKED[] && return
    WRECKED[] = true
    println("  [WRECK] hard impact at ", round(v*3.6, digits=0), " km/h — engine disconnected, race over")

    closes = !isnan(closing)  && closing  > WRECK_CLOSE
    bnds   = !isnan(bnd_peak) && bnd_peak > 1.0e3
    println("  [WRECK]   cause: ", !isempty(cause) ? cause :
            bnds && !closes ? "BOUNDARY penetration peak $(round(bnd_peak, digits=0)) -- this is the " *
                              "fence/wall test, which is INVISIBLE geometry, not a drawn object" :
            closes && !bnds ? "CLOSING speed $(round(closing, digits=1)) m/s into a solid" :
            closes && bnds  ? "BOTH: closing $(round(closing, digits=1)) m/s and boundary peak $(round(bnd_peak, digits=0))" :
                              "neither threshold exceeded (closing=$closing bnd_peak=$bnd_peak) -- unexpected")
    isnan(x) || println("  [WRECK]   at world (", round(x, digits=1), ", ", round(z, digits=1), ")")

    bnds && println("  [WRECK]   boundary: ", round(BND_NL[], digits=3), " m past the edge",
                    " (FENCE_GRACE=", FENCE_GRACE, " m), inward-normal speed ",
                    round(BND_VN[], digits=2), " m/s, off-track distance ",
                    round(BND_OFF[], digits=1), " m")

    if TRAIL_SECS > 0.0 && !isempty(TRAIL_BUF)
        t0 = TRAIL_BUF[end][1]
        println("  [WRECK]   approach (last ", round(t0 - TRAIL_BUF[1][1], digits=1),
                " s, 10 Hz):  t-Δ    x       z      v km/h   slip°   off m   lat m   lapdist")
        prev = nothing
        for e in TRAIL_BUF
            (t, x, z, vv, th, off, lat, lap) = e
            slip = NaN
            if prev !== nothing
                dx = x - prev[2]; dz = z - prev[3]
                if dx*dx + dz*dz > 1.0e-6
                    course = atan(dz, dx)
                    d = course - th
                    while d >  pi; d -= 2pi; end
                    while d < -pi; d += 2pi; end
                    slip = abs(d) * 180/pi
                end
            end
            println("  [WRECK]     ", lpad(round(t - t0, digits=1), 5), "  ",
                    lpad(round(x, digits=1), 7), " ", lpad(round(z, digits=1), 7), "  ",
                    lpad(round(vv*3.6, digits=0), 6), "  ",
                    lpad(isnan(slip) ? "  -" : string(round(slip, digits=0)), 6), "  ",
                    lpad(round(off, digits=1), 6), "  ",
                    lpad(isnan(lat) ? "   -" : string(round(lat, digits=1)), 6), "  ",
                    lpad(isnan(lap) ? "    -" : string(round(Int, lap)), 7))
            prev = e
        end
    end
    flush(stdout)
end

"""E95: detach any still-attached wheel whose HUB is inside a solid. Runs every frame while the car
is still moving, so a second corner that reaches the barrier later comes off too — which is what
happens at higher impact speed, and is exactly what the PO described. Deliberately per-corner: at a
glancing hit only the corner that touched leaves the car."""
function detach_hit_wheels!(x, z, θ, v; fence_nx = 0.0, fence_nz = 0.0, fence = false)
    cθ, sθ = cos(θ), sin(θ)

    if fence
        @inbounds for (bx, by, front, r, nm) in WHEELS
            is_loose(nm) && continue
            wx = x + bx*cθ - by*sθ
            wz = z + bx*sθ + by*cθ

            lead = (wx - x)*(-fence_nx) + (wz - z)*(-fence_nz)
            lead <= 0.35 && continue

            vwx = v*cθ; vwz = v*sθ
            vn  = vwx*fence_nx + vwz*fence_nz
            tgx = vwx - vn*fence_nx; tgz = vwz - vn*fence_nz
            outn = abs(vn)*WHEEL_REST
            push!(LOOSE_WHEELS, (wx, 0.33, wz,
                                 tgx*0.7 + fence_nx*outn,
                                 2.5 + 0.10*abs(v),
                                 tgz*0.7 + fence_nz*outn,
                                 0.0, v/max(r, 0.1), nm))
            println("  [WRECK] ", nm, " torn off on the wall at ", round(abs(v)*3.6, digits=0), " km/h")
            flush(stdout)
        end
    end
    @inbounds for (i, (bx, by, front, r, nm)) in enumerate(WHEELS)
        is_loose(nm) && continue
        wx = x + bx*cθ - by*sθ
        wz = z + bx*sθ + by*cθ
        for k in eachindex(SOLIDS)
            kind = SOLIDS[k][4]
            kind === :soft && continue
            (gap, nx, nz) = solid_gap(wx, wz, k)
            gap >= 0.35 && continue
            push!(LOOSE_WHEELS, (wx, 0.33, wz,
                                 v*cθ + nx*3.5,
                                 3.0 + 0.15*abs(v),
                                 v*sθ + nz*3.5,
                                 0.0, v/max(r, 0.1), nm))
            println("  [WRECK] ", nm, " torn off at ", round(abs(v)*3.6, digits=0), " km/h")
            flush(stdout)
            break
        end
    end
end

"""E95: ballistic + bounce integration for the torn-off wheels. `gz` gives ground height. `W`: the player's (default) or
the AI cars' (AIWHEELOFF-1) list."""
function step_loose_wheels!(dt, gz, W::Vector{LooseWheel} = LOOSE_WHEELS)
    isempty(W) && return
    @inbounds for i in eachindex(W)
        (x,y,z,vx,vy,vz,sp,sr,nm) = W[i]
        vy -= 9.81*dt
        x += vx*dt; y += vy*dt; z += vz*dt
        g = gz(x, z) + 0.33
        if y <= g
            y = g
            vy = -vy*0.35
            vx *= 0.86; vz *= 0.86
            abs(vy) < 0.4 && (vy = 0.0)
        end
        sp += sr*dt
        sr *= (1.0 - 0.25*dt)
        W[i] = (x,y,z,vx,vy,vz,sp,sr,nm)
    end
end
include(joinpath(@__DIR__, "ai_wheeloff.jl"))
include(joinpath(@__DIR__, "wheel_hits.jl"))
const LOOSE_WALLS = WreckBody.WallPlane[]
const CLU_LAST = Ref(-1.0)

const CLU_NOW    = Ref(0.0)
const STALL_RPM  = parse(Float64, get(ENV, "JM_STALL_RPM",  "300.0"))
const STALL_SECS = parse(Float64, get(ENV, "JM_STALL_SECS", "0.5"))
const STALL_T    = Ref(0.0)
const FPS_T0 = Ref(0.0); const FPS_ACC = Ref(0.0); const FPS_N = Ref(0)
const CAR3D = !haskey(ENV, "JM_2D")

build_carX(; kw...)        = CAR3D ? DriveRT3D.build_car3d(; kw...) : DriveRT.build_car(; kw...)
step_carX!(c, a...; kw...) = CAR3D ? DriveRT3D.step_car3d!(c, a...; kw...) : DriveRT.step_car!(c, a...; kw...)
telemetryX(c)              = CAR3D ? DriveRT3D.telemetry3d(c) : DriveRT.telemetry(c)
respawnX!(c; groundz=nothing) = CAR3D ? DriveRT3D.respawn3d!(c; groundz=groundz) : DriveRT.respawn!(c)
containX!(c, x, z; kw...)  = CAR3D ? DriveRT3D.contain3d!(c, x, z; kw...) : DriveRT.contain!(c, x, z; kw...)
bumpX!(c, dvx, dvz, dr, dvy=0.0, dpp=0.0, dq=0.0) = CAR3D ? DriveRT3D.bump3d!(c, dvx, dvz, dr, dvy, dpp, dq) : DriveRT.bump!(c, dvx, dvz, dr)

player_yawrate(cs) = WBODY[] === nothing ? (CAR3D ? Float64(DriveRT3D.yawrate3d(cs)) : 0.0) : (WBODY[].R * WBODY[].ω)[2]
const WBODY_KICKS = Ref(0)
function player_bump!(cs, dvx, dvz, dr, dvy=0.0, dpp=0.0, dq=0.0)
    b = WBODY[]
    b === nothing && return bumpX!(cs, dvx, dvz, dr, dvy, dpp, dq)
    WreckBody.kick!(b, WreckBody.V3(dvx, dvy, -dvz), Float64(dr))
    (WBODY_KICKS[] += 1) <= 20 && (println("  [crashbody] struck by a car: dv ", round(hypot(dvx, dvz), digits = 2), " m/s, yaw ",
                                           round(dr, digits = 2), " rad/s"); flush(stdout))
    nothing
end

const AICarT  = DriveRT3D.Car3D
const AIbuild = DriveRT3D.build_cars3d
const AIyaw   = DriveRT3D.yawrate3d
const AIbump! = DriveRT3D.bump3d!
const AIplace! = DriveRT3D.place3d!
const FENCE = parse(Float64, get(ENV, "JM_FENCE", "13.0"))
const RACEEND_SECS = parse(Float64, get(ENV, "JM_RACEEND_SECS", "12"))
const RACEEND_T0 = Ref(-1.0)
const RUNOFF_W = parse(Float64, get(ENV, "JM_RUNOFF_W", "20.0"))
const FENCE_GRACE = parse(Float64, get(ENV, "JM_FENCE_GRACE", "2.5"))
const FENCE_FAR  = parse(Float64, get(ENV, "JM_FENCE_FAR", "16.0"))

const GRASS_DRAG = parse(Float64, get(ENV, "JM_GRASS_DRAG", "0.30"))
const GRASS_SLIP = parse(Float64, get(ENV, "JM_GRASS_SLIP", "0.15"))

const GRASS_DRAG_K = parse(Float64, get(ENV, "JM_GRASS_DRAG_K", "0.0047"))
const GRASS_N = Ref(0)

const WIND_V = parse(Float64, get(ENV, "JM_WIND", "0")); const WIND_DIR = deg2rad(parse(Float64, get(ENV, "JM_WIND_DIR", "0")))
WIND_V != 0 && println("  [wind] ", WIND_V, " m/s from ", rad2deg(WIND_DIR), " deg")
const WIND_DIAG = parse(Float64, get(ENV, "JM_WIND_DIAG", "0")); const WIND_DIAG_T = Ref(-Inf)
const GRASS_DIAG = parse(Float64, get(ENV, "JM_GRASS_DIAG", "0")); const GRASS_DIAG_T = Ref(-Inf)
const GRASS_MU   = clamp(parse(Float64, get(ENV, "JM_GRASS_MU", "0.5")), 0.1, 1.0)

const WETNESS = let w = clamp(something(tryparse(Int, get(ENV, "JM_WETNESS", "1")), 1), 1, 7)
    (SKIDPAD || w in 2:5) ? 1 : w
end
const WET_A    = WETNESS == 7 ? 0.78 : WETNESS == 6 ? 0.855 : 1.0
const WET_V0   = 66.0
const WET_PACE = WETNESS == 7 ? 0.845 : WETNESS == 6 ? 0.862 : 1.0
"Wet tyre-friction factor at speed v [m/s] (1 when dry)."
wet_mu(v::Real) = WET_A == 1.0 ? 1.0 : WET_A/(1.0 + (abs(Float64(v))/WET_V0)^4)
WETNESS > 1 && println("  [wet] track wetness ", WETNESS, WETNESS == 7 ? " (extremely wet)" : " (very wet)", ": tyre grip x",
                       round(wet_mu(20.0), digits = 2), " at 72 km/h, x", round(wet_mu(60.0), digits = 2), " at 216 km/h; AI pace x", WET_PACE)
const ROAD_HALFW = parse(Float64, get(ENV, "JM_ROAD_HALFW", "9.0"))

const SOLID_EXCL_HW = parse(Float64, get(ENV, "JM_SOLID_EXCL_HW", "4.0"))

const _YAWNEG = true
const OBJ_YAW_SIGN = parse(Float64, get(ENV, "JM_OBJ_YAW_SIGN", _YAWNEG ? "-1" : "1"))
const GSTAND_YAW = deg2rad(parse(Float64, get(ENV, "JM_GSTAND_YAW", (TRACKSEL == "spa" && OBJ_YAW_SIGN > 0) ? "90" : "0")))

const OBJ_YAW_ADD = deg2rad(parse(Float64, get(ENV, "JM_OBJ_YAW_ADD", "0")))

const SEC_TWOSIDED = get(ENV, "JM_SEC_TWOSIDED", "0") != "0"
const OBJ_USE_RECZ = get(ENV, "JM_OBJ_RECZ", "1") != "0"
const OBJ_RECZ_TOL = parse(Float64, get(ENV, "JM_OBJ_RECZ_TOL", "0.5"))

const GRANDL_YAW = deg2rad(parse(Float64, get(ENV, "JM_GRANDL_YAW", OBJ_YAW_SIGN > 0 ? "180" : "0")))

const OBJYAW_FIX = Dict(lowercase(String(strip(split(x, "=")[1]))) => deg2rad(parse(Float64, split(x, "=")[2]))
                        for x in split(get(ENV, "JM_OBJYAW_FIX", ""), ","; keepempty = false))
objyawfix(nm) = (startswith(lowercase(nm), "gstand") ? GSTAND_YAW : lowercase(nm) == "grandl" ? GRANDL_YAW : 0.0) + OBJ_YAW_ADD + get(OBJYAW_FIX, lowercase(nm), 0.0)

const OBJ_YAW_FLIP = Set(split(lowercase(get(ENV, "JM_OBJ_YAW_FLIP", (TRACKSEL != "nurburgring" && OBJ_YAW_SIGN < 0) ? "" : "gstand")), ","; keepempty = false))
objyawsign(nm) = lowercase(nm) in OBJ_YAW_FLIP ? -1.0 : 1.0

const OBJ_MIRROR = get(ENV, "JM_OBJ_MIRROR", "1") != "0"
const KEEP_GRASS = haskey(ENV, "JM_KEEP_GRASS")
println(CAR3D ? "  PHYSICS: full-3D vehicle (default) — heave/pitch/roll + suspension travel + jumps" :
                "  PHYSICS: planar 2-D model (JM_2D)")
GLFW.Init()
GLFW.WindowHint(GLFW.VISIBLE, false)
GLFW.WindowHint(GLFW.CONTEXT_VERSION_MAJOR, 4); GLFW.WindowHint(GLFW.CONTEXT_VERSION_MINOR, 5)
GLFW.WindowHint(GLFW.OPENGL_PROFILE, GLFW.OPENGL_CORE_PROFILE)
GLFW.WindowHint(GLFW.OPENGL_FORWARD_COMPAT, true)
GLFW.WindowHint(GLFW.SAMPLES, MSAA)
GLFW.WindowHint(GLFW.RESIZABLE, false)

win = GLFW.CreateWindow(W, H, "Julia Racer — $(uppercasefirst(TRACKSEL))" * (FULLSCREEN ? "" : " (loading…)"))
go_fullscreen!() = (FULLSCREEN && GLFW.GetWindowMonitor(win).handle == C_NULL &&
                    GLFW.SetWindowMonitor(win, GLFW.GetPrimaryMonitor(), 0, 0, W, H, GLFW.DONT_CARE); nothing)
GLFW.MakeContextCurrent(win); GLFW.SwapInterval(1)

const KEY_LATCH = Dict{GLFW.Key,Int}()

GLFW.SetKeyCallback(win, (w, k, sc, act, mods) -> (if act == GLFW.PRESS && (k == GLFW.KEY_I || k == GLFW.KEY_LEFT || k == GLFW.KEY_ESCAPE)
                                                       KEY_LATCH[k] = get(KEY_LATCH, k, 0) + 1
                                                       println("  [key] ", k == GLFW.KEY_I ? "I" : k == GLFW.KEY_LEFT ? "left arrow" : "ESC",
                                                               " down at +", round(time() - _T0, digits = 1), " s"); flush(stdout)
                                                   end; nothing))
took_key!(k) = (n = get(KEY_LATCH, k, 0); n > 0 && (KEY_LATCH[k] = 0); n > 0)
const IPRESS_SEEN = Ref(false)
const HUD_UNDER_BAND = get(ENV, "JM_HUD_UNDER_BAND", "0") == "1"
const STOP_T = Ref(-1.0)
glEnable(GL_DEPTH_TEST); glEnable(GL_MULTISAMPLE)
glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE)
glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

const prog = Render.program(); glUseProgram(prog)
glUniform3f(Render.uloc(prog,"uLightDir"), 0.4f0, 1.0f0, 0.25f0)
skyprog = Render.skyprogram(); skyvao = Render.empty_vao()
hudprog = Render.hud_program(); (hudvao, hudvbo) = Render.hud_buffers()

const TEXT_HUD = !haskey(ENV, "JM_NO_TEXT_HUD")
textprog = Render.text_program(); (textvao, textvbo) = Render.hud_buffers()
FONT = TEXT_HUD ? Render.load_font(joinpath(@__DIR__, "assets"), 18) : nothing
TEXT_HUD && FONT === nothing && println("  [texthud] no font atlas under demo/native/assets -- text overlay off (run JuliaMotorMTK/tools/make_font_atlas.py)")

include(joinpath(@__DIR__, "track_sections.jl"))
const SEGNAME_SECS = parse(Float64, get(ENV, "JM_SEGNAME_SECS", "6.0"))
const SEGFONT = FONT === nothing ? nothing : something(Render.load_font(joinpath(@__DIR__, "assets"), 32), FONT)
const SECTIONS = isdefined(Main, :LAPLEN) ? track_sections(TRACKSEL, LAPLEN) : Tuple{Float64,String}[]
const SEG = Ref((0, "", -1.0e9))
const SEGNAME_FORCE = get(ENV, "JM_SEGNAME_FORCE", "")
isempty(SECTIONS) || println("  [trackseg] ", length(SECTIONS), " named sections for ", TRACKSEL)

const TEXTHUD_DRIVER = Dict("Ferrari" => "C Amon", "Brabham" => "J Brabham", "BRM" => "J Stewart",
                            "Eagle" => "D Gurney", "Cooper" => "J Bonnier", "Lotus" => "J Clark", "Honda" => "J Surtees")
lapfmt(t) = t <= 0 ? "--:--.--" : begin
    m = floor(Int, t/60); sr = t - 60m; sec = floor(Int, sr); cc = clamp(floor(Int, (sr - sec)*100), 0, 99)
    string(lpad(m, 2, '0'), ":", lpad(sec, 2, '0'), ".", lpad(cc, 2, '0')) end
include(joinpath(@__DIR__, "hud_timing.jl"))
include(joinpath(@__DIR__, "hud_band.jl"))
include(joinpath(@__DIR__, "tyre_temp.jl"))
depthprog = Render.depthprogram(); (shadowfbo, shadowtex) = Render.make_shadow_fbo()
const LIGHTDIR = Float32[0.4, 1.0, 0.25]

struct ColourGrade
    zenith::NTuple{3,Float32}; horizon::NTuple{3,Float32}
    cloud::Float32
    suncol::NTuple{3,Float32}; ambsky::NTuple{3,Float32}; sat::Float32
    ringtint::NTuple{3,Float32}
end
const GRADE_CLASSIC   = ColourGrade((0.40,0.56,0.78),(0.78,0.78,0.75),1.0, (1,1,1),(0.95,0.93,0.86),1.0, (1,1,1))
const GRADE_SUNNY = ColourGrade((0.20,0.47,0.85),(0.80,0.88,0.97),1.0, (1.07,1.0,0.85),(0.72,0.82,0.99),1.18, (1.28,1.27,1.30))
const GRADE_SKIDPAD = ColourGrade((0.20,0.42,0.78),(0.62,0.74,0.88),0.18, (1,1,1),(0.95,0.93,0.86),1.0, (1,1,1))

const GRADE_ZAND  = ColourGrade((0.24,0.46,0.80),(0.74,0.83,0.92),0.45, (1.07,1.0,0.85),(0.70,0.80,0.98),1.30, (1.12,1.10,1.06))

const GRADE_SPA   = ColourGrade((0.40,0.56,0.70),(0.80,0.85,0.90),0.50, (1.07,1.0,0.86),(0.76,0.83,0.94),1.34, (1.10,1.13,1.05))
const GRADE_SPA_OLD = ColourGrade((0.22,0.45,0.82),(0.72,0.82,0.93),0.50, (1.07,1.0,0.85),(0.70,0.80,0.98),1.34, (1.10,1.13,1.05))

const GRADE_MONZA = ColourGrade((0.40,0.57,0.68),(0.86,0.89,0.92),0.42, (1.07,1.02,0.90),(0.74,0.83,0.95),1.16, (1.17,1.15,1.11))
const GRADE_MONZA_OLD = ColourGrade((0.31,0.51,0.80),(0.85,0.90,0.96),0.34, (1.07,1.02,0.90),(0.74,0.83,0.97),1.18, (1.17,1.15,1.11))

const GRADE_WATK  = ColourGrade((0.60,0.66,0.74),(0.80,0.82,0.84),0.28, (1.08,1.02,0.90),(0.82,0.85,0.90),1.12, (1.24,1.15,1.03))
const GRADE_WATK_OLD = ColourGrade((0.28,0.48,0.80),(0.82,0.87,0.93),0.40, (1.10,1.02,0.86),(0.74,0.82,0.96),1.26, (1.21,1.16,1.06))

const GRADE_NURB  = ColourGrade((0.38,0.48,0.62),(0.72,0.76,0.80),0.70, (1.02,1.00,0.92),(0.74,0.80,0.90),1.10, (1.02,1.05,1.00))
const GRADE_NURB_OLD = ColourGrade((0.42,0.48,0.56),(0.66,0.68,0.70),1.0, (0.92,0.92,0.95),(0.72,0.74,0.80),0.88, (0.90,0.91,0.96))

const GRADE_OVERCAST = ColourGrade((0.66,0.67,0.69),(0.76,0.765,0.77),1.0, (1.0,1.0,0.98),(0.88,0.89,0.92),1.12, (1.10,1.10,1.10))

const ZAND_CLOUD = parse(Float32, get(ENV,"JM_CLOUD","0.18"))
const GRADE_ZANDOVER = ColourGrade(GRADE_OVERCAST.zenith, GRADE_OVERCAST.horizon, ZAND_CLOUD,
                                   GRADE_OVERCAST.suncol, GRADE_OVERCAST.ambsky, GRADE_OVERCAST.sat,
                                   GRADE_OVERCAST.ringtint)
const GRADE_BYTRACK = Dict("nurburgring"=>GRADE_NURB, "monza"=>GRADE_MONZA, "spa"=>GRADE_SPA,
                           "watglen"=>GRADE_WATK, "zandvoort"=>GRADE_ZANDOVER)
const GRADE_TAB = Dict("OVERCAST"=>GRADE_OVERCAST, "NURB"=>GRADE_NURB, "MONZA"=>GRADE_MONZA,
                       "SPA"=>GRADE_SPA, "WATK"=>GRADE_WATK, "WATKOLD"=>GRADE_WATK_OLD,
                       "NURBOLD"=>GRADE_NURB_OLD, "MONZAOLD"=>GRADE_MONZA_OLD,
                       "SPAOLD"=>GRADE_SPA_OLD, "ZAND"=>GRADE_ZAND,
                       "SUNNY"=>GRADE_SUNNY, "Classic"=>GRADE_CLASSIC, "SKIDPAD"=>GRADE_SKIDPAD)

const GRADE0 = SKIDPAD ? GRADE_SKIDPAD :
              haskey(ENV, "JM_GRADE") ? get(GRADE_TAB, uppercase(ENV["JM_GRADE"]), GRADE_OVERCAST) :
              get(GRADE_BYTRACK, TRACKSEL, GRADE_OVERCAST)
const ENG = EngineAudio.build_lotus()

if !haskey(ENV,"JM_EXPOSURE")

    Render.EXPOSURE[] = 1.0f0
end
const GRADE_DRY = haskey(ENV,"JM_SAT") ?
    ColourGrade(GRADE0.zenith, GRADE0.horizon, GRADE0.cloud, GRADE0.suncol, GRADE0.ambsky,
                parse(Float32, ENV["JM_SAT"]), GRADE0.ringtint) : GRADE0

const GRADE = (WETNESS > 1 && get(ENV, "JM_WET_LOOK", "1") != "0") ? let g = GRADE_DRY, k = WETNESS >= 7 ? 0.8f0 : 0.65f0,
        mixg(c, t) = ntuple(i -> Float32(c[i] + k*(t[i] - c[i])), 3), sc(c, f) = ntuple(i -> Float32(c[i]*f), 3)
    ColourGrade(mixg(g.zenith, (0.46f0, 0.48f0, 0.51f0)), mixg(g.horizon, (0.62f0, 0.63f0, 0.65f0)), 1.0f0,
                sc(g.suncol, 1 - 0.45k), sc(g.ambsky, 1 - 0.2k), Float32(g.sat*(1 - 0.25k)), sc(g.ringtint, 1 - 0.3k))
end : GRADE_DRY
tstamp("texture load begins"); print("loading textures… "); flush(stdout)
const TEXIDX = Render.classic_texture_index(ZD)
tstamp("  texture INDEX built")

const TRACK_BRIGHT = parse(Float32, get(ENV,"JM_TRACK_BRIGHT","0.72"))
const TRACK_AMB    = parse(Float32, get(ENV,"JM_TRACK_AMB","0.34"))

const TERR_B, TERR_A = let v = parse.(Float32, split(get(ENV, "JM_TERRAIN_GRADE", NURB ? "1.25,1.4" : "1,1"), ","))
    (v[1], v[end])
end

const LAPWIN = parse(Float64, get(ENV, "JM_LAPWIN", (NURB || WATGLEN || MONZA || SPA || ZANDV) ? "914" : "0"))
const LAPBIN = 100.0
const _LW = (LAPWIN > 0 && (@isdefined TRKSURF)) ? let P = TRKSURF.pos, c = 50.0
        xs = [p[1] for p in P]; zs = [p[3] for p in P]; x0 = minimum(xs) - 1; z0 = minimum(zs) - 1
        g = Dict{Tuple{Int,Int},Vector{Int}}()
        for i in eachindex(P); push!(get!(g, (floor(Int, (xs[i]-x0)/c), floor(Int, (zs[i]-z0)/c)), Int[]), i); end
        (xs = xs, zs = zs, S = TRKSURF.lapdist, g = g, x0 = x0, z0 = z0, c = c, L = TRKSURF.lap_length)
    end : nothing
"lap distance of the centreline node nearest the horizontal point (Classic x, y) -- any distance from the road"
function lapnear(x, z)
    W = _LW; cx = floor(Int, (x - W.x0)/W.c); cz = floor(Int, (z - W.z0)/W.c); bd = Inf; bi = 0
    for r in 0:400
        for dx in -r:r, dz in -r:r
            (abs(dx) == r || abs(dz) == r) || continue
            v = get(W.g, (cx + dx, cz + dz), nothing); v === nothing && continue
            for i in v; d = (W.xs[i] - x)^2 + (W.zs[i] - z)^2; d < bd && (bd = d; bi = i); end
        end
        bi > 0 && bd <= (r*W.c)^2 && break
    end
    bi == 0 ? NaN : W.S[bi]
end
lapwin_in(s, c) = (L = _LW.L; abs(mod(s - c + L/2, L) - L/2) <= LAPWIN + LAPBIN)

const LAPWIN_TWOLEG = get(ENV, "JM_LAPWIN_TWOLEG", "1") != "0"

const LAPWIN_NEAR = parse(Float64, get(ENV, "JM_LAPWIN_NEAR", (SPA || ZANDV || NURB) ? "150" : "75"))
"(lap distance, plan distance²) of the centreline node nearest (Classic x, y) -- `lapnear` with its distance"
function lapnear_d(x, z)
    W = _LW; cx = floor(Int, (x - W.x0)/W.c); cz = floor(Int, (z - W.z0)/W.c); bd = Inf; bi = 0
    for r in 0:400
        for dx in -r:r, dz in -r:r
            (abs(dx) == r || abs(dz) == r) || continue
            v = get(W.g, (cx + dx, cz + dz), nothing); v === nothing && continue
            for i in v; d = (W.xs[i] - x)^2 + (W.zs[i] - z)^2; d < bd && (bd = d; bi = i); end
        end
        bi > 0 && bd <= (r*W.c)^2 && break
    end
    bi == 0 ? (NaN, Inf) : (W.S[bi], bd)
end
"the lap distance of the OTHER leg near (x, z) (see above), NaN when there is none"
function lapnear2(x, z, s1, d1sq)
    (LAPWIN_TWOLEG && isfinite(s1)) || return NaN
    W = _LW; L = W.L; far = LAPWIN + LAPBIN; lim = max(1.6*sqrt(d1sq) + 25.0, LAPWIN_NEAR)
    cx = floor(Int, (x - W.x0)/W.c); cz = floor(Int, (z - W.z0)/W.c); bd = lim^2; bi = 0
    for r in 0:(ceil(Int, lim/W.c) + 1)
        for dx in -r:r, dz in -r:r
            (abs(dx) == r || abs(dz) == r) || continue
            v = get(W.g, (cx + dx, cz + dz), nothing); v === nothing && continue
            for i in v
                abs(mod(W.S[i] - s1 + L/2, L) - L/2) > far || continue
                d = (W.xs[i] - x)^2 + (W.zs[i] - z)^2; d < bd && (bd = d; bi = i)
            end
        end
    end
    bi == 0 ? NaN : W.S[bi]
end
const LAPWIN_SPRITES = get(ENV, "JM_LAPWIN_SPRITES", "1") != "0"
const BBLAP = Float64[]; const BBLAP2 = Float64[]
const OBJLAP2 = Float64[]
const LAPNB = _LW === nothing ? 0 : ceil(Int, _LW.L / LAPBIN)
const NDUP2 = Ref(0)
const TRACKLAPOFF = Vector{Int32}[]
const OBJLAP = Float64[]

const TRACKBAND = get(ENV, "JM_TRACKBAND", "1") != "0" && LAPWIN >= 312.0 + LAPBIN
const TRACKBAND_NEAR = 117.0; const TRACKBAND_MID = 312.0
const TRACKBAND_N = Ref((tagged = 0, untagged = 0, trees = 0)); const NBANDDROP = Ref(0)
if _LW !== nothing
    let t0 = time(), L = _LW.L
        bin(s) = clamp(floor(Int32, mod(s, L) / LAPBIN), Int32(0), Int32(LAPNB - 1))

        tri = map(TRACK) do tp
            v = tp.verts; nt = length(v) ÷ 33
            SL = Vector{Float64}(undef, nt); S2 = similar(SL); tg = zeros(Int32, nt)
            for t in 0:nt-1
                o = 33t
                cx = (v[o+1] + v[o+12] + v[o+23]) / 3; cz = (v[o+3] + v[o+14] + v[o+25]) / 3
                sl, dsq = lapnear_d(cx, -cz)
                s2 = lapnear2(cx, -cz, sl, dsq)
                SL[t+1] = sl; S2[t+1] = s2
                if TRACKBAND
                    k = Render.CLASSIC3DO.bandkey((v[o+1], -v[o+3], v[o+2]), (v[o+12], -v[o+14], v[o+13]), (v[o+23], -v[o+25], v[o+24]))
                    tg[t+1] = max(get(Render.CLASSIC3DO.BANDTAG, k, Int32(0)), Int32(0))
                end
            end
            (SL, S2, tg)
        end
        tpos = Dict{Int32,Vector{Float64}}()
        for (sl, _, tg) in tri, t in eachindex(tg); tg[t] > 0 && !isnan(sl[t]) && push!(get!(() -> Float64[], tpos, tg[t] >> 3), sl[t]); end
        tree = Dict{Int32,Float64}()
        for (k, s) in tpos
            m = atan(sum(sin, 2pi .* s ./ L), sum(cos, 2pi .* s ./ L)) * L / 2pi
            q = sort!(mod.(s .- m .+ L/2, L) .- L/2); tree[k] = m + q[(length(q) + 1) ÷ 2]
        end
        TRACKBAND_N[] = (tagged = sum(count(>(0), x[3]) for x in tri), untagged = sum(count(==(0), x[3]) for x in tri), trees = length(tree))
        for (pi, tp) in enumerate(TRACK)
            v = tp.verts; nt = length(v) ÷ 33; sl, s2, tg = tri[pi]
            banded = any(>(0), tg); NB = Int32(LAPNB)
            b = Vector{Int32}(undef, nt); dup = Tuple{Int,Int32}[]
            seen = Set{NTuple{9,Float32}}()
            for t in 0:nt-1
                g = tg[t+1]; m = g & Int32(7); s = m > 0 ? get(tree, g >> 3, sl[t+1]) : sl[t+1]
                bs = isnan(s) ? Int32(0) : bin(s)
                if count_ones(m) > 1
                    o = 33t; key = ntuple(j -> v[o + 11*((j-1)÷3) + (j-1)%3 + 1], 9)
                    key in seen && (b[t+1] = Int32(-1); NBANDDROP[] += 1; continue)
                    push!(seen, key)
                end
                first = true
                for bd in Int32(0):Int32(3)
                    (bd == 0 ? m == 0 : m & (Int32(1) << (bd - 1)) != 0) || continue
                    first ? (b[t+1] = bs + bd*NB; first = false) : push!(dup, (t, bs + bd*NB))
                    isnan(s2[t+1]) || (push!(dup, (t, bin(s2[t+1]) + bd*NB)); NDUP2[] += 1)
                end
            end
            if !isempty(dup)
                resize!(v, 33*(nt + length(dup)))
                for (k, (t, bb)) in enumerate(dup); copyto!(v, 33*(nt + k - 1) + 1, v, 33t + 1, 33); push!(b, bb); end
                nt += length(dup)
            end
            ord = sortperm(b; alg = MergeSort)
            nd = count(<(0), b)
            w = similar(v); for (k, t) in enumerate(ord); copyto!(w, 33(k-1) + 1, v, 33(t-1) + 1, 33); end
            resize!(v, length(w) - 33nd); copyto!(v, 1, w, 33nd + 1, length(v))
            off = zeros(Int32, (banded ? 4LAPNB : LAPNB) + 1); for x in b; x >= 0 && (off[x + 2] += 3); end
            push!(TRACKLAPOFF, cumsum(off))
        end
        println("  [lapwin] track drawn ", LAPWIN, " m ahead/behind along the lap (Classic); ", length(TRACK), " parts in ",
                LAPNB, " bins of ", LAPBIN, " m (", round(time() - t0, digits = 1), " s; JM_LAPWIN=0 reverts); ",
                NDUP2[], " triangles near two legs of the lap drawn with either (JM_LAPWIN_TWOLEG=0 off)")
        TRACKBAND && println("  [trackband] ", TRACKBAND_N[].tagged, " track triangles in ", TRACKBAND_N[].trees, " segment trees drawn one band ",
                             "by lap distance (near ", TRACKBAND_NEAR, " m, mid ", TRACKBAND_MID, " m), ", TRACKBAND_N[].untagged,
                             " always, ", NBANDDROP[], " copies shared by bands drawn once (JM_TRACKBAND=0: every band); parser ", Render.CLASSIC3DO.BANDSTAT[])

        if get(ENV, "JM_LAPWIN_WHO", "") != ""
            _f = split(ENV["JM_LAPWIN_WHO"], ":"); c = parse(Float64, _f[1]); R = length(_f) >= 2 ? parse(Float64, _f[2]) : 250.0
            P = TRKSURF.pos; Sd = TRKSURF.lapdist; k = argmin(abs.(Sd .- c)); ex, ez = P[k][1], P[k][3]
            who = Dict{String,Tuple{Int,Float64,Set{Int}}}()
            for (pi, tp) in enumerate(TRACK)
                v = tp.verts; off = TRACKLAPOFF[pi]
                for b in 0:length(off)-2
                    lapwin_in((b % LAPNB)*LAPBIN + LAPBIN/2, c) && continue
                    for t in div(off[b+1], 3):div(off[b+2], 3)-1
                        o = 33t; cx = (v[o+1] + v[o+12] + v[o+23])/3; cz = -(v[o+3] + v[o+14] + v[o+25])/3
                        d = hypot(cx - ex, cz - ez); d <= R || continue
                        w = get(who, tp.tex, (0, Inf, Set{Int}())); push!(w[3], round(Int, b*LAPBIN))
                        who[tp.tex] = (w[1] + 1, min(w[2], d), w[3])
                    end
                end
            end
            println("  [lapwin who] eye at s ", round(Int, Sd[k]), ": hidden within ", R, " m, by texture (tris, nearest m, bins):")
            for (t, w) in sort(collect(who), by = x -> x[2][2]); println("      ", rpad(t, 12), lpad(w[1], 6), lpad(round(Int, w[2]), 6), "  ", sort(collect(w[3]))); end
        end
        if get(ENV, "JM_LAPWIN_DIAG", "0") == "1"
            P = TRKSURF.pos; Sd = TRKSURF.lapdist; worst = Tuple{Float64,Float64,String}[]
            for k in 1:max(1, round(Int, 50/max(Sd[min(end, 2)] - Sd[1], 1e-3))):length(P)
                ex, ez = P[k][1], P[k][3]; c = Sd[k]; best = (Inf, "")
                kn = k < length(P) ? k + 1 : 1; hx = P[kn][1] - ex; hz = P[kn][3] - ez; hn = hypot(hx, hz) + 1e-9
                AHEAD = get(ENV, "JM_LAPWIN_DIAG_AHEAD", "0") == "1"
                for (pi, tp) in enumerate(TRACK)
                    v = tp.verts; off = TRACKLAPOFF[pi]

                    shown = Set{NTuple{3,Int32}}()
                    for b in 0:length(off)-2
                        lapwin_in((b % LAPNB)*LAPBIN + LAPBIN/2, c) || continue
                        for t in div(off[b+1], 3):div(off[b+2], 3)-1
                            o = 33t; push!(shown, (round(Int32, 100*(v[o+1]+v[o+12]+v[o+23])), round(Int32, 100*(v[o+2]+v[o+13]+v[o+24])), round(Int32, 100*(v[o+3]+v[o+14]+v[o+25]))))
                        end
                    end
                    for b in 0:length(off)-2
                        lapwin_in((b % LAPNB)*LAPBIN + LAPBIN/2, c) && continue
                        for t in div(off[b+1], 3):div(off[b+2], 3)-1
                            o = 33t; cx = (v[o+1] + v[o+12] + v[o+23])/3; cz = -(v[o+3] + v[o+14] + v[o+25])/3
                            (round(Int32, 100*(v[o+1]+v[o+12]+v[o+23])), round(Int32, 100*(v[o+2]+v[o+13]+v[o+24])), round(Int32, 100*(v[o+3]+v[o+14]+v[o+25]))) in shown && continue
                            d = (cx - ex)^2 + (cz - ez)^2
                            if AHEAD
                                cosb = ((cx - ex)*hx + (cz - ez)*hz) / (sqrt(d) + 1e-9) / hn
                                cosb < cosd(50) && continue
                            end
                            d < best[1]^2 && (best = (sqrt(d), string(tp.tex)))
                        end
                    end
                end
                push!(worst, (c, best[1], best[2]))
            end
            sort!(worst, by = x -> x[2])
            println("  [lapwin diag] nearest HIDDEN triangle by eye position (lap m: distance m, texture), the 12 closest:")
            for w in worst[1:min(end, 12)]; println("      s ", round(Int, w[1]), ": ", round(w[2], digits = 1), " m  ", w[3]); end
            println("  [lapwin diag] eye positions with a hidden triangle within 150 m: ", count(w -> w[2] < 150, worst), " of ", length(worst))
        end
    end
end
"""TRACKBAND-1: a banded part's vertex ranges round lap distance `c` -- untagged geometry over the window; near bins
(within 117 m) of the near band, mid bins (to 312 m) of the mid band, the rest of the window of the far band. `whole`:
the untagged and far geometry over the whole lap (Monza's graded track draw has no lap window)"""
function bandranges!(r, off, c, whole = false)
    empty!(r); N = LAPNB; L = _LW.L
    bin(s) = clamp(floor(Int, mod(s, L) / LAPBIN), 0, N - 1)
    function span!(bd, a, n)
        (n <= 0 || n > N) && return
        o = bd*N
        if a + n <= N; push!(r, (Int(off[o+a+1]), Int(off[o+a+n+1] - off[o+a+1])))
        else; push!(r, (Int(off[o+a+1]), Int(off[o+N+1] - off[o+a+1]))); push!(r, (Int(off[o+1]), Int(off[o+a+n-N+1] - off[o+1]))); end
    end
    if isnan(c); for bd in 0:3; span!(bd, 0, N); end; return r; end
    b0 = bin(c - LAPWIN); b1 = bin(c + LAPWIN); n0 = bin(c - TRACKBAND_NEAR); n1 = bin(c + TRACKBAND_NEAR)
    m0 = bin(c - TRACKBAND_MID); m1 = bin(c + TRACKBAND_MID); d(a, b) = mod(b - a, N)
    span!(1, n0, d(n0, n1) + 1)
    span!(2, m0, d(m0, n0)); span!(2, mod(n1 + 1, N), d(n1, m1))
    if whole
        span!(0, 0, N); span!(3, mod(m1 + 1, N), d(m1, m0) - 1)
    else
        span!(0, b0, d(b0, b1) + 1)
        span!(3, b0, d(b0, m0)); span!(3, mod(m1 + 1, N), d(m1, b1))
    end
    r
end
const BANDR = Tuple{Int,Int}[]
"the part's vertex ranges inside the lap window round lap distance `c` (one, or two across the line)"
function lapranges(off, c)
    isnan(c) && return ((0, Int(off[end])), (0, 0))
    b0 = clamp(floor(Int, mod(c - LAPWIN, _LW.L) / LAPBIN), 0, LAPNB - 1)
    b1 = clamp(floor(Int, mod(c + LAPWIN, _LW.L) / LAPBIN), 0, LAPNB - 1)
    b0 <= b1 ? ((Int(off[b0+1]), Int(off[b1+2] - off[b0+1])), (0, 0)) :
               ((Int(off[b0+1]), Int(off[end] - off[b0+1])), (0, Int(off[b1+2])))
end

if _LW !== nothing && get(ENV, "JM_TRACKBAND_DIAG", "") != ""
    for c in parse.(Float64, split(ENV["JM_TRACKBAND_DIAG"], ","))
        n1 = n0 = w1 = w0 = 0
        for off in TRACKLAPOFF
            if length(off) > LAPNB + 1
                n1 += sum(x -> x[2], bandranges!(BANDR, off, c); init = 0) ÷ 3
                w1 += sum(x -> x[2], bandranges!(BANDR, off, c, true); init = 0) ÷ 3
                for bd in 0:3
                    vw = view(off, bd*LAPNB+1:bd*LAPNB+LAPNB+1)
                    for (f, n) in lapranges(vw, c); n > 0 && (n0 += (f == 0 ? n - Int(vw[1]) : n) ÷ 3); end
                end
                w0 += off[end] ÷ 3
            else
                n1 += (k = sum(x -> x[2], lapranges(off, c)) ÷ 3); n0 += k; w1 += off[end] ÷ 3; w0 += off[end] ÷ 3
            end
        end
        println("  [trackband diag] eye at s ", round(Int, c), ": ", n1, " of ", n0, " triangles drawn (", w1, " of ", w0, " whole lap)")
    end
    exit(0)
end
const trackItems = Render.build_classic(TRACK, TEXIDX)

if get(ENV, "JM_TEXDUMP", "") != ""
    for (ti, p) in enumerate(TRACK)
        lowercase(p.tex) == lowercase(ENV["JM_TEXDUMP"]) || continue
        t = trackItems[ti].tex; t == 0 && (println("  [texdump] ", p.tex, " part ", ti, ": no texture"); continue)
        glBindTexture(GL_TEXTURE_2D, t)
        for lv in 0:12
            w = Ref{GLint}(0); h = Ref{GLint}(0)
            glGetTexLevelParameteriv(GL_TEXTURE_2D, lv, GL_TEXTURE_WIDTH, w); glGetTexLevelParameteriv(GL_TEXTURE_2D, lv, GL_TEXTURE_HEIGHT, h)
            (w[] == 0 || h[] == 0) && break
            px = Vector{UInt8}(undef, 4*w[]*h[]); glGetTexImage(GL_TEXTURE_2D, lv, GL_RGBA, GL_UNSIGNED_BYTE, px)
            a = px[4:4:end]; m = a .> 0
            mc = any(m) ? round.([sum(Float64.(px[c:4:end][m])) / count(m) for c in 1:3]; digits = 1) : [NaN, NaN, NaN]
            println("  [texdump] ", p.tex, " part ", ti, " tex ", t, " level ", lv, " ", w[], "x", h[], "  alpha max ", maximum(a), "  rgb mean where alpha>0 ", mc)
            open(joinpath(get(ENV, "JM_SHOTS_DIR", tempdir()), "texdump_$(ti)_L$(lv).ppm"), "w") do io
                write(io, "P6\n$(2w[]) $(h[])\n255\n")
                for y in 0:h[]-1, x in 0:2w[]-1
                    o = 4*(y*w[] + (x % w[])); x < w[] ? write(io, px[o+1], px[o+2], px[o+3]) : write(io, px[o+4], px[o+4], px[o+4])
                end
            end
        end
        flush(stdout)
    end
end
tstamp("  build_classic done (GL uploads)")

const TRACKCULL = get(ENV, "JM_TRACKCULL", "1") != "0"
const TRACKCULL_SH = TRACKCULL && get(ENV, "JM_TRACKCULL_SHADOW", "1") != "0"
const TRACKBOUND = map(TRACK) do tp
    v = tp.verts; n = length(v) ÷ 11
    n == 0 && return ((0f0, 0f0, 0f0), 0f0)
    lo = [Inf32, Inf32, Inf32]; hi = [-Inf32, -Inf32, -Inf32]
    for k in 1:11:length(v)-10, j in 1:3
        lo[j] = min(lo[j], v[k+j-1]); hi[j] = max(hi[j], v[k+j-1])
    end
    c = ((lo[1]+hi[1])/2, (lo[2]+hi[2])/2, (lo[3]+hi[3])/2)
    (c, Float32(sqrt(sum(((hi .- lo) ./ 2) .^ 2))) + 0.5f0)
end
length(TRACKBOUND) == length(trackItems) || error("PERF-3: TRACK/trackItems not 1:1")
const TRACK_ASPGROOVE = [(lt_ = lowercase(tp.tex); occursin("asp", lt_) || startswith(lt_, "groove")) for tp in TRACK]

const GROOVE_LATE = get(ENV, "JM_GROOVE_LATE", "1") != "0"
const TRACK_GROOVE = [startswith(lowercase(tp.tex), "groove") for tp in TRACK]
const TRACK_ORDER = GROOVE_LATE ? vcat(findall(!, TRACK_GROOVE), findall(TRACK_GROOVE)) : collect(eachindex(TRACK))
println("  [cull] PERF-3: track part radii p50 ", round(sort(last.(TRACKBOUND))[max(1, end ÷ 2)], digits=1),
        " m, p90 ", round(sort(last.(TRACKBOUND))[max(1, (9*end) ÷ 10)], digits=1), " m, max ",
        round(maximum(last.(TRACKBOUND); init=0f0), digits=1), " m over ", length(TRACKBOUND), " parts")

const _AI_HANDS = Dict{String,Render.M4}()
ai_hand(cm) = get!(_AI_HANDS, cm.name) do
    zh = Float32(sum(w[2] for w in cm.wheelspec) / max(length(cm.wheelspec), 1))
    Render.translate(0f0, 0f0, zh) * AI_HAND * Render.translate(0f0, 0f0, -zh)
end
ai_pose(p) = Render.pose_matrix(p[1], p[2], -p[3], p[4], p[5], p[6])
ai_body_mat(p, cm) = ai_pose(p) * ai_hand(cm) * Render.translate(cm.body_off[1], cm.body_off[2], cm.body_off[3])
ai_wheel_mat(p, cm, wx, wz, r, spin) = ai_pose(p) * ai_hand(cm) * Render.translate(wx, r, wz) * Render.rotz(Float32(spin))
ai_car_mat(p, cm) = ai_pose(p) * ai_hand(cm)

const AI_OWNSPIN = get(ENV, "JM_AI_OWNSPIN", "1") != "0"
const AI_SPIN = Float64[]; const AI_SPIN_P = NTuple{2,Float64}[]
function ai_spin_step!(poses)
    while length(AI_SPIN) < length(poses); push!(AI_SPIN, 0.0); push!(AI_SPIN_P, (NaN, NaN)); end
    for k in eachindex(poses)
        p = poses[k]; q = AI_SPIN_P[k]
        d = isnan(q[1]) ? 0.0 : hypot(p[1] - q[1], p[3] - q[2])
        d < 5.0 && (AI_SPIN[k] -= d/0.33)
        AI_SPIN_P[k] = (Float64(p[1]), Float64(p[3]))
    end
    nothing
end
ai_spin(k, spin) = (AI_OWNSPIN && k <= length(AI_SPIN)) ? AI_SPIN[k] : spin

const WET_SPRAY = (WETNESS > 1 && get(ENV, "JM_WET_LOOK", "1") != "0") ?
    Float32(parse(Float64, get(ENV, "JM_WET_SPRAY", "0.14")) * (WETNESS >= 7 ? 1.25 : 1.0)) : 0f0
const SPRAY_P = Dict{Int,NTuple{3,Float64}}()
const SPRAY_V = Dict{Int,NTuple{3,Float32}}()
const SPRAY_DIAG = Ref(0.0)
function spray_track!(k, x, z, t)
    q = get(SPRAY_P, k, (NaN, NaN, NaN)); SPRAY_P[k] = (Float64(x), Float64(z), Float64(t))
    (isnan(q[1]) || t <= q[3]) && return
    dt = t - q[3]; vx = (x - q[1])/dt; vz = (z - q[2])/dt
    hypot(vx, vz) > 120 && return
    o = get(SPRAY_V, k, (0f0, 0f0, 0f0)); a = Float32(clamp(dt/0.25, 0.0, 1.0))
    nx = o[1] + a*(Float32(vx) - o[1]); nz = o[2] + a*(Float32(vz) - o[2])
    SPRAY_V[k] = (nx, nz, hypot(nx, nz))
end
const SPRAY_DIAG_DONE = Ref(false)
const SPRAY_DIAG_AT = parse(Float64, get(ENV, "JM_SPRAY_DIAG", "0"))
function spray_step!(cs, poses)
    WET_SPRAY > 0 || return
    t = Float64(cs.t)
    if SPRAY_DIAG_AT > 0 && t >= SPRAY_DIAG_AT && !SPRAY_DIAG_DONE[]
        println("  [spray] t ", round(t; digits = 2), " speeds ", sort([round(Int, v[3]) for v in values(SPRAY_V)]), " peak alpha ", round(SPRAY_DIAG[]; digits = 3))
        SPRAY_DIAG_DONE[] = true
    end
    spray_track!(0, cs.x, -cs.z, t)
    for k in eachindex(poses); spray_track!(k, poses[k][1], -poses[k][3], t); end
end
const SPRAY_ITEM = Ref{Any}(nothing)
function spray_item()
    SPRAY_ITEM[] === nothing || return SPRAY_ITEM[]
    n = 64; px = Vector{UInt8}(undef, 4n*n)
    for j in 0:n-1, i in 0:n-1
        r = hypot((i + 0.5)/n - 0.5, (j + 0.5)/n - 0.5) * 2
        a = clamp(1 - r, 0.0, 1.0)^1.6
        o = 4(j*n + i); px[o+1] = 0xe4; px[o+2] = 0xe6; px[o+3] = 0xe8; px[o+4] = round(UInt8, 255a)
    end
    tex = Ref{GLuint}(0); glGenTextures(1, tex); glBindTexture(GL_TEXTURE_2D, tex[])
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, n, n, 0, GL_RGBA, GL_UNSIGNED_BYTE, px)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    v = Float32[]
    for (x, y, u, w) in ((-0.5f0, 0f0, 0f0, 1f0), (0.5f0, 0f0, 1f0, 1f0), (0.5f0, 1f0, 1f0, 0f0),
                         (-0.5f0, 0f0, 0f0, 1f0), (0.5f0, 1f0, 1f0, 0f0), (-0.5f0, 1f0, 0f0, 0f0))
        append!(v, (x, y, 0f0, 0f0, 0f0, 1f0, 1f0, 1f0, 1f0, u, w))
    end
    vao, nv = Render.upload(v)
    SPRAY_ITEM[] = Render.Item(vao, nv, tex[], (1f0, 1f0, 1f0))
end

function spray_draw!(prog, vp, eye, cs, poses)
    WET_SPRAY > 0 || return
    it = spray_item()
    glDepthMask(GL_FALSE); glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE)
    for k in 0:length(poses)
        v = get(SPRAY_V, k, (0f0, 0f0, 0f0)); v[3] > 8f0 || continue
        x, y, z = k == 0 ? (Float32(cs.x), Float32(cs.y), Float32(-cs.z)) : (Float32(poses[k][1]), Float32(poses[k][2]), Float32(-poses[k][3]))
        bx = -v[1]/v[3]; bz = -v[2]/v[3]; lx = -bz; lz = bx
        f = clamp((v[3] - 8f0)/32f0, 0f0, 1f0)
        for side in (-1f0, 1f0), (d, sz, af) in ((1.3f0, 0.9f0, 1.0f0), (2.4f0, 1.4f0, 0.75f0), (3.8f0, 2.0f0, 0.5f0), (5.6f0, 2.7f0, 0.3f0))
            a = WET_SPRAY * f * af
            SPRAY_DIAG[] = max(SPRAY_DIAG[], a)
            pos = (x + bx*d + lx*side*0.72f0, y - 0.05f0, z + bz*d + lz*side*0.72f0)
            Render.draw(prog, it, vp, Render.billboard_model(pos, sz, 0.75f0*sz, eye); unlit=true, alpha=a, tint=GRADE.ringtint)
        end
    end
    glDepthMask(GL_TRUE); glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE)
end
function draw_ai_depth!(dp, poses, chassis, spin)
    for (k, (p, cm)) in enumerate(zip(poses, chassis))
        bm = ai_body_mat(p, cm); for it in cm.body; Render.draw_depth(dp, it, bm); end
        for it in get(AI_AXLES, cm.name, AI_NOAXLE); Render.draw_depth(dp, it, ai_car_mat(p, cm)); end
        sk = ai_spin(k, spin)
        for (wx,wz,_,r,nm) in cm.wheelspec, it in cm.wheels[nm]; ai_wheel_off(k, nm) && continue; Render.draw_depth(dp, it, ai_wheel_mat(p, cm, wx, wz, r, sk)); end
    end
    isempty(AI_LOOSE_WHEELS) || draw_ai_loose_wheels!((it, M) -> Render.draw_depth(dp, it, M), chassis)
end
function draw_ai_main!(prog, vp, poses, chassis, spin, nowheels::Bool)
    for (k, (p, cm)) in enumerate(zip(poses, chassis))
        bm = ai_body_mat(p, cm); for it in cm.body; Render.draw(prog, it, vp, bm; bright=AI_BRIGHT, spec=0.10, ambfill=AI_AMB); end
        for it in get(AI_AXLES, cm.name, AI_NOAXLE); Render.draw(prog, it, vp, ai_car_mat(p, cm); bright=1.1, spec=0.3, ambfill=0.55); end
        nowheels && continue
        sk = ai_spin(k, spin)
        for (wx,wz,_,r,nm) in cm.wheelspec, it in cm.wheels[nm]; ai_wheel_off(k, nm) && continue; Render.draw(prog, it, vp, ai_wheel_mat(p, cm, wx, wz, r, sk)); end
    end
    isempty(AI_LOOSE_WHEELS) || draw_ai_loose_wheels!((it, M) -> Render.draw(prog, it, vp, M), chassis)
end

@inline function inlightbox(lvp::AbstractMatrix, pos, r::Float32)
    x = Float32(pos[1]); y = Float32(pos[2]); z = Float32(pos[3])
    for i in 1:3
        c = Float32(lvp[i,1])*x + Float32(lvp[i,2])*y + Float32(lvp[i,3])*z + Float32(lvp[i,4])
        s = sqrt(Float32(lvp[i,1])^2 + Float32(lvp[i,2])^2 + Float32(lvp[i,3])^2)
        abs(c) > 1f0 + r * s && return false
    end
    true
end

monza_surf(t) = (startswith(t,"trrow") || startswith(t,"asp")) ? :road :
                (startswith(t,"armco") || startswith(t,"yarmc") || startswith(t,"brdgarm") || startswith(t,"brdgfen")) ? :dark :
                occursin(r"^s\d\d", t) ? :bank :
                :other
tstamp("  [E80] track categories / crowd tint begins")
const TRACKCAT = MONZA ? [monza_surf(lowercase(p.tex)) for p in TRACK] : Symbol[]

const ROAD_GAIN = parse(Float32, get(ENV, "JM_ROAD_GAIN", "0.72"))
const VEG_GAIN  = parse(Float32, get(ENV, "JM_VEG_GAIN", "0.81"))

const _VEGCACHE = Dict{String,Bool}()
function is_veg_tex(t)
    get!(_VEGCACHE, t) do
        r = Render.tex_rgba(TEXIDX, t); r === nothing && return false
        w, h, px = r; n = w*h; (n > 0 && length(px) >= 4n) || return false
        sr = 0.0; sg = 0.0; sb = 0.0; c = 0
        @inbounds for i in 0:max(1, n ÷ 4096):n-1
            px[4i+4] < 0x80 && continue
            sr += px[4i+1]; sg += px[4i+2]; sb += px[4i+3]; c += 1
        end
        c == 0 && return false
        sr /= c; sg /= c; sb /= c
        (sg > sb + 15 && sg >= sr - 10)
    end
end

const _FOLCACHE = Dict{String,Bool}()
function is_foliage_tex(t)
    get!(_FOLCACHE, t) do
        r = Render.tex_rgba(TEXIDX, t); r === nothing && return false
        w, h, px = r; n = w*h; (n > 0 && length(px) >= 4n) || return false
        sr = 0.0; sg = 0.0; sb = 0.0; c = 0
        @inbounds for i in 0:max(1, n ÷ 4096):n-1
            px[4i+4] < 0x80 && continue
            sr += px[4i+1]; sg += px[4i+2]; sb += px[4i+3]; c += 1
        end
        c == 0 && return false
        sr /= c; sg /= c; sb /= c
        (sg >= sr + 10 && sg >= sb)
    end
end
const OBJ_VEG_GRADE = get(ENV, "JM_OBJ_VEG", "1") != "0"

const OBJ_BRIGHT = parse(Float32, get(ENV, "JM_OBJ_BRIGHT", "1.05"))
const OBJ_AMB    = parse(Float32, get(ENV, "JM_OBJ_AMB", "0.55"))

const OBJ_FLAT = let f = get(ENV, "JM_OBJ_FLAT", "0.88,0.22")
    f == "0" ? (0f0, 0f0) : (parse(Float32, split(f, ",")[1]), parse(Float32, split(f, ",")[2]))
end

const OBJVEG = Dict{String,Vector{Bool}}()

const BACKDROP_ON = NURB && get(ENV, "JM_BACKDROP", "1") != "0"
const BACKDROP_R_TIERG  = parse(Float32, get(ENV, "JM_BACKDROP_R_TIERG",  "500"))
const BACKDROP_R_CASTLE = parse(Float32, get(ENV, "JM_BACKDROP_R_CASTLE", "1200"))
isbackdrop(nm) = BACKDROP_ON && (startswith(nm, "tierg") || startswith(nm, "inhcas"))
backdrop_r(nm) = startswith(nm, "tierg") ? BACKDROP_R_TIERG : BACKDROP_R_CASTLE
const OBJBACKDROP = Dict{String,Vector{Bool}}()

const WET_SHEEN = (WETNESS > 1 && get(ENV, "JM_WET_LOOK", "1") != "0") ?
    Float32(clamp(parse(Float64, get(ENV, "JM_WET_SHEEN", "0.8")) * (WETNESS >= 7 ? 1.15 : 1.0), 0.0, 1.0)) : 0f0
const TRACKWET = Float32[WET_SHEEN > 0 && ROAD_TEX(lowercase(p.tex)) ? WET_SHEEN : 0f0 for p in TRACK]
WET_SHEEN > 0 && println("  [wetlook] road sheen ", WET_SHEEN, " on ", count(>(0), TRACKWET), " road parts (JM_WET_SHEEN)")
const TRACKGAIN = let
    vegtex(t) = is_veg_tex(t) ? VEG_GAIN : 1f0
    g = Float32[]
    for p in TRACK
        lt = lowercase(p.tex)

        vegname = occursin("gras", lt) || startswith(lt, "bank") || occursin("verge", lt) || occursin("field", lt) || occursin("wiese", lt)
        push!(g, ROAD_TEX(lt) ? (MONZA ? 1f0 : ROAD_GAIN) : vegname ? VEG_GAIN : vegtex(lt))
    end
    println("  [grade] GRADEGOLD-1: ", count(==(ROAD_GAIN), g), " road parts x", ROAD_GAIN, ", ", count(==(VEG_GAIN), g),
            " vegetation parts x", VEG_GAIN, " (of ", length(g), ")")
    g
end

monza_obj_grade(nm) =
    (nm in ("paddock","cgroad","chicane","ascari","bnk_trcr") || occursin(r"^sec\d", nm)) ? :road :
    ((startswith(nm,"nbank")||startswith(nm,"sbank")||startswith(nm,"sbnk")||startswith(nm,"nbnk")||startswith(nm,"bnk")) &&
        !occursin("ads",nm) && !occursin("shel",nm) && !occursin("pep",nm)) ? :bank :
    :keep

const CROWD_TINT = (parse(Float32, get(ENV,"JM_CROWD_TR","1.16")),
                    parse(Float32, get(ENV,"JM_CROWD_TG","1.02")),
                    parse(Float32, get(ENV,"JM_CROWD_TB","0.66")))
is_crowd_obj(nm) = occursin("stand", nm) || occursin("tribun", nm) || occursin("crowd", nm) ||
                   occursin("spect", nm) || startswith(nm,"grnd") || startswith(nm,"pplrow") ||
                   startswith(nm,"peprow") || startswith(nm,"plrow") || startswith(nm,"pitpe") ||
                   startswith(nm,"ppl") || startswith(nm,"people") || (startswith(nm,"pelf") && nm != "pelfspa") ||
                   startswith(nm,"p_s")

tstamp("  [E80] horizon ring begins")
const HORIZON_RING = if !SKIDPAD

    let hp = get(ENV, "JM_HORIZ3DO", "1") != "0" ? track_file("horiz", ".3do") : "",
        hz = (hp != "" && isfile(hp)) ? Render.build_horizon_3do(TEXIDX, hp) : Render.Item[]
        println("  [horizon] ", isempty(hz) ? "name-guessed ring (no usable horiz.3do)" : "horiz.3do: $(length(hz)) textured panels")
        isempty(hz) ? Render.build_horizon(TEXIDX) : hz
    end
else
    try
        Render.build_horizon(Render.classic_texture_index(joinpath(CLASSICBASE, "nurburg")))
    catch e
        println("  skidpad horizon (nurburg) unavailable (", e, ") — clear sky"); nothing
    end
end

const OBJ_FLATCOL = get(ENV, "JM_OBJ_FLATCOL", "1") != "0"
const _OBJ_SKIP = SKIDPAD || (NURB && get(ENV, "JM_RING_OBJECTS", "1") == "0")
if _OBJ_SKIP
    global OBJECTS = Any[]; global OBJNODE = Int[]
    global BILLBOARDS = Tuple{Render.Item,NTuple{3,Float32},Float32,Float32}[]
    global STATICTREES = Tuple{Render.Item,NTuple{3,Float32},Float32,Float32,Float32}[]
    global SOLIDS = Tuple{Float64,Float64,Float64,Symbol}[]
    global OBJINSTS = Tuple{String,Float32,Float32,Float32,Symbol,Bool}[]

    classicvis_seg(ex, ez) = 0
    global OBJ_LVERTS = Dict{String,Vector{Tuple{Float32,Float32}}}(); global OBJ_YAW = Dict{Tuple{Float64,Float64},Float64}(); global OBJ_YMN = Dict{String,Float32}(); global OBJ_YMX = Dict{String,Float32}(); global OBJ_RECZ = Dict{Tuple{Float64,Float64},Float64}(); global OBJ_VERTS = Dict{String,Any}()

    if (@isdefined RINGSPRITES) && !isempty(RINGSPRITES) && get(ENV,"JM_RING_BB","1") != "0"
        RING_BB_HALFW = parse(Float64, get(ENV,"JM_RING_BB_HALFW","5.0"))
        BB_MARKER_TEX = Set(["collision", "fake", "shadow", "lshad"])
        RING_BB_WIDE  = parse(Float32, get(ENV,"JM_WIDE_PANEL","30"))
        local nbb=0; local notex=0; local onrd=0; local nwide=0
        bbcache = Dict{String,Any}()
        for sp in RINGSPRITES

            hr = JRTrack.hat(TRKSURF, Float64(sp.x), Float64(sp.y))
            if hr.found && abs(hr.lateral) < RING_BB_HALFW; onrd += 1; continue; end

            bb = get!(bbcache, lowercase(sp.name)) do
                r = nothing
                for tn in sp.texs
                    lowercase(strip(tn)) in BB_MARKER_TEX && continue
                    r = Render.build_billboard(tn, TEXIDX)
                    r !== nothing && break
                end
                r
            end
            if bb === nothing; notex += 1; continue; end
            item, tw, th = bb
            w = sp.w > 0f0 ? sp.w : sp.h*tw/max(th,1f0)

            if w > RING_BB_WIDE

                if get(ENV,"JM_RING_WIDE_DROP","1") != "0"; nwide += 1; continue; end
                push!(STATICTREES, (item, (sp.x, sp.z, -sp.y), Float32(w), Float32(sp.h),
                                    Float32(-(sp.yaw + sp.aax))))
                nwide += 1
                continue
            end
            push!(BILLBOARDS, (item, (sp.x, sp.z, -sp.y), Float32(w), Float32(sp.h)))
            nbb += 1
        end

        if get(ENV,"JM_FLOATDIAG","") != ""
            gaps = Tuple{Float64,String,Float64,Float64}[]
            for sp in RINGSPRITES

                gh = JRTrack.hat3d(TERRAIN, Float64(sp.x), Float64(sp.y); ref=Float64(sp.z)+2.0)
                gh[3] || continue
                push!(gaps, (Float64(sp.z) - gh[1], String(sp.name), Float64(sp.z), gh[1]))
            end

            let xs=[Float64(sp.x) for sp in RINGSPRITES], ys=[Float64(sp.y) for sp in RINGSPRITES],
                zs=[Float64(sp.z) for sp in RINGSPRITES]
                println("   [ctrl] sp.x ", round(minimum(xs)), "..", round(maximum(xs)),
                        "   sp.y ", round(minimum(ys)), "..", round(maximum(ys)),
                        "   sp.z ", round(minimum(zs)), "..", round(maximum(zs)))

                for (lbl, zf) in (("-sp.y", -1.0), ("+sp.y", +1.0))
                    gs = Float64[]
                    for sp in RINGSPRITES
                        g = JRTrack.hat3d(TERRAIN, Float64(sp.x), zf*Float64(sp.y); ref=Float64(sp.z)+2.0)
                        g[3] && push!(gs, abs(Float64(sp.z) - g[1]))
                    end
                    isempty(gs) && continue
                    sort!(gs)
                    println("   [ctrl] z=", lbl, ": ", length(gs), " hits, median |gap| ",
                            round(gs[cld(length(gs),2)],digits=1), " m, 90th ",
                            round(gs[cld(9*length(gs),10)],digits=1), " m")
                end
            end
            if !isempty(gaps)
                sort!(gaps, by=g->-abs(g[1]))
                nfloat = count(g -> g[1] >  0.5, gaps)
                nsunk  = count(g -> g[1] < -0.5, gaps)
                println("== JM_FLOATDIAG: ", length(gaps), " Ring sprites grounded against the terrain ==")
                println("   floating >0.5 m above: ", nfloat, "   sunk >0.5 m below: ", nsunk,
                        "   within +/-0.5 m: ", length(gaps)-nfloat-nsunk)
                nlist = parse(Int, get(ENV,"JM_FLOATDIAG_N","20"))
                for g in gaps[1:min(end,nlist)]
                    println("   ", rpad(g[2],14), " base=", rpad(round(g[3],digits=1),9),
                            " ground=", rpad(round(g[4],digits=1),9),
                            " gap=", g[1] > 0 ? "+" : "", round(g[1],digits=1), " m")
                end
            end
            flush(stdout)
        end
        if get(ENV,"JM_RING_BB_DIAG","") != ""

            lats = Float64[]
            for sp in RINGSPRITES
                hr = JRTrack.hat(TRKSURF, Float64(sp.x), Float64(sp.y))
                hr.found && push!(lats, abs(hr.lateral))
            end
            sort!(lats)
            println("  == JM_RING_BB_DIAG: |lateral| of the ", length(lats), " on-ribbon stubs ==")
            for q in (0.05,0.1,0.25,0.5,0.75,0.9)
                println("     p", rpad(round(Int,100q),3), " = ", round(lats[max(1,ceil(Int,q*length(lats)))],digits=1), " m")
            end
            for edge in (3.0,4.0,5.0,6.0,7.0,9.0,12.0)
                println("     |lat| < ", rpad(edge,5), " → ", count(<(edge), lats), " dropped")
            end

            hs = Float64[]; dz = Float64[]
            for sp in RINGSPRITES
                push!(hs, Float64(sp.h))
                gh = JRTrack.hat3d(TERRAIN, Float64(sp.x), Float64(sp.y); ref=Inf)
                gh[3] && push!(dz, Float64(sp.z) - Float64(gh[1]))
            end
            sort!(hs); sort!(dz)
            q(v,f) = v[max(1,ceil(Int,f*length(v)))]
            println("     sprite HEIGHT  m : p10=", round(q(hs,0.1),digits=1), " p50=", round(q(hs,0.5),digits=1),
                    " p90=", round(q(hs,0.9),digits=1), " max=", round(hs[end],digits=1))
            println("     base ABOVE ground: p10=", round(q(dz,0.1),digits=1), " p50=", round(q(dz,0.5),digits=1),
                    " p90=", round(q(dz,0.9),digits=1), " max=", round(dz[end],digits=1), "   (n=", length(dz), ")")

            cnt = Dict{String,Int}(); hmax = Dict{String,Float64}()
            for sp in RINGSPRITES
                cnt[sp.name] = get(cnt,sp.name,0)+1
                hmax[sp.name] = max(get(hmax,sp.name,0.0), Float64(sp.h))
            end
            tex1 = Dict{String,String}(); wmax = Dict{String,Float64}()
            for sp in RINGSPRITES
                tex1[sp.name] = isempty(sp.texs) ? "(none)" : first(sp.texs)
                wmax[sp.name] = max(get(wmax,sp.name,0.0), Float64(sp.w))
            end
            println("     -- most-placed stub families (name, count, height, width, 1st texture) --")
            for (nm,c) in sort(collect(cnt), by=x->-x[2])[1:min(end,12)]
                println("        ", rpad(nm,14), rpad(c,6), rpad(string(round(hmax[nm],digits=1),"m"),8),
                        rpad(string(round(wmax[nm],digits=1),"m"),8), tex1[nm])
            end
            seenn = Set{String}()
            println("     -- ALL texture strings for the tall stubs --")
            for sp in RINGSPRITES
                (sp.h > 10 && !(sp.name in seenn)) || continue
                push!(seenn, sp.name)
                println("        ", rpad(sp.name,12), " → ", join(sp.texs, " | "))
                length(seenn) >= 8 && break
            end
            println("     -- families whose texture looks like PEOPLE/CROWD --")
            shown = Set{String}()
            for sp in RINGSPRITES
                tx = isempty(sp.texs) ? "" : lowercase(join(sp.texs," "))
                (occursin("pp",tx) || occursin("people",tx) || occursin("zusch",tx) ||
                 occursin("crowd",tx) || occursin("flag",tx) || occursin("mann",tx)) || continue
                sp.name in shown && continue; push!(shown, sp.name)
                println("        ", rpad(sp.name,12), rpad(string(round(sp.h,digits=1),"m"),8),
                        rpad(string("w=",round(sp.w,digits=1)),9), rpad(cnt[sp.name],5), join(sp.texs," | "))
            end
            println("     -- every family taller than 8 m --")
            shown2 = Set{String}()
            for (nm,h) in sort(collect(hmax), by=x->-x[2])
                h > 8 || break
                nm in shown2 && continue; push!(shown2, nm)
                println("        ", rpad(nm,12), rpad(string(round(h,digits=1),"m"),8), rpad(cnt[nm],5), tex1[nm])
            end
            println("     -- TALLEST families (these are what render as giants) --")
            for (nm,h) in sort(collect(hmax), by=x->-x[2])[1:min(end,12)]
                println("        ", rpad(nm,14), rpad(string(round(h,digits=1),"m"),8),
                        rpad(cnt[nm],6), tex1[nm])
            end
        end
        println("  E76-S8 Ring billboards: ", nbb, " placed, ", notex, " no texture, ", onrd,
                " on-road, ", nwide, " wide → static panels")
        flush(stdout)
    end
else
if get(ENV,"JM_HAT_COUNT","0") != "0"

    JRTrack.HAT_COUNT_ON[] = true
    JRTrack.HAT_TIME_ON[]  = get(ENV,"JM_HAT_TIME","0") != "0"
    JRTrack.hat_reset!()
end

tstamp("  [E80] trackside objects + billboards + trees begin")
const DATPACK = TRACKDAT
const TMPOBJ = mktempdir()

const _ZD_LC = Dict(lowercase(f) => f for f in (isdir(ZD) ? readdir(ZD) : String[]))
_zd_file(nm) = (f = get(_ZD_LC, lowercase(nm), nothing); f === nothing ? joinpath(ZD, nm) : joinpath(ZD, f))
objpath0(nm) = (p=_zd_file(nm*".3do"); isfile(p) ? p :
    (v=get(DATPACK, lowercase(nm*".3do"), nothing); v===nothing ? "" :
     (tp=joinpath(TMPOBJ, nm*".3do"); isfile(tp)||write(tp,v); tp)))

const OBJ_ALIAS = Dict(lowercase(strip(split(x, "=")[1])) => lowercase(strip(split(x, "=")[2]))
                       for x in split(get(ENV, "JM_OBJ_ALIAS", "house1a=house1"), ","; keepempty = false) if occursin("=", x))
objpath(nm) = (p = objpath0(nm); (p == "" && haskey(OBJ_ALIAS, lowercase(nm))) ? objpath0(OBJ_ALIAS[lowercase(nm)]) : p)

const CROWD_TEX = ("ltraymap","lshad")

const CLASSICW_NODEGRID = !CLASSICW_ON ? Dict{Tuple{Int,Int},Vector{Int}}() : let g = Dict{Tuple{Int,Int},Vector{Int}}()
    for (k, p) in enumerate(CLASSICREF); push!(get!(g, (floor(Int, p[1]/16), floor(Int, p[2]/16)), Int[]), k); end
    g
end
"locate without a hint: nearest Classic node from the node grid (within ~150 m), then the hinted search"
function classicw_locate_any(x, z)
    ci = floor(Int, x/16); cj = floor(Int, z/16); best = 0; bd = Inf
    for r in 0:10
        for i in ci-r:ci+r, j in cj-r:cj+r
            (abs(i - ci) == r || abs(j - cj) == r) || continue
            for k in get(CLASSICW_NODEGRID, (i, j), Int[])
                d = (CLASSICREF[k][1] - x)^2 + (CLASSICREF[k][2] - z)^2
                d < bd && (bd = d; best = k)
            end
        end
        best != 0 && r >= 1 && break
    end
    best == 0 ? nothing : classicw_locate(x, z; hint = best)
end
"the free interval that holds Classic's road at (section, fraction)"
function classicw_road_interval(sec, f)
    v = CLASSICWALLS.secs[sec]; bw = -1.0; mid = 0.0
    for k in 1:length(v)-1
        (v[k].typ & 0x7ff) == 1 || continue
        a = CLASSICWall.edge(CLASSICWALLS, sec, k, f); b = CLASSICWall.edge(CLASSICWALLS, sec, k + 1, f)
        b - a > bw && (bw = b - a; mid = (a + b)/2)
    end
    CLASSICWall.free_interval(CLASSICWALLS, sec, f, mid, mid; hmin = CLASSICW_HMIN)
end
let objnames=Set{String}()
    for f in readdir(ZD); endswith(lowercase(f),".3do") && push!(objnames, lowercase(replace(f,r"\.3do$"i=>""))); end
    for k in keys(DATPACK); endswith(k,".3do") && push!(objnames, replace(k,r"\.3do$"=>"")); end
    for (a, m) in OBJ_ALIAS; m in objnames && push!(objnames, a); end
    insts = CLASSICTrack.trackside_objects(ZTRK; objnames=objnames)

    let pitpush = parse(Float64, get(ENV, "JM_PITBLDG_PUSH", TRACKSEL=="watglen" ? "5.0" : "0.0")),

        boxpush = parse(Float64, get(ENV, "JM_STARTBOX_PUSH", "0.0"))

        pushof(nm) = nm=="pitbldg" ? pitpush : nm=="startbox" ? boxpush : 0.0
        if (pitpush > 0.0 || boxpush > 0.0) && !isempty(ALIGNED)
            nearest(ix,iy) = (bd=Inf; bj=1; for (j,p) in enumerate(ALIGNED); d=(p[1]-ix)^2+(p[2]-iy)^2; d<bd && (bd=d; bj=j); end; ALIGNED[bj])
            insts = map(insts) do i
                push = pushof(i.name); push <= 0.0 && return i
                lx,ly = nearest(i.x, i.y); dx=i.x-lx; dy=i.y-ly; d=hypot(dx,dy)
                (d < 1e-3 || d > 18.0) && return i
                CLASSICTrack.ObjInst(i.name, i.x+dx/d*push, i.y+dy/d*push, i.z, i.yaw, i.scale, i.pitch, i.roll, i.node)
            end
        end
    end

    treeish(nm) = startswith(nm,"tree") || startswith(nm,"newt")

    _zpeople = (ZANDV && get(ENV, "JM_KEEP_ZANDCROWD", "0") == "0") ?
        Tuple(vcat([["people0$k", "People0$k", "PEOPLE0$k"] for k in 1:9]...)) : ()
    obj_extra_excl(nm) = (nm=="startbox" && !haskey(ENV,"JM_STARTBOX_KEEP")) ?
        ("sfbox01","sfbox02","sfbox03","hay01","hay02", _zpeople...) : _zpeople
    objmesh=Dict{String,Any}(); ymn=Dict{String,Float32}(); ymx=Dict{String,Float32}(); bbinfo=Dict{String,Any}()

    _keep_armco = SPA && get(ENV, "JM_KEEP_ARMCO", "1") != "0"
    _keep_lowsolid = get(ENV, "JM_KEEP_LOWSOLID", "1") != "0"

    _tall_min = parse(Float32, get(ENV, "JM_TALL_MIN", "0.25"))
    tallenough(nm) = (get(ymx,nm,0f0)-get(ymn,nm,0f0)) > _tall_min || startswith(lowercase(nm), "km_") ||
                     (_keep_armco && startswith(lowercase(nm), "armco")) ||
                     (_keep_lowsolid && startswith(lowercase(nm), "bushrow"))

    lxmn=Dict{String,Float32}(); lxmx=Dict{String,Float32}(); lzmn=Dict{String,Float32}(); lzmx=Dict{String,Float32}()
    objverts=Dict{String,Any}()
    lverts=Dict{String,Vector{Tuple{Float32,Float32}}}()

    _e92 = (n=Ref(0), ext=Ref(0.0), bld=Ref(0.0))
    for inst in insts
        (haskey(objmesh, inst.name) || haskey(bbinfo, inst.name)) && continue
        p = objpath(inst.name)
        if p == ""; objmesh[inst.name]=nothing; continue; end
        try

            _lp = let hr = JRTrack.hat(TRKSURF, Float64(inst.x), Float64(inst.y)); hr.found ? hr.lapdist*(500/0.0254) : NaN end
            _te = time(); full = Render.extract_classic_car(p; track=true, mirror=OBJ_MIRROR, flatcol=OBJ_FLATCOL, lappos=_lp, laplen=TRKSURF.lap_length*(500/0.0254)); _e92.ext[] += time() - _te; _e92.n[] += 1

            force_flat = treeish(inst.name) && !((MONZA || WATGLEN || get(ENV,"JM_MESHTREES","0") != "0") && get(ENV,"JM_FLATTREES","0") == "0")
            if isempty(full) || force_flat
                h, wid, strs, aax = Render.billboard_stub(p); bb=nothing
                for s in strs; bb = Render.build_billboard(s, TEXIDX); bb !== nothing && break; end
                get(ENV, "JM_BBDIAG", "") != "" && println("  JM_BBDIAG stub ", inst.name, ": h ", h, " w ", wid, " strings ", strs)
                bbinfo[inst.name] = bb===nothing ? nothing : (bb[1], bb[2], bb[3], h, wid, aax)
            else

                parts = Render.extract_classic_car(p; track=true, mirror=OBJ_MIRROR, flatcol=OBJ_FLATCOL, lappos=_lp, laplen=TRKSURF.lap_length*(500/0.0254), dedup=(get(ENV,"JM_OBJ_DEDUP","old")=="orient" ? :orient : true), exclude=(CROWD_TEX..., obj_extra_excl(inst.name)...))
                if isempty(parts); objmesh[inst.name]=nothing
                else
                    lo=Inf32; hi=-Inf32; for pp in parts, k in 2:11:length(pp.verts); v=pp.verts[k]; lo=min(lo,v); hi=max(hi,v); end
                    ymn[inst.name]=lo; ymx[inst.name]=hi

                    xl=Inf32; xh=-Inf32; zl=Inf32; zh=-Inf32
                    for pp in parts, k in 1:11:length(pp.verts)-2
                        vx=pp.verts[k]; vz=pp.verts[k+2]
                        xl=min(xl,vx); xh=max(xh,vx); zl=min(zl,vz); zh=max(zh,vz)
                    end
                    lxmn[inst.name]=xl; lxmx[inst.name]=xh; lzmn[inst.name]=zl; lzmx[inst.name]=zh; objverts[inst.name]=parts

                    fpground(t) = (l = lowercase(t);
                                   startswith(l,"grass") || startswith(l,"asph") ||
                                   startswith(l,"terr")  || startswith(l,"for_") ||
                                   startswith(l,"tre"))
                    fpkeep = get(ENV,"JM_FP_KEEP_GROUND","0") != "0"
                    vs = Tuple{Float32,Float32}[]
                    for pp in parts
                        (fpkeep || !fpground(pp.tex)) || continue
                        for k in 1:(11*13):length(pp.verts)-2
                            push!(vs, (pp.verts[k], pp.verts[k+2]))
                        end
                    end

                    if isempty(vs)
                        for pp in parts, k in 1:(11*13):length(pp.verts)-2
                            push!(vs, (pp.verts[k], pp.verts[k+2]))
                        end
                    end
                    lverts[inst.name] = vs
                    twinboard_classify!(lowercase(inst.name), parts)

                    OBJ_VEG_GRADE && (OBJVEG[lowercase(inst.name)] = Bool[is_foliage_tex(lowercase(pp.tex)) for pp in parts])
                    isbackdrop(lowercase(inst.name)) && (OBJBACKDROP[lowercase(inst.name)] = Bool[lowercase(pp.tex) != "grass" for pp in parts])
                    _tb = time(); objmesh[inst.name] = Render.build_classic(parts, TEXIDX); _e92.bld[] += time() - _tb
                    OBJ_CPU[lowercase(inst.name)] = parts
                end
            end
        catch; objmesh[inst.name]=nothing; end
    end
    if get(ENV,"JM_MESH_TIME","0") != "0"
        println("[mesh] ", _e92.n[], " distinct meshes loaded   extract ",
                round(_e92.ext[],digits=2), " s   build_classic ", round(_e92.bld[],digits=2), " s   total ",
                round(_e92.ext[]+_e92.bld[],digits=2), " s")
        flush(stdout)
    end
    tstamp("  [E80] .. distinct-mesh LOOP done (post-mesh per-instance work follows)")

    groundz(x,y) = (h=JRTrack.hat3d(TERRAIN, Float64(x), Float64(y); ref=Inf); h[3] ? Float32(h[1]) : -999f0)

    TRKCX = sum(p[1] for p in ALIGNED)/length(ALIGNED); TRKCY = sum(p[2] for p in ALIGNED)/length(ALIGNED)

    function edgez(x, y)

        local tx = TRKCX; local ty = TRKCY

        if !haskey(ENV, "JM_EDGEZ_CENTROID")
            bd = Inf
            for p in ALIGNED
                dd = (p[1]-x)^2 + (p[2]-y)^2
                dd < bd && (bd = dd; tx = p[1]; ty = p[2])
            end
        end
        dx = tx - x; dy = ty - y; d = hypot(dx, dy)
        d < 1f-3 && return trkzlo

        lowest(px, py) = begin
            g = groundz(px, py)
            g <= -900f0 && return g
            get(ENV,"JM_EDGEZ_TOP","0") != "0" && return g
            r = Float64(g)
            for _ in 1:8
                h = JRTrack.hat3d(TERRAIN, Float64(px), Float64(py); ref = r - 0.6)
                h[3] || break
                r = h[1]
            end
            Float32(r)
        end

        if get(ENV,"JM_EDGEZ_RAY","0") == "0"
            for rad in 8f0:8f0:Float32(min(d, 400.0))
                best = Inf32
                for k in 0:11
                    th = Float32(2pi*k/12)
                    g = lowest(x + cos(th)*rad, y + sin(th)*rad)
                    g > -900f0 && g < best && (best = g)
                end
                best < Inf32 && return best
            end
        end
        ux = dx/d; uy = dy/d; s = 0f0
        while s < d
            gz = lowest(x + ux*s, y + uy*s); gz > -900f0 && return gz
            s += 8f0
        end
        trkzlo
    end

    _recz_used = Ref(0)
    ploz(i)  = begin
        gz = groundz(i.x, i.y); g = gz > -900f0 ? gz : edgez(Float32(i.x), Float32(i.y))
        if OBJ_USE_RECZ && isfinite(i.z) && (!(gz > -900f0) || abs(Float64(g) - Float64(i.z)) > OBJ_RECZ_TOL)
            _recz_used[] += 1; Float32(i.z)
        else
            g
        end
    end

    trkzlo=Inf32; trkzhi=-Inf32
    for t in TRACKMESH.tris, vi in 1:3; z=Float32(t.p[vi][3]); trkzlo=min(trkzlo,z); trkzhi=max(trkzhi,z); end
    onground(i) = (gz = groundz(i.x, i.y); gz > -900f0 || (trkzlo-150f0 < Float32(i.z) < trkzhi+150f0))

    function on_road(x, y, halfw)
        hr = JRTrack.hat(TRKSURF, Float64(x), Float64(y))
        hr.found && abs(hr.lateral) < halfw
    end

    standcrowd(nm) = startswith(nm,"grndpe") || startswith(nm,"pitpeo") || startswith(nm,"pitppl") ||
                     startswith(nm,"pplrow") || startswith(nm,"peprow") || startswith(nm,"plrow") ||
                     startswith(nm,"ppl") || startswith(nm,"people") || (startswith(nm,"pelf") && nm != "pelfspa") ||
                     startswith(nm,"p_s") || startswith(nm,"spect") || startswith(nm,"crowd") ||
                     startswith(nm,"grndp")

    _droptest = split(get(ENV,"JM_DROPTEST",""), ',', keepempty=false)
    _keeptest = split(get(ENV,"JM_KEEPTEST",""), ',', keepempty=false)

    _dropcrowdrows = get(ENV,"JM_KEEP_CROWDROWS","0") == "0"

    _zandcrowd_off = ZANDV && get(ENV, "JM_KEEP_ZANDCROWD", "0") == "0"

    isperson(nm) = startswith(nm, "single") || PeopleFilter.is_loose_person(nm)
    function person_unsafe(i)
        (SAFE_SPECT && isperson(lowercase(String(i.name)))) || return false
        hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y)); hr.found || return false
        b = get(bbinfo, i.name, nothing); w = b === nothing ? 0.0 : Float64(b[5])
        reach = (w/2) * abs(sin(Float64(i.yaw) - atan(-hr.perp[2], hr.perp[1])))
        abs(hr.lateral) - reach < ROAD_HALFW + SPECT_MARGIN
    end

    _spaflags = SPA && get(ENV, "JM_SPAFLAGS", "1") != "0"
    function spaflags_keep(i)
        _spaflags || return false
        occursin(r"^(people\d|peprow|pelfspa|fla\d|flagger|spaflg|speaker)", lowercase(String(i.name))) || return false
        hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
        hr.found && 8430.0 <= hr.lapdist <= 8660.0 && hr.lateral > 0
    end
    dropi(i) = drop(i.name) && !spaflags_keep(i)
    drop(nm) = (!isempty(_keeptest) && any(p->startswith(nm,p), _keeptest)) ? false :
               (_dropcrowdrows && standcrowd(nm)) || (_zandcrowd_off && standcrowd(nm)) ||
               (!isempty(_droptest) && any(p->startswith(nm,p), _droptest)) ||

               (NURB && startswith(lowercase(nm), "trow") && get(ENV, "JM_RING_TROW_OBJ", "0") == "0") || (!standcrowd(nm) && (
               (startswith(nm,"grass") && !KEEP_GRASS) || (startswith(nm,"herbe") && !KEEP_GRASS) || nm == "infield" ||

               (!SAFE_SPECT && startswith(nm,"single")) ||
               (startswith(nm,"intree") && !WATGLEN) ||
               ((startswith(nm,"treesrb") || startswith(nm,"treefill")) && !WATGLEN) ||

               (MONZA_TREEBANK_DROP && (startswith(nm,"trbk") || startswith(nm,"brbk") || startswith(nm,"tuntbk"))) ||

               (!SAFE_SPECT && PeopleFilter.is_loose_person(nm))))
    istree(nm) = startswith(nm,"tree") || startswith(nm,"newt") || startswith(nm,"intree")

    onroad_crowd(i) = standcrowd(i.name) && on_road(i.x, i.y, ROAD_HALFW)

    _bldg_wide = get(ENV,"JM_BLDG_WIDE","1") != "0"
    bldgish(nm) = startswith(nm,"lasad") || startswith(nm,"chut") || startswith(nm,"haus") ||
                  startswith(nm,"house") || startswith(nm,"ferme") || startswith(nm,"bldg") ||
                  startswith(nm,"hotel") || startswith(nm,"bld") ||
                  (_bldg_wide && (occursin(r"^(ho|bu)\d", nm) || startswith(nm,"casa") ||
                                  startswith(nm,"clhouse") || occursin("hotel", nm)))

    _keep_bldg = (SPA && get(ENV, "JM_KEEP_BLDG", "1") != "0") ? ("house46", "lasad1") : ()
    onroad_bldg(i) = bldgish(lowercase(i.name)) && !any(k -> startswith(lowercase(i.name), lowercase(k)), _keeptest) &&
                     !(lowercase(i.name) in _keep_bldg) && begin
        hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
        hit = hr.found && abs(hr.lateral) < 4.0
        hit && println("  E68: building ", i.name, " centred ON the road (lat=", round(hr.lateral,digits=1), " m, s=", round(Int,hr.lapdist), ") — dropped")
        hit
    end

    if get(ENV,"JM_PERPDIAG","") != ""
        rys = Float64[]
        for i in insts
            standcrowd(i.name) || continue
            hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
            hr.found || continue
            push!(rys, abs(rad2deg(rem2pi(Float64(i.yaw) - atan(-hr.perp[2], hr.perp[1]), RoundNearest))))
        end
        if !isempty(rys)
            println("== JM_PERPDIAG: crowd row yaw RELATIVE to the track perpendicular (", length(rys), " rows) ==")
            println("   0deg = row runs ALONG the perpendicular (crosses the track) ; 90deg = row runs PARALLEL to the track")
            for lo in 0:15:165
                c = count(r -> lo <= r < lo+15, rys)
                println("   ", rpad(string(lo,"-",lo+15,"deg"), 12), rpad(c,6), "#"^min(60, c))
            end
            println("   dropped by the 60-120deg window: ", count(r -> 60.0 <= r <= 120.0, rys))
        end
        flush(stdout)
    end
    perp_crowd(i) = get(ENV,"JM_NO_PERP","0") == "0" && standcrowd(i.name) && begin
        hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
        if hr.found && abs(hr.lateral) < ROAD_HALFW + 12.0
            ry = abs(rad2deg(rem2pi(Float64(i.yaw) - atan(-hr.perp[2], hr.perp[1]), RoundNearest)))
            60.0 <= ry <= 120.0
        else
            false
        end
    end

    onroad_fp_edge = parse(Float64, get(ENV,"JM_ASPHALT_HALFW","4.1"))

    vegcrowd(nm) = startswith(nm,"bush") || startswith(nm,"shrub") || startswith(nm,"strauch") ||
                   startswith(nm,"hedge") || startswith(nm,"haie") || standcrowd(nm) ||
                   startswith(nm,"ppl") || startswith(nm,"people") || startswith(nm,"pplrow") ||
                   (get(ENV,"JM_ONROAD_FP_BLDG","1") != "0" && bldgish(nm)) ||
                   (get(ENV,"JM_ONROAD_FP_ARMCO","1") != "0" && startswith(nm,"arm")) ||
                   (NURB && get(ENV,"JM_ONROAD_FP_ALL","1") != "0")

    _spa_bldg_fp = SPA && get(ENV, "JM_KEEP_BLDG", "1") != "0"
    onroad_fp(i) = get(ENV,"JM_ONROAD_FP","1") != "0" && vegcrowd(lowercase(i.name)) && !(lowercase(i.name) in _keep_bldg) &&
                   !(_spa_bldg_fp && occursin(r"^(house|ho\d|lasad)", lowercase(i.name))) && begin
        vs = get(lverts, i.name, nothing)
        if vs === nothing || isempty(vs)
            false
        else
            th = OBJ_YAW_SIGN * -Float64(i.yaw) + Float64(objyawfix(i.name)); c, sn = cos(th), sin(th)
            near = Inf; ontar = false

            tarmac_test = get(ENV, "JM_ONROAD_FP_TARMAC", "1") != "0" && ROADHAT !== TERRAIN0 && bldgish(lowercase(i.name))
            for (lx, lz) in vs
                rx =  lx*c + lz*sn; rz = -lx*sn + lz*c
                px_ = Float64(i.x) + rx; pz_ = Float64(i.y) - rz
                hr = JRTrack.hat(TRKSURF, px_, pz_)
                hr.found && (near = min(near, abs(hr.lateral)))
                if tarmac_test && !ontar && hr.found && hr.on_track && JRTrack.hat3d(ROADHAT, px_, pz_; ref = Inf)[3]
                    ontar = true
                end
            end
            hit = near < onroad_fp_edge || ontar
            hit && get(ENV,"JM_ONROAD_FP_DIAG","") != "" &&
                println("  E69-S5: ", i.name, " footprint reaches ", round(near,digits=1),
                        " m of the centreline — dropped")
            hit
        end
    end

    FPGROUND = get(ENV,"JM_FPGROUND","0") != "0"
    FPQ      = parse(Float64, get(ENV,"JM_FPGROUND_Q","0.15"))
    plozfp(i) = begin
        base = ploz(i)
        vs = get(lverts, i.name, nothing)
        (!FPGROUND || vs === nothing || isempty(vs)) && return base
        th = OBJ_YAW_SIGN * -Float64(i.yaw) + Float64(objyawfix(i.name)); c, sn = cos(th), sin(th)
        zs = Float32[]
        for (lx, lz) in vs
            rx =  lx*c + lz*sn; rz = -lx*sn + lz*c
            px = Float32(i.x) + Float32(rx); py = Float32(i.y) - Float32(rz)

            gz = groundz(px, py)
            gz <= -900f0 && (gz = edgez(px, py))
            gz > -900f0 && push!(zs, gz)
        end
        length(zs) < 3 && return base
        sort!(zs)
        zs[max(1, min(length(zs), ceil(Int, FPQ*length(zs))))]
    end

    hr_lap(i) = (h = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y)); h.found ? h.lapdist : -1.0)
    if get(ENV,"JM_OVERHANG","") != ""

        pat = lowercase(ENV["JM_OVERHANG"])
        halfw = parse(Float64, get(ENV,"JM_OVERHANG_HALFW","6.0"))

        println("== JM_OVERHANG: /", pat, "/ over ", length(insts), " instances, corridor ±", halfw, " m ==")
        nointr = 0; nofp = 0; hits = Tuple{String,Int,Float64,Float64,Float64,Bool,Float64,Float64}[]
        for i in insts
            (pat == "1" || occursin(pat, lowercase(i.name))) || continue
            vs = get(lverts, i.name, nothing)
            if vs === nothing || isempty(vs)
                nofp += 1
                pat == "1" || println("   [overhang] ", rpad(i.name,10), " NO FOOTPRINT (billboard/panel path)")
                continue
            end
            th = OBJ_YAW_SIGN * -Float64(i.yaw) + Float64(objyawfix(i.name)); c, sn = cos(th), sin(th)
            base = Float64(plozfp(i))
            nover = 0; ntot = 0; minlat = Inf; hi = -Inf
            for (lx, lz) in vs
                rx =  lx*c + lz*sn; rz = -lx*sn + lz*c
                px = Float64(i.x) + rx; py = Float64(i.y) - rz
                hr = JRTrack.hat(TRKSURF, px, py)
                hr.found || continue
                ntot += 1
                minlat = min(minlat, abs(hr.lateral))
                if abs(hr.lateral) < halfw
                    nover += 1
                    hi = max(hi, base - Float64(hr.height))
                end
            end
            ntot == 0 && continue
            if nover > 0

                kept = get(objmesh,i.name,nothing) !== nothing && !drop(i.name) &&
                       !onroad_crowd(i) && !perp_crowd(i) && !onroad_bldg(i) && !onroad_fp(i) &&
                       tallenough(i.name) && onground(i)

                _ho = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
                _olat = _ho.found ? _ho.lateral : NaN
                _ry = _ho.found ?
                      abs(rad2deg(rem2pi(Float64(i.yaw) - atan(-_ho.perp[2], _ho.perp[1]), RoundNearest))) : NaN
                _ry = isnan(_ry) ? _ry : (_ry > 90 ? 180 - _ry : _ry)
                push!(hits, (i.name, nover, minlat, hi, hr_lap(i), kept, _olat, _ry))
            else
                nointr += 1
            end
            pat == "1" && continue
            println("   [overhang] ", rpad(i.name,10),
                    " footprint pts ", rpad(ntot,5),
                    " over road(<", halfw, "m): ", rpad(nover,5),
                    " nearest |lat| ", rpad(round(minlat,digits=1),7),
                    nover > 0 ? string(" base sits ", round(hi,digits=1), " m above road") : "")
        end
        if pat == "1"
            sort!(hits, by=h->(!h[6], h[3]))
            nkept = count(h->h[6], hits)
            println("   ", nointr, " instances clear of the road, ", length(hits), " intruding of which ",
                    nkept, " ACTUALLY RENDER, ", nofp, " without a mesh footprint (not tested)")

            _topspec = get(ENV,"JM_OVERHANG_TOP","14")
            _top = _topspec == "all" ? length(hits) : parse(Int,_topspec)
            for h in hits[1:min(end,_top)]
                println("      ", h[6] ? "RENDERS " : "dropped ", rpad(h[1],12), "pts over road ", rpad(h[2],5),
                        " nearest |lat| ", rpad(round(h[3],digits=1),7),
                        " origin |lat| ", rpad(isnan(h[7]) ? "?" : string(round(abs(h[7]),digits=1)),7),
                        " yaw-vs-road ", rpad(isnan(h[8]) ? "?" : string(round(Int,h[8])), 4),
                        " base ", rpad(round(h[4],digits=1),7), " m   lapdist ", round(Int,h[5]))
            end

            fam = Dict{String,Vector{Tuple{Float64,Float64}}}()
            for h in hits
                h[6] || continue
                k = rstrip(h[1], ['0','1','2','3','4','5','6','7','8','9'])
                push!(get!(fam, k, Tuple{Float64,Float64}[]), (h[3], h[4]))
            end
            if !isempty(fam)
                println("   -- rendering intruders by family (", length(fam), " families, ", nkept, " instances) --")
                for k in sort!(collect(keys(fam)), by=k->(-length(fam[k]), k))
                    v = fam[k]
                    println("      ", rpad(k,14), rpad(length(v),5),
                            " nearest |lat| ", rpad(round(minimum(first.(v)),digits=1),7),
                            " base above road ", round(minimum(last.(v)),digits=1), " … ",
                            round(maximum(last.(v)),digits=1), " m")
                end
            end
        end
        flush(stdout)

        get(ENV,"JM_OVERHANG_EXIT","") != "" && exit(0)
    end
    if get(ENV,"JM_EDGEZ_RANK","") != ""

        rows = Tuple{String,Float64,Float64,Float64,Float64,Float64}[]
        for i in insts
            groundz(i.x, i.y) > -900f0 && continue
            bd = Inf; tx = TRKCX; ty = TRKCY
            for p2 in ALIGNED
                dd = (p2[1]-i.x)^2 + (p2[2]-i.y)^2
                dd < bd && (bd = dd; tx = p2[1]; ty = p2[2])
            end
            hs = Float64[]
            for (gx, gy) in ((tx, ty), (TRKCX, TRKCY))
                dx = gx - i.x; dy = gy - i.y; d = hypot(dx,dy)
                if d < 1e-3; push!(hs, NaN); continue; end
                ux = dx/d; uy = dy/d; sd = 0.0; got = NaN
                while sd < d
                    g = groundz(Float32(i.x+ux*sd), Float32(i.y+uy*sd))
                    if g > -900f0
                        r = Float64(g)
                        for _ in 1:8
                            h = JRTrack.hat3d(TERRAIN, Float64(i.x+ux*sd), Float64(i.y+uy*sd); ref=r-0.6)
                            h[3] || break
                            r = h[1]
                        end
                        got = r; break
                    end
                    sd += 8.0
                end
                push!(hs, got)
            end
            (isnan(hs[1]) || isnan(hs[2])) && continue
            hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
            push!(rows, (i.name, hs[1], hs[2], hs[1]-hs[2], hr.found ? hr.lapdist : -1.0, Float64(i.z)))
        end
        sort!(rows, by=r->-abs(r[4]))
        println("== JM_EDGEZ_RANK: off-HAT objects by |nearest-march − centroid-march| height ==")

        println("   name          nearest   centroid   delta    authored-z   lapdist")
        for r in rows[1:min(end,12)]
            println("   ", rpad(r[1],13), rpad(round(r[2],digits=1),10), rpad(round(r[3],digits=1),11),
                    rpad(round(r[4],digits=1),9), rpad(round(r[6],digits=1),13), round(Int,r[5]))
        end
        println("   (", count(r->abs(r[4])>1.0, rows), " of ", length(rows), " off-HAT objects move >1 m)")
        flush(stdout)
    end
    if get(ENV,"JM_EDGEZ_TRACE","") != ""
        pat = lowercase(ENV["JM_EDGEZ_TRACE"])
        shown = 0
        for i in insts
            occursin(pat, lowercase(i.name)) || continue
            shown >= 4 && break
            gz0 = groundz(i.x, i.y)
            gz0 > -900f0 && continue
            shown += 1

            bd = Inf; tx = TRKCX; ty = TRKCY
            for p2 in ALIGNED
                dd = (p2[1]-i.x)^2 + (p2[2]-i.y)^2
                dd < bd && (bd = dd; tx = p2[1]; ty = p2[2])
            end
            for (lbl, gx, gy) in (("nearest-centreline", tx, ty), ("lap-centroid", TRKCX, TRKCY))
                dx = gx - i.x; dy = gy - i.y; d = hypot(dx,dy)
                d < 1e-3 && continue
                ux = dx/d; uy = dy/d; sdist = 0.0; hitx = NaN; hity = NaN; hz = -999f0
                while sdist < d
                    px = Float32(i.x + ux*sdist); py = Float32(i.y + uy*sdist)
                    g = groundz(px, py)
                    if g > -900f0; hitx=px; hity=py; hz=g; break; end
                    sdist += 8.0
                end
                if hz > -900f0

                    hs = Float64[]; r = Inf
                    for _ in 1:6
                        h = JRTrack.hat3d(TERRAIN, Float64(hitx), Float64(hity); ref=r)
                        h[3] || break
                        push!(hs, h[1]); r = h[1] - 0.6
                    end
                    println("   [edgez] ", rpad(i.name,10), rpad(lbl,20),
                            "march ", rpad(round(Int,sdist),5), "m → h=", rpad(round(hz,digits=1),7),
                            " surfaces at hit: ", join(round.(hs,digits=1), ", "))
                else
                    println("   [edgez] ", rpad(i.name,10), rpad(lbl,20), "march found NO HAT in ", round(Int,d), " m")
                end
            end
        end
        flush(stdout)
    end
    tstamp("  [E80] .. object mesh placement done; OBJECTS build begins")
    graze_mesh = get(ENV,"JM_GRAZE_MESH","0") != "0"
    OBJ_VEG_GRADE && println("  [objveg] SPAYELLOW-1: ", sum(count(v) for v in values(OBJVEG); init=0), " of ",
                             sum(length(v) for v in values(OBJVEG); init=0),
                             " placed-object PARTS are foliage -> graded like the track's vegetation ",
                             "(", round(TRACK_BRIGHT*VEG_GAIN, digits=2), "/", round(TRACK_AMB*VEG_GAIN, digits=2),
                             ") instead of the grandstand grade (1.05/0.55); JM_OBJ_VEG=0 reverts")

    _rbmesh = Dict{String,Any}()
    function objitems(i)
        RINGBANNER_DROP && NURB || return objmesh[i.name]
        h = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
        ringbanner_zone(h.found, h.lapdist, h.lateral) || return objmesh[i.name]
        ps = get(objverts, lowercase(i.name), get(objverts, i.name, nothing))
        (ps === nothing || !any(p -> lowercase(p.tex) in RINGBANNER_TEX, ps)) && return objmesh[i.name]
        get!(_rbmesh, i.name) do
            println("  [ringbanner] ", i.name, ": drawn without ", join(RINGBANNER_TEX, ","))
            Render.build_classic(filter(p -> !(lowercase(p.tex) in RINGBANNER_TEX), ps), TEXIDX)
        end
    end
    objkeep(i) = get(objmesh,i.name,nothing) !== nothing &&
                 !dropi(i) && !onroad_crowd(i) && !perp_crowd(i) && !onroad_bldg(i) && !onroad_fp(i) && tallenough(i.name) && onground(i) && !person_unsafe(i)
    global OBJECTS = [(objitems(i), Render.translate(Float32[i.x, plozfp(i), -i.y]) * Render.roty(Float32(OBJ_YAW_SIGN * objyawsign(i.name) * -i.yaw + objyawfix(i.name))) *
                       Render.rotz(Float32(OBJ_PR[1]*i.pitch)) * Render.rotx(Float32(objrollsign(i.name)*i.roll)), istree(i.name) && (graze_mesh || !(MONZA || WATGLEN)), (Float32(i.x), plozfp(i), Float32(-i.y)), lowercase(i.name))
                      for i in insts if objkeep(i)]

    global OBJNODE = [i.node for i in insts if objkeep(i)]

    global OBJBOUND = map(OBJECTS) do (items, mat, grz, opos, onm)
        pts = NTuple{3,Float32}[]
        for pp in get(objverts, onm, get(objverts, uppercase(onm), [])), k in 1:11:length(pp.verts)-10
            w = mat * Float32[pp.verts[k], pp.verts[k+1], pp.verts[k+2], 1f0]; push!(pts, (w[1], w[2], w[3]))
        end
        isempty(pts) && return ((opos[1], opos[2], opos[3]), 80f0)
        c = (sum(first, pts)/length(pts), sum(p -> p[2], pts)/length(pts), sum(last, pts)/length(pts))
        r = maximum(p -> sqrt((p[1]-c[1])^2 + (p[2]-c[2])^2 + (p[3]-c[3])^2), pts)
        (c, max(Float32(r), 5f0))
    end
    get(ENV, "JM_CULLBOUND", "1") == "0" && (OBJBOUND = [((o[4][1], o[4][2], o[4][3]), 80f0) for o in OBJECTS])
    println("  [cull] CULLBOUND-1: object bounding radii p50 ", round(sort([b[2] for b in OBJBOUND])[max(1, length(OBJBOUND)÷2)], digits=1),
            " m, max ", round(maximum([b[2] for b in OBJBOUND]; init=0f0), digits=1), " m")

    if get(ENV, "JM_OBJNEAR", "") != ""
        _q = parse.(Float64, split(ENV["JM_OBJNEAR"], ","))
        for (items, mat, grz, opos, onm) in OBJECTS
            hypot(opos[1] - _q[1], -opos[3] - _q[2]) < _q[3] || continue
            for pp in get(objverts, onm, get(objverts, uppercase(onm), []))
                v = pp.verts; lats = Float64[]; lds = Float64[]; hs = Float64[]
                for k in 1:11:length(v)-10
                    w = mat * Float32[v[k], v[k+1], v[k+2], 1f0]
                    hr = JRTrack.hat(TRKSURF, Float64(w[1]), Float64(-w[3]))
                    hr.found || continue
                    push!(lats, hr.lateral); push!(lds, hr.lapdist); push!(hs, w[2] - hr.height)
                end
                isempty(lats) && continue
                println("  [objnear] ", rpad(onm, 10), rpad(pp.tex, 9), " lat ", round.(extrema(lats), digits=1), " s ", round.(extrema(lds)), " h-road ", round.(extrema(hs), digits=1))
            end
        end
        flush(stdout)
    end

    if get(ENV,"JM_OBJDIAG","") != ""
        why = Dict{String,Int}(); kept = 0
        for i in insts
            r = get(objmesh,i.name,nothing) === nothing ? "no-mesh" :
                drop(i.name)              ? "drop() junk filter" :
                onroad_crowd(i)           ? "onroad_crowd" :
                perp_crowd(i)             ? "perp_crowd" :
                onroad_bldg(i)            ? "onroad_bldg" :
                onroad_fp(i)              ? "onroad_fp (footprint on road)" :
                !(tallenough(i.name)) ? "under 1 m tall" :
                !onground(i)              ? "not on ground" : ""
            r == "" ? (kept += 1) : (why[r] = get(why,r,0)+1)
        end
        println("== JM_OBJDIAG ", length(insts), " instances -> ", kept, " OBJECTS")
        for (k,v) in sort(collect(why), by=x->-x[2])
            println("   removed by ", rpad(k,32), v)
        end

        if get(ENV,"JM_OBJDIAG_AT","") != ""
            _f = split(ENV["JM_OBJDIAG_AT"], ":")
            _c = parse(Float64, _f[1]); _hw = length(_f) >= 2 ? parse(Float64, _f[2]) : 100.0
            _rows = Tuple{Float64,String,Float64,String}[]
            for i in insts
                _hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
                _hr.found || continue
                _d = abs(rem(_hr.lapdist - _c, TRKSURF.lap_length, RoundNearest))
                _d <= _hw || continue
                _r = get(objmesh,i.name,nothing) === nothing ? "NO MESH" :
                     drop(i.name)              ? "drop() junk filter" :
                     onroad_crowd(i)           ? "onroad_crowd" :
                     perp_crowd(i)             ? "perp_crowd" :
                     onroad_bldg(i)            ? "onroad_bldg" :
                     onroad_fp(i)              ? "onroad_fp (footprint on road)" :
                     !(tallenough(i.name)) ? string("under 1 m tall (", round(get(ymx,i.name,0f0)-get(ymn,i.name,0f0), digits=2), " m)") :
                     !onground(i)              ? "not on ground" : "kept"
                push!(_rows, (_hr.lapdist, lowercase(String(i.name)), _hr.lateral, _r))
            end
            sort!(_rows, by = x -> x[1])
            println("== JM_OBJDIAG_AT s=", _c, " +-", _hw, " m: ", length(_rows), " instance(s)")
            println("   (only the DROPPED ones are listed unless JM_OBJDIAG_AT_ALL=1)")
            _all = get(ENV,"JM_OBJDIAG_AT_ALL","") != ""
            for (_s, _n, _l, _r) in _rows
                (_all || _r != "kept") || continue
                println("      s=", lpad(round(Int,_s), 6), " lat ", lpad(round(_l, digits=1), 7),
                        "  ", rpad(_n, 12), _r == "kept" ? "kept" : string("DROPPED by ", _r))
            end
            _nd = count(r -> r[4] != "kept", _rows)
            println("   ", _nd, " dropped, ", length(_rows) - _nd, " kept in this window")
        end

        if get(ENV,"JM_OBJDIAG_NEAR","") != ""
            _f = split(ENV["JM_OBJDIAG_NEAR"], ":"); _a = findfirst(i -> lowercase(String(i.name)) == lowercase(_f[1]), insts)
            _r = length(_f) >= 2 ? parse(Float64, _f[2]) : 200.0
            if _a !== nothing
                _ax = Float64(insts[_a].x); _ay = Float64(insts[_a].y)
                println("== JM_OBJDIAG_NEAR ", _f[1], " (", round(_ax, digits=1), ", ", round(_ay, digits=1), ") r=", _r)
                _rows2 = Tuple{Float64,String}[]
                for i in insts
                    _d = hypot(Float64(i.x) - _ax, Float64(i.y) - _ay); _d <= _r || continue
                    _v = get(objmesh,i.name,nothing) === nothing ? "NO MESH" : drop(i.name) ? "drop() junk filter" :
                         onroad_crowd(i) ? "onroad_crowd" : perp_crowd(i) ? "perp_crowd" : onroad_bldg(i) ? "onroad_bldg" :
                         onroad_fp(i) ? "onroad_fp" : !(tallenough(i.name)) ? "under 1 m tall" : !onground(i) ? "not on ground" : "kept"
                    push!(_rows2, (_d, string(rpad(lowercase(String(i.name)), 12), " d ", lpad(round(Int, _d), 4), " m  at (",
                                               round(i.x, digits=1), ", ", round(i.y, digits=1), ")  ", _v)))
                end
                for r in sort(_rows2, by = first); println("      ", r[2]); end
            end
        end
        flush(stdout)
    end

    get(ENV,"JM_BLINDDIAG","") != "" && let
        _P = TRKSURF.pos; _S = TRKSURF.lapdist; _n = length(_P); _L = TRKSURF.lap_length
        _deg = parse(Float64, get(ENV,"JM_BLINDDIAG_DEG","40"))
        _hd(k) = (q = _P[mod1(k+1,_n)]; p = _P[k]; atan(q[3]-p[3], q[1]-p[1]))
        _ahead(k, d) = (j = k; acc = 0.0; while acc < d; jn = mod1(j+1,_n); acc += hypot(_P[jn][1]-_P[j][1], _P[jn][3]-_P[j][3]); j = jn; j == k && break; end; j)

        _spr = Set{Tuple{String,Int,Int}}()
        (@isdefined RINGSPRITES) && RINGSPRITES !== nothing &&
            foreach(sp -> push!(_spr, (lowercase(String(sp.name)), round(Int, sp.x), round(Int, sp.y))), RINGSPRITES)
        _vd(i) = get(objmesh,i.name,nothing) === nothing ?
                     ((lowercase(String(i.name)), round(Int, i.x), round(Int, i.y)) in _spr ? "kept (sprite)" :
                      get(bbinfo, i.name, nothing) === nothing ? "NO MESH" :
                      drop(i.name) ? "billboard, drop() junk filter" : "kept (billboard)") :
                 drop(i.name) ? "drop() junk filter" :
                 onroad_crowd(i) ? "onroad_crowd" : perp_crowd(i) ? "perp_crowd" : onroad_bldg(i) ? "onroad_bldg" :
                 onroad_fp(i) ? "onroad_fp" : !(tallenough(i.name)) ? "under min height" : !onground(i) ? "not on ground" : "kept"
        _turns = Tuple{Int,Float64}[]; k = 1; _last = -1e9
        while k <= _n
            j = _ahead(k, 60.0); dh = rad2deg(abs(rem(_hd(j) - _hd(k), 2pi, RoundNearest)))
            if dh >= _deg && rem(_S[k] - _last + _L, _L) > 120.0

                jt = _ahead(k, 150.0); tot = rem(_hd(jt) - _hd(k), 2pi, RoundNearest)
                push!(_turns, (k, rad2deg(tot))); _last = _S[k]
            end
            k += 1
        end
        println("== JM_BLINDDIAG: ", length(_turns), " turn(s) >= ", _deg, " deg in 60 m (lap ", round(Int,_L), " m)")
        _nk0 = 0
        for (k, tot) in _turns
            p = _P[k]; h = _hd(k); ux, uz = cos(h), sin(h)
            rows = Tuple{Float64,Float64,String,String}[]
            for i in insts
                dx = Float64(i.x) - p[1]; dz = Float64(i.y) - p[3]
                t = dx*ux + dz*uz; o = -dx*uz + dz*ux
                (0.0 <= t <= 200.0 && abs(o) <= 15.0) || continue
                hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
                hr.found && hr.on_track && continue
                push!(rows, (t, o, lowercase(String(i.name)), _vd(i)))
            end
            sort!(rows, by = first); nk = count(r -> startswith(r[4], "kept"), rows); nk == 0 && (_nk0 += 1)
            println("   turn s=", lpad(round(Int,_S[k]),6), "  ", lpad(round(Int,tot),4), " deg  straight-on: ",
                    nk, " kept / ", length(rows) - nk, " dropped", nk == 0 ? "   <-- NOTHING KEPT AHEAD" : "")
            for r in rows
                (!startswith(r[4], "kept") || get(ENV,"JM_BLINDDIAG_ALL","") != "") || continue
                println("        ", lpad(round(Int,r[1]),4), " m ahead ", lpad(round(r[2],digits=1),6), " m off  ",
                        rpad(r[3], 12), r[4])
            end
        end
        println("   ", _nk0, " of ", length(_turns), " turns have nothing kept straight on")
    end
    flush(stdout)

    solidR(nm) = startswith(nm,"hut")||startswith(nm,"pitbldg")||startswith(nm,"hotel")||startswith(nm,"bigbosch")||nm=="mega2"||startswith(nm,"longtent")||
                 startswith(nm,"chut")||startswith(nm,"lasad")||startswith(nm,"haus")||startswith(nm,"house")||startswith(nm,"ferme") ? 5.0 :
                 startswith(nm,"gstand")||startswith(nm,"grand")||startswith(nm,"tribun")||startswith(nm,"camstnd")||startswith(nm,"mgrand") ? 6.0 :
                 startswith(nm,"tower")||startswith(nm,"megafon") ? 2.0 :
                 startswith(nm,"haie")||startswith(nm,"bush")||startswith(nm,"shrub")||startswith(nm,"hedge")||startswith(nm,"haystk") ? 1.5 :
                 startswith(nm,"armco")||startswith(nm,"barrier")||startswith(nm,"fence")||startswith(nm,"wall") ? 1.2 :
                 startswith(nm,"caravn")||startswith(nm,"vwvan")||startswith(nm,"ftruck")||startswith(nm,"ambul")||nm=="car2"||startswith(nm,"rescu") ? 2.4 : 0.0

    GEOM_WMAX = parse(Float64, get(ENV,"JM_GEOM_WMAX","40.0"))

    GEOM_RMAX = parse(Float64, get(ENV,"JM_GEOM_RMAX","8.0"))

    person_like(n) = occursin(r"^(flagger|peo|ppl|crowd|grndpe|spect|marshal|pit(ppl|peo))", n)

    ground_like(n) = occursin(r"^(fill|.*fillgrnd|pitgrnd|pitfill|grnd|ground|terr|ter\d)", n)

    backdrop_like(n) = occursin(r"bk\d*$", n)
    _geomwhy = Dict{String,String}()
    function geomR(orig, nm)
        person_like(nm) && (_geomwhy[nm] = "person"; return 0.0)
        ground_like(nm) && (_geomwhy[nm] = "terrain"; return 0.0)
        backdrop_like(nm) && (_geomwhy[nm] = "backdrop"; return 0.0)
        haskey(lxmn, orig) || (_geomwhy[nm] = "no-mesh"; return 0.0)
        w = lxmx[orig] - lxmn[orig]; d = lzmx[orig] - lzmn[orig]
        h = get(ymx, orig, 0f0) - get(ymn, orig, 0f0)
        dims = "$(round(w,digits=1))x$(round(d,digits=1))x$(round(h,digits=1))"
        h > 1.0f0 || (_geomwhy[nm] = "flat $dims"; return 0.0)

        if max(w, d) > GEOM_WMAX
            _geomwhy[nm] = "wide $dims (backdrop/composite)"; return 0.0
        end
        r = Float64(min(w, d)) / 2
        r < 0.5      && (_geomwhy[nm] = "thin $dims (backdrop panel)"; return 0.0)
        r > GEOM_RMAX && (_geomwhy[nm] = "huge $dims (composite .3do)"; return 0.0)
        _geomwhy[nm] = "SOLID r=$(round(r,digits=1)) $dims"
        r
    end

    _boxrej = String[]; _discshr = String[]
    function box_covers_tarmac(cx, cz, hx, hz, ψ)
        (ROADHAT === TERRAIN0 || get(ENV, "JM_SOLID_ROADCHECK", "1") == "0") && return false

        if min(hx, hz) < 0.5
            if JRTrack.hat3d(ROADHAT, cx, cz; ref = Inf)[3]
                hr0 = JRTrack.hat(TRKSURF, cx, cz)
                (hr0.found && hr0.on_track) && return true
            end
            return false
        end
        c = cos(ψ); sn = sin(ψ)
        nx = max(3, ceil(Int, 5hx)); nz = max(3, ceil(Int, 5hz))
        for ix in 0:nx, iz in 0:nz
            lx = -hx + 2hx*ix/nx; lz = -hz + 2hz*iz/nz
            px = cx + c*lx + sn*lz; pz = cz + sn*lx - c*lz

            if JRTrack.hat3d(ROADHAT, px, pz; ref = Inf)[3]
                hr = JRTrack.hat(TRKSURF, px, pz)
                (hr.found && hr.on_track) && return true
            end
        end
        false
    end

    function disc_clear_radius(ox, oz, r)
        (ROADHAT === TERRAIN0 || get(ENV, "JM_SOLID_ROADCHECK", "1") == "0") && return r
        best = r
        n = max(3, ceil(Int, 5r))
        for ix in -n:n, iz in -n:n
            px = ox + r*ix/n; pz = oz + r*iz/n

            d = hypot(px - ox, pz - oz) - CARW - 0.1; d >= best && continue
            if JRTrack.hat3d(ROADHAT, px, pz; ref = Inf)[3]
                hr = JRTrack.hat(TRKSURF, px, pz)
                (hr.found && hr.on_track) && (best = max(d, 0.0))
            end
        end
        best
    end
    global SOLIDS = Tuple{Float64,Float64,Float64,Symbol}[]
    global SOLIDNAMES = String[]
    _solidseen = Set{Tuple{Float64,Float64,Float64,Symbol}}(); _soliddup = Ref(0)
    _solidtilt = String[]
    empty!(SOLIDBOX)
    _geomn = 0; _undrawn_solid = Ref(0)
    for i in insts
        nml = lowercase(i.name)
        r = solidR(nml)
        if r <= 0.0 && get(ENV,"JM_GEOMSOLID","1") != "0"
            r = geomR(i.name, nml); r > 0.0 && (_geomn += 1)
        end
        (r <= 0.0 || !onground(i)) && continue
        on_road(i.x, i.y, SOLID_EXCL_HW) && continue

        if get(ENV, "JM_SOLID_UNDRAWN", "0") == "0" && get(objmesh, i.name, nothing) !== nothing &&
           (dropi(i) || onroad_crowd(i) || perp_crowd(i) || onroad_bldg(i) || onroad_fp(i))
            _undrawn_solid[] += 1; continue
        end

        if get(ENV,"JM_SOLID_KEEP_HIDDEN","0") == "0"
            (onroad_fp(i) || onroad_bldg(i) || onroad_crowd(i) || perp_crowd(i)) && continue

            solid_exempt(n) = startswith(n,"armco") || startswith(n,"fence") || startswith(n,"rail") ||
                              startswith(n,"barrier") || startswith(n,"wall") || startswith(n,"bushrow") ||
                              startswith(n,"haie")

            if !solid_exempt(nml) && get(objmesh, i.name, nothing) !== nothing
                drop(i.name) && continue
                (get(ymx,i.name,0f0) - get(ymn,i.name,0f0)) > 1.0f0 || continue
            end
        end

        _tilted = (abs(i.pitch) > 0.35 || abs(i.roll) > 0.35) && get(ENV, "JM_SOLID_TILTED", "0") == "0"

        _key = (Float64(i.x), Float64(i.y), r, solidkind(nml))
        if _key in _solidseen; _soliddup[] += 1; continue; end
        push!(_solidseen, _key)
        push!(SOLIDS, _key)
        push!(SOLIDNAMES, nml)

        if INVIS_SOLIDS_DIAG && solid_exempt(nml) && !objkeep(i)
            _h = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
            push!(INVIS_SOLID_ROWS, (_h.found ? _h.lapdist : NaN, _h.found ? _h.lateral : NaN, nml, r))
        end

        _boxable = r >= parse(Float64, get(ENV, "JM_SOLID_BOX_R", "1.2")) && haskey(lxmn, i.name) && get(ENV, "JM_SOLID_BOX", "1") != "0"
        _boxable && _tilted && push!(_solidtilt, nml)
        if _boxable && !_tilted
            ψ = Float64(OBJ_YAW_SIGN * -i.yaw + objyawfix(i.name)); c = cos(ψ); sn = sin(ψ)
            wx = Float64(lxmx[i.name] - lxmn[i.name]); wz = Float64(lzmx[i.name] - lzmn[i.name])
            pv = get(objverts, i.name, nothing)
            if max(wx, wz) > 2*GEOM_RMAX && pv !== nothing && length(pv) >= 2 && get(ENV, "JM_SOLID_PARTS", "1") != "0"
                nb = 0
                for pp in pv
                    xl = Inf; xh = -Inf; zl = Inf; zh = -Inf
                    for k in 1:11:length(pp.verts)
                        xl = min(xl, pp.verts[k]); xh = max(xh, pp.verts[k]); zl = min(zl, pp.verts[k+2]); zh = max(zh, pp.verts[k+2])
                    end
                    (isfinite(xl) && isfinite(zl)) || continue
                    hx = Float64(xh - xl)/2; hz = Float64(zh - zl)/2
                    (min(hx, hz) < 0.25 || max(hx, hz) > 60.0) && continue
                    clx = Float64(xh + xl)/2; clz = Float64(zh + zl)/2
                    ent = (Float64(i.x) + c*clx + sn*clz, Float64(i.y) + sn*clx - c*clz, r, solidkind(nml))
                    if box_covers_tarmac(ent[1], ent[2], hx, hz, ψ)
                        push!(_boxrej, string(nml, "@", round(Int, ent[1]), ",", round(Int, ent[2]))); continue
                    end
                    if nb == 0
                        SOLIDS[end] = ent; push!(SOLIDBOX, (hx, hz, ψ))
                    else
                        push!(SOLIDS, ent); push!(SOLIDNAMES, nml); push!(SOLIDBOX, (hx, hz, ψ))
                    end
                    nb += 1
                end
                nb == 0 && push!(SOLIDBOX, nothing)
            else
                hx = wx/2; hz = wz/2
                clx = Float64(lxmx[i.name] + lxmn[i.name])/2; clz = Float64(lzmx[i.name] + lzmn[i.name])/2
                bcx = Float64(i.x) + c*clx + sn*clz; bcz = Float64(i.y) + sn*clx - c*clz
                if hx < 0.15 && hz < 0.15
                    push!(SOLIDBOX, nothing)
                elseif box_covers_tarmac(bcx, bcz, hx, hz, ψ)
                    push!(_boxrej, string(nml, "@", round(Int, bcx), ",", round(Int, bcz)))
                    push!(SOLIDBOX, nothing)
                else
                    SOLIDS[end] = (bcx, bcz, r, solidkind(nml))
                    push!(SOLIDBOX, (hx, hz, ψ))
                end
            end
        else
            push!(SOLIDBOX, nothing)
        end
    end
    isempty(_solidtilt) || println("  [solid] E60-S6: ", length(_solidtilt), " tilted placement(s) (> 20° pitch/roll) given a disc, not a footprint box: ",
                                   join(sort(unique(_solidtilt)), ", "), " (JM_SOLID_TILTED=1 boxes them)")

    if ROADHAT !== TERRAIN0 && get(ENV, "JM_SOLID_ROADCHECK", "1") != "0"
        for k in eachindex(SOLIDS)
            (k <= length(SOLIDBOX) && SOLIDBOX[k] === nothing) || continue
            (ox, oz, r, kd) = SOLIDS[k]
            r2 = disc_clear_radius(ox, oz, r)
            if r2 < r - 0.05
                push!(_discshr, string(k <= length(SOLIDNAMES) ? SOLIDNAMES[k] : "?", "@", round(Int, ox), ",", round(Int, oz),
                                       " ", round(r, digits=1), "→", round(max(r2, 0.0), digits=1)))

                SOLIDS[k] = (ox, oz, r2 < 0.5 ? -1000.0 : r2, kd)
            end
        end
    end
    isempty(_discshr) || println("  ROAD-1: ", length(_discshr), " disc(s) shrunk to clear the road: ",
                                 join(_discshr[1:min(end, 12)], " "), length(_discshr) > 12 ? " …" : "")
    isempty(_boxrej) || println("  ROAD-1: ", length(_boxrej), " solid box(es) rejected for covering tarmac (disc fallback): ",
                                join(unique(_boxrej)[1:min(end, 12)], " "), length(unique(_boxrej)) > 12 ? " …" : "")

    if get(ENV,"JM_RAIL_SOLID","1") != "0"
        let cell = parse(Float64, get(ENV,"JM_RAIL_CELL","8.0")),
            acc = Dict{Tuple{Int,Int},Vector{Float64}}(),
            railtex2(tx) = (lt = lowercase(String(tx)); startswith(lt,"armco") || startswith(lt,"fenc") ||
                            startswith(lt,"stfce") || startswith(lt,"sarmc") || startswith(lt,"yarmc") ||
                            startswith(lt,"gd_rail") || startswith(lt,"rail") || startswith(lt,"brdgarm") ||
                            startswith(lt,"brdgfen")),
            ntri = 0
            _rbn = (e = get(ENV, "JM_RAILBOX_NEAR", ""); isempty(e) ? nothing : parse.(Float64, split(e, ",")))
            RAIL_ZSIGN = get(ENV, "JM_RAIL_MIRROR_OLD", "0") != "0" ? 1f0 : -1f0
            RAIL_OBB = get(ENV, "JM_RAIL_OBB", "1") != "0"
            railpts = Dict{Tuple{Int,Int},Vector{Tuple{Float64,Float64}}}()
            for prt in TRACKMAIN
                railtex2(prt.tex) || continue
                v = prt.verts; n = length(v) ÷ 11
                for t in 0:3:(n-3)
                    xs = (v[11t+1], v[11(t+1)+1], v[11(t+2)+1])
                    ys = (v[11t+2], v[11(t+1)+2], v[11(t+2)+2])

                    zs = RAIL_ZSIGN .* (v[11t+3], v[11(t+1)+3], v[11(t+2)+3])
                    cx = sum(xs)/3; cz = sum(zs)/3
                    if _rbn !== nothing && hypot(cx - _rbn[1], cz - _rbn[2]) < _rbn[3]
                        _hr = JRTrack.hat(TRKSURF, cx, cz)
                        println("  [railnear] ", rpad(prt.tex, 9), " c=(", round(cx, digits=1), ",", round(cz, digits=1), ") x ", round.(extrema(xs), digits=1),
                                " z ", round.(extrema(zs), digits=1), " y ", round.(extrema(ys), digits=2), "  ribbon ", _hr.found ? "lat $(round(_hr.lateral, digits=1)) h $(round(_hr.height, digits=2))" : "none")
                    end
                    kx = floor(Int, cx/cell); kz = floor(Int, cz/cell)
                    k = (kx, kz)
                    e = get!(acc, k, [Inf, -Inf, Inf, -Inf, Inf, -Inf])

                    lo_x = kx*cell; hi_x = (kx+1)*cell; lo_z = kz*cell; hi_z = (kz+1)*cell
                    e[1] = min(e[1], clamp(minimum(xs), lo_x, hi_x)); e[2] = max(e[2], clamp(maximum(xs), lo_x, hi_x))
                    e[3] = min(e[3], clamp(minimum(zs), lo_z, hi_z)); e[4] = max(e[4], clamp(maximum(zs), lo_z, hi_z))
                    e[5] = min(e[5], minimum(ys)); e[6] = max(e[6], maximum(ys))
                    pv = get!(railpts, k, Tuple{Float64,Float64}[])
                    for c_ in 1:3; push!(pv, (clamp(Float64(xs[c_]), lo_x, hi_x), clamp(Float64(zs[c_]), lo_z, hi_z))); end
                    ntri += 1
                end
            end
            nadded = 0; nreject = 0; nover = 0; nobb = 0; ntrim = 0
            for (kc, e) in acc
                hx = (e[2] - e[1]) / 2; hz = (e[4] - e[3]) / 2
                (hx < 0.05 && hz < 0.05) && continue
                ox = (e[1] + e[2]) / 2; oz = (e[3] + e[4]) / 2; ψb = 0.0

                if RAIL_OBB && length(get(railpts, kc, ())) >= 3
                    pv = railpts[kc]; mx = sum(first, pv)/length(pv); mz = sum(last, pv)/length(pv)
                    sxx = sum(p -> (p[1]-mx)^2, pv); szz = sum(p -> (p[2]-mz)^2, pv); sxz = sum(p -> (p[1]-mx)*(p[2]-mz), pv)
                    φ = 0.5*atan(2sxz, sxx - szz); c_ = cos(φ); s_ = sin(φ)
                    u = [c_*(p[1]-mx) + s_*(p[2]-mz) for p in pv]; w = [-s_*(p[1]-mx) + c_*(p[2]-mz) for p in pv]
                    ox = mx + c_*(maximum(u)+minimum(u))/2 - s_*(maximum(w)+minimum(w))/2
                    oz = mz + s_*(maximum(u)+minimum(u))/2 + c_*(maximum(w)+minimum(w))/2
                    hx = max((maximum(u) - minimum(u))/2, 0.15); hz = max((maximum(w) - minimum(w))/2, 0.15); ψb = φ

                    ns = max(3, ceil(Int, 2hx/0.4) + 1); ls = range(-hx, hx; length = ns); off = trues(ns)
                    for (i, lx) in enumerate(ls), lz in (-hz, 0.0, hz)
                        off[i] || continue
                        qx = ox + c_*lx - s_*lz; qz = oz + s_*lx + c_*lz
                        if JRTrack.hat3d(ROADHAT, qx, qz; ref = Inf)[3]
                            hq = JRTrack.hat(TRKSURF, qx, qz); (hq.found && hq.on_track) && (off[i] = false)
                        end
                    end
                    if all(off)
                        cands = [(ox, oz, hx, hz, ψb)]
                    else
                        cands = NTuple{5,Float64}[]; i = 1
                        while i <= ns
                            off[i] || (i += 1; continue)
                            j = i; while j < ns && off[j+1]; j += 1; end

                            l0 = ls[i] + (i > 1 ? step(ls)/2 : 0.0); l1 = ls[j] - (j < ns ? step(ls)/2 : 0.0)
                            if l1 - l0 >= 0.8
                                lm = (l0 + l1)/2
                                push!(cands, (ox + c_*lm, oz + s_*lm, (l1 - l0)/2, hz, ψb))
                            end
                            i = j + 1
                        end
                        isempty(cands) ? (nreject += 1) : (ntrim += 1)
                        if isempty(cands) && get(ENV, "JM_RAILBOX_DIAG", "0") != "0"
                            hr_ = JRTrack.hat(TRKSURF, ox, oz)
                            rd = hr_.found ? rad2deg(abs(rem(φ - atan(hr_.perp[2], hr_.perp[1]) - pi/2, pi, RoundNearest))) : NaN
                            println("  [railbox] rejected: s ", hr_.found ? round(Int, hr_.lapdist) : -1, " lat ",
                                    hr_.found ? round(hr_.lateral, digits=1) : NaN, " len ", round(2hx, digits=1),
                                    " angle-to-road ", round(rd, digits=0), " y-above-road ",
                                    hr_.found ? round(e[5] - hr_.height, digits=1) : NaN)
                        end
                    end
                    nobb += 1
                else
                    cands = [(ox, oz, hx, hz, ψb)]
                end
                for (ox, oz, hx, hz, ψb) in cands
                    if ψb == 0.0 && box_covers_tarmac(ox, oz, hx, hz, 0.0)
                        nreject += 1
                        continue
                    end
                    ovh = let hr = JRTrack.hat(TRKSURF, ox, oz), gr = JRTrack.hat3d(TERRAIN0, ox, oz; ref = e[5] - 0.5)
                        groundy = hr.found ? hr.height : (gr[3] ? Float64(gr[1]) : NaN)
                        if isfinite(groundy) && e[5] - groundy > parse(Float64, get(ENV, "JM_RAIL_OVERHEAD", "2.0"))
                            get(ENV, "JM_RAILBOX_DIAG", "0") != "0" && println("  [railbox] overhead: (", round(ox, digits=1), ", ", round(oz, digits=1),
                                ") rail y ", round(e[5], digits=1), "..", round(e[6], digits=1), " road ", round(groundy, digits=1))
                            true
                        else
                            false
                        end
                    end
                    ovh && (nover += 1; continue)
                    push!(SOLIDS, (ox, oz, max(hx, hz), :wall))
                    push!(SOLIDNAMES, "railbox")
                    push!(SOLIDBOX, (hx, hz, ψb))
                    nadded += 1
                end
            end
            println("== JM_RAIL_SOLID: ", ntri, " rail tris -> ", nadded, " collision boxes of <=",
                    cell, " m (", RAIL_OBB ? "$(nobb) oriented cells, $(ntrim) trimmed off the road, " : "axis-aligned, ", nreject, " rejected for covering tarmac, ", nover, " overhead); solids ",
                    length(SOLIDS) - nadded, " -> ", length(SOLIDS))
            flush(stdout)
        end
    end

    if get(ENV, "JM_RAILHIT_SELFTEST", "0") != "0"
        ib = findall(==("railbox"), SOLIDNAMES); hit = 0; ctl = 0; fp = 0
        nprobe = 0
        for (j, k) in enumerate(ib)
            (ox, oz, r, _) = SOLIDS[k]

            hr = JRTrack.hat(TRKSURF, ox, oz)
            if hr.found
                sg = hr.lateral >= 0 ? -1.0 : 1.0
                px = ox; pz = oz
                for d in 0.2:0.1:6.0
                    qx = ox + sg*d*hr.perp[1]; qz = oz + sg*d*hr.perp[2]
                    th_ = atan(oz - qz, ox - qx)
                    car_gap(qx, qz, th_, k)[1] < 0.0 ? (px = qx; pz = qz) : break
                end
                if (px, pz) != (ox, oz)
                    nprobe += 1
                    solid_hit(px, pz, atan(oz - pz, ox - px), 80/3.6) === nothing || (hit += 1)
                end
            end
            th = 2pi*j/max(length(ib), 1); px = ox + 20cos(th); pz = oz + 20sin(th)
            all(s -> hypot(px - s[1], pz - s[2]) - s[3] > 8.0, SOLIDS) || continue
            ctl += 1; solid_hit(px, pz, th, 80/3.6) === nothing || (fp += 1)
        end
        println("RAILHIT_RESULT track=", TRACKSEL, " railboxes=", length(ib), " probed=", nprobe, " contact=", hit,
                " control_points=", ctl, " control_contacts=", fp, " solids=", length(SOLIDS))
        flush(stdout); exit(0)
    end

    if get(ENV,"JM_SOLIDHIT","") != ""
        for spec2 in split(get(ENV,"JM_SOLIDHIT",""), ";")
            pr = split(spec2, ",")
            length(pr) >= 4 || continue
            hx0 = parse(Float64, strip(pr[1])); hz0 = parse(Float64, strip(pr[2]))
            hth = deg2rad(parse(Float64, strip(pr[3]))); hv = parse(Float64, strip(pr[4]))/3.6

            let nb = parse(Int, get(ENV,"JM_SOLIDHIT_BENCH","0"))
                if nb > 0
                    solid_hit(hx0, hz0, hth, hv)
                    t0 = time_ns()
                    for _ in 1:nb; solid_hit(hx0, hz0, hth, hv); end
                    dt = (time_ns() - t0) / 1e9
                    println("== JM_SOLIDHIT_BENCH ", nb, " calls in ", round(dt, digits=3), " s = ",
                            round(1e6*dt/nb, digits=3), " us/call   (", length(SOLIDS), " solids)")

                    function _bench_typed(x, z, th, v, solids::Vector{Tuple{Float64,Float64,Float64,Symbol}},
                                          boxes::Vector{Union{Nothing,NTuple{3,Float64}}})
                        acc = 0.0
                        @inbounds for k in eachindex(solids)
                            (ox, oz, r, _) = solids[k]
                            b = k <= length(boxes) ? boxes[k] : nothing
                            g = b === nothing ? hypot(x-ox, z-oz) - r :
                                                max(abs((x-ox)*cos(b[3]) + (z-oz)*sin(b[3])) - b[1],
                                                    abs(-(x-ox)*sin(b[3]) + (z-oz)*cos(b[3])) - b[2])
                            acc += g
                        end
                        acc
                    end
                    _bench_typed(hx0, hz0, hth, hv, SOLIDS, SOLIDBOX)
                    t1 = time_ns()
                    for _ in 1:nb; _bench_typed(hx0, hz0, hth, hv, SOLIDS, SOLIDBOX); end
                    dt1 = (time_ns() - t1) / 1e9
                    println("== JM_SOLIDHIT_BENCH typed-args equivalent: ", round(dt1, digits=3), " s = ",
                            round(1e6*dt1/nb, digits=3), " us/call  ->  ", round(dt/max(dt1,1e-9), digits=1),
                            "x faster than the global-reading loop")
                end
            end
            res = solid_hit(hx0, hz0, hth, hv)
            if res === nothing
                println("== JM_SOLIDHIT (", hx0, ", ", hz0, ") hdg ", strip(pr[3]), "deg ",
                        strip(pr[4]), "km/h  ->  NO CONTACT")
            else
                println("== JM_SOLIDHIT (", hx0, ", ", hz0, ") hdg ", strip(pr[3]), "deg ",
                        strip(pr[4]), "km/h  ->  CONTACT  impulse dvx=", round(res[1],digits=2),
                        " dvz=", round(res[2],digits=2), " dr=", round(res[3],digits=3),
                        " lift=", round(res[4],digits=2))
            end
        end
        flush(stdout)
    end

    if get(ENV,"JM_SOLIDNEAR","")!=""
      for spec1 in split(get(ENV,"JM_SOLIDNEAR",""), ";")
        let pr = split(spec1, ",")
            qx = parse(Float64, strip(pr[1])); qz = parse(Float64, strip(pr[2]))
            qr = length(pr) >= 3 ? parse(Float64, strip(pr[3])) : 60.0
            println("== JM_SOLIDNEAR solids within ", qr, " m of (", qx, ", ", qz, ") ==")
            rows = [(hypot(ox-qx, oz-qz), ox, oz, r, kind, i) for (i,(ox,oz,r,kind)) in enumerate(SOLIDS) if hypot(ox-qx, oz-qz) <= qr]
            sort!(rows)
            for (d, ox, oz, r, kind, i) in rows
                nm = i <= length(SOLIDNAMES) ? SOLIDNAMES[i] : "?"
                println("   ", rpad(nm, 10), " kind=", rpad(string(kind), 6), " r=", lpad(round(r, digits=1), 5),
                        "  at (", round(ox, digits=1), ", ", round(oz, digits=1), ")  dist=", round(d, digits=1), " m")
            end
            isempty(rows) && println("   (none)")
            for (d, ox, oz, r, kind, i) in rows
                b = SOLIDBOX[i]; b === nothing && continue
                println("   box ", rpad(SOLIDNAMES[i], 10), " half ", round(b[1], digits=1), " x ", round(b[2], digits=1),
                        " m, yaw ", round(rad2deg(b[3]), digits=1), "°, centre (", round(ox, digits=1), ", ", round(oz, digits=1), ")")

                let c = cos(b[3]), sn = sin(b[3])
                    cx = [ox + sx*b[1]*c + sz*b[2]*sn for sx in (-1,1), sz in (-1,1)]
                    cz = [oz + sx*b[1]*sn - sz*b[2]*c for sx in (-1,1), sz in (-1,1)]
                    println("      box corners   x ", round(minimum(cx), digits=1), "..", round(maximum(cx), digits=1),
                            "  z ", round(minimum(cz), digits=1), "..", round(maximum(cz), digits=1))
                end
                for inst in insts
                    lowercase(inst.name) == SOLIDNAMES[i] || continue
                    hypot(Float64(inst.x) - ox, Float64(inst.y) - oz) <= 20.0 || continue
                    pv = get(objverts, inst.name, nothing); pv === nothing && continue
                    M = Render.translate(Float32[inst.x, plozfp(inst), -inst.y]) * Render.roty(Float32(OBJ_YAW_SIGN * -inst.yaw + objyawfix(inst.name)))
                    xlo = Inf; xhi = -Inf; zlo = Inf; zhi = -Inf
                    for pp in pv, k in 1:11:length(pp.verts)
                        q = M * Float32[pp.verts[k], pp.verts[k+1], pp.verts[k+2], 1f0]
                        px = Float64(q[1]); pz = -Float64(q[3])
                        xlo = min(xlo, px); xhi = max(xhi, px); zlo = min(zlo, pz); zhi = max(zhi, pz)
                    end
                    println("      drawn vertices x ", round(xlo, digits=1), "..", round(xhi, digits=1),
                            "  z ", round(zlo, digits=1), "..", round(zhi, digits=1), "   (origin ", round(Float64(inst.x), digits=1), ", ", round(Float64(inst.y), digits=1), ")")
                    break
                end
                spec = get(ENV, "JM_HATPROBE", "")
                occursin(";", spec) || continue
                for tok in split(spec, ";")
                    isempty(strip(tok)) && continue
                    pr = split(tok, ","); px = parse(Float64, strip(pr[1])); pz = parse(Float64, strip(pr[2]))
                    (g, nx, nz) = box_gap(px, pz, ox, oz, b[1], b[2], b[3])
                    (gd, _, _) = disc_gap(px, pz, ox, oz, r)
                    println("      at (", round(px, digits=1), ", ", round(pz, digits=1), ")  box gap ", round(g, digits=2),
                            "  disc gap ", round(gd, digits=2), (g < CARHALF ? "  <- CONTACT" : ""))
                end
            end

            ndup = length(SOLIDS) - length(unique(SOLIDS))
            println("   SOLIDS total ", length(SOLIDS), ", exact duplicates left ", ndup, " (", _soliddup[], " dropped at build)")

            println("   placements within ", qr, " m and whether the renderer keeps them:")
            for i in insts
                d = hypot(Float64(i.x) - qx, Float64(i.y) - qz); d <= qr || continue
                r = get(objmesh,i.name,nothing) === nothing ? "no-mesh" :
                    drop(i.name)              ? "drop() junk filter" :
                    onroad_crowd(i)           ? "onroad_crowd" :
                    perp_crowd(i)             ? "perp_crowd" :
                    onroad_bldg(i)            ? "onroad_bldg" :
                    onroad_fp(i)              ? "onroad_fp (footprint on road)" :
                    !(tallenough(i.name)) ? "under 1 m tall" :
                    !onground(i)              ? "not on ground" : "DRAWN"
                println("   ", rpad(i.name, 10), " at (", round(Float64(i.x), digits=1), ", ", round(Float64(i.y), digits=1),
                        ")  dist=", round(d, digits=1), " m  -> ", r)
            end
        end
      end

      get(ENV, "JM_SOLIDNEAR_CONTINUE", "0") == "0" && (flush(stdout); exit(0))
    end
    if INVIS_SOLIDS_DIAG
        println("  [invis solids] ", TRACKSEL, ": ", length(INVIS_SOLID_ROWS), " exempt-class solids whose object is not drawn")
        for r in sort(INVIS_SOLID_ROWS, by = x -> abs(x[2])); println("      s ", isnan(r[1]) ? "off-ribbon" : round(Int, r[1]), " lat ", round(r[2], digits = 1), "  ", r[3], "  r ", r[4]); end
    end
    if get(ENV,"JM_SOLIDDIAG","")!=""

        cnt = Dict{String,Int}(); for n in SOLIDNAMES; cnt[n]=get(cnt,n,0)+1; end

        let seen = Set{String}()
            println("== JM_SOLIDDIAG whitelist radius vs mesh footprint (name: r_list vs half-extents w/2 x d/2) ==")
            for i in insts
                nml = lowercase(i.name); nml in seen && continue
                rl = solidR(nml); rl > 0.0 || continue
                haskey(lxmn, i.name) || continue
                push!(seen, nml)
                w = Float64(lxmx[i.name] - lxmn[i.name]); d = Float64(lzmx[i.name] - lzmn[i.name])
                hmin = min(w, d)/2; hmax = max(w, d)/2
                tag = rl > hmax + 0.5 ? "AIR-HIT (r > long half-extent)" :
                      rl > hmin + 0.5 ? "air on the short side" :
                      rl < hmin - 1.0 ? "PENETRABLE (r < short half-extent)" : "ok"
                println("   ", rpad(nml, 10), " r=", lpad(rl, 4), "  footprint ", round(w, digits=1), " x ", round(d, digits=1),
                        "  half ", round(hmin, digits=1), "/", round(hmax, digits=1), "  ", tag)
            end
        end

    if get(ENV,"JM_RAILSOLID","0") != "0"
        railtex(tx) = (lt = lowercase(String(tx)); startswith(lt,"armco") || startswith(lt,"fenc") ||
                        startswith(lt,"stfce") || startswith(lt,"sarmc") || startswith(lt,"yarmc") ||
                        startswith(lt,"gd_rail") || startswith(lt,"rail") || startswith(lt,"brdgarm") ||
                        startswith(lt,"brdgfen"))
        drawn = 0; drawntris = 0
        for prt in TRACKMAIN
            if railtex(prt.tex); drawn += 1; drawntris += length(prt.verts) ÷ 33; end
        end
        solid = count(railtex, SOLIDNAMES)
        println("== JM_RAILSOLID track=", TRACKSEL, "  DRAWN rail/fence parts=", drawn,
                " (", drawntris, " tris)   COLLIDABLE rail/fence solids=", solid,
                "   solids total=", length(SOLIDS))

        let cell = 8.0, cells = Set{Tuple{Int,Int}}(), ntri = 0
            for prt in TRACKMAIN
                railtex(prt.tex) || continue
                v = prt.verts; n = length(v) ÷ 11
                for t in 0:3:(n-3)
                    cx = (v[11t+1] + v[11(t+1)+1] + v[11(t+2)+1]) / 3
                    cz = (v[11t+3] + v[11(t+1)+3] + v[11(t+2)+3]) / 3
                    push!(cells, (floor(Int, cx/cell), floor(Int, cz/cell)))
                    ntri += 1
                end
            end
            println("== JM_RAILSOLID sizing: ", ntri, " rail tris occupy ", length(cells),
                    " cells of ", cell, " m  ->  solids would go ", length(SOLIDS), " -> ",
                    length(SOLIDS) + length(cells),
                    " (x", round((length(SOLIDS) + length(cells)) / max(length(SOLIDS),1), digits=1),
                    " on a linear per-car-per-tick scan)")
        end
        flush(stdout)
    end

        println("== JM_SOLIDDIAG boxes: ", count(b -> b !== nothing, SOLIDBOX), " of ", length(SOLIDS), " solids carry a mesh-footprint box (SOLID-BOX)")
        println("== JM_SOLIDDIAG ", length(SOLIDS), " solids (", _geomn, " candidates from the shape rule): ",
                join(["$(n)×$(c)" for (n,c) in sort(collect(cnt))], ", "))

        rej = Dict{String,Vector{String}}()
        for (n,why) in _geomwhy
            startswith(why,"SOLID") && continue
            k = split(why)[1]; push!(get!(rej, k, String[]), n)
        end
        for (k,v) in sort(collect(rej))
            sort!(v)
            println("   shape rule rejected ", rpad(k,7), " ×", rpad(length(v),4), " e.g. ", join(v[1:min(end,8)], " "))
        end
        flush(stdout)
    end

    WIDE_PANEL = parse(Float32, get(ENV,"JM_WIDE_PANEL","30"))

    DROP_FOREST = get(ENV,"JM_DROP_FOREST", (MONZA || WATGLEN) ? "0" : "1")!="0"
    tstamp("  [E80] .. OBJECTS built; billboard/tree loop begins")
    global BILLBOARDS = Tuple{Render.Item,NTuple{3,Float32},Float32,Float32}[]
    global STATICTREES = Tuple{Render.Item,NTuple{3,Float32},Float32,Float32,Float32}[]
    _bbwhy = get(ENV, "JM_BBDIAG", "") != ""

    _stubkeep = CLASSICW_ON && get(ENV, "JM_CLASSICSTUB_KEEP", "1") != "0"
    classic_offroad(x, y) = _stubkeep && let r = classicw_locate_any(Float64(x), Float64(y))
        r === nothing && return false
        (lo, hi) = classicw_road_interval(r[1], r[2])
        isfinite(lo) && isfinite(hi) && (r[3] < lo - 1.0 || r[3] > hi + 1.0)
    end
    for i in insts
        bb = get(bbinfo, i.name, nothing); bb === nothing && continue
        dropi(i) && (_bbwhy && println("  JM_BBDIAG ", i.name, ": drop()"); true) && continue
        person_unsafe(i) && continue

        if standcrowd(i.name)
            _w = 0.0
            let b = get(bbinfo, i.name, nothing)
                b !== nothing && (_w = Float64(b[5]) > 0 ? Float64(b[5]) : 0.0)
            end
            _hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))

            _reach = 0.0
            if _hr.found
                _ry = Float64(i.yaw) - atan(-_hr.perp[2], _hr.perp[1])
                _reach = (_w/2) * abs(sin(_ry))
            end
            if _hr.found && abs(_hr.lateral) - _reach < ROAD_HALFW + 1.5
                get(ENV,"JM_CROWDROW_DIAG","") != "" &&
                    println("  E79: crowd row ", i.name, " reaches ", round(abs(_hr.lateral)-_reach, digits=1),
                            " m of the centreline (origin ", round(abs(_hr.lateral),digits=1),
                            ", width ", round(_w,digits=1), ", across-road reach ",
                            round(_reach,digits=1), ") — dropped")
                continue
            end
        end
        onground(i) || (_bbwhy && println("  JM_BBDIAG ", i.name, ": not on the ground"); false) || continue

        on_road(i.x, i.y, spaflags_keep(i) ? 6.0 : ROAD_HALFW) && !classic_offroad(i.x, i.y) && (_bbwhy && println("  JM_BBDIAG ", i.name, ": on the road (E31)"); true) && continue
        gz = ploz(i)
        item, tw, th, h, wid, aax = bb
        w = wid > 0f0 ? wid : h*tw/max(th,1f0)

        eyaw = Float64(i.yaw) + (get(ENV,"JM_AAX","0") != "0" ? Float64(aax) : 0.0)
        if w > WIDE_PANEL

            spans_road = false
            if !DROP_FOREST

                sx, sy = cos(eyaw), sin(eyaw)
                sawpos = false; sawneg = false
                for f in -0.5:0.0625:0.5
                    hr = JRTrack.hat(TRKSURF, Float64(i.x + sx*f*w), Float64(i.y + sy*f*w))
                    (hr.found && abs(hr.lateral) < ROAD_HALFW) || continue
                    hr.lateral >= 0 ? (sawpos = true) : (sawneg = true)
                    if sawpos && sawneg; spans_road = true; break; end
                end
                spans_road && println("  E65: forest panel ", i.name, " (", round(Int,h), "×", round(Int,w), " m) CROSSES the road — dropped")
            end

            (DROP_FOREST || spans_road) || push!(STATICTREES, (item, (Float32(i.x), gz, Float32(-i.y)), Float32(w), Float32(h), Float32(-eyaw)))
        else
            push!(BILLBOARDS, (item, (Float32(i.x), gz, Float32(-i.y)), Float32(w), Float32(h)))
        end
    end
    tstamp("  [E80] .. billboard/tree loop done")

    if get(ENV,"JM_SOLIDGATE","") != ""
        q(v) = round(Int, v / 0.5)
        drawn = Set{Tuple{Int,Int}}()
        for o in OBJECTS;    p4 = o[4]; push!(drawn, (q(p4[1]), q(-p4[3]))); end
        for b in BILLBOARDS; pb = b[2]; push!(drawn, (q(pb[1]), q(-pb[3]))); end
        bad = Tuple{Float64,Float64,Float64,Symbol,Int}[]
        for (k, sd) in enumerate(SOLIDS)
            (q(sd[1]), q(sd[2])) in drawn || push!(bad, (sd[1], sd[2], sd[3], sd[4], k))
        end

        nameat(x, z) = begin
            best = ""; bd = 4.0
            for i in insts
                d = hypot(Float64(i.x) - x, Float64(i.y) - z)
                d < bd && (bd = d; best = i.name)
            end
            best
        end

        drawnames = Set{String}()
        for o in OBJECTS;    push!(drawnames, String(o[5])); end
        for i in insts
            for b in BILLBOARDS
                if abs(Float64(b[2][1]) - Float64(i.x)) < 0.5 && abs(-Float64(b[2][3]) - Float64(i.y)) < 0.5
                    push!(drawnames, lowercase(i.name)); break
                end
            end
        end

        exempt(n) = startswith(n,"armco") || startswith(n,"fence") || startswith(n,"rail") ||
                    startswith(n,"barrier") || startswith(n,"wall") || startswith(n,"bushrow") ||
                    startswith(n,"haie")
        undrawn = Tuple{String,Float64,Float64}[]; anchor = Dict{String,Int}(); exempted = Dict{String,Int}()
        for b in bad
            n = lowercase(nameat(b[1], b[2]))

            if isempty(n) && b[5] <= length(SOLIDNAMES); n = lowercase(SOLIDNAMES[b[5]]); end
            n = isempty(n) ? "(unnamed)" : n
            if exempt(n);      exempted[n] = get(exempted,n,0)+1
            elseif n in drawnames; anchor[n] = get(anchor,n,0)+1
            else push!(undrawn, (n, b[1], b[2])); end
        end
        println("== JM_SOLIDGATE ", TRACKSEL, ": ", length(SOLIDS), " solids, ", length(drawn),
                " drawn cells -> ", length(undrawn), " UNDRAWN-BUT-SOLID (",
                length(anchor), " anchor classes / ", sum(values(anchor); init=0), " objs, ",
                sum(values(exempted); init=0), " track-geometry exempt)")
        if !isempty(undrawn)
            cnt = Dict{String,Int}()
            for (n,_,_) in undrawn; cnt[n] = get(cnt,n,0)+1; end
            for (n,c) in sort(collect(cnt)); println("     INVISIBLE: ", n, " ×", c); end
        end
        if !isempty(exempted)
            println("     (exempt: rendered as TRACK GEOMETRY, not placed objects -- gate does not cover these:)")
            for (n,c) in sort(collect(exempted)); println("       ", n, " ×", c); end
        end
        if !isempty(anchor)
            println("     (anchor-mismatch classes, drawn elsewhere -- gate blind spot, NOT a defect:)")
            for (n,c) in sort(collect(anchor)); println("       ", n, " ×", c); end
        end

        if !isempty(undrawn) && get(ENV,"JM_SOLIDGATE_WHY","") != ""
            seen = Set{String}()
            println("     ---- why undrawn (name: has-mesh / dropped / billboard-name) ----")
            for (n,_,_) in undrawn
                n in seen && continue; push!(seen, n)

                orig = ""; for i in insts; lowercase(i.name) == n && (orig = i.name; break); end
                hasmesh = !isempty(orig) && get(objmesh, orig, nothing) !== nothing
                dropped = !isempty(orig) && drop(orig)
                inbb    = n in drawnames
                println("       ", rpad(n, 12), "  mesh=", hasmesh ? "YES" : "no ",
                        "  dropped=", dropped ? "YES" : "no ", "  billboard-name=", inbb ? "YES" : "no ",
                        hasmesh && !dropped ? "   <-- GENUINELY INVISIBLE" : "")
            end
        end
        get(ENV,"JM_SOLIDGATE_EXIT","") != "" && exit(isempty(undrawn) ? 0 : 1)
    end

    global OBJINSTS = [begin
        ismesh = get(objmesh,i.name,nothing) !== nothing; isbb = get(bbinfo,i.name,nothing) !== nothing; og = onground(i)

        kmesh  = ismesh && !drop(i.name) && !onroad_crowd(i) && !perp_crowd(i) &&
                 (get(ENV,"JM_OBJINSTS_LOOSE","0") != "0" || (!onroad_bldg(i) && !onroad_fp(i))) &&
                 tallenough(i.name) && og
        kbb    = isbb   && !drop(i.name) && !(standcrowd(i.name) && on_road(i.x, i.y, ROAD_HALFW+1.0)) && og && !on_road(i.x, i.y, ROAD_HALFW)
        issolid = solidR(lowercase(i.name)) > 0.0 && og && !on_road(i.x, i.y, SOLID_EXCL_HW)
        (i.name, Float32(i.x), Float32(i.y), ploz(i), kmesh ? :mesh : kbb ? :bb : :dropped, issolid)
    end for i in insts]

    global OBJ_LVERTS = lverts
    global OBJ_YMN = Dict{String,Float32}(k => v for (k, v) in ymn)
    global OBJ_YMX = Dict{String,Float32}(k => v for (k, v) in ymx)
    global OBJ_VERTS = objverts

    global OBJ_RECZ = Dict{Tuple{Float64,Float64},Float64}((round(Float64(i.x), digits=2), round(Float64(i.y), digits=2)) => Float64(i.z) for i in insts)
    global OBJ_YAW = Dict{Tuple{Float64,Float64},Float64}((round(Float64(i.x), digits=2), round(Float64(i.y), digits=2)) => OBJ_YAW_SIGN * -Float64(i.yaw) + Float64(objyawfix(i.name)) for i in insts)
    if get(ENV,"JM_FOOTPRINT","")!=""

        edge = parse(Float64, get(ENV,"JM_ASPHALT_HALFW","4.1"))
        rows = NTuple{6,Any}[]
        for i in insts
            haskey(lverts, i.name) || continue
            drop(i.name) && continue
            isempty(lverts[i.name]) && continue
            th = OBJ_YAW_SIGN * -Float64(i.yaw) + Float64(objyawfix(i.name))
            c, sn = cos(th), sin(th)
            lats = Float64[]
            for (lx, lz) in lverts[i.name]
                rx =  lx*c + lz*sn
                rz = -lx*sn + lz*c
                hr = JRTrack.hat(TRKSURF, Float64(i.x) + rx, Float64(i.y) - rz)
                hr.found && push!(lats, hr.lateral)
            end
            isempty(lats) && continue
            lo, hi = minimum(lats), maximum(lats)

            near = minimum(abs.(lats))
            pen = max(0.0, edge - near)
            hr0 = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
            pen > 0.05 && push!(rows, (i.name, round(near,digits=1), round(hi,digits=1), round(pen,digits=1),
                                       hr0.found ? round(hr0.lapdist,digits=0) : -1.0,
                                       hr0.found ? round(hr0.lateral,digits=1) : 999.0))
        end
        sort!(rows, by=x->-x[4])
        println("== E71-S8 footprints crossing the asphalt (edge ±", edge, " m) — ", length(rows), " instances ==")
        println("   name            near|lat|  lat_hi   PENETRATION  lapdist   origin_lat")
        for r in rows[1:min(end,30)]
            println("   ", rpad(r[1],16), rpad(r[2],8), rpad(r[3],8), rpad(r[4],13), rpad(r[5],10), r[6])
        end

        println("   -- local mesh extents of the flagged buildings (w x d x h, m) --")
        seenb = Set{String}()
        for r in rows
            occursin(r"^(house|bu\d|casa|clhouse|eauhotel|ho\d)", r[1]) || continue
            r[1] in seenb && continue; push!(seenb, r[1])
            haskey(lxmn,r[1]) || continue
            w = lxmx[r[1]]-lxmn[r[1]]; d = lzmx[r[1]]-lzmn[r[1]]; h = get(ymx,r[1],0f0)-get(ymn,r[1],0f0)
            npts = haskey(lverts,r[1]) ? length(lverts[r[1]]) : 0
            println("      ", rpad(r[1],12), rpad(round(w,digits=1),8), "x ", rpad(round(d,digits=1),8),
                    "x ", rpad(round(h,digits=1),7), "  pts=", npts,
                    (w>25 || d>25) ? "   <-- COMPOSITE (too big for one building)" : "")
            length(seenb) > 14 && break
        end
        bl = filter(r -> occursin(r"^(house|bu\d|casa|clhouse|eauhotel)", r[1]), rows)
        println("   -- of which BUILDINGS: ", length(bl))
        for r in bl[1:min(end,15)]
            println("      ", rpad(r[1],16), "pen=", rpad(r[4],7), "lapdist=", rpad(r[5],10), "origin_lat=", r[6])
        end
        flush(stdout)
    end
    if get(ENV,"JM_SPOTMESH","")!=""

        for tok in split(get(ENV,"JM_SPOTMESH",""), ",")
            isempty(strip(tok)) && continue
            want = parse(Float64, strip(tok))
            acc = Dict{String,Vector{Float64}}(); lats = Dict{String,Vector{Float64}}()
            for t in TRACKMESH.tris
                cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
                cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
                cz = (Float64(t.p[1][3])+Float64(t.p[2][3])+Float64(t.p[3][3]))/3
                hr = JRTrack.hat(TRKSURF, cx, cy)
                (hr.found && abs(hr.lapdist - want) < 40.0 && abs(hr.lateral) < 30.0) || continue
                lt = lowercase(t.tex); lt = lt=="" ? "<none>" : lt
                push!(get!(acc, lt, Float64[]), cz)
                push!(get!(lats, lt, Float64[]), hr.lateral)
            end
            println("== JM_SPOTMESH lapdist ", want, " ±40 m, |lat|<30 m — meshes present ==")
            println("   texture          tris   z_min    z_mean   z_max    lat range        road?")
            if isempty(acc); println("   (nothing found)"); end
            for (lt,zs) in sort(collect(acc), by=x->-length(x[2]))
                ls = lats[lt]
                println("   ", rpad(lt,16), rpad(length(zs),7),
                        rpad(round(minimum(zs),digits=1),9), rpad(round(sum(zs)/length(zs),digits=1),9),
                        rpad(round(maximum(zs),digits=1),9),
                        rpad(string(round(minimum(ls),digits=1),"..",round(maximum(ls),digits=1)),17),
                        ROAD_TEX(lt) ? "ROAD" : "")
            end
            flush(stdout)
        end
    end
    if get(ENV,"JM_OBJFIND","") != ""

        let pat = _ofre = Regex(replace(get(ENV,"JM_OBJFIND",""), "," => "|"), "i")
            hits = [i for i in OBJINSTS if occursin(pat, String(i[1]))]

            haskey(ENV, "JM_OBJFIND_FATE") && filter!(i -> String(i[5]) == ENV["JM_OBJFIND_FATE"], hits)
            println("== JM_OBJFIND /", get(ENV,"JM_OBJFIND",""), "/i -- ", length(hits), " placed instances ==")
            if isempty(hits)
                println("(none matched -- either not placed, OR the PATTERN did not match: it is a REGEX, so use pit|tower|grand. E72-S10: commas are now accepted as alternation, but a pattern that cannot match is indistinguishable from an absent object -- check a name you KNOW is placed before trusting a null.)")
            else
                for h in hits[1:min(end, parse(Int, get(ENV, "JM_OBJFIND_N", "20")))]
                    hr = JRTrack.hat(TRKSURF, Float64(h[2]), Float64(h[3]))

                    gz = groundz(h[2], h[3])
                    dz = gz > -900f0 ? round(Float64(h[4]) - Float64(gz), digits=1) : NaN
                    println("   ", rpad(h[1],14), "fate=", rpad(String(h[5]),9), "solid=", rpad(h[6],7),
                            hr.found ? string("lapdist=", rpad(round(hr.lapdist,digits=0),9),
                                              "lat=", rpad(round(hr.lateral,digits=1),8)) : rpad("off-ribbon",19),
                            "base=", rpad(round(Float64(h[4]),digits=1),8),
                            gz > -900f0 ? string("ground=", rpad(round(Float64(gz),digits=1),8),
                                                 "Δ=", dz, dz < -1.0 ? "  *** BURIED ***" : "")
                                        : "OFF-HAT")
                end
                length(hits) > 20 && println("   ... ", length(hits)-20, " more")
            end
            flush(stdout)
        end

        let tot = Dict{Symbol,Int}()
            for oi in OBJINSTS
                occursin(_ofre, oi[1]) || continue
                tot[oi[5]] = get(tot, oi[5], 0) + 1
            end
            println("   -- all matches by fate: ", join(["$(k)=$(v)" for (k,v) in sort(collect(tot), by=x->-x[2])], "  "))
        end
    end
    if get(ENV,"JM_OBJDIAG","")!=""

        let sc = [i.scale for i in insts]
            nz = count(x -> !(0.999 <= x <= 1.001), sc)
            println("== E71-S5 instance scale: n=", length(sc), "  min=", round(minimum(sc),digits=3),
                    "  max=", round(maximum(sc),digits=3), "  mean=", round(sum(sc)/length(sc),digits=3),
                    "  non-unity=", nz, " (", round(100nz/length(sc),digits=1), "%)")
            hs = [(i.name, i.scale) for i in insts if occursin(r"^(house|bu\d|casa)", i.name)]
            if !isempty(hs)
                hv = [x[2] for x in hs]
                println("   buildings only: n=", length(hs), "  min=", round(minimum(hv),digits=3),
                        "  max=", round(maximum(hv),digits=3), "  mean=", round(sum(hv)/length(hv),digits=3))
                for nm in ("house43","house29","house26","house4","bu5")
                    v = [x[2] for x in hs if x[1]==nm]
                    isempty(v) || println("     ", rpad(nm,10), "n=", rpad(length(v),5),
                                          "scale ", round(minimum(v),digits=3), " … ", round(maximum(v),digits=3))
                end
            end
            flush(stdout)
        end
        kn = unique([i.name for i in insts if get(objmesh,i.name,nothing)!==nothing && !drop(i.name) && onground(i)])
        tall = sort([(nm, get(ymx,nm,0f0)-get(ymn,nm,0f0)) for nm in kn], by=x->-x[2])
        println("== JM_OBJDIAG tallest kept geometry objects =="); for (nm,h) in tall[1:min(end,25)]; println("   ", rpad(nm,16), round(h,digits=1), " m"); end
        bbn = unique([i.name for i in insts if get(bbinfo,i.name,nothing)!==nothing && !drop(i.name) && onground(i)])
        bbt = sort([(nm, bbinfo[nm][4], bbinfo[nm][5]) for nm in bbn], by=x->-x[2])
        println("== JM_OBJDIAG billboards (name  h×w m) =="); for (nm,h,wd) in bbt; println("   ", rpad(nm,16), round(h,digits=1), " × ", round(wd,digits=1)); end

        flo = sort([(i.name, ploz(i), groundz(i.x,i.y)>-900f0) for i in insts
                    if (get(objmesh,i.name,nothing)!==nothing||get(bbinfo,i.name,nothing)!==nothing) && !drop(i.name) && onground(i)], by=x->-x[2])
        println("== JM_OBJDIAG highest-placed (trkzlo=", round(trkzlo,digits=1), " trkzhi=", round(trkzhi,digits=1), ") =="); seen=Set{String}()
        for (nm,py,onhat) in flo; nm in seen && continue; push!(seen,nm); length(seen)>22 && break; println("   ", rpad(nm,16), "y=", rpad(round(py,digits=1),7), onhat ? "on-HAT" : "OFF-HAT"); end; flush(stdout)

        onroad_objs = NTuple{4,Any}[]
        for i in insts
            (get(objmesh,i.name,nothing)===nothing || drop(i.name) || !onground(i)) && continue
            tallenough(i.name) || continue
            hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))

            roadθ = atan(-hr.perp[2], hr.perp[1])
            relyaw = rad2deg(rem2pi(Float64(i.yaw) - roadθ, RoundNearest))
            hr.found && abs(hr.lateral) < parse(Float64,get(ENV,"JM_OBJDIAG_LAT","$(ROAD_HALFW+4.0)")) && push!(onroad_objs, (i.name, round(hr.lateral,digits=1), round(hr.lapdist,digits=0), round(relyaw,digits=0)))
        end
        sort!(onroad_objs, by=x->abs(x[2]))
        println("== JM_OBJDIAG mesh objects in/near the road (|lat| < ROAD_HALFW+4 = ", round(ROAD_HALFW+4.0,digits=1), " m;  relyaw 0/±180=parallel, ±90=PERPENDICULAR) ==")
        for (nm,lat,ld,ry) in onroad_objs; println("   ", rpad(nm,16), "lat=", rpad(lat,7), " lapdist=", rpad(ld,7), " relyaw=", ry, "°"); end; flush(stdout)
    end

    if get(ENV,"JM_CROWDDIAG","")!=""
        cr = NTuple{4,Any}[]
        for i in insts
            standcrowd(i.name) || continue
            hr = JRTrack.hat(TRKSURF, Float64(i.x), Float64(i.y))
            lat = hr.found ? round(hr.lateral,digits=1) : NaN
            ry  = hr.found ? round(rad2deg(rem2pi(Float64(i.yaw)-atan(-hr.perp[2],hr.perp[1]), RoundNearest))) : NaN
            push!(cr, (lowercase(i.name), lat, hr.found ? round(hr.lapdist,digits=0) : NaN, ry))
        end
        sort!(cr, by=x->(x[1], isnan(x[2]) ? 1e9 : abs(x[2])))
        println("== JM_CROWDDIAG crowd rows (lat NaN = off the road ribbon = out on the dunes; relyaw ±90 = PERPENDICULAR) ==")
        for (nm,lat,ld,ry) in cr; println("   ", rpad(nm,12), "lat=", rpad(lat,8), " lapdist=", rpad(ld,8), " relyaw=", ry); end

        nkept = count(i -> standcrowd(i.name) && !drop(i.name), insts)
        println("   (", length(cr), " crowd rows PLACED)")
        println("   (", nkept, " crowd rows KEPT after drop())"); flush(stdout)
    end
end

end
if !_OBJ_SKIP
if CLASSICW_ON && get(ENV, "JM_CLASSICWALL_INTERIOR", "0") != "0"
    let tex = Dict{String,Vector{Any}}(), nin = Ref(0), nunc = Ref(0), nsamp = Ref(0)
        covered(x, z) = any(k -> solid_gap(x, z, k)[1] <= 0.3, eachindex(SOLIDS))
        function probe(x, z, label)
            r = classicw_locate_any(x, z); r === nothing && return
            (sec, f, lat) = r
            (lo, hi) = classicw_road_interval(sec, f)
            (lo + 0.5 < lat < hi - 0.5) || return
            nin[] += 1
            covered(x, z) && return
            nunc[] += 1
            e = get!(tex, label, Any[0, Inf, 0.0, 0]); e[1] += 1
            d = min(lat - lo, hi - lat); d < e[2] && (e[2] = d; e[4] = sec - 1)
            e[3] = max(e[3], min(lat - lo, hi - lat))
        end
        for t in TRACKMESH.tris
            p1, p2, p3 = t.p
            ux, uy, uz = p2[1]-p1[1], p2[2]-p1[2], p2[3]-p1[3]; vx, vy, vz = p3[1]-p1[1], p3[2]-p1[2], p3[3]-p1[3]
            cx, cy, cz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx; nl = sqrt(cx^2 + cy^2 + cz^2)
            (nl < 1e-9 || abs(cz)/nl > 0.3) && continue
            zlo = min(p1[3], p2[3], p3[3]); max(p1[3], p2[3], p3[3]) - zlo < CLASSICW_HMIN && continue
            P = ((p1[1], p1[2]), (p2[1], p2[2]), (p3[1], p3[2]))
            d(a, b) = hypot(a[1]-b[1], a[2]-b[2])
            (a, b) = d(P[1], P[2]) >= max(d(P[1], P[3]), d(P[2], P[3])) ? (P[1], P[2]) :
                     d(P[1], P[3]) >= d(P[2], P[3]) ? (P[1], P[3]) : (P[2], P[3])
            L = d(a, b); m = max(1, ceil(Int, L/2))
            for q in 0:m
                x = a[1] + (b[1] - a[1])*q/m; z = a[2] + (b[2] - a[2])*q/m
                h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf)
                (h[3] && zlo > h[1] + 1.2) && continue
                nsamp[] += 1
                probe(x, z, "mesh:" * lowercase(t.tex))
            end
        end
        for o in OBJECTS
            probe(Float64(o[4][1]), -Float64(o[4][3]), "obj:" * o[5])
        end
        println("  [classicwall interior] ", TRACKSEL, ": ", nsamp[], " mesh samples + ", length(OBJECTS), " objects; inside the road's free interval ",
                nin[], "; NOT covered by any solid ", nunc[])
        for (k, e) in sort(collect(tex), by = x -> -x[2][1])[1:min(end, 25)]
            println("     ", rpad(k, 26), lpad(e[1], 6), "  nearest to a wall face ", round(e[2], digits=1), " m (sec ", e[4],
                    "), deepest inside ", round(e[3], digits=1), " m")
        end
    end
    exit(0)
end

const OBJGROUND_ON = get(ENV, "JM_OBJGROUND", "1") != "0"
objground_tex(lt) = occursin("gras", lt) || occursin("asph", lt) || lt == "asfa" || startswith(lt, "aspgr")
const OBJGROUND = let tris = JRTrack.Tri[]
    if OBJGROUND_ON
        for ob in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(ob[5])), nothing); ps === nothing && continue
            M = ob[2]
            tf(i, v) = (Float64(M[1,1]*v[i] + M[1,2]*v[i+1] + M[1,3]*v[i+2] + M[1,4]), Float64(M[2,1]*v[i] + M[2,2]*v[i+1] + M[2,3]*v[i+2] + M[2,4]),
                        -Float64(M[3,1]*v[i] + M[3,2]*v[i+1] + M[3,3]*v[i+2] + M[3,4]))
            for p in ps
                objground_tex(lowercase(p.tex)) || continue
                for k in 1:33:length(p.verts)-32
                    a = tf(k, p.verts); b = tf(k + 11, p.verts); c = tf(k + 22, p.verts)
                    u = b .- a; v = c .- a
                    n = (u[2]*v[3] - u[3]*v[2], u[3]*v[1] - u[1]*v[3], u[1]*v[2] - u[2]*v[1]); nl = sqrt(sum(abs2, n))
                    (nl > 1e-9 && abs(n[2])/nl > 0.7) && push!(tris, JRTrack.Tri(a, b, c))
                end
            end
        end
    end
    isempty(tris) ? nothing : JRTrack.TriangleHAT(tris; cell = 20.0)
end
OBJGROUND === nothing || println("  [objground] AISINK-1: ", length(OBJGROUND.tris), " placed-object ground triangles (where TERRAIN has none)")
"""AISINK-1: the placed objects' ground at (x, z) [game frame] at or below `ref`, ONLY where TERRAIN has no surface there
(NaN otherwise, or with no object ground under it)."""
function objground_at(x::Real, z::Real, ref::Real = Inf)
    OBJGROUND === nothing && return NaN
    JRTrack.hat3d(TERRAIN, x, z; ref = Inf)[3] && return NaN
    r = JRTrack.hat3d(OBJGROUND, x, z; ref = ref)
    r[3] ? Float64(r[1]) : NaN
end
const OBS_ON   = CLASSICW_ON && get(ENV, "JM_OBSTACLES", "1") != "0"
const OBS_NOBACKDROP = get(ENV, "JM_OBS_BACKDROP", "0") == "0"
const OBS_SEG  = NTuple{5,Float64}[]
const OBS_KIND = Symbol[]
const OBS_NAME = String[]
const OBS_GRID = Dict{Tuple{Int,Int},Vector{Int32}}()
const OBS_C    = 4.0
const OBS_PREV = Ref((NaN, NaN))
const DRAWN_GRID = Dict{Tuple{Int,Int},Vector{NTuple{4,Float64}}}()
const OBS_STAT = Ref((hits = 0, clamps = 0, maxpen = 0.0))
"is (x, z) on Classic's asphalt strip (+-0.3 m) at its own section?"
function classicw_on_asphalt(x, z)
    r = classicw_locate_any(x, z); r === nothing && return false
    (sec, f, lat) = r; v = CLASSICWALLS.secs[sec]
    for k in 1:length(v)-1
        (v[k].typ & 0x7ff) == 1 || continue
        CLASSICWall.edge(CLASSICWALLS, sec, k, f) - 0.3 <= lat <= CLASSICWall.edge(CLASSICWALLS, sec, k + 1, f) + 0.3 && return true
    end
    false
end

"x-extent of triangle (x, z) clipped to the strip a <= z <= b, or nothing"
function tri_strip_xspan(p1, p2, p3, a, b)
    lo = Inf; hi = -Inf
    P = (p1, p2, p3)
    for k in 1:3
        (x1, z1) = P[k]; (x2, z2) = P[k % 3 + 1]
        a <= z1 <= b && (lo = min(lo, x1); hi = max(hi, x1))
        for zc in (a, b)
            (z1 - zc) * (z2 - zc) < 0 || continue
            x = x1 + (x2 - x1) * (zc - z1) / (z2 - z1); lo = min(lo, x); hi = max(hi, x)
        end
    end
    lo <= hi ? (lo, hi) : nothing
end
function road_cells!(S, tris)
    for t in tris
        ROAD_TEX(lowercase(t.tex)) || continue
        p1 = (Float64(t.p[1][1]), Float64(t.p[1][2])); p2 = (Float64(t.p[2][1]), Float64(t.p[2][2])); p3 = (Float64(t.p[3][1]), Float64(t.p[3][2]))
        for j in floor(Int, min(p1[2], p2[2], p3[2])) - 1:floor(Int, max(p1[2], p2[2], p3[2])) + 1
            r = tri_strip_xspan(p1, p2, p3, j - 1.0, j + 2.0); r === nothing && continue
            for i in floor(Int, r[1]) - 1:floor(Int, r[2]) + 1; push!(S, (i, j)); end
        end
    end
    S
end
const ROADCELLS = Set{Tuple{Int,Int}}()
CLASSICW_ON && road_cells!(ROADCELLS, TRACKMESH0.tris)
"on the DRAWN road (the road-only HAT, +-0.3 m)? Fast; falls back to Classic's asphalt strip where there is no road-only HAT."
function obs_on_road(x, z)
    ROADHAT === TERRAIN0 && return classicw_on_asphalt(x, z)
    (floor(Int, x), floor(Int, z)) in ROADCELLS || return false
    for (dx, dz) in ((0.0, 0.0), (0.3, 0.0), (-0.3, 0.0), (0.0, 0.3), (0.0, -0.3))
        JRTrack.hat3d(ROADHAT, x + dx, z + dz; ref = Inf)[3] && return true
    end
    false
end
"grid cells a segment (plus margin r) passes through -- walked along it, not its bounding box (a 185 m diagonal
forest face spans ~2000 bbox cells but ~50 along its length)"
function seg_cells(ax, az, bx, bz, r; C = OBS_C)
    out = Set{Tuple{Int,Int}}(); L = hypot(bx - ax, bz - az); n = max(1, ceil(Int, L/(C/2)))
    m = ceil(Int, r/C)
    for q in 0:n
        x = ax + (bx - ax)*q/n; z = az + (bz - az)*q/n; ci = floor(Int, x/C); cj = floor(Int, z/C)
        for i in ci-m:ci+m, j in cj-m:cj+m; push!(out, (i, j)); end
    end
    out
end
function obs_add!(ax, az, bx, bz, r, kind, name)
    push!(OBS_SEG, (ax, az, bx, bz, r)); push!(OBS_KIND, kind); push!(OBS_NAME, name); id = Int32(length(OBS_SEG))
    for c in seg_cells(ax, az, bx, bz, r); push!(get!(OBS_GRID, c, Int32[]), id); end
end
"CLASSICWALL-1: one drawn part's near-vertical faces -> obstacles. A function barrier: the sources vector is untyped, and in
the caller every vertex/matrix access dispatched dynamically (Spa's build took 33 s)."
function obs_build_part!(p, M, oname, ground, softname, bump)
        isempty(p.tex) && return
        tx = lowercase(p.tex); ground(tx) && (bump(:ground); return)

        (!isempty(oname) && OBS_NOBACKDROP && occursin(r"bk\d*$", lowercase(oname))) && (bump(:backdrop); return)

        get(ENV, "JM_OBSWHO", "") == tx && println("  [obswho] ", tx, " from ", isempty(oname) ? "the track mesh" : "object $(oname)",
            M === nothing ? "" : " at render ($(round(M[1,4], digits=1)), $(round(M[2,4], digits=1)), $(round(M[3,4], digits=1)))",
            ", ", length(p.verts) ÷ 33, " triangles")
        kind =(is_foliage_tex(p.tex) || softname(tx) || softname(oname)) ? :soft : :wall
        v = p.verts
        for k in 1:33:length(v)-32
            P = ntuple(3) do q
                o = k + 11(q-1); vx = Float64(v[o]); vy = Float64(v[o+1]); vz = Float64(v[o+2])
                if M !== nothing
                    (vx, vy, vz) = (M[1,1]*vx + M[1,2]*vy + M[1,3]*vz + M[1,4], M[2,1]*vx + M[2,2]*vy + M[2,3]*vz + M[2,4],
                                    M[3,1]*vx + M[3,2]*vy + M[3,3]*vz + M[3,4])
                end
                (vx, -vz, vy)
            end
            ux, uy, uz = P[2][1]-P[1][1], P[2][2]-P[1][2], P[2][3]-P[1][3]; wx, wy, wz = P[3][1]-P[1][1], P[3][2]-P[1][2], P[3][3]-P[1][3]
            cx, cy, cz = uy*wz - uz*wy, uz*wx - ux*wz, ux*wy - uy*wx; nl = sqrt(cx^2 + cy^2 + cz^2)
            (nl < 1e-9 || abs(cz)/nl > 0.3) && continue
            zlo = min(P[1][3], P[2][3], P[3][3]); zhi = max(P[1][3], P[2][3], P[3][3])
            zhi - zlo < CLASSICW_HMIN && (bump(:low); continue)
            d(a, b) = hypot(a[1]-b[1], a[2]-b[2])
            (a, b) = d(P[1], P[2]) >= max(d(P[1], P[3]), d(P[2], P[3])) ? (P[1], P[2]) :
                     d(P[1], P[3]) >= d(P[2], P[3]) ? (P[1], P[3]) : (P[2], P[3])
            for c in seg_cells(a[1], a[2], b[1], b[2], 0.0); push!(get!(DRAWN_GRID, c, NTuple{4,Float64}[]), (a[1], a[2], b[1], b[2])); end
            mx = (a[1] + b[1])/2; mz = (a[2] + b[2])/2
            h = JRTrack.hat3d(TERRAIN, mx, mz; ref = Inf)
            if h[3]
                zlo > h[1] + 1.2 && (bump(:overhead); continue)
                zhi < h[1] + CLASSICW_HMIN && (bump(:buried); continue)
            end

            Lf = hypot(b[1] - a[1], b[2] - a[2]); mq = max(2, ceil(Int, Lf))
            any(q -> obs_on_road(a[1] + (b[1] - a[1])*q/mq, a[2] + (b[2] - a[2])*q/mq), 0:mq) && (bump(:onroad); continue)
            obs_add!(a[1], a[2], b[1], b[2], 0.0, kind, tx)
        end
    end

if !SKIDPAD && get(ENV, "JM_GROUND_TREES", "1") != "0"
    let nb = 0, nt = 0, no = 0
        gnd(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        for i in eachindex(BILLBOARDS)
            b = BILLBOARDS[i]; g = gnd(Float64(b[2][1]), -Float64(b[2][3])); isnan(g) && continue
            lift = Float64(b[2][2]) - g
            (0.5 < lift < 30.0) || continue
            BILLBOARDS[i] = (b[1], (b[2][1], Float32(g), b[2][3]), b[3], b[4]); nb += 1
        end
        for i in eachindex(STATICTREES)
            b = STATICTREES[i]; g = gnd(Float64(b[2][1]), -Float64(b[2][3])); isnan(g) && continue
            lift = Float64(b[2][2]) - g
            (0.5 < lift < 30.0) || continue
            STATICTREES[i] = (b[1], (b[2][1], Float32(g), b[2][3]), b[3], b[4], b[5]); nt += 1
        end
        for i in eachindex(OBJECTS)
            o = OBJECTS[i]; ps = get(OBJ_CPU, lowercase(String(o[5])), nothing); ps === nothing && continue
            nf = 0; na = 0
            for p in ps; c = length(p.verts) ÷ 11; na += c; is_foliage_tex(p.tex) && (nf += c); end
            nf > 0.5*na || continue
            M = o[2]; lo = Inf
            for p in ps, k in 1:11:length(p.verts)-10
                vx = Float64(p.verts[k]); vy = Float64(p.verts[k+1]); vz = Float64(p.verts[k+2])
                wx = M[1,1]*vx + M[1,2]*vy + M[1,3]*vz + M[1,4]; wy = M[2,1]*vx + M[2,2]*vy + M[2,3]*vz + M[2,4]
                wz = M[3,1]*vx + M[3,2]*vy + M[3,3]*vz + M[3,4]
                g = gnd(wx, -wz); isnan(g) || (lo = min(lo, wy - g))
            end
            (isfinite(lo) && 0.5 < lo < 30.0) || continue
            M2 = Render.translate(0f0, -Float32(lo), 0f0) * M
            OBJECTS[i] = (o[1], M2, o[3], (o[4][1], o[4][2] - Float32(lo), o[4][3]), o[5]); no += 1
        end
        println("  FLOAT-1: set down onto the ground: ", nb, " billboards, ", nt, " forest panels, ", no, " foliage objects (JM_GROUND_TREES=0 disables)")
    end
end

const OBJ_SCENEDUP = falses(length(OBJECTS))

const CLASSICVIS_ALL = NURB && get(ENV, "JM_CLASSICVIS", "0") == "1"
const CLASSICVIS_VEIL = NURB && get(ENV, "JM_CLASSICVIS_VEIL", "1") != "0"
const CLASSICVIS = CLASSICVIS_ALL || CLASSICVIS_VEIL
isveil(nm) = (n = lowercase(nm); (startswith(n, "half") && length(n) == 6) || n in ("adena-r2", "bergw-r1", "breid-ra", "wehr-r1b") || n in VEILX)
const CLASSICVIS_SEGS, CLASSICVIS_VIS = CLASSICVIS ? CLASSICTrack.segment_visibility(ZTRK) : (Float64[], Dict{Int,BitVector}())
const _GV_ORD = filter(k -> isfinite(CLASSICVIS_SEGS[k]), sortperm(CLASSICVIS_SEGS)); const _GV_S = CLASSICVIS_SEGS[_GV_ORD]
const _GV_R = length(_GV_S) > 2 ? (_GV_S[end] + (_GV_S[end] - _GV_S[end-1])) / LAPLEN : 1.0
const _GV_R_PRE = _GV_R

const CLASSICVIS_GAP = parse(Float64, get(ENV, "JM_CLASSICVIS_GAP", "200"))
function _gv_bridge(v::BitVector)
    w = copy(v); last_s = NaN; last_i = 0
    for k in _GV_ORD
        v[k] || continue
        sk = CLASSICVIS_SEGS[k]
        if last_i > 0 && sk - last_s <= CLASSICVIS_GAP
            for j in _GV_ORD; (last_s < CLASSICVIS_SEGS[j] < sk) && (w[j] = true); end
        end
        last_s = sk; last_i = k
    end
    w
end

const CLASSICVIS_LEAD = parse(Float64, get(ENV, "JM_CLASSICVIS_LEAD", "150"))
const CLASSICVIS_LANDMASS = ("adena-r2", "breid-ra")

const CLASSICVIS_LEADX = Dict("bkbill" => parse(Float64, get(ENV, "JM_CLASSICVIS_LEAD_BKBILL", "400")))
_gv_leadfor(nm) = (n = lowercase(nm); n in CLASSICVIS_LANDMASS ? CLASSICVIS_LEAD : get(CLASSICVIS_LEADX, n, 0.0))
function _gv_lead(v::BitVector, lead)
    lead > 0 || return v
    w = copy(v); L = lead * _GV_R
    for k in _GV_ORD
        v[k] || continue
        sk = CLASSICVIS_SEGS[k]
        for j in _GV_ORD; (sk - L <= CLASSICVIS_SEGS[j] < sk) && (w[j] = true); end
    end
    w
end
end
if !_OBJ_SKIP
const OBJVIS = Union{Nothing,BitVector}[(CLASSICVIS_ALL || (CLASSICVIS_VEIL && isveil(OBJECTS[k][5]))) ?
                                         (local v = get(CLASSICVIS_VIS, OBJNODE[k], nothing); v === nothing ? nothing :
                                          _gv_bridge(_gv_lead(v, _gv_leadfor(OBJECTS[k][5])))) : nothing
                                        for k in eachindex(OBJNODE)]

if CLASSICVIS && get(ENV, "JM_CLASSICVIS_DIAG", "") != ""
    let want = Set(lowercase.(split(ENV["JM_CLASSICVIS_DIAG"], ",")))
        for k in eachindex(OBJNODE)
            lowercase(OBJECTS[k][5]) in want || continue
            local v = OBJVIS[k]; v === nothing && (println("  [classicvis] ", OBJECTS[k][5], " node ", OBJNODE[k], ": no window (drawn always)"); continue)
            ss = sort([CLASSICVIS_SEGS[j] / _GV_R_PRE for j in _GV_ORD if v[j]])
            runs = Tuple{Float64,Float64}[]
            for x in ss
                (isempty(runs) || x - runs[end][2] > 40.0) ? push!(runs, (x, x)) : (runs[end] = (runs[end][1], x))
            end
            hr = JRTrack.hat(TRKSURF, Float64(OBJECTS[k][4][1]), -Float64(OBJECTS[k][4][3]))
            println("  [classicvis] ", OBJECTS[k][5], " node ", OBJNODE[k], " at s ", hr.found ? round(Int, hr.lapdist) : -1,
                    ": camera windows ", [(round(Int, a), round(Int, b)) for (a, b) in runs])
        end
    end
end

const CLASSICCLIP = NURB ? get(ENV, "JM_CLASSICCLIP", "0") : "0"
const OBJCELLS = CLASSICCLIP != "0" ? CLASSICTrack.object_cells(ZTRK) : Dict{Int,Tuple{Float64,Vector{NTuple{4,NTuple{4,Float64}}}}}()
const OBJCLIP = Union{Nothing,Vector{NTuple{4,NTuple{4,Float32}}}}[
    (CLASSICCLIP == "all" || (CLASSICCLIP == "1" && isveil(OBJECTS[k][5]))) && haskey(OBJCELLS, OBJNODE[k]) ?
        (let (zg, L) = OBJCELLS[OBJNODE[k]], h = Float64(OBJECTS[k][4][2]) - zg
             [ntuple(j -> (Float32(c[j][1]), Float32(c[j][3]), Float32(-c[j][2]), Float32(c[j][4] - c[j][3]*h)), 4) for c in L]
         end) : nothing for k in eachindex(OBJNODE)]
CLASSICCLIP != "0" && println("  [classicclip] ", count(!isnothing, OBJCLIP), " objects drawn per Classic cell, ",
                          sum(x -> x === nothing ? 0 : length(x), OBJCLIP; init = 0), " cell draws")

function classicvis_seg(ex, ez)
    (CLASSICVIS && !isempty(_GV_S)) || return 0
    hr = JRTrack.hat(TRKSURF, Float64(ex), -Float64(ez)); hr.found || return 0
    _GV_ORD[clamp(searchsortedlast(_GV_S, hr.lapdist * _GV_R), 1, length(_GV_S))]
end
CLASSICVIS && println("  [classicvis] ", length(CLASSICVIS_SEGS), " segment trees; ", count(!isnothing, OBJVIS), " of ", length(OBJVIS),
                  " drawn objects carry a Classic window (the rest draw everywhere); dlong/lapdist ", round(_GV_R, digits = 4))
if NURB && get(ENV, "JM_OBJ_SCENEDUP", "1") != "0"
    let t0 = time(), cells = Set{Tuple{Int32,Int32}}(), q = 0.25f0, ndup = 0, nfaces = 0
        for p in TRACK
            v = p.verts
            for t in 0:33:length(v)-33
                push!(cells, (round(Int32, (v[t+1] + v[t+12] + v[t+23]) / 3 / q), round(Int32, (v[t+3] + v[t+14] + v[t+25]) / 3 / q)))
            end
        end
        for (k, o) in enumerate(OBJECTS)
            (startswith(lowercase(String(o[5])), "half") || get(ENV, "JM_OBJ_SCENEDUP_ALL", "0") != "0") || continue
            parts = get(OBJ_CPU, lowercase(String(o[5])), nothing); parts === nothing && continue
            M = o[2]; n = 0; hit = 0
            for pp in parts
                v = pp.verts
                for t in 0:33:length(v)-33
                    c = M * Float32[(v[t+1] + v[t+12] + v[t+23]) / 3, (v[t+2] + v[t+13] + v[t+24]) / 3, (v[t+3] + v[t+14] + v[t+25]) / 3, 1f0]
                    cx = round(Int32, c[1] / q); cz = round(Int32, c[3] / q); n += 1
                    found = false
                    for dx in -1:1, dz in -1:1
                        (cx + dx, cz + dz) in cells && (found = true; break)
                    end
                    found && (hit += 1)
                end
            end
            if n > 0 && hit >= 0.8 * n
                OBJ_SCENEDUP[k] = true; ndup += 1; nfaces += n
            end
        end
        println("  [scenedup] ", ndup, " of ", length(OBJECTS), " placed objects (", nfaces, " faces) duplicate the track scenery -- ",
                "not drawn twice (", round(time() - t0, digits = 1), " s; JM_OBJ_SCENEDUP=0 reverts)")
    end
end
if OBS_ON
    let t0 = time(), horiz = Dict{String,Vector{Int}}(), nskip = Dict{Symbol,Int}()
        softname(n) = occursin("hay", n) || occursin("straw", n) || occursin("hedge", n) || occursin("hecke", n) ||
                      occursin("haie", n) || occursin("shrub", n) || occursin("bush", n)

        for p in TRACK, k in 1:33:length(p.verts)-32
            v = p.verts
            ux, uy, uz = v[k+11]-v[k], v[k+12]-v[k+1], v[k+13]-v[k+2]; wx, wy, wz = v[k+22]-v[k], v[k+23]-v[k+1], v[k+24]-v[k+2]
            cx, cy, cz = uy*wz - uz*wy, uz*wx - ux*wz, ux*wy - uy*wx; nl = sqrt(cx^2 + cy^2 + cz^2)
            nl < 1e-9 && continue
            e = get!(horiz, lowercase(p.tex), [0, 0]); e[2] += 1; abs(cy)/nl > 0.7 && (e[1] += 1)
        end
        ground(t) = (e = get(horiz, t, [0, 1]); e[1] > 0.5*e[2])
        bump(k) = (nskip[k] = get(nskip, k, 0) + 1)

        srcs = Any[(p, nothing, "") for p in TRACK]
        for o in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(o[5])), nothing)
            ps === nothing && continue
            for p in ps; push!(srcs, (p, o[2], o[5])); end
        end
        for (p, M, oname) in srcs
            obs_build_part!(p, M, oname, ground, softname, bump)
        end
        nf = length(OBS_SEG)

        for b in BILLBOARDS
            (x, z) = (Float64(b[2][1]), -Float64(b[2][3]))
            obs_on_road(x, z) && (bump(:onroad); continue)
            tall = b[4] >= 3f0
            obs_add!(x, z, x, z, tall ? 0.35 : 0.3*Float64(b[3]), tall ? :wall : :soft, "billboard")
        end

        nfp = 0
        for (it, pos, w, h, yaw) in STATICTREES
            h < 1.5f0 && continue
            cx = Float64(pos[1]); cz = -Float64(pos[3]); ex = cos(-Float64(yaw))*0.5*Float64(w); ez = sin(-Float64(yaw))*0.5*Float64(w)
            (obs_on_road(cx - ex, cz - ez) || obs_on_road(cx, cz) || obs_on_road(cx + ex, cz + ez)) && (bump(:onroad); continue)
            obs_add!(cx - ex, cz - ez, cx + ex, cz + ez, 0.0, :wall, "forest")
            for c in seg_cells(cx - ex, cz - ez, cx + ex, cz + ez, 0.0); push!(get!(DRAWN_GRID, c, NTuple{4,Float64}[]), (cx - ex, cz - ez, cx + ex, cz + ez)); end
            nfp += 1
        end
        nfp > 0 && println("  CLASSICWALL-1 obstacles: + ", nfp, " forest-edge panels (hard)")
        println("  CLASSICWALL-1 obstacles: ", nf, " drawn faces + ", length(OBS_SEG) - nf, " billboards solid for the player (",
                count(==(:soft), OBS_KIND), " soft); skipped ", nskip, "  [", round(time() - t0, digits = 1), " s]")
    end
end

if OBS_ON && get(ENV, "JM_CLASSICWALL_INVIS", "0") != "0"
    let rows = Any[], nface = 0, nedge = 0, sacc = cumsum([0.0; CLASSICWALLS.seclen]), stp = 0.1
        for (si, v) in enumerate(CLASSICWALLS.secs), f in stp/2:stp:1.0
            (lo, hi, hlo, hhi) = classicw_road_interval(si, f)
            (px, pz, nx, nz) = classicw_frame(si, f)
            for (e, h, side) in ((lo, hlo, -1.0), (hi, hhi, 1.0))
                isfinite(e) || continue
                isinf(h) && (nedge += 1; continue)
                nface += 1
                x = px + nx*e; z = pz + nz*e
                vis = !isnan(classicw_hit(DRAWN_GRID, px, pz, nx, nz, e, 1.0; C = OBS_C)) ||
                      any(k -> solid_gap(x, z, k)[1] <= 1.0, eachindex(SOLIDS))
                if !vis
                    h0 = JRTrack.hat3d(TERRAIN, x - side*nx*1.0, z - side*nz*1.0; ref = Inf)
                    for dd in 0.0:0.5:2.0
                        hh = JRTrack.hat3d(TERRAIN, x + side*nx*dd, z + side*nz*dd; ref = Inf)
                        (h0[3] && hh[3] && hh[1] - h0[1] >= 0.25) && (vis = true; break)
                    end
                end
                vis || push!(rows, (round(Int, sacc[si] + f*CLASSICWALLS.seclen[si]), side < 0 ? "L" : "R", round(e, digits = 1), round(h, digits = 2), si - 1))
            end
        end
        m = stp*sum(CLASSICWALLS.seclen)/length(CLASSICWALLS.secs)
        println("  [classicwall invis] ", TRACKSEL, ": ", nface, " road-facing wall samples, ", length(rows), " INVISIBLE (",
                round(100*length(rows)/max(nface, 1), digits = 1), "%, ~", round(Int, length(rows)*m), " m); world-edge samples ", nedge)

        st = Any[]
        for r in sort(rows, by = r -> (r[2], r[1]))
            if !isempty(st) && st[end][2] == r[2] && r[1] - st[end][4] <= 3m
                st[end] = (st[end][1], st[end][2], st[end][3], r[1], st[end][5], max(st[end][6], r[4]))
            else
                push!(st, (r[1], r[2], r[3], r[1], r[5], r[4]))
            end
        end
        for q in sort(st, by = q -> -(q[4] - q[1]))[1:min(end, 15)]
            println("     s ", q[1], "-", q[4], " ", q[2], "  lat ", q[3], "  h ", q[6], " m  (sec ", q[5], ")")
        end
    end
    exit(0)
end
"closest points between segments p1-p2 and q1-q2 (2-D): (distance, point on p, point on q)"
function seg_seg(p1x, p1z, p2x, p2z, q1x, q1z, q2x, q2z)
    function pt_seg(px, pz, ax, az, bx, bz)
        tx = bx - ax; tz = bz - az; L2 = tx*tx + tz*tz
        u = L2 > 0 ? clamp(((px - ax)*tx + (pz - az)*tz)/L2, 0.0, 1.0) : 0.0
        cx = ax + u*tx; cz = az + u*tz
        (hypot(px - cx, pz - cz), cx, cz)
    end
    best = (Inf, 0.0, 0.0, 0.0, 0.0)
    (d, cx, cz) = pt_seg(p1x, p1z, q1x, q1z, q2x, q2z); d < best[1] && (best = (d, p1x, p1z, cx, cz))
    (d, cx, cz) = pt_seg(p2x, p2z, q1x, q1z, q2x, q2z); d < best[1] && (best = (d, p2x, p2z, cx, cz))
    (d, cx, cz) = pt_seg(q1x, q1z, p1x, p1z, p2x, p2z); d < best[1] && (best = (d, cx, cz, q1x, q1z))
    (d, cx, cz) = pt_seg(q2x, q2z, p1x, p1z, p2x, p2z); d < best[1] && (best = (d, cx, cz, q2x, q2z))
    best
end
"gap (<0 = penetration) and outward normal between the car capsule and obstacle i, keeping the side the car came from"
function obs_gap(i, x, z, θ, px, pz)
    (ax, az, bx, bz, r) = OBS_SEG[i]
    c1x = x + CARLF*cos(θ); c1z = z + CARLF*sin(θ); c2x = x - CARLF*cos(θ); c2z = z - CARLF*sin(θ)
    (d, cx, cz, ox, oz) = seg_seg(c1x, c1z, c2x, c2z, ax, az, bx, bz)
    tx = bx - ax; tz = bz - az; L = hypot(tx, tz)

    if L >= 2.0 && OBS_KIND[i] === :wall && !isnan(px)
        nx = -tz/L; nz = tx/L
        sp = (px - ax)*nx + (pz - az)*nz
        sp = sp >= 0 ? 1.0 : -1.0

        u1 = ((c1x - ax)*tx + (c1z - az)*tz)/L^2; u2 = ((c2x - ax)*tx + (c2z - az)*tz)/L^2
        if (0.0 <= u1 <= 1.0) || (0.0 <= u2 <= 1.0)
            s1 = sp*((c1x - ax)*nx + (c1z - az)*nz); s2 = sp*((c2x - ax)*nx + (c2z - az)*nz)
            sd = min(0.0 <= u1 <= 1.0 ? s1 : Inf, 0.0 <= u2 <= 1.0 ? s2 : Inf)
            return (sd - CARW - r, sp*nx, sp*nz)
        end
    end
    d < 1e-9 && return (-(CARW + r), cos(θ + pi), sin(θ + pi))
    ((d - CARW - r), (cx - ox)/d, (cz - oz)/d)
end
const OBS_BUF = Int32[]; const OBS_MARK = Int[]; const OBS_STAMP = Ref(0)
const OBS_EMPTY = Int32[]
"indices of obstacles near (x, z): a reused buffer, de-duplicated by stamp (no allocation, no linear search)"
function obs_near(x, z)
    length(OBS_MARK) < length(OBS_SEG) && resize!(OBS_MARK, length(OBS_SEG)) |> m -> fill!(m, 0)
    OBS_STAMP[] += 1; st = OBS_STAMP[]; empty!(OBS_BUF)
    ci = floor(Int, x/OBS_C); cj = floor(Int, z/OBS_C)
    @inbounds for i in ci-1:ci+1, j in cj-1:cj+1
        for id in get(OBS_GRID, (i, j), OBS_EMPTY)
            OBS_MARK[id] == st && continue
            OBS_MARK[id] = st; push!(OBS_BUF, id)
        end
    end
    OBS_BUF
end
function obs_contact(x, z, θ, dt)
    Fx = 0.0; Fy = 0.0; Mz = 0.0; peak = 0.0; hard = 0.0; closing = 0.0
    (px, pz) = OBS_PREV[]
    (isnan(px) || hypot(x - px, z - pz) > 60.0) && (px = NaN; pz = NaN)
    for i in obs_near(x, z)
        (g, nx, nz) = obs_gap(i, x, z, θ, px, pz)
        g < 2.0 && OBS_KIND[i] !== :soft && (CLASSICW_NEAR[] = true)
        g >= 0.0 && continue
        vn = WVX[]*nx + WVZ[]*nz
        (fx, fy, mz) = DriveRT3D.contact_force(-g, nx, nz, vn, θ; kind = OBS_KIND[i], dt = dt)
        Fx += fx; Fy += fy; Mz += mz; peak = max(peak, hypot(fx, fy))
        OBS_KIND[i] === :soft || (hard = max(hard, hypot(fx, fy)); closing = max(closing, -vn))
    end
    (Fx, Fy, Mz, peak, hard, closing)
end
"after the step: no deeper than CLASSICW_PEN into a hard obstacle, measured from the side the car came from"
function obs_clamp!(c)
    (px, pz) = OBS_PREV[]

    (isnan(px) || hypot(c.x - px, c.z - pz) > 60.0) && ((px, pz) = (c.x, c.z))
    for _ in 1:3
        worst = 0.0; wn = (0.0, 0.0)
        for i in obs_near(c.x, c.z)
            OBS_KIND[i] === :soft && continue
            (g, nx, nz) = obs_gap(i, c.x, c.z, c.θ, px, pz)
            g < 0.0 && (OBS_STAT[] = (hits = OBS_STAT[].hits + 1, clamps = OBS_STAT[].clamps, maxpen = max(OBS_STAT[].maxpen, -g)))
            -g - CLASSICW_PEN > worst && (worst = -g - CLASSICW_PEN; wn = (nx, nz))
        end
        worst <= 0.0 && break
        (nx, nz) = wn
        (wx, wz) = DriveRT3D.world_velocity(c)
        vn = wx*nx + wz*nz; vn < 0.0 && (wx -= vn*nx; wz -= vn*nz)
        cθ = cos(c.θ); sθ = sin(c.θ)
        xn = c.x + worst*nx; zn = c.z + worst*nz
        c.s_pos(c.integ, [xn, zn]); c.x = xn; c.z = zn
        c.s_vel(c.integ, [wx*cθ + wz*sθ, -wx*sθ + wz*cθ])
        OBS_STAT[] = (hits = OBS_STAT[].hits, clamps = OBS_STAT[].clamps + 1, maxpen = OBS_STAT[].maxpen)
    end
    OBS_PREV[] = (c.x, c.z)
    nothing
end

if !SKIDPAD && haskey(ENV, "JM_FINDTEX")
    let pf = lowercase(ENV["JM_FINDTEX"]), rows = Set{Tuple{String,Int,Int}}()
        for ob in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(ob[5])), nothing); ps === nothing && continue
            any(p -> startswith(lowercase(p.tex), pf), ps) || continue
            hr = JRTrack.hat(TRKSURF, Float64(ob[4][1]), -Float64(ob[4][3]))
            hr.found && push!(rows, (String(ob[5]), round(Int, hr.lapdist), round(Int, hr.lateral)))
        end
        for p in TRACK
            startswith(lowercase(p.tex), pf) || length(p.verts) < 11 || continue
            startswith(lowercase(p.tex), pf) || continue
            hr = JRTrack.hat(TRKSURF, Float64(p.verts[1]), -Float64(p.verts[3]))
            hr.found && push!(rows, ("track:" * p.tex, round(Int, hr.lapdist), round(Int, hr.lateral)))
        end
        println("  [findtex] ", pf, ": ", sort(collect(rows), by = r -> r[2]))
    end
    exit(0)
end

if !SKIDPAD && haskey(ENV, "JM_OBJPROFILE")
    if ENV["JM_OBJPROFILE"] == "all"
      let tot = 0.0, n = 0, worst = Tuple{Float64,String}[]
        gnd2(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        for ob in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(ob[5])), nothing); ps === nothing && continue
            vs = [(Float64(p.verts[k]), Float64(p.verts[k+1]), Float64(p.verts[k+2])) for p in ps for k in 1:11:length(p.verts)-10]
            xl = extrema(v[1] for v in vs); xl[2] - xl[1] < 15.0 && continue
            M = ob[2]; e = 0.0; ne = 0
            for side in (1, 2)
                lo = Inf
                for v in vs
                    (side == 1 ? v[1] < xl[1] + 0.15*(xl[2] - xl[1]) : v[1] > xl[2] - 0.15*(xl[2] - xl[1])) || continue
                    wx = M[1,1]*v[1] + M[1,2]*v[2] + M[1,3]*v[3] + M[1,4]; wy = M[2,1]*v[1] + M[2,2]*v[2] + M[2,3]*v[3] + M[2,4]
                    wz = M[3,1]*v[1] + M[3,2]*v[2] + M[3,3]*v[3] + M[3,4]
                    g = gnd2(wx, -wz); isnan(g) || (lo = min(lo, wy - g))
                end
                isfinite(lo) && (e += abs(lo); ne += 1)
            end
            ne == 2 && (tot += e/2; n += 1; push!(worst, (e/2, String(ob[5]))))
        end
        sort!(worst, by = w -> -w[1])
        println("  [objprofile] ", TRACKSEL, " JM_OBJ_PR=", OBJ_PR, ": ", n, " objects >= 15 m long; mean |end height above ground| ",
                round(tot/max(n, 1), digits = 2), " m; worst ", [(w[2], round(w[1], digits = 1)) for w in worst[1:min(end, 5)]])
        exit(0)
      end
    end
    let nm = lowercase(ENV["JM_OBJPROFILE"]), pt = parse(Float64, get(ENV, "JM_OBJPROFILE_PITCH", "0.026"))
        gnd(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        for ob in OBJECTS
            lowercase(String(ob[5])) == nm || continue
            ps = OBJ_CPU[nm]; M = ob[2]
            vs = [(Float64(p.verts[k]), Float64(p.verts[k+1]), Float64(p.verts[k+2])) for p in ps for k in 1:11:length(p.verts)-10]
            xl = extrema(v[1] for v in vs)
            for pitch in (0.0, pt, -pt)
                bins = [Float64[] for _ in 1:10]
                for v in vs

                    lx = v[1]*cos(pitch) - v[2]*sin(pitch); ly = v[1]*sin(pitch) + v[2]*cos(pitch); lz = v[3]
                    wx = M[1,1]*lx + M[1,2]*ly + M[1,3]*lz + M[1,4]; wy = M[2,1]*lx + M[2,2]*ly + M[2,3]*lz + M[2,4]; wz = M[3,1]*lx + M[3,2]*ly + M[3,3]*lz + M[3,4]
                    g = gnd(wx, -wz); isnan(g) && continue
                    b = clamp(floor(Int, 10*(v[1] - xl[1])/max(xl[2] - xl[1], 1e-6)) + 1, 1, 10); push!(bins[b], wy - g)
                end
                println("  [objprofile] ", nm, " pitch ", round(pitch, digits = 3), ": local x ", round.(xl, digits = 1), "  height above ground per bin (lo..hi): ",
                        join([isempty(b) ? "-" : "$(round(minimum(b), digits=1))..$(round(maximum(b), digits=1))" for b in bins], " | "))
            end
        end
    end
    exit(0)
end

end
if !_OBJ_SKIP
if haskey(ENV, "JM_SHIFT_DUMP")
    open(ENV["JM_SHIFT_DUMP"], "w") do io
        acc = 0.0
        for k in eachindex(ALIGNED0)
            k > 1 && (acc += hypot(ALIGNED0[k][1] - ALIGNED0[k-1][1], ALIGNED0[k][2] - ALIGNED0[k-1][2]))
            println(io, acc, " ", k <= length(RECENTRE_SHIFT) ? RECENTRE_SHIFT[k] : NaN)
        end
    end
end
if CLASSICW_ON && haskey(ENV, "JM_RIBBON_VS_CLASSIC")
    let rows = Tuple{Float64,Float64}[], sacc = 0.0, P = RIBBON0.pos
        for k in eachindex(P)
            k > 1 && (sacc += hypot(P[k][1] - P[k-1][1], P[k][3] - P[k-1][3]))
            r = classicw_locate_any(P[k][1], P[k][3]); r === nothing && continue
            (sec, f, lat) = r; v = CLASSICWALLS.secs[sec]; a0 = NaN; b0 = NaN
            for q in 1:length(v)-1
                (v[q].typ & 0x7ff) == 1 || continue
                ea = CLASSICWall.edge(CLASSICWALLS, sec, q, f); eb = CLASSICWall.edge(CLASSICWALLS, sec, q + 1, f)
                (isnan(a0) || eb - ea > b0 - a0) && (a0 = ea; b0 = eb)
            end
            isnan(a0) || push!(rows, (sacc, lat - (a0 + b0)/2))
        end
        d = [r[2] for r in rows]; n = length(d); sv = [r[1] for r in rows]
        wig = [d[k] - sum(d[max(1, k-8):min(n, k+8)])/length(max(1, k-8):min(n, k+8)) for k in 1:n]
        println("  [ribbon-vs-classic] ", TRACKSEL, ": ", n, " nodes; offset p50 ", round(sort(abs.(d))[n÷2], digits = 2), " m; wiggle p90 ",
                round(sort(abs.(wig))[9n÷10], digits = 2), " max ", round(maximum(abs.(wig)), digits = 2), " m")
        worst = sortperm(abs.(wig), rev = true)
        picked = Int[]
        for k in worst; all(j -> abs(sv[j] - sv[k]) > 60, picked) && push!(picked, k); length(picked) >= 10 && break; end
        for k in sort(picked); println("     s ", round(Int, sv[k]), "  offset ", round(d[k], digits = 2), "  wiggle ", round(wig[k], digits = 2)); end
    end
    exit(0)
end

if !SKIDPAD && get(ENV, "JM_FLOAT", "0") != "0"
    let rows = Any[]
        gnd(x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); h[3] ? Float64(h[1]) : NaN)
        near(x, z) = (hr = JRTrack.hat(TRKSURF, x, z); hr.found && abs(hr.lateral) < 20.0 ? (hr.lapdist, hr.lateral) : nothing)
        for b in BILLBOARDS
            x = Float64(b[2][1]); z = -Float64(b[2][3]); nr = near(x, z); nr === nothing && continue
            g = gnd(x, z); isnan(g) && continue
            lift = Float64(b[2][2]) - g
            lift > 0.5 && push!(rows, ("billboard h$(round(Int, b[4]))", nr[1], nr[2], lift))
        end
        for o in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(o[5])), nothing); ps === nothing && continue
            M = o[2]; lo = Inf; px = Float64(o[4][1]); pz = -Float64(o[4][3]); nr = near(px, pz); nr === nothing && continue
            for p in ps, k in 1:11:length(p.verts)-10
                vx = Float64(p.verts[k]); vy = Float64(p.verts[k+1]); vz = Float64(p.verts[k+2])
                wx = M[1,1]*vx + M[1,2]*vy + M[1,3]*vz + M[1,4]; wy = M[2,1]*vx + M[2,2]*vy + M[2,3]*vz + M[2,4]
                wz = M[3,1]*vx + M[3,2]*vy + M[3,3]*vz + M[3,4]
                g = gnd(wx, -wz); isnan(g) || (lo = min(lo, wy - g))
            end
            isfinite(lo) && lo > 0.5 && push!(rows, (String(o[5]), nr[1], nr[2], lo))
        end
        println("  [float] ", TRACKSEL, ": ", length(rows), " billboards/objects within 20 m of the road stand > 0.5 m above the ground")
        for r in sort(rows, by = r -> -r[4])[1:min(end, 25)]
            println("     ", rpad(r[1], 16), " s ", round(Int, r[2]), "  lat ", round(r[3], digits = 1), "  base ", round(r[4], digits = 2), " m above ground")
        end
    end
    exit(0)
end

if OBS_ON && haskey(ENV, "JM_OBSAT")
    let v = parse.(Float64, split(ENV["JM_OBSAT"], ",")), x = v[1], z = v[2], θ = length(v) > 2 ? deg2rad(v[3]) : 0.0
        println("  [obsat] obstacles near (", x, ", ", z, "):")
        for i in obs_near(x, z)
            (ax, az, bx, bz, r) = OBS_SEG[i]
            d = hypot((ax + bx)/2 - x, (az + bz)/2 - z); d > 6.0 + hypot(bx - ax, bz - az)/2 && continue
            g = obs_gap(i, x, z, θ, x, z)[1]
            println("     ", rpad(OBS_NAME[i], 14), " ", OBS_KIND[i], "  seg (", round(ax, digits=1), ",", round(az, digits=1), ")-(",
                    round(bx, digits=1), ",", round(bz, digits=1), ") r ", round(r, digits=2), "  gap ", round(g, digits=2))
        end
    end
    exit(0)
end

if OBS_ON && get(ENV, "JM_CORRIDOR", "0") != "0"
    let hits = Dict{String,Vector{Float64}}(), wallhits = Float64[]
        A = ALIGNED; nA = length(A); sacc = 0.0
        for k in 1:nA, ln in (-1.5, 0.0, 1.5)
            j = k == nA ? 1 : k + 1; tx = A[j][1] - A[k][1]; tz = A[j][2] - A[k][2]; tl = max(hypot(tx, tz), 1e-9)
            ln == -1.5 && k > 1 && (sacc += hypot(A[k][1] - A[k-1][1], A[k][2] - A[k-1][2]))
            sv = sacc; θ = atan(tz, tx); x = A[k][1] - tz/tl*ln; z = A[k][2] + tx/tl*ln
            for i in obs_near(x, z)
                OBS_KIND[i] === :soft && continue
                g = obs_gap(i, x, z, θ, x, z)[1]
                g < -0.05 && push!(get!(hits, OBS_NAME[i], Float64[]), sv)
            end
            CLASSICW_PREV[] = NaN; CLASSICW_LAST[] = (NaN, NaN); CLASSICW_HINT[] = 0
            (_, _, _, glo, ghi, _, _, _, _, ge) = classicwall_gaps(x, z, θ)
            min(glo, ghi) < -0.05 && push!(wallhits, sv)
            if ln == 0.0 && haskey(ENV, "JM_CORRIDOR_DIAG")
                (d0, d1) = parse.(Float64, split(ENV["JM_CORRIDOR_DIAG"], ":"))
                if d0 <= sv <= d1 && round(Int, sv) % parse(Int, get(ENV, "JM_CORRIDOR_STEP", "15")) == 0
                    (sc, fc, lc) = classicw_locate(x, z; hint = 0)
                    v = CLASSICWALLS.secs[sc]
                    strips = join(["$(round(CLASSICWall.edge(CLASSICWALLS, sc, k, fc), digits=1))$(v[k].wall && v[k].height >= CLASSICW_HMIN ? "W" : (v[k].typ & 0x7ff) == 1 ? "a" : "")" for k in 1:length(v)], " ")
                    println("   [cdiag] s ", round(Int, sv), " sec ", sc - 1, " f ", round(fc, digits = 2), " our centre at Classic lat ", round(lc, digits = 2),
                            "  strips: ", strips)
                end
            end
        end
        function grp(v)
            out = Tuple{Int,Int}[]
            for k in sort(unique(round.(Int, v)))
                (!isempty(out) && k - out[end][2] <= 5) ? (out[end] = (out[end][1], k)) : push!(out, (k, k))
            end
            out
        end
        println("  [corridor] ", TRACKSEL, ": road blocked by hard obstacles at ", sum(length, values(hits); init = 0), " lane-stations; by Classic walls at ", length(wallhits))
        for (nm, v) in sort(collect(hits), by = x -> -length(x[2]))[1:min(end, 20)]
            println("     ", rpad(nm, 14), length(v), "  s ", join(["$(a)-$(b)" for (a, b) in grp(v)][1:min(end, 8)], ", "))
        end
        isempty(wallhits) || println("     Classic walls   ", length(wallhits), "  s ", join(["$(a)-$(b)" for (a, b) in grp(wallhits)][1:min(end, 12)], ", "))
    end
    exit(0)
end

println(length(OBJECTS), " trackside objects + ", length(BILLBOARDS), " billboards + ", length(STATICTREES), " forest panels + ", length(SOLIDS), " solid (collidable)"); flush(stdout)
if get(ENV,"JM_HAT_COUNT","0") != "0"
    st = JRTrack.hat_stats()
    print("[hat] ", st.calls, " calls in the placement block")
    get(ENV,"JM_HAT_TIME","0") != "0" && print("  total ", round(st.total_s,digits=2), " s  ",
                                               round(st.per_call_us,digits=2), " us/call")
    println(); flush(stdout)
    JRTrack.HAT_COUNT_ON[] = false
end
tstamp("  [E80] trackside objects/billboards/trees DONE")
end

if get(ENV,"JM_TEXDIAG","")!=""

    th = Dict{String,Vector{Float32}}()
    for t in TRACKMESH.tris, vi in 1:3; push!(get!(th,t.tex,Float32[]), Float32(t.p[vi][3])); end
    rows = [(tex, length(zs)÷3, minimum(zs), sum(zs)/length(zs), maximum(zs)) for (tex,zs) in th]
    sort!(rows, by=r->-r[4])
    println("== JM_TEXDIAG track-mesh textures by mean height (tex  ntri  zmin  zmean  zmax) ==")
    for (tex,n,zmn,zmu,zmx) in rows; println("   ", rpad(tex=="" ? "<none>" : tex,14), rpad(n,7), " zmin=", rpad(round(zmn,digits=1),8), "zmean=", rpad(round(zmu,digits=1),8), "zmax=", round(zmx,digits=1)); end
    flush(stdout)
end

const VISORP = filter(p -> p.tex == "__visor__", CARP)
const visorItems = Render.build_classic(VISORP, CLASSICTEX)

const VISOR_ALPHA = parse(Float32, get(ENV, "JM_VISOR_ALPHA", "0.08"))
const TEAM_WIN_ALPHA = parse(Float32, get(ENV, "JM_TEAM_WIN_ALPHA", "0.3"))
const carItems = Render.build_classic(filter(p -> p.tex != "__visor__", CARP), CLASSICTEX; tag="carp")

pipeItems  = Render.build_classic(PIPEP, CLASSICTEX; tag="extra")
axleItems  = Render.build_classic(AXLEP, CLASSICTEX; tag="extra")
fsuspSynItems = Render.build_classic(FSUSP_SYN, CLASSICTEX; tag="extra")
carItemsIn = isempty(CARPIN) ? Render.Item[] : Render.build_classic(filter(p -> p.tex != "__visor__", CARPIN), CLASSICTEX)

const TACH_ITEMS = TACH_ON ? map(((tip, tail, w0, w1, col, lift),) -> begin
        vao, n = Render.upload(_needle_verts(0.0, tip, tail, w0, w1, col; lift = lift)); Render.Item(vao, n, GLuint(0), col)
    end, ((0.80f0, 0.30f0, 0.040f0, 0.018f0, (0.85f0, 0.06f0, 0.05f0), 0.002f0),
          (0.86f0, 0.32f0, 0.040f0, 0.016f0, (0.96f0, 0.96f0, 0.94f0), 0.0035f0))) : Render.Item[]

const DIAL_ITEMS = map(DIALS) do d
    vao, n = Render.upload(_dial_needle_verts(d.f, 0.78f0, 0.22f0, 0.045f0, 0.018f0, (0.96f0, 0.96f0, 0.94f0); lift = 0.003f0))
    Render.Item(vao, n, GLuint(0), (0.96f0, 0.96f0, 0.94f0))
end

gaugeItems = (get(ENV,"JM_GAUGE", get(ENV,"JM_COCKPIT_DRESS","1") != "0" ? "0" : "1") == "0") ?
    Render.Item[] : Render.build_classic(GAUGEP, CLASSICTEX)
windItems  = Render.build_classic(WINDP, CLASSICTEX)
mirrorItems = Render.build_classic(MIRRORP, CLASSICTEX)

tstamp("  [E80] mirrors + wheels begin")
const MIRROR_RTT = get(ENV,"JM_MIRROR_RTT","1") != "0"
const MIRW, MIRH = 384, 192

const MIRROR_UNFLIP = get(ENV, "JM_MIRROR_UNFLIP", "1") != "0"
(mirfbo, mirtex) = MIRROR_RTT ? Render.make_mirror_fbo(MIRW, MIRH) : (GLuint(0), GLuint(0))

const MIRROR_EVERY = parse(Int, get(ENV,"JM_MIRROR_EVERY","1"))

const MIRROR_HALVES = parse(Int, get(ENV,"JM_MIRROR_HALVES","2"))

const MIRROR_ADAPT    = get(ENV,"JM_MIRROR_ADAPT","1") != "0"

const MIRROR_ADAPT_MS = parse(Float64, get(ENV,"JM_MIRROR_ADAPT_MS","18.0"))
const MIRROR_RESUME_MS = parse(Float64, get(ENV,"JM_MIRROR_RESUME_MS","14.0"))
const MIRROR_STARVED  = Ref(false)
const MIRROR_ADAPT_N  = parse(Int,     get(ENV,"JM_MIRROR_ADAPT_N","3"))
const MIRROR_EMA      = Ref(0.0)
const MIRROR_T0       = Ref(0.0)
const MIRROR_SKIPPED  = Ref(0)
const MIRROR_GLASS_FRAC = 0.88f0
function mirror_glass_quads(parts, tex)
    items = Render.Item[]
    for side in (-1, 1)
        xmn=ymn=zmn=Inf32; xmx=ymx=zmx=-Inf32
        for p in parts
            v = p.verts
            for i in 1:11:length(v)-10
                sign(v[i+2]) == side || continue
                xmn=min(xmn,v[i]); xmx=max(xmx,v[i]); ymn=min(ymn,v[i+1]); ymx=max(ymx,v[i+1]); zmn=min(zmn,v[i+2]); zmx=max(zmx,v[i+2])
            end
        end
        isfinite(xmn) || continue
        c  = Float32[(xmn+xmx)/2, (ymn+ymx)/2, (zmn+zmx)/2]
        e  = Float32[xmx-xmn, ymx-ymn, zmx-zmn]
        na = argmin(e)
        ua, va = na==1 ? (3,2) : na==2 ? (3,1) : (1,2)
        eyerig = Float32[0.46, 0.40, 0]
        ns = Float32(sign(eyerig[na] - c[na])); ns == 0 && (ns = -1f0)
        hu = e[ua]/2 * MIRROR_GLASS_FRAC; hv = e[va]/2 * MIRROR_GLASS_FRAC

        if ua == 3 && get(ENV, "JM_MIRROR_GLASS_BBOX", "1") == "0"
            R = e[va]/2
            outer = side < 0 ? zmn : zmx
            c[3] = outer - side*R
            hu = R * MIRROR_GLASS_FRAC; hv = hu
        end
        nrm = Float32[0,0,0]; nrm[na] = ns

        u0, u1 = side < 0 ? (0f0, 0.5f0) : (0.5f0, 1f0)
        MIRROR_UNFLIP && ((u0, u1) = (u1, u0))
        get(ENV, "JM_MIRROR_DIAG", "0") != "0" && println("  [mirglass] side ", side, " c ", round.(c, digits=3), " e ", round.(e, digits=3),
                                                         " normal axis ", na, " ns ", ns, " half ", round(hu, digits=3), "x", round(hv, digits=3))
        q = Float32[]
        corner(mu, mv) = begin
            p = copy(c); p[na] += ns*(e[na]/2 + 0.004f0)
            p[ua] += (mu-0.5f0)*2hu*ns; p[va] += (mv-0.5f0)*2hv
            append!(q, p); append!(q, nrm); push!(q, mu, mv, 0f0, u0+(u1-u0)*mu, mv)
        end
        for (mu,mv) in ((0f0,0f0),(1f0,0f0),(1f0,1f0), (0f0,0f0),(1f0,1f0),(0f0,1f0)); corner(mu,mv); end
        vao, n = Render.upload(q)
        push!(items, Render.Item(vao, n, tex, (1f0,1f0,1f0)))
    end
    items
end

if get(ENV, "JM_MIRROR_DIAG", "0") != "0"
    for p in MIRRORP
        local v = p.verts; xs = v[1:11:end]; ys = v[2:11:end]; zs = v[3:11:end]
        println("  [mirpart] ", rpad(p.tex, 9), " n ", length(xs), " x ", round.(extrema(xs), digits=3), " y ", round.(extrema(ys), digits=3), " z ", round.(extrema(zs), digits=3))
    end
end

function mirror_glass_ring(parts, tex)
    items = Render.Item[]
    rings = filter(p -> lowercase(p.tex) == "lrm", parts)
    for side in (-1, 1)
        u0, u1 = side < 0 ? (0f0, 0.5f0) : (0.5f0, 1f0)
        MIRROR_UNFLIP && ((u0, u1) = (u1, u0))
        q = Float32[]
        for p in rings
            v = p.verts; nv = length(v) ÷ 11
            for t in 0:3:nv-3
                base = (t*11 + 1, (t+1)*11 + 1, (t+2)*11 + 1)
                sign(v[base[1]+2] + v[base[2]+2] + v[base[3]+2]) == side || continue
                for b in base
                    uu, vv = v[b+9], v[b+10]
                    mu = 0.5f0 + (uu - 0.5f0)/MIRROR_GLASS_FRAC; mv = 0.5f0 + (vv - 0.5f0)/MIRROR_GLASS_FRAC
                    append!(q, v[b:b+5]); push!(q, mu, mv, 0f0, u0 + (u1 - u0)*clamp(uu, 0f0, 1f0), 1f0 - vv)
                end
            end
        end
        isempty(q) && continue
        vao, n = Render.upload(q)
        push!(items, Render.Item(vao, n, tex, (1f0,1f0,1f0)))
    end
    items
end
mirGlassItems = !MIRROR_RTT ? Render.Item[] :
    (get(ENV, "JM_MIRROR_GLASS_RING", "1") != "0" && any(p -> lowercase(p.tex) == "lrm", MIRRORP)) ?
        mirror_glass_ring(MIRRORP, mirtex) : mirror_glass_quads(filter(p -> lowercase(p.tex) in ("mirror", "lrm"), MIRRORP), mirtex)

const SUSP_INBOARD = parse(Float32, get(ENV,"JM_SUSP_INBOARD","0.0"))
function susp_inboard(parts)
    SUSP_INBOARD == 0f0 && return parts
    out = Render.TrackPart[]
    for pp in parts
        v = copy(pp.verts)
        for k in 1:11:length(v)-2
            z = v[k+2]

            v[k+2] = z > 0 ? max(z - SUSP_INBOARD, 0f0) : min(z + SUSP_INBOARD, 0f0)
        end
        push!(out, Render.TrackPart(v, pp.tex, pp.col))
    end
    out
end
if get(ENV,"JM_SUSP_INBOARD_DIAG","") != ""
    for (nm, ps) in (("FSUSPP", FSUSPP), ("RSUSPP_A", RSUSPP_A), ("RSUSPP_B", RSUSPP_B))
        for (tag, q) in ((" before", ps), (" after ", susp_inboard(ps)))
            lo = Inf32; hi = -Inf32
            for pp in q, k in 1:11:length(pp.verts)-2
                z = abs(pp.verts[k+2]); lo = min(lo, z); hi = max(hi, z)
            end
            println("   [inboard] ", rpad(nm,10), tag, "  |z| ", round(lo,digits=3), " … ", round(hi,digits=3))
        end
    end

    flush(stdout); exit(0)
end

fsuspItems  = Render.build_classic(susp_inboard(FSUSPP), CLASSICTEX; tag="fsusp")
driverItems = Render.build_classic(DRIVERP, CLASSICTEX)
helmItems   = Render.build_classic(HELMP, CLASSICTEX)

const TYRE_ALB = parse(Float32, get(ENV,"JM_TYRE_ALB","0.17"))

const WHEEL_WRAP = get(ENV, "JM_WHEEL_WRAP", "1") != "0" ? Dict(
    "lotwlf" => "llftire0", "lotwrf" => "lrftire0",
    "lotwlr" => "llrtire0", "lotwrr" => "lrrtire0") : Dict{String,String}()
const WHEEL_DRESS = Dict(
    "lotwlf" => ("l1out", "l1in", "loftex1"), "lotwrf" => ("l1in", "l1out", "loftex1"),
    "lotwlr" => ("l1out", "l1in", "lortex1"), "lotwrr" => ("l1in", "l1out", "lortex1"))
load_wheel(nm) = haskey(WHEEL_WRAP, nm) ?
    Render.build_classic(Render.extract_classic_car(joinpath(LOTDIR, WHEEL_WRAP[nm]*".3do");
                    exclude=("ltraymap","lshad"), tint=(TYRE_ALB,TYRE_ALB,TYRE_ALB+0.02f0)), CLASSICTEX) :
    Render.build_classic(Render.extract_classic_car(joinpath(LOTDIR,nm*".3do");
                    exclude=("ltraymap","lshad"), tint=(TYRE_ALB,TYRE_ALB,TYRE_ALB+0.02f0),
                    wheel_dress=get(WHEEL_DRESS, nm, nothing)), CLASSICTEX)
const WHEELITEMS = Dict(nm => load_wheel(nm) for nm in ("lotwlf","lotwrf","lotwlr","lotwrr"))
tstamp("  [E80] wheel models loaded")
swItems = Render.build_classic(SWPARTS, CLASSICTEX)
shiftItems = isempty(SHIFTP) ? Render.Item[] : Render.build_classic(SHIFTP, CLASSICTEX)
const SHIFT_TW = Ref(0.0)

const SUITDIR = let d = get(ENV, "JM_SUIT", joinpath(dirname(LOTDIR), "murasama", "JackSeller 16-bit Driving Suits", "lotus"))
    (d != "0" && isfile(joinpath(d, "lotarms.mip"))) ? d : ""
end
const SUITTEX = SUITDIR == "" ? CLASSICTEX : Render.classic_texture_index(SUITDIR)
println("  driver suit: ", SUITDIR == "" ? "base lotus textures" : SUITDIR)
handItems = Render.build_classic(HANDP, CLASSICTEX)
armItems  = Render.build_classic(ARMP, SUITTEX)
rsusp2Items = Render.build_classic(RSUSPP2, CLASSICTEX; tag="rsusp2")
rsuspItemsA = Render.build_classic(susp_inboard(RSUSPP_A), CLASSICTEX; tag="rsuspa")
rsuspItemsB = Render.build_classic(susp_inboard(RSUSPP_B), CLASSICTEX; tag="rsuspb")

rsfix(side) = SuspPose.rsfix(side, Render.translate, Render.rotx, Render.scalexyz)

const RS_SWAP = get(ENV,"JM_RS_SWAP","0") != "0"
const RSFIX_A = rsfix(RS_SWAP ? -1 : 1)
const RSFIX_B = rsfix(RS_SWAP ? 1 : -1)

function _bake_clip(parts, M; zmax=Float32(WTRACK_R), ymin=-0.12f0)
    out = Render.TrackPart[]
    R = [M[r,c] for r in 1:3, c in 1:3]
    for p in parts
        v = p.verts; n = length(v) ÷ 33
        keep = Float32[]
        for t in 0:n-1
            base = t*33
            ok = true
            tv = zeros(Float32, 33)
            for k in 0:2
                o = base + k*11
                x=v[o+1]; y=v[o+2]; z=v[o+3]
                tx = Float32(M[1,1]*x+M[1,2]*y+M[1,3]*z+M[1,4])
                ty = Float32(M[2,1]*x+M[2,2]*y+M[2,3]*z+M[2,4])
                tz = Float32(M[3,1]*x+M[3,2]*y+M[3,3]*z+M[3,4])
                (abs(tz) > zmax || ty < ymin) && (ok = false)
                nx=v[o+4]; ny=v[o+5]; nz=v[o+6]
                tv[k*11+1]=tx; tv[k*11+2]=ty; tv[k*11+3]=tz
                tv[k*11+4]=Float32(R[1,1]*nx+R[1,2]*ny+R[1,3]*nz)
                tv[k*11+5]=Float32(R[2,1]*nx+R[2,2]*ny+R[2,3]*nz)
                tv[k*11+6]=Float32(R[3,1]*nx+R[3,2]*ny+R[3,3]*nz)
                tv[k*11+7]=v[o+7]; tv[k*11+8]=v[o+8]
                tv[k*11+9]=v[o+9]; tv[k*11+10]=v[o+10]; tv[k*11+11]=v[o+11]
            end
            ok && append!(keep, tv)
        end
        isempty(keep) || push!(out, Render.TrackPart(keep, p.tex, p.col))
    end
    out
end
const RS_BAKECLIP = get(ENV,"JM_RS_BAKECLIP","1") != "0"
if RS_BAKECLIP
    global rsuspItemsA = Render.build_classic(_bake_clip(susp_inboard(RSUSPP_A), RSFIX_A), CLASSICTEX; tag="rsuspa")
    global rsuspItemsB = Render.build_classic(_bake_clip(susp_inboard(RSUSPP_B), RSFIX_B), CLASSICTEX; tag="rsuspb")
end

const RS_SPEC = parse(Float32, get(ENV,"JM_RS_SPEC","0.25"))

const RSUSP_ON = get(ENV,"JM_RSUSP","0") != "0"

if get(ENV,"JM_FSUSP_DIAG","") != ""

    for me in (0.5f0, 1.0f0, 1.5f0, 2.0f0, Inf32)
        pp = Render.extract_classic_car(LOT3DO; only=("lsusp1",), maxlat=1.3f0, maxedge=me)
        n = sum(length(x.verts) ÷ 11 for x in pp; init=0) ÷ 3
        println("   FSUSPP maxedge=", me, "  -> ", length(pp), " parts, ", n, " tris")
    end
    for me in (1.0f0, Inf32)
        pp = Render.extract_classic_car(LOT3DO; only=("frontlot",), maxlat=1.3f0, maxedge=me)
        n = sum(length(x.verts) ÷ 11 for x in pp; init=0) ÷ 3
        println("   frontlot maxedge=", me, " -> ", length(pp), " parts, ", n, " tris")
    end
    flush(stdout)
end
if get(ENV,"JM_RSUSP_WORLD","") != ""

    bb(parts) = begin
        lo = [Inf32,Inf32,Inf32]; hi = [-Inf32,-Inf32,-Inf32]
        for pp in parts, i in 1:11:length(pp.verts)-10
            for k in 1:3
                v = pp.verts[i+k-1]
                v < lo[k] && (lo[k] = v); v > hi[k] && (hi[k] = v)
            end
        end
        (lo, hi)
    end
    xf(parts, M) = [begin
        v = copy(pp.verts)
        for i in 1:11:length(v)-10
            x,y,z = v[i],v[i+1],v[i+2]
            v[i]   = M[1,1]*x + M[1,2]*y + M[1,3]*z + M[1,4]
            v[i+1] = M[2,1]*x + M[2,2]*y + M[2,3]*z + M[2,4]
            v[i+2] = M[3,1]*x + M[3,2]*y + M[3,3]*z + M[3,4]
        end
        (verts=v, tex=pp.tex)
    end for pp in parts]
    for (nm, parts, M) in (("RSUSPP_A raw", RSUSPP_A, nothing),
                           ("RSUSPP_A × RSFIX_A", RSUSPP_A, RSFIX_A),
                           ("RSUSPP_B raw", RSUSPP_B, nothing),
                           ("RSUSPP_B × RSFIX_B", RSUSPP_B, RSFIX_B))
        pl = M === nothing ? parts : xf(parts, M)
        lo, hi = bb(pl)
        println("   [rsusp] ", rpad(nm,22),
                " x ", rpad(string(round(lo[1],digits=2),"…",round(hi[1],digits=2)),16),
                " y ", rpad(string(round(lo[2],digits=2),"…",round(hi[2],digits=2)),16),
                " z ", rpad(string(round(lo[3],digits=2),"…",round(hi[3],digits=2)),16),
                " span ", round(max(hi[1]-lo[1], hi[2]-lo[2], hi[3]-lo[3]), digits=2), " m")
    end
    flush(stdout)
end
if get(ENV,"JM_CARGROUPS","") != ""

    for g in (6600, 3560, 27288, 39792)
        parts = Render.extract_classic_car(LOT3DO; include_groups=(g,))
        tot = 0
        rows = []
        for pp in parts
            n = length(pp.verts) ÷ 11
            zmn=Inf32; zmx=-Inf32; xmn=Inf32; xmx=-Inf32; ymn=Inf32; ymx=-Inf32
            for i in 1:11:length(pp.verts)-10
                x=pp.verts[i]; y=pp.verts[i+1]; z=pp.verts[i+2]
                zmn=min(zmn,z); zmx=max(zmx,z); xmn=min(xmn,x); xmx=max(xmx,x); ymn=min(ymn,y); ymx=max(ymx,y)
            end
            push!(rows, (pp.tex, n, zmn, zmx, xmx-xmn, ymx-ymn, zmx-zmn)); tot += n
        end
        println("== JM_CARGROUPS group ", g, ": ", length(rows), " parts, ", tot, " tris ==")

        for (tex,n,zmn,zmx,dx,dy,dz) in sort(rows, by=r->-max(r[5],r[6],r[7]))
            flat = (min(dx,dy,dz) < 0.06) ? "  FLAT" : "  3-D"
            mark = (tex in ("lshok","lsusp5","lsusp7","lsusp1","lbrdisc","frontlot") ? "   <<< MISSING FROM CARP" : "") *
                   string("   size ", round(dx,digits=2), "×", round(dy,digits=2), "×", round(dz,digits=2), flat)
            println("     ", rpad(tex,14), rpad(n,7), rpad(string(round(zmn,digits=2),"…",round(zmx,digits=2)),18), mark)
        end
    end
    flush(stdout)
end
if get(ENV,"JM_CARPARTS","") != ""
    allp = Render.extract_classic_car(LOT3DO)
    kept = Set{String}()
    for pp in CARP; push!(kept, pp.tex); end
    excl = Set{String}((_HAND_EXC...,_LOTBLACK_EXC...,_EXTRA_EXC...,_GARBAGE_EXC...,
                        DRIVER_TEX...,MIRROR_TEX...,Render.STEER_TEX...))
    println("== JM_CARPARTS: Lotus 49 .3do contents (unfiltered) ==")
    println("   texture       tris   longitudinal x     lateral z          height y           in CARP?  excluded by name?")
    rows = []
    for pp in allp
        local v = pp.verts; n = length(v) ÷ 11
        zmn=Inf32; zmx=-Inf32; ymn=Inf32; ymx=-Inf32; xmn=Inf32; xmx=-Inf32
        for i in 1:11:length(v)-10
            x=v[i]; y=v[i+1]; z=v[i+2]
            xmn=min(xmn,x); xmx=max(xmx,x)
            ymn=min(ymn,y); ymx=max(ymx,y); zmn=min(zmn,z); zmx=max(zmx,z)
        end
        push!(rows, (pp.tex, n, zmn, zmx, ymn, ymx, xmn, xmx))
    end
    for (tex,n,zmn,zmx,ymn,ymx,xmn,xmx) in sort(rows, by=r->-r[2])
        println("   ", rpad(tex,14), rpad(n,7),
                rpad(string(round(xmn,digits=2),"…",round(xmx,digits=2)),19),
                rpad(string(round(zmn,digits=2),"…",round(zmx,digits=2)),19),
                rpad(string(round(ymn,digits=2),"…",round(ymx,digits=2)),19),
                rpad(tex in kept ? "yes" : "NO", 10),
                tex in excl ? "EXCLUDED" : "")
    end
    flush(stdout)
end

if get(ENV,"JM_WHEELFIT","") != ""
    ba = Render.parts_bbox(RSUSPP_A); bb = Render.parts_bbox(RSUSPP_B)
    bc = Render.parts_bbox(CARP)
    outb = max(abs(ba.zmin), abs(ba.zmax), abs(bb.zmin), abs(bb.zmax))
    println("== JM_WHEELFIT: drawn wheel placement vs the mesh the suspension is authored at ==")
    println("   rear-susp mesh lateral  A: ", round(ba.zmin,digits=3), " … ", round(ba.zmax,digits=3),
            "   B: ", round(bb.zmin,digits=3), " … ", round(bb.zmax,digits=3))
    println("   outermost suspension point   |z| = ", round(outb,digits=3), " m")
    println("   car body (CARP) lateral       ", round(bc.zmin,digits=3), " … ", round(bc.zmax,digits=3))
    println("   wheels drawn at half-track    front ", WTRACK_F, "   rear ", WTRACK_R, " m")
    println("   REAR gap wheel-centre − suspension outermost = ",
            round(WTRACK_R - outb, digits=3), " m")
    println("   (a positive gap is empty space the linkage cannot span — the visual 'detached wheels')")
    flush(stdout)
end

function split_fists(parts, tex)
    L=Float32[]; R=Float32[]
    for p in parts
        v = p.verts
        for t in 1:33:length(v)-32
            zm = (v[t+2]+v[t+13]+v[t+24])/3
            append!(zm > 0 ? L : R, @view v[t:t+32])
        end
    end
    [Render.Item(Render.upload(a)..., tex, (1f0,1f0,1f0)) for a in (L, R)]
end
handLR = isempty(handItems) ? Render.Item[] : split_fists(HANDP, handItems[1].tex)
const HAND_GRIP = deg2rad(parse(Float32, get(ENV,"JM_HAND_GRIP","30")))

const SHIFT_HAND = get(ENV, "JM_SHIFT_HAND", "0") != "0"
const SHIFT_HAND_HOLD = parse(Float64, get(ENV, "JM_SHIFT_HAND_HOLD", "0.45"))
const SHIFT_HAND_UP = Float32[0, parse(Float32, get(ENV, "JM_SHIFT_HAND_UP", "0.03")), 0]
const SHIFT_KNOB = let ps = Vector{Float32}[]
    for p in SHIFTP, i in 0:11:length(p.verts)-11; push!(ps, Float32[p.verts[i+1], p.verts[i+2], p.verts[i+3]]); end
    isempty(ps) ? nothing : ps[argmax([q[2] for q in ps])]
end

const FIST_C = let c = [zeros(Float32, 3), zeros(Float32, 3)], n = [0, 0]
    for p in HANDP, t in 1:33:length(p.verts)-32
        v = p.verts; k = (v[t+2]+v[t+13]+v[t+24])/3 > 0 ? 1 : 2
        for j in (t, t+11, t+22); c[k] .+= Float32[v[j], v[j+1], v[j+2]]; n[k] += 1; end
    end
    [n[k] > 0 ? c[k] ./ n[k] : c[k] for k in 1:2]
end

function fistrot(k)
    sp = get(ENV, "JM_FIST_ROT", "0"); (isempty(sp) || sp == "0") && return Render.ident()
    f = split(sp, "_"); ax = f[1] == "x" ? Float32[1, 0, 0] : f[1] == "y" ? Float32[0, 1, 0] : Float32[0, 0, 1]
    θ = deg2rad(parse(Float32, f[2])) * (length(f) > 2 && f[3] == "mirror" && k == 2 ? -1f0 : 1f0)
    Render.translate(FIST_C[k]) * Render.rotaxis(ax, Float32(θ)) * Render.translate(-FIST_C[k])
end
const SHIFT_HAND_K = SHIFT_PIVOT === nothing ? 2 : (SHIFT_PIVOT[3] > 0 ? 1 : 2)
const SHIFT_LASTM = Ref(Render.ident())
const SHIFT_HAND_ST = Ref((-99, -1e9))
"Weight 0..1 of the lever-side glove's way to the knob, from the gear sequence (wall clock, as the lever's own motion)."
const SHIFT_HAND_FORCE = haskey(ENV, "JM_SHIFT_HAND_FORCE") ? parse(Float64, ENV["JM_SHIFT_HAND_FORCE"]) : nothing
function shift_hand_u(g)
    SHIFT_HAND_FORCE === nothing || return SHIFT_HAND_FORCE
    lg, t0 = SHIFT_HAND_ST[]; tn = time()
    el(t) = tn - t
    u(t) = (e = el(t); x = e < 0.1 ? e/0.1 : e < 0.1 + SHIFT_HAND_HOLD ? 1.0 : 1.0 - (e - 0.1 - SHIFT_HAND_HOLD)/0.1; x = clamp(x, 0.0, 1.0); x*x*(3 - 2x))
    if lg == -99
        SHIFT_HAND_ST[] = (g, -1e9)
    elseif g != lg
        u0 = u(t0)
        SHIFT_HAND_ST[] = (g, u0 > 0 ? tn - 0.1*min(u0, 1.0) : tn)
    end
    u(SHIFT_HAND_ST[][2])
end
const ARMS_ON_WHEEL = get(ENV, "JM_ARMS_ON_WHEEL", "0") != "0"
const HAND_CULL = get(ENV, "JM_HAND_CULL", "1") != "0"
gripmat(sgn) = Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(sgn*HAND_GRIP)) * Render.translate(-SWCENTER)

const ARMS = get(ENV,"JM_ARMS","1") != "0"

const ARMFIX = begin
    px, py = parse(Float32, get(ENV,"JM_ARM_X0","0.74")), parse(Float32, get(ENV,"JM_ARM_Y0","0.26"))
    sy     = parse(Float32, get(ENV,"JM_ARM_SY","0.62"))
    sz     = parse(Float32, get(ENV,"JM_ARM_SZ","1.0"))
    dx, dy = parse(Float32, get(ENV,"JM_ARM_DX","-0.02")), parse(Float32, get(ENV,"JM_ARM_DY","-0.06"))
    Render.translate(Float32[dx, dy, 0]) *
        Render.translate(Float32[px, py, 0]) * Render.scalexyz(-1f0, -sy, sz) * Render.translate(Float32[-px, -py, 0])
end

const ARMS2 = get(ENV, "JM_ARMS2", "1") != "0"

const SHOULDER = (parse(Float32, get(ENV, "JM_SHOULDER_X", "0.05")), parse(Float32, get(ENV, "JM_SHOULDER_Y", "-0.20")),
                  parse(Float32, get(ENV, "JM_SHOULDER_Z", "0.40")))
const ARM_THICK = parse(Float32, get(ENV, "JM_ARM_THICK", "0.85"))
const ARM_STRIPE = get(ENV, "JM_ARM_STRIPE", "1") != "0"
"""Per side (+1 = mesh z > 0): the sleeve's (wrist, far-end) points -- the ends of its principal axis, the wrist the lower."""
function sleeve_ends(parts, side)
    P = NTuple{3,Float64}[]
    for p in parts, i in 1:11:length(p.verts)-10
        v = p.verts; sign(v[i+2]) == side && push!(P, (v[i], v[i+1], v[i+2]))
    end
    isempty(P) && return nothing
    c = (sum(p[1] for p in P), sum(p[2] for p in P), sum(p[3] for p in P)) ./ length(P)
    C = zeros(3, 3)
    for p in P, a in 1:3, b in 1:3; C[a, b] += (p[a] - c[a]) * (p[b] - c[b]); end
    F = eigen(Symmetric(C))
    ax = F.vectors[:, 3]; nf = F.vectors[:, 1]

    if ARM_STRIPE
        sn = zeros(3)
        for p in parts, i in 1:33:length(p.verts)-32
            v = p.verts; sign(v[i+2]) == side || continue
            um = (v[i+9] + v[i+20] + v[i+31]) / 3; 0.44 <= um <= 0.56 || continue
            e1 = (v[i+11]-v[i], v[i+12]-v[i+1], v[i+13]-v[i+2]); e2 = (v[i+22]-v[i], v[i+23]-v[i+1], v[i+24]-v[i+2])
            n = [e1[2]*e2[3]-e1[3]*e2[2], e1[3]*e2[1]-e1[1]*e2[3], e1[1]*e2[2]-e1[2]*e2[1]]
            fc = ((v[i]+v[i+11]+v[i+22])/3, (v[i+1]+v[i+12]+v[i+23])/3, (v[i+2]+v[i+13]+v[i+24])/3)
            sum(n .* (collect(fc) .- collect(c))) < 0 && (n .*= -1)
            sn .+= n
        end
        sum(sn .^ 2) > 1e-12 && (nf = sn ./ sqrt(sum(sn .^ 2)))
    end
    pr = [sum((p .- c) .* ax) for p in P]
    a = c .+ Tuple(ax .* minimum(pr)); b = c .+ Tuple(ax .* maximum(pr))
    a[2] <= b[2] ? (a, b, nf) : (b, a, nf)
end
const SLEEVE_ENDS = ARMS2 ? (sleeve_ends(ARMP, 1), sleeve_ends(ARMP, -1)) : (nothing, nothing)
armLR = (ARMS2 && !isempty(armItems)) ? split_fists(ARMP, armItems[1].tex) : Render.Item[]
"""The raw-frame matrix that puts the sleeve's wrist W on H and its far end E on the shoulder S: a rotation of the axis
E->W onto S->H, stretched along it to length |S-H| and scaled ARM_THICK across."""
const EYE_RAW = (parse(Float64, get(ENV, "JM_EYE_X", "0.095")), parse(Float64, get(ENV, "JM_EYE_Y", "0.40")), 0.0)
const ARM_ROLL = get(ENV, "JM_ARM_ROLL", "0") != "0"
const SLEEVE_LASTR = Ref(Matrix(1.0I, 3, 3))
const SLEEVE_HOLD = get(ENV, "JM_ARM_HOLDROLL", "1") != "0"
const ARM_ROLLWHEEL = get(ENV, "JM_ARM_ROLLWHEEL", "0") != "0"
const ARM_LOCK_DEG = Tuple(parse.(Float64, split(get(ENV, "JM_ARM_LOCK_DEG", "50,130"), ",")))

const ARM_LOCK_S = let v = get(ENV, "JM_ARM_LOCK_S", "")
    isempty(v) ? nothing : Tuple(parse.(Float64, split(v, ",")))
end
const ARM_INTO_STRAIGHT = 0.12
const ARM_INTO = 0.06
const SLEEVE_NREF = Any[nothing, nothing]

const CUFF = get(ENV, "JM_CUFF", "1") != "0"

const CUFF_LOCK_DEG = Tuple(parse.(Float64, split(get(ENV, "JM_CUFF_LOCK_DEG", "15,60"), ",")))
const CUFF_INTO = (parse(Float64, get(ENV, "JM_CUFF_INTO_STRAIGHT", "0.14")), parse(Float64, get(ENV, "JM_CUFF_INTO_LOCK", "0.04")))
const CUFF_RAW = let out = Any[nothing, nothing]
    sw0 = Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(SW_ROT)) * Render.translate(-SWCENTER[1], -SWCENTER[2], -SWCENTER[3])
    for (k, sgn) in ((1, 1), (2, -1))
        Q = Vector{Float32}[]
        for p in HANDP, t in 1:33:length(p.verts)-32
            v = p.verts; ((v[t+2] + v[t+13] + v[t+24])/3 > 0 ? 1 : 2) == k || continue
            for j in (t, t+11, t+22); push!(Q, (fistrot(k) * Float32[v[j], v[j+1], v[j+2], 1f0])[1:3]); end
        end
        isempty(Q) && continue
        Sraw = (inv(sw0 * gripmat(sgn)) * Float32[SHOULDER[1], SHOULDER[2], sgn*SHOULDER[3], 1f0])[1:3]
        c = sum(Q) ./ length(Q); d = Sraw .- c; d ./= sqrt(sum(d .^ 2))
        pr = [sum((q .- c) .* d) for q in Q]; lo, hi = extrema(pr)
        sel = [Q[i] for i in eachindex(Q) if pr[i] >= hi - 0.25*(hi - lo)]
        out[k] = Tuple(Float64.(sum(sel) ./ length(sel)))
    end
    println("  [cuff] sleeves end at the gloves' cuffs: ", out[1] === nothing ? "-" : round.(out[1], digits = 3), " / ",
            out[2] === nothing ? "-" : round.(out[2], digits = 3), CUFF ? "" : "  (off: JM_CUFF=0)")
    Tuple(out)
end
function sleeve_mat(W, E, H, S, nf; target = nothing, thick = ARM_THICK)
    a = collect(W .- E); la = sqrt(sum(a .^ 2)); a ./= la
    b = collect(H .- S); lb = sqrt(sum(b .^ 2)); b ./= lb
    v = [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1]]; cth = sum(a .* b); sv = sqrt(sum(v .^ 2))
    K = [0 -v[3] v[2]; v[3] 0 -v[1]; -v[2] v[1] 0]
    R = sv < 1e-9 ? (cth > 0 ? Matrix(1.0I, 3, 3) : -Matrix(1.0I, 3, 3)) : Matrix(1.0I, 3, 3) + K + K*K*((1 - cth)/sv^2)
    Sx = thick .* Matrix(1.0I, 3, 3) .+ (lb/la - thick) .* (a * a')

    n1 = R * collect(nf); n1 .-= b .* sum(n1 .* b)

    d = target !== nothing ? collect(target) : collect(EYE_RAW) .- (collect(H) .+ collect(S)) ./ 2; d .-= b .* sum(d .* b)
    if (ARM_ROLL || ARM_STRIPE || target !== nothing) && sqrt(sum(n1 .^ 2)) > 1e-6 && sqrt(sum(d .^ 2)) > 1e-6
        n1 ./= sqrt(sum(n1 .^ 2)); d ./= sqrt(sum(d .^ 2))
        φ = atan(sum(b .* [n1[2]*d[3]-n1[3]*d[2], n1[3]*d[1]-n1[1]*d[3], n1[1]*d[2]-n1[2]*d[1]]), sum(n1 .* d))
        Kb = [0 -b[3] b[2]; b[3] 0 -b[1]; -b[2] b[1] 0]
        R = (Matrix(1.0I, 3, 3) + sin(φ)*Kb + (1 - cos(φ))*Kb*Kb) * R
    end
    L = R * Sx; t = collect(H) .- L * collect(W)
    SLEEVE_LASTR[] = R
    Render.M4(L[1,1], L[2,1], L[3,1], 0, L[1,2], L[2,2], L[3,2], 0, L[1,3], L[2,3], L[3,3], 0, t[1], t[2], t[3], 1)
end
println(count(it->it.tex!=0, trackItems), "/", length(trackItems), " track + ",
        count(it->it.tex!=0, carItems), "/", length(carItems), " Lotus parts textured")
tstamp("  [E80] track items counted")

const AIBASE = joinpath(classic_root(), "cars", "cars67")
const BODY_FLOOR = Render.parts_bbox(CARP).ymin + BODY_OFF[2]
aiwheels(lf,rf,lr,rr) = Tuple{Float32,Float32,Bool,Float32,String}[
    ( 1.05f0, 0.62f0, true,  0.31f0, lf), ( 1.05f0, -0.62f0, true,  0.31f0, rf),
    (-1.15f0, 0.66f0, false, 0.34f0, lr), (-1.15f0, -0.66f0, false, 0.34f0, rr)]

const AI_FIELD = get(ENV, "JM_AI_FIELD", "all67")

const ALL67 = [
    ("Lotus",   "lotus",    "lotus.3do",    ("lotulf","loturf","lotulr","loturr"), "CLA5A",  "J Clark"),
    ("Eagle",   "eagle",    "eagle.3do",    ("eotwlf","eotwrf","eotwlr","eotwrr"), "gur9a",  "D Gurney"),
    ("Ferrari", "ferrari",  "ferrari.3do",  ("f222lf","f222rf","f444lr","f444rr"), "amo3a",  "C Amon"),
    ("Brabham", "brabham",  "brabham.3do",  ("brablf","brabrf","brablr","brabrr"), "bra1a",  "J Brabham"),
    ("Cooper",  "coventry", "coventry.3do", ("cooplf","cooprf","cooplr","cooprr"), "bon11a", "J Bonnier"),
    ("Honda",   "murasama", "murasama.3do", ("hondlf","hondrf","hondlr","hondrr"), "sur7a",  "J Surtees"),
    ("BRM",     "brm",      "brm.3do",      ("brm2lf","brm2rf","brm4lr","brm4rr"), "ste8a",  "J Stewart"),
]

const EXT67X = get(ENV, "JM_EXT67X", "1") != "0" && get(ENV, "JM_CARSET67X", "1") != "0"
const _EXT67X = Dict("Honda" => ("c28", "c28.3do", ("hondlf", "hondrf", "hondlr", "hondrr")),
                     "BRM"   => ("c17", "c17.3do", ("115wlf", "115wrf", "115wlr", "115wrr")),

                     "Ferrari" => ("c26", "c26.3do", ("f222lf", "f222rf", "f444lr", "f444rr")),
                     "Cooper"  => ("c20", "c20.3do", ("t81slf", "t81srf", "t81slr", "t81srr")),
                     "Brabham" => ("c09", "c09.3do", ("brablf", "brabrf", "brablr", "brabrr")),
                     "Eagle"   => ("c06", "c06.3do", ("t1gwlf", "t1gwrf", "t1gwlr", "t1gwrr")))
const EXT67X_ON = Set{String}()

function classic_folder_view(src)
    dats = filter(f -> endswith(lowercase(f), ".dat"), readdir(src; join = true))
    key = string(hash((abspath(src), [(basename(f), filesize(f), mtime(f)) for f in dats])); base = 16)
    v = joinpath(get(ENV, "JM_TEXCACHE_DIR", joinpath(homedir(), ".cache", "juliamotor")), "classicview", basename(src) * "-" * key)
    isfile(joinpath(v, ".done")) && return v
    mkpath(v)
    loose = Set{String}()
    for f in readdir(src)
        endswith(f, "~") && continue
        push!(loose, lowercase(f)); l = joinpath(v, f)
        islink(l) || isfile(l) || symlink(joinpath(abspath(src), f), l)
    end
    n = 0
    for f in dats, (nm, bytes) in Render.CLASSICDat.parse_dat(f)
        (endswith(lowercase(nm), ".3do") && !(lowercase(nm) in loose)) || continue
        write(joinpath(v, nm), bytes); n += 1
    end
    touch(joinpath(v, ".done"))
    println("  [classicview] ", basename(src), ": ", length(loose), " loose files + ", n, " meshes from its .dat -> ", v)
    v
end
for (i, sp) in enumerate(ALL67)
    (EXT67X && haskey(_EXT67X, sp[1])) || continue
    c, body, w = _EXT67X[sp[1]]; src = normpath(joinpath(AIBASE, "..", "1967-X", c))
    isdir(src) && any(f -> lowercase(f) == body, lowercase.(readdir(src))) || continue
    d = try classic_folder_view(src) catch e; println("  [ext67x] ", sp[1], ": no folder view (", e, ") -- cars67"); continue end
    ALL67[i] = (sp[1], d, body, w, sp[5], sp[6]); push!(EXT67X_ON, sp[1])
    println("  [ext67x] ", sp[1], ": exterior from 1967-X/", c)
end
const PLAYER_CAR = let c = lowercase(strip(get(ENV, "JM_PLAYER_CAR", "lotus")))

    rf = get(ENV, "JM_REPLAY", "")
    if !isempty(rf) && isfile(rf)
        m = try; match(r"\((\w+)\)", first(deserialize(rf).names)); catch; nothing; end
        c = m === nothing ? "lotus" : lowercase(m[1])
    end
    i = findfirst(sp -> lowercase(sp[1]) == c, ALL67)
    i === nothing && println("  [carchoice] JM_PLAYER_CAR=", c, " is not a '67 car (", join(lowercase.(first.(ALL67)), ", "), ") -- the Lotus")
    i === nothing ? "Lotus" : ALL67[i][1]
end
const PLAYER_SPEC = ALL67[findfirst(sp -> sp[1] == PLAYER_CAR, ALL67)]
const PLAYER_CM = Ref{Union{Nothing,Render.CLASSICCarModel}}(nothing)

car_type(name::AbstractString) = (i = findfirst(sp -> sp[1] == name, ALL67); i === nothing ? 0xff : UInt8(i - 1))
const NET_BY_TYPE = Dict{String,Render.CLASSICCarModel}()
const NETCARS = Ref(Dict{Int,UInt8}())
NETLINK === nothing || (NETLINK.car = car_type(PLAYER_CAR); NETLINK.carname = PLAYER_CAR)

const NET_CARSWITCH = Ref("")
const NET_SWITCHED = Ref(false)
const NET_NOTE = Ref(("", -Inf))
"""A guest on the host's car takes the first free one: its name, or "" if the host's car is unknown or not ours."""
function net_free_car()
    hc = NETLINK.remote[0x01].car
    (hc != 0xff && hc == NETLINK.car && Int(hc) < length(ALL67)) || return ""
    i = findfirst(k -> k - 1 != Int(hc), eachindex(ALL67))
    NETLINK.car = UInt8(i - 1); NETLINK.carname = ALL67[i][1]; NET_CARSWITCH[] = ALL67[i][1]
    NET_NOTE[] = (string("The host drives the ", ALL67[Int(hc) + 1][1], ": you drive the ", ALL67[i][1], "."), time())
    println("  netplay: ", NET_NOTE[][1]); flush(stdout)
    ALL67[i][1]
end

function net_early_car(mine)
    (NETMODE == "join" && NETLINK !== nothing) || return mine
    tmax = parse(Float64, get(ENV, "JM_NET_CARWAIT", "20")); t0 = time(); tk = -Inf
    while time() - t0 < tmax && !haskey(NETLINK.remote, 0x01) && isempty(NETLINK.refused)
        time() - tk >= 0.5 && (tk = time(); NetPlay.send_pose!(NETLINK, NET_ID, NetPlay.KNOCK_TICK, 0, 0, 0, 0; car = NETLINK.car))
        NetPlay.poll!(NETLINK); sleep(0.05)
    end
    if !haskey(NETLINK.remote, 0x01)
        println("  netplay: no word from the host within ", tmax, " s -- building the ", mine, "; a later car switch moves the body only")
        return mine
    end
    NET_SWITCHED[] = true
    c = net_free_car()
    isempty(c) ? mine : c
end
const PLAYER_MINE = Ref(false)
PLAYER_CAR == "Lotus" || println("  [carchoice] driving the ", PLAYER_CAR, " (", PLAYER_SPEC[6], "'s chassis): its body, wheels and cockpit;",
                                 " its own physics (CLASSIC67CARS-1: the Lotus 49 model with its mass, dimensions and engine)")
const AISPECS = AI_FIELD == "all67" ? filter(sp -> sp[1] != PLAYER_CAR, ALL67) : AI_FIELD == "old" ? [
    ("Ferrari", "ferrari",  "ferrari.3do",  ("f222lf","f222rf","f444lr","f444rr"), "amo3a",  "C Amon"),
    ("Brabham", "brabham",  "brabham.3do",  ("brablf","brabrf","brablr","brabrr"), "bra1a",  "J Brabham"),
    ("BRM",     "brm",      "brm.3do",      ("brm2lf","brm2rf","brm4lr","brm4rr"), "ste8a",  "J Stewart"),
    ("Eagle",   "eagle",    "eagle.3do",    ("eotwlf","eotwrf","eotwlr","eotwrr"), "gur9a",  "D Gurney"),
    ("Cooper",  "coventry", "coventry.3do", ("cooplf","cooprf","cooplr","cooprr"), "bon11a", "J Bonnier"),
    ("Honda",   "murasama", "murasama.3do", ("hondlf","hondrf","hondlr","hondrr"), "sur7a",  "J Surtees"),
] : [
    ("Lotus",   "lotus",    "lotus.3do",    ("lotulf","loturf","lotulr","loturr"), "CLA5A",  "J Clark"),
    ("Lotus",   "lotus",    "lotus.3do",    ("lotulf","loturf","lotulr","loturr"), "hil6a",  "G Hill"),
    ("Cooper",  "coventry", "coventry.3do", ("cooplf","cooprf","cooplr","cooprr"), "bon11a", "J Bonnier"),
    ("Ferrari", "ferrari",  "ferrari.3do",  ("f222lf","f222rf","f444lr","f444rr"), "amo3a",  "C Amon"),
    ("Brabham", "brabham",  "brabham.3do",  ("brablf","brabrf","brablr","brabrr"), "bra1a",  "J Brabham"),
    ("Eagle",   "eagle",    "eagle.3do",    ("eotwlf","eotwrf","eotwlr","eotwrr"), "gur9a",  "D Gurney"),
]
const AIDRIVERS = [sp[6] for sp in AISPECS]

const AICAR_PHYS_BY = Dict(
    "Ferrari" => (390.0, 560.0),
    "Brabham" => (330.0, 525.0),
    "BRM"     => (400.0, 615.0),
    "Eagle"   => (395.0, 555.0),
    "Cooper"  => (360.0, 600.0),
    "Lotus"   => (400.0, 500.0),
    "Honda"   => (420.0, 610.0),
)

const AICAR_PHYS_SRC = get(ENV, "JM_AI_PHYS_OLD", "0") != "0" ? AICAR_PHYS_BY :
    Dict(k => (400.0*c.kP, 501.0 + c.dm) for (k, c) in DriveRT3D.CAR_PHYS)
const AICAR_PHYS = [AICAR_PHYS_SRC[sp[1]] for sp in AISPECS]
const AI_SLEEVE_EXC = get(ENV,"JM_AI_SLEEVES","0") != "0" ? () :
    ("arml","armr","arms","ferarms","fersho","fehand","braarms","brasho","brhand",
     "frarm","frarm2","frarm3","drvarms","bmhand","sho128","eagarm","eagarms","eahand",
     "coparms","copsho","cohand","lotarms","lotsho","lohand","honarms","honsho","handho",
     "300arms","300sho","300hnd","115arms","115sho","115hnd",
     "312arms","312sho","312hnd","b24arms","b24sho","b24hnd","t1garm2","t1ghn3","t81arms","t81sho","t81hn3")

const DRVHELM = get(ENV, "JM_DRVHELM", "1") != "0"
const AI_SLEEVES_ON = get(ENV, "JM_AI_ARMS", "1") != "0" && get(ENV, "JM_AI_SLEEVES", "0") == "0"
const AI_ARM_NAMES = ("arml", "armr", "arms", "ferarms", "braarms", "frarm", "frarm2", "frarm3", "drvarms", "eagarm", "eagarms",
                      "coparms", "lotarms", "honarms", "300arms", "115arms", "312arms", "b24arms", "t1garm2", "t81arms")
const AI_HAND_NAMES = ("fehand", "brhand", "bmhand", "eahand", "cohand", "lohand", "handho", "300hnd", "115hnd", "312hnd", "b24hnd", "t1ghn3", "t81hn3")
const AI_SHOULDER = (parse(Float64, get(ENV, "JM_AI_SHOULDER_DX", "-0.56")), parse(Float64, get(ENV, "JM_AI_SHOULDER_DY", "0.10")),
                     parse(Float64, get(ENV, "JM_AI_SHOULDER_Z", "0.26")), parse(Float64, get(ENV, "JM_AI_ARM_THICK", "1.2")))

function ai_helmet_skin(dir, wrapname)
    get(ENV, "JM_AI_HELMET", "1") == "0" && return nothing
    nm = lowercase(first(wrapname, 3)) * "helm"
    any(f -> lowercase(f) == nm * ".mip", readdir(joinpath(AIBASE, dir))) || return nothing
    (nm, -0.6, 0.6, 0.28, 0.2)
end
AICARMODELS = Render.CLASSICCarModel[]
tstamp("  [E80] AI car models begin")

_ncars = max(N_AI, NETMODE == "" ? 0 : 1, NETMODE == "join" ? N_AI_REQ : 0)

_loadspecs = PLAYER_CAR == "Lotus" ? AISPECS[1:min(_ncars, end)] : vcat(AISPECS[1:min(_ncars, end)], [PLAYER_SPEC])

NETMODE == "" || (_loadspecs = vcat(_loadspecs, [sp for sp in ALL67 if !any(q -> q[1] == sp[1], _loadspecs)]))

const AI_PARKED_SUSP_GROUPS = Dict("eagle" => Set([29108, 39200]), "brabham" => Set([32916, 48284]), "cooper" => Set([30048, 41624]))

AI_PARKED_SUSP_GROUPS["lotus"] = Set([27288, 39792])

AI_PARKED_SUSP_GROUPS["honda"] = Set([26832, 35840])

"Honda" in EXT67X_ON && (AI_PARKED_SUSP_GROUPS["honda"] = Set([24724, 33472]))

"Ferrari" in EXT67X_ON && (AI_PARKED_SUSP_GROUPS["ferrari"] = Set([25992, 35864]))
"Brabham" in EXT67X_ON && (AI_PARKED_SUSP_GROUPS["brabham"] = Set([34672, 50040]))

const AI_REAR_MODE = get(ENV, "JM_AI_PARKED_SUSP", "0") != "0" ? :raw : (get(ENV, "JM_AI_REAR_HIDE", "0") != "0" ? :hide : :pose)
if !SKIDPAD && !isempty(_loadspecs)
    for (nm, dir, body, w, wrapname, _drvname) in _loadspecs
        print("  loading AI car: $nm … "); flush(stdout)

        _rg = get(AI_PARKED_SUSP_GROUPS, lowercase(nm), Set{Int}())

        _amode = (AI_REAR_MODE === :pose && get(ENV, "JM_AI_REAR_POSE", "0") == "0") ? :hide : AI_REAR_MODE
        Render.CLASSIC3DO.HIDE_GROUPS[] = _amode === :hide ? _rg : Set{Int}()

        _wrap = get(ENV, "JM_AI_BODY_WRAP", "1") == "0" ? nothing :
                let fs = readdir(joinpath(AIBASE, dir)), want = lowercase(wrapname) * ".3do"
                    i = findfirst(f -> lowercase(f) == want, fs)
                    i === nothing && (i = findfirst(f -> occursin(r"^[a-z]+d\.3do$"i, f), fs))
                    i === nothing ? nothing : fs[i]
                end
        _wrap !== nothing && print("[", _wrap, "] ")
        _bare = body; _wrap !== nothing && (body = _wrap)

        _hel = nothing

        _hmesh = isfile(joinpath(AIBASE, dir, "helmeg.3do")) ? joinpath(AIBASE, dir, "helmeg.3do") : joinpath(LOTDIR, "helmeg.3do")
        if DRVHELM && _wrap !== nothing && occursin("classicview", dir) && isfile(_hmesh)
            _wp = Render.extract_classic_car(joinpath(AIBASE, dir, _wrap); maxlat = 0.95f0)
            _hn = unique([p.tex for p in _wp if occursin(r"^[a-z]+\d*helm$"i, p.tex)])
            _fn = unique([p.tex for p in _wp if occursin(r"^[a-z]+\d*face$"i, p.tex)])
            if !isempty(_hn)
                _cx = maximum(p.verts[i] for p in _wp if p.tex == _hn[1] for i in 1:11:length(p.verts)-10)
                _cy = maximum(p.verts[i+1] for p in _wp if p.tex == _hn[1] for i in 1:11:length(p.verts)-10)
                _hel = (helm = _hn[1], excl = (_hn..., _fn...), off = (_cx - 0.17f0, _cy - 0.14f0))
            end
        end

        push!(AICARMODELS, Render.load_classic_car(nm, joinpath(AIBASE,dir), body, aiwheels(w...); hub3do = _bare,
                              skip_ptypes = (get(ENV, "JM_AI_KEEP_81D", "0") != "0" || dir == "lotus") ? nothing : Set{UInt32}([0x81D]),
                              exclude=("ltraymap","lshad",AI_SLEEVE_EXC...,(haskey(ENV, "JM_AI_EXC_FLAT") ? ("",) : ())...,(_hel === nothing ? () : _hel.excl)...,
                                       split(get(ENV, "JM_AI_EXC_TEX", ""), ","; keepempty = false)...),
                              maxlat=parse(Float32, get(ENV,"JM_AI_MAXLAT", string(CARP_MAXLAT))),
                              body_floor=BODY_FLOOR,
                              rear_groups=(_amode === :pose ? collect(_rg) : Int[]),
                              rear_lat=parse(Float32, get(ENV, "JM_AI_REAR_LAT", "0.66")),
                              sleeves=AI_SLEEVES_ON ? (AI_ARM_NAMES, AI_HAND_NAMES, AI_SHOULDER) : nothing,
                              skin=ai_helmet_skin(dir, wrapname),
                              drop_unbound=occursin("classicview", dir) && get(ENV, "JM_EXT67X_UNBOUND", "0") == "0"))
        if _hel !== nothing
            hp = [Render.TrackPart(let v = copy(p.verts); for i in 1:11:length(v)-10; v[i] += _hel.off[1]; v[i+1] += _hel.off[2]; end; v end,
                                   p.tex == "" ? _hel.helm : p.tex, p.col) for p in Render.extract_classic_car(_hmesh; maxlat = 0.95f0)]
            append!(AICARMODELS[end].body, Render.build_classic(hp, Render.classic_texture_index(joinpath(AIBASE, dir))))
            print("[helmet ", _hel.helm, " at ", round.(_hel.off; digits = 2), "] ")
        end
        println("$(length(AICARMODELS[end].body)) parts")
        if haskey(ENV, "JM_AITEX")
            let b = AICARMODELS[end].body, u = [it for it in b if it.tex == 0]
                println("    [aitex] ", nm, ": ", length(u), " of ", length(b), " parts UNTEXTURED; their colours ",
                        unique([round.(it.col, digits = 2) for it in u])[1:min(end, 6)], "; triangles ", sum(it.n for it in u; init = 0) ÷ 3,
                        " of ", sum(it.n for it in b; init = 0) ÷ 3)
            end
        end
        Render.CLASSIC3DO.HIDE_GROUPS[] = Set{Int}()
    end
end

const AI_HAND = get(ENV, "JM_AI_HAND", "1") == "0" ? Render.scalexyz(1f0, 1f0, 1f0) : CARHAND
const AI_NOAXLE = Render.Item[]
const AI_AXLES = Dict{String,Vector{Render.Item}}()
if get(ENV, "JM_AI_AXLES", "1") != "0"
    for cm in AICARMODELS
        haskey(AI_PARKED_SUSP_GROUPS, lowercase(cm.name)) || continue
        parts = Render.TrackPart[]
        zh = Float32(sum(w[2] for w in cm.wheelspec) / length(cm.wheelspec))
        for (wx, wz, front, r, _) in cm.wheelspec
            front && continue
            zs = sign(wz - zh); zi = zh + 0.16f0*zs; zo = wz - zs*0.05f0
            push!(parts, Render.TrackPart(_rod_verts!(Float32[], Float32[wx, r + AXLE_Y, zi], Float32[wx, r + AXLE_Y, zo], AXLE_R, (0.84f0, 0.02f0)),
                                          "axlelot", (1f0, 1f0, 1f0)))
        end
        isempty(parts) || (AI_AXLES[cm.name] = Render.build_classic(parts, CLASSICTEX; tag="extra"))
    end
    isempty(AI_AXLES) || println("  [ai axles] built driveshafts for: ", join(keys(AI_AXLES), ", "))
end

if !SKIDPAD && length(AICARMODELS) == length(_loadspecs)
    for cm in AICARMODELS; haskey(NET_BY_TYPE, cm.name) || (NET_BY_TYPE[cm.name] = cm); end
    PLAYER_CAR != "Lotus" && (PLAYER_CM[] = AICARMODELS[findlast(sp -> sp[1] == PLAYER_CAR, _loadspecs)])
    resize!(AICARMODELS, min(length(AICARMODELS), _ncars))
end

const FAR_M = parse(Float32, get(ENV, "JM_FAR", "3000"))

const TRACK_MAXD = parse(Float32, get(ENV, "JM_TRACK_MAXD", "2400"))
const PROJ = Render.perspective_revz(deg2rad(62f0), Float32(W/H3), 0.35f0, FAR_M)

proj_full(fovdeg, near) = Render.perspective_revz(Float32(2atan(tan(deg2rad(fovdeg)/2)*H/H3)), Float32(W/H), Float32(near), FAR_M)
const PROJ_F = proj_full(62.0, 0.35)
tstamp("  [E80] AI car models done / projection")

tstamp("  [E80] .. AI cars done -> input/camera defs begin")

const PROJ_COCKPIT = Render.perspective_revz(deg2rad(parse(Float32,get(ENV,"JM_FOV","50"))), Float32(W/H3), 0.20f0, FAR_M)
const PROJ_COCKPIT_F = proj_full(parse(Float64, get(ENV,"JM_FOV","50")), 0.20)

const FS_BTN = something(tryparse(Int, get(ENV, "JM_FULLSCREEN_BTN", "")),
                         isfile(_JOYCONF) ? something((m = match(r"(?m)^fullscreen_btn\s+(\d+)", read(_JOYCONF, String)); m === nothing ? nothing : parse(Int, m[1])), 0) : 0)
const FS_ON = Ref(FULLSCREEN); const FS_PREV = Ref(false)
function toggle_fullscreen!()
    FS_ON[] = !FS_ON[]
    FS_ON[] ? GLFW.SetWindowMonitor(win, GLFW.GetPrimaryMonitor(), 0, 0, W, H, GLFW.DONT_CARE) :
              GLFW.SetWindowMonitor(win, GLFW.Monitor(C_NULL), 40, 40, W, H, GLFW.DONT_CARE)
    println("  [fullscreen] ", FS_ON[] ? "full screen" : "window", " (controller button ", FS_BTN, ")"); flush(stdout)
end

mutable struct Ctl; prevUp::Bool; prevDn::Bool; prevV::Bool; prevG::Bool; prevM::Bool; prevRec::Bool; prevRestart::Bool; view::Int; auto::Bool; cluWarned::Bool; end

const CTL = Ctl(false,false,false,false,false,false,false, parse(Int, get(ENV,"JM_VIEW","1")), get(ENV,"ZAND_SHIFT","auto") != "manual", false)

const ARM_KEY_K = getfield(GLFW, Symbol("KEY_", get(ENV, "JM_ARM_KEY_NAME", "SPACE")))
const ARM_KEY_T = Ref(0.0)

const KEY_AT = let v = split(get(ENV, "JM_KEY_AT", ""), ":"); length(v) == 2 ? (parse(Float64, v[1]), getfield(GLFW, Symbol("KEY_", v[2]))) : nothing end
const KEY_AT_T = Ref(0.0)
key(k) = GLFW.GetKey(win, k) == GLFW.PRESS || (ARM_KEY > 0 && k == ARM_KEY_K && ARM_KEY_T[] > 0 && time() - ARM_KEY_T[] < 0.15) ||
         (KEY_AT !== nothing && k == KEY_AT[2] && KEY_AT_T[] > 0 && time() - KEY_AT_T[] < 0.15)

const ANYKEYS = Tuple(vcat([GLFW.KEY_SPACE, GLFW.KEY_ENTER, GLFW.KEY_TAB, GLFW.KEY_BACKSPACE,
                            GLFW.KEY_UP, GLFW.KEY_DOWN, GLFW.KEY_LEFT, GLFW.KEY_RIGHT,
                            GLFW.KEY_LEFT_SHIFT, GLFW.KEY_RIGHT_SHIFT, GLFW.KEY_LEFT_CONTROL, GLFW.KEY_RIGHT_CONTROL,
                            GLFW.KEY_LEFT_ALT, GLFW.KEY_RIGHT_ALT],
                           [getfield(GLFW, Symbol("KEY_", c)) for c in 'A':'Z'],
                           [getfield(GLFW, Symbol("KEY_", c)) for c in '0':'9']))
anykey() = any(key(k) for k in ANYKEYS)
const JOYREPORT = Ref(false)
const JOYTRACE_T = Ref(-1.0)

const CLUTCH_GATE = Ref(-1.0)

const CLU_HIST = Ref(-1.0e9); const GEAR_NOW = Ref(0); const RPM_NOW = Ref(0.0); const V_NOW = Ref(0.0)

function read_input()
    thr=brk=str=clu=0.0; up=dn=false
    js = SMOKE_NOJOY ? nothing : GLFW.GetJoystickAxes(GLFW.JOYSTICK_1)
    if !JOYREPORT[] && js !== nothing && !isempty(js)
        JOYREPORT[] = true
        println("  [joy] ", length(js), " axes raw = ", join(round.(js, digits=2), ", "))
        _bs = GLFW.GetJoystickButtons(GLFW.JOYSTICK_1)
        println("        buttons = ", _bs === nothing ? "none" : string(length(_bs)),
                "   shift-up btn ", JOYMAP.up_btn, ", shift-down btn ", JOYMAP.dn_btn,
                "   clutch axis ", JOYMAP.clutch.axis)
        flush(stdout)
    end
    if js !== nothing && !isempty(js)
        bs = GLFW.GetJoystickButtons(GLFW.JOYSTICK_1)
        str, thr, brk, clu, up, dn = JoyCfg.apply(JOYMAP, js, bs)
        if FS_BTN > 0
            fp = bs !== nothing && length(bs) >= FS_BTN && bs[FS_BTN] != 0
            (fp && !FS_PREV[]) && toggle_fullscreen!(); FS_PREV[] = fp
        end
        STEER_GAIN[] > 0 && (str = JoyCfg.steer_physical(JOYMAP, js, STEER_GAIN[]; lim = STEER_LIM[]))
    end

    key(GLFW.KEY_W) && (thr=1.0); key(GLFW.KEY_S) && (brk=1.0)
    key(GLFW.KEY_A) && (str=1.0); key(GLFW.KEY_D) && (str=-1.0)
    SMOKE && haskey(ENV, "JM_TEST_STEER") && (str = parse(Float64, ENV["JM_TEST_STEER"]))
    SMOKE && haskey(ENV, "JM_TEST_BRAKE") && (brk = parse(Float64, ENV["JM_TEST_BRAKE"]))
    SMOKE && haskey(ENV, "JM_TEST_CLUTCH") && (clu = parse(Float64, ENV["JM_TEST_CLUTCH"]))
    key(GLFW.KEY_C) && (clu=1.0)
    ku=key(GLFW.KEY_E); kd=key(GLFW.KEY_Q)
    upE = (ku && !CTL.prevUp) || (up && !CTL.prevUp); dnE = (kd && !CTL.prevDn) || (dn && !CTL.prevDn)
    CTL.prevUp = ku||up; CTL.prevDn = kd||dn

    CLU_HIST[] = clu >= 0.25 ? time() : CLU_HIST[]
    if get(ENV,"JM_CLUTCH_REQ","1") != "0" && !CTL.auto && (upE || dnE)
        g2 = clamp(GEAR_NOW[] + (upE ? 1 : -1), 1, length(DriveRT3D.GEARS))
        tgt = GEAR_NOW[] >= 1 ? abs(V_NOW[]) / DriveRT3D.RW_R * DriveRT3D.GEARS[g2] * DriveRT3D.final_eff() * 60/(2π) : 0.0
        JoyCfg.shift_ok(clu, time() - CLU_HIST[], RPM_NOW[], tgt) || (upE = false; dnE = false)
    end
    kv = key(GLFW.KEY_V); (kv && !CTL.prevV) && (CTL.view = 1-CTL.view); CTL.prevV = kv

    kg = key(GLFW.KEY_G)
    if kg && !CTL.prevG
        if CTL.auto && clu < 0.4
            CLUTCH_GATE[] = 2.0
            println("  [gearbox] staying in AUTO -- move the CLUTCH SLIDER to the DISENGAGED end ",
                    "before switching to MANUAL. Slider is at ", round(clu, digits=2), " (need > 0.4). ",
                    "The on-screen bar shows the axis and the threshold.")
            flush(stdout)
        else
            CTL.auto = !CTL.auto
            println("  [gearbox] ", CTL.auto ? "AUTO" : "MANUAL — the clutch slider is yours; no launch assist")
            flush(stdout)
        end
    end
    CTL.prevG = kg
    km = key(GLFW.KEY_M); (km && !CTL.prevM) && (ENG.master[] = ENG.master[]>0 ? 0.0 : 0.7); CTL.prevM = km

    rkey  = key(GLFW.KEY_R); shift = key(GLFW.KEY_LEFT_SHIFT) || key(GLFW.KEY_RIGHT_SHIFT)
    recover = rkey && shift && !CTL.prevRec; CTL.prevRec = rkey && shift

    ctrl = key(GLFW.KEY_LEFT_CONTROL) || key(GLFW.KEY_RIGHT_CONTROL)
    rk = rkey && !shift
    restart = rk && !CTL.prevRestart; CTL.prevRestart = rk
    rst = recover

    CLU_NOW[] = clu

    if get(ENV,"JM_TRACE_CLUTCH","0") != "0" && abs(clu - CLU_LAST[]) > 0.05
        println("  [clutch] clu=", round(clu, digits=2),
                "   (0 = ENGAGED / slider up,  1 = DISENGAGED / slider down)   throttle ",
                round(thr, digits=2))
        flush(stdout)
        CLU_LAST[] = clu
    end
    if clu > 0.9 && thr > 0.2 && !CTL.cluWarned
        CTL.cluWarned = true
        println("  [clutch] held IN (", round(clu,digits=2), ") with throttle — the engine will rev ",
                "but the car will not move. If you are not holding it, the slider (axis 4) is parked ",
                "at the clutch-in end; move it to the other end to release.")
        flush(stdout)
    end
    (DriveInput(throttle=clamp(thr,0,1), brake=clamp(brk,0,1), steer=clamp(str,-STEER_LIM[],STEER_LIM[]),
                clutch=(WRECKED[] ? 1.0 : clu),
                shift_up=(WRECKED[] ? false : upE), shift_down=(WRECKED[] ? false : dnE),
                autoshift=(WRECKED[] && false) || CTL.auto), rst, recover, restart)
end

tstamp("  [E80] .. read_input defined")

const TILT_REF_UP = 1.5; const TILT_MAX = deg2rad(25.0)
const TILT_REFINF = get(ENV, "JM_TERRAIN_TILT_REFINF", "0") != "0"
const TILTDIAG = get(ENV, "JM_TILTDIAG", "0") != "0"; const TILT_N = Ref(0)
const ROLLDIAG = get(ENV, "JM_ROLLDIAG", "0") != "0"; const ROLLDIAG_N = Ref(0)
const ROLL_TWICE = get(ENV, "JM_ROLL_TWICE", "0") != "0"
tilt_ref(cs) = (TILT_REFINF || !hasproperty(cs, :y) || !isfinite(cs.y)) ? Inf : Float64(cs.y) + TILT_REF_UP
function terrain_pitch(cs)
    SKIDPAD && return 0.0
    L = 1.5; fx = cos(cs.θ); fz = sin(cs.θ)
    hf = JRTrack.hat3d(TERRAIN, cs.x+fx*L, cs.z+fz*L; ref=tilt_ref(cs))
    hr = JRTrack.hat3d(TERRAIN, cs.x-fx*L, cs.z-fz*L; ref=tilt_ref(cs))
    (hf[3] && hr[3]) ? clamp(atan(hf[1]-hr[1], 2L), -TILT_MAX, TILT_MAX) : 0.0
end

function terrain_roll(cs)
    SKIDPAD && return 0.0
    L = 1.3; lx = -sin(cs.θ); lz = cos(cs.θ)
    hl = JRTrack.hat3d(TERRAIN, cs.x+lx*L, cs.z+lz*L; ref=tilt_ref(cs))
    hr = JRTrack.hat3d(TERRAIN, cs.x-lx*L, cs.z-lz*L; ref=tilt_ref(cs))
    (hl[3] && hr[3]) ? clamp(atan(hl[1]-hr[1], 2L), -TILT_MAX, TILT_MAX) : 0.0
end

const DIFFN = Ref(0)
if get(ENV, "JM_TILTPROBE", "") != ""
    let rd = deserialize(ENV["JM_TILTPROBE"]), st = 1 + 4*rd.ncar, d = rd.data, nbad = 0, worst = 0.0
        refinf(cs) = (x = cs.x, y = Inf, z = cs.z, θ = cs.θ)
        for i in 0:rd.nframes-1
            b = i*st; cs = (x = Float64(d[b+2]), y = Float64(d[b+3]), z = Float64(d[b+4]), θ = Float64(d[b+5]))
            ro = terrain_roll(refinf(cs)); po = terrain_pitch(refinf(cs)); rn = terrain_roll(cs); pn = terrain_pitch(cs)
            m = max(abs(ro), abs(po)); worst = max(worst, m)
            if abs(po - pn) > deg2rad(2) || abs(ro - rn) > deg2rad(2)
                DIFFN[] += 1
                DIFFN[] <= 30 && println("  [tiltprobe diff] frame ", i, " t=", round(d[b+1], digits = 1), " (", round(cs.x, digits = 1), ", ",
                                         round(cs.z, digits = 1), ")  old roll/pitch ", round(rad2deg(ro), digits = 1), "/", round(rad2deg(po), digits = 1),
                                         "  new ", round(rad2deg(rn), digits = 1), "/", round(rad2deg(pn), digits = 1))
            end
            if m > deg2rad(20)
                nbad += 1
                nbad <= 25 && println("  [tiltprobe] frame ", i, " t=", round(d[b+1], digits = 1), " (", round(cs.x, digits = 1), ", ",
                                      round(cs.z, digits = 1), ")  OLD roll ", round(rad2deg(ro), digits = 1), " pitch ", round(rad2deg(po), digits = 1),
                                      "  ->  NEW roll ", round(rad2deg(rn), digits = 1), " pitch ", round(rad2deg(pn), digits = 1))
            end
        end

        R = Float64[]
        for i in 0:15:rd.nframes-1
            b = i*st; cs = (x = Float64(d[b+2]), y = Float64(d[b+3]), z = Float64(d[b+4]), θ = Float64(d[b+5]))
            rn = rad2deg(terrain_roll(cs)); pn = rad2deg(terrain_pitch(cs)); push!(R, abs(rn))
            if abs(rn) > 3 || abs(pn) > 4
                hr = JRTrack.hat(TRKSURF, cs.x, cs.z)
                println("  [tiltprobe big] t=", round(d[b+1], digits = 1), " lapdist ", hr.found ? round(Int, hr.lapdist) : -1, " lat ",
                        hr.found ? round(hr.lateral, digits = 1) : NaN, "  roll ", round(rn, digits = 1), " pitch ", round(pn, digits = 1))
            end
        end
        isempty(R) || println("  [tiltprobe] drawn |roll| p50 ", round(sort(R)[cld(end, 2)], digits = 2), " p90 ", round(sort(R)[ceil(Int, 0.9*end)], digits = 2),
                              " max ", round(maximum(R), digits = 2), " deg")
        println("  [tiltprobe] ", rd.nframes, " recorded frames: ", nbad, " with the old drawn tilt > 20 deg (worst ",
                round(rad2deg(worst), digits = 1), " deg)"); flush(stdout); exit(0)
    end
end

const CHASE_D  = parse(Float32, get(ENV,"JM_CHASE_D","5.75"))
const CHASE_H  = parse(Float32, get(ENV,"JM_CHASE_H","1.93"))
const CHASE_LY = parse(Float32, get(ENV,"JM_CHASE_LY","0.63"))
const PROJ_CHASE = Render.perspective_revz(deg2rad(parse(Float32, get(ENV,"JM_CHASE_FOV","51"))), Float32(W/H3), 0.35f0, FAR_M)
const PROJ_CHASE_F = proj_full(parse(Float64, get(ENV,"JM_CHASE_FOV","51")), 0.35)
const CHASE_MIN = parse(Float64, get(ENV,"JM_CHASE_MIN","0.45"))
function camera(cs, pitch=0.0, roll=0.0)
    wx,wy,wz = cs.x, cs.y, -cs.z; fx,fz = cos(cs.θ), -sin(cs.θ)
    if CTL.view == 1

        eye=[wx-fx*CHASE_D, wy+CHASE_H, wz-fz*CHASE_D]; ctr=[wx+fx*2, wy+CHASE_LY, wz+fz*2]

        if @isdefined(TERRAIN)
            h = JRTrack.hat3d(TERRAIN, Float64(eye[1]), Float64(-eye[3]); ref=Inf)
            h[3] && (eye[2] = max(eye[2], Float64(h[1]) + CHASE_MIN))
        end
        return (band_shown() ? PROJ_CHASE : PROJ_CHASE_F) * Render.lookat(Float32.(eye), Float32.(ctr), Float32[0,1,0]), Float32.(eye)
    end

    ex,ey,ez,drop = parse(Float32,get(ENV,"JM_EYE_X","0.095")), parse(Float32,get(ENV,"JM_EYE_Y","0.40")), 0.0f0, parse(Float32,get(ENV,"JM_EYE_DROP","0.22"))
    R = Render.roty(Float32(cs.θ)) * Render.rotz(Float32(pitch)) * Render.rotx(Float32(roll))
    if WBODY[] !== nothing

        _m = WreckBody.model_matrix(WBODY[]); R = Render.M4(ntuple(i -> i <= 12 ? _m[i] : i == 16 ? 1f0 : 0f0, Val(16))...)
    elseif isfinite(REP_BODY[][1])
        R = Render.roty(Float32(cs.θ)) * Render.rotz(Float32(REP_BODY[][1])) * Render.rotx(Float32(REP_BODY[][2]))
    end
    R3(a,b,c) = (w = R * Float32[a,b,c,0f0]; Float32[w[1],w[2],w[3]])
    eye = Float32[wx,wy,wz] + R3(BODY_OFF[1]+ex, BODY_OFF[2]+ey, BODY_OFF[3]+ez)
    ctr = eye + R3(4f0, -drop, 0f0)
    up  = R3(0f0, 1f0, 0f0)
    (band_shown() ? PROJ_COCKPIT : PROJ_COCKPIT_F) * Render.lookat(eye, ctr, up), eye
end

const PROJ_MIRROR = (MIRROR_UNFLIP ? Render.scalexyz(1f0,1f0,1f0) : Render.scalexyz(-1f0,1f0,1f0)) *
                    Render.perspective_revz(deg2rad(parse(Float32,get(ENV,"JM_MIRROR_FOV","95"))), Float32(MIRW÷2)/Float32(MIRH), 0.35f0, 3000f0)
function mirror_camera(cs, pitch=0.0, roll=0.0, side=1)
    wx,wy,wz = cs.x, cs.y, -cs.z
    R = Render.roty(Float32(cs.θ)) * Render.rotz(Float32(pitch)) * Render.rotx(Float32(roll))
    R3(a,b,c) = (w = R * Float32[a,b,c,0f0]; Float32[w[1],w[2],w[3]])

    mx = parse(Float32,get(ENV,"JM_MIRCAM_X","0.55")); my = parse(Float32,get(ENV,"JM_MIRCAM_Y","0.45")); mz = parse(Float32,get(ENV,"JM_MIRCAM_Z","0.55"))
    eye = Float32[wx,wy,wz] + R3(BODY_OFF[1]+mx, BODY_OFF[2]+my, side*mz)

    drop = parse(Float32, get(ENV,"JM_MIRROR_DROP","-0.45")); yawout = parse(Float32, get(ENV,"JM_MIRROR_YAWOUT","0.8"))
    ctr = eye + R3(-4f0, drop, side*yawout)
    PROJ_MIRROR * Render.lookat(eye, ctr, R3(0f0,1f0,0f0)), eye
end

const REPLAY_CAMS  = (:cockpit, :chase, :tv, :f10, :nose, :rsusp)
const REPLAY_CAM_LABEL = ("COCKPIT", "CHASE (above rear)", "TV / DISTANT",
                          "F10 REAR", "NOSE / FRONT", "RR SUSPENSION")

const AICAM = parse(Int, get(ENV, "JM_AICAM", "0"))

const AICAM_SEQ = [(parse(Int, split(q, ":")[1]), Symbol(split(q, ":")[2])) for q in split(get(ENV, "JM_AICAM_SEQ", ""), ",", keepempty = false)]
const AICAM_SEG = 25
const AICAM_R = Ref(isempty(AICAM_SEQ) ? AICAM : AICAM_SEQ[1][1])
const AIVIEW_R = Ref(isempty(AICAM_SEQ) ? :classicchase : AICAM_SEQ[1][2])
const AIPOSE_TRACE = get(ENV, "JM_AIPOSE_TRACE", "0") == "all" ? -1 : parse(Int, get(ENV, "JM_AIPOSE_TRACE", "0"))
function aicam_camera(p)
    v0 = CTL.view; CTL.view = 1
    r = camera((x = p[1], y = p[2], z = p[3], θ = p[4]))
    CTL.view = v0
    r
end

"REPCAM-EARTH-1: the heading of the focus car's path at replay time rt (physics x, z; θ when it never moved)."
function rep_travel_heading(replay_poses, rt, focus, θ)
    pos(t) = (p = replay_poses(t); q = focus == 0 ? p[1] : p[2][focus]; (Float64(q[1]), Float64(q[3])))
    (x1, z1) = pos(rt)
    for back in (0.4, 0.8, 1.6, 3.2, 6.4, 12.8, 25.6, 51.2, 102.4)
        rt - back < 0 && break
        (x0, z0) = pos(rt - back)
        hypot(x1 - x0, z1 - z0) >= 1.5 && return atan(z1 - z0, x1 - x0)
    end
    θ
end

function replay_camera(mode, x, y, z, θ, ψ = NaN; pitch = 0.0, roll = 0.0)
    wx, wy, wz = Float32(x), Float32(y), Float32(-z)

    mode in (:chase, :tv, :f10, :nose) && isfinite(ψ) && (θ = ψ)
    fx, fz = Float32(cos(θ)), Float32(-sin(θ))
    rx, rz = -fz, fx
    P = Float32[wx, wy, wz]
    if mode === :cockpit
        ex,ey,drop = parse(Float32,get(ENV,"JM_EYE_X","0.095")), parse(Float32,get(ENV,"JM_EYE_Y","0.40")), parse(Float32,get(ENV,"JM_EYE_DROP","0.22"))

        R = Render.roty(Float32(θ)) * Render.rotz(Float32(pitch)) * Render.rotx(Float32(roll))
        R3(a,b,c) = (w = R*Float32[a,b,c,0f0]; Float32[w[1],w[2],w[3]])
        eye = P + R3(BODY_OFF[1]+ex, BODY_OFF[2]+ey, 0f0)
        ctr = eye + R3(4f0, -drop, 0f0); up = R3(0f0,1f0,0f0)
        return (band_shown() ? PROJ_COCKPIT : PROJ_COCKPIT_F) * Render.lookat(eye, ctr, up), eye
    end
    eye, ctr = if mode === :chase
        (Float32[wx-fx*9, wy+3.2f0, wz-fz*9],  Float32[wx+fx*3, wy+0.6f0, wz+fz*3])
    elseif mode === :tv
        (Float32[wx+rx*15-fx*5, wy+9.5f0, wz+rz*15-fz*5],  Float32[wx, wy+0.6f0, wz])
    elseif mode === :f10
        (Float32[wx-fx*6, wy+1.3f0, wz-fz*6],  Float32[wx+fx*8, wy+0.9f0, wz+fz*8])
    elseif mode === :nose
        (Float32[wx+fx*7, wy+1.2f0, wz+fz*7],  Float32[wx, wy+0.7f0, wz])
    else
        cx = wx - fx*1.8f0 + rx*1.0f0; cz = wz - fz*1.8f0 + rz*1.0f0
        (Float32[cx+rx*1.6f0-fx*0.4f0, wy+0.75f0, cz+rz*1.6f0-fz*0.4f0],  Float32[cx, wy+0.40f0, cz])
    end
    (band_shown() ? PROJ : PROJ_F) * Render.lookat(eye, ctr, Float32[0,1,0]), eye
end

tstamp("  [E80] .. camera/replay-camera defs done")
if get(ENV,"JM_ROADTEX_CENSUS","")!=""
  let

    near = Dict{String,Int}(); far = Dict{String,Int}()
    for t in TRACKMESH.tris
        cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
        cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
        hr = JRTrack.hat(TRKSURF, cx, cy)
        hr.found || continue
        lt = lowercase(t.tex)
        if abs(hr.lateral) < 5.0
            near[lt] = get(near,lt,0) + 1
        elseif abs(hr.lateral) < 25.0
            far[lt] = get(far,lt,0) + 1
        end
    end

    if get(ENV,"JM_ROADTEX_AT","") != ""
        want = [parse(Float64,x) for x in split(ENV["JM_ROADTEX_AT"], ",")]
        for w in want
            loc = Dict{String,Int}(); nrec = 0; ntot = 0
            for t in TRACKMESH.tris
                cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
                cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
                hr = JRTrack.hat(TRKSURF, cx, cy)
                (hr.found && abs(hr.lapdist - w) < 125.0 && abs(hr.lateral) < 12.0) || continue
                lt = lowercase(t.tex); loc[lt] = get(loc,lt,0)+1
                ntot += 1; ROAD_TEX(lt) && (nrec += 1)
            end
            println("   -- lapdist ", round(Int,w), " ±125 m, |lat|<12 m: ", ntot, " tris, ",
                    nrec, " recognised as road --")
            for (nm,c) in sort(collect(loc), by=x->-x[2])[1:min(end,8)]
                println("        ", rpad(nm,14), rpad(c,6), ROAD_TEX(nm) ? "road" : "*** not road ***")
            end
            isempty(loc) && println("        (NO triangles at all within 12 m of the centreline)")
        end
    end

    suspect = [(nm, c, get(far,nm,0)) for (nm,c) in near
               if !ROAD_TEX(nm) && c >= 60 && c/(c + get(far,nm,0) + 1e-9) >= 0.65]
    sort!(suspect, by=x->-x[2])
    println("== E69-S6 VERDICT: missed textures that are mostly ON the racing surface ==")
    if isempty(suspect)
        println("   none — every unrecognised surface here is predominantly off-road (grass /")
        println("   sand / fence / barrier). The classifier is sound for this track regardless")
        println("   of the headline percentage.")
    else
        println("   ⚠️ these look like ROAD and are being excluded:")

        for (nm,on,off) in suspect
            sd = Float64[]; lt2 = Float64[]
            for t in TRACKMESH.tris
                lowercase(t.tex) == nm || continue
                cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
                cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
                hr = JRTrack.hat(TRKSURF, cx, cy); hr.found || continue
                push!(sd, hr.lapdist); push!(lt2, abs(hr.lateral))
            end
            loc = isempty(sd) ? "?" : string(round(Int,minimum(sd)), "–", round(Int,maximum(sd)), " m")
            nearest = isempty(lt2) ? NaN : minimum(lt2)
            med = isempty(lt2) ? NaN : sort(lt2)[cld(length(lt2),2)]
            println("      ", rpad(nm,14), "on ", rpad(on,6), "off ", rpad(off,6),
                    rpad(string(round(100*on/(on+off), digits=1),"%"),8),
                    " lapdist ", rpad(loc,16),
                    " nearest |lat| ", rpad(round(nearest,digits=1),6),
                    " median |lat| ", round(med,digits=1))
        end
    end

    if get(ENV,"JM_TEXLAT","") != ""
        println("== JM_TEXLAT: where the ACCEPTED road textures actually sit ==")
        for nm in split(ENV["JM_TEXLAT"], ",")
            lt3 = Float64[]
            for t in TRACKMESH.tris
                lowercase(t.tex) == lowercase(nm) || continue
                cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
                cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
                hr = JRTrack.hat(TRKSURF, cx, cy)
                hr.found && abs(hr.lateral) < 25.0 && push!(lt3, abs(hr.lateral))
            end
            if isempty(lt3)
                println("   ", rpad(nm,12), "(none)")
            else
                sort!(lt3)
                println("   ", rpad(nm,12), "n=", rpad(length(lt3),7),
                        "median |lat| ", rpad(round(lt3[cld(end,2)],digits=1),7),
                        "p25 ", rpad(round(lt3[max(1,cld(end,4))],digits=1),7),
                        ROAD_TEX(lowercase(nm)) ? "ACCEPTED" : "rejected")
            end
        end
    end
    println("== ROAD_TEX census: textures of tris within |lat| < 5 m of the centreline ==")
    println("   (recognised? = does the current ROAD_TEX classifier accept the name)")
    for (lt,n) in sort(collect(near), by=x->-x[2])[1:min(end,18)]
        println("   ", rpad(lt=="" ? "<none>" : lt, 16), rpad(n,7),
                ROAD_TEX(lt) ? "recognised" : "*** MISSED ***",
                "   (off-road count ", get(far,lt,0), ")")
    end
    tot = sum(values(near)); rec = sum(n for (lt,n) in near if ROAD_TEX(lt); init=0)
    println("   --> ", rec, "/", tot, " near-centreline tris recognised (",
            tot>0 ? round(100rec/tot,digits=1) : 0.0, "%)")
    flush(stdout)
    end
end

if get(ENV,"JM_ROADWIDTH","")!=""
  let

    step = parse(Float64, get(ENV,"JM_ROADWIDTH_STEP","500"))

    halfb = parse(Float64, get(ENV,"JM_ROADWIDTH_BUCKET","12.0"))
    buckets = Dict{Int,Vector{Float64}}()
    nrt = 0
    for t in TRACKMESH.tris
        ROAD_TEX(lowercase(t.tex)) || continue
        nrt += 1
        for vi in 1:3
            x = Float64(t.p[vi][1]); y = Float64(t.p[vi][2])
            hr = JRTrack.hat(TRKSURF, x, y)
            hr.found || continue
            b = round(Int, hr.lapdist / step)
            r = hr.lapdist - b*step
            abs(r) <= halfb || continue
            push!(get!(buckets, b, Float64[]), hr.lateral)
        end
    end
    println("   ⚠️ E73-S8: this census samples road VERTICES inside a ±", halfb,
            " m LONGITUDINAL slice, so on a coarsely-meshed track (Monza, Watkins) a station can")
    println("      hold no vertices at all, or only the narrow `groove` strip's. Use JM_ROADWIDTH2")
    println("      (coverage) for any width verdict; this one is kept only to reproduce old numbers.")
    println("== E71-S7 rendered road width from the mesh (ROAD_TEX tris=", nrt,
            ", bucket ±", halfb, " m, step ", step, " m) ==")
    println("   lapdist    lat_min   lat_max    WIDTH    n")
    tot = Float64[]
    for b in sort(collect(keys(buckets)))
        v = buckets[b]; length(v) >= 6 || continue
        lo, hi = minimum(v), maximum(v)
        push!(tot, hi-lo)
        println("   ", rpad(b*step,10), rpad(round(lo,digits=1),10), rpad(round(hi,digits=1),10),
                rpad(round(hi-lo,digits=1),9), length(v))
    end
    if !isempty(tot)
        st = sort(tot)
        println("   --> median width ", round(st[cld(end,2)],digits=1), " m   min ",
                round(minimum(tot),digits=1), "   max ", round(maximum(tot),digits=1),
                "   (⚠️ the old 'gold Spa ~11 m' reference is UNVERIFIED — E71-S13)")
    end
    flush(stdout)
    end
end

if get(ENV,"JM_ROADWIDTH2","")!=""
  let

    stepS = parse(Float64, get(ENV,"JM_ROADWIDTH2_STEP","250.0"))
    maxlat = parse(Float64, get(ENV,"JM_ROADWIDTH2_MAXLAT","22.0"))
    dlat  = 0.25
    rt = [t for t in TRACKMESH.tris if ROAD_TEX(lowercase(t.tex))]

    cls = Dict{Int,Vector{Any}}()
    for t in rt
        cx = (Float64(t.p[1][1])+Float64(t.p[2][1])+Float64(t.p[3][1]))/3
        cy = (Float64(t.p[1][2])+Float64(t.p[2][2])+Float64(t.p[3][2]))/3
        hr = JRTrack.hat(TRKSURF, cx, cy); hr.found || continue
        push!(get!(cls, round(Int, hr.lapdist/100.0), Any[]), t)
    end
    intri(px,py,t) = begin
        ax,ay = Float64(t.p[1][1]), Float64(t.p[1][2])
        bx,by = Float64(t.p[2][1]), Float64(t.p[2][2])
        cx2,cy2 = Float64(t.p[3][1]), Float64(t.p[3][2])
        d = (by-cy2)*(ax-cx2) + (cx2-bx)*(ay-cy2)
        abs(d) < 1e-12 && return false
        l1 = ((by-cy2)*(px-cx2) + (cx2-bx)*(py-cy2))/d
        l2 = ((cy2-ay)*(px-cx2) + (ax-cx2)*(py-cy2))/d
        l3 = 1.0 - l1 - l2
        l1 >= -1e-9 && l2 >= -1e-9 && l3 >= -1e-9
    end
    println("== E73-S8 road width by COVERAGE (road tris=", length(rt), ", step ", stepS, " m) ==")
    println("   lapdist   left    right   WIDTH   gaps")
    widths = Float64[]
    nst = max(1, floor(Int, LAPLEN/stepS))
    for k in 0:nst
        sdist = k*stepS

        bi = argmin([abs(TRKSURF.lapdist[i] - sdist) for i in 1:length(TRKSURF.lapdist)])
        p0 = ALIGNED[min(bi, length(ALIGNED))]
        p1 = ALIGNED[min(bi+1, length(ALIGNED))]
        tx, ty = Float64(p1[1]-p0[1]), Float64(p1[2]-p0[2])
        L = hypot(tx,ty); L < 1e-6 && continue
        nx, ny = -ty/L, tx/L
        cand = Any[]
        for kk in (round(Int,sdist/100.0)-1):(round(Int,sdist/100.0)+1)
            haskey(cls,kk) && append!(cand, cls[kk])
        end
        isempty(cand) && continue
        cov = Bool[]
        lats = collect(-maxlat:dlat:maxlat)
        for lt in lats
            px = Float64(p0[1]) + nx*lt; py = Float64(p0[2]) + ny*lt
            hit = false
            for t in cand; if intri(px,py,t); hit = true; break; end; end
            push!(cov, hit)
        end
        any(cov) || continue

        i0 = findfirst(x->x, cov); i1 = findlast(x->x, cov)
        mid = cld(length(cov),2)
        lo, hi = mid, mid
        if cov[mid]
            while lo > 1 && cov[lo-1]; lo -= 1; end
            while hi < length(cov) && cov[hi+1]; hi += 1; end
        else
            lo, hi = i0, i1
        end
        w = (hi-lo)*dlat
        ngap = count(i -> !cov[i] && cov[i-1], 2:length(cov))
        push!(widths, w)
        println("   ", rpad(round(Int,sdist),9), rpad(round(lats[lo],digits=1),8),
                rpad(round(lats[hi],digits=1),8), rpad(round(w,digits=1),8), ngap)
    end
    if !isempty(widths)
        st = sort(widths)
        println("   --> median ", round(st[cld(end,2)],digits=1), " m   min ", round(minimum(widths),digits=1),
                "   max ", round(maximum(widths),digits=1), "   stations ", length(widths), "/", nst+1)
    end
    flush(stdout)
    end
end

tstamp("  [E80] .. main() defined; physics build next")

const TARMAC_MU = get(ENV, "JM_TARMACMU", "1") != "0"
@inline function wheel_mu(cx::Float64, cz::Float64, c::Float64, s::Float64, xi::Float64, yi::Float64)
    px = cx + xi*c - yi*s; pz = cz + xi*s + yi*c
    hw = JRTrack.hat(TRKSURF, px, pz)
    (hw.found && abs(hw.lateral) > ROAD_HALFW) || return 1.0
    (TARMAC_MU && ROADHAT !== TERRAIN0 && JRTrack.hat3d(ROADHAT, px, pz; ref = Inf)[3]) ? 1.0 : GRASS_MU
end

SKIDPAD || wheel_mu(0.0, 0.0, 1.0, 0.0, 1.314, 0.75)

precompile(JRPhysics.DriveRT3D.damage_impact!, (Float64, Float64, Float64))
precompile(carhit_damage!, (JRPhysics.DriveRT3D.Car3D, Float64, Float64, Float64))
precompile(Render.text_draw, (UInt32, UInt32, UInt32, Render.Font, Vector{Float32}, Int, Int))

if CLASSICW_ON
    precompile(classicwall_contact, (Float64, Float64, Float64, Float64)); precompile(classicwall_gaps, (Float64, Float64, Float64))
    precompile(obs_contact, (Float64, Float64, Float64, Float64)); precompile(obs_gap, (Int32, Float64, Float64, Float64, Float64, Float64))
    precompile(seg_seg, ntuple(_ -> Float64, 8)); precompile(CLASSICWall.free_interval, (CLASSICWall.Walls, Int, Float64, Float64, Float64))
    precompile(classicwall_clamp!, (JRPhysics.DriveRT3D.Car3D,)); precompile(obs_clamp!, (JRPhysics.DriveRT3D.Car3D,))
end

const PICKS = [Tuple(parse.(Float64, split(q, ","))) for q in split(get(ENV, "JM_PICK", ""), ";", keepempty = false)]
function mt_hit(o, d, a, b, c)
    e1 = b .- a; e2 = c .- a
    pv = (d[2]*e2[3] - d[3]*e2[2], d[3]*e2[1] - d[1]*e2[3], d[1]*e2[2] - d[2]*e2[1])
    det = e1[1]*pv[1] + e1[2]*pv[2] + e1[3]*pv[3]; abs(det) < 1e-12 && return Inf
    tv = o .- a; u = (tv[1]*pv[1] + tv[2]*pv[2] + tv[3]*pv[3])/det; (u < 0 || u > 1) && return Inf
    qv = (tv[2]*e1[3] - tv[3]*e1[2], tv[3]*e1[1] - tv[1]*e1[3], tv[1]*e1[2] - tv[2]*e1[1])
    v = (d[1]*qv[1] + d[2]*qv[2] + d[3]*qv[3])/det; (v < 0 || u + v > 1) && return Inf
    t = (e2[1]*qv[1] + e2[2]*qv[2] + e2[3]*qv[3])/det
    t > 0.05 ? t : Inf
end

const PICKS_BANDMIR = [Tuple(parse.(Float64, split(q, ","))) for q in split(get(ENV, "JM_PICK_BANDMIR", ""), ";", keepempty = false)]
function pick_report(vp, eye, label; picks = PICKS, w = W, h = H)
    iv = inv(Matrix{Float64}(vp))
    unp(x, y, z) = (q = iv*[x, y, z, 1.0]; (q[1]/q[4], q[2]/q[4], q[3]/q[4]))
    for (px, py) in picks
        nx = 2*(px + 0.5)/w - 1; ny = 1 - 2*(py + 0.5)/h
        a = unp(nx, ny, 1.0); b = unp(nx, ny, 0.5)
        d = b .- a; d = d ./ sqrt(sum(abs2, d)); o = a

        hits = Tuple{Float64,String,String}[]
        function tri!(src, nm, P1, P2, P3)
            t = mt_hit(o, d, P1, P2, P3); isfinite(t) || return
            e1 = P2 .- P1; e2 = P3 .- P1; n = (e1[2]*e2[3] - e1[3]*e2[2], e1[3]*e2[1] - e1[1]*e2[3], e1[1]*e2[2] - e1[2]*e2[1])
            push!(hits, (t, src, nm * (n[1]*d[1] + n[2]*d[2] + n[3]*d[3] < 0 ? " [front]" : " [back]")))
        end
        for (pi, p) in enumerate(TRACK)
            v = p.verts
            cl = !((@isdefined SEC_FROM) && (@isdefined TRACK_BOARDCULL)) ? "?" : pi >= SEC_FROM ? "section" : (pi <= length(TRACK_RAILCULL) && TRACK_RAILCULL[pi]) ? "rail" :
                 (pi <= length(TRACK_BOARDCULL) && TRACK_BOARDCULL[pi]) ? "board" : "2-sided"
            for k in 1:33:length(v)-32
                tri!("track", p.tex * " #" * string(pi) * " " * cl, (Float64(v[k]), Float64(v[k+1]), Float64(v[k+2])), (Float64(v[k+11]), Float64(v[k+12]), Float64(v[k+13])),
                     (Float64(v[k+22]), Float64(v[k+23]), Float64(v[k+24])))
            end
        end
        for ob in OBJECTS
            ps = get(OBJ_CPU, lowercase(String(ob[5])), nothing); ps === nothing && continue
            M = ob[2]; tf(i, v) = (M[1,1]*v[i] + M[1,2]*v[i+1] + M[1,3]*v[i+2] + M[1,4], M[2,1]*v[i] + M[2,2]*v[i+1] + M[2,3]*v[i+2] + M[2,4],
                                    M[3,1]*v[i] + M[3,2]*v[i+1] + M[3,3]*v[i+2] + M[3,4])
            for p in ps, k in 1:33:length(p.verts)-32
                tri!("object", String(ob[5]) * "/" * p.tex, tf(k, p.verts), tf(k + 11, p.verts), tf(k + 22, p.verts))
            end
        end
        for (it, pos, w, h, yaw) in STATICTREES
            c_ = cos(Float64(yaw)); s_ = sin(Float64(yaw))
            cn(lx, ly) = (pos[1] + lx*w*c_, pos[2] + ly*h, pos[3] - lx*w*s_)
            tri!("forest", "panel", cn(-0.5, 0), cn(0.5, 0), cn(0.5, 1)); tri!("forest", "panel", cn(-0.5, 0), cn(0.5, 1), cn(-0.5, 1))
        end
        for (it, pos, w, h) in BILLBOARDS
            M = copy(Render.billboard_model(pos, w, h, eye))
            cn(lx, ly) = (M[1,1]*lx + M[1,2]*ly + M[1,4], M[2,1]*lx + M[2,2]*ly + M[2,4], M[3,1]*lx + M[3,2]*ly + M[3,4])
            tri!("billboard", "h$(round(Int, h))", cn(-0.5, 0), cn(0.5, 0), cn(0.5, 1)); tri!("billboard", "h$(round(Int, h))", cn(-0.5, 0), cn(0.5, 1), cn(-0.5, 1))
        end
        sort!(hits, by = h -> h[1])
        println("  [pick] ", label, " pixel (", round(Int, px), ",", round(Int, py), "): ", isempty(hits) ? "nothing (sky)" : "")

        isempty(hits) && d[2] < 0 && println("         ray: ", join([string(round.(o .+ t .* d, digits = 1)) for t in 10:10:80], " "))
        seen = Set{String}()
        for (t, src, nm) in hits
            key = src * nm; key in seen && continue; push!(seen, key)
            P = o .+ t .* d; hr = JRTrack.hat(TRKSURF, P[1], -P[3])
            println("     ", rpad(round(t, digits = 1), 7), " m  ", rpad(src, 9), rpad(nm, 28), " world (", round(P[1], digits = 1), ", ", round(-P[3], digits = 1),
                    ") h ", round(P[2], digits = 1), hr.found ? "  lapdist $(round(Int, hr.lapdist)) lat $(round(hr.lateral, digits = 1))" : "")
            length(seen) >= 4 && break
        end
    end
    flush(stdout)
end

if get(ENV, "JM_LIFTPROBE", "") != ""
    let rd = deserialize(ENV["JM_LIFTPROBE"]), st = 1 + 4*rd.ncar, d = rd.data
        tris = NTuple{3,NTuple{3,Float64}}[]
        for p in TRACK
            ROAD_TEX(lowercase(p.tex)) || continue
            v = p.verts
            for k in 1:33:length(v)-32
                tg = get(Render.CLASSIC3DO.BANDTAG, Render.CLASSIC3DO.bandkey((v[k], -v[k+2], v[k+1]), (v[k+11], -v[k+13], v[k+12]), (v[k+22], -v[k+24], v[k+23])), Int32(0))
                (tg != 0 && (tg & 1) == 0) && continue
                push!(tris, ((v[k], v[k+1], v[k+2]), (v[k+11], v[k+12], v[k+13]), (v[k+22], v[k+23], v[k+24])))
            end
        end
        cx = [(t[1][1] + t[2][1] + t[3][1])/3 for t in tris]; cz = [(t[1][3] + t[2][3] + t[3][3])/3 for t in tris]
        G = Float64[]; P = Float64[]; nbig = 0
        rg = split(get(ENV, "JM_LIFTPROBE_RANGE", ""), ","); r0, r1 = length(rg) == 2 ? parse.(Float64, rg) : (NaN, NaN)
        for i in 0:(isnan(r0) ? 5 : 1):rd.nframes-1
            b = i*st; x = Float64(d[b+2]); y = Float64(d[b+3]); z = Float64(d[b+4])
            o = (x, y + 1.5, -z); dn = (0.0, -1.0, 0.0); best = Inf
            for j in eachindex(tris)
                (abs(cx[j] - x) < 25 && abs(cz[j] + z) < 25) || continue
                t = mt_hit(o, dn, tris[j]...); t < best && (best = t)
            end
            isfinite(best) || continue
            road = y + 1.5 - best; gap = y - road; push!(G, gap)
            ht = JRTrack.hat3d(TERRAIN, x, z; ref = y + 1.5); phys = ht[3] ? y - ht[1] : NaN; push!(P, phys)
            if !isnan(r0)
                hr = JRTrack.hat(TRKSURF, x, z)
                if hr.found && r0 <= hr.lapdist <= r1 && ht[3] && ht[1] - road > 0.04 && (DIFFN[] += 1) <= 6
                    for p in TRACK
                        v = p.verts
                        for k in 1:33:length(v)-32
                            tt = mt_hit(o, dn, (v[k], v[k+1], v[k+2]), (v[k+11], v[k+12], v[k+13]), (v[k+22], v[k+23], v[k+24]))
                            isfinite(tt) && println("     hit ", rpad(p.tex, 14), " h ", round(y + 1.5 - tt, digits = 3), ROAD_TEX(lowercase(p.tex)) ? "  (road)" : "")
                        end
                    end
                end
                (hr.found && r0 <= hr.lapdist <= r1) && println("  [liftprobe range] t=", round(d[b+1], digits = 2), " lapdist ", round(hr.lapdist, digits = 1),
                    " lat ", round(hr.lateral, digits = 1), "  drawn road ", round(road, digits = 3), "  physics ", round(ht[3] ? ht[1] : NaN, digits = 3), "  car ", round(y, digits = 3))
                continue
            end
            if abs(gap) > 0.15
                nbig += 1
                hr = JRTrack.hat(TRKSURF, x, z)
                nbig <= 60 && println("  [liftprobe] t=", round(d[b+1], digits = 1), " lapdist ", hr.found ? round(Int, hr.lapdist) : -1,
                                      " lat ", hr.found ? round(hr.lateral, digits = 1) : NaN, "  car above drawn road ", round(gap, digits = 2),
                                      " m  above physics ground ", round(phys, digits = 2), " m")
            end
        end
        sg = sort(G)
        println("  [liftprobe] ", length(G), " samples: car - drawn road p5 ", round(sg[max(1, round(Int, 0.05end))], digits = 3), " p50 ",
                round(sg[cld(end, 2)], digits = 3), " p95 ", round(sg[round(Int, 0.95end)], digits = 3), " max ", round(sg[end], digits = 3),
                "; car - physics ground p50 ", round(sort(filter(isfinite, P))[cld(count(isfinite, P), 2)], digits = 3), "; ", nbig, " gaps > 0.15 m")
        flush(stdout); exit(0)
    end
end

const _DX_GROUNDPROF  = get(ENV, "JM_GROUNDPROF", "") != ""
const _DX_HOLECENSUS  = get(ENV, "JM_HOLECENSUS", "") != ""
const _DX_START_S     = haskey(ENV, "JM_START_S")
const _DX_OFFTRACK    = haskey(ENV, "JM_OFFTRACK_PROBE")
const _DX_BRIDGEPROBE = get(ENV, "JM_BRIDGEPROBE", "") != ""
const _DX_SWEEP       = haskey(ENV, "JM_SWEEP")
const _DX_ROADSWEEP   = get(ENV, "JM_ROADSWEEP", "") != ""
const _DX_ROADTRIS    = get(ENV, "JM_ROADTRIS", "") != ""
const _DX_ROADEDGE    = get(ENV, "JM_ROADEDGE", "") != ""
const _DX_OBJCENSUS   = get(ENV, "JM_OBJCENSUS", "") != ""
const _DX_CLASSICLAT_WHY  = get(ENV, "JM_AI_CLASSICLAT_WHY", "0") != "0"
const _DX_LINE_Y      = haskey(ENV, "JM_LINE_Y")
const _DX_PACEDIAG    = haskey(ENV, "JM_PACEDIAG")
const _DX_PIT_DIAG    = get(ENV, "JM_PIT_DIAG", "") != ""
const _DX_AI_TEST     = haskey(ENV, "JM_AI_TEST")
const _DX_JOYTRACE    = get(ENV, "JM_JOYTRACE", "") != ""
const _DX_SHOTS       = !isempty(SHOTS_AT)

include(joinpath(@__DIR__, "main_diag.jl"))
include(joinpath(@__DIR__, "main_loop.jl"))
const _MAIN_Q = quote
function main()
    tstamp("  [E80] .. main() entered (its compile is the gap above)")
    REPLAY_FILE = REPLAY_FILE_R[]

    if get(ENV, "JM_FREECHECK", "0") != "0"

        get(ENV, "JM_FREECHECK_CONTROL", "0") != "0" &&
            include_string(Main, "function _freecheck_control(x)\n    [y + _freecheck_no_such_name for y in x]\nend\n", joinpath(@__DIR__, "main_loop.jl"))

        function _freenames(m::Method, miss::Set{Symbol}, seen::Set{Any})
            m in seen && return miss; push!(seen, m)
            ci = try; Base.uncompressed_ast(m); catch; return miss; end
            _walk(x) = if x isa GlobalRef
                    if !isdefined(x.mod, x.name); push!(miss, x.name)
                    else
                        v = getfield(x.mod, x.name)
                        if v isa DataType && v <: Function && startswith(String(x.name), "#")
                            for mm in Base._methods_by_ftype(Tuple{v, Vararg{Any}}, -1, Base.get_world_counter())
                                _freenames(mm.method, miss, seen)
                            end
                        end
                    end
                elseif x isa Expr; foreach(_walk, x.args)
                elseif x isa Core.ReturnNode && isdefined(x, :val); _walk(x.val)
                end
            foreach(_walk, ci.code)
            miss
        end
        bad = Base.invokelatest() do
            nb = 0
            for f in (getfield(Main, n) for n in names(Main; all = true) if isdefined(Main, n))
                f isa Function || continue
                for m in methods(f)
                    file = String(m.file)
                    (endswith(file, "main_loop.jl") || endswith(file, "main_diag.jl")) || continue
                    miss = setdiff(_freenames(m, Set{Symbol}(), Set{Any}()), (:Profile,))
                    isempty(miss) || (nb += 1; println("  [freecheck] ", m.name, " (", basename(file), ":", m.line, "): undefined ", join(sort(collect(miss)), ", ")))
                end
            end
            nb
        end

        shadow = Base.invokelatest() do
            sh = Symbol[]
            for m in methods(main)
                ci = try; Base.uncompressed_ast(m); catch; nothing; end
                ci === nothing && continue
                for nm in unique(ci.slotnames)
                    st = String(nm); (isempty(st) || startswith(st, "#")) && continue
                    isdefined(Main, nm) && (v = getfield(Main, nm); v isa Function || v isa Type || isconst(Main, nm)) && push!(sh, nm)
                end
            end
            sh
        end
        println("  [freecheck] main() locals shadowing a global function/const: ", isempty(shadow) ? "none" : join(sort(shadow), ", "))
        println("  [freecheck] ", bad == 0 ? "OK: every moved function's globals exist" : "$(bad) function(s) name undefined globals")
        flush(stdout); exit(bad == 0 ? 0 : 1)
    end
    let c = Render.clamp_untiled_textures!()
        println("  textures clamped to edge on untiled axes: S ", c[1], ", T ", c[2], " of ", length(Render.UVRANGE))
    end

    PLAYER_G = Ref(NaN)

    GROUND_SMOOTH = parse(Float64, get(ENV, "JM_GROUND_SMOOTH", "1.0"))
    PLAYER_HDG = Ref(NaN)
    function ground_smoothed(x, z, g0)
        (GROUND_SMOOTH <= 0.0 || !isfinite(PLAYER_HDG[]) || !(g0 > -900f0) || SKIDPAD) && return g0
        hx = cos(PLAYER_HDG[]); hz = sin(PLAYER_HDG[])
        acc = Float64(g0); nacc = 1
        for d in (-2.0, -1.0, 1.0, 2.0)
            h = JRTrack.hat3d(TERRAIN, x + hx*d*GROUND_SMOOTH, z + hz*d*GROUND_SMOOTH; ref=Inf)
            h[3] || return g0
            acc += Float64(h[1]); nacc += 1
        end
        Float32(acc / nacc)
    end
    STEP_GUARD_ON = get(ENV, "JM_STEP_GUARD", "1") != "0"
    STEP_GUARD_HITS = Ref(0)
    function groundz_phys(x, y)
        g = groundz(x, y)
        g > -900f0 || return NaN32
        g = ground_smoothed(x, y, g)

        if TRK_CAL.on && !SKIDPAD
            hr = JRTrack.hat(TRKSURF_G, x, y)

            hl = GROUND_LAT_SMOOTH ? hr : JRTrack.hat(TRKSURF, x, y)
            if hr.found && hl.found
                al = abs(hl.lateral); _ld = ground_ribbon_ld(hr.lapdist)
                if al < TRK_ROAD_LAT + TRK_BLEND
                    ht = trk_road(_ld, hl.lateral)
                    w = smoothblend((TRK_ROAD_LAT + TRK_BLEND - al) / TRK_BLEND) * trk_conf(_ld, hl.lateral)

                    gm = Float64(g)
                    if w < 0.999
                        hd = JRTrack.hat3d(TERRAIN, x, y; ref = ht + 3.0)
                        hd[3] && (gm = ground_smoothed(x, y, Float32(hd[1])))
                    end
                    g = Float32(w*ht + (1-w)*gm)
                end
            end
        end
        gf = Float64(g)
        ok, held = STEP_GUARD_ON ? StepGuard.step_guard(gf, PLAYER_G[], WALL_CLIMB) : (true, gf)
        if !ok
            STEP_GUARD_HITS[] += 1
            STEP_GUARD_HITS[] <= 5 && println("  ⚠ step guard: ground jumped ", round(gf - PLAYER_G[], digits=2),
                                              " m at (", round(x, digits=1), ", ", round(y, digits=1),
                                              ") -- holding ", round(PLAYER_G[], digits=2), " m (JM_STEP_GUARD=0 disables)")
            return Float32(held)
        end
        PLAYER_G[] = gf
        Float32(gf)
    end

    groundz_ai(x, y) = (r = ground_road(x, y); r[2] ? Float32(r[1]) : Float32(objground_at(x, y)))
    cs0 = SKIDPAD ? (x=0.0, z=0.0, θ=0.0) : spawn(CAR; v0=0.0)
    LASTZ = Ref(0.0); ONTRACK = Ref(true)
    LASTGX = Ref(cs0.x); LASTGZ = Ref(cs0.z)
    OFFDIST = Ref(0.0)
    PLAYER_CDA = Ref(1.0)
    BND_FX = Ref(0.0); BND_FY = Ref(0.0); BND_MZ = Ref(0.0)

    WALL_CLIMB = parse(Float64, get(ENV, "JM_WALL_CLIMB", "3.0"))
    function groundz(x, z; acquire = false)
        SKIDPAD && return 0.0
        h = JRTrack.hat3d(TERRAIN, x, z; ref=Inf)

        if h[3] && (acquire || !MONZA || Float64(h[1]) <= LASTZ[] + WALL_CLIMB)
            LASTZ[] = Float64(h[1]); ONTRACK[] = true
        else
            ONTRACK[] = false
        end
        LASTZ[]
    end
    y0spawn = groundz(cs0.x, cs0.z; acquire=true)

    θ0spawn = if SKIDPAD
        cs0.θ
    else

        base = min(2, length(ALIGNED)-1); j = base; acc = 0.0
        while acc < 8.0 && j < length(ALIGNED)
            acc += hypot(ALIGNED[j+1][1]-ALIGNED[j][1], ALIGNED[j+1][2]-ALIGNED[j][2]); j += 1
        end
        atan(ALIGNED[j][2]-ALIGNED[base][2], ALIGNED[j][1]-ALIGNED[base][1])
    end

    if FUEL_MODE != "legacy" && !SKIDPAD && DriveRT3D.CORNER_W[] !== nothing && isfinite(FUEL_SETUP_L[])
        FUEL_START_L[] = fuel_start_litres(LAPLEN)
        w = DriveRT3D.CORNER_W[]
        cw = DriveRT3D.corner_weights_with_fuel(Dict(:LF => w[1], :RF => w[2], :LR => w[3], :RR => w[4]), FUEL_SETUP_L[], FUEL_START_L[])
        DriveRT3D.set_corner_weights!(cw; source = "setup + fuel $(round(FUEL_START_L[], digits = 1)) L ($FUEL_MODE)")
        println("  fuel:    ", round(FUEL_START_L[], digits = 1), " L (", FUEL_MODE, ")  -> mass ", round(DriveRT3D.MASS[], digits = 1),
                " kg, front ", round(100DriveRT3D.FRONT_FRAC[], digits = 1), " %   (setup ibt ", FUEL_SETUP_L[], " L)")
    end
    tstamp("physics build (mtkcompile) begins")

    CAR3D && (DriveRT3D.set_car!(net_early_car(PLAYER_CAR)); println("  car:     ", DriveRT3D.car_describe()))
    cs = build_carX(x0=cs0.x, z0=cs0.z, θ0=θ0spawn, v0=0.0, y0=y0spawn)
    if CAR3D
        DriveRT3D.set_air!(cs, DriveRT3D.air_density(TRACKSEL))
        DriveRT3D.FUEL_RHO[] = DriveRT3D.air_density(TRACKSEL)
        println("  air:     ", round(DriveRT3D.air_density(TRACKSEL), digits = 3), " kg/m3 (", TRACKSEL, ")")
    end
    tstamp("physics build done — game loop imminent")

    if get(ENV, "JM_MTK_TWICE", "0") != "0"
        _t2 = time()
        build_carX(x0=cs0.x, z0=cs0.z, θ0=θ0spawn, v0=0.0, y0=y0spawn)
        println("[E80] second build_carX: ", round(time()-_t2, digits=1), " s",
                "   (first was the phase above; a fast second call means COMPILATION, not work)")
        flush(stdout)
    end

    CLINE  = !SKIDPAD ? RaceAI.build_line(CLASSICTrack.defold(ALIGNED; label = "AI rail"),
                                          (x, z) -> (r = ground_road(x, z); r[2] ? r[1] : NaN)) : nothing
    HUD_MAP_ON && CLINE !== nothing && trackmap_build!(TRACKMAP, CLINE.x, CLINE.z, CLINE.s, CLINE.total)

    if _DX_GROUNDPROF && CLINE !== nothing
        let f = split(ENV["JM_GROUNDPROF"], ":"), s0 = parse(Float64, f[1]), s1 = parse(Float64, f[2]), la = parse(Float64, f[3])
            println("  [groundprof] s  x  z  | ribbon lapdist lateral | spline corr conf | ground  d2(ground) mm")
            prev = (NaN, NaN)
            for s in s0:0.25:s1
                _a = 10.0*floor(s/10.0); _f = (s - _a)/10.0
                pa = RaceAI.pose_at(CLINE, _a, la); pb = RaceAI.pose_at(CLINE, _a + 10.0, la)
                x = pa[1] + _f*(pb[1] - pa[1]); z = pa[3] + _f*(pb[3] - pa[3])
                hr = JRTrack.hat(TRKSURF, x, z)
                sp = hr.found ? CLASSICTrack.trk_height(TRKALT, hr.lapdist * TRK_SSCALE, TRK_CAL.sign * hr.lateral) + TRK_CAL.off : NaN
                co = hr.found ? trk_corr(hr.lapdist, hr.lateral) : NaN
                cf = hr.found ? trk_conf(hr.lapdist, hr.lateral) : NaN
                PLAYER_G[] = NaN; g = Float64(groundz_phys(x, z))
                d2 = isnan(prev[1]) ? NaN : (g - 2prev[2] + prev[1]) * 1000
                println("  [groundprof] ", round(s, digits = 2), "  ", round(x, digits = 2), " ", round(z, digits = 2), " | ",
                        hr.found ? round(hr.lapdist, digits = 2) : NaN, " ", hr.found ? round(hr.lateral, digits = 2) : NaN, " | ",
                        round(sp, digits = 4), " ", round(co, digits = 4), " ", round(cf, digits = 3), " | ", round(g, digits = 4), "  ", round(d2, digits = 2))
                prev = (prev[2], g)
            end
        end
        exit(0)
    end

    if CLASSICW_ON && CLINE !== nothing && haskey(ENV, "JM_CLASSICWPROF")
        let f = split(ENV["JM_CLASSICWPROF"], ":"), s0 = parse(Float64, f[1]), s1 = parse(Float64, f[2])
            println("  [classicwprof] s  x  z | Classic section f lat | road interval | walls around the point | autodrive offset -> target lat, walls")
            _ado = autodrive_offsets(CLINE)
            _adtgt(s) = (o = _ado[2][mod1(round(Int, s / _ado[1]) + 1, length(_ado[2]))]; q = RaceAI.pose_at(CLINE, s, o);
                         l = classicw_locate_any(q[1], q[3]); l === nothing ? "" : (w = CLASSICWall.free_interval(CLASSICWALLS, l[1], l[2], l[3], l[3]; hmin = CLASSICW_HMIN);
                         string(" | ", round(o, digits = 2), " -> ", round(l[3], digits = 2), " [", round(w[1], digits = 2), ", ", round(w[2], digits = 2), "]")))
            for s in s0:5.0:s1
                p = RaceAI.pose_at(CLINE, s, 0.0); loc = classicw_locate_any(p[1], p[3])
                loc === nothing && (println("  [classicwprof] ", s, "  not located"); continue)
                (sec, fr, lat, _, _) = loc
                (rlo, rhi, _, _) = classicw_road_interval(sec, fr)
                (wlo, whi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, sec, fr, lat, lat; hmin = CLASSICW_HMIN)
                println("  [classicwprof] ", s, "  ", round(p[1], digits = 1), " ", round(p[3], digits = 1), " | ", sec, " ",
                        round(fr, digits = 3), " ", round(lat, digits = 2), " | [", round(rlo, digits = 2), ", ", round(rhi, digits = 2),
                        "] | [", round(wlo, digits = 2), ", ", round(whi, digits = 2), "]", _adtgt(s))
            end
        end
        exit(0)
    end
    CLINE !== nothing && println("  CLINE: centreline length = ", round(Int, CLINE.total), " m  (", TRACKSEL, ")")
    if AUTODRIVE && AUTODRIVE_ROAD && CLASSICW_ON && CLINE !== nothing
        AD_OFF[] = autodrive_offsets(CLINE)
        ADLINE[] = autodrive_line(CLINE, AD_OFF[], (x, z) -> (r = ground_road(x, z); r[2] ? r[1] : NaN))
        let o = AD_OFF[][2], st = AD_OFF[][1], m = findall(x -> abs(x) > 0.1, o)
            println("  [autodrive] line kept on Classic's road: ", round(Int, st*length(m)), " m shifted",
                    isempty(m) ? "" : string(", max ", round(maximum(abs.(o)), digits = 2), " m at s ", round(Int, st*(argmax(abs.(o)) - 1))),
                    " (JM_AUTODRIVE_ROAD=0: CLINE as is)")
        end
    end

    if TRACE_DS && CLINE !== nothing
        nb = max(1, ceil(Int, CLINE.total/50))
        bins = zeros(Int, nb)
        for sv in CLINE.s
            bins[clamp(floor(Int, sv/50)+1, 1, nb)] += 1
        end
        println("[vtx] centreline vertices per 50 m over ", round(Int, CLINE.total), " m, ",
                length(CLINE.s), " vertices total")
        for b in 1:nb
            lo = (b-1)*50
            mark = (1700 <= lo <= 1900) ? "   <-- wreck band" : ""
            println("[vtx]   s ", lpad(lo,5), "..", lpad(lo+49,5), "  n=", lpad(bins[b],4), mark)
        end
        flush(stdout)
    end

    _DX_HOLECENSUS && CLINE !== nothing && dx_holecensus(CLINE)
    if CLINE !== nothing && haskey(ENV,"JM_LATDIAG")
        pl = RaceAI.project(CLINE, cs.x, cs.z); ph = JRTrack.hat(TRKSURF, cs.x, cs.z)
        println("  LATDIAG spawn: CLINE lat=", round(pl[2],digits=2), "  TRKSURF found=", ph.found,
                " lat=", round(ph.lateral,digits=2), " on_track=", ph.on_track, " (ROAD_HALFW=", ROAD_HALFW, ")")
        for d in 5.0:5.0:40.0
            sp = RaceAI.pose_at(CLINE, RaceAI.project(CLINE, cs.x, cs.z)[1]+d, 0.0)
            ph2 = JRTrack.hat(TRKSURF, sp[1], sp[3]); pl2 = RaceAI.project(CLINE, sp[1], sp[3])
            println("    +", Int(d), "m on centreline: CLINE lat=", round(pl2[2],digits=2), "  TRKSURF lat=", round(ph2.lateral,digits=2), " on_track=", ph2.on_track)
        end
        flush(stdout)
    end

    place_at_s! = function (s0raw::Float64, lat::Float64 = 0.0)
        (CLINE === nothing || !CAR3D) && return nothing
        s0 = clamp(s0raw, 0.0, CLINE.total)

        PLAYER_G[] = NaN
        CLASSICW_ON && classicw_teleported!()
        p  = RaceAI.pose_at(CLINE, s0, lat)
        DriveRT3D.place3d!(cs, p[1], p[3], p[4]; v = 0.0)
        cs.s_vreset(cs.integ, zeros(14))
        h = groundz(p[1], p[3]; acquire=true); isfinite(h) && (cs.zref = Float64(h))

        if ROADHAT !== TERRAIN0
            rh = JRTrack.hat3d(ROADHAT, p[1], p[3]; ref = Inf)
            if rh[3] && (!isfinite(h) || abs(Float64(rh[1]) - Float64(h)) > 0.3)
                cs.zref = Float64(rh[1]); PLAYER_G[] = cs.zref
                println("  place_at_s!: full HAT gave ", round(Float64(h), digits = 2), " m, road surface ", round(Float64(rh[1]), digits = 2), " m -- using the road")
            end
        end
        cs.heave = 0.0; cs.pitch = 0.0; cs.roll = 0.0; cs.y = cs.zref
        p
    end
    if _DX_START_S && CLINE !== nothing && CAR3D
        s0 = clamp(parse(Float64, ENV["JM_START_S"]), 0.0, CLINE.total)

        _lane = get(ENV, "JM_START_LANE", "") == "race" ? RaceAI.racelane(CLINE, s0) : 0.0
        haskey(ENV, "JM_START_LAT") && (_lane = parse(Float64, ENV["JM_START_LAT"]))
        p  = place_at_s!(s0, _lane)
        haskey(ENV, "JM_START_LAT") && println("  [offband] at lat ", _lane, ": TERRAIN ", (h_ = JRTrack.hat3d(TERRAIN, p[1], p[3]; ref = Inf); h_[3] ? round(Float64(h_[1]), digits = 2) : "none"),
                                               ", object ground ", round(objground_at(p[1], p[3]), digits = 2), ", the car's y ", round(cs.y, digits = 2))
        println("  JM_START_S: car placed at s=", round(Int, s0), " m, ", _lane == 0.0 ? "on the centreline" :
                "on the racing line (lat $(round(_lane, digits = 2)) m)", " (x=", round(Int, p[1]), " z=", round(Int, p[3]), ")")
        if get(ENV,"JM_OBJDIAG","")!="" && isdefined(Main,:OBJINSTS)
            rad = parse(Float64, get(ENV,"JM_SPOT_RAD","70"))
            near = NTuple{3,Any}[]
            for (nm,ox,oz,oy,kind,issolid) in OBJINSTS
                d = hypot(Float64(ox)-p[1], Float64(oz)-p[3])
                d < rad && push!(near, (round(d,digits=1), "$nm $(kind)$(issolid ? "·SOLID" : "")", round(Float64(oy),digits=1)))
            end
            sort!(near, by=x->x[1])
            println("  JM_SPOT objects within ", round(Int,rad), " m of the car (d  name kind  base-y):")
            for (d,lbl,oy) in near; println("     d=", rpad(d,7), rpad(lbl,22), " y=", oy); end

            road_h = JRTrack.hat3d(TERRAIN, p[1], p[3]; ref=Inf)[1]
            println("  JM_HATSCAN ahead (s  top=ref∞  follow=road+3):  road_h@car=", round(road_h,digits=1))
            for ds in 0.0:10.0:140.0
                sp = RaceAI.pose_at(CLINE, min(s0+ds, CLINE.total), 0.0)
                ht = JRTrack.hat3d(TERRAIN, sp[1], sp[3]; ref=Inf)
                hf = JRTrack.hat3d(TERRAIN, sp[1], sp[3]; ref=road_h+3.0)
                println("     +", rpad(Int(ds),4), "m  top=", rpad(ht[3] ? round(ht[1],digits=1) : "MISS", 8),
                        "follow=", rpad(hf[3] ? round(hf[1],digits=1) : "MISS", 8),
                        (ht[3] && hf[3] && ht[1]-hf[1] > 1.5) ? "  <== OVERPASS SPIKE" : "")
            end

            function cover_tex(qx, qz)
                out = Tuple{String,Float64}[]
                for t in TRACKMESH.tris
                    ax,az = Float64(t.p[1][1]), Float64(t.p[1][2]); bx,bz = Float64(t.p[2][1]), Float64(t.p[2][2]); cx,cz = Float64(t.p[3][1]), Float64(t.p[3][2])
                    d = (bz-cz)*(ax-cx)+(cx-bx)*(az-cz); abs(d) < 1e-9 && continue
                    wa = ((bz-cz)*(qx-cx)+(cx-bx)*(qz-cz))/d; wb = ((cz-az)*(qx-cx)+(ax-cx)*(qz-cz))/d; wc = 1-wa-wb
                    (wa>=-0.01 && wb>=-0.01 && wc>=-0.01) || continue
                    h = wa*Float64(t.p[1][3])+wb*Float64(t.p[2][3])+wc*Float64(t.p[3][3])
                    push!(out, (t.tex, round(h,digits=1)))
                end
                sort!(out, by=x->-x[2]); out
            end
            for ds in (15.0, 120.0)
                sp = RaceAI.pose_at(CLINE, min(s0+ds,CLINE.total), 0.0)
                println("  JM_COVER s+", Int(ds), " (x=", round(Int,sp[1]), " z=", round(Int,sp[3]), "): ",
                        join([string(tx=="" ? "<none>" : tx, "@", h) for (tx,h) in cover_tex(sp[1], sp[3])], "  "))
            end
            LASTZ[] = road_h
            println("  JM_DRIVETEST follow-ref groundz stepping forward from s=", round(Int,s0), ":")
            for ds in 0.0:5.0:160.0
                sp = RaceAI.pose_at(CLINE, min(s0+ds, CLINE.total), 0.0)
                hrb = JRTrack.hat(TRKSURF, sp[1], sp[3])
                g = groundz(sp[1], sp[3])
                println("     +", rpad(Int(ds),4), "m  groundz=", rpad(round(g,digits=1),7),
                        " ribbon=", rpad(hrb.found ? round(hrb.height,digits=1) : "MISS",7),
                        " lat=", rpad(round(hrb.lateral,digits=1),6), " ontrack=", ONTRACK[])
            end
            flush(stdout)
        end
    end

    OFFTRACK_PROBE = Ref(0)
    if _DX_OFFTRACK && CLINE !== nothing && CAR3D
        pf = [parse(Float64, x) for x in split(ENV["JM_OFFTRACK_PROBE"], ":")]
        ps, plat, pkmh, pdeg = pf[1], pf[2], pf[3], pf[4]
        pslip = length(pf) >= 5 ? pf[5] : 0.0
        p0 = place_at_s!(clamp(ps, 0.0, CLINE.total))
        θ0 = p0[4]; nLx = -sin(θ0); nLz = cos(θ0)
        px = p0[1] + nLx*plat; pz = p0[3] + nLz*plat
        DriveRT3D.place3d!(cs, px, pz, θ0 + deg2rad(pdeg); v = pkmh/3.6)
        pslip != 0.0 && cs.s_vel(cs.integ, [pkmh/3.6*cos(deg2rad(pslip)), pkmh/3.6*sin(deg2rad(pslip))])
        cs.s_vreset(cs.integ, zeros(14))
        hP = groundz(px, pz; acquire=true); isfinite(hP) && (cs.zref = Float64(hP)); cs.y = cs.zref
        PLAYER_G[] = NaN
        OFFTRACK_PROBE[] = parse(Int, get(ENV, "JM_OFFTRACK_FRAMES", "300"))
        println("  JM_OFFTRACK_PROBE: car placed at s=", round(Int, ps), " lat=", plat, " m, heading ",
                pdeg, " deg off the line, ", pkmh, " km/h (x=", round(px, digits=1), " z=", round(pz, digits=1),
                " ground=", isfinite(hP) ? round(Float64(hP), digits=2) : NaN, ")")
    end

    _DX_BRIDGEPROBE && CLINE !== nothing && dx_bridgeprobe(CLINE)
    if _DX_SWEEP && CLINE !== nothing && isdefined(Main,:OBJINSTS)
        olat(ox,oz) = (hr = JRTrack.hat(TRKSURF, Float64(ox), Float64(oz)); hr.found ? round(hr.lateral,digits=1) : 999.0)
        step = (v = tryparse(Float64, get(ENV,"JM_SWEEP","")); v === nothing || v < 1 ? 15.0 : v)

        BLOCK_LAT = ROAD_HALFW - 1.0

        OBJ_MAX_DY = parse(Float64, get(ENV,"JM_OBJ_MAX_DY","3.0"))
        objdy(ox,oz,oy) = (rh = JRTrack.hat3d(TERRAIN, Float64(ox), Float64(oz); ref=Inf); rh[3] ? round(Float64(oy)-rh[1],digits=1) : 0.0)
        kept_mesh  = [(nm,ox,oz,olat(ox,oz),objdy(ox,oz,oy)) for (nm,ox,oz,oy,kind,issolid) in OBJINSTS if kind===:mesh]
        kept_bb    = [(nm,ox,oz,olat(ox,oz),objdy(ox,oz,oy)) for (nm,ox,oz,oy,kind,issolid) in OBJINSTS if kind===:bb]

        _slive = Dict{Tuple{Float64,Float64},Float64}()
        for (sx, sz, sr, _) in SOLIDS
            k = (round(Float64(sx), digits=2), round(Float64(sz), digits=2)); _slive[k] = max(get(_slive, k, -Inf), Float64(sr))
        end
        live_r(ox, oz) = get(_slive, (round(Float64(ox), digits=2), round(Float64(oz), digits=2)), -1.0)
        use_tarmac = ROADHAT !== TERRAIN0 && get(ENV, "JM_SWEEP_CORRIDOR", "0") == "0"
        tarmac_at(x, z) = JRTrack.hat3d(ROADHAT, Float64(x), Float64(z); ref=Inf)[3] &&
                          (hr_ = JRTrack.hat(TRKSURF, Float64(x), Float64(z)); hr_.found && hr_.on_track)

        solid_reaches(ox, oz, r) = begin
            R = r + 0.1; n = max(3, ceil(Int, 4R)); hit = false
            for ix in -n:n, iz in -n:n
                hypot(ix, iz) > n && continue
                tarmac_at(ox + R*ix/n, oz + R*iz/n) && (hit = true; break)
            end
            hit
        end

        SWEEP_DEPTH = parse(Float64, get(ENV, "JM_SWEEP_DEPTH", "1.0"))
        tarmac_deep(x, z) = tarmac_at(x, z) && (SWEEP_DEPTH <= 0 ||
            all(((dx, dz),) -> tarmac_at(x + dx, z + dz), ((SWEEP_DEPTH, 0.0), (-SWEEP_DEPTH, 0.0), (0.0, SWEEP_DEPTH), (0.0, -SWEEP_DEPTH))))
        mesh_on_tarmac(nm, ox, oz) = begin
            vs = get(OBJ_LVERTS, nm, nothing)
            (vs === nothing || isempty(vs)) && return tarmac_deep(ox, oz)
            th = get(OBJ_YAW, (round(Float64(ox), digits=2), round(Float64(oz), digits=2)), 0.0); c_, s_ = cos(th), sin(th)
            w(lx, lz) = (Float64(ox) + lx*c_ + lz*s_, Float64(oz) - (-lx*s_ + lz*c_))
            pts = [w(lx, lz) for (lx, lz) in vs]
            if length(pts) <= 16
                for i in 1:length(pts), j in i+1:length(pts)
                    (a, b) = (pts[i], pts[j]); n = ceil(Int, hypot(b[1]-a[1], b[2]-a[2]))
                    for k in 1:n-1; push!(pts, (a[1] + (b[1]-a[1])*k/n, a[2] + (b[2]-a[2])*k/n)); end
                end
            end
            any(p -> tarmac_deep(p[1], p[2]), pts)
        end

        solid_on_tarmac(k) = begin
            (ox, oz, r, _) = SOLIDS[k]; b = k <= length(SOLIDBOX) ? SOLIDBOX[k] : nothing
            R = (b === nothing ? r : hypot(b[1], b[2])) + 0.1; n = max(2, ceil(Int, 4R))
            for ix in -n:n, iz in -n:n
                px_ = ox + R*ix/n; pz_ = oz + R*iz/n
                solid_gap(px_, pz_, k)[1] < 0 && tarmac_at(px_, pz_) && return true
            end
            false
        end
        kept_solid = [(k <= length(SOLIDNAMES) ? SOLIDNAMES[k] : "solid$k", SOLIDS[k][1], SOLIDS[k][2], olat(SOLIDS[k][1], SOLIDS[k][2]),
                       0.0, SOLIDS[k][3], k) for k in eachindex(SOLIDS) if SOLIDS[k][3] > 0]

        solid_onroad = Dict(k => (use_tarmac ? solid_on_tarmac(k) : abs(lat) < ROAD_HALFW) for (nm,ox,oz,lat,dy,r,k) in kept_solid)
        n_inert = count(((nm,ox,oz,oy,kind,issolid),) -> issolid && live_r(ox,oz) <= 0, OBJINSTS)
        println("  census frame: ", use_tarmac ? "tarmac (ROADHAT + on_track)" : "corridor |lat| < ROAD_HALFW-1",
                "  live solids=", length(kept_solid), "  inert by clearance=", n_inert)
        println("\n==== JM_SWEEP ", uppercasefirst(TRACKSEL), "  (step=", round(Int,step),
                " m, total=", round(Int,CLINE.total), " m, ROAD_HALFW=", ROAD_HALFW, ") ====")
        groundz(RaceAI.pose_at(CLINE,0.0,0.0)[1], RaceAI.pose_at(CLINE,0.0,0.0)[3]; acquire=true)
        prevtop = JRTrack.hat3d(TERRAIN, RaceAI.pose_at(CLINE,0.0,0.0)[1], RaceAI.pose_at(CLINE,0.0,0.0)[3]; ref=Inf)
        nclean = 0; anoms = String[]
        s = 0.0
        while s <= CLINE.total
            p = RaceAI.pose_at(CLINE, s, 0.0); px, pz = p[1], p[3]
            top = JRTrack.hat3d(TERRAIN, px, pz; ref=Inf)
            hr  = JRTrack.hat(TRKSURF, px, pz)
            flags = String[]
            top[3] || push!(flags, "OFF-HAT(hole)")
            (top[3] && prevtop[3] && abs(top[1]-prevtop[1]) > 3.0) && push!(flags, "WALL/CLIFF Δh=$(round(top[1]-prevtop[1],digits=1))m")
            (!hr.found || abs(hr.lateral) > ROAD_HALFW) && push!(flags, "FALSE-GRASS lat=$(hr.found ? round(hr.lateral,digits=1) : "MISS")")
            sobs = ["$nm(lat=$lat,r=$(round(r,digits=1)))" for (nm,ox,oz,lat,dy,r,k) in kept_solid
                    if hypot(ox-px, oz-pz) < ROAD_HALFW + r + 8.0 && solid_onroad[k]]
            mobs = ["$nm(lat=$lat,dy=$dy)" for (nm,ox,oz,lat,dy) in kept_mesh if hypot(ox-px, oz-pz) < ROAD_HALFW && dy < OBJ_MAX_DY && (use_tarmac ? mesh_on_tarmac(nm, ox, oz) : abs(lat) < BLOCK_LAT)]
            bobs = ["$nm(lat=$lat,dy=$dy)" for (nm,ox,oz,lat,dy) in kept_bb   if hypot(ox-px, oz-pz) < ROAD_HALFW && dy < OBJ_MAX_DY && (use_tarmac ? tarmac_at(ox, oz) : abs(lat) < BLOCK_LAT)]
            isempty(sobs) || push!(flags, "SOLID-ON-ROAD(collidable!): " * join(unique(sobs)[1:min(end,4)], ","))
            isempty(mobs) || push!(flags, "ON-ROAD MESH: " * join(unique(mobs)[1:min(end,4)], ","))
            isempty(bobs) || push!(flags, "ON-ROAD BILLBOARD: " * join(unique(bobs)[1:min(end,4)], ","))
            if isempty(flags); nclean += 1
            else; push!(anoms, string("  s=", lpad(round(Int,s),5), "  ", join(flags, " | ")))
            end
            prevtop = top; s += step
        end
        println(length(anoms), " anomaly point(s), ", nclean, " clean:")
        for a in anoms; println(a); end
        flush(stdout); exit(0)
    end

    _DX_ROADSWEEP && CLINE !== nothing && dx_roadsweep(CLINE)

    _DX_ROADTRIS && dx_roadtris()

    _DX_ROADEDGE && dx_roadedge()

    _DX_OBJCENSUS && dx_objcensus()
    AILINE = (CLINE !== nothing && N_AI > 0) ? CLINE : nothing

    if AILINE !== nothing && get(ENV, "JM_AI_CLASSICLINE", "1") != "0"

        if MONZA || ZANDV || get(ENV, "JM_AI_CLASSICLINE_ALL", "1") != "0"
            lp = joinpath(ZD, "race.lp")
            if isfile(lp)
                gv = CLASSICLP.lp_speed_mps(CLASSICLP.read_lp(lp))
                ini = joinpath(ZD, "track.ini"); adj = 1.0; vcap = 2.41*36.0
                if isfile(ini)
                    for l in eachline(ini)
                        m = match(r"^\s*dlong_speed_adj_coeff\s*=\s*([0-9.]+)", l); m !== nothing && (adj = parse(Float64, m[1]))
                        m = match(r"^\s*dlong_speed_maximum\s*=\s*([0-9.]+)", l);   m !== nothing && (vcap = parse(Float64, m[1])*36.0)
                    end
                end
                RaceAI.set_classic_speeds!(min.(gv .* adj, vcap))

                if get(ENV, "JM_AI_CLASSICLAT", "1") != "0"
                    lp1 = joinpath(ZD, "pass1.lp"); lp2 = joinpath(ZD, "pass2.lp")
                    if isfile(lp1) && isfile(lp2)
                        gr = Float64.(CLASSICLP.read_lp(lp).dlat)
                        g1 = Float64.(CLASSICLP.read_lp(lp1).dlat)
                        g2 = Float64.(CLASSICLP.read_lp(lp2).dlat)
                        nrec = length(gr)
                        sgn = TRK_CAL.on ? TRK_CAL.sign : 1.0

                        A0 = let ref = get(ENV, "JM_CLASSICREF", "rigid")
                            if ref == "recentred"
                                ALIGNED
                            elseif ref == "rigid"
                                rigid_refit(ALIGNED0, ALIGNED)
                            else
                                ALIGNED0
                            end
                        end
                        println("  AI racing LINE reference (JM_CLASSICREF): ", get(ENV, "JM_CLASSICREF", "rigid"))
                        nA = length(A0)
                        sA = zeros(nA)
                        for k in 2:nA; sA[k] = sA[k-1] + hypot(A0[k][1]-A0[k-1][1], A0[k][2]-A0[k-1][2]); end
                        lenA = sA[end] + hypot(A0[1][1]-A0[end][1], A0[1][2]-A0[end][2])
                        function classic_to_ours(dlat, i)
                            t = (i-1)/nrec * lenA
                            k = clamp(searchsortedlast(sA, t), 1, nA); j = k % nA + 1
                            f = (t - sA[k]) / max((j == 1 ? lenA : sA[j]) - sA[k], 1e-6)
                            px = A0[k][1] + (A0[j][1]-A0[k][1])*f; pz = A0[k][2] + (A0[j][2]-A0[k][2])*f
                            tx = A0[j][1]-A0[k][1]; tz = A0[j][2]-A0[k][2]; tl = max(hypot(tx,tz), 1e-9)
                            nx = -tz/tl; nz = tx/tl
                            d = sgn*dlat
                            hr = JRTrack.hat(TRKSURF, px + nx*d, pz + nz*d)
                            hr.found ? hr.lateral : d
                        end
                        corr(v) = [classic_to_ours(v[i], i) for i in 1:min(nrec, length(v))]

                        function classic_world(dlat, i)
                            t = (i-1)/nrec * lenA
                            k = clamp(searchsortedlast(sA, t), 1, nA); j = k % nA + 1
                            f = (t - sA[k]) / max((j == 1 ? lenA : sA[j]) - sA[k], 1e-6)
                            px = A0[k][1] + (A0[j][1]-A0[k][1])*f; pz = A0[k][2] + (A0[j][2]-A0[k][2])*f
                            tx = A0[j][1]-A0[k][1]; tz = A0[j][2]-A0[k][2]; tl = max(hypot(tx,tz), 1e-9)
                            d = sgn*dlat
                            (px - tz/tl*d, pz + tx/tl*d)
                        end
                        function onroad_frac(v)
                            m = min(nrec, length(v)); m == 0 && return (1.0, 0)
                            n_off = 0
                            for i in 1:m
                                q = classic_world(v[i], i)
                                JRTrack.hat3d(ROADHAT, q[1], q[2]; ref = Inf)[3] || (n_off += 1)
                            end
                            (1.0 - n_off/m, n_off)
                        end

                        function onroad_car_frac(v)
                            m = min(nrec, length(v)); m == 0 && return (1.0, 0.0)
                            n_off = 0; dmax = 0.0
                            for i in 1:m
                                q = classic_world(v[i], i)
                                JRTrack.hat3d(ROADHAT, q[1], q[2]; ref = Inf)[3] && continue
                                d = 2.0
                                for r in 0.05:0.05:2.0
                                    if any(a -> JRTrack.hat3d(ROADHAT, q[1] + r*cos(a), q[2] + r*sin(a); ref = Inf)[3], 0:pi/12:2pi-0.01)
                                        d = r; break
                                    end
                                end
                                dmax = max(dmax, d); d > CARW && (n_off += 1)
                            end
                            (1.0 - n_off/m, dmax)
                        end

                        lpmn = joinpath(ZD, "minrace.lp"); lpmx = joinpath(ZD, "maxrace.lp")
                        bnd_lo = nothing; bnd_hi = nothing
                        if isfile(lpmn) && isfile(lpmx)
                            try
                                bnd_lo = corr(Float64.(CLASSICLP.read_lp(lpmn).dlat))
                                bnd_hi = corr(Float64.(CLASSICLP.read_lp(lpmx).dlat))
                            catch e
                                println("  AI racing LINE: minrace/maxrace unreadable (", sprint(showerror, e),
                                        ") -- keeping the fixed band")
                                bnd_lo = nothing; bnd_hi = nothing
                            end
                        end
                        cr, c1, c2 = corr(gr), corr(g1), corr(g2)

                        if _DX_CLASSICLAT_WHY && ROADHAT !== TERRAIN0
                            _tex = Dict{String,Int}(); _none = Ref(0); _n = Ref(0)
                            for i in 1:nrec
                                q = classic_world(Float64(CLASSICLP.read_lp(lp).dlat[i]), i)
                                JRTrack.hat3d(ROADHAT, q[1], q[2]; ref = Inf)[3] && continue
                                _n[] += 1
                                best = ""; bd = Inf
                                for t in TRACKMESH.tris
                                    cx = (t.p[1][1]+t.p[2][1]+t.p[3][1])/3
                                    cy = (t.p[1][2]+t.p[2][2]+t.p[3][2])/3
                                    d2 = (cx-q[1])^2 + (cy-q[2])^2
                                    d2 < bd && (bd = d2; best = t.tex)
                                end
                                bd > 400.0 ? (_none[] += 1) : (_tex[lowercase(best)] = get(_tex, lowercase(best), 0) + 1)
                            end

                            let _d = Float64[]
                                for i in 1:nrec
                                    q = classic_world(Float64(CLASSICLP.read_lp(lp).dlat[i]), i)
                                    JRTrack.hat3d(ROADHAT, q[1], q[2]; ref = Inf)[3] && continue
                                    lo = Inf
                                    for r in -20.0:0.25:20.0, ax in 1:2
                                        p2 = ax == 1 ? (q[1]+r, q[2]) : (q[1], q[2]+r)
                                        JRTrack.hat3d(ROADHAT, p2[1], p2[2]; ref = Inf)[3] && (lo = min(lo, abs(r)))
                                    end
                                    push!(_d, lo)
                                end
                                if !isempty(_d)
                                    u = sort(_d); m = length(u)
                                    println("  AI racing LINE [why] distance from a rejected record to the nearest road: p50 ",
                                            round(u[m÷2], digits=2), " p90 ", round(u[max(1,9m÷10)], digits=2),
                                            " max ", round(u[end], digits=2), " m  (", count(isinf, _d),
                                            " with none within 20 m)")
                                end
                            end
                            println("  AI racing LINE [why] ", _n[], " records rejected by ROADHAT; nearest track-mesh texture:")
                            for (k, v) in sort(collect(_tex), by = x -> -x[2])[1:min(10, end)]
                                println("     ", rpad(k, 12), v, "  (", round(100v/max(_n[],1), digits=1), "%)")
                            end
                            _none[] > 0 && println("     (", _none[], " with no track triangle within 20 m)")
                        end
                        _tol = parse(Float64, get(ENV, "JM_AI_CLASSICLAT_TOL", "0.02"))
                        _refuse = false

                        _rd = 1.0
                        if ROADHAT !== TERRAIN0
                            _rd = onroad_frac(Float64.(CLASSICLP.read_lp(lp).dlat))[1]
                            _rdtol = parse(Float64, get(ENV, "JM_AI_CLASSICROAD_TOL", "0.95"))
                            if _rd < _rdtol && get(ENV, "JM_AI_CLASSICROAD_GRADED", "1") != "0"
                                _rc, _dmx = onroad_car_frac(Float64.(CLASSICLP.read_lp(lp).dlat))
                                println("  AI racing LINE graded on-road test: centre on road ", round(100*_rd, digits=1),
                                        "%, car body overlapping road ", round(100*_rc, digits=1),
                                        "% (centre within CARW ", CARW, " m), furthest off-road centre ",
                                        _dmx >= 2.0 ? ">= 2" : round(_dmx, digits=2), " m (bar ", round(100*_rdtol, digits=1),
                                        "%; JM_AI_CLASSICROAD_GRADED=0 reverts to the centre-only test)")
                                _rc >= _rdtol && (_rd = _rc)
                            end
                            if _rd < _rdtol
                                _refuse = true
                                println("  AI racing LINE: Classic's line REFUSED on ", TRACKSEL, " -- only ",
                                        round(100*_rd, digits=1), "% of the mapped line lands on the road-only",
                                        " mesh (bar ", round(100*_rdtol, digits=1), "%). The mapping, not Classic's",
                                        " data: Classic's own minrace/maxrace bracket its race line, so the line is",
                                        " sound and our reconciliation is what puts it on the grass.",
                                        " JM_AI_CLASSICROAD_TOL lowers the bar, JM_AI_CLASSICLAT=0 disables outright.")
                            end
                        end
                        if bnd_lo !== nothing && bnd_hi !== nothing
                            _m = min(length(cr), length(bnd_lo), length(bnd_hi))
                            _in = count(k -> min(bnd_lo[k], bnd_hi[k]) - 1e-6 <= cr[k] <= max(bnd_lo[k], bnd_hi[k]) + 1e-6, 1:_m)
                            _out = 1.0 - _in/_m
                            _btol = parse(Float64, get(ENV, "JM_AI_CLASSICBAND_TOL", "0.05"))
                            println("  AI racing LINE: mapped race line inside Classic's own mapped corridor at ",
                                    _in, " of ", _m, " records (", round(100*_in/_m, digits=1), "%); bar ",
                                    round(100*(1-_btol), digits=1), "%")
                            if _out > _btol
                                _refuse = true
                                println("  AI racing LINE: Classic's line REFUSED on ", TRACKSEL,
                                        " -- the mapping does not preserve Classic's own corridor (",
                                        round(100*_out, digits=1), "% of records fall outside it, bar ",
                                        round(100*_btol, digits=1), "%); keeping our own racing line.",
                                        " JM_AI_CLASSICBAND_TOL raises the bar, JM_AI_CLASSICLAT=0 disables outright.")
                            end
                        else
                            _off = count(x -> abs(x) > 4.5, cr) / length(cr)
                            if _off > _tol
                                _refuse = true
                                println("  AI racing LINE: Classic's line REFUSED on ", TRACKSEL, " -- no corridor",
                                        " files, and ", round(100*_off, digits=1), "% of its records map beyond",
                                        " 4.5 m of our centreline (bar ", round(100*_tol, digits=1), "%);",
                                        " keeping our own racing line. JM_AI_CLASSICLAT_TOL raises the bar.")
                            end
                        end
                        if !_refuse
                        RaceAI.set_classic_lateral!(cr, c1, c2)

                        if ROADHAT !== TERRAIN0
                            let (fr, nr) = onroad_frac(Float64.(CLASSICLP.read_lp(lp).dlat)),
                                (f1, n1) = onroad_frac(g1), (f2, n2) = onroad_frac(g2)
                                println("  AI racing LINE on the ROAD-ONLY mesh: race ", round(100*fr, digits=1),
                                        "% (", nr, " off), pass1 ", round(100*f1, digits=1), "% (", n1,
                                        " off), pass2 ", round(100*f2, digits=1), "% (", n2, " off)")

                                let (r1, m1) = onroad_car_frac(g1), (r2, m2) = onroad_car_frac(g2)
                                    println("  AI passing rails, graded (car body overlaps the road): pass1 ", round(100*r1, digits=1),
                                            "% (furthest ", round(m1, digits=2), " m), pass2 ", round(100*r2, digits=1), "% (furthest ",
                                            round(m2, digits=2), " m)")
                                end

                                let (fc, nc) = onroad_frac(zeros(nrec)), noff = 0
                                    for i in 1:nrec
                                        sv = 3.0*(i-1)
                                        q = RaceAI.pose_at(AILINE, sv, RaceAI.racelane(AILINE, sv))
                                        JRTrack.hat3d(ROADHAT, q[1], q[3]; ref = Inf)[3] || (noff += 1)
                                    end
                                    println("  AI racing LINE control: Classic's reference line (dlat 0) ",
                                            round(100*fc, digits=1), "% on road (", nc, " off); OUR racing line ",
                                            round(100*(1 - noff/nrec), digits=1), "% (", noff, " off)",
                                            " -- if these are not near 100%, the road mesh is the fault and",
                                            " the Classic numbers above mean nothing")
                                end
                            end
                        end
                        if bnd_lo !== nothing && bnd_hi !== nothing
                            RaceAI.set_classic_band!(bnd_lo, bnd_hi)
                            let w = [abs(bnd_hi[k] - bnd_lo[k]) for k in 1:min(length(bnd_lo), length(bnd_hi))]
                                sort!(w)
                                println("  AI racing BAND: Classic's minrace/maxrace corridor -> our frame; width min ",
                                        round(w[1], digits=2), " p50 ", round(w[length(w)÷2], digits=2), " max ",
                                        round(w[end], digits=2), " m (was a fixed +-", RaceAI.LANE_MAX,
                                        " m; margin ", RaceAI.BAND_MARGIN, " m each side; JM_AI_CLASSICBAND=0 reverts)")
                            end
                        end
                        let q = sort(copy(cr)), m = length(q)
                            println("  AI racing LINE: corrected lateral p05 ", round(q[max(1,m÷20)], digits=2),
                                    " p50 ", round(q[m÷2], digits=2), " p95 ", round(q[19m÷20], digits=2),
                                    " m; |lat| > 4.5 m on ", count(x -> abs(x) > 4.5, cr), " of ", m,
                                    " records (a line off the road would show here)")
                        end
                        println("  AI racing LINE: Classic race/pass1/pass2.lp (", nrec, " records, lateral sign ",
                                sgn, ", re-centre shift mean ",
                                round(sum(abs, RECENTRE_SHIFT)/max(length(RECENTRE_SHIFT),1), digits=2),
                                " m) -> our frame: race median ", round(sum(cr)/length(cr), digits=2),
                                " m, rails ", round(sum(c1)/length(c1), digits=2), " / ",
                                round(sum(c2)/length(c2), digits=2), " m  <- ", lp1)
                        end
                    else
                        @warn "JM_AI_CLASSICLAT: no pass1.lp/pass2.lp in $ZD -- AI keep our own racing line"
                    end
                end
                println("  AI speed profile: Classic race.lp (", length(gv), " records, adj ", adj, ", cap ", round(vcap, digits=1), " m/s)  <- ", lp)
            else
                @warn "JM_AI_CLASSICLINE: no race.lp in $ZD -- AI keep the κ speed model"
            end
        else
            @warn "JM_AI_CLASSICLINE: $(TRACKSEL) is re-centred, so Classic's dlong index does not align -- AI keep the κ speed model"
        end
    end

    if _DX_LINE_Y && AILINE !== nothing
        n = length(AILINE.x); diffs = Float64[]
        for k in 1:max(1, n ÷ 200):n
            g = groundz(AILINE.x[k], AILINE.z[k])
            push!(diffs, AILINE.y[k] - Float64(g))
        end
        sort!(diffs)
        md = diffs[max(1, length(diffs) ÷ 2)]
        println("  [line_y] ", length(diffs), " samples  line.y - groundz:  min ",
                round(minimum(diffs), digits=3), "  median ", round(md, digits=3),
                "  max ", round(maximum(diffs), digits=3), " m")
        flush(stdout)
    end
    AICARS = AILINE === nothing ? RaceAI.AICar[] : RaceAI.init_cars(AILINE, N_AI; start_s = 30.0)

    if !isempty(AICARS)
        pw = [AICAR_PHYS[i][1]/AICAR_PHYS[i][2] for i in 1:length(AICARS)]
        pwmax = maximum(pw)

        _temper = clamp(something(tryparse(Float64, get(ENV, "JM_AI_TEMPER", "0.35")), 0.35), 0.0, 1.0)
        for (i, c) in enumerate(AICARS); c.pace = (1.0 - _temper*(1.0 - pw[i]/pwmax))*WET_PACE; end

        AI_RNG[] = Random.MersenneTwister(AI_SEED)
        _form = parse(Float64, get(ENV, "JM_AI_FORM", "0.012"))
        for c in AICARS; c.pace *= 1.0 + _form*clamp(randn(AI_RNG[]), -2.0, 2.0); end

        for (rank, ci) in enumerate(sortperm([c.pace for c in AICARS], rev=true))

            AICARS[ci].s = 9.0*(length(AICARS) + 1 - rank); AICARS[ci].lap = 0; AICARS[ci].lane = isodd(rank) ? -2.4 : 2.4
        end
        println("  → AI pace spread (power/weight, gridded fastest-first): ",
                join(["$(AISPECS[i][1]) $(round(Int,AICARS[i].pace*100))%" for i in 1:length(AICARS)], ", "))
    end

    if haskey(ENV, "JM_AIPLACE") && !isempty(AICARS)
        let f = split(ENV["JM_AIPLACE"], ":"), k = parse(Int, f[1])
            if 1 <= k <= length(AICARS)
                AICARS[k].s = mod(parse(Float64, f[2]), AILINE.total); length(f) >= 3 && (AICARS[k].lane = parse(Float64, f[3]))
                println("  JM_AIPLACE: AI ", k, " stands at s=", round(AICARS[k].s, digits = 1), " lane ", round(AICARS[k].lane, digits = 2))
            end
        end
    end
    AICHASSIS = AICARMODELS[1:length(AICARS)]

    AIPHYS = AICarT[]
    if AI_PHYSICS && AILINE !== nothing && isempty(REPLAY_FILE)
        print("  building $(length(AICARS)) physics AI (shared JM 3-D model)… "); flush(stdout)
        poses = [(p = RaceAI.pose_at(AILINE, c.s, c.lane); (p[1], p[3], p[4], 0.0)) for c in AICARS]
        AIPHYS = AIbuild(poses)
        println("done")
    end
    if AI_LOOSE && CAR3D && AILINE !== nothing && isempty(REPLAY_FILE) && isempty(LOOSE_CARS) && !isempty(AICARS)
        for _ in 1:LOOSE_N; push!(LOOSE_CARS, DriveRT3D.with_lotus(() -> DriveRT3D.build_car3d(; x0 = 0.0, v0 = 0.0))); end

        let pc = LOOSE_CARS[1], p0 = RaceAI.pose_at(AILINE, 0.0, 0.0)
            DriveRT3D.place3d!(pc, p0[1], p0[3], p0[4]; v = 10.0)
            DriveRT3D.step_car3d!(pc, 0.0, LOOSE_BRAKE, 0.0, 1/60; manual = true, groundz = groundz_ai)
            DriveRT3D.step_car3d!(pc, 0.1, 0.0, 0.0, 1/60; manual = false, groundz = groundz_ai)
        end
    end

    AI_T0    = AILINE === nothing ? 0.0 : RaceAI.natural_laptime(AILINE)

    _DX_PACEDIAG && AILINE !== nothing && dx_pacediag(AILINE)
    AI_TGT   = AI_REFLAP * 100.0 / AI_PCT
    AI_SCALE = AILINE === nothing ? 1.0 : clamp(AI_T0 / max(AI_TGT, 1.0), 0.4, 2.2)
    if AILINE !== nothing
        println("  → AI grid: ", join((m.name for m in AICHASSIS), ", "))
        println("  → AI pace: ", round(Int,AI_PCT), "% of Classic (ref ", round(AI_REFLAP,digits=1),
                "s → target ", round(AI_TGT,digits=1), "s; rail ", round(AI_T0,digits=1),
                "s, scale ", round(AI_SCALE,digits=2), ")")
    end

    CLASSICAI_T = nothing; CLASSICAI_SC = 1.0; CLASSICAI_F = Ref{Any}(nothing); CLASSICAI_PH = Ref(0); CLASSICAI_PD = Ref((0.0, 0.0))
    if CLASSICAI_ON && AILINE !== nothing && TRK_CAL.on && all(f -> isfile(joinpath(ZD, f * ".lp")), ("race", "minrace", "maxrace", "pass1", "pass2"))
        try
            lines = Dict(k => CLASSICAI.read_line(joinpath(ZD, f * ".lp")) for (k, f) in
                         ((CLASSICAI.RACE, "race"), (CLASSICAI.MINR, "minrace"), (CLASSICAI.MAXR, "maxrace"), (CLASSICAI.PASS1, "pass1"), (CLASSICAI.PASS2, "pass2")))

            if AI_PITLINE && isfile(joinpath(ZD, "pit.lp"))
                lines[CLASSICAI.PIT] = CLASSICAI.read_line(joinpath(ZD, "pit.lp"))
                let lp = lines[CLASSICAI.PIT], lr = lines[CLASSICAI.RACE], n = length(lp.d)
                    k = n; while k > 1 && abs(lp.d[k-1] - lr.d[min(k-1, length(lr.d))]) > 3.0; k -= 1; end
                    PIT_IN[] = 3.0*(k - 1)
                    kb = k - 1 + argmin([lp.v[j] for j in k:n]); PIT_BOX[] = 3.0*(kb - 1)

                    ke = kb; while ke < n && abs(lp.d[ke + 1] - lp.d[kb]) < 1.5; ke += 1; end
                    PIT_END[] = 3.0*(ke - 1)
                    println("  [pit] Classic pit line: in at ", round(Int, PIT_IN[]), " m, pit lane (boxes) ", round(Int, PIT_BOX[]), "-",
                            round(Int, PIT_END[]), " m of ", round(Int, 3.0*n), " (JM_AI_PITLINE=0: stop at the edge of the straight, as before)")
                end
            end
            trkb = read(track_file(CLASSICNAME, ".trk"))
            secs = CLASSICAI.trk_sections(trkb)
            r0 = CLASSICAI.Ref(secs...)
            num = 0.0
            glap = 3.0*length(lines[CLASSICAI.RACE].d)
            for x in 0:3.0:glap-3
                k = CLASSICAI.curv(r0, x*r0.lap/glap); abs(k) > 1/150 && (num += sign(k)*CLASSICAI.dlat(lines[CLASSICAI.RACE], x))
            end
            raw = CLASSICTrack.trk_centreline(track_file(CLASSICNAME, ".trk"))
            A0  = rigid_refit(ALIGNED0, ALIGNED)
            ref = CLASSICAI.fit_transform(r0, raw, A0; sgn = num >= 0 ? 1.0 : -1.0)
            adj = 1.0; vcap = 2.41*36.0; sepc = 1.0
            ini = joinpath(ZD, "track.ini")
            if isfile(ini)
                for l in eachline(ini)
                    m = match(r"^\s*dlong_speed_adj_coeff\s*=\s*([0-9.]+)", l); m !== nothing && (adj = parse(Float64, m[1]))
                    m = match(r"^\s*dlong_speed_maximum\s*=\s*([0-9.]+)", l);   m !== nothing && (vcap = parse(Float64, m[1])*36.0)
                    m = match(r"^\s*track_dlong_sep_coeff\s*=\s*([0-9.]+)", l); m !== nothing && (sepc = parse(Float64, m[1]))
                end
            end
            P = CLASSICAI.params(RaceAI.classic_ai(); sep_coeff = sepc, adj = adj, vcap = vcap)

            function gh(s, d, x, z)
                hr = JRTrack.hat(TRKSURF, x, z)
                hr.found && return trk_road(hr.lapdist, hr.lateral)
                h = mesh_ground(x, z); h[1]
            end

            dsh = Float64[]
            if get(ENV, "JM_AI_ROADSHIFT", "1") != "0"
                n3 = max(1, floor(Int, ref.lap/3.0)); raw_d = zeros(n3); hint = 1; na = length(ALIGNED); nrej = Ref(0)
                for i in 1:n3
                    sa = (i - 1)*3.0
                    w0 = CLASSICAI.world(ref, sa, 0.0); w1 = CLASSICAI.world(ref, sa, 1.0); ex = w1[1] - w0[1]; ez = w1[2] - w0[2]
                    bd = Inf; bp = (w0[1], w0[2])
                    for kk in (i == 1 ? (1:na) : (hint - 40):(hint + 40))
                        k = mod1(kk, na); j = k == na ? 1 : k + 1
                        ax, az = ALIGNED[k]; vx = ALIGNED[j][1] - ax; vz = ALIGNED[j][2] - az; L2 = vx*vx + vz*vz
                        u = L2 > 0 ? clamp(((w0[1] - ax)*vx + (w0[2] - az)*vz)/L2, 0.0, 1.0) : 0.0
                        px = ax + u*vx; pz = az + u*vz; d2 = (px - w0[1])^2 + (pz - w0[2])^2
                        d2 < bd && (bd = d2; bp = (px, pz); hint = k)
                    end
                    raw_d[i] = ((bp[1] - w0[1])*ex + (bp[2] - w0[2])*ez)/max(ex*ex + ez*ez, 1e-9)

                    if AI_ROADSHIFT_CHECK
                        onr(x, z) = JRTrack.hat3d(ROADHAT, x, z; ref = Inf)[3]
                        dr = CLASSICAI.dlat(lines[CLASSICAI.RACE], sa)
                        (!onr(w0[1] + (dr + raw_d[i])*ex, w0[2] + (dr + raw_d[i])*ez) && onr(w0[1] + dr*ex, w0[2] + dr*ez)) && (raw_d[i] = 0.0; nrej[] += 1)
                    end
                end
                med = [sort([raw_d[mod1(i + q, n3)] for q in -3:3])[4] for i in 1:n3]

                sg_ = parse(Float64, get(ENV, "JM_AI_DSH_SIGMA", "6")); hw_ = ceil(Int, 3*sg_)
                g = [exp(-q^2/(2*sg_^2)) for q in -hw_:hw_]; g ./= sum(g)
                dsh = [sum(g[q + hw_ + 1]*med[mod1(i + q, n3)] for q in -hw_:hw_) for i in 1:n3]
                slope_ = parse(Float64, get(ENV, "JM_AI_DSH_SLOPE", "0.05"))*3.0
                if slope_ > 0
                    fw = copy(dsh); bw = copy(dsh)
                    for _ in 1:2, i in 1:n3; j = mod1(i - 1, n3); fw[i] = clamp(fw[i], fw[j] - slope_, fw[j] + slope_); end
                    for _ in 1:2, i in n3:-1:1; j = mod1(i + 1, n3); bw[i] = clamp(bw[i], bw[j] - slope_, bw[j] + slope_); end
                    dsh = (fw .+ bw) ./ 2
                end
                if haskey(ENV, "JM_AI_DSHDUMP")
                    open(ENV["JM_AI_DSHDUMP"], "w") do io
                        for i in 1:n3
                            sa = (i - 1)*3.0; sg = sa/(ref.lap/glap)
                            println(io, sa, " ", raw_d[i], " ", dsh[i], " ", CLASSICAI.dlat(lines[CLASSICAI.RACE], sg), " ",
                                    CLASSICAI.dlat(lines[CLASSICAI.MINR], sg), " ", CLASSICAI.dlat(lines[CLASSICAI.MAXR], sg))
                        end
                    end
                end
                println("  → AI frame: lateral correction to our road, |shift| mean ", round(sum(abs, dsh)/n3, digits = 2),
                        " m, max ", round(maximum(abs, dsh), digits = 2), " m (JM_AI_ROADSHIFT=0 off); ", nrej[],
                        " samples kept on Classic's frame where the correction left the road (MONZAROAD-1)")
            end
            CLASSICAI_T = CLASSICAI.Track(lines, ref, glap, P, gh, dsh, 3.0)

            pk = get(ENV, "JM_AI_PITKEEP", "-3.0")
            CLASSICAI.KEEP[] = (SPA && pk != "0") ? [(13750.0, 200.0, parse(Float64, pk), 99.0)] : NTuple{4,Float64}[]
            if _DX_PIT_DIAG && haskey(lines, CLASSICAI.PIT)
                for x in PIT_IN[]-60:30.0:glap-1
                    dp = CLASSICAI.dlat(lines[CLASSICAI.PIT], x); dr = CLASSICAI.dlat(lines[CLASSICAI.RACE], x)
                    wp = CLASSICAI.tworld(CLASSICAI_T, x, dp); wr = CLASSICAI.tworld(CLASSICAI_T, x, dr)
                    println("  [pitdiag] s ", round(Int, x), " pit d ", round(dp, digits = 1), " race d ", round(dr, digits = 1),
                            "  world gap ", round(hypot(wp[1] - wr[1], wp[2] - wr[2]), digits = 1), " m",
                            "  pit on road ", JRTrack.hat3d(ROADHAT, wp[1], wp[2]; ref = Inf)[3], " corridor ", round.(CLASSICAI.corridor(CLASSICAI_T, x), digits = 1))
                end
            end
            gs = CLASSICAI.grip_scale(CLASSICAI_T)

            c1 = CLASSICAI.Car(CLASSICAI_T, 1, 10.0, CLASSICAI.dlat(lines[CLASSICAI.RACE], 10.0); v = 30.0)
            f1 = CLASSICAI.Field(CLASSICAI_T, [c1]); t0 = -1.0; tk = 0
            while tk < 36*400
                CLASSICAI.tick!(f1, nothing, gs, Inf); tk += 1
                (c1.lap == 1 && t0 < 0) && (t0 = tk/36)
                c1.lap == 2 && break
            end
            tnat = t0 > 0 ? tk/36 - t0 : NaN
            CLASSICAI_SC = isfinite(tnat) ? gs*clamp(tnat/max(AI_TGT, 1.0), 0.4, 1.0) : gs

            function classicai_natlap(sc)
                cc = CLASSICAI.Car(CLASSICAI_T, 1, 10.0, CLASSICAI.dlat(lines[CLASSICAI.RACE], 10.0); v = 30.0)
                ff = CLASSICAI.Field(CLASSICAI_T, [cc]); ta = -1.0; tq = 0
                while tq < 36*1200
                    CLASSICAI.tick!(ff, nothing, sc, Inf); tq += 1
                    (cc.lap == 1 && ta < 0) && (ta = tq/36)
                    cc.lap == 2 && break
                end
                ta > 0 && cc.lap == 2 ? tq/36 - ta : NaN
            end

            if isfinite(tnat) && get(ENV, "JM_AI_PACE_CAL", "1") != "0"
                _tc = time(); lo_ = 0.25*gs; hi_ = gs; tlo_ = classicai_natlap(lo_); nev = 1
                isfinite(tlo_) || (tlo_ = Inf)
                if AI_TGT >= tlo_
                    CLASSICAI_SC = lo_
                elseif AI_TGT <= tnat
                    CLASSICAI_SC = gs
                else
                    for _ in 1:8
                        mid_ = (lo_ + hi_)/2; tm_ = classicai_natlap(mid_); nev += 1
                        (!isfinite(tm_) || tm_ > AI_TGT) ? (lo_ = mid_) : (hi_ = mid_)
                    end
                    CLASSICAI_SC = (lo_ + hi_)/2
                end
                println("  → AI pace calibrated: scale ", round(CLASSICAI_SC, digits = 3), " laps in ",
                        round(classicai_natlap(CLASSICAI_SC), digits = 1), " s (target ", round(AI_TGT, digits = 1), " s; ", nev,
                        " headless laps, ", round(time() - _tc, digits = 1), " s)"); flush(stdout)
            end

            non = 0; ntot = 0
            for x in 0:6.0:glap-1
                w = CLASSICAI.tworld(CLASSICAI_T, x, CLASSICAI.dlat(lines[CLASSICAI.RACE], x)); ntot += 1
                JRTrack.hat3d(ROADHAT, w[1], w[2]; ref = Inf)[3] && (non += 1)
            end
            println("  → AI model: Classic (AICLASSIC-2). lap ", round(glap, digits = 1), " m (arcs ", round(ref.lap, digits = 1), "), +dlat ",
                    ref.sgn > 0 ? "left" : "right", ", grip scale ", round(gs, digits = 3), ", natural lap ", round(tnat, digits = 1),
                    " s, pace scale ", round(CLASSICAI_SC, digits = 3), " (target ", round(AI_TGT, digits = 1), " s); race line on our road ",
                    round(100*non/max(ntot, 1), digits = 1), " %")
        catch e
            println("  → AI model: Classic unavailable (", sprint(showerror, e), ") -- the rail field runs instead")
            CLASSICAI_T = nothing
        end
    end
    "the Classic field, built from AICARS (grid order, pace) the first time it is needed after a (re)start"
    function classicai_field!()
        f = CLASSICAI_F[]
        if f === nothing || length(f.cars) != length(AICARS)
            cars = CLASSICAI.Car[]
            for (i, c) in enumerate(AICARS)
                p = RaceAI.pose_at(AILINE, c.s, c.lane)
                (s, d, _) = CLASSICAI.tlocate(CLASSICAI_T, p[1], p[3])
                gc = CLASSICAI.Car(CLASSICAI_T, i, s, d; v = 0.0, pace = c.pace)
                gc.react = round(Int, CLASSICAI_T.P.start_hiatus) + rand(AI_RNG[], 0:24)
                push!(cars, gc)
            end
            f = CLASSICAI.Field(CLASSICAI_T, cars; seed = rand(AI_RNG[], UInt32)); CLASSICAI_F[] = f
        end
        f
    end

    if AI_PHYSICS && haskey(ENV, "JM_AI_TEST") && !isempty(AIPHYS)
        for (i,pc) in enumerate(AIPHYS); p = RaceAI.pose_at(AILINE, AICARS[i].s, AICARS[i].lane); AIplace!(pc, p[1], p[3], p[4]; v=12.0); end
        N=length(AIPHYS); maxr=0.0; spins=0; maxlat=0.0; aidist=zeros(N); stuck=zeros(Int,N); scon=zeros(Int,N); lastx=[pc.x for pc in AIPHYS]; lastz=[pc.z for pc in AIPHYS]
        nstep=5400
        for _ in 1:nstep
            for (i,pc) in enumerate(AIPHYS)
                s,lat = RaceAI.project(AILINE, pc.x, pc.z; hint = AICARS[i].s); AICARS[i].s=s; AICARS[i].lane=lat; AICARS[i].v=pc.v
                maxlat = max(maxlat, abs(lat))
                aidist[i] += hypot(pc.x-lastx[i], pc.z-lastz[i]); lastx[i]=pc.x; lastz[i]=pc.z
                offhat = !JRTrack.hat3d(TERRAIN, pc.x, pc.z; ref=Inf)[3]
                railθ = RaceAI.pose_at(AILINE, s, 0.0)[4]
                spun = abs(atan(sin(pc.θ - railθ), cos(pc.θ - railθ))) > SPIN_LIM
                if pc.v < 1.4 || abs(lat) > 9.0 || offhat || spun
                    stuck[i]+=1; scon[i]+=1
                    lim = (offhat || abs(lat)>12.0 || spun) ? 6 : (abs(lat)>9.0 ? 18 : 90)
                    if scon[i] > lim; rp=RaceAI.pose_at(AILINE, s+10.0, 0.0); AIplace!(pc, rp[1], rp[3], rp[4]; v=max(8.0,pc.v*0.6)); scon[i]=0; end
                else; scon[i]=0; end
            end
            vts = RaceAI.plan!(AICARS, AILINE; scale=1.0, amax=AI_AMAX)
            for (i,pc) in enumerate(AIPHYS)
                r = AIyaw(pc); maxr = max(maxr, abs(r)); abs(r) > 2.5 && (spins += 1)
                thr,brk,st = RaceAI.controller(AILINE, AICARS[i].s, AICARS[i].lane, AICARS[i].tlane, vts[i], pc.x, pc.z, pc.θ, pc.v, r; power=AI_POWER)
                DriveRT3D.step_car3d!(pc, thr, brk, st, 1/60; manual=false, groundz=groundz_ai)
                ho = solid_hit(pc.x, pc.z, pc.θ, pc.v); ho !== nothing && AIbump!(pc, ho[1], ho[2], ho[3], ho[4], ho[5], ho[6])
            end
        end
        avgkmh = round.(Int, aidist ./ 90 .* 3.6)
        println("  AI self-test on $(TRACKSEL) (90s): dist=", round.(Int,aidist), "m  avg_kmh=$avgkmh  max_yaw=$(round(maxr,digits=2))  spins=$spins  max_lat=$(round(maxlat,digits=1))m  stuck=$stuck")
        println(spins < 30 && minimum(aidist) > 800 && maximum(stuck) < 200 ? "  ✓ physics AI lap the real track cleanly" : "  ⚠ AI struggle here — tune controller")
        exit(0)
    end

    if _DX_AI_TEST && !AI_PHYSICS && AILINE !== nothing && !isempty(AICARS)
        nsec = clamp(something(tryparse(Int, get(ENV,"JM_AI_TEST","90")), 90), 1, 1200)
        N=length(AICARS); maxlat=0.0; aidist=zeros(N)
        lastp=[RaceAI.pose_at(AILINE, c.s, c.lane) for c in AICARS]
        for _ in 1:nsec*60
            poses,_ = RaceAI.step_field!(AICARS, AILINE, 1/60; scale=AI_SCALE, player=(-1e9,0.0,100.0), rel=AI_REL)
            for (i,p) in enumerate(poses)
                maxlat = max(maxlat, abs(RaceAI.project(AILINE, p[1], p[3])[2]))
                aidist[i] += hypot(p[1]-lastp[i][1], p[3]-lastp[i][3]); lastp[i]=p
            end
        end
        avgkmh = round.(Int, aidist ./ nsec .* 3.6)
        println("  KINEMATIC AI self-test on $(TRACKSEL) ($(nsec)s): dist=", round.(Int,aidist), "m  avg_kmh=$avgkmh  max_lat=$(round(maxlat,digits=1))m")

        let (fs, fv) = RaceAI.free_speed_profile(AILINE; scale=AI_SCALE),
            ds = length(fs) > 1 ? fs[2]-fs[1] : 5.0
            RaceAI.aistat_reset!()
            AICARS2 = RaceAI.init_cars(AILINE, N; start_s = 30.0)
            for (i, c) in enumerate(AICARS2); c.pace = AICARS[i].pace; end
            deficit_ep = zeros(Int, N); ep_run = zeros(Int, N); worst = zeros(N); lapsdone = zeros(Int, N)
            frames = nsec*60
            for f in 1:frames
                RaceAI.step_field!(AICARS2, AILINE, 1/60; scale=AI_SCALE, player=(-1e9,0.0,100.0), rel=AI_REL)
                for (i, c) in enumerate(AICARS2)
                    k = clamp(floor(Int, mod(c.s, AILINE.total) / ds) + 1, 1, length(fv))
                    d = fv[k] - c.v
                    worst[i] = max(worst[i], d)
                    if d > 5.0; ep_run[i] += 1; if ep_run[i] == 30; deficit_ep[i] += 1; end
                    else; ep_run[i] = 0; end
                    lapsdone[i] = c.lap
                end
            end
            st = RaceAI.AISTAT; tl = max(1, sum(lapsdone))
            println("  [E89] field of $N over $(nsec)s, $(sum(lapsdone)) car-laps:")
            println("  [E89]   fall-back episodes (deficit >5 m/s for >0.5 s) per car: ", deficit_ep,
                    "  = ", round(sum(deficit_ep)/tl, digits=2), " per car-lap;  worst deficit per car (m/s): ", round.(worst, digits=1))
            println("  [E89]   rail switches: engage=", st.engage, " release=", st.release,
                    "  = ", round((st.engage+st.release)/tl, digits=2), " per car-lap")
            println("  [E89]   speed-match frames=", st.match, "  queue-snaps=", st.qsnap, "  side-pushes=", st.sidepush, "  mishaps=", st.mishap)
        end

        for (i,c) in enumerate(AICARS)
            oldp = c.lap + c.s/AILINE.total; newp = c.lap + mod(c.s, AILINE.total)/AILINE.total
            println("    ", rpad(AISPECS[i][1],8), " lap=", c.lap, "  OLD prog=", round(oldp,digits=2),
                    " (2× bug)   NEW prog=", round(newp,digits=2), " ✓")
        end
        println(minimum(aidist) > 800 && maxlat < 12.0 ? "  ✓ kinematic AI lap the real track on-rail (no spins possible)" : "  ⚠ field not advancing / off-rail — check AILINE")
        exit(0)
    end

    ffb = get(ENV, "JM_FFB_FAKE", "0") != "0" ? FFB.FFBDevice(Cint(-1), Int16(0), zeros(UInt8, 48), true, "fake") :
          (FFB_ON && !SMOKE) ? FFB.open_ffb() : nothing
    (ffb !== nothing && ffb.ok) ? println("  force feedback: ON  (", ffb.path, ", gain ", FFB_GAIN, ")") :
                                  println("  force feedback: off", FFB_ON ? " (no wheel found)" : " (JM_NOFFB)")
    spin = 0.0; last = time(); frames = 0; titleT = last
    net_last = Ref(-1.0)
    ai_stuck = zeros(Int, length(AIPHYS))
    ai_lap_prev = zeros(Int, length(AICARS))
    ai_onroad = trues(length(AIPHYS))
    ffb_f = 0.0
    ffb_jolt = 0.0
    fy_lp = 0.0
    tc_hud = ntuple(_->(0.0,0.0,1.0), 4)
    v_prev = cs.v; pitch_dyn = 0.0; pitch_ter = 0.0; roll_ter = 0.0
    cam_pitch = 0.0; cam_roll = 0.0

    lap_t0 = cs.t; last_lap = 0.0; best_lap = 0.0; prev_laps = cs.laps; tsamp = 0; race_done = false; enterPrev = false

    player_laps = Float64[]
    ai_lapt0 = zeros(length(AICARS)); ai_best = fill(Inf, length(AICARS))
    player_s_prev = CLINE === nothing ? 0.0 : RaceAI.project(CLINE, cs.x, cs.z)[1]
    player_prog = 0.0
    fmt_lap(s) = (m=floor(Int,s/60); sec=s-60m; si=floor(Int,sec); ms=round(Int,(sec-si)*1000);
                  "$m:$(lpad(si,2,'0')).$(lpad(ms,3,'0'))")

    function plausible_lap(track, t)
        t > 0 || return false
        ref = get(REF_LAP, track, 0.0)
        ref <= 0 || t >= 0.5 * ref
    end

    banking_allowed() = !AUTODRIVE && !SMOKE
    function save_human_best(track, t)
        if !banking_allowed()
            println("  (not banking a best lap: this run has no human driver)")
            return
        end
        if !plausible_lap(track, t)
            println("  ⚠ not banking an implausible best lap for ", track, ": ", round(t, digits=3),
                    " s (Classic reference ", round(get(REF_LAP, track, 0.0), digits=1), " s)")
            return
        end
        try
            path = joinpath(@__DIR__, "human_best.txt")
            bests = Dict{String,Float64}()
            isfile(path) && for ln in eachline(path)
                sp = split(strip(ln), '\t')
                length(sp) == 2 && (v = tryparse(Float64, sp[2])) !== nothing &&
                    plausible_lap(sp[1], v) && (bests[sp[1]] = v)
            end
            (!haskey(bests, track) || t < bests[track]) || return
            bests[track] = t
            open(path, "w") do io; for (k,v) in bests; println(io, "$k\t$(round(v, digits=3))"); end; end
        catch e; @warn "human_best write failed" e; end
    end

    DO_QUAL  = IS_RACE && N_AI > 0 && !SKIDPAD && haskey(ENV,"JM_QUAL") && !SMOKE
    phase    = Ref(DO_QUAL ? :practice : :race)
    player_grid = Ref(0); player_finpos = Ref(0)

    HOLD_START = IS_RACE && N_AI > 0 && !SKIDPAD && (!SMOKE || get(ENV, "JM_SMOKE_COUNTDOWN", "0") != "0") && isempty(REPLAY_FILE)
    race_go    = Ref(!HOLD_START)

    SMOKE_HOLD = SMOKE ? parse(Float64, get(ENV, "JM_SMOKE_HOLD", "0")) : 0.0
    SMOKE_HOLD > 0 && (race_go[] = false)
    cd_armed   = Ref(false)
    cd_said    = Ref(false)
    cd_t0      = Ref(-1.0)
    cd_left    = Ref(-1.0)

    ai_release = Ref(-1.0)
    launch_done = Ref(false)

    ai_quals = [AI_TGT * (1 + 0.004*(i-1)) for i in 1:length(AICARS)]
    ROW = 9.0; GRID_LANE = 2.2
    function form_grid!(qtime)
        order = RaceAI.grid_order(qtime, ai_quals)

        POLE && (order = vcat(0, filter(!=(0), order)))
        prank = findfirst(==(0), order)

        sp = RaceAI.project(AILINE, cs0.x, cs0.z)[1]
        for (i, c) in enumerate(AICARS)
            r = findfirst(==(i), order)
            c.s = mod(sp - (r - prank)*ROW, AILINE.total)

            c.v = 0.0; c.lap = (r > prank) ? -1 : 0
            c.lane = isodd(r) ? GRID_LANE : -GRID_LANE; c.tlane = c.lane
            AI_PHYSICS && (gp = RaceAI.pose_at(AILINE, c.s, c.lane); AIplace!(AIPHYS[i], gp[1], gp[3], gp[4]; v=0.0))
        end
        println(POLE ? "\n  ═══ GRID (you on POLE — JM_POLE) ═══" :
                isfinite(qtime) ? "\n  ═══ GRID (from your practice best) ═══" : "\n  ═══ GRID ═══")
        for (p, id) in enumerate(order)
            println("   P$p  ", id==0 ? (isfinite(qtime) ? "You — $(fmt_lap(qtime))" : "You (no practice lap)") : AICHASSIS[id].name)
        end
        println("  → You start P$prank of $(length(order)) at s=", round(sp, digits=1), " m — $(COUNTDOWN > 0 ? "wait for the countdown" : "floor it to launch the field")\n"); flush(stdout)
        prank
    end
    COUNTDOWN > 0 && HOLD_START && println("\n  COUNTDOWN START — ", round(Int, COUNTDOWN),
        " s on the clock, then GREEN for the whole field at once.")
    DO_QUAL && println("\n  PRACTICE ($(round(Int,PRACTICE_SEC/60)) min) — lap to set your grid slot (your best lap = your\n  starting position; no lap = back of the grid).  Press T to ACCELERATE TIME straight to the race.")

    if !DO_QUAL && HOLD_START
        player_grid[] = form_grid!(Inf)
    end

    FUEL_ON   = !SKIDPAD
    burn_lap  = FUEL_ON ? FUEL_LPK * LAPLEN/1000 : 0.0
    fuel_laps = IS_RACE ? (RACE_LAPS + FUEL_MARGIN) : 40
    fuel      = Ref(FUEL_ON ? burn_lap * fuel_laps : 0.0)
    FUEL1     = FUEL_ON && isfinite(FUEL_START_L[])
    if FUEL1
        burn_lap = FUEL_PLAN_LPK * LAPLEN/1000; fuel_laps = FUEL_START_L[]/max(burn_lap, 1e-6); fuel[] = FUEL_START_L[]
    end
    refuel!() = (fuel[] = FUEL1 ? FUEL_START_L[] : burn_lap * fuel_laps; CAR3D && FUEL1 && DriveRT3D.set_fuel_burnt!(cs, 0.0))
    fuel_mass_t = Ref(-1e9)
    if FUEL_ON
        println("  → fuel: ", round(fuel[],digits=1), " L = ", fuel_laps, " laps",
                IS_RACE ? " ($RACE_LAPS race + $FUEL_MARGIN margin)" : "",
                "  (", round(burn_lap,digits=2), " L/lap)")
    end

    TRACE_PROG = get(ENV,"JM_TRACE_PROG","") != ""
    prog_bad = Ref(false); prog_t0 = Ref(0.0)
    function standings()
        isempty(REP_STAND) || return copy(REP_STAND)

        pp = if CLINE !== nothing && get(ENV,"JM_OLD_PLAYERPROG","") == ""
            player_prog / CLINE.total
        else
            cs.laps + (FUEL_ON && LAPLEN > 0 ? clamp(cs.lapdist/LAPLEN, 0.0, 1.0) : 0.0)
        end
        entries = Tuple{Int,Float64}[(0, pp)]
        for (i,c) in enumerate(AICARS)

            push!(entries, (i, c.lap + (AILINE === nothing ? 0.0 : mod(c.s, AILINE.total)/AILINE.total)))
        end
        sort!(entries, by = e -> -e[2])
        entries
    end
    ent_name(id) = id == 0 ? "You" : (id <= length(AIDRIVERS) ? "$(AIDRIVERS[id]) ($(AICHASSIS[id].name))" : AICHASSIS[id].name)
    texthud_name(id) = id == 0 ? "You" : (haskey(ENV, "JM_TEXTHUD_CHASSIS") ? AICHASSIS[id].name :
                                          (id <= length(AIDRIVERS) ? AIDRIVERS[id] : AICHASSIS[id].name))
    ibt_samples = IBTREC ? Dict{String,Float64}[] : nothing

    function flush_ibt!(samples)
        (samples === nothing || isempty(samples)) && return
        try
            tmpl = ibt_open(IBTTMPL)
            ts = Dates.format(Dates.now(), "yyyy-mm-dd HH-MM-SS")
            odir = REC_DIR
            mkpath(odir)
            out = joinpath(odir, "lotus49_$(IBTNAME) $(ts).ibt"); n = 2
            while isfile(out); out = joinpath(odir, "lotus49_$(IBTNAME) $(ts) ($n).ibt"); n += 1; end

            write_ibt(out, tmpl, samples; yaml = GARAGE_PP[] === nothing ? nothing :
                                                 Garage.carsetup_yaml(session_yaml(tmpl), GARAGE_PP[]; maxlen = session_space(tmpl)))
            println("  wrote reference telemetry: ", out, "  (", length(samples), " ticks, ", filesize(out)÷1024, " KB)")
        catch e
            println("  .ibt export failed: ", e)
        end
        empty!(samples); nothing
    end

    REPLAY_REC = isempty(REPLAY_FILE) && !haskey(ENV,"JM_NOREPLAY") && (!SMOKE || haskey(ENV,"JM_REPLAY_REC"))
    replay_buf = REPLAY_REC ? Float32[] : nothing; replay_t = Ref(-1.0); REPLAY_NCAR = 1 + length(AICARS)
    tele_buf = REPLAY_REC ? Float32[] : nothing
    empty!(REPLAY_WHEELBUF)
    tyre_lp = Ref(ntuple(_ -> (0.0, 0.0, 0.0, 0.0, 0.0), 4))
    REPLAY_AUTOSAVE = parse(Float64, get(ENV, "JM_REPLAY_AUTOSAVE", "20"))
    replay_saved = Ref(time())
    replay_names = String[ent_name(i) for i in 0:length(AICARS)]
    PLAYER_CAR == "Lotus" || (replay_names[1] = "You ($(PLAYER_CAR))")

    function new_replay_path()
        _odir = REC_DIR
        base = "replay_$(TRACKSEL) $(length(AICARS))ai $(Dates.format(Dates.now(), "yyyy-mm-dd HH-MM-SS"))"
        f = joinpath(_odir, base * ".jmr"); n = 2
        while isfile(f); f = joinpath(_odir, base * " ($n).jmr"); n += 1; end
        f
    end
    replay_out = Ref(REPLAY_REC ? new_replay_path() : "")
    IBTREC && println("  recording reference .ibt telemetry (JM_IBT) — template: ", basename(IBTTMPL))

    REPLAY = !isempty(REPLAY_FILE)
    repd = REPLAY ? deserialize(REPLAY_FILE) : nothing
    REPLAY ? (_nw = replay_wheels_load!(repd); _nw > 0 && println("  [replay] torn-off wheels: ", _nw, " records")) : (empty!(REP_WB); empty!(REP_WIDX))
    empty!(REP_STAND)
    REPCLK_ON[] = false; REPCLK_I[] = -1
    rep_rt = Ref(parse(Float64,get(ENV,"JM_REPLAY_T","0.0"))); rep_play = Ref(true); rep_speed = Ref(1.0)
    rep_psp = Ref(false); rep_pup = Ref(false); rep_pdn = Ref(false)
    rep_phome = Ref(false); rep_pend = Ref(false); rep_ppu = Ref(false); rep_ppd = Ref(false)
    rep_focus = Ref(clamp(parse(Int,get(ENV,"JM_REPLAY_FOCUS","0")), 0, REPLAY ? repd.ncar-1 : 0))
    rep_cam = Ref(clamp(parse(Int,get(ENV,"JM_REPLAY_CAM","2")), 1, length(REPLAY_CAMS)))
    rep_pn = Ref(false); rep_pv = Ref(false)
    rep_dur = REPLAY ? max(repd.nframes - 1, 0)/repd.fps : 0.0

    if INSTANT_AT_END[]
        INSTANT_AT_END[] = false; rep_rt[] = rep_dur; rep_play[] = false
        println("  [instant] the replay opens at ", round(rep_dur, digits = 2), " s of ", round(rep_dur, digits = 2), " s, paused (<- scrubs back)")
    end
    rep_st  = REPLAY ? 1 + 4*repd.ncar : 0

    if REPLAY && haskey(ENV, "JM_REPLAY_ZDIAG")
        let d = repd.data, nf = repd.nframes, bins = Dict{Int,Vector{Tuple{Float64,Float64}}}(), allr = Float64[]
            for f in 0:nf-1
                b = f*rep_st; x = Float64(d[b+2]); y = Float64(d[b+3]); z = Float64(d[b+4])
                hr = JRTrack.hat(TRKSURF, x, z); hr.found || continue
                r = JRTrack.hat3d(ROADHAT, x, z; ref = y + 3.0); t = JRTrack.hat3d(TERRAIN, x, z; ref = y + 3.0)
                dr = r[3] ? y - r[1] : NaN; dt_ = t[3] ? y - t[1] : NaN
                isnan(dr) || push!(allr, dr)
                push!(get!(bins, floor(Int, hr.lapdist/50), Tuple{Float64,Float64}[]), (dr, dt_))
            end

            let lagerr = zeros(16), lagn = zeros(Int, 16)
                for f in 16:nf-1
                    b = f*rep_st; y = Float64(d[b+3]); hr = JRTrack.hat(TRKSURF, Float64(d[b+2]), Float64(d[b+4])); hr.found || continue
                    (2000 <= hr.lapdist <= 2400 || 3400 <= hr.lapdist <= 3500 || 3850 <= hr.lapdist <= 3950) || continue
                    for k in 0:15
                        bk = (f-k)*rep_st; r = JRTrack.hat3d(ROADHAT, Float64(d[bk+2]), Float64(d[bk+4]); ref = y + 3.0)
                        r[3] && (lagerr[k+1] += abs(y - r[1]); lagn[k+1] += 1)
                    end
                end
                println("  [zdiag] mean |y - road under the car k frames earlier| over the bad sections, k = 0..15 (", round(repd.fps, digits = 1), " fps): ",
                    join([string(k, ":", round(lagerr[k+1]/max(lagn[k+1], 1), digits = 2)) for k in 0:15], "  "))
            end
            sort!(allr); med = isempty(allr) ? NaN : allr[cld(length(allr), 2)]
            println("  [zdiag] ", nf, " frames; car height - drawn road: median ", round(med, digits = 3), " m (the car's reference height on a flat road)")
            println("  [zdiag] lap distance covered: ", isempty(bins) ? "none" : string(50minimum(keys(bins)), " .. ", 50maximum(keys(bins)) + 50, " m (", length(bins), " bins)"))
            println("  [zdiag] 50 m bins where the car sits > 0.35 m off its usual height above the road (s, n, road min/mean/max, ground mean):")
            for k in sort(collect(keys(bins)))
                v = bins[k]; rs = filter(!isnan, [a for (a, _) in v]); gs = filter(!isnan, [c for (_, c) in v]); isempty(rs) && continue
                mn, mx, me = minimum(rs) - med, maximum(rs) - med, sum(rs)/length(rs) - med
                (mx > 0.35 || mn < -0.35) && println("     s ", lpad(50k, 6), "  n ", lpad(length(v), 4), "  road ", round(mn, digits = 2), " / ",
                    round(me, digits = 2), " / ", round(mx, digits = 2), "  ground ", isempty(gs) ? "-" : round(sum(gs)/length(gs) - med, digits = 2))
            end
        end

        let d = repd.data, nf = repd.nframes, wb = Dict{Int,Vector{Float64}}(), tb = Dict{Int,Vector{Float64}}()
            for f in 0:nf-1
                b = f*rep_st; cs = (x = Float64(d[b+2]), y = Float64(d[b+3]), z = Float64(d[b+4]), θ = Float64(d[b+5]))
                hr = JRTrack.hat(TRKSURF, cs.x, cs.z); hr.found || continue
                pt = terrain_pitch(cs); rt = terrain_roll(cs)
                M = Render.translate(cs.x, cs.y, -cs.z) * Render.roty(Float32(cs.θ)) * Render.rotz(Float32(pt)) * Render.rotx(Float32(rt)) * CARHAND
                gaps = Float64[]
                for (wx, wz, _, _, _) in WHEELS
                    q = M * Float32[wx, 0f0, wz, 1f0]; X = Float64(q[1]); Y = Float64(q[2]); Z = -Float64(q[3])
                    h = JRTrack.hat3d(ROADHAT, X, Z; ref = Y + 1.5); h[3] || (h = JRTrack.hat3d(TERRAIN, X, Z; ref = Y + 1.5))
                    h[3] && push!(gaps, Y - h[1])
                end
                isempty(gaps) && continue
                if haskey(ENV, "JM_REPLAY_ZDIAG_AT") && 0 <= hr.lapdist - parse(Float64, ENV["JM_REPLAY_ZDIAG_AT"]) < 3
                    c0 = JRTrack.hat3d(ROADHAT, cs.x, cs.z; ref = cs.y + 3.0); ci = JRTrack.hat3d(ROADHAT, cs.x, cs.z; ref = Inf)
                    t0 = JRTrack.hat3d(TERRAIN, cs.x, cs.z; ref = cs.y + 3.0); ti = JRTrack.hat3d(TERRAIN, cs.x, cs.z; ref = Inf)
                    println("  [zdiag@] s ", round(hr.lapdist, digits = 1), " lat ", round(hr.lateral, digits = 2), " y ", round(cs.y, digits = 2),
                        "  road(ref y+3) ", round(c0[1], digits = 2), " road(Inf) ", round(ci[1], digits = 2),
                        "  terr(ref y+3) ", round(t0[1], digits = 2), " terr(Inf) ", round(ti[1], digits = 2),
                        "  pitch ", round(rad2deg(pt), digits = 1), " roll ", round(rad2deg(rt), digits = 1), "  wheel gaps ", round.(gaps, digits = 2))
                    for (wx, wz, _, _, nm) in WHEELS
                        q = M * Float32[wx, 0f0, wz, 1f0]; X = Float64(q[1]); Y = Float64(q[2]); Z = -Float64(q[3])
                        a = JRTrack.hat3d(ROADHAT, X, Z; ref = Y + 1.5); bb = JRTrack.hat3d(TERRAIN, X, Z; ref = Y + 1.5)
                        h2 = JRTrack.hat(TRKSURF, X, Z)
                        println("        ", nm, " body (", round(wx, digits = 2), ",", round(wz, digits = 2), ") at lat ", round(h2.lateral, digits = 2),
                            " Y ", round(Y, digits = 2), "  road ", a[3] ? round(a[1], digits = 2) : "-", "  terr ", bb[3] ? round(bb[1], digits = 2) : "-")
                    end
                end
                k = floor(Int, hr.lapdist/50)
                push!(get!(wb, k, Float64[]), maximum(gaps)); push!(get!(wb, -k-1, Float64[]), minimum(gaps))
                push!(get!(tb, k, Float64[]), rad2deg(pt))
            end
            println("  [zdiag] 50 m bins where a DRAWN WHEEL's contact is > 0.25 m above or below the road under it (s: highest wheel max, lowest wheel min, drawn pitch range deg):")
            for k in sort(filter(>=(0), collect(keys(wb))))
                hi = maximum(wb[k]); lo = minimum(wb[-k-1]); pr = tb[k]
                (hi > 0.25 || lo < -0.25) && println("     s ", lpad(50k, 6), "  up ", round(hi, digits = 2), "  down ", round(lo, digits = 2),
                    "  pitch ", round(minimum(pr), digits = 1), " .. ", round(maximum(pr), digits = 1))
            end
        end
        exit(0)
    end

    rep_np, rep_na = REPLAY && hasproperty(repd, :tele) && !isempty(repd.tele) ? (length(repd.tele_p), length(repd.tele_ai)) : (0, 0)
    rep_nt0 = rep_np + rep_na*((REPLAY ? repd.ncar : 1) - 1)
    _rch(v, nm) = (k = findfirst(==(nm), v); k === nothing ? 0 : k)
    rep_kp = (REPLAY_ROLL && rep_np > 0) ? (_rch(repd.tele_p, "pitch"), _rch(repd.tele_p, "roll"), _rch(repd.tele_p, "body"),
                                            _rch(repd.tele_ai, "pitch"), _rch(repd.tele_ai, "roll")) : (0, 0, 0, 0, 0)

    function replay_poses(rt)
        d = repd.data; nf = repd.nframes; f = clamp(rt*repd.fps, 0.0, nf-1.0)
        i0 = floor(Int, f); i1 = min(i0+1, nf-1); g = f - i0
        b0 = i0*rep_st; b1 = i1*rep_st
        lerp(o) = d[b0+o]*(1-g) + d[b1+o]*g
        lerpθ(o) = (a=d[b0+o]; a + (mod(d[b1+o]-a+π, 2π)-π)*g)
        tl = rep_np > 0 ? repd.tele : d; t0 = i0*rep_nt0; t1 = i1*rep_nt0
        player = (lerp(2), lerp(3), lerp(4), lerpθ(5), NaN, NaN)
        if rep_kp[3] > 0
            w0 = tl[t0+rep_kp[3]] > 0.5f0; w1 = tl[t1+rep_kp[3]] > 0.5f0
            if w0 || w1
                A0 = (Float64(d[b0+5]), Float64(tl[t0+rep_kp[1]]), Float64(tl[t0+rep_kp[2]]))
                A1 = (Float64(d[b1+5]), Float64(tl[t1+rep_kp[1]]), Float64(tl[t1+rep_kp[2]]))
                A = (w0 && w1) ? replay_att_lerp(A0, A1, g) : w0 ? A0 : A1
                player = (lerp(2), lerp(3), lerp(4), A[1], A[2], A[3])
            end
        end
        ais = NTuple{6,Float64}[]
        for k in 1:(repd.ncar-1)
            o = 5 + 4*(k-1)
            if rep_kp[4] > 0
                oa = rep_np + rep_na*(k-1)
                A = replay_att_lerp((Float64(d[b0+o+4]), Float64(tl[t0+oa+rep_kp[4]]), Float64(tl[t0+oa+rep_kp[5]])),
                                    (Float64(d[b1+o+4]), Float64(tl[t1+oa+rep_kp[4]]), Float64(tl[t1+oa+rep_kp[5]])), g)
                push!(ais, (lerp(o+1), lerp(o+2), lerp(o+3), A[1], A[2], A[3]))
            else
                push!(ais, (lerp(o+1), lerp(o+2), lerp(o+3), lerpθ(o+4), 0.0, 0.0))
            end
        end
        (player, ais)
    end

    rep_swap = Ref(false)
    REP_AICOCKPIT = get(ENV, "JM_REPLAY_AICOCKPIT", "1") != "0"
    rep_ai_lotus(k) = REPLAY && k >= 1 && k + 1 <= length(repd.names) && occursin("Lotus", repd.names[k+1])

    REP_TEAMCOCKPIT = REP_AICOCKPIT && get(ENV, "JM_REPLAY_TEAMCOCKPIT", "1") != "0"
    rep_team = Ref{Any}(nothing)
    _team_cache = Dict{String,Any}()
    _TEAM_D = Dict("Cooper" => ("coventry", "covd.3do"), "Ferrari" => ("ferrari", "ferd.3do"), "Brabham" => ("brabham", "brad.3do"),
                   "BRM" => ("brm", "brmd.3do"), "Eagle" => ("eagle", "eagd.3do"),
                   "Honda" => ("murasama", "MURd.3DO"))

    CARSET67X = get(ENV, "JM_CARSET67X", "1") != "0"
    _TEAM_D67X = Dict("Honda" => "c28", "BRM" => "c17", "Ferrari" => "c26", "Cooper" => "c20", "Brabham" => "c09", "Eagle" => "c06")
    function _team_src(key)
        dir, f = _TEAM_D[key]
        if CARSET67X && haskey(_TEAM_D67X, key)
            c = _TEAM_D67X[key]; xd = normpath(joinpath(AIBASE, "..", "1967-X", c))
            hit = isdir(xd) ? findfirst(n -> lowercase(n) == c * "d.3do", readdir(xd)) : nothing

            if hit !== nothing
                v = try classic_folder_view(xd) catch; xd end
                return (v, readdir(xd)[hit])
            end
        end
        (joinpath(AIBASE, dir), f)
    end

    TEAM_MIRRORS = get(ENV, "JM_TEAM_MIRRORS", "1") != "0"

    TEAM_TACH_ON = get(ENV, "JM_TEAM_TACH", "1") != "0"

    _TEAM_TACH = Dict("c28" => ("300dshr", (0.041f0, 0.260f0), 0.156f0, -300.0, 30.0, 4500.0, 15500.0),
                      "c26" => ("fdashl",  (0.998f0, 0.270f0), 0.152f0, -300.0, 30.0, 3500.0, 14500.0),
                      "c17" => ("115ldash", (0.986f0, 0.234f0), 0.146f0, 86.0, 24.0),
                      "c20" => ("sifdshl", (0.998f0, 0.186f0), 0.146f0, 103.0, 25.7),
                      "c06" => ("t1gd2hl", (0.998f0, 0.322f0), 0.185f0, 103.0, 25.7),
                      "c09" => ("bbdashl", (0.986f0, 0.205f0), 0.152f0, 110.0, 28.7))

    TEAM_DIALS_ON = get(ENV, "JM_TEAM_DIALS", "1") != "0"

    TEAM_DROP_ON = get(ENV, "JM_TEAM_DROP", "1") != "0"
    _TEAM_DROP = Dict("c26" => ("fbody", "fdasha"), "c09" => ("bbdasha", "bdasha"))

    TEAM_NEEDLE_DROP = get(ENV, "JM_TEAM_NEEDLE_DROP", "1") != "0"

    TEAM_DASH_TOP = get(ENV, "JM_TEAM_DASH_TOP", "1") != "0"
    function _drop_static_needles!(ps, frames)
        nd = 0
        isempty(frames) && return nd
        for p in ps
            v = p.verts; keep = Float32[]
            for t in 0:33:length(v)-33
                P = [Float32[v[t+11k+1], v[t+11k+2], v[t+11k+3]] for k in 0:2]
                em = max(norm(P[1]-P[2]), norm(P[2]-P[3]), norm(P[1]-P[3]))
                thin = em < 0.07f0 && norm(cross(P[2]-P[1], P[3]-P[1])) / 2 / em^2 < 0.15f0
                g = (P[1] + P[2] + P[3]) / 3
                hit = thin && any(f -> (w = g - f.c; h = dot(w, f.n); abs(h) < 0.03f0 && norm(w - h*f.n) < 1.15f0*norm(f.right)), frames)
                hit ? (nd += 1) : append!(keep, @view v[t+1:t+33])
            end
            length(keep) == length(v) || (empty!(v); append!(v, keep))
        end
        filter!(p -> !isempty(p.verts), ps)
        nd
    end
    _TEAM_DIALS = Dict("c28" => [("300dshl", (0.543f0, 0.289f0), 0.094f0, :oiltemp,
                                  ((80.0, -136.0), (90.0, -79.0), (100.0, -28.0), (110.0, 0.0), (120.0, 28.0), (130.0, 79.0), (140.0, 140.0))),
                                 ("300dshl", (0.750f0, 0.254f0), 0.094f0, :fuelpsi, ((0.0, -135.0), (60.0, 135.0))),
                                 ("300dshr", (0.322f0, 0.254f0), 0.094f0, :water,
                                  ((40.0, -135.0), (80.0, -77.0), (100.0, -29.0), (120.0, 29.0), (130.0, 77.0), (140.0, 135.0))),
                                 ("300dshr", (0.523f0, 0.289f0), 0.094f0, :oilpsi, ((0.0, -132.0), (100.0, 132.0)))],

                       "c17" => [("115rdash", (0.295f0, 0.293f0), 0.113f0, :oilpsi,
                                  ((0.0, -90.0), (20.0, -60.0), (40.0, -25.0), (60.0, 25.0), (120.0, 60.0), (160.0, 90.0))),
                                 ("115rdash", (0.295f0, 0.293f0), 0.113f0, :water,
                                  ((50.0, -125.0), (60.0, -150.0), (70.0, -170.0), (80.0, -180.0), (90.0, -210.0), (100.0, -255.0)))],

                       "c26" => [("fdashl", (0.704f0, 0.332f0), 0.098f0, :oilpsi, ((0.0, -90.0), (60.0, -48.0), (80.0, 0.0), (100.0, 48.0), (120.0, 90.0))),
                                 ("fdashl", (0.704f0, 0.352f0), 0.098f0, :oiltemp, ((40.0, -95.0), (80.0, -180.0), (120.0, -263.0))),
                                 ("fdashr", (0.305f0, 0.350f0), 0.105f0, :water,
                                  ((40.0, -129.0), (60.0, -102.0), (80.0, -82.0), (100.0, -12.0), (120.0, 56.0), (140.0, 129.0))),
                                 ("fdashr", (0.518f0, 0.536f0), 0.088f0, :fuelpsi, ((0.0, -40.0), (15.0 * 14.223, 38.0)))],

                       "c20" => [("sifdshl", (0.718f0, 0.210f0), 0.090f0, :water,
                                  ((40.0, -138.0), (60.0, -114.0), (80.0, -90.0), (100.0, -38.0), (110.0, 3.0), (120.0, 42.0), (130.0, 90.0), (140.0, 136.0))),
                                 ("sifdshl", (0.738f0, 0.425f0), 0.088f0, :fuelpsi, ((0.0, -135.0), (60.0, 135.0))),
                                 ("sifdshr", (0.283f0, 0.212f0), 0.090f0, :oiltemp, ((0.0, -135.0), (60.0, 135.0))),
                                 ("sifdshr", (0.282f0, 0.428f0), 0.090f0, :oilpsi, ((10.0, -135.0), (70.0, 135.0)))],

                       "c06" => [("t1gd2hl", (0.347f0, 0.348f0), 0.098f0, :water,
                                  ((40.0, -138.0), (60.0, -114.0), (80.0, -90.0), (100.0, -38.0), (110.0, 3.0), (120.0, 42.0), (130.0, 90.0), (140.0, 136.0))),
                                 ("t1gd2hl", (0.687f0, 0.363f0), 0.100f0, :oiltemp,
                                  ((0.0, -90.0), (20.0, -68.0), (40.0, -20.0), (60.0, 22.0), (100.0, 66.0), (120.0, 90.0))),
                                 ("t1gd2hl", (0.688f0, 0.384f0), 0.100f0, :oilpsi,
                                  ((0.0, -90.0), (40.0, -138.0), (60.0, -180.0), (80.0, -225.0), (100.0, -270.0))),
                                 ("t1gd2hl", (0.204f0, 0.555f0), 0.105f0, :fuelpsi,
                                  ((0.0, -140.0), (20.0, -90.0), (40.0, -45.0), (60.0, 0.0), (80.0, 45.0), (100.0, 90.0), (110.0, 140.0)))],

                       "c09" => [("bbdashl", (0.713f0, 0.201f0), 0.088f0, :oilpsi, ((0.0, -90.0), (160.0, 90.0))),
                                 ("bbdashl", (0.713f0, 0.221f0), 0.088f0, :oiltemp,
                                  ((30.0, -90.0), (50.0, -115.0), (70.0, -147.0), (90.0, -180.0), (110.0, -214.0), (130.0, -244.0), (150.0, -270.0))),
                                 ("bbdashr", (0.241f0, 0.201f0), 0.088f0, :fuelpsi, ((0.0, -90.0), (160.0, 90.0))),
                                 ("bbdashr", (0.241f0, 0.221f0), 0.088f0, :water,
                                  ((30.0, -90.0), (50.0, -115.0), (70.0, -147.0), (90.0, -180.0), (110.0, -214.0), (130.0, -244.0), (150.0, -270.0)))])

    function _team_dial_frame(parts, tex, uv, ru)
        best = Float32[]; bm = -Inf32
        for p in parts
            lowercase(p.tex) == tex || continue
            v = p.verts
            for t in 0:33:length(v)-33
                U = [v[t+11k+10] for k in 0:2]; V = [v[t+11k+11] for k in 0:2]
                d = (U[2]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(V[2]-V[1]); abs(d) < 1f-9 && continue
                b2 = ((uv[1]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(uv[2]-V[1])) / d
                b3 = ((U[2]-U[1])*(uv[2]-V[1]) - (uv[1]-U[1])*(V[2]-V[1])) / d
                m = min(1 - b2 - b3, b2, b3)
                m > bm && (bm = m; best = v[t+1:t+33])
            end
        end
        (isempty(best) || bm < -0.25f0) && return nothing
        tp = best; U = [tp[11k+10] for k in 0:2]; V = [tp[11k+11] for k in 0:2]
        P = [Float32[tp[11k+1], tp[11k+2], tp[11k+3]] for k in 0:2]
        d = (U[2]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(V[2]-V[1])
        b2 = ((uv[1]-U[1])*(V[3]-V[1]) - (U[3]-U[1])*(uv[2]-V[1])) / d
        b3 = ((U[2]-U[1])*(uv[2]-V[1]) - (uv[1]-U[1])*(V[2]-V[1])) / d
        c = P[1] + b2*(P[2]-P[1]) + b3*(P[3]-P[1]); e1 = P[2]-P[1]; e2 = P[3]-P[1]
        right = (( e1*(V[3]-V[1]) - e2*(V[2]-V[1])) / d) * ru
        up    = -((-e1*(U[3]-U[1]) + e2*(U[2]-U[1])) / d) * ru
        nrm = normalize(cross(right, up)); dot(nrm, Float32[0.46, 0.40, 0] - c) < 0 && (nrm = -nrm)
        sgn = dot(Vector{Float32}((Render.rotaxis(nrm, 0.1) * Float32[up..., 0])[1:3]), right) > 0 ? 1f0 : -1f0
        (c = c, right = right, up = up, n = nrm, sgn = sgn)
    end
    function _team_mirror_texs(full)
        b = Dict{String,NTuple{4,Float32}}()
        for p in full, i in 0:11:length(p.verts)-11
            x, y, z = p.verts[i+1], p.verts[i+2], abs(p.verts[i+3])
            o = get(b, p.tex, (Inf32, -Inf32, Inf32, -Inf32))
            b[p.tex] = (min(o[1], x), max(o[2], x), min(o[3], y), max(o[4], z))
        end
        Set(t for (t, (x0, x1, y0, zm)) in b if !startswith(t, "__") && x0 >= 0.3f0 && x1 <= 0.8f0 && y0 >= 0.2f0 && 0.28f0 <= zm <= 0.45f0)
    end

    TEAM_DASHLO = get(ENV, "JM_TEAM_DASHLO", "0") != "0"
    _dash_hi(t) = occursin(r"(ldash|rdash|dshl|dshr|dashl|dashr|dashls|dashrs|d\dhl|d\dhr)$", lowercase(t))
    _dash_lo(t) = occursin(r"(dasha|^\d*dsh|dha|d\dsh)$", lowercase(t))
    REP_TEAMLIVE = get(ENV, "JM_REPLAY_TEAMLIVE", "1") != "0"

    TEAM_SHOULDER = Tuple(parse.(Float64, split(get(ENV, "JM_TEAM_SHOULDER", "0.0,-0.28,0.42"), ",")))
    TEAM_ARM_THICK = parse(Float32, get(ENV, "JM_TEAM_ARM_THICK", "1.2"))
    TEAM_SLEEVE_LIVE = get(ENV, "JM_TEAM_SLEEVE_LIVE", "1") != "0"
    TEAM_GRIP = deg2rad(parse(Float64, get(ENV, "JM_TEAM_GRIP", "20")))
    TEAM_ARMS = get(ENV, "JM_TEAM_ARMS", "1") != "0"
    TEAM_LEVER_AT_LOTUS = get(ENV, "JM_TEAM_LEVER_LOTUS", "1") != "0"
    _TEAM_WHEEL    = Set(["cteerh2", "csterlo", "fsteer2", "fsterlog", "rteerh2", "rsterlo", "sterbrm", "esterh1", "esterlo", "steerhh"])
    _TEAM_WHEEL_LO = Set(["cteerl2", "fsteer1", "rteerl2", "steerl3", "esterl1"])
    _TEAM_VISOR    = Dict("Eagle" => (0.87f0, 0.79f0, 0.69f0))
    _team_win(t) = (t = lowercase(t); occursin(r"win\d*$", t) || startswith(t, "wind") || occursin("glas", t))
    _team_skip(t) = (t = lowercase(t); t in ("plaface", "plahelm", "knees", "neck", "front1", "front3", "lid", "cid", "rid", "fid", "eid", "bid", "hid",
                                             "arms") || endswith(t, "helm") || endswith(t, "sho") || _team_arm(t) || endswith(t, "kne"))
    _team_arm(t) = (t = lowercase(t); t != "arms" && occursin(r"arms?\d*$", t))
    _team_hand(t) = (t = lowercase(t); startswith(t, "hand") || occursin(r"(hand|hnd|hn\d)$", t))
    _team_wheeltex(t) = (t = lowercase(t); t in _TEAM_WHEEL || endswith(t, "ster") || endswith(t, "stlog"))
    function rep_team_items_ai(k)
        (REPLAY && REP_TEAMCOCKPIT && k >= 1 && k + 1 <= length(repd.names)) || return nothing
        key = nothing
        for kk in keys(_TEAM_D); occursin("(" * kk * ")", repd.names[k+1]) && (key = kk); end
        key === nothing && return nothing
        team_items(key)
    end
    rep_team_items(k) = k == 0 ? (PLAYER_CM[] === nothing ? nothing : team_items(PLAYER_CAR)) : rep_team_items_ai(k)

    function team_items(key)
        haskey(_TEAM_D, key) || return nothing
        get!(_team_cache, key) do
            tdir, f = _team_src(key); path = joinpath(tdir, f)
            isfile(path) || return nothing
            ckw = (exclude = ("ltraymap", "lshad"), cockpit_clean = true, grey = (TUB_GREY, TUB_GREY + 0.01f0, TUB_GREY + 0.02f0),
                   visor = REP_TEAMLIVE, visorcol = get(_TEAM_VISOR, key, (0.84f0, 0.84f0, 0.85f0)))

            _tsw = HAND_SWITCH == "" ? get(ENV, "JM_SWITCH_VARS", "") : "14=" * HAND_SWITCH
            ps = withenv("JM_SWITCH_VARS" => _tsw) do
                Render.extract_classic_car(path; ckw..., maxlat = parse(Float32, get(ENV, "JM_COCKPIT_MAXLAT", "0.30")))
            end
            mtex = TEAM_MIRRORS ? _team_mirror_texs(Render.extract_classic_car(path; ckw...)) : Set{String}()

            if MIRROR_RTT && get(ENV, "JM_TEAM_MIRSTATIC", "0") == "0"
                _mstatic(t) = (t = lowercase(t); endswith(t, "ext") || (t != "mirror" && occursin(r"mir|rte\d$", t)))
                filter!(!_mstatic, mtex); filter!(p -> !_mstatic(p.tex), ps)
            end
            if !isempty(mtex)
                filter!(p -> !(p.tex in mtex), ps)
                append!(ps, Render.extract_classic_car(path; ckw..., only = Tuple(mtex)))
            end
            xmin(f) = minimum((p.verts[i+1] for p in ps if f(p.tex) for i in 0:11:length(p.verts)-11); init = Inf32)
            xhi = xmin(_dash_hi)
            dlo = TEAM_DASHLO ? String[] : unique([p.tex for p in ps if _dash_lo(p.tex) && xmin(==(p.tex)) < xhi])

            if get(ENV, "JM_TEAM_NOSE", "0") == "0" && isfinite(xhi)
                stem = lowercase(splitext(f)[1]); cut = xhi + parse(Float32, get(ENV, "JM_TEAM_NOSE_CUT", key == "Honda" ? "-0.15" : "0.12")); nn = 0
                for p in ps
                    lowercase(p.tex) == stem || continue
                    keep = Float32[]
                    for t in 0:33:length(p.verts)-33
                        max(p.verts[t+1], p.verts[t+12], p.verts[t+23]) > cut ? (nn += 1) : append!(keep, @view p.verts[t+1:t+33])
                    end
                    resize!(p.verts, length(keep)); copyto!(p.verts, keep)
                end
                filter!(p -> !isempty(p.verts), ps)
                nn > 0 && println("  [teamnose] ", key, ": the shell's nose (", nn, " triangles ahead of x ", round(cut; digits = 2), ") left out")
            end

            let ex = split(lowercase(get(ENV, "JM_TEAM_EXCLUDE", "")), ","; keepempty = false)
                isempty(ex) || filter!(p -> !((isempty(p.tex) ? "_none_" : lowercase(p.tex)) in ex), ps)
            end
            isempty(dlo) || filter!(p -> !(p.tex in dlo), ps)
            TEAM_DROP_ON && filter!(p -> !(lowercase(p.tex) in get(_TEAM_DROP, lowercase(first(split(basename(tdir), "-"))), ())), ps)

            function fc(q)
                pp = Render.extract_classic_car(q; only = ("plaface",), cockpit_clean = true); xs = Float32[]; ys = Float32[]
                for p in pp, i in 0:11:length(p.verts)-11; push!(xs, p.verts[i+1]); push!(ys, p.verts[i+2]); end
                isempty(xs) ? nothing : (sum(xs)/length(xs), sum(ys)/length(ys))
            end
            ft = fc(path); fl = fc(joinpath(LOTDIR, "lotd.3DO"))
            d = (ft === nothing || fl === nothing) ? (0f0, 0f0, 0f0) : (fl[1] - ft[1], fl[2] - ft[2], 0f0)

            tarms = TEAM_ARMS ? filter(p -> _team_arm(p.tex), ps) : typeof(ps)()
            ps = filter(p -> !_team_skip(p.tex), ps)
            for p in ps, i in 0:11:length(p.verts)-11; p.verts[i+1] += d[1]; p.verts[i+2] += d[2]; end
            for p in tarms, i in 0:11:length(p.verts)-11; p.verts[i+1] += d[1]; p.verts[i+2] += d[2]; end

            if get(ENV, "JM_TEAM_EARS", "0") == "0"
                nears = 0; skin = lowercase(splitext(basename(path))[1])
                for p in ps
                    t_ = lowercase(p.tex)
                    (length(p.verts) >= 33 && (isempty(t_) || t_ == skin)) || continue
                    keep = Float32[]
                    for t in 0:33:length(p.verts)-33
                        cx = (p.verts[t+1] + p.verts[t+12] + p.verts[t+23])/3; cy = (p.verts[t+2] + p.verts[t+13] + p.verts[t+24])/3
                        cz = abs(p.verts[t+3] + p.verts[t+14] + p.verts[t+25])/3
                        dark = !isempty(t_) || max(p.verts[t+7], p.verts[t+8], p.verts[t+9]) < 0.2f0
                        (dark && 0.40f0 < cx < 0.60f0 && 0.18f0 < cz < 0.31f0 && cy > 0.24f0) ? (nears += 1) : append!(keep, @view p.verts[t+1:t+33])
                    end
                    resize!(p.verts, length(keep)); p.verts .= keep
                end
                filter!(p -> !isempty(p.verts), ps)
                nears > 0 && println("  [teamears] ", key, ": ", nears, " mirror-bracket triangles by the side screens left out")
            end

            if TEAM_GRIP != 0
                wq = filter(p -> _team_wheeltex(p.tex) && !endswith(lowercase(p.tex), "stlog") && !_team_hand(p.tex), ps)
                c0 = zeros(3); n0 = 0
                for p in wq, i in 0:11:length(p.verts)-11; c0 .+= p.verts[i+1:i+3]; n0 += 1; end
                if n0 > 0 && length(wq[1].verts) >= 33
                    c0 ./= n0; v = wq[1].verts; a1 = v[12:14] .- v[1:3]; b1 = v[23:25] .- v[1:3]
                    k = [a1[2]*b1[3]-a1[3]*b1[2], a1[3]*b1[1]-a1[1]*b1[3], a1[1]*b1[2]-a1[2]*b1[1]]
                    k ./= max(sqrt(sum(k .^ 2)), 1e-9); sum(k .* SWAXIS) < 0 && (k = -k)
                    rod(x, th) = x .* cos(th) .+ [k[2]*x[3]-k[3]*x[2], k[3]*x[1]-k[1]*x[3], k[1]*x[2]-k[2]*x[1]] .* sin(th) .+ k .* (sum(k .* x)*(1 - cos(th)))
                    for p in ps
                        _team_hand(p.tex) || continue
                        for tb in 0:33:length(p.verts)-33
                            th = sign(p.verts[tb+3] + p.verts[tb+14] + p.verts[tb+25]) * TEAM_GRIP
                            for i in tb:11:tb+22
                                x = rod(Float64.(p.verts[i+1:i+3]) .- c0, th) .+ c0; p.verts[i+1:i+3] .= Float32.(x)
                                p.verts[i+4:i+6] .= Float32.(rod(Float64.(p.verts[i+4:i+6]), th))
                            end
                        end
                    end
                end
            end

            let fr = get(ENV, "JM_TEAM_FIST_ROT", "")
                if !isempty(fr)
                    f_ = split(fr, "_"); ax = f_[1] == "x" ? [1.0, 0, 0] : f_[1] == "y" ? [0, 1.0, 0] : [0, 0, 1.0]
                    ang = deg2rad(parse(Float64, f_[2])); mirr = length(f_) > 2 && f_[3] == "mirror"
                    rot(x, th) = x .* cos(th) .+ [ax[2]*x[3]-ax[3]*x[2], ax[3]*x[1]-ax[1]*x[3], ax[1]*x[2]-ax[2]*x[1]] .* sin(th) .+ ax .* (sum(ax .* x)*(1 - cos(th)))
                    for sd in (-1, 1)
                        Q = [Float64.(p.verts[i+1:i+3]) for p in ps if _team_hand(p.tex) for i in 0:11:length(p.verts)-11 if sign(p.verts[i+3]) == sd]
                        isempty(Q) && continue
                        cc = sum(Q) ./ length(Q); th = (mirr && sd < 0) ? -ang : ang
                        for p in ps
                            _team_hand(p.tex) || continue
                            for tb in 0:33:length(p.verts)-33
                                sign(p.verts[tb+3] + p.verts[tb+14] + p.verts[tb+25]) == sd || continue
                                for i in tb:11:tb+22
                                    p.verts[i+1:i+3] .= Float32.(rot(Float64.(p.verts[i+1:i+3]) .- cc, th) .+ cc)
                                    p.verts[i+4:i+6] .= Float32.(rot(Float64.(p.verts[i+4:i+6]), th))
                                end
                            end
                        end
                    end
                end
            end
            sleeves = Any[]
            if !isempty(tarms) && TEAM_SLEEVE_LIVE

                hands = filter(p -> _team_hand(p.tex), ps)
                for side in (-1, 1)
                    sp_ = Render.TrackPart[]
                    for p in tarms
                        keep = Float32[]
                        for t in 0:(length(p.verts) ÷ 33)-1
                            b = t*33; sign(p.verts[b+3] + p.verts[b+14] + p.verts[b+25]) == side && append!(keep, @view p.verts[b+1:b+33])
                        end
                        isempty(keep) || push!(sp_, Render.TrackPart(keep, p.tex, p.col))
                    end
                    se = isempty(sp_) ? nothing : sleeve_ends(sp_, side); se === nothing && continue
                    W, E, nf = se
                    S = (TEAM_SHOULDER[1], TEAM_SHOULDER[2], side*TEAM_SHOULDER[3])
                    Q = [Float64[p.verts[i+1], p.verts[i+2], p.verts[i+3]] for p in hands for i in 0:11:length(p.verts)-11 if sign(p.verts[i+3]) == side]
                    isempty(Q) && continue
                    c = sum(Q) ./ length(Q); dS = collect(S) .- c; dS ./= sqrt(sum(abs2, dS))
                    pr = [sum((q .- c) .* dS) for q in Q]; lo, hi = extrema(pr)
                    sel = [Q[i] for i in eachindex(Q) if pr[i] >= hi - 0.25*(hi - lo)]
                    cuff = Tuple(sum(sel) ./ length(sel))
                    sleeve_mat(W, E, cuff, S, nf; thick = TEAM_ARM_THICK); nref = SLEEVE_LASTR[] * collect(nf)
                    push!(sleeves, (parts = sp_, W = W, E = E, nf = nf, cuff = cuff, S = S, nref = nref, thick = TEAM_ARM_THICK))
                end
            end
            if !isempty(tarms) && !TEAM_SLEEVE_LIVE
                hands = filter(p -> _team_hand(p.tex), ps)
                for side in (-1, 1)
                    ends = Render.sleeve_axis_ends(tarms, side); ends === nothing && continue
                    W, E = ends
                    hp = [Float64[p.verts[i+1], p.verts[i+2], p.verts[i+3]] for p in hands for i in 0:11:length(p.verts)-11 if sign(p.verts[i+3]) == side]
                    isempty(hp) && continue
                    H = Tuple(sum(hp) ./ length(hp))
                    S = (TEAM_SHOULDER[1], TEAM_SHOULDER[2], side*TEAM_SHOULDER[3])
                    for p in tarms
                        keep = Float32[]
                        for t in 0:(length(p.verts) ÷ 33)-1
                            b = t*33
                            sign(p.verts[b+3] + p.verts[b+14] + p.verts[b+25]) == side || continue
                            append!(keep, @view p.verts[b+1:b+33])
                        end
                        isempty(keep) || push!(ps, Render.TrackPart(Render.lay_along!(keep, W, E, H, S, TEAM_ARM_THICK), p.tex, p.col))
                    end
                end
            end

            idx = Render.classic_texture_index(tdir)

            mglass = (MIRROR_RTT && !isempty(mtex)) ? mirror_glass_quads(filter(p -> lowercase(p.tex) == "mirror", ps), mirtex) : Render.Item[]
            isempty(mglass) || filter!(p -> lowercase(p.tex) != "mirror", ps)
            tspec = TEAM_TACH_ON ? get(_TEAM_TACH, lowercase(first(split(basename(tdir), "-"))), nothing) : nothing
            tf = tspec === nothing ? nothing : _team_dial_frame(ps, lowercase(tspec[1]), tspec[2], tspec[3])
            tach = tf === nothing ? nothing :
                (f = tf, d0 = tspec[4], dk = tspec[5], lo = length(tspec) >= 7 ? tspec[6] : 0.0, hi = length(tspec) >= 7 ? tspec[7] : 16000.0,
                 item = let (vao, n) = Render.upload(_dial_needle_verts(tf, 0.80f0, 0.20f0, 0.040f0, 0.015f0, (0.96f0, 0.96f0, 0.94f0); lift = 0.003f0))
                     Render.Item(vao, n, GLuint(0), (0.96f0, 0.96f0, 0.94f0)) end)
            dials = Any[]
            if TEAM_DIALS_ON
                for (tx, uv, ru, sym, sc) in get(_TEAM_DIALS, lowercase(first(split(basename(tdir), "-"))), [])
                    df = _team_dial_frame(ps, lowercase(tx), uv, ru); df === nothing && continue
                    vao, n = Render.upload(_dial_needle_verts(df, 0.78f0, 0.22f0, 0.045f0, 0.018f0, (0.96f0, 0.96f0, 0.94f0); lift = 0.003f0))
                    push!(dials, (f = df, scale = sc, sym = sym, item = Render.Item(vao, n, GLuint(0), (0.96f0, 0.96f0, 0.94f0))))
                end
            end
            nstat = TEAM_NEEDLE_DROP ? _drop_static_needles!(ps, vcat(tach === nothing ? Any[] : Any[tach.f], Any[d.f for d in dials])) : 0
            REP_TEAMLIVE || return (body = Render.build_classic(ps, idx), wheel = Render.Item[], hands = Render.Item[], shift = Render.Item[], glass = Render.Item[], wglass = Render.Item[],
                                    swc = Float32[0, 0, 0], swa = Float32[1, 0, 0], shp = Float32[0, 0, 0], mglass = mglass, tach = tach, dials = dials,
                                    dash = Render.Item[], arms = Render.Item[], lever_left = false, lever_d = Float32[0, 0, 0], sleeves = Any[], shlen = 0f0, shtip = Float32[0, 0, 0], shR = Render.ident(),
                                    handsLR = [Render.Item[], Render.Item[]], fistc = [Float32[0, 0, 0], Float32[0, 0, 0]])
            lw(t) = lowercase(t)
            filter!(p -> !(lw(p.tex) in _TEAM_WHEEL_LO), ps)
            isw(p) = _team_wheeltex(p.tex) || _team_hand(p.tex)
            iss(p) = endswith(lw(p.tex), "shift")
            isg(p) = p.tex == "__visor__"

            isgw(p) = TEAM_WIN_ALPHA > 0 && _team_win(p.tex)
            wp = filter(isw, ps); sp = filter(iss, ps); gp = filter(isg, ps); wgp = filter(isgw, ps)
            bp = filter(p -> !(isw(p) || iss(p) || isg(p) || isgw(p)), ps)
            if get(ENV, "JM_TEAM_PARTDIAG", "0") != "0"
                for p in wgp
                    r = Render.tex_rgba(idx, p.tex); r === nothing && continue
                    w, h, px = r; al = [Int(px[4i+4]) for i in 0:w*h-1]
                    println("  [teamwin] ", key, " ", p.tex, ": ", length(p.verts) ÷ 33, " tris, texture ", w, "x", h, " alpha mean ",
                            round(sum(al)/length(al)/255; digits = 3), " p10/p50/p90 ", join([round(sort(al)[max(1, round(Int, length(al)*q))]/255; digits = 2) for q in (0.1, 0.5, 0.9)], "/"))
                end
            end

            q = filter(p -> _team_wheeltex(p.tex) && !endswith(lw(p.tex), "stlog"), wp); c = zeros(3); nn = 0
            for p in q, i in 0:11:length(p.verts)-11; c .+= p.verts[i+1:i+3]; nn += 1; end
            swc = Float32.(nn > 0 ? c ./ nn : [0.0, 0.0, 0.0]); swa = Float32[1, 0, 0]
            if !isempty(q) && length(q[1].verts) >= 33
                v = q[1].verts; a = v[12:14] .- v[1:3]; b = v[23:25] .- v[1:3]; n3 = [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1]]
                swa = Float32.(n3 ./ max(sqrt(sum(n3 .^ 2)), 1e-9)); sum(swa .* SWAXIS) < 0 && (swa = -swa)
            end
            ys = [p.verts[i+2] for p in sp for i in 0:11:length(p.verts)-11]
            shp = isempty(ys) ? Float32[0, 0, 0] : let y0 = minimum(ys), L = [p.verts[i+1:i+3] for p in sp for i in 0:11:length(p.verts)-11 if p.verts[i+2] < y0 + 0.01f0]
                Float32.(sum(L) ./ length(L)) end
            if get(ENV, "JM_TEAM_PARTDIAG", "0") == "2"
                grp = Dict{NTuple{3,Float32},Vector{Float32}}()
                for p in bp, t in 0:33:length(p.verts)-33
                    isempty(p.tex) || continue
                    k = (round(p.verts[t+7]; digits = 2), round(p.verts[t+8]; digits = 2), round(p.verts[t+9]; digits = 2))
                    append!(get!(grp, k, Float32[]), p.verts[t+1:t+33])
                end
                for (k, v) in sort(collect(grp); by = e -> -length(e[2]))
                    xs = v[1:11:end]; ys = v[2:11:end]; zs = v[3:11:end]
                    for t in 0:33:length(v)-33
                        cx = (v[t+1] + v[t+12] + v[t+23])/3; cy = (v[t+2] + v[t+13] + v[t+24])/3; cz = (v[t+3] + v[t+14] + v[t+25])/3
                        println("  [teamuntri] ", join([round(v[t+o]; digits = 3) for o in (1, 2, 3, 12, 13, 14, 23, 24, 25)], " "))
                    end
                    println("  [teamuntex] ", key, " col ", k, ": ", length(v) ÷ 33, " tris  x ", round(minimum(xs); digits = 2), "..", round(maximum(xs); digits = 2),
                            "  y ", round(minimum(ys); digits = 2), "..", round(maximum(ys); digits = 2), "  |z| ", round(minimum(abs.(zs)); digits = 2), "..", round(maximum(abs.(zs)); digits = 2))
                end
            end
            get(ENV, "JM_TEAM_PARTDIAG", "0") != "0" && println("  [teamparts] ", key, ": ", join(["$(p.tex):$(length(p.verts) ÷ 33)" for p in bp], " "))

            dp = TEAM_DASH_TOP ? filter(p -> _dash_hi(p.tex), bp) : empty(bp)
            ap = TEAM_DASH_TOP ? filter(p -> _team_arm(p.tex), bp) : empty(bp)
            bp = filter(p -> !any(q -> q === p, dp) && !any(q -> q === p, ap), bp)
            shtip = let V = [p.verts[i+1:i+3] for p in sp for i in 0:11:length(p.verts)-11]
                isempty(V) ? copy(shp) : V[argmax([sum(abs2, v .- shp) for v in V])] end
            shR = let u = Float64.(shtip .- shp), n = sqrt(sum(abs2, u))
                ax = n < 1e-4 ? zeros(3) : [-u[3], 0.0, u[1]] ./ n; sa = sqrt(sum(abs2, ax))
                sa < 1e-6 ? Render.ident() : Render.rotaxis(ax, atan(sa, u[2] / n))
            end

            hpart = filter(p -> _team_hand(p.tex), wp)
            sidep(s) = [Render.TrackPart(Float32[x for t in 0:(length(p.verts) ÷ 33)-1 if sign(p.verts[t*33+3] + p.verts[t*33+14] + p.verts[t*33+25]) == s
                                                   for x in p.verts[t*33+1:t*33+33]], p.tex, p.col) for p in hpart]
            hlr = [filter(p -> !isempty(p.verts), sidep(s)) for s in (-1, 1)]
            fistc = [let P = [p.verts[i+1:i+3] for p in hs for i in 0:11:length(p.verts)-11]; isempty(P) ? Float32[0, 0, 0] : Float32.(sum(P) ./ length(P)) end for hs in hlr]
            it = (body = Render.build_classic(bp, idx), wheel = Render.build_classic(filter(p -> !_team_hand(p.tex), wp), idx),
                  hands = Render.build_classic(filter(p -> _team_hand(p.tex), wp), idx), shift = Render.build_classic(sp, idx),
                  glass = Render.build_classic(gp, idx), wglass = Render.build_classic(wgp, idx), swc = swc, swa = swa, shp = shp, mglass = mglass, tach = tach, dials = dials,
                  dash = Render.build_classic(dp, idx), arms = Render.build_classic(ap, idx),
                  lever_left = lowercase(key) in TEAM_LEVER_LEFT, lever_d = get(TEAM_LEVER_DT, lowercase(key), Float32[0, 0, 0]),
                  sleeves = [(items = Render.build_classic(q.parts, idx), W = q.W, E = q.E, nf = q.nf, cuff = q.cuff, S = q.S, nref = q.nref, thick = q.thick) for q in sleeves],
                  shlen = isempty(ys) ? 0f0 : Float32(maximum(ys) - minimum(ys)),
                  shtip = shtip, shR = shR, handsLR = [Render.build_classic(h, idx) for h in hlr], fistc = fistc)
            println("  [teamcockpit] ", key, ": ", occursin("classicview", path) ? "1967-X/" * first(split(basename(dirname(path)), "-")) * "/" * basename(path) * " (view)" : relpath(path, AIBASE), " -- ", length(bp), " parts + wheel/gloves ", length(wp), ", lever ", length(sp),
                    ", glass ", length(gp), ", mirrors ", join(sort(collect(mtex)), "/"), " (live ", length(mglass), "), lo-res dash dropped ", join(dlo, "/"), ", tach ", tach === nothing ? "none" : "$(tspec[1]) at $(round.(tach.f.c; digits = 3))", ", dials ", length(dials), " (static needle slivers dropped ", nstat, ")",
                    "; seat moved ", round.(d; digits = 3), " m to the Lotus's; wheel centre ", round.(swc; digits = 3), "; lever foot ", round.(shp; digits = 3), " ", join(["$(p.tex):$(length(p.verts) ÷ 33) tris y $(round(minimum(p.verts[2:11:end]); digits = 3))..$(round(maximum(p.verts[2:11:end]); digits = 3))" for p in sp], " "))
            it
        end
    end
    (PLAYER_CM[] !== nothing && !REPLAY) && (rep_team[] = team_items(PLAYER_CAR))
    function rep_ai_inputs(rt, k)
        h = 0.1
        _, a0 = replay_poses(max(rt - h, 0.0)); _, a1 = replay_poses(min(rt + h, rep_dur))
        dθ = mod(a1[k][4] - a0[k][4] + π, 2π) - π
        ds = hypot(a1[k][1] - a0[k][1], a1[k][3] - a0[k][3]); dtt = max(min(rt + h, rep_dur) - max(rt - h, 0.0), 1e-3)
        v = ds/dtt; r = dθ/dtt
        δ = v > 2.0 ? atan(2.41*r/v) : 0.0
        g = 1
        for gg in 1:length(DriveRT3D.GEARS)
            (v/DriveRT3D.RW_R*DriveRT3D.GEARS[gg]*DriveRT3D.FINAL[]*60/(2π) >= 5000.0) && (g = gg)
        end
        rpm = clamp(v/DriveRT3D.RW_R*DriveRT3D.GEARS[g]*DriveRT3D.FINAL[]*60/(2π), 2000.0, 9500.0)
        (v = v, steer = clamp(δ/DriveRT3D.MAXSTEER, -1.0, 1.0), rpm = rpm, gear = g)
    end

    REP_NT = (REPLAY && hasproperty(repd, :tele) && !isempty(repd.tele)) ? length(repd.tele_p) + length(repd.tele_ai)*(repd.ncar-1) : 0
    REP_CH = REP_NT == 0 ? Dict{String,Int}() : Dict(nm => k for (k, nm) in enumerate(repd.tele_p))
    REP_TYRES = all(c -> haskey(REP_CH, c), REPLAY_TYRE_CH)
    REP_TT = all(c -> haskey(REP_CH, c), REPLAY_TT_CH)

    rep_sk = (REP_NT > 0 && AILINE !== nothing && haskey(REP_CH, "prog")) ?
             (REP_CH["prog"], something(findfirst(==("s"), repd.tele_ai), 0), something(findfirst(==("lap"), repd.tele_ai), 0)) : (0, 0, 0)
    rep_tyres = Ref{Any}(nothing)
    rep_tyshow = Ref(get(ENV, "JM_REPLAY_TYRES", "1") != "0"); rep_pw = Ref(false)
    function replay_tele(rt)
        tl = repd.tele; nf = length(tl) ÷ REP_NT; f = clamp(rt*repd.fps, 0.0, nf-1.0)
        i0 = floor(Int, f); i1 = min(i0+1, nf-1); g = f - i0
        ch(nm) = (k = get(REP_CH, nm, 0); k == 0 ? 0.0 : Float64(tl[i0*REP_NT+k])*(1-g) + Float64(tl[i1*REP_NT+k])*g)
        kg = get(REP_CH, "gear", 0)
        (speed = ch("speed"), rpm = ch("rpm"), steer = ch("steer"), throttle = ch("throttle"), brake = ch("brake"),
         clutch = ch("clutch"), gear = kg == 0 ? 0 : round(Int, tl[(g < 0.5 ? i0 : i1)*REP_NT+kg]),
         tyres = REP_TYRES ? ntuple(w -> ntuple(q -> ch(REPLAY_TYRE_CH[5(w-1)+q]), 5), 4) : nothing,
         tt = REP_TT ? ntuple(j -> ch(REPLAY_TT_CH[j]), 24) : nothing)
    end
    REPLAY && println("  ▶ REPLAY: $(basename(REPLAY_FILE)) — $(repd.nframes) frames, $(round(rep_dur,digits=1))s, $(repd.ncar) cars\n" *
        "  SPACE play/pause · ←/→ seek · PgUp/PgDn ±10 s · Home/End start/end · ↑/↓ speed · V switch ANGLE (cockpit/chase/TV/F10/nose/RR-susp) · C switch CAR · W tyres panel · F band on/off · Esc quit")

    telem = SMOKE ? nothing : open("$(TRACKSEL)_racer_$(round(Int,time())).txt", "w")

    _tview = get(ENV, "JM_VIEW", "?")
    _tjoy  = isfile(joinpath(@__DIR__, "joystick.conf"))
    telem !== nothing && write(telem,
        "# $(TRACKSEL)_racer telemetry — Lotus 49 @ $(TRACKSEL)\n" *
        "# run: ai=$(N_AI_REQ) ai_pct=$(AI_PCT) ai_rel=$(AI_REL) ai_amax=$(AI_AMAX) view=$(_tview) joystick=$(_tjoy)\n" *
        "# t\tlap\tlapdist\tkmh\tthr\tbrk\tsteer\tclu\tgear\trpm\tx\tz\tlat\talong\tontrack\n")
    println("\n  Drive:  W/S gas·brake   A/D steer   E/Q shift   C clutch   R restart race   ⇧R recover-to-track   V view   G auto⇄manual   M mute   F band on/off   Esc quit"); flush(stdout)
    println("  AUTO gearbox by default — just press the throttle and go (no clutch needed).  Press G for")
    println("  MANUAL: hold the clutch (C / stick button) to shift E/Q (release it too low and it bogs).")
    println("  Lap times top-left: white = last, green = best.  Telemetry → ./$(TRACKSEL)_racer_*.txt")
    println("  (Logitech joystick works natively — push=throttle, pull=brake, roll=steer)\n")
    EngineAudio.start(ENG)

    show_after = parse(Int, get(ENV, "JM_SHOW_AFTER", "2")); win_shown = Ref(false); show_t0 = time()
    reveal!() = (win_shown[] = true; (SMOKE && !haskey(ENV, "JM_SMOKE_SHOW")) || (GLFW.ShowWindow(win); go_fullscreen!()))
    show_after <= 0 && reveal!()
    crash_plat = Ref(NaN); crash_thru = Ref(false); crash_invis = Ref(0); crash_ninv = Ref(0)
    crash_i = Ref(0); crash_t0 = Ref(0); crash_rec = Ref((0.0, 0.0, 0.0)); crash_sum = Ref((0, 0, 0.0, 0.0))
    crash_jit = Ref((Inf, -Inf, Inf, -Inf))
    if CRASH_ON && CLINE !== nothing && CLASSICW_ON
        if startswith(CRASH_SPEC, "auto")
            stp = parse(Float64, split(CRASH_SPEC, ":")[2]); ang = parse(Float64, get(ENV, "JM_CRASH_ANG", "45"))
            vv = parse(Float64, get(ENV, "JM_CRASH_V", "55"))
            for s0 in stp/2:stp:CLINE.total, sd in (-1.0, 1.0); push!(CRASHES, (s0, sd, ang, vv)); end
        else
            for sp in split(CRASH_SPEC, ";", keepempty = false)

                f = split(sp, ":"); push!(CRASHES, (parse(Float64, f[1]), uppercase(f[2]) == "L" ? 1.0 : -1.0, parse(Float64, f[3]), parse(Float64, f[4])))
            end
        end
        println("  JM_CRASH: ", length(CRASHES), " scenarios, ", CRASH_FRAMES, " frames each"); flush(stdout)
    end
    aicam_pose = Ref{Union{Nothing,NTuple{6,Float64}}}(nothing); ai_poses_prev = Ref(NTuple{6,Float64}[])

    shot_idx = Ref(0); shot_t0 = Ref(0); shots_done = Ref(isempty(SHOTS))
    if SMOKE && !isempty(SHOTS)
        shot_idx[] = 1; shot_t0[] = 0
        CTL.view = SHOTS[1].view
        for (k, v) in SHOTS[1].env; ENV[k] = v; end
        place_at_s!(SHOTS[1].s, parse(Float64, get(ENV, "JM_SHOT_LAT", "0")))
        println("  JM_SHOTS: ", length(SHOTS), " shots → ", SHOTS_DIR); flush(stdout)
    end

    if get(ENV, "JM_WARM", "1") != "0"
        _wf = joinpath(@__DIR__, "warm_statements.jl"); _wok = 0; _wbad = 0; _wt = time()
        if isfile(_wf)
            for _l in eachline(_wf)
                startswith(_l, "precompile(") || continue
                try
                    Core.eval(Main, Meta.parse(_l)) === true ? (_wok += 1) : (_wbad += 1)
                catch
                    _wbad += 1
                end
            end
        end
        println("  [warm] ", _wok, " in-race methods compiled ahead (", _wbad, " skipped) in ",
                round(Int, 1000 * (time() - _wt)), " ms"); flush(stdout)
    end
    ctrace_start!("ready", 0.0); ctrace_tick!()
    view0 = CTL.view
    empty!(KEY_LATCH)
    while !GLFW.WindowShouldClose(win)
        GLFW.PollEvents()
        (key(GLFW.KEY_ESCAPE) || took_key!(GLFW.KEY_ESCAPE)) && (println("  [exit] ESC at t=", round(cs.t, digits = 2)); flush(stdout); break)
        kf = key(GLFW.KEY_F); (HUDBAND_ON && kf && !BAND_PF[]) && (BAND_HIDDEN[] = !BAND_HIDDEN[]); BAND_PF[] = kf

        (KEY_AT !== nothing && KEY_AT_T[] == 0 && cs.t >= KEY_AT[1]) && (KEY_AT_T[] = time(); println("  [test] key ", KEY_AT[2], " pressed at t=", round(cs.t, digits = 2)); flush(stdout))

        _ik = took_key!(GLFW.KEY_I) | key(GLFW.KEY_I); _lk = took_key!(GLFW.KEY_LEFT)

        (race_go[] && cs.v < 0.5) ? (STOP_T[] < 0 && (STOP_T[] = cs.t)) : (STOP_T[] = -1.0)
        _stopped = STOP_T[] >= 0 && cs.t - STOP_T[] >= 1.0
        (_ik && !IPRESS_SEEN[]) && (println("  [instant] I pressed at t=", round(cs.t, digits = 2), ": race over ", race_done, ", wrecked ", WRECKED[],
                                            ", stopped ", _stopped, ", recording ", REPLAY_REC, ", instant replay on ", INSTANT_ON); flush(stdout)); IPRESS_SEEN[] = _ik
        (INSTANT_ON && REPLAY_REC && ((race_done || WRECKED[]) && (_ik || _lk) || _stopped && _ik)) && (INSTANT_REQ[] = true; break)

        (!win_shown[] && show_after >= 2) && (CTL.view = frames == 0 ? 1 - view0 : view0)

        if SMOKE && isempty(SHOTS) && !CRASH_ON && frames >= SMOKE_FRAMES
            INSTANT_TEST && REPLAY_REC && (INSTANT_REQ[] = true)
            break
        end
        CRASH_ON && crash_i[] > length(CRASHES) && break
        SMOKE && shots_done[] && !isempty(SHOTS) && break

        AUTODRIVE && race_done && !haskey(ENV, "JM_NO_AUTOEXIT") && break

        now = time()
        ALLOCPROF > 0 && (ALLOC_MARK[1] = Base.gc_bytes())
        dt = FIXED_DT > 0 ? FIXED_DT : clamp(now-last, 0.0, 0.05)
        last = now
        GEAR_NOW[] = cs.gear; RPM_NOW[] = cs.rpm; V_NOW[] = cs.v
        inp, rst, recover, restart = read_input()
        (rst, recover, restart) = input_hooks_step!(cs, dt, rst, recover, restart, race_go, LASTZ)
        inp = input_overrides(cs, inp, cd_armed, phase, race_go, HOLD_START)

        if restart
            ctrace_start!("restart")
            _rs = restart_session!(cs, cs0, groundz, groundz_phys, AICARS, AILINE, CLINE, DO_QUAL, HOLD_START, FUEL1, FUEL_ON, CLASSICAI_F, PLAYER_G, REPLAY_NCAR, ai_best, ai_lap_prev, ai_lapt0, burn_lap, fuel_laps, fuel, cd_armed, cd_left, cd_said, cd_t0, flush_ibt!, form_grid!, ibt_samples, launch_done, new_replay_path, phase, player_finpos, player_grid, player_laps, race_go, replay_buf, replay_names, replay_out, replay_saved, replay_t, tele_buf, player_s_prev)
            lap_t0 = _rs.lap_t0; last_lap = _rs.last_lap; best_lap = _rs.best_lap; prev_laps = _rs.prev_laps
            race_done = _rs.race_done; player_prog = _rs.player_prog; player_s_prev = _rs.player_s_prev
        end

        if AUTODRIVE && CLINE !== nothing
            inp = autodrive_step!(cs, CLINE, frames, dt, groundz, inp, ONTRACK)

            (START_ARM && HOLD_START && phase[] == :race && !race_go[]) &&
                (inp = JRTrack.DriveInput(throttle = 0.0, brake = 1.0, steer = inp.steer, clutch = inp.clutch,
                                             shift_up = false, shift_down = false, autoshift = inp.autoshift))
        end

        if SKIDDRIVE !== nothing
            inp = skiddrive_input(cs, frames)
        end
        if REPLAY
            (inp, rep_ai_raw) = replay_step!(cs, inp, dt, REP_AICOCKPIT, REP_NT, REP_TEAMCOCKPIT, rep_ai_inputs, rep_ai_lotus, rep_cam, rep_dur, rep_focus, rep_pdn, rep_pend, rep_phome, rep_play, rep_pn, rep_ppd, rep_ppu, rep_psp, rep_pup, rep_pv, rep_pw, rep_rt, rep_speed, rep_swap, rep_team, rep_team_items, rep_tyres, rep_tyshow, repd, replay_poses, replay_tele)
            minimum(rep_sk) > 0 && replay_standings!(repd.tele, REP_NT, rep_sk[1], length(repd.tele_p), length(repd.tele_ai), rep_sk[2],
                rep_sk[3], repd.ncar, clamp(round(Int, rep_rt[]*repd.fps), 0, length(repd.tele) ÷ REP_NT - 1), Float64(AILINE.total))
            (rep_sk[1] > 0 && CLINE !== nothing) && replay_lapclock!(LAPCLOCK, repd.tele, REP_NT, rep_sk[1], repd.data, rep_st,
                clamp(floor(Int, rep_rt[]*repd.fps), 0, length(repd.tele) ÷ REP_NT - 1), Float64(CLINE.total), HUD_SECFRACS)
            @goto skipsim
        end

        accelNow = key(GLFW.KEY_T) || key(GLFW.KEY_ENTER) || key(GLFW.KEY_KP_ENTER)
        _pe = practice_end_step!(cs, accelNow, enterPrev, last_lap, best_lap, race_done, lap_t0, phase, player_grid, cd_t0, launch_done, FUEL_ON, form_grid!, refuel!)
        last_lap = _pe.last_lap; best_lap = _pe.best_lap; race_done = _pe.race_done; lap_t0 = _pe.lap_t0
        enterPrev = accelNow

        ctrace_tick!()
        lap_t0 = countdown_step!(cs, AICARS, HOLD_START, ai_release, cd_armed, cd_left, cd_said, cd_t0, phase, race_go, lap_t0)
        lap_t0 = green_light_step!(cs, inp, AICARS, HOLD_START, ai_release, phase, race_go, lap_t0, SMOKE_HOLD)

        inp = launch_assist(cs, inp, rst, race_go, launch_done)

        inp = fuel_step!(cs, inp, dt, FUEL1, FUEL_ON, fuel, fuel_mass_t, race_go, rst)

        joytrace_report!(cs, inp, race_go)
        rst && FUEL_ON && refuel!()
        if recover && CLINE !== nothing && CAR3D
            recover_step!(cs, CLINE, groundz, PLAYER_G)

        elseif rst; respawn_step!(cs, cs0, groundz, groundz_phys, PLAYER_G)
        else

            (ffb_jolt, _tgc) = player_drive_step!(cs, inp, dt, rst, groundz, groundz_phys, roll_ter, ffb_jolt, race_go, BND_FX, BND_FY, BND_MZ, OFFDIST, OFFTRACK_PROBE, ONTRACK, PLAYER_CDA, PLAYER_HDG, STEP_GUARD_HITS)
            jolt_report!(cs, frames, CLINE, STEP_GUARD_HITS, dt, rst)
            CLASSICW_PHYS && (CLASSICW_T[] = (CLASSICW_T[][1] + time() - _tgc, CLASSICW_T[][2]))
            if CRASH_ON && !isempty(CRASHES) && crash_i[] <= length(CRASHES) && frames >= 2
                crash_step!(cs, frames, place_at_s!, crash_i, crash_invis, crash_jit, crash_ninv, crash_plat, crash_rec, crash_sum, crash_t0, crash_thru)
            end
            if !SKIDPAD

            (player_prog, player_s_prev, ffb_jolt) = track_position_step!(cs, dt, rst, race_go, player_prog, player_s_prev, ffb_jolt, groundz_phys,
                                                                    BND_FX, BND_FY, BND_MZ, CLINE, LASTGX, LASTGZ, OFFDIST, ONTRACK, TRACE_PROG, prog_bad, prog_t0)
            end
        end

        spin -= cs.v*dt/0.33
        ENG.rpm[] = isfinite(cs.rpm) ? cs.rpm : 700.0

        if ffb !== nothing && ffb.ok
            (fy_lp, ffb_f) = ffb_step!(ffb, cs, inp, dt, fy_lp, ffb_f, ffb_jolt)
        end
        ffb_jolt *= exp(-(dt > 1e-4 ? dt : 1/60)/0.06)

        if ibt_samples !== nothing && !rst
            ibt_sample!(ibt_samples, cs, inp, fuel)
        end

        _lt = lap_timing_step!(cs, prev_laps, lap_t0, last_lap, best_lap, race_done, phase, player_laps, telem, AICARS, standings, player_finpos, player_grid, AI_TGT, player_prog, CLINE, FUEL_ON, AILINE, ai_best, ent_name, fmt_lap, save_human_best, banking_allowed, plausible_lap)
        prev_laps = _lt.prev_laps; lap_t0 = _lt.lap_t0; last_lap = _lt.last_lap; best_lap = _lt.best_lap; race_done = _lt.race_done

        (CLINE !== nothing && !REPLAY) && lapclock_step!(LAPCLOCK, player_prog - cs.laps*CLINE.total, cs.t - lap_t0, cs.laps, CLINE.total, last_lap,
                                            race_go[] && !race_done, HUD_SECFRACS)
        if telem !== nothing && (tsamp += 1) % 6 == 0
            write(telem, "$(round(cs.t,digits=2))\t$(cs.laps)\t$(round(cs.lapdist,digits=1))\t$(round(cs.v*3.6,digits=1))\t$(round(inp.throttle,digits=2))\t$(round(inp.brake,digits=2))\t$(round(inp.steer,digits=2))\t$(round(inp.clutch,digits=2))\t$(cs.gear)\t$(round(Int,cs.rpm))\t$(round(cs.x,digits=1))\t$(round(cs.z,digits=1))\t$(round(cs.lateral,digits=2))\t$(round(cs.along,digits=2))\t$(cs.ontrack ? 1 : 0)\n")
        end

        @label skipsim

        _bc = body_camera_step!(cs, inp, dt, v_prev, pitch_ter, roll_ter, pitch_dyn, cam_pitch, cam_roll, CLINE, REPLAY,
                                rep_focus, rep_swap, REPLAY ? rep_ai_raw : nothing, rep_cam, aicam_pose)
        v_prev = _bc.v_prev; pitch_ter = _bc.pitch_ter; roll_ter = _bc.roll_ter; pitch_dyn = _bc.pitch_dyn
        cam_pitch = _bc.cam_pitch; cam_roll = _bc.cam_roll; vp = _bc.vp; eye = _bc.eye
        carModel = _bc.carModel; tiltModel = _bc.tiltModel; bodyModel = _bc.bodyModel
        δ = Float32(inp.steer * (CAR3D ? DriveRT3D.MAXSTEER : SKIDPAD ? 0.30 : CAR.max_steer))

        wheelmat(wx,wz,steer,r,nm="") = carModel * CARHAND * Render.translate(wx, r, wz) *
                     (steer ? Render.roty(Float32(HSIGN)*δ) : Render.ident()) * Render.rotz(Float32(wheel_spin_angle(nm, spin)))

        loosemat(lx,ly,lz,sp) = Render.translate(lx, ly, -lz) * Render.rotz(Float32(sp)) * CARHAND

        ai_hit = Ref(false); ddt = dt > 1e-4 ? dt : 1/60

        aibankP(pc) = (isfinite(pc.pitch) ? pc.pitch : 0.0, (isfinite(pc.roll) ? pc.roll : 0.0) + terrain_roll(pc))
        aibankK(p)  = (cc=(x=p[1], y=p[2], z=p[3], θ=p[4]); (terrain_pitch(cc), terrain_roll(cc)))

        ai_height(x, z) = ground_road(x, z)
        ai_ground(p) = RaceAI.reground(p, ai_height)
        AICAM_R[] > 0 && (aicam_pose[] = AICAM_R[] <= length(ai_poses_prev[]) ? ai_poses_prev[][AICAM_R[]] : nothing)
        ai_poses = ai_poses_step!(cs, ddt, ai_hit, REPLAY ? rep_ai_raw : nothing, AICARS, AIPHYS, CLASSICAI_T, phase, race_go, race_done, ai_release, aibankP, aibankK, ai_ground, groundz, groundz_ai, ai_onroad, ai_stuck, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, ent_name, classicai_field!, REPLAY, AILINE, AI_SCALE)

        frames == 1 && warm_ai_compile!(cs, ddt, ai_hit, AICARS, AILINE, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, CLASSICAI_T, ent_name, classicai_field!, phase, race_done, race_go, ai_poses, AIPHYS, PLAYER_CDA, ffb_jolt, ai_onroad, ai_stuck, groundz_ai, aibankP, groundz)

        ai_lap_step!(cs, phase, race_go, AICARS, ai_lap_prev, ai_lapt0, ai_best, ent_name, fmt_lap)

        if RAM_AI !== nothing && CAR3D && race_go[] && !RAM_STATE[].done && !isempty(ai_poses)
            ram_ai_step!(cs, ai_poses, ddt, AICARS, PLAYER_G, groundz)
        end

        if frames == 1
            warm_midrace_compile!(cs, inp, dt, ai_poses, ddt, AICARS, AILINE, CLASSICAI_T, PLAYER_G, aibankP, frames, classicai_field!, groundz, groundz_ai, groundz_phys, roll_ter, CLINE)
            HUDBAND_ON && precompile(band_overlay_draw!, typeof.((cs, W, H, AICARS, standings, texthud_name)))
            HUDBAND_ON && precompile(band_tc, (NTuple{4,NTuple{5,Float64}},))
            if HUDBAND_ON && FONT !== nothing
                precompile(section_delta, (Int,))
                precompile(Core.kwcall, (NamedTuple{(:scale, :range),Tuple{Float64,Float64}}, typeof(delta_bar!), Vector{Float32}, Vector{Float32},
                                         typeof(FONT), Float64, Float64, Float64, Float64, Float64))
            end
            precompile(restart_session!, typeof.((cs, cs0, groundz, groundz_phys, AICARS, AILINE, CLINE, DO_QUAL, HOLD_START, FUEL1, FUEL_ON, CLASSICAI_F, PLAYER_G, REPLAY_NCAR, ai_best, ai_lap_prev, ai_lapt0, burn_lap, fuel_laps, fuel, cd_armed, cd_left, cd_said, cd_t0, flush_ibt!, form_grid!, ibt_samples, launch_done, new_replay_path, phase, player_finpos, player_grid, player_laps, race_go, replay_buf, replay_names, replay_out, replay_saved, replay_t, tele_buf, player_s_prev)))
        end
        if (!isempty(LOOSE_OF) || LOOSE_DROP !== nothing) && !REPLAY && !isempty(ai_poses)
            loose_cars_step!(cs, ai_poses, ddt, AICARS, AILINE, CLASSICAI_T, PLAYER_G, aibankP, frames, classicai_field!, groundz, groundz_ai)
        end
        AIWALL_TEST === nothing || REPLAY || aiwall_test_step!(Float64(cs.t), ai_poses, AICARS)
        if !isempty(AI_WRECK) && !REPLAY && !isempty(ai_poses)
            ai_wrecks_step!(ai_poses, ddt > 1e-4 ? Float64(ddt) : 1/60, groundz_ai, AICARS, AILINE, CLASSICAI_T, classicai_field!)
        end
        CAR3D && (PLAYER_CDA[] = 1.0)
        ai_poses = ai_poses_post(cs, dt, ai_poses, AICARS, REPLAY, ai_poses_prev, CLASSICAI_T)
        (!REPLAY && !isempty(ai_poses)) && (yellow_step!(Float64(cs.t), Float64(cs.x), Float64(cs.z), CLASSICAI_T); ai_gone_poses!(ai_poses))
        ai_spin_step!(ai_poses)
        spray_step!(cs, ai_poses)

        aipose_trace(cs, ai_poses, AICARS)
        if !REPLAY && race_go[] && !rst && !isempty(ai_poses)
            ffb_jolt = ai_contact_step!(cs, ai_poses, AICARS, AIPHYS, PLAYER_CDA, ffb_jolt)
        end
        if !REPLAY && !rst
            wheel_drop_step!(Float64(cs.t), ai_poses, groundz)
            _wj = wheel_hits_step!(cs, ai_poses, AICARS, classicai_field!, CLASSICAI_T); _wj > 0 && (ffb_jolt = _wj)
        end

        if NET_COLLIDE && NETLINK !== nothing && !REPLAY && race_go[] && !rst && !isempty(NETPOSES[])
            pm = 560.0
            pvx = WVX[]; pvz = WVZ[]
            for (nid, p) in NETPOSES[]
                dx = p[1] - cs.x; dz = p[3] - cs.z; d = hypot(dx, dz)
                (d < 1e-3 || !car_near(dx, dz, d, cs.θ, p[4])) && continue
                nx = dx/d; nz = dz/d
                avx, avz = get(NETVEL[], nid, (0.0, 0.0))
                lat = -dx*sin(cs.θ) + dz*cos(cs.θ)
                ffb_jolt = clamp(-sign(lat)*0.6, -1.0, 1.0)
                imp = car_contact_impulse(cs.x, cs.z, cs.θ, pvx, pvz, player_yawrate(cs), p[1], p[3], avx, avz; θb = p[4], b_fixed = true)
                imp === nothing && continue
                (pdvx, pdvz, pdr, j) = imp
                player_bump!(cs, pdvx, pdvz, pdr, imp[5], imp[6], imp[7])
                carhit_damage!(cs, pdvx, pdvz, j)
                vrel = j/((1 + CAR_E)*CAR_M/2); scrub = 0.0
                ffb_jolt = clamp(ffb_jolt - sign(lat)*(j/pm)*0.4 - 0.5, -1.0, 1.0)
                NET_HITS[] += 1
                NET_DIAG > 0 && println("  [netcollide] t=", round(cs.t, digits = 2), " car ", nid, " closing ", round(vrel, digits = 2),
                                        " m/s, d ", round(d, digits = 2), " m, scrub ", round(scrub, digits = 2), " m/s")
            end
        end

        replay_tyre_lp_step!(cs, dt, replay_buf, tyre_lp)

        cs.t < replay_t[] && (replay_t[] = cs.t - 1.0)

        frames == 1 && replay_buf !== nothing && !race_go[] &&
            warm_replay_record!(cs, inp, AICARS, ai_poses, CLINE, REPLAY_AUTOSAVE, REPLAY_NCAR, phase, replay_names, tyre_lp)
        replay_record!(cs, inp, AICARS, ai_poses, CLINE, REPLAY_AUTOSAVE, REPLAY_NCAR, phase, race_go, rst, replay_buf, replay_names, replay_out, replay_saved, replay_t, tele_buf, tyre_lp,
                       CLINE === nothing ? 0.0 : Float64(player_prog / CLINE.total))

        aiCar(p)  = Render.pose_matrix(p[1], p[2], -p[3], p[4], p[5], p[6])
        aiBody(p, cm) = ai_body_mat(p, cm)
        aiWheel(p,cm,wx,wz,r) = ai_wheel_mat(p, cm, wx, wz, r, spin)

        NETLINK !== nothing && netlink_step!(cs, inp, frames, ai_poses, AICARS, CLINE, aibankK, ai_ground, net_last)

        WHEELGAP > 0 && (frames % WHEELGAP) == 0 && wheelgap_probe(cs, ai_poses)
        ALLOCPROF > 0 && (ALLOC_MARK[2] = Base.gc_bytes())
        DSTAT && (DS_LOOP[] += (DS_PREV[] > 0 ? now - DS_PREV[] : 0.0); DS_PREV[] = now)
        DSTAT && (GPUSYNC && glFinish(); DS_T[1] = time(); Render.NDRAW[] = 0; Render.NTRI[] = 0; DS_SIMACC[1] += DS_T[1] - now; DS_SIMACC[2] += DS_SIM[1]; DS_SIMACC[3] += DS_SIM[2]; fill!(DS_SIM, 0.0))

        _tp_d = time()
        lightVP = shadow_pass_step!(cs, depthprog, shadowfbo, bodyModel, wheelmat, loosemat, ai_poses, AICHASSIS, spin)

        drawworld = (vp_, eye_, flip::Bool) -> drawworld_impl!(vp_, eye_, flip, cs, AICHASSIS, _tp_d, aiBody, aiWheel, ai_poses, spin)
        DSTAT && (GPUSYNC && glFinish(); DS_T[2] = time(); DS_N[1] = Render.NDRAW[]; DS_N[4] = Render.NTRI[])

        (mirror_live, _mir_alt) = mirror_gate(REPLAY, frames)
        frames == 1 && precompile(mirror_render!, typeof.((drawworld, cs, cam_pitch, cam_roll, frames, _mir_alt, lightVP, bodyModel, wheelmat, loosemat)))
        if mirror_live
            mirror_render!(drawworld, cs, cam_pitch, cam_roll, frames, _mir_alt, lightVP, bodyModel, wheelmat, loosemat)
        end
        (BANDMIR_ON && band_shown()) && band_mirror_render!(drawworld, cs, lightVP)
        DSTAT && (GPUSYNC && glFinish(); DS_T[3] = time(); DS_N[2] = Render.NDRAW[]; DS_N[5] = Render.NTRI[])

        main_pass_begin!(vp, eye, lightVP)

        ALLOCPROF > 0 && (ALLOC_MARK[3] = Base.gc_bytes())
        _t_world = time()
        drawworld(vp, eye, false)
        FRAMEPROF > 0 && (PROF_WORLD[] += time() - _t_world)

        if !isempty(NET_CARSWITCH[])
            _nc = NET_CARSWITCH[]; NET_CARSWITCH[] = ""
            PLAYER_CM[] = _nc == "Lotus" ? nothing : get(NET_BY_TYPE, _nc, nothing)
            rep_team[] = PLAYER_CM[] === nothing ? nothing : team_items(_nc)
            replay_names[1] = _nc == "Lotus" ? "You" : "You ($(_nc))"
        end
        PLAYER_MINE[] = PLAYER_CM[] !== nothing && !rep_swap[]
        if PLAYER_MINE[] && CTL.view != 0
            player_chassis_draw!(PLAYER_CM[], vp, tiltModel, carModel, Float32(δ), Float64(spin))
        else
            player_car_draw!(cs, inp, vp, bodyModel, wheelmat, loosemat, REPLAY, rep_team, mirror_live, frames, TEAM_LEVER_AT_LOTUS)
            (PLAYER_MINE[] && COCKPIT_OWNWHEELS) && player_chassis_draw!(PLAYER_CM[], vp, tiltModel, carModel, Float32(δ), Float64(spin); body = false)
        end
        α_tc = clamp(dt/0.10, 0.0, 1.0)
        tc_hud = ntuple(i -> ntuple(j -> tc_hud[i][j] + (cs.tc[i][j]-tc_hud[i][j])*α_tc, 3), 4)
        ALLOCPROF > 0 && (ALLOC_MARK[4] = Base.gc_bytes())
        DSTAT && (GPUSYNC && glFinish(); DS_T[4] = time(); DS_N[3] = Render.NDRAW[]; DS_N[6] = Render.NTRI[])
        _t_hud = time()

        replay_tyres_draw!(W, H, REPLAY, rep_cam, rep_focus, rep_tyres, rep_tyshow)

        HUD_UNDER_BAND && hud_draw_step!(cs, inp, tc_hud, last_lap, best_lap, cd_armed, cd_left, race_go, phase, race_done, player_finpos, HOLD_START)
        glViewport(0, 0, W, H)
        texthud_draw!(cs, W, H, AICARS, best_lap, fmt_lap, lap_t0, last_lap, player_finpos, race_done, race_go, standings, texthud_name, ai_poses,
                      (HUDBAND_ON && REPLAY && rep_tyres[] !== nothing) ? band_tc(rep_tyres[]) : tc_hud)

        HUD_UNDER_BAND || hud_draw_step!(cs, inp, tc_hud, last_lap, best_lap, cd_armed, cd_left, race_go, phase, race_done, player_finpos, HOLD_START)
        wreck_notice_draw!(W, H, REPLAY)
        REPLAY || yellow_flag_draw!(Float64(cs.t), W, H)
        CLUTCH_GATE[] > 0 && (CLUTCH_GATE[] -= dt)

        FRAMEPROF > 0 && (PROF_HUD[] += time() - _t_hud)
        frameprof_report!()
        fpsdiag_report!(cs)

        allocprof_report!()
        pace_step!(frames, now)

        gc_step!(frames)
        DSTAT && (GPUSYNC && glFinish(); DS_T[5] = time())
        GLFW.SwapBuffers(win)
        frame_stats_step!()

        frame_dumps!(frames)

        dx_shots!(cs, frames, REPLAY, eye, race_done, rep_rt, vp)
        aicam_seq_step!(frames)

        shots_step!(frames, vp, eye, W, H, place_at_s!, shot_idx, shot_t0, shots_done)

        frames += 1
        stall_log!(cs, frames, inp, win_shown)
        if !win_shown[] && frames >= show_after
            println("  window revealed after ", frames, " hidden warm-up frames (", round(Int, 1000*(time() - show_t0)), " ms)"); flush(stdout)
            tstamp("  [E80] .. window revealed (launch done)")
            Base.invokelatest(perf4_reveal_marker)
            reveal!()
        end

        allocsites_report!(cs, frames)

        mirror_ema_step!(frames)
        titleT = title_step!(cs, win, now, titleT, REPLAY, FUEL_ON, repd, rep_focus, rep_cam, rep_rt, rep_dur, rep_play, rep_speed, AICARS, standings, race_go, phase, cd_armed, cd_left, race_done, player_finpos, best_lap, fmt_lap, player_grid, fuel, burn_lap)
    end
    telem !== nothing && close(telem)

    get(ENV, "JM_AISPIN_DIAG", "0") != "0" && println("  [aispin] shared ", round(spin, digits = 2), " drawn per car ",
                                                     [round(ai_spin(k, spin), digits = 2) for k in eachindex(AI_SPIN)])
    flush_ibt!(ibt_samples)
    end_reports()
    if replay_buf !== nothing && !isempty(replay_buf)
        write_replay(replay_out[], replay_buf, tele_buf, REPLAY_NCAR, replay_names, CLINE; final = true)
    end
    ffb !== nothing && FFB.close_ffb(ffb)
    EngineAudio.stop!(ENG)

    if INSTANT_REQ[] && replay_buf !== nothing && !isempty(replay_buf)
        INSTANT_REQ[] = false
        instant_replay_reset!()
        REPLAY_FILE_R[] = replay_out[]; INSTANT_AT_END[] = true
        println("  [instant] replaying this session in this process: ", basename(replay_out[])); flush(stdout)
        tstamp("  [instant] main() again, as the replay")
        return :replay
    end

    if WARM_HOLD
        wp = warm_hold(replay_names)
        if wp !== nothing
            instant_replay_reset!()
            REPLAY_FILE_R[] = wp; INSTANT_AT_END[] = false
            println("  [instant] replaying this session in this process: ", basename(wp)); flush(stdout)
            tstamp("  [warm] main() again, as the replay")
            return :replay
        end
    end
    GLFW.Terminate()
    CLASSICW_PHYS && CLASSICW_T[][2] > 0 && println("  CLASSICWALL-1 cost: ", round(1e6*CLASSICW_T[][1]/CLASSICW_T[][2], digits = 1),
                                            " us/frame over ", CLASSICW_T[][2], " frames (walls + drawn obstacles, contact + clamp)")

    SMOKE && CLASSICAI_F[] !== nothing && println("  [classicai] field contact ticks: ", CLASSICAI_F[].stats)
    ffb !== nothing && ffb.path == "fake" && println("  [ffb] fake device: last level ", reinterpret(Int16, FFB._getu16(ffb.buf, 16)))
    TT_RUN[] >= 5.0 && tt_save(Float64(cs.t), "session end")
    println("bye"); flush(stdout); flush(stderr)

    (@isdefined TMPTRK) && try rm(TMPTRK; recursive = true, force = true) catch end
    (@isdefined TMPOBJ) && try rm(TMPOBJ; recursive = true, force = true) catch end
    ccall(:_exit, Cvoid, (Cint,), 0)
end
end

if _MAIN_LOWER !== nothing
    let thunk = fetch(_MAIN_LOWER)
        tstamp("  [E80] .. main() pre-lowered in the background -- defining")
        Core.eval(Main, thunk)
    end
else
    Core.eval(Main, _MAIN_Q)
end
get(ENV, "JM_CHECK_DIAG", "0") == "1" && (check_diag_globals() || exit(1))
while main() === :replay; end
