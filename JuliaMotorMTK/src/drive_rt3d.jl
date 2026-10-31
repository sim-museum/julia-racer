

module DriveRT3D

using ModelingToolkit, OrdinaryDiffEq
using ModelingToolkit: t_nounits as t, D_nounits as D
const setp = ModelingToolkit.setp

const HERE = @__DIR__
for f in ("tyre.jl","powertrain.jl","chassis_parts.jl","vehicle_3d.jl")
    include(joinpath(HERE, "components", f))
end

export Car3D, build_car3d, air_density, set_air!, Chassis, CHASSIS, set_chassis!, chassis_from_setup, describe_chassis, toe_rad, bias_torque, set_shift_rpm!, set_suspension!, wheel_rate, set_ride_height!, step_car3d!, telemetry3d, respawn3d!, contain3d!, extforce3d!, contact_force, cap_total_contact, wheelmu3d!, world_velocity, damage_hit!, damage_impact!, damage_engine!, damage_mu, engine_power, engine_dead, damaged, damage_reset!

const GEARS = [2.23, 1.72, 1.32, 1.09, 0.916]

const REVERSE_ON = get(ENV, "JM_REVERSE", "1") != "0"
gearratio(g::Int) = g == -1 ? -GEARS[1] : g <= 0 ? 0.0 : GEARS[g]
const FINAL = Ref(4.11)
const TRANS_SRC = Ref("built-in fallback (SKIDPAD setup -- NOT from an ibt)")

"""    set_transmission!(gears, final; source) -> nothing

Install the gearbox read from an ibt session. Must be called BEFORE a Car3D is built: the
final drive is an MTK parameter baked in at construction, so a later change would silently
apply to the ratios and not to the drivetrain.
"""
function set_transmission!(gears::AbstractVector, final::Real; source::AbstractString="unknown")
    length(gears) == length(GEARS) ||
        error("set_transmission!: expected $(length(GEARS)) ratios, got $(length(gears))")
    all(g -> g > 0, gears) || error("set_transmission!: non-positive gear ratio in $gears")
    final > 0 || error("set_transmission!: non-positive final drive $final")
    GEARS .= float.(gears); FINAL[] = float(final); TRANS_SRC[] = source
    nothing
end

const SHIFT_RPM = Ref(8900.0)
const SHIFT_SRC = Ref("built-in (Lotus 49 shift light, 8900 rpm -- NOT read from an ibt this session)")
function set_shift_rpm!(rpm::Real; source::AbstractString = "unknown")
    (isfinite(rpm) && 4000 < rpm < 12000) || error("set_shift_rpm!: implausible shift rpm $rpm")
    SHIFT_RPM[] = float(rpm); SHIFT_SRC[] = source; nothing
end

"""Where the live gearbox came from — print this at startup so a fallback cannot hide."""
transmission_source() = TRANS_SRC[]

const MASS       = Ref(617.0)
const FRONT_FRAC = Ref(0.455)

"""    set_mass!(m, front_frac; source) -> nothing

Install the car mass and its front share, derived from an ibt session's CornerWeights. Like
set_transmission!, this must run BEFORE a Car3D is built — m and front_frac are MTK parameters
baked in at construction.
"""

const CORNER_W = Ref{Union{Nothing,NTuple{4,Float64}}}(nothing)
const MASS_LEGACY = haskey(ENV, "JM_MASS_LEGACY")
const WHEELBASE = 2.41
function set_corner_weights!(cw; source::AbstractString = "unknown")
    w = (float(cw[:LF]), float(cw[:RF]), float(cw[:LR]), float(cw[:RR]))
    all(x -> x > 0, w) || error("set_corner_weights!: $w")
    CORNER_W[] = w
    set_mass!(sum(w)/9.81, (w[1] + w[2])/sum(w); source)
end

const FUEL_TANK_X_FRONT = 0.82
const FUEL_FRONT_SHARE = 1 - FUEL_TANK_X_FRONT/WHEELBASE
"""The corner weights [N] with `litres` in the tank instead of the setup's `setup_litres`: the difference at
FUEL_KG_PER_L, split FUEL_FRONT_SHARE front, half each side."""
function corner_weights_with_fuel(cw, setup_litres, litres)
    dW = (litres - setup_litres)*FUEL_KG_PER_L*9.81
    Dict(:LF => cw[:LF] + dW*FUEL_FRONT_SHARE/2, :RF => cw[:RF] + dW*FUEL_FRONT_SHARE/2,
         :LR => cw[:LR] + dW*(1 - FUEL_FRONT_SHARE)/2, :RR => cw[:RR] + dW*(1 - FUEL_FRONT_SHARE)/2)
end
const _TANK3D = IdDict{Any,Any}()
"FUEL-1: the fuel burnt since this car was built [kg] (the FuelTank's runtime parameter)."
function set_fuel_burnt!(c, kg)
    s = get!(_TANK3D, c.sys) do
        try ModelingToolkit.setp(c.sys, c.sys.tank.m_burn) catch; nothing end
    end
    s === nothing || s(c.integ, float(kg))
    nothing
end

"CG arms (a, b) [m] for the vehicle: from the session's front share, or the shipped 1.314 / 1.096."
cg_arms() = (MASS_LEGACY || CORNER_W[] === nothing) ? (CAR[].dL == 0.0 ? (1.314, 1.096) : (1.314, 1.096) .* (wheelbase()/WHEELBASE)) :
            ((1 - FRONT_FRAC[])*wheelbase(), FRONT_FRAC[]*wheelbase())

struct CarPhys; name::String; dm::Float64; dL::Float64; dtf::Float64; dtr::Float64; kR::Float64; kP::Float64; end
const CAR_PHYS = Dict(
    "Lotus"   => CarPhys("Lotus",     0.0,  0.000,  0.000,  0.000, 1.0,          1.0),
    "Eagle"   => CarPhys("Eagle",    44.0,  0.036,  0.000, -0.025, 9500/9000,    400/400),
    "Ferrari" => CarPhys("Ferrari",  47.0, -0.013, -0.074, -0.114, 10000/9000,   390/400),
    "Brabham" => CarPhys("Brabham",   7.0, -0.025, -0.190, -0.152, 8800/9000,    330/400),
    "Cooper"  => CarPhys("Cooper",   77.0,  0.077, -0.029, -0.094, 1.0,          355/400),
    "Honda"   => CarPhys("Honda",    89.0,  0.041, -0.060, -0.107, 11500/9000,   414/400),
    "BRM"     => CarPhys("BRM",     125.0,  0.025, -0.025, -0.025, 10500/9000,   405/400))
const CAR = Ref(CAR_PHYS["Lotus"])
"Make `name` the car the next build_car3d builds (its engine too). Unknown names, or JM_CARPHYS=0, give the Lotus."
function set_car!(name::AbstractString)
    CAR[] = get(ENV, "JM_CARPHYS", "1") == "0" ? CAR_PHYS["Lotus"] : get(CAR_PHYS, String(name), CAR_PHYS["Lotus"])
    ENGINE_KR[] = CAR[].kR; ENGINE_KP[] = CAR[].kP
    nothing
end
wheelbase() = WHEELBASE + CAR[].dL
final_eff() = FINAL[]*CAR[].kR
car_mass() = MASS[] + CAR[].dm
"The car's inertia scales (yaw, roll, pitch) against the Lotus's."
function car_iscale()
    c = CAR[]; mr = car_mass()/MASS[]; lr = (wheelbase()/WHEELBASE)^2; tr = ((3.0 + c.dtf + c.dtr)/3.0)^2
    (mr*lr, mr*tr, mr*lr)
end
car_describe() = (c = CAR[]; c.name == "Lotus" ? "Lotus 49 (the reference car)" :
    string(c.name, ": the Lotus 49 model with mass ", round(car_mass(), digits = 1), " kg (", c.dm >= 0 ? "+" : "", c.dm, "), wheelbase ",
           round(wheelbase(), digits = 3), " m, tracks ", round(1.5 + c.dtf, digits = 3), " / ", round(1.5 + c.dtr, digits = 3),
           " m, engine x", round(c.kP, digits = 3), " power over x", round(c.kR, digits = 3), " rpm (redline ", round(Int, 9500c.kR),
           "), final ", round(final_eff(), digits = 3)))
"Run f with the Lotus as the car (the AI's physics cars), then restore."
function with_lotus(f)
    old = CAR[]; set_car!("Lotus")
    try f() finally CAR[] = old; ENGINE_KR[] = old.kR; ENGINE_KP[] = old.kP end
end

function set_mass!(m::Real, front_frac::Real; source::AbstractString="unknown")
    m > 0 || error("set_mass!: non-positive mass $m")
    0.0 < front_frac < 1.0 || error("set_mass!: front_frac out of range: $front_frac")
    MASS[] = float(m); FRONT_FRAC[] = float(front_frac)
    nothing
end

const MR2 = parse(Float64, get(ENV, "JM_MR2_F", "0.6559"))

const MR2_R = parse(Float64, get(ENV, "JM_MR2_R", "0.4473"))

const KARBR_COMP = get(ENV, "JM_KARBR_COMP", "0") != "0"
const KS  = Ref((19_677.0, 19_677.0, 21_470.0, 21_470.0))
const KS_SRC = Ref("built-in fallback (Nurburgring setup -- NOT from an ibt)")

"""    set_suspension!(fl, fr, rl, rr; source) -> nothing

Install per-corner WHEEL rates [N/m] from an ibt session's four SpringRate channels. Like
set_transmission! and set_mass!, this must run BEFORE a Car3D is built: the rates are MTK
parameters baked in at construction.

Pass ibt SpringRate values in N/mm through `wheel_rate` to get the units and motion ratio right.
"""
function set_suspension!(fl::Real, fr::Real, rl::Real, rr::Real; source::AbstractString="unknown")
    all(k -> k > 0, (fl, fr, rl, rr)) || error("set_suspension!: non-positive rate in $((fl,fr,rl,rr))")
    KS[] = (float(fl), float(fr), float(rl), float(rr)); KS_SRC[] = source
    nothing
end

"""CARPHYS-1 S7: wheel travel (bump, droop) [m] per corner FL FR RL RR from the garage's shock deflection static/max
and packer [mm]: bump = (max − static − packer)/MR, droop = −static/MR. MRCONSIST-1 S2: MR is the SHOCK's (garage fit
MR 0.814 / 0.693, garage_lotus49.toml), not the spring's; JM_KARBR_COMP=1 keeps the old sqrt(MR2) (0.78 / 0.648)."""
const MR_SHOCK = (0.814256, 0.693304)
function travel_from_setup(static, maxd, packer)
    mr = KARBR_COMP ? (sqrt(MR2), sqrt(MR2), sqrt(MR2_R), sqrt(MR2_R)) : (MR_SHOCK[1], MR_SHOCK[1], MR_SHOCK[2], MR_SHOCK[2])
    ntuple(i -> ((maxd[i] - static[i] - packer[i])/1000/mr[i], -static[i]/1000/mr[i]), 4)
end

const TRAVEL_REF = ((61.8, 60.5, 58.1, 59.2), (104.6, 104.6, 113.0, 113.0), (0.0, 0.0, 0.0, 0.0))
const NOSTOPS = haskey(ENV, "JM_NOSTOPS")

Base.@kwdef mutable struct Chassis
    diff::Any = nothing
    toe::NTuple{2,Float64} = (0.0, 0.0)
    karb::NTuple{2,Float64} = (0.0, 0.0)
    cscale::NTuple{2,Float64} = (1.0, 1.0)
    bias::Float64 = 0.585
    bias_p::Float64 = 0.535

    travel::Union{Nothing,NTuple{4,NTuple{2,Float64}}} = NOSTOPS ? nothing : travel_from_setup(TRAVEL_REF...)
    camber::Union{Nothing,NTuple{4,Float64}} = nothing
    pressure::Union{Nothing,NTuple{4,Float64}} = nothing
    src::String = "built-in (spool rear, no toe, default bars/dampers/bias -- NOT from an ibt)"
end
const CHASSIS = Ref(Chassis())
set_chassis!(ch::Chassis) = (CHASSIS[] = ch; nothing)

"""Toe-in per wheel [rad] from the garage's toe [mm], read as the toe at the tyre's edge -- across its diameter."""

const TOE_SCALE = parse(Float64, get(ENV, "JM_TOE_SCALE", "1.0"))
toe_rad(mm::Real; rear::Bool = false) = atan(TOE_SCALE*float(mm)/1000, 2*(rear ? RW_R : 0.30))

"""Brake TORQUE split front from the garage's PRESSURE bias [%]. BRAKE-2 fitted 0.585 at the default's 53.5 %:
the ratio of front to rear torque per unit pressure (bigger front calipers/discs) is then fixed, and any other
pressure split maps through it."""
function bias_torque(pct::Real)
    kA = 0.585*(1 - 0.535) / (0.535*(1 - 0.585))
    p = float(pct)/100
    p*kA / (p*kA + (1 - p))
end

const ARB_ID = Dict{Tuple{String,String},NTuple{2,Float64}}(
    ("0.375\" 2", "0.6875\" 5")    => (1121.0, 12714.0),
    ("0.375\" 5", "0.6875\" firm") => (1758.0, 19940.0),

    ("0.375\" soft", "0.6875\" 5") => (567.0, 12714.0),
    ("0.375\" 4", "0.6875\" 5")    => (3535.0, 12714.0),
    ("0.375\" firm", "0.6875\" 5") => (6804.0, 12714.0),
)

const ARB_ARM_FRONT = Dict("soft" => 0.37, "1" => 0.37, "2" => 1.0, "3" => 1.65, "4" => 2.30, "5" => 3.37, "firm" => 4.43, "6" => 4.43)

const _KC = KARBR_COMP
const ARB_BAR_F = _KC ? 0.46*(1121.0 + 12714.0) : 8004.0; const ARB_BAR_R = _KC ? 0.54*(1121.0 + 12714.0) : 9396.0
const ARB_LAW_F = _KC ? Dict("soft" => 0.76, "1" => 0.76, "2" => 1.0, "3" => 1.24, "4" => 1.48, "5" => 1.72, "6" => 1.96, "firm" => 2.20, "7" => 2.20) :
    Dict("soft" => 0.75, "1" => 0.75, "2" => 1.0, "3" => 1.184, "4" => 1.368, "5" => 1.552, "6" => 1.736, "firm" => 1.92, "7" => 1.92)
const ARB_LAW_R = _KC ? Dict("soft" => 0.20, "1" => 0.20, "2" => 0.40, "3" => 0.60, "4" => 0.80, "5" => 1.0, "6" => 1.285, "firm" => 1.57, "7" => 1.57) :
    Dict("soft" => 0.33, "1" => 0.33, "2" => 0.4975, "3" => 0.665, "4" => 0.8325, "5" => 1.0, "6" => 1.235, "firm" => 1.47, "7" => 1.47)
_armof(s) = (m = match(r"\"\s*(\S+)\s*$", s); m === nothing ? "" : lowercase(m.captures[1]))
_firstnum(s) = (m = match(r"[-+]?\d+(?:\.\d+)?", s); m === nothing ? NaN : parse(Float64, m.match))
const ARB_REF = (("0.375\" 2", "0.6875\" 5"), 0.375, 0.6875)
const DAMPER_ID = Dict{NTuple{4,Float64},NTuple{2,Float64}}(
    (4.0, 8.0, 4.0, 7.0) => (1.0, 1.0),

    (6.0, 9.0, 9.0, 12.0) => (1.00, 1.15),
)

const DAMPER_CLICK_LAW = ((3002.0, 184.0), (1499.0, 67.0), (1326.0, 85.0), (303.0, 18.5))
function damper_scale_from_clicks(clk::NTuple{4,Float64})
    c(k, n) = DAMPER_CLICK_LAW[k][1] + DAMPER_CLICK_LAW[k][2]*n
    ((c(1, clk[1]) + c(2, clk[2]))/(c(1, 4.0) + c(2, 8.0)), (c(3, clk[3]) + c(4, clk[4]))/(c(3, 4.0) + c(4, 7.0)))
end

"""    chassis_from_setup(sp; source) -> Chassis

The session's diff, toe, bars, dampers and brake bias as model inputs (sp = Setup.setup_params of the ibt)."""
function chassis_from_setup(sp; source::AbstractString = "unknown")
    notes = String[]
    diff = all(isfinite, (sp.diff_preload_Nm, sp.diff_drive_ramp, sp.diff_coast_ramp)) ?
        (sp.diff_preload_Nm, sp.diff_drive_ramp, sp.diff_coast_ramp, isfinite(sp.diff_plates) ? sp.diff_plates : 4.0) :
        (push!(notes, "no Differential block: spool"); nothing)
    toe = (isfinite(sp.toe_front_mm) ? toe_rad(sp.toe_front_mm) : 0.0, isfinite(sp.toe_rear_mm) ? toe_rad(sp.toe_rear_mm; rear = true) : 0.0)
    karb = get(ARB_ID, (sp.arb_front, sp.arb_rear)) do

        df = occursin(r"(?i)disc", sp.arb_front) ? 0.0 : _firstnum(sp.arb_front)
        dr = occursin(r"(?i)disc", sp.arb_rear)  ? 0.0 : _firstnum(sp.arb_rear)
        ref = ARB_ID[ARB_REF[1]]
        if isfinite(df) && isfinite(dr)

            af = get(ARB_ARM_FRONT, _armof(sp.arb_front), NaN)/ARB_ARM_FRONT["2"]
            isfinite(af) || (af = 1.0; push!(notes, "front arms $(_armof(sp.arb_front)) unknown: arms 2"))
            push!(notes, "bars $(sp.arb_front) / $(sp.arb_rear) estimated from 261004's by diameter⁴ (front × arm law $(round(af, digits = 2)))")
            (ref[1]*(df/ARB_REF[2])^4*af, ref[2]*(dr/ARB_REF[3])^4)
        else
            push!(notes, "bars $(sp.arb_front) / $(sp.arb_rear) unreadable: 261004's"); ref
        end
    end
    clk = (sp.bump_clicks[:LF], sp.rebound_clicks[:LF], sp.bump_clicks[:LR], sp.rebound_clicks[:LR])
    cscale = get(DAMPER_ID, clk) do
        all(isfinite, clk) || (push!(notes, "dampers $(clk) unreadable: hand-set"); return (1.0, 1.0))
        sc = damper_scale_from_clicks(clk)
        push!(notes, "dampers $(Int.(clk)) by reference's click law (SETUPMODEL-1 S3): x$(round(sc[1], digits = 2))/$(round(sc[2], digits = 2))")
        sc
    end
    bias = isfinite(sp.brake_bias_pct) ? bias_torque(sp.brake_bias_pct) : 0.585
    bias_p = isfinite(sp.brake_bias_pct) ? sp.brake_bias_pct/100 : 0.535
    ks = (:LF, :RF, :LR, :RR)
    travel = NOSTOPS ? nothing : hasproperty(sp, :shock_defl_mm) && all(k -> all(isfinite, sp.shock_defl_mm[k]), ks) ?
        travel_from_setup(Tuple(sp.shock_defl_mm[k][1] for k in ks), Tuple(sp.shock_defl_mm[k][2] for k in ks),
                          Tuple(isfinite(sp.packer_mm[k]) ? sp.packer_mm[k] : 0.0 for k in ks)) :
        (push!(notes, "no shock deflection in the setup: 261004's travel"); travel_from_setup(TRAVEL_REF...))
    cd = [sp.camber_deg[k] for k in (:LF, :RF, :LR, :RR)]
    camber = all(isfinite, cd) ? Tuple(deg2rad.(cd)) : (push!(notes, "no camber in the setup: upright tyres"); nothing)

    if PHYSFID_S9 && !haskey(ENV, "JM_ARB_FRONT_SHARE") && !haskey(ENV, "JM_ARB_RESPLIT")
        df = occursin(r"(?i)disc", sp.arb_front) ? 0.0 : _firstnum(sp.arb_front)
        dr = occursin(r"(?i)disc", sp.arb_rear)  ? 0.0 : _firstnum(sp.arb_rear)
        lf = get(ARB_LAW_F, _armof(sp.arb_front), NaN); lr = get(ARB_LAW_R, _armof(sp.arb_rear), NaN)
        if all(isfinite, (df, dr, lf, lr))
            karb = (ARB_BAR_F*(df/0.375)^4*lf, ARB_BAR_R*(dr/0.6875)^4*lr)
            push!(notes, "bars $(sp.arb_front) / $(sp.arb_rear): front $(round(Int, karb[1])), rear $(round(Int, karb[2])) N/m (RARBARMS-1 per-bar laws)")
        else
            f = 0.46; tot = karb[1] + karb[2]; karb = (f*tot, (1 - f)*tot)
            push!(notes, "bars $(sp.arb_front) / $(sp.arb_rear) unreadable: ARB front share $(f)")
        end
    elseif haskey(ENV, "JM_ARB_FRONT_SHARE") || PHYSFID_S9
        f = parse(Float64, get(ENV, "JM_ARB_FRONT_SHARE", "0.46")); tot = karb[1] + karb[2]; karb = (f*tot, (1 - f)*tot)
        push!(notes, "ARB front share $(f)")
    end
    pk = hasproperty(sp, :cold_pressure_kPa) ? [sp.cold_pressure_kPa[k] for k in (:LF, :RF, :LR, :RR)] : Float64[]
    pressure = (length(pk) == 4 && all(isfinite, pk)) ? Tuple(Float64.(pk)) : nothing
    Chassis(; diff, toe, karb, cscale, bias, bias_p, travel, camber, pressure, src = source * (isempty(notes) ? "" : "  (" * join(notes, "; ") * ")"))
end

"""One line per chassis input, for the launch log: a silent fallback is the defect E100 exists to prevent."""
describe_chassis(ch::Chassis = CHASSIS[]) =
    "diff " * (ch.diff === nothing ? "spool" : "LSD preload $(round(Int, ch.diff[1])) N·m ramps $(round(Int, ch.diff[2]))/$(round(Int, ch.diff[3]))° plates $(round(Int, ch.diff[4]))") *
    "   toe-in F $(round(rad2deg(ch.toe[1]), digits = 2))° R $(round(rad2deg(ch.toe[2]), digits = 2))°" *
    "   +roll k F $(round(Int, ch.karb[1])) R $(round(Int, ch.karb[2])) N/m   dampers x$(round(ch.cscale[1], digits = 2))/$(round(ch.cscale[2], digits = 2))" *
    "   brake bias $(round(100ch.bias_p, digits = 1)) % (torque split $(round(ch.bias, digits = 3)))" *
    "   travel " * (ch.travel === nothing ? "unlimited" : join((string(round(Int, 1000t[1]), "/", round(Int, 1000t[2])) for t in ch.travel), " ") * " mm") *
    "   camber " * (ch.camber === nothing ? "none (upright)" : join((string(round(rad2deg(c), digits = 2)) for c in ch.camber), "/") * "°") *
    "   <- $(ch.src)"

"""Wheel rate [N/m] from an ibt SpringRate [N/mm]. One place for the unit and motion-ratio change."""
wheel_rate(spring_N_per_mm::Real; rear::Bool = false) = float(spring_N_per_mm) * 1000.0 * (rear ? MR2_R : MR2)

"""
    wrecks(closing, bnd_peak, speed; close_ms, bnd_peak_max, vmin_ms) -> Bool

E95/E99: does this contact end the race?  PO 2026-08-30: *"a graze at speed should scrub you
but not end your race."*

Two independent ways to be a hard hit — a high CLOSING speed into a solid, or a large boundary
penetration peak (the fence/wall case, where the contact is spread over frames and the closing
speed alone under-reads it) — and then a speed gate, so a slow shunt in the pits never totals
the car however square it is.

Closing speed, not peak force: force saturates at ~296 kN for any real contact, so it cannot
tell a graze from a square hit. That was measured, and it is why this takes `closing`.

S371: extracted because the gate had its OWN copy of this rule and the two had already drifted —
the copy was missing the boundary branch entirely and hardcoded a threshold the sim exposes as
tunable. A gate asserting its own reimplementation tests nothing about the sim.
"""
function wrecks(closing::Real, bnd_peak::Real, speed::Real;
                close_ms::Real = 12.0, bnd_peak_max::Real = 1.0e3, vmin_ms::Real = 50.0/3.6)
    hard = (closing > close_ms) || (bnd_peak > bnd_peak_max)
    return hard && abs(speed) > vmin_ms
end

const DAMAGE = [0.0, 0.0, 0.0, 0.0]

"""Onset: below this closing speed a contact is a rub and costs nothing (the PO's low-impulse
exception). Above it, damage grows with the speed excess."""
const DMG_ONSET_MS = 3.0

"""Grip a fully-damaged corner keeps. Not zero: a bent corner still rolls, it just will not hold
the car, and a zero would make the wheel a skid mark rather than a handicap."""
const DMG_MU_FLOOR = 0.35

"""    damage_hit!(corner, closing) -> Float64

Accumulate damage at one corner from a contact with the given closing speed (m/s). Returns the
corner's new damage. Corners are 1-4 = FL FR RL RR.
"""
function damage_hit!(corner::Int, closing::Real)
    1 <= corner <= 4 || error("damage_hit!: corner $corner out of range")
    excess = float(closing) - DMG_ONSET_MS
    excess <= 0 && return DAMAGE[corner]

    DAMAGE[corner] = clamp(DAMAGE[corner] + (1.0 - DAMAGE[corner]) * min(excess / 20.0, 1.0),
                           0.0, 1.0)
    return DAMAGE[corner]
end

"""Grip multiplier for a corner, 1.0 undamaged down to DMG_MU_FLOOR. MULTIPLY this into the
surface μ rather than replacing it — a damaged wheel on grass is worse than either alone."""
damage_mu(corner::Int) = 1.0 - (1.0 - DMG_MU_FLOOR) * DAMAGE[corner]

"""    damage_impact!(fx, fy, closing) -> nothing

Damage the corners that actually FACED the impact, from the body-frame contact force.

E94-P4 first spread damage over all four corners because the contact POINT was not available.
It does not need to be: `contact_force` returns the force in the BODY frame, and a contact can
only push — so the blow arrived from the direction OPPOSITE the force it produced. Each corner is
then weighted by how squarely it faces that direction, and a corner facing away takes nothing.

Weights are normalised so the most-exposed corner takes the full hit; the rest scale down by the
cosine. No tuning constants, and nothing here that a knob could adjust.

Falls back to spreading evenly when the force is degenerate (|F| ~ 0), which is the honest
behaviour when the direction genuinely is not known.
"""
function damage_impact!(fx::Real, fy::Real, closing::Real)
    mag = sqrt(float(fx)^2 + float(fy)^2)
    if mag < 1e-6
        for c in 1:4; damage_hit!(c, closing); end
        damage_engine!(closing)
        return nothing
    end
    nx = -float(fx) / mag; ny = -float(fy) / mag
    w = ntuple(4) do i
        (xi, yi) = WHEELS[i]
        r = sqrt(xi^2 + yi^2)
        max(0.0, (xi/r) * nx + (yi/r) * ny)
    end
    wmax = maximum(w)
    wmax <= 0.0 && return nothing
    for c in 1:4
        damage_hit!(c, closing * (w[c] / wmax))
    end
    damage_engine!(closing)
    return nothing
end

"""True if any corner carries damage — for telemetry and for the HUD to say so."""
damaged() = any(d -> d > 0.0, DAMAGE) || ENGINE_DAMAGE[] > 0.0

const ENGINE_DAMAGE = Ref(0.0)
const ENG_ONSET_MS  = 8.0
const ENG_MIN_POWER = 0.25

"""    damage_engine!(closing) -> Float64

Accumulate engine damage from a contact's closing speed. Saturating like the corners, so repeated
heavy hits kill it and no single one can overshoot.
"""
function damage_engine!(closing::Real)
    excess = float(closing) - ENG_ONSET_MS
    excess <= 0 && return ENGINE_DAMAGE[]
    ENGINE_DAMAGE[] = clamp(ENGINE_DAMAGE[] + (1.0 - ENGINE_DAMAGE[]) * min(excess / 30.0, 1.0),
                            0.0, 1.0)
    return ENGINE_DAMAGE[]
end

"""Throttle multiplier: 1.0 healthy, falling to ENG_MIN_POWER, and 0.0 once the engine is DEAD.
Multiply this into the driver's throttle -- a damaged engine makes less power for the same pedal."""
function engine_power()
    d = ENGINE_DAMAGE[]
    d >= 1.0 && return 0.0
    return 1.0 - (1.0 - ENG_MIN_POWER) * d
end

"""True once the engine is destroyed. The stall rule (E98) then does the rest: with no power the
revs fall through the floor and MANUAL drops to AUTO, which is what a driver would want."""
engine_dead() = ENGINE_DAMAGE[] >= 1.0

"""Clear all damage. Called on respawn: that is a new car, not a repaired one."""
damage_reset!() = (fill!(DAMAGE, 0.0); ENGINE_DAMAGE[] = 0.0; nothing)

"""Mass and front share from an ibt session's corner weights (N) — the conversion in one place."""
function mass_from_corner_weights(cw)
    tot = sum(values(cw))
    tot > 0 || error("mass_from_corner_weights: corner weights sum to $tot")
    (tot / 9.81, (cw[:LF] + cw[:RF]) / tot)
end
const MAXSTEER = 0.30

const WHEEL_LOCK_DEG = 420.0
const RW_R = 0.334

const RH0 = 0.075
const RIDE_H = Ref((RH0, RH0, RH0, RH0))
const RIDE_H_SRC = Ref("built-in fallback (0.075 m at every corner -- NOT from an ibt, and no rake)")

"""    set_ride_height!(fl, fr, rl, rr; source) -> nothing

Install per-corner STATIC ride heights [m] from an ibt session's four RideHeight channels.
Unlike the gearbox/mass/springs this may be called at any time: it feeds the reported channel
rather than an MTK parameter.
"""
function set_ride_height!(fl::Real, fr::Real, rl::Real, rr::Real; source::AbstractString="unknown")
    all(h -> h > 0, (fl, fr, rl, rr)) || error("set_ride_height!: non-positive height in $((fl,fr,rl,rr))")
    RIDE_H[] = (float(fl), float(fr), float(rl), float(rr)); RIDE_H_SRC[] = source
    nothing
end

const TC_ON   = get(ENV, "JM_TC", "0") == "1" && !haskey(ENV, "JM_NOTC")

const TC_SLIP = parse(Float64, get(ENV, "JM_TC_SLIP", "0.072"))
const TC_VLO  = parse(Float64, get(ENV, "JM_TC_VLO", "25.0"))

const TC_STEERGATE = get(ENV, "JM_TC_STEERGATE", "0") != "0"
const TC_STLO = parse(Float64, get(ENV, "JM_TC_STLO", "0.10"))
const TC_STHI = parse(Float64, get(ENV, "JM_TC_STHI", "0.25"))
const TC_VHI  = parse(Float64, get(ENV, "JM_TC_VHI", "38.0"))

const WHEELS = ((1.314, 0.75), (1.314, -0.75), (-1.096, 0.75), (-1.096, -0.75))

const JM_RH_DIAG = Ref(get(ENV, "JM_RH_DIAG", "") != "")

const JM_RH_DIAG_T = Ref(parse(Float64, get(ENV, "JM_RH_DIAG_T", "0.5")))
const SLIPDIAG_T = Ref(-1.0e9)
const _rhn = Ref(0)

mutable struct Car3D
    sys; integ
    s_thr; s_brk; s_st; s_gr; s_clu; s_we
    s_zr::NTuple{4,Any}
    s_vr::NTuple{4,Any}
    s_pos; s_vel
    s_vreset
    s_fx; s_fy; s_mz; s_cda
    s_mu::NTuple{4,Any}
    getall
    gear::Int
    zref::Float64
    zr_prev::NTuple{4,Float64}

    x::Float64; y::Float64; z::Float64; θ::Float64
    v::Float64; t::Float64; rpm::Float64; gear_n::Int
    tc::NTuple{4,NTuple{3,Float64}}
    lapdist::Float64; laps::Int; lateral::Float64; along::Float64; ontrack::Bool
    pitch::Float64; roll::Float64; vacc::Float64; heave::Float64
    rh::NTuple{4,Float64}

    grounded::Bool

    toff::Float64
end

function _musetters(sys)
    try
        (setp(sys,sys.FL.μscale), setp(sys,sys.FR.μscale), setp(sys,sys.RL.μscale), setp(sys,sys.RR.μscale))
    catch
        nop = (integ, v) -> nothing
        (nop, nop, nop, nop)
    end
end

"""One corner spec at wheel rate `ks`, keeping every other value at the axle's shipped default."""

_karb_r(ks) = KARBR_COMP ? 0.5 * float(ks) * (MR2/MR2_R - 1.0) : 0.0
_corner(axle::Symbol, ks::Real; ch::Chassis = CHASSIS[], i::Int = 0) = (d = _dknee(ch); axle === :f ?
    (ks = float(ks), cs = 2500.0*ch.cscale[1]*d.scale, clo = d.clo, vk = d.vk, karb = 0.0, m_s = _msprung(i, 120.0), m_u = 20.0, kt = 180_000.0, ct = 1000.0) :
    (ks = float(ks), cs = 3000.0*ch.cscale[2]*d.scale, clo = d.clo, vk = d.vk, karb = _karb_r(ks), m_s = _msprung(i, 148.0), m_u = 20.0, kt = 200_000.0, ct = 1100.0))

const DAMPER_LINEAR = haskey(ENV, "JM_DAMPER_LINEAR")
_dknee(ch::Chassis) = (DAMPER_LINEAR || ch.camber === nothing) ? (scale = 1.0, clo = 1.0, vk = 0.10) : (scale = 1.3, clo = 0.30, vk = 0.08)

_msprung(i, dflt) = (MASS_LEGACY || CORNER_W[] === nothing || i == 0) ? dflt : CORNER_W[][i]/9.80665*(car_mass()/MASS[]) - 20.0

_chassis_kw(ch::Chassis = CHASSIS[]) = (diff = ch.diff, toe_f = ch.toe[1], toe_r = ch.toe[2], brake_bias_p = ch.bias_p, travel = ch.travel, karb_f = ch.karb[1], karb_r = ch.karb[2],

                                        camber = ch.camber, relax = (ch.camber === nothing && !haskey(ENV, "JM_RELAX")) ? nothing : RELAX,
                                        front_hubs = FRONT_HUBS[],
                                        tank_x = cg_arms()[1] - FUEL_TANK_X_FRONT)

const FRONT_HUBS = Ref(!haskey(ENV, "JM_FRONTSPOOL"))

const RELAX_DEFAULT = (1.25, 0.05)
const RELAX = let e = lowercase(get(ENV, "JM_RELAX", ""))
    e == "none" ? nothing : isempty(e) ? RELAX_DEFAULT : Tuple(parse.(Float64, split(e, ",")))[1:2]
end

_wheel_u0(sys, v0) = CHASSIS[].diff === nothing ? [sys.ωr => v0/RW_R] : [sys.ωRL => v0/RW_R, sys.ωRR => v0/RW_R]

const ANTISTALL = Ref(get(ENV, "JM_ANTISTALL", "1") != "0")

const TRACK_ALT_M = Dict("monza" => 182.7, "skidpad" => 302.0, "watglen" => 469.2, "nurburgring" => 620.7,
                         "spa" => 410.0, "zandvoort" => 5.0)
_p_alt(h) = (1 - 2.25577e-5*h)^5.25588
"Air density [kg/m3] at the track's altitude, reference's 25.56 C (Monza's 1.1509 as the anchor)."
air_density(track::AbstractString) = haskey(ENV, "JM_AIR_RHO") ? parse(Float64, ENV["JM_AIR_RHO"]) :
    get(ENV, "JM_TRACKAIR", "1") == "0" || !haskey(TRACK_ALT_M, track) ? 1.10 :
    1.1509*_p_alt(TRACK_ALT_M[track])/_p_alt(182.7)
"Set the car's air density (drag, lift, engine)."
set_air!(c, ρ) = ModelingToolkit.setp(c.sys, c.sys.ρair)(c.integ, ρ)

function build_car3d(; x0 = 0.0, z0 = 0.0, θ0 = 0.0, v0 = 0.0, y0 = 0.0,
                     brush = !haskey(ENV, "JM_MAGIC"), dt = 1/300)
    _cp = CAR[]; _car = _cp.name == "Lotus" ? NamedTuple() : (tf = 1.5 + _cp.dtf, tr = 1.5 + _cp.dtr, iscale = car_iscale())
    sys = mtkcompile(DrivenVehicle3D(name = :car, brush = brush, final = final_eff(), m = car_mass(), front_frac = FRONT_FRAC[],
                    fl_corner = _corner(:f, KS[][1]; i = 1), fr_corner = _corner(:f, KS[][2]; i = 2),
                    rl_corner = _corner(:r, KS[][3]; i = 3), rr_corner = _corner(:r, KS[][4]; i = 4),
                    a = cg_arms()[1], b = cg_arms()[2]; _car..., _chassis_kw()...))
    println(brush ? "  TYRE (3-D): physics-based brush model (default)" : "  TYRE (3-D): Magic-Formula tyre (JM_MAGIC)")
    prob = ODEProblem(sys, [sys.u => v0, sys.ωf => v0/0.30, _wheel_u0(sys, v0)...,
                            sys.ωe => 209.4, sys.X => x0, sys.Y => z0, sys.ψ => θ0], (0.0, 1e7))

    integ = init(prob, Rosenbrock23(); save_everystep = false, dense = false, adaptive = false, dt = dt)
    ModelingToolkit.setp(sys, sys.antistall)(integ, ANTISTALL[] ? 1.0 : 0.0)
    getall = ModelingToolkit.getsym(sys, [sys.X, sys.Y, sys.ψ, sys.u, sys.v, sys.rpm,
        sys.FL.Fx, sys.FR.Fx, sys.RL.Fx, sys.RR.Fx, sys.FL.Fy, sys.FR.Fy, sys.RL.Fy, sys.RR.Fy,
        sys.z, sys.th, sys.ph, sys.az, sys.FzFL, sys.FzFR, sys.FzRL, sys.FzRR, sys.ωr])
    c = Car3D(sys, integ, setp(sys,sys.throttle), setp(sys,sys.brake), setp(sys,sys.δ),
              setp(sys,sys.gear), setp(sys,sys.clutch), ModelingToolkit.setu(sys, sys.ωe),
              (setp(sys,sys.zrFL),setp(sys,sys.zrFR),setp(sys,sys.zrRL),setp(sys,sys.zrRR)),
              (setp(sys,sys.vrFL),setp(sys,sys.vrFR),setp(sys,sys.vrRL),setp(sys,sys.vrRR)),
              ModelingToolkit.setu(sys,[sys.X,sys.Y]), ModelingToolkit.setu(sys,[sys.u,sys.v]),
              ModelingToolkit.setu(sys,[sys.z,sys.w,sys.th,sys.q,sys.ph,sys.pp,
                  sys.wFL.zu,sys.wFL.vu,sys.wFR.zu,sys.wFR.vu,sys.wRL.zu,sys.wRL.vu,sys.wRR.zu,sys.wRR.vu]),
              setp(sys,sys.Fx_ext),setp(sys,sys.Fy_ext),setp(sys,sys.Mz_ext),setp(sys,sys.CdA_scale),
              _musetters(sys),
              getall, 1, y0, ntuple(_->0.0,4),
              x0, y0, z0, θ0, v0, 0.0, 0.0, 1, ntuple(_->(0.0,0.0,0.0),4),
              0.0, 0, 0.0, 0.0, true, 0.0, 0.0, 9.80665, 0.0, RIDE_H[], false, 0.0)
    c.s_gr(c.integ, GEARS[c.gear]); getall(integ)
    for _ in 1:3; step_car3d!(c, 0.3, 0.0, 0.0, 1/60); end
    for _ in 1:3; step_car3d!(c, 0.3, 0.0, 0.0, 1/60; clutch = 0.5, manual = true); end
    reinit!(c.integ); c.gear = 0; c.s_gr(c.integ, 0.0)
    a = getall(c.integ); c.x = a[1]; c.z = a[2]; c.θ = a[3]; c.v = sqrt(a[4]^2 + a[5]^2); c.rpm = a[6]
    c.zref = y0; c.zr_prev = ntuple(_->0.0, 4)
    delete!(_VREF, c)
    c
end

"""Compile the 3-D car ONCE and build `length(poses)` cars sharing the system (so a field of
3-D AI doesn't pay N× mtkcompile).  `poses` = Vector of (x0,z0,θ0,v0)."""
function build_cars3d(poses; brush = !haskey(ENV, "JM_MAGIC"), dt = 1/300)
    with_lotus(() -> _build_cars3d(poses; brush, dt))
end
function _build_cars3d(poses; brush = !haskey(ENV, "JM_MAGIC"), dt = 1/300)
    sys = mtkcompile(DrivenVehicle3D(name = :car, brush = brush, final = FINAL[], m = MASS[], front_frac = FRONT_FRAC[],
                    fl_corner = _corner(:f, KS[][1]; i = 1), fr_corner = _corner(:f, KS[][2]; i = 2),
                    rl_corner = _corner(:r, KS[][3]; i = 3), rr_corner = _corner(:r, KS[][4]; i = 4),
                    a = cg_arms()[1], b = cg_arms()[2]; _chassis_kw()...))
    s_thr=setp(sys,sys.throttle); s_brk=setp(sys,sys.brake); s_st=setp(sys,sys.δ)
    s_gr=setp(sys,sys.gear); s_clu=setp(sys,sys.clutch); s_we=ModelingToolkit.setu(sys,sys.ωe)
    s_zr=(setp(sys,sys.zrFL),setp(sys,sys.zrFR),setp(sys,sys.zrRL),setp(sys,sys.zrRR))
    s_vr=(setp(sys,sys.vrFL),setp(sys,sys.vrFR),setp(sys,sys.vrRL),setp(sys,sys.vrRR))
    s_pos=ModelingToolkit.setu(sys,[sys.X,sys.Y]); s_vel=ModelingToolkit.setu(sys,[sys.u,sys.v])
    s_vreset=ModelingToolkit.setu(sys,[sys.z,sys.w,sys.th,sys.q,sys.ph,sys.pp,
                  sys.wFL.zu,sys.wFL.vu,sys.wFR.zu,sys.wFR.vu,sys.wRL.zu,sys.wRL.vu,sys.wRR.zu,sys.wRR.vu])
    s_fx=setp(sys,sys.Fx_ext); s_fy=setp(sys,sys.Fy_ext); s_mz=setp(sys,sys.Mz_ext); s_cda=setp(sys,sys.CdA_scale)
    s_mu=_musetters(sys)
    getall=ModelingToolkit.getsym(sys, [sys.X, sys.Y, sys.ψ, sys.u, sys.v, sys.rpm,
        sys.FL.Fx, sys.FR.Fx, sys.RL.Fx, sys.RR.Fx, sys.FL.Fy, sys.FR.Fy, sys.RL.Fy, sys.RR.Fy,
        sys.z, sys.th, sys.ph, sys.az, sys.FzFL, sys.FzFR, sys.FzRL, sys.FzRR, sys.ωr])
    cars=Car3D[]
    for (x0,z0,θ0,v0) in poses
        prob = ODEProblem(sys, [sys.u=>v0, sys.ωf=>v0/0.30, _wheel_u0(sys, v0)...,
                                sys.ωe=>209.4, sys.X=>x0, sys.Y=>z0, sys.ψ=>θ0], (0.0,1e7))
        integ = init(prob, Rosenbrock23(); save_everystep=false, dense=false, adaptive=false, dt=dt)
        c = Car3D(sys, integ, s_thr,s_brk,s_st,s_gr,s_clu,s_we, s_zr, s_vr, s_pos,s_vel, s_vreset,
                  s_fx,s_fy,s_mz,s_cda, s_mu,
                  getall, 1, 0.0, ntuple(_->0.0,4),
                  x0, 0.0, z0, θ0, v0, 0.0, 0.0, 1, ntuple(_->(0.0,0.0,0.0),4),
                  0.0, 0, 0.0, 0.0, true, 0.0, 0.0, 9.80665, 0.0, RIDE_H[], false, 0.0)
        c.s_gr(c.integ, GEARS[c.gear]); getall(integ)
        for _ in 1:3; step_car3d!(c, 0.3, 0.0, 0.0, 1/60); end
        for _ in 1:3; step_car3d!(c, 0.3, 0.0, 0.0, 1/60; clutch=0.5, manual=true); end
        reinit!(c.integ); c.gear=0; c.s_gr(c.integ, 0.0)
        a=getall(c.integ); c.x=a[1]; c.z=a[2]; c.θ=a[3]; c.v=sqrt(a[4]^2+a[5]^2); c.rpm=a[6]
        c.zref=0.0; c.zr_prev=ntuple(_->0.0,4); delete!(_VREF, c)
        push!(cars, c)
    end
    cars
end

const VFRAME_INERTIAL = lowercase(get(ENV, "JM_VFRAME", "inertial")) != "legacy"
const _VREF = IdDict{Any,Float64}()
const _VSTATE = IdDict{Any,Any}()
_vstate(c) = get!(_VSTATE, c.sys) do
    vs = [c.sys.w, c.sys.wFL.vu, c.sys.wFR.vu, c.sys.wRL.vu, c.sys.wRR.vu]
    (ModelingToolkit.getsym(c.sys, vs), ModelingToolkit.setu(c.sys, vs))
end
"Advance the 3-D car by dt.  Inputs as DriveRT.step_car!.  `groundz(x,z)->h`
gives terrain elevation (used to drive the suspension under each wheel)."
function step_car3d!(c::Car3D, throttle, brake, steer, dt;
                     clutch = 0.0, up = false, dn = false, manual = false,
                     groundz = (x,z)->0.0, lock = 1.0)
    if manual
        c.s_clu(c.integ, clamp(clutch, 0, 1))
        up && c.gear < 5 && (c.gear += 1; c.s_gr(c.integ, gearratio(c.gear)))
        dn && (c.gear > 0 || (REVERSE_ON && c.gear == 0 && abs(c.v) < 1.0)) && (c.gear -= 1; c.s_gr(c.integ, gearratio(c.gear)))
    else

        if REVERSE_ON && dn && abs(c.v) < 1.0 && 0 <= c.gear <= 1
            c.gear = -1; c.s_gr(c.integ, gearratio(-1))
        elseif up && c.gear == -1
            c.gear = 0
        end

        if c.gear == 0
            g = 1
            while g < 5 && abs(c.v) > 0.9 * (9500.0*2π/60) / (GEARS[g]*FINAL[]) * RW_R; g += 1; end
            c.gear = g; c.s_gr(c.integ, GEARS[g])
        end
        held = abs(c.v) < 1.0 && throttle < 0.05
        ae = held ? 0.0 : clamp((c.rpm - 1400.0)/1000.0, 0, 1) * clamp(max(2*throttle, c.v/2), 0, 1)
        c.s_clu(c.integ, clamp(1.0 - ae, 0, 1))
    end
    thr = clamp(throttle, 0, 1)
    if TC_ON && thr > 0.0
        a0 = c.getall(c.integ)

        gspeed = clamp((abs(a0[4]) - TC_VLO) / (TC_VHI - TC_VLO), 0.0, 1.0)
        gsteer = TC_STEERGATE ?
                 clamp((abs(steer) - TC_STLO) / max(TC_STHI - TC_STLO, 1e-6), 0.0, 1.0) : 0.0
        gate = max(gspeed, gsteer)
        if gate > 0.0
            κr = (a0[23]*RW_R - a0[4]) / max(abs(a0[4]), 3.0)
            κr > TC_SLIP && (thr *= clamp(1.0 - 4.0*gate*(κr - TC_SLIP)/TC_SLIP, 0.05, 1.0))
        end
    end
    c.s_thr(c.integ, thr); c.s_brk(c.integ, clamp(brake, 0, 1))
    c.s_st(c.integ, clamp(steer, -lock, lock) * MAXSTEER)

    Δ = max(dt, 1e-3); nsub = max(1, round(Int, Δ*300)); subdt = Δ/nsub
    local a = c.getall(c.integ)
    local terr = ntuple(_->0.0, 4)
    local grounded_last = false
    for _ in 1:nsub
        x = a[1]; z = a[2]; θ = a[3]; cosθ = cos(θ); sinθ = sin(θ)
        u = a[4]; v = a[5]
        vwx = u*cosθ - v*sinθ;  vwz = u*sinθ + v*cosθ
        spd = hypot(vwx, vwz); dirx = spd>0.5 ? vwx/spd : cosθ;  dirz = spd>0.5 ? vwz/spd : sinθ
        terr = ntuple(4) do i
            xi, yi = WHEELS[i]
            wx = x + xi*cosθ - yi*sinθ;  wz = z + xi*sinθ + yi*cosθ
            h = groundz(wx, wz);  isfinite(h) ? Float64(h) : c.zref
        end

        dL = 1.0
        vr = ntuple(4) do i
            xi, yi = WHEELS[i]
            wx = x + xi*cosθ - yi*sinθ;  wz = z + xi*sinθ + yi*cosθ
            ha = groundz(wx + dirx*dL, wz + dirz*dL)
            isfinite(ha) ? clamp((Float64(ha) - terr[i]) / dL * spd, -25.0, 25.0) : 0.0
        end
        terr_cg = sum(terr)/4
        ΣFz = a[19] + a[20] + a[21] + a[22]
        grounded = ΣFz > 0.15 * 617 * 9.80665
        grounded_last = grounded; c.grounded = grounded
        vr_cg = (vr[1]+vr[2]+vr[3]+vr[4])/4

        if VFRAME_INERTIAL

            vref = isfinite(terr_cg) ? vr_cg + clamp(terr_cg - c.zref, -1.5, 1.5)*10.0 : get(_VREF, c, 0.0)

            vref0 = get(_VREF, c, vref)
            if vref != vref0
                gs = _vstate(c)
                gs[2](c.integ, gs[1](c.integ) .+ (vref0 - vref)); OrdinaryDiffEq.SciMLBase.u_modified!(c.integ, true)
            end
            _VREF[c] = vref; c.zref += vref*subdt
            zr = ntuple(i -> terr[i] - c.zref, 4)
            for i in 1:4; c.s_zr[i](c.integ, zr[i]); c.s_vr[i](c.integ, vr[i] - vref); end
        else
        (grounded && isfinite(terr_cg)) && (c.zref += vr_cg*subdt + clamp(terr_cg - c.zref, -1.5, 1.5)*10.0*subdt)
        zr = ntuple(i -> terr[i] - c.zref, 4)
        for i in 1:4; c.s_zr[i](c.integ, zr[i]); c.s_vr[i](c.integ, vr[i]); end
        end
        c.zr_prev = zr
        step!(c.integ, subdt, true)
        a = c.getall(c.integ)
    end
    c.x = a[1]; c.z = a[2]; c.θ = a[3]
    c.v = sqrt(a[4]^2 + a[5]^2); c.t = c.integ.t + c.toff

    c.rpm = clamp(a[6], 0.0, 9700.0*CAR[].kR); c.gear_n = c.gear
    c.heave = a[15]; c.pitch = a[16]; c.roll = a[17]; c.vacc = a[18]

    fztot = a[19] + a[20] + a[21] + a[22]

    if haskey(ENV, "JM_SLIPDIAG")
        _sd_every = parse(Float64, get(ENV, "JM_SLIPDIAG", "0.25"))
        if c.t - SLIPDIAG_T[] >= _sd_every
            SLIPDIAG_T[] = c.t
            fzf = a[19] + a[20]; fzr = a[21] + a[22]
            fyf = a[11] + a[12]; fyr = a[13] + a[14]
            uf  = fzf > 1.0 ? abs(fyf)/fzf : 0.0
            ur  = fzr > 1.0 ? abs(fyr)/fzr : 0.0
            println("  [slip] t=", round(c.t, digits=2),
                    " v=", round(c.v*3.6, digits=1), " km/h",
                    "  front |Fy|/Fz=", round(uf, digits=3),
                    "  rear=", round(ur, digits=3),
                    "  load f/r=", round(fzf, digits=0), "/", round(fzr, digits=0), " N",
                    "  yaw=", round(rad2deg(yawrate3d(c)), digits=1), " deg/s")
            flush(stdout)
        end
    end
    airborne = fztot < 600.0
    if !(isfinite(c.pitch) && isfinite(c.roll) && isfinite(c.heave)) || abs(c.heave) > 12.0 ||
       (!airborne && (abs(c.pitch) > 0.7 || abs(c.roll) > 0.7)) ||
       (airborne && (abs(c.pitch) > 12.0 || abs(c.roll) > 12.0))
        c.s_vreset(c.integ, zeros(14)); delete!(_VREF, c)
        c.heave = 0.0; c.pitch = 0.0; c.roll = 0.0; c.vacc = 9.80665
    end
    if c.rpm < 350.0 && (clamp(clutch, 0, 1) > 0.5 || c.gear == 0)
        c.s_we(c.integ, 209.44); c.rpm = 2000.0
    end
    c.y = c.zref + c.heave
    mg4 = 617.0*9.80665/4

    μc = (1.36, 1.36, 1.40, 1.40)
    c.tc = ntuple(i -> (a[6+i]/mg4, a[10+i]/mg4, μc[i]*max(a[18+i],1.0)/mg4), 4)

    c.rh = ntuple(i -> RIDE_H[][i] + (WHEELS[i][1]*c.pitch + WHEELS[i][2]*c.roll + c.heave) - (terr[i]-c.zref), 4)

    if JM_RH_DIAG[] && any(x -> !isfinite(x) || abs(x) > JM_RH_DIAG_T[], c.rh)
        _rhn[] += 1
        if _rhn[] <= 40
            local rr = round.(c.rh, digits=3)
            local tt = round.(terr, digits=2)
            println("  JM_RH_DIAG #", _rhn[], " rh=", rr, " zref=", round(c.zref, digits=2),
                    " terr=", tt, " heave=", round(c.heave, digits=3),
                    " grounded=", grounded_last, " y=", round(c.y, digits=2))
        end
    end

    let gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET3D, c.sys)
        rr = gs[1](c.integ)

        (!isfinite(rr) || abs(rr) > 10.0) && gs[2](c.integ, isfinite(rr) ? clamp(rr, -0.8, 0.8) : 0.0)
    end
    (!isfinite(c.v) || abs(c.v) > 110) && return respawn3d!(c)
    if !manual && c.gear >= 1

        grpm = (a[4]/RW_R)*GEARS[c.gear]*FINAL[]*60/(2π)
        up_rpm = SHIFT_RPM[]
        if grpm > up_rpm && c.gear < 5;                      c.gear += 1; c.s_gr(c.integ, GEARS[c.gear])
        elseif grpm < 4100 && c.gear > 1 && throttle < 0.85; c.gear -= 1; c.s_gr(c.integ, GEARS[c.gear]); end
    end
    c
end

const _TC = IdDict{Any,Any}()
function telemetry3d(c::Car3D)
    g = get!(_TC, c.sys) do
        lsd = hasproperty(c.sys, :ωRL)
        ModelingToolkit.getsym(c.sys, [c.sys.u, c.sys.v, c.sys.r, c.sys.ax, c.sys.ay, c.sys.ωf, c.sys.ωr,
                                       lsd ? c.sys.ωRL : c.sys.ωr, lsd ? c.sys.ωRR : c.sys.ωr])
    end
    a = g(c.integ)
    (u=a[1], v=a[2], r=a[3], ax=a[4], ay=a[5], ωf=a[6], ωr=a[7], ωRL=a[8], ωRR=a[9], vacc=c.vacc,
     pitch=c.pitch, roll=c.roll, rh=c.rh, grounded=c.grounded)
end

const _BRK3D = IdDict{Any,Any}()
"""CARPHYS-1 S6: the brake line pressures [bar], (front, rear) -- the .ibt's brakeLinePress (left = right per axle)."""
function brakepress3d(c::Car3D)
    g = get!(_BRK3D, c.sys) do
        try ModelingToolkit.getsym(c.sys, [c.sys.brk.pF, c.sys.brk.pR]) catch; nothing end
    end
    g === nothing ? nothing : g(c.integ)
end

const _RIM3D = IdDict{Any,Any}()
"""CARPHYS-1 S5: the steering column's rim torque [N·m] (SteeringColumn: front axle Fy on the fitted 4.78 cm trail,
10:1 rack; positive turns the rim left), or `nothing` on a model without the column."""
function rimtorque3d(c::Car3D)
    g = get!(_RIM3D, c.sys) do
        try ModelingToolkit.getsym(c.sys, c.sys.col.τ) catch; nothing end
    end
    g === nothing ? nothing : g(c.integ)
end

const _GRIP3D = IdDict{Any,Any}()
"""REPLAY-6: each tyre's force and grip, FL FR RL RR, as (Fx, Fy, μx·Fz, μy·Fz, ξ) -- forces in mg/4 (the static corner
weight), the friction ellipse's semi-axes from the tyre model's own load-sensitive, off-road-scaled μ, and ξ the brush's
normalised slip (1 = the whole contact patch sliding). `nothing` for a tyre model without those variables (JM_MAGIC)."""
function tyregrip3d(c::Car3D)
    g = get!(_GRIP3D, c.sys) do
        s = c.sys
        try
            ModelingToolkit.getsym(s, [s.FL.Fx, s.FR.Fx, s.RL.Fx, s.RR.Fx, s.FL.Fy, s.FR.Fy, s.RL.Fy, s.RR.Fy,
                                       s.FL.Fz, s.FR.Fz, s.RL.Fz, s.RR.Fz, s.FL.μxe, s.FR.μxe, s.RL.μxe, s.RR.μxe,
                                       s.FL.μye, s.FR.μye, s.RL.μye, s.RR.μye, s.FL.ξ, s.FR.ξ, s.RL.ξ, s.RR.ξ])
        catch
            nothing
        end
    end
    g === nothing && return nothing
    a = g(c.integ); mg4 = 617.0*9.80665/4
    ntuple(i -> (a[i]/mg4, a[4+i]/mg4, a[12+i]*max(a[8+i], 0.0)/mg4, a[16+i]*max(a[8+i], 0.0)/mg4, a[20+i]), 4)
end

"Fence collision: snap onto the boundary (xnew,znew) + bleed speed (E7).
E42: `settle` zeroes the VERTICAL subsystem and re-anchors zref to the terrain under the
snap point — without it, snapping back from a steep off-world slide leaves the suspension
loaded against the old (down-a-field) height, so re-entry slams the contact and the car
'superball'-bounces back onto the track.  Pass `groundz` so zref lands on the real edge height."
function contain3d!(c::Car3D, xnew, znew; vdamp = 0.45, settle = false, groundz = nothing)
    a = c.getall(c.integ)
    c.s_pos(c.integ, [xnew, znew])
    c.s_vel(c.integ, [a[4]*vdamp, a[5]*vdamp])
    c.x = xnew; c.z = znew; c.v = sqrt((a[4]*vdamp)^2 + (a[5]*vdamp)^2)
    if settle
        c.s_vreset(c.integ, zeros(14)); delete!(_VREF, c)
        c.heave = 0.0; c.pitch = 0.0; c.roll = 0.0; c.vacc = 9.80665
        if groundz !== nothing
            h = groundz(xnew, znew); isfinite(h) && (c.zref = Float64(h))
        end
        c.zr_prev = ntuple(_ -> 0.0, 4); c.y = c.zref
    end
    c
end

function respawn3d!(c::Car3D; groundz = nothing)

    g0 = get(ENV,"JM_SPAWN_IN_GEAR","0") != "0" ? 1 : 0

    c.toff += c.integ.t
    reinit!(c.integ); c.gear = g0; c.s_gr(c.integ, gearratio(g0))
    c.s_vreset(c.integ, zeros(14)); delete!(_VREF, c)
    a = c.getall(c.integ); c.x = a[1]; c.z = a[2]; c.θ = a[3]; c.v = a[4]; c.rpm = a[6]
    c.heave = 0.0; c.pitch = 0.0; c.roll = 0.0; c.vacc = 9.80665

    if groundz !== nothing
        h = groundz(c.x, c.z); c.zref = isfinite(h) ? Float64(h) : 0.0
    end
    c.zr_prev = ntuple(_ -> 0.0, 4); c.y = c.zref
    c
end

"""E56 spring-damper trackside CONTACT kernel (the human car's all-Modelica collision response).
Given a penetration `δ` [m, >0] into an object, the world-frame OUTWARD normal `(nx,nz)` (object→car),
the car's velocity component `vn` along +n [m/s; <0 = driving INTO the object], heading `θ`, and the
object `kind`, return the body-frame `(Fx,Fy,Mz)` to feed `extforce3d!` BEFORE the step.

  :wall  — soft spring + STRONG two-sided damper ⇒ INELASTIC: drive in, stop dead, stay put.
           (E94: this line used to say "stiff spring ... ELASTIC bounce-back", which described
            neither the code below it nor the behaviour anyone wanted.)
  :soft  — weak spring (with a crush `give` dead-zone) + two-sided viscous damper ⇒ the car drives
           IN, the damper bleeds its speed, and the soft spring barely pushes back ⇒ it gets STUCK
           (a hedge / hay row), yet can't pass clean through (deep penetration past `give` resists).

The outward force is clamped to a per-frame impulse ≤ m·CONTACT_DVMAX (a penalty contact) so a stiff
wall held constant over a render frame can't blow the integrator up.  Contact can only PUSH, not pull."""
const SPRING_DMAX = parse(Float64, get(ENV, "JM_SPRING_DMAX", "0.25"))

const VN_OUT_MAX = parse(Float64, get(ENV, "JM_VN_OUT_MAX", "0.25"))
const CONTACT_DVMAX = 8.0

const CONTACT_DVMAX_SOFT = 1.0

const SOFT_DVMAX_OVERRIDE = Ref(NaN)
soft_dvmax(δ) = isfinite(SOFT_DVMAX_OVERRIDE[]) ? SOFT_DVMAX_OVERRIDE[] :
    CONTACT_DVMAX_SOFT + (CONTACT_DVMAX - CONTACT_DVMAX_SOFT) * clamp((δ - 0.3) / 0.5, 0.0, 1.0)
function contact_force(δ, nx, nz, vn, θ; kind = :wall, m = 617.0, dt = 1/60, arm = 1.4)
    δ <= 0.0 && return (0.0, 0.0, 0.0)

    if kind === :soft
        k = 8.0e4; c = 7.0e4; give = 0.3; twoSided = true
    else

        if get(ENV, "JM_ELASTIC_WALL", "0") != "0"
            k = 4.0e5; c = 1.5e5; give = 0.0; twoSided = true
        else
            k = 1.0e5; c = 3.0e5; give = 0.05; twoSided = true
        end
    end
    δeff = max(δ - give, 0.0)

    δspring = min(δeff, SPRING_DMAX)
    damp = twoSided ? -c*vn : -c*min(vn, 0.0)
    Fn = max(k*δspring + damp, 0.0)
    Fn = min(Fn, m*(kind === :soft ? soft_dvmax(δ) : CONTACT_DVMAX)/max(dt, 1e-3))

    Fn = min(Fn, m*max(VN_OUT_MAX - vn, 0.0)/max(dt, 1e-3))

    if vn > 0.0 && δ > 0.0

        Fn = -m*max(vn - VN_OUT_MAX, 0.0)/max(dt, 1e-3)
        kind === :soft && (Fn = max(Fn, -m*soft_dvmax(δ)/max(dt, 1e-3)))
    end
    cθ = cos(θ); sθ = sin(θ)
    Fx = Fn*( nx*cθ + nz*sθ);  Fy = Fn*(-nx*sθ + nz*cθ)
    rx = (-nx*cθ - nz*sθ)*arm; ry = ( nx*sθ - nz*cθ)*arm
    Mz = rx*Fy - ry*Fx
    (Fx, Fy, Mz)
end

"""SPA-BARRIER (PO replay 2026-09-03, Spa s≈7208 m): `contact_force` bounds ONE contact so the car
leaves at ≤ VN_OUT_MAX -- but `solid_contact` SUMS every overlapping solid, and Spa's placement list
carries the same house three times at one spot (38 exact duplicates track-wide). Three stacked
discs each granted the full "cancel the approach + 0.25 m/s" impulse, so a 37 m/s hit left at
−37 + 3·37.25 ≈ +75 m/s: the replay shows the car thrown back the way it came at 70 m/s -- the
PO's "bounce off an invisible barrier". Cap the TOTAL body-frame contact force the same way the
kernel caps one: along the net push direction the outcome may reach VN_OUT_MAX and never more.
`vbx`,`vby` are the car's world velocity in the body frame (the same frame as `Fx`,`Fy`).
Returns (Fx, Fy, scale); scale < 1 means the cap bit. Pure, so the gate can prove it."""
function cap_total_contact(Fx, Fy, vbx, vby; m = 617.0, dt = 1/60, vout = VN_OUT_MAX)
    F = hypot(Fx, Fy)
    F <= 1e-9 && return (Fx, Fy, 1.0)
    ux = Fx/F; uy = Fy/F
    v_along = vbx*ux + vby*uy

    Fmax = m*min(CONTACT_DVMAX, max(vout - v_along, 0.0))/max(dt, 1e-3)
    F <= Fmax && return (Fx, Fy, 1.0)
    sc = Fmax/F
    (Fx*sc, Fy*sc, sc)
end

"""E56 ALL-MODELICA CONTACT: feed body-frame external force `Fx`,`Fy` [N] + yaw moment `Mz` [N·m]
and an aero drag scale `CdA_scale` (≤1 in a slipstream = less drag = tow) into the chassis ODE.
Unlike `bump3d!` (an instantaneous velocity/rate STATE hack), these are PARAMETER ports the solver
INTEGRATES over the step — so a spring-damper contact law `F = kδ + cδ̇` becomes a real, momentum-
conserving collision (wall = stiff k → bounce; hedge = weak k + strong c → drive in & get stuck).
Call once per frame BEFORE `step_car3d!`; the values HOLD until the next call, so pass the *total*
contact force for the frame and call with no contact (defaults) to release (`extforce3d!(c)`)."""
function extforce3d!(c::Car3D; Fx = 0.0, Fy = 0.0, Mz = 0.0, CdA_scale = 1.0)
    c.s_fx(c.integ, Fx); c.s_fy(c.integ, Fy); c.s_mz(c.integ, Mz)
    c.s_cda(c.integ, clamp(CdA_scale, 0.0, 2.0))
    c
end

"""E56 grass grip: set each wheel's tyre friction multiplier (FL,FR,RL,RR; 1 = tarmac, <1 = grass).
A wheel on the verge loses real grip in the brush tyre model — so two wheels on the grass pull the
car and cost cornering/braking, exactly as in Classic — replacing the bumpX! grass drag/yaw hack.  Call
before step_car3d! each frame; pass 1.0 for wheels on the racing surface."""
function wheelmu3d!(c::Car3D, μFL, μFR, μRL, μRR)
    c.s_mu[1](c.integ, clamp(μFL, 0.05, 1.0)); c.s_mu[2](c.integ, clamp(μFR, 0.05, 1.0))
    c.s_mu[3](c.integ, clamp(μRL, 0.05, 1.0)); c.s_mu[4](c.integ, clamp(μRR, 0.05, 1.0))
    c
end

const _RSET3D = IdDict()
function yawrate3d(c::Car3D)
    gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET3D, c.sys)
    gs[1](c.integ)
end
const _PSI3D = IdDict()
"Place the 3-D car at world (x,z) facing θ with forward speed v (e.g. onto a grid slot)."
function place3d!(c::Car3D, x, z, θ; v = 0.0)
    c.s_pos(c.integ, [x, z])
    try
        pset = get!(() -> ModelingToolkit.setu(c.sys, c.sys.ψ), _PSI3D, c.sys)
        pset(c.integ, θ)
    catch; end
    c.s_vel(c.integ, [v, 0.0])
    c.x = x; c.z = z; c.θ = θ; c.v = v
    c
end
"""HEDGEJOLT-1 (PO 2026-10-18: "when the car ran into a hedge, there was levitating and fibrilating"): a frozen wreck was
re-placed every frame with `place3d!` -- position, heading, speed -- but its yaw rate and vertical states ran on: a corner
the impact left 1.1 m down drove the body up and down 0.2 m and the nose +-7 deg between placements, for as long as the
wreck sat there. `pin3d!` also zeroes the yaw rate and the vertical subsystem and puts the body on the ground at `h`."""
function pin3d!(c::Car3D, x, z, θ, h)
    place3d!(c, x, z, θ; v = 0.0)
    gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET3D, c.sys)
    gs[2](c.integ, 0.0)
    c.s_vreset(c.integ, zeros(14)); delete!(_VREF, c)
    c.heave = 0.0; c.pitch = 0.0; c.roll = 0.0; c.vacc = 9.80665
    isfinite(h) && (c.zref = Float64(h))
    c.zr_prev = ntuple(_ -> 0.0, 4); c.y = c.zref
    OrdinaryDiffEq.SciMLBase.u_modified!(c.integ, true)
    c
end
const _WSET3D = IdDict()
const _PPSET3D = IdDict()
const _QSET3D = IdDict()
"""
World-frame velocity `(vx, vz)` of the car, taken EXACTLY from the solver state.

E96-S6: contact code needs the true direction of travel, and `c.v` is an unsigned SPEED -- the
`v*cos(θ), v*sin(θ)` form cannot express a car moving opposite its own nose, which is what turned
the no-rebound clamp into an accelerator (E96-S2). E96-S2 worked around that by finite-differencing
the position, which is correct in direction but lags one frame; that lag is the leading suspect for
the residual overshoot E96-S3 could not remove.

The solver has carried the answer all along: `u` (longitudinal) and `v` (LATERAL) body-frame
velocities, which `bump3d!` already rotates into the world exactly this way. No differencing, no
lag, no extra state to keep in sync.
"""
function world_velocity(c::Car3D)
    a = c.getall(c.integ); θ = a[3]; u = a[4]; v = a[5]
    (u*cos(θ) - v*sin(θ), u*sin(θ) + v*cos(θ))
end

"""Rigid-body collision impulse (3-D car): world-frame velocity change (dvx,dvz) + yaw-rate dr
+ an optional VERTICAL kick `dvy` (heave velocity `w` → the car JUMPS) + a ROLL-rate kick `dpp`
(roll velocity → CARTWHEEL when a spinning wheel climbs) + a PITCH-rate kick `dq` (a rear wheel
climbing a hay bale pitches the nose down ⇒ LIFTS THE REAR, Classic-style)."""
function bump3d!(c::Car3D, dvx, dvz, dr, dvy = 0.0, dpp = 0.0, dq = 0.0)
    a = c.getall(c.integ); θ = a[3]; u = a[4]; v = a[5]
    wvx = u*cos(θ) - v*sin(θ) + dvx
    wvz = u*sin(θ) + v*cos(θ) + dvz
    nu =  wvx*cos(θ) + wvz*sin(θ)
    nv = -wvx*sin(θ) + wvz*cos(θ)
    c.s_vel(c.integ, [nu, nv])
    if dr != 0.0
        try
            gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET3D, c.sys)
            gs[2](c.integ, gs[1](c.integ) + dr)
        catch; end
    end
    if dvy != 0.0
        try
            ws = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.w), ModelingToolkit.setu(c.sys, c.sys.w)), _WSET3D, c.sys)
            ws[2](c.integ, ws[1](c.integ) + dvy)
        catch; end
    end
    if dpp != 0.0
        try
            ps = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.pp), ModelingToolkit.setu(c.sys, c.sys.pp)), _PPSET3D, c.sys)
            ps[2](c.integ, ps[1](c.integ) + dpp)
        catch; end
    end
    if dq != 0.0
        try
            qs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.q), ModelingToolkit.setu(c.sys, c.sys.q)), _QSET3D, c.sys)
            qs[2](c.integ, qs[1](c.integ) + dq)
        catch; end
    end
    c.v = sqrt(nu^2 + nv^2)
    c
end

"""
    stall_step(t, dt; auto, wrecked, rpm, clutch, rpm_floor, secs) -> (t′, fire)

E98: in MANUAL, a stalled engine drops straight to AUTO (PO 2026-08-30).

All four conditions are required, and each is load-bearing:
  * MANUAL — auto is already the destination, so firing there is meaningless.
  * rpm below the floor — the engine is actually dead.
  * clutch ENGAGED (< 0.4) — an idling engine with the clutch OUT is not stalled. That is the
    PO's own case: "applying throttle with the slider down just revs the engine, as it should".
    With the clutch riding, a low rpm is the driver's choice, not a failure.
  * NOT wrecked — a wreck decouples the engine deliberately and must not read as a stall.

Sustained for `secs` so a launch dip cannot trigger it; the timer resets the moment any
condition lifts. Returns the new accumulator and whether the switch should fire.

S366: extracted from the render loop so it can be gated. It was four booleans and an
accumulator woven into a 900-line frame body, which meant the only way to test the PO's
rule was to drive the car and watch — and every condition above is a place to get it
backwards silently.
"""

function stall_step(t::Float64, dt::Float64; auto::Bool, wrecked::Bool, rpm::Float64,
                    clutch::Float64, rpm_floor::Float64=300.0, secs::Float64=0.5,
                    speed::Float64=0.0, vmax::Float64=Inf)
    if !auto && !wrecked && rpm < rpm_floor && clutch < 0.4 && speed < vmax
        t += (dt > 1e-4 ? dt : 1/60)
        return t >= secs ? (0.0, true) : (t, false)
    end
    return (0.0, false)
end

end
