

module Garage

using TOML, Printf

export GarageParams, Knobs, Derived, PARAMS, solve_static, knobs_from_setup, base_knobs, KNOB_SPECS,
       arb_rate, gear_speeds_kmh, describe_derived, apply_garage, carsetup_yaml, read_garage_file, write_garage_file, GARAGE_TOML,
       julia_setup

const G = 9.80665
const CORNERS = (:LF, :RF, :LR, :RR)
const INCH64 = 25.4/64

Base.@kwdef struct GarageParams

    L::NTuple{2,Float64}      = (149.6, 120.0)
    MR::NTuple{2,Float64}     = (0.83, 0.80)
    MRq::NTuple{2,Float64}    = (0.0, 0.0)
    MRk::NTuple{2,Float64}    = (0.83, 0.80)
    bar_shown::Float64        = 1.0
    rh_mix::NTuple{2,Float64} = (0.0, 0.0)
    s0::NTuple{2,Float64}     = (61.7, 58.7)
    R0::NTuple{2,Float64}     = (90.0, 110.0)
    unsprung::NTuple{2,Float64} = (190.0, 260.0)
    kt::NTuple{2,Float64}     = (300.0, 300.0)
    kt_p::Float64             = 0.0
    rp::NTuple{2,Float64}     = (0.0, 0.0)
    karb::NTuple{2,Float64}   = (10.0, 10.0)
    arm_beta::NTuple{2,Float64} = (0.08, 0.08)
    kbump::NTuple{2,Float64}  = (200.0, 200.0)
    bump_gap::NTuple{2,Float64} = (5.0, 5.0)
    trackratio::Float64       = 1.0
    track_f::Float64          = 1530.0
    scrub::NTuple{2,Float64}  = (0.0, 0.0)
    cam_patch::Float64        = 0.0
    cam_ref::NTuple{2,Float64} = (-0.4, -0.5)

    cam_jack::Float64         = 0.0
    cam_jack2::Float64        = 0.0

    W0::Float64               = 5936.0 - 43.2*0.75*G
    front0::Float64           = 0.448
    fuel_kgL::Float64         = 0.75
    fuel_front::Float64       = 0.66

    cam_gain::NTuple{2,Float64} = (0.027, 0.02)
    cam_roll::NTuple{2,Float64} = (0.0, 0.0)
    toe_gain::Float64         = 0.016

    rpm_r::Float64            = 3145.2
end

Base.@kwdef mutable struct Knobs
    perch::NTuple{4,Float64}   = (165.1, 165.1, 103.6, 103.6)
    spring::NTuple{4,Float64}  = (30.0, 30.0, 48.0, 48.0)
    packer::NTuple{4,Float64}  = (0.0, 0.0, 0.0, 0.0)
    camber::NTuple{4,Float64}  = (-0.4, -0.4, -0.5, -0.5)
    pressure::NTuple{4,Float64} = (152.0, 152.0, 152.0, 152.0)
    bump::NTuple{4,Int}        = (4, 4, 4, 4)
    rebound::NTuple{4,Int}     = (8, 8, 7, 7)
    toe_front::Float64         = -1.0
    toe_rear::Float64          = 1.0
    farb_dia::Float64          = 0.375
    farb_arm::String           = "2"
    rarb_dia::Float64          = 0.6875
    rarb_arm::String           = "5"
    fuel::Float64              = 43.2
    brake_bias::Float64        = 53.5
    steering_ratio::Float64    = 10.0
    final_drive::Float64       = 4.22
    gears::Vector{Float64}     = [2.23, 1.72, 1.32, 1.04, 0.846]
    diff_plates::Int           = 4
    diff_preload::Float64      = 41.0
    diff_drive::Float64        = 75.0
    diff_coast::Float64        = 60.0
end
base_knobs() = Knobs()

struct Derived
    cw::NTuple{4,Float64}
    rh::NTuple{4,Float64}
    shock::NTuple{4,Float64}
    shock_max::NTuple{4,Float64}
    camber::NTuple{4,Float64}
    cross::Float64
    toe_front::Float64
    z::NTuple{4,Float64}
    roll_deg::Float64
    ok::Bool
end

armhole(a::AbstractString) = (s = lowercase(strip(a)); s == "soft" ? 1 : s == "firm" ? 7 : parse(Int, s))
const ARB_REF = ((0.375, 2), (0.6875, 5))

"""    arb_rate(P, axle, dia, arm) -> N/mm at the wheel

A torsion bar: rate ∝ diameter⁴ / lever², 1/lever growing by `arm_beta` (relative) per hole toward firm."""
function arb_rate(P::GarageParams, axle::Int, dia::Float64, arm::AbstractString)
    dia <= 0 && return 0.0
    dref, href = ARB_REF[axle]
    h = armhole(arm)

    P.karb[axle] * (dia/dref)^4 * max(0.05, 1 + P.arm_beta[axle]*(h - href))^2
end

axle(i) = i <= 2 ? 1 : 2
side(i) = isodd(i) ? 1.0 : -1.0

function corner_force(P::GarageParams, K::Knobs, i::Int, z::Float64)
    a = axle(i)

    s = P.s0[a] + P.MR[a]*z + 0.5*P.MRq[a]*(P.MR[a]*z)^2
    mr = P.MR[a] + P.MRq[a]*P.MR[a]^2*z

    x = (P.s0[a] - K.perch[i] + P.L[a]) + P.MRk[a]*z
    Fs = K.spring[i] * max(x, 0.0)
    smax = (a == 1 ? 104.6 : 113.0) - K.packer[i]
    e = s - (smax - P.bump_gap[a])
    Fb = e > 0 ? P.kbump[a]*e^2/(2*P.bump_gap[a]) : 0.0
    (; W = P.unsprung[a] + P.MRk[a]*Fs + mr*Fb, s, smax)
end

tyre_rate(P::GarageParams, a::Int, p::Float64) = P.kt[a] * (1 + P.kt_p*(p - 152.0))

"""    solve_static(K, P; warp = 0) -> Derived

Place the car on its springs: find the chassis plane (heave, pitch, roll) and the four wheel loads such that every
corner's spring + bar + bump force carries its load through its tyre, and the loads balance the weight and its CG.
`warp` [mm] lifts the LF/RR ground and drops RF/LR (a garage floor that is not flat; the skidpad's is)."""
function solve_static(K::Knobs, P::GarageParams = PARAMS[]; warp::Float64 = 0.0)
    Wt = P.W0 + K.fuel*P.fuel_kgL*G
    Wf = P.W0*P.front0 + K.fuel*P.fuel_kgL*G*P.fuel_front
    Wr = Wt - Wf
    kbar = (arb_rate(P, 1, K.farb_dia, K.farb_arm), arb_rate(P, 2, K.rarb_dia, K.rarb_arm))
    tr = (1.0, P.trackratio)
    gnd = (warp/2, -warp/2, -warp/2, warp/2)

    function resid(v)
        hf, hr, ro = v[1], v[2], v[3]
        z = (v[4], v[5], v[6], v[7])
        r = zeros(eltype(v), 7)
        W = ntuple(4) do i
            a = axle(i)
            F = corner_force(P, K, i, z[i]).W
            o = isodd(i) ? i+1 : i-1
            F + kbar[a]*(z[i] - z[o])
        end
        for i in 1:4
            a = axle(i)
            rh = (a == 1 ? hf : hr) + side(i)*ro*tr[a]/2 - gnd[i]
            dγ = K.camber[i] - P.cam_ref[a]
            r[i] = rh - (P.R0[a] + P.rp[a]*(K.pressure[i] - 152.0) + P.cam_jack*dγ + P.cam_jack2*dγ^2 -
                         z[i] - W[i]/tyre_rate(P, a, K.pressure[i]))
        end
        r[5] = (W[1] + W[2]) - Wf
        r[6] = (W[3] + W[4]) - Wr

        rdeg = ro/1500*180/pi
        cam = ntuple(i -> K.camber[i] - P.cam_gain[axle(i)]*z[i] + side(i)*P.cam_roll[axle(i)]*rdeg, 4)
        r[7] = sum(W[i]*side(i)*(P.track_f*tr[axle(i)]/2 + P.scrub[axle(i)]*z[i] +
                                 P.cam_patch*(cam[i] - P.cam_ref[axle(i)])) for i in 1:4)
        r
    end
    v = [P.R0[1] - 20.0, P.R0[2] - 20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ok = false
    for it in 1:60
        r = resid(v)
        J = zeros(7, 7); h = 1e-6
        for j in 1:7
            vp = copy(v); vp[j] += h
            J[:, j] = (resid(vp) .- r) ./ h
        end
        dv = try J \ r catch; fill(NaN, 7) end
        all(isfinite, dv) || break
        v .-= dv
        if maximum(abs, dv) < 1e-9
            ok = true; break
        end
    end
    hf, hr, ro = v[1], v[2], v[3]
    z = (v[4], v[5], v[6], v[7])
    cf = ntuple(i -> corner_force(P, K, i, z[i]), 4)
    W = ntuple(4) do i
        o = isodd(i) ? i+1 : i-1
        cf[i].W + kbar[axle(i)]*(z[i] - z[o])
    end

    hfm = (1 - P.rh_mix[1])*hf + P.rh_mix[1]*hr
    hrm = (1 - P.rh_mix[2])*hr + P.rh_mix[2]*hf
    rh = ntuple(i -> (axle(i) == 1 ? hfm : hrm) + side(i)*ro*(axle(i) == 1 ? 1.0 : P.trackratio)/2 - gnd[i], 4)
    rolldeg = ro/1500*180/pi

    cam = ntuple(i -> K.camber[i] - P.cam_gain[axle(i)]*z[i] + side(i)*P.cam_roll[axle(i)]*rolldeg, 4)
    toe = K.toe_front + P.toe_gain*(z[1] + z[2])
    Wshown = ntuple(4) do i
        o = isodd(i) ? i+1 : i-1
        W[i] - (1 - P.bar_shown)*kbar[axle(i)]*(z[i] - z[o])
    end
    Derived(Wshown, rh, ntuple(i -> cf[i].s, 4), ntuple(i -> cf[i].smax, 4), cam,
            100*(Wshown[2] + Wshown[3])/sum(Wshown), toe, z, rolldeg, ok)
end

"""Gear top speeds [km/h] as the garage shows them (rev limit x rolling radius / overall ratio)."""
gear_speeds_kmh(K::Knobs, P::GarageParams = PARAMS[]) =
    [P.rpm_r * 2π / 60 / (g*K.final_drive) * 3.6 for g in K.gears]

const GARAGE_TOML = joinpath(@__DIR__, "garage_lotus49.toml")

"""GarageParams from the shared constants file (the fit's output; see the header of garage_lotus49.toml)."""
function params_from_toml(fn::AbstractString = GARAGE_TOML)
    d = TOML.parsefile(fn)["params"]
    kw = Dict{Symbol,Any}()
    for f in fieldnames(GarageParams)
        haskey(d, String(f)) || continue
        v = d[String(f)]
        kw[f] = v isa AbstractVector ? Tuple(Float64.(v)) : Float64(v)
    end
    GarageParams(; kw...)
end
const PARAMS = Ref(params_from_toml())

const KNOB_SPECS = TOML.parsefile(GARAGE_TOML)["knob"]

const JULIA_SETUP = get(TOML.parsefile(GARAGE_TOML), "julia_setup", Dict{String,Any}())
"""    julia_setup(K, key) -> K′

The session's knobs `K` with julia's own changes for setup `key` (the TOML's [julia_setup.<key>]); `K` is not changed."""
function julia_setup(K, key::AbstractString)
    K = deepcopy(K)
    for (f, v) in get(JULIA_SETUP, key, Dict{String,Any}())
        T = fieldtype(typeof(K), Symbol(f))
        setfield!(K, Symbol(f), T <: Tuple ? T(Tuple(v)) : T <: AbstractString ? String(v) : T(v))
    end
    K
end

"""Knobs from a parsed .ibt CarSetup (Setup.setup_params), so a session's setup is the garage's starting point."""
function knobs_from_setup(sp)
    c4(d) = (d[:LF], d[:RF], d[:LR], d[:RR])
    dia(s) = (m = match(r"(\d*\.\d+|\d+)", s); occursin("isc", s) || m === nothing ? 0.0 : parse(Float64, m.captures[1]))
    fa = split(sp.arb_front); ra = split(sp.arb_rear)
    Knobs(; perch = haskey(sp, :perch_mm) ? c4(sp.perch_mm) : Knobs().perch,
          spring = Float64.(c4(sp.spring_rate_Npmm)), packer = Float64.(c4(sp.packer_mm)),
          camber = Float64.(c4(sp.camber_deg)), pressure = Float64.(c4(sp.cold_pressure_kPa)),
          bump = Int.(round.(c4(sp.bump_clicks))), rebound = Int.(round.(c4(sp.rebound_clicks))),
          toe_front = sp.toe_front_mm, toe_rear = sp.toe_rear_mm,
          farb_dia = dia(sp.arb_front), farb_arm = String(fa[end]), rarb_dia = dia(sp.arb_rear), rarb_arm = String(ra[end]),
          fuel = sp.fuel_L, brake_bias = sp.brake_bias_pct, steering_ratio = sp.steering_ratio,
          final_drive = sp.final_drive, gears = collect(Float64, sp.gear_ratios), diff_plates = Int(sp.diff_plates),
          diff_preload = sp.diff_preload_Nm, diff_drive = sp.diff_drive_ramp, diff_coast = sp.diff_coast_ramp)
end

"""    calibrate_session(sp) -> Knobs

The session's knobs, with the camber and front-toe knobs set so the model DISPLAYS the session's own static camber and
toe (the garage shows the result of a knob, not the knob)."""
function calibrate_session(sp, P::GarageParams = PARAMS[])
    K = knobs_from_setup(sp)
    want_c = K.camber; want_t = K.toe_front
    for _ in 1:3
        D = solve_static(K, P)
        K.camber = ntuple(i -> K.camber[i] + (want_c[i] - D.camber[i]), 4)
        K.toe_front += want_t - D.toe_front
    end
    K
end

"""    apply_garage(sp, K; Ks = calibrate_session(sp)) -> sp′

The session setup `sp` (Setup.setup_params) with the garage knobs `K` applied. Knob values are taken as set; the
DERIVED numbers are the session's own (reference's, exact) plus the model's change from the session's knobs `Ks` to `K`
-- so an untouched garage is the session to the digit, and the model's residual error cancels near it."""
function apply_garage(sp, K::Knobs; Ks::Knobs = calibrate_session(sp), P::GarageParams = PARAMS[])
    D = solve_static(K, P); Ds = solve_static(Ks, P)
    c4(d) = (d[:LF], d[:RF], d[:LR], d[:RR])
    d4(t) = Dict(:LF => t[1], :RF => t[2], :LR => t[3], :RR => t[4])
    cw = c4(sp.corner_weight_N) .+ (D.cw .- Ds.cw)
    rh = c4(sp.ride_height_mm) .+ (D.rh .- Ds.rh)
    sh = ntuple(i -> (c4(sp.shock_defl_mm)[i][1] + D.shock[i] - Ds.shock[i], D.shock_max[i]), 4)
    ca = c4(sp.camber_deg) .+ (D.camber .- Ds.camber)
    armstr(dia, arm) = (dia <= 0 ? "Disconnected" : string(dia) * "\"") * " " * arm
    gears = collect(Float64, K.gears)
    merge(sp, (; corner_weight_N = d4(cw), ride_height_mm = d4(rh), shock_defl_mm = d4(sh), camber_deg = d4(ca),
               cross_weight_pct = 100*(cw[2] + cw[3])/sum(cw),
               spring_rate_Npmm = d4(K.spring), perch_mm = d4(K.perch), packer_mm = d4(K.packer),
               cold_pressure_kPa = d4(K.pressure), bump_clicks = d4(Float64.(K.bump)), rebound_clicks = d4(Float64.(K.rebound)),
               toe_front_mm = sp.toe_front_mm + (D.toe_front - Ds.toe_front),
               toe_rear_mm = K.toe_rear,
               arb_front = armstr(K.farb_dia, K.farb_arm), arb_rear = armstr(K.rarb_dia, K.rarb_arm),
               fuel_L = K.fuel, brake_bias_pct = K.brake_bias, steering_ratio = K.steering_ratio,
               final_drive = K.final_drive, gear_ratios = gears,
               diff_plates = Float64(K.diff_plates), diff_preload_Nm = K.diff_preload,
               diff_drive_ramp = K.diff_drive, diff_coast_ramp = K.diff_coast))
end

const KNOB_FIELDS = (:perch, :spring, :packer, :camber, :pressure, :bump, :rebound, :toe_front, :toe_rear, :farb_dia,
                     :farb_arm, :rarb_dia, :rarb_arm, :fuel, :brake_bias, :steering_ratio, :final_drive, :gears,
                     :diff_plates, :diff_preload, :diff_drive, :diff_coast)

function write_garage_file(fn::AbstractString, K::Knobs; base::AbstractString = "default")
    d = Dict{String,Any}("base" => base)
    k = Dict{String,Any}()
    for f in KNOB_FIELDS
        v = getfield(K, f)
        k[String(f)] = v isa Tuple ? collect(v) : v
    end
    d["knobs"] = k
    open(io -> TOML.print(io, d), fn, "w")
    fn
end

"""Names of the knobs that differ between two garages (what the sim reports as "changed from the session")."""
function knob_diff(K::Knobs, Kref::Knobs)
    same(a, b) = a isa AbstractString || b isa AbstractString ? string(a) == string(b) :
                 length(a) == length(b) && all(isapprox.(collect(a), collect(b); atol = 1e-6))
    [String(f) for f in KNOB_FIELDS if !same(getfield(K, f), getfield(Kref, f))]
end

"""    read_garage_file(fn, Kbase) -> (base, Knobs)

Knobs from a garage file; any knob the file does not name keeps its value in `Kbase` (the session's)."""
function read_garage_file(fn::AbstractString, Kbase::Knobs)
    d = TOML.parsefile(fn)
    K = deepcopy(Kbase)
    k = get(d, "knobs", Dict{String,Any}())
    for f in KNOB_FIELDS
        haskey(k, String(f)) || continue
        v = k[String(f)]; T = fieldtype(Knobs, f)
        setfield!(K, f, T <: Tuple ? T(Tuple(v)) : T <: AbstractString ? String(v) : T <: AbstractVector ? Float64.(v) : T(v))
    end
    (get(d, "base", "default"), K)
end

"""One line of a setup's derived numbers (a Setup.setup_params NamedTuple, e.g. apply_garage's result)."""
function describe_derived_pp(sp)
    c4(d) = (d[:LF], d[:RF], d[:LR], d[:RR])
    f1(x) = string(round(x, digits = 1))
    "cw " * join((string(round(Int, w)) for w in c4(sp.corner_weight_N)), "/") * " N  rh " * join(f1.(c4(sp.ride_height_mm)), "/") *
    " mm  springs " * join((string(round(Int, k)) for k in c4(sp.spring_rate_Npmm)), "/") * " N/mm  cross " * f1(sp.cross_weight_pct) *
    "%  fuel " * f1(sp.fuel_L) * " L  final " * string(sp.final_drive)
end

function describe_derived(D::Derived)
    f1(x) = string(round(x, digits = 1))
    "cw " * join((string(round(Int, w)) for w in D.cw), "/") * " N  rh " * join(f1.(D.rh), "/") *
    " mm  shock " * join(f1.(D.shock), "/") * " mm  camber " * join(f1.(D.camber), "/") *
    "  cross " * f1(D.cross) * "%  toeF " * string(round(Int, D.toe_front))
end

"""A ratio as reference prints it: two decimals at 1 and above ("1.40"), three below ("0.846")."""
ratio_str(r) = r >= 1 ? @sprintf("%.2f", r) : @sprintf("%.3f", r)

"""    carsetup_yaml(yaml, sp; maxlen) -> String

The session YAML with its CarSetup block rewritten from a setup NamedTuple (apply_garage's), in reference's own value
formats, so an exported lap says which car drove it. Lines the garage does not own are left as they are -- except that a
result longer than `maxlen` (the .ibt's reserved space) drops the template's tyre-wear lines (LastHotPressure,
LastTemps*, TreadRemaining: the template session's tyres, not julia's) until it fits."""
function carsetup_yaml(yaml::AbstractString, sp; maxlen::Int = typemax(Int))
    corner = Dict("LeftFront" => :LF, "RightFront" => :RF, "LeftRear" => :LR, "RightRear" => :RR)
    f1(x) = string(round(x, digits = 1)); f0(x) = string(round(Int, x)); sg(x) = (x >= 0 ? "+" : "") * f0(x)
    f1s(x) = (r = round(x, digits = 1); (r == 0 && signbit(r) ? "-" : r >= 0 ? "+" : "") * string(abs(r) == 0 ? 0.0 : r))
    function val(sec, sub, key)
        c = get(corner, sub, nothing)
        if sec == "Chassis" && c !== nothing
            key == "CornerWeight"      && return f0(sp.corner_weight_N[c]) * " N"
            key == "RideHeight"        && return f1(sp.ride_height_mm[c]) * " mm"
            key == "ShockDeflection"   && return f1(sp.shock_defl_mm[c][1]) * " mm " * f1(sp.shock_defl_mm[c][2]) * " mm"
            key == "SpringPerchOffset" && return f1(sp.perch_mm[c]) * " mm"
            key == "SpringRate"        && return f0(sp.spring_rate_Npmm[c]) * " N/mm"
            key == "Packer"            && return f1(sp.packer_mm[c]) * " mm"
            key == "BumpStiffness"     && return sg(sp.bump_clicks[c]) * " clicks"
            key == "ReboundStiffness"  && return sg(sp.rebound_clicks[c]) * " clicks"
            key == "Camber"            && return f1s(sp.camber_deg[c]) * " deg"
        elseif sec == "Tires" && c !== nothing && key == "ColdPressure"
            return f0(sp.cold_pressure_kPa[c]) * " kPa"
        elseif sec == "Chassis" && sub == "Front"
            key == "ArbArms"     && return String(last(split(sp.arb_front)))
            key == "ArbDiameter" && return String(first(split(sp.arb_front)))
            key == "BrakeBias"   && return @sprintf("%.2f%%", sp.brake_bias_pct)
            key == "ToeIn"       && return sg(sp.toe_front_mm) * " mm"
            key == "SteeringRatio" && return @sprintf("%g:1", sp.steering_ratio)
            key == "CrossWeight" && return f1(sp.cross_weight_pct) * "%"
        elseif sec == "Chassis" && sub == "Rear"
            key == "FuelLevel"   && return f1(sp.fuel_L) * " L"
            key == "ArbArms"     && return String(last(split(sp.arb_rear)))
            key == "ArbDiameter" && return String(first(split(sp.arb_rear)))
            key == "ToeIn"       && return sg(sp.toe_rear_mm) * " mm"
        elseif sec == "Drivetrain" && sub == "Transmission"
            key == "FinalDrive" && return ratio_str(sp.final_drive)
            gi = findfirst(==(key), ("FirstGear", "SecondGear", "ThirdGear", "FourthGear", "FifthGear"))
            if gi !== nothing && gi <= length(sp.gear_ratios)
                K = Knobs(; final_drive = sp.final_drive, gears = collect(Float64, sp.gear_ratios))
                return ratio_str(sp.gear_ratios[gi]) * ", " * f1(gear_speeds_kmh(K)[gi]) * " Km/h"
            end
        elseif sec == "Drivetrain" && sub == "Differential"
            key == "ClutchPlates"   && return f0(sp.diff_plates)
            key == "Preload"        && return f0(sp.diff_preload_Nm) * " Nm"
            key == "DriveRampAngle" && return f0(sp.diff_drive_ramp) * " deg"
            key == "CoastRampAngle" && return f0(sp.diff_coast_ramp) * " deg"
        end
        nothing
    end
    lines = split(yaml, '\n'; keepempty = true)
    out = String[]; inblk = false; path = String[]
    for l in lines
        if startswith(l, "CarSetup:")
            inblk = true; push!(out, l); continue
        end
        ind = length(l) - length(lstrip(l, ' '))
        if inblk && !isempty(strip(l)) && ind == 0
            inblk = false
        end
        if inblk && !isempty(strip(l))
            k, v = split(strip(l), ':'; limit = 2)
            depth = ind
            resize!(path, max(depth - 1, 0)); push!(path, String(k))
            if !isempty(strip(v)) && length(path) == 3
                nv = val(path[1], path[2], path[3])
                nv === nothing || (push!(out, l[1:ind] * k * ": " * nv); continue)
            end
        end
        push!(out, String(l))
    end
    y = join(out, '\n')
    if length(y) > maxlen
        wear = r"^\s+(LastHotPressure|LastTemps\w*|TreadRemaining):"
        out = [l for l in out if !occursin(wear, l)]
        y = join(out, '\n')
    end
    y
end

end
