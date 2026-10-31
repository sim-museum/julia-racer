

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

const CDA_IBT = 0.480
const CRR_IBT = 0.0139

const PHYSFID_S9 = get(ENV, "JM_PHYSFID_S9", "1") != "0"
const AERO_CLAF = PHYSFID_S9 ? -0.06 : -0.12
const AERO_CLAR = PHYSFID_S9 ? -0.04 : 0.0
const AERO_CLAF_RH = 0.0

const LSD_K    = 0.20
const LSD_WEPS = 0.1
const EFRIC_T0 = 14.24
const EFRIC_K  = 0.00389
const EFRIC_R0 = 2353.0
const EFRIC_W  = 351.0

const ENGINE_LUMPED = haskey(ENV, "JM_ENGINE_LUMPED")
const IE_ENGINE = 0.0522
const IE_DRIVELINE = 0.1482

const DRIVELINE_INPUT = !haskey(ENV, "JM_DRIVELINE_AXLE")
const EFREE_T0 = 4.84
const EFREE_K  = 0.00276
"The engine's own zero-throttle drag [N·m] (S14, free revs)."
engine_drag_free(rpm) = (EFREE_T0 + EFREE_K*rpm) * 0.5*(1 + tanh((rpm - EFRIC_R0)/EFRIC_W))
"The gearbox's share of the in-gear coast drag [N·m at the input shaft]: EFRIC minus the engine's own, × (1 − f) like it."
gearbox_drag(rpm, throttle) = (1 - throttle_map(throttle, rpm)) *
    ((EFRIC_T0 - EFREE_T0) + (EFRIC_K - EFREE_K)*rpm) * 0.5*(1 + tanh((rpm - EFRIC_R0)/EFRIC_W))

const WOT_KNOTS_S14 = ((1875.0, 219.0), (2125.0, 231.0), (2375.0, 235.5), (3500.0, 235.5), (4400.0, 236.5), (4900.0, 255.0),
                       (5400.0, 273.5), (5900.0, 288.5), (6400.0, 302.5), (6850.0, 311.0), (7150.0, 312.0), (7650.0, 307.5),
                       (8100.0, 300.5), (8650.0, 284.5), (8900.0, 278.5))
const WOT_KNOTS_S6 = ((3500.0, 235.5), (4400.0, 236.5), (4900.0, 255.0), (5400.0, 273.5), (5900.0, 288.5),
                   (6400.0, 302.5), (6850.0, 311.0), (7150.0, 312.0), (7650.0, 307.5), (8100.0, 300.5),
                   (8650.0, 284.5), (8900.0, 278.5))
"Measured WOT crank torque: the knots joined by straight lines, as Σ ramps (symbolic-safe: max only)."
const WOT_KNOTS = ENGINE_LUMPED ? WOT_KNOTS_S6 : WOT_KNOTS_S14
function wot_torque(rpm)
    k = WOT_KNOTS
    T = k[1][2] + 0.0*rpm; s0 = 0.0
    for i in 1:length(k)-1
        s = (k[i+1][2] - k[i][2]) / (k[i+1][1] - k[i][1])
        T += (s - s0)*max(0.0, rpm - k[i][1]); s0 = s
    end
    T
end

const THROTTLE_KNOTS = ((0.0, 0.0), (0.1, 0.02), (0.2, 0.13), (0.3, 0.21), (0.4, 0.33), (0.5, 0.45), (0.6, 0.58),
                        (0.7, 0.72), (0.8, 0.84), (0.9, 0.95), (1.0, 1.0))
const THROTTLE_LINEAR = haskey(ENV, "JM_THROTTLE_LINEAR")

const THROTTLE_KNOTS_LO_S6 = ((0.0, 0.0), (0.1, 0.02), (0.2, 0.12), (0.3, 0.22), (0.4, 0.34), (0.5, 0.48), (0.6, 0.62),
                              (0.7, 0.77), (0.8, 0.90), (0.9, 0.96), (1.0, 1.0))
const THROTTLE_KNOTS_HI_S6 = ((0.0, 0.0), (0.1, 0.02), (0.2, 0.11), (0.3, 0.19), (0.4, 0.27), (0.5, 0.41), (0.6, 0.53),
                              (0.7, 0.70), (0.8, 0.87), (0.9, 0.95), (1.0, 1.0))

const THROTTLE_KNOTS_LOW = ((0.0, 0.0), (0.1, 0.02), (0.2, 0.13), (0.3, 0.235), (0.4, 0.363), (0.5, 0.514), (0.6, 0.63),
                            (0.7, 0.76), (0.8, 0.88), (0.9, 0.965), (1.0, 1.0))
const THROTTLE_KNOTS_LO = ((0.0, 0.0), (0.1, 0.01), (0.2, 0.11), (0.3, 0.212), (0.4, 0.294), (0.5, 0.449), (0.6, 0.570),
                           (0.7, 0.706), (0.8, 0.850), (0.9, 0.96), (1.0, 1.0))
const THROTTLE_KNOTS_HI = ((0.0, 0.0), (0.1, 0.005), (0.2, 0.055), (0.3, 0.110), (0.4, 0.235), (0.5, 0.361), (0.6, 0.530),
                           (0.7, 0.690), (0.8, 0.840), (0.9, 0.95), (1.0, 1.0))

const THROTTLE_KNOTS_LO_GF = ((0.0, 0.0), (0.1, 0.005), (0.2, 0.086), (0.3, 0.180), (0.4, 0.292), (0.5, 0.412), (0.6, 0.553),
                              (0.7, 0.699), (0.8, 0.871), (0.9, 0.96), (1.0, 1.0))
const THROTTLE_KNOTS_HI_GF = ((0.0, 0.0), (0.1, 0.005), (0.2, 0.077), (0.3, 0.157), (0.4, 0.221), (0.5, 0.349), (0.6, 0.493),
                              (0.7, 0.650), (0.8, 0.822), (0.9, 0.95), (1.0, 1.0))
const THROTTLE_1D = haskey(ENV, "JM_THROTTLE_1D")
"Pedal → torque fraction through a knot table (straight lines, as Σ ramps: symbolic-safe)."
function _knots(thr, k)
    f = 0.0*thr; s0 = 0.0
    for i in 1:length(k)-1
        s = (k[i+1][2] - k[i][2]) / (k[i+1][1] - k[i][1])
        f += (s - s0)*max(0.0, thr - k[i][1]); s0 = s
    end
    f
end
"Pedal (and rpm) → torque fraction: ENGINERHO-1 S3's three columns blended by rpm; without rpm, or JM_THROTTLE_1D, S8's map."
function throttle_map(thr, rpm = nothing)
    THROTTLE_LINEAR && return thr
    (rpm === nothing || THROTTLE_1D) && return _knots(thr, THROTTLE_KNOTS)
    wr = min(max((rpm - 4750.0)/2750.0, 0.0), 1.4)
    if get(ENV, "JM_THROTTLE_S6", "") == "1" || get(ENV, "JM_THROTTLE_GOLDFRAME", "") == "1"
        s6 = get(ENV, "JM_THROTTLE_S6", "") == "1"
        lo = _knots(thr, s6 ? THROTTLE_KNOTS_LO_S6 : THROTTLE_KNOTS_LO_GF)
        return lo + wr*(_knots(thr, s6 ? THROTTLE_KNOTS_HI_S6 : THROTTLE_KNOTS_HI_GF) - lo)
    end
    wl = min(max((4750.0 - rpm)/1250.0, 0.0), 1.8)
    lo = _knots(thr, THROTTLE_KNOTS_LO)
    lo + wr*(_knots(thr, THROTTLE_KNOTS_HI) - lo) + wl*(_knots(thr, THROTTLE_KNOTS_LOW) - lo)
end

const CLUTCH_KNOTS_S9 = ((0.0, 0.0), (0.05, 1.0), (0.15, 14.0), (0.25, 71.0), (0.35, 142.0), (0.45, 202.0), (0.55, 321.0),
                         (0.65, 407.0), (1.0, 760.0))

const CLUTCH_KNOTS_S14 = ((0.0, 0.0), (0.05, 0.5), (0.10, 2.0), (0.15, 16.7), (0.20, 47.7), (0.25, 77.4), (0.30, 117.6),
                          (0.35, 162.4), (0.40, 212.3), (0.45, 245.5), (0.50, 300.5), (0.55, 338.3), (0.60, 412.8), (1.0, 797.0))
const CLUTCH_KNOTS = ENGINE_LUMPED ? CLUTCH_KNOTS_S9 : CLUTCH_KNOTS_S14
const CLUTCH_LINEAR = haskey(ENV, "JM_CLUTCH_LINEAR")
"Clutch torque capacity [N·m] at engagement `e` (0 out … 1 in), through CLUTCH_KNOTS (Σ ramps: symbolic-safe)."
function clutch_capacity(e)
    CLUTCH_LINEAR && return 500.0*e
    k = CLUTCH_KNOTS
    T = 0.0*e; s0 = 0.0
    for i in 1:length(k)-1
        s = (k[i+1][2] - k[i][2]) / (k[i+1][1] - k[i][1])
        T += (s - s0)*max(0.0, e - k[i][1]); s0 = s
    end
    T
end

const RHO_WOT = 1.099
const ENGINE_RHO = get(ENV, "JM_ENGINERHO", "1") != "0"

const ENGINE_KR = Ref(1.0)
const ENGINE_KP = Ref(1.0)
function engine_torque(rpm, throttle; kw...)
    kR = ENGINE_KR[]; kP = ENGINE_KP[]
    (kR == 1.0 && kP == 1.0) ? _engine_torque(rpm, throttle; kw...) : (kP/kR)*_engine_torque(rpm/kR, throttle; kw...)
end
function engine_torque_free(rpm, throttle; kw...)
    kR = ENGINE_KR[]; kP = ENGINE_KP[]
    (kR == 1.0 && kP == 1.0) ? _engine_torque_free(rpm, throttle; kw...) : (kP/kR)*_engine_torque_free(rpm/kR, throttle; kw...)
end
function _engine_torque(rpm, throttle; redline = 9500.0, T0 = EFRIC_T0, k = EFRIC_K,
                       r0 = EFRIC_R0, w = EFRIC_W, dens = 1.0)
    wot = max(0.0, wot_torque(rpm))*dens
    cut = 0.5*(1 - tanh((rpm - redline)/200.0))
    fric = (T0 + k*rpm) * 0.5*(1 + tanh((rpm - r0)/w))
    f = throttle_map(throttle, rpm)
    f*wot*cut - (1 - f)*fric
end
"The ENGINE alone (IRGOLD-1 S14): as engine_torque, with the engine's own drag instead of the in-gear total."
function _engine_torque_free(rpm, throttle; redline = 9500.0, dens = 1.0)
    wot = max(0.0, wot_torque(rpm))*dens; cut = 0.5*(1 - tanh((rpm - redline)/200.0))
    f = throttle_map(throttle, rpm)
    f*wot*cut - (1 - f)*engine_drag_free(rpm)
end

function LongitudinalVehicle(; name,
        m = 617.0, Rw_f = 0.30, Rw_r = 0.334, Iw = 1.0, Ieng = 0.10, η = 0.9,
        gear = 1.72, final = 4.11, bias = 0.535, Tbrake_max = 3000.0,
        CdA = CDA_IBT, ρair = 1.10, Fzf = 2752.0, Fzr = 3300.0,
        throttle = 1.0, brake = 0.0,
        tyre_f = TYRE_SKIDPAD_FRONT, tyre_r = TYRE_SKIDPAD_REAR)
    ps = @parameters m=m Rw_f=Rw_f Rw_r=Rw_r Iw=Iw Ieng=Ieng η=η gear=gear final=final bias=bias Tbrake_max=Tbrake_max CdA=CdA ρair=ρair Fzf=Fzf Fzr=Fzr throttle=throttle brake=brake
    vars = @variables u(t)=15.0 ωf(t)=50.0 ωr(t)=45.0 rpm(t) κf(t) κr(t) Fxf(t) Fxr(t) ax(t)
    gr = gear*final
    eqs = [
        rpm ~ ωr*gr*60/(2π),
        κf  ~ (ωf*Rw_f - u)/(u + 0.5),
        κr  ~ (ωr*Rw_r - u)/(u + 0.5),
        Fxf ~ 2*tyre_fx(Fzf/2, κf; p = tyre_f),
        Fxr ~ 2*tyre_fx(Fzr/2, κr; p = tyre_r),

        2*Iw*D(ωf) ~ -brake*Tbrake_max*bias*tanh(ωf) - Fxf*Rw_f,
        (2*Iw + Ieng*gr^2)*D(ωr) ~ engine_torque(rpm, throttle)*gr*η
                                   - brake*Tbrake_max*(1 - bias)*tanh(ωr) - Fxr*Rw_r,

        ax ~ (Fxf + Fxr - 0.5*ρair*CdA*u^2)/m,
        D(u) ~ ax,
    ]
    System(eqs, t, vars, ps; name)
end

const FUEL_RPM   = (4000.0, 5000.0, 6000.0, 7000.0, 8000.0)

const FUEL_PEDAL = get(ENV, "JM_FUEL_KNOTS_OLD", "0") == "1" ? (0.0, 0.25, 0.5, 0.75, 1.0) : (0.0, 0.3, 0.55, 0.8, 1.0)
const FUEL_KGH   = ((1.7, 2.0, 2.2, 2.3, 2.6),
                    (8.9, 10.5, 12.4, 11.8, 10.6),
                    (22.0, 27.2, 30.8, 33.1, 32.0),
                    (38.6, 48.4, 56.3, 64.7, 65.4),
                    (44.8, 57.5, 67.5, 75.7, 77.9))
const FUEL_KG_PER_L = 0.75
const FUEL_MAX_L    = 179.5
function _interp1(xs, x)
    x <= xs[1] && return (1, 0.0); x >= xs[end] && return (length(xs) - 1, 1.0)
    i = findlast(k -> xs[k] <= x, 1:length(xs)); (i, (x - xs[i])/(xs[i+1] - xs[i]))
end

const FUEL_RHO_REF = 1.097
const FUEL_RHO = Ref(FUEL_RHO_REF)
"Fuel burn [kg/h] at pedal `thr` (0-1) and `rpm` (the gold's table, bilinear; x the air density over the Ring's)."
fuel_rate_kgh(thr, rpm) = _fuel_rate_ring(thr, rpm) * (get(ENV, "JM_FUEL_RHO", "1") == "0" ? 1.0 : FUEL_RHO[]/FUEL_RHO_REF)
function _fuel_rate_ring(thr, rpm)
    i, fi = _interp1(FUEL_PEDAL, clamp(thr, 0, 1)); j, fj = _interp1(FUEL_RPM, rpm)
    a = FUEL_KGH[i][j]*(1 - fj) + FUEL_KGH[i][j+1]*fj; b = FUEL_KGH[i+1][j]*(1 - fj) + FUEL_KGH[i+1][j+1]*fj
    a*(1 - fi) + b*fi
end
