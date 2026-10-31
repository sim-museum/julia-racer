

"Brush saturation 1−(1−ξ)³ — the adhesion→sliding force fraction (parabolic pressure)."
brush_sat(ξ) = ξ < 1.0 ? ξ*(3.0 - ξ*(3.0 - ξ)) : 1.0

"""IRFIT-261004 BRAKE-2: SLIDING friction below static. Once the whole patch slides (ξ > 1) the friction
falls from the peak toward `rs`·peak as the slide deepens: rs + (1 − rs)·exp(−((ξ − 1)₊/ws)²). Exactly 1
up to ξ = 1 and flat just past it (Gaussian onset), so the adhesion curve and the peak are unchanged;
a LOCKED wheel (κ = −1, ξ ≈ Cκ/3μ ≈ 5) gets rs. Without it the brush held its peak at any slip, so the
car braked at ~1.45 g and never locked, where the gold locks at full pedal and then slides at ~1.0 g."""
brush_slide(ξ, rs, ws) = rs + (1.0 - rs)*exp(-(max(ξ - 1.0, 0.0)/ws)^2)

"Load-sensitive friction coefficient μ(Fz).  The load factor is CLAMPED to [0.4,1.6]:
friction varies with load but can never reach 0 or go negative (which would make the
brush's μ-division blow up on a load transient) — physical AND numerically safe."
brush_mu(Fz, μ, kμ, Fz0) = μ * clamp(1.0 - kμ * (Fz/Fz0 - 1.0), 0.4, 1.6)

const _GRIP = parse(Float64, get(ENV, "JM_GRIP", "1.0"))

const C_ABL = 0.40

const BRUSH_FRONT = (μ = 1.277*_GRIP, μx = 1.438*_GRIP, Cα = 30.21, Cκ = 23.7, kμ = 0.082, Fz0 = 1415.0, rs = 0.629, ws = 1.5, rsy = 0.975)
const BRUSH_REAR  = (μ = 1.446*_GRIP, μx = 1.498*_GRIP, Cα = 34.0, Cκ = 23.2, kμ = 0.082, Fz0 = 1670.0, rs = 0.629, ws = 1.5, rsy = 0.975)

const CAMBER_CG = 1.0
const CAMBER_KG = 6.0
const CAMBER_RC = (1.0, 0.8)
const CAMBER_MU = (1.36, 1.446)

const TYRE2 = (μf = 1.347, μr = 1.494, Cαf = 26.17, Cαr = 27.77)

const IRG_S20 = get(ENV, "JM_IRG_S20", "1") != "0"

const PHYSFID_S2 = get(ENV, "JM_PHYSFID_S2", "1") != "0"

const US1 = get(ENV, "JM_US1", "1") != "0"
const TYRE3 = (μf = parse(Float64, get(ENV, "JM_TYRE3_MUF", "1.40")), μr = parse(Float64, get(ENV, "JM_TYRE3_MUR", US1 ? "1.52" : "1.65")), Cαf = parse(Float64, get(ENV, "JM_TYRE3_CAF", PHYSFID_S2 ? "26.0" : IRG_S20 ? "23.0" : "24.0")), Cαr = parse(Float64, get(ENV, "JM_TYRE3_CAR", "27.77")),
               nshf = 5.0, nshr = 5.0, kμ = 0.082, cabl = IRG_S20 ? 0.35 : 0.40, nsr = IRG_S20 ? 0.75 : 1.0)
const USE_TYRE3 = !haskey(ENV, "JM_TYRE2")

const COLUMN_KIN = !haskey(ENV, "JM_COLUMN_RELAXED")

const BRUSH_NS = 1.0

"Pure-lateral brush force Fy(Fz, α) — for fitting/validation."
function brush_fy(Fz, α; p = BRUSH_FRONT)
    μ = brush_mu(Fz, p.μ, p.kμ, p.Fz0)
    ξ = p.Cα*abs(sin(α)) / (3.0*μ)
    sign(α) * μ*Fz * brush_sat(ξ) * brush_slide(ξ, p.rs, p.ws)
end

"Pure-longitudinal brush force Fx(Fz, κ) — uses the LONGITUDINAL friction μx."
function brush_fx(Fz, κ; p = BRUSH_FRONT)
    μ = brush_mu(Fz, p.μx, p.kμ, p.Fz0)
    ξ = p.Cκ*abs(κ) / (3.0*μ)
    sign(κ) * μ*Fz * brush_sat(ξ) * brush_slide(ξ, p.rs, p.ws)
end

"""Combined brush force (Fx, Fy).  The deflection vector is (Cκ·κ, Cα·sinα); the
friction LIMIT is an ellipse (μx longitudinal, μy lateral), so the directional
friction is μ_dir = 1/√((cosψ/μx)² + (sinψ/μy)²) along the deflection direction ψ.
Magnitude = μ_dir·Fz·brush_sat(ξ).  Pure lateral ⇒ μy·Fz; pure longitudinal ⇒ μx·Fz;
the friction ellipse + the per-direction stiffness emerge from the physics."""
function brush_forces(Fz, α, κ; p = BRUSH_FRONT)
    μy = brush_mu(Fz, p.μ,  p.kμ, p.Fz0)
    μx = brush_mu(Fz, p.μx, p.kμ, p.Fz0)

    ξx = p.Cκ*κ / (3.0*μx);  ξy = p.Cα*sin(α) / (3.0*μy)
    ξ  = sqrt(ξx^2 + ξy^2 + 1e-9)
    s  = brush_sat(ξ) * brush_slide(ξ, p.rs, p.ws)
    (μx*Fz*s*ξx/ξ, μy*Fz*s*ξy/ξ)
end
