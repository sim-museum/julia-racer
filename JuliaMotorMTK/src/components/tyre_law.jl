

"Sine Magic-Formula branch: sin(C·atan(x − E·(x − atan x)))."
mf_branch(x, C, E) = sin(C * atan(x - E * (x - atan(x))))

"Load-dependent slip stiffness K(Fz) = p1·Fz0·sin(2·atan(Fz/(p2·Fz0))) — rises
then saturates with load, the standard MF shape (N/rad for α, N for κ)."
mf_stiffness(Fz, Fz0, p1, p2) = p1 * Fz0 * sin(2 * atan(Fz / (p2 * Fz0)))

"Default isothermal tyre parameters (placeholders — fitted to .ibt later)."
const TYRE_DEFAULTS = (
    Fz0 = 3000.0,
    μy  = 1.30,
    μx  = 1.30,
    Cy  = 1.40,
    Cx  = 1.60,
    Ey  = -0.5,
    Ex  = -0.5,
    pKy1 = 14.0,
    pKy2 = 1.71,
    pKx1 = 16.6,
    t0   = 0.035,
    Bt   = 8.0,
)

const TYRE_SKIDPAD_FRONT = (
    Fz0 = 1415.0, μy = 1.45, μx = 1.45, Cy = 1.40, Cx = 1.6,
    Ey = 0.40, Ex = -0.5, pKy1 = 25.7, pKy2 = 1.71, pKx1 = 16.6, t0 = 0.035, Bt = 8.0)
const TYRE_SKIDPAD_REAR = (
    Fz0 = 1670.0, μy = 1.55, μx = 1.55, Cy = 1.45, Cx = 1.6,
    Ey = 0.329, Ex = -0.5, pKy1 = 33.8, pKy2 = 1.71, pKx1 = 16.6, t0 = 0.035, Bt = 8.0)

"Pure-Julia lateral force Fy(Fz, α) — mirror of the symbolic `Tyre.Fy` eq."
function tyre_fy(Fz, α; p = TYRE_DEFAULTS)
    Ky = mf_stiffness(Fz, p.Fz0, p.pKy1, p.pKy2)
    By = Ky / (p.Cy * p.μy * Fz + 1e-6)
    p.μy * Fz * mf_branch(By * α, p.Cy, p.Ey)
end

"Pure-Julia longitudinal force Fx(Fz, κ)."
function tyre_fx(Fz, κ; p = TYRE_DEFAULTS)
    Kx = p.pKx1 * Fz
    Bx = Kx / (p.Cx * p.μx * Fz + 1e-6)
    p.μx * Fz * mf_branch(Bx * κ, p.Cx, p.Ex)
end

function tyre_forces(Fz, α, κ; p = TYRE_DEFAULTS)
    Ky = mf_stiffness(Fz, p.Fz0, p.pKy1, p.pKy2)
    By = Ky / (p.Cy * p.μy * Fz + 1e-6)
    sx = κ; sy = sin(α)
    Fmag = p.μy * Fz * mf_branch(By * sqrt(sx^2 + sy^2 + 1e-9), p.Cy, p.Ey)
    σ = sqrt(sx^2 + sy^2 + 0.02^2)
    (Fmag * sx / σ, Fmag * sy / σ)
end
