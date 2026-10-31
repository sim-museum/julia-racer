

const WHEEL_HITS = get(ENV, "JM_WHEEL_HITS", "1") != "0"
const WHEEL_M = 20.0
const WHEEL_E = 0.3
const WHEEL_R = 0.33
const CAR_HL = 1.95; const CAR_HW = 0.85

"""Car (x, z, heading θ, world velocity (vx, vz), mass mc) against wheel w (game frame). Returns nothing, or (the wheel
after the hit, the car's dv (x, z), how far under the car's middle it was struck 0..1, the closing speed)."""
function wheel_hit(x, z, θ, vx, vz, mc, w::LooseWheel)
    (wx, wy, wz, wvx, wvy, wvz, sp, sr, nm) = w
    dx = wx - x; dz = wz - z
    f = dx*cos(θ) + dz*sin(θ); l = -dx*sin(θ) + dz*cos(θ)
    (abs(f) < CAR_HL + WHEEL_R && abs(l) < CAR_HW + WHEEL_R) || return nothing
    d = max(hypot(dx, dz), 1e-3); nx = dx/d; nz = dz/d
    vc = (vx - wvx)*nx + (vz - wvz)*nz
    vc > 0.5 || return nothing
    J = (1 + WHEEL_E)*vc/(1/WHEEL_M + 1/mc)
    under = clamp(1 - max(abs(f)/CAR_HL, abs(l)/CAR_HW), 0.0, 1.0)
    w2 = (wx + 0.05nx, wy, wz + 0.05nz, wvx + J/WHEEL_M*nx, max(wvy, 0.0) + 0.25*J/WHEEL_M, wvz + J/WHEEL_M*nz, sp, sr + vc/WHEEL_R, nm)
    (w2, (-J/mc*nx, -J/mc*nz), under, vc)
end

"""TD261030-1 (6), one frame: every loose wheel against the player's car and each AI car."""
function wheel_hits_step!(cs, ai_poses, AICARS, classicai_field!, CLASSICAI_T)
    (WHEEL_HITS && (!isempty(LOOSE_WHEELS) || !isempty(AI_LOOSE_WHEELS))) || return 0.0
    jolt = 0.0
    mc = DriveRT3D.car_mass()
    for W in (LOOSE_WHEELS, AI_LOOSE_WHEELS), i in eachindex(W)

        h = wheel_hit(cs.x, cs.z, cs.θ, WVX[], WVZ[], mc, W[i])
        if h !== nothing
            W[i] = h[1]
            up = 1.2*h[3]*clamp(h[4]/6.0, 0.0, 1.0)
            player_bump!(cs, h[2][1], h[2][2], 0.0, up, 0.0, 0.0)
            jolt = clamp(0.5 + 0.05h[4], 0.0, 1.0)
            println("  [wheelhit] t=", round(cs.t, digits = 2), " your car hit a loose wheel (", W[i][9], ") at ", round(h[4]*3.6), " km/h",
                    h[3] > 0.3 ? ", ran over it" : ""); flush(stdout)
        end
        for k in eachindex(ai_poses)
            (k in AI_GONE || haskey(AI_WRECK, k) || haskey(LOOSE_OF, k) || k > length(AICARS)) && continue
            p = ai_poses[k]; v = AICARS[k].v
            h = wheel_hit(p[1], p[3], p[4], v*cos(p[4]), v*sin(p[4]), mc, W[i])
            h === nothing && continue
            W[i] = h[1]
            dv = hypot(h[2]...)*(1 + h[3])
            if CLASSICAI_T !== nothing
                gc = classicai_field!().cars[k]; gc.v = max(gc.v - dv, 0.0)
            end
            println("  [wheelhit] t=", round(cs.t, digits = 2), " AI ", k, " hit a loose wheel (", W[i][9], ") at ", round(h[4]*3.6), " km/h, -",
                    round(dv*3.6, digits = 1), " km/h"); flush(stdout)
        end
    end
    jolt
end

const WHEEL_DROP = (f = split(get(ENV, "JM_WHEEL_DROP", ""), ":"); length(f) == 2 && haskey(ENV, "JM_SMOKE") ? (parse(Float64, f[1]), parse(Float64, f[2])) : nothing)
const WHEEL_DROP_DONE = Ref(false)
function wheel_drop_step!(t, ai_poses, groundz)
    (WHEEL_DROP === nothing || WHEEL_DROP_DONE[] || t < WHEEL_DROP[1] || isempty(ai_poses)) && return nothing
    WHEEL_DROP_DONE[] = true
    p = ai_poses[1]; x = p[1] + WHEEL_DROP[2]*cos(p[4]); z = p[3] + WHEEL_DROP[2]*sin(p[4])
    h = groundz(x, z); y = (h > -900 ? Float64(h) : Float64(p[2])) + WHEEL_R
    nm = isempty(AICARMODELS) ? "wheel" : String((AICARMODELS[1]::Render.CLASSICCarModel).wheelspec[1][5])
    push!(AI_LOOSE_WHEELS, (x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0, nm)); push!(AI_LOOSE_OWNER, 1)
    println("  [wheeldrop] t=", round(t, digits = 2), " a loose wheel ", WHEEL_DROP[2], " m ahead of AI 1 at (", round(x, digits = 1), ", ",
            round(z, digits = 1), ")"); flush(stdout)
    nothing
end
