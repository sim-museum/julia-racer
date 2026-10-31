

const CAR_M = 617.0; const CAR_IZZ = 890.0; const CAR_E = 0.12; const CAR_MU = 0.4
const CAR_IYY = 850.0; const CAR_IXX = 120.0; const CAR_HCG = 0.30

const NOSE_H = 0.18; const BODY_H = 0.30

const TREAD_MU = parse(Float64, get(ENV, "JM_TREAD_MU", "0.7"))
const CONTACT_VERT = get(ENV, "JM_CONTACT_VERT", "1") != "0"
const TYRE_MU = parse(Float64, get(ENV, "JM_TYRE_CONTACT_MU", "1.0"))
const TYRE_E = parse(Float64, get(ENV, "JM_TYRE_CONTACT_E", "0.5"))
const CARCONTACT_OLD = get(ENV, "JM_CARCONTACT", "") == "old"

const CAR_PARTS = (
    (0.105, 0.0, 1.755, 0.35, false),
    (1.314, 0.75, 0.300, 0.110, true), (1.314, -0.75, 0.300, 0.110, true),
    (-1.096, 0.75, 0.334, 0.150, true), (-1.096, -0.75, 0.334, 0.150, true))

const CAR_BOUND_D = 3.8 + 0.8
const CONTACT_DT = 1/60
car_near(dx, dz, d, θa, θb) = CARCONTACT_OLD ? d <= contact_d(dx, dz, d, θa, θb) : d <= CAR_BOUND_D

"Corners of a part of the car at (x, z) heading θ, world frame, grown by `m` on every side."
function _part_corners(x, z, θ, p, m = 0.0)
    c = cos(θ); s = sin(θ)
    px = x + p[1]*c - p[2]*s; pz = z + p[1]*s + p[2]*c
    ax = (c*(p[3] + m), s*(p[3] + m)); ay = (-s*(p[4] + m), c*(p[4] + m))
    ((px + ax[1] + ay[1], pz + ax[2] + ay[2]), (px + ax[1] - ay[1], pz + ax[2] - ay[2]),
     (px - ax[1] - ay[1], pz - ax[2] - ay[2]), (px - ax[1] + ay[1], pz - ax[2] + ay[2]))
end

"Separating-axis test of two oriented rectangles (corner tuples): nothing, or (depth, nx, nz) with n from P toward Q."
function _sat(P, Q)
    best = Inf; bn = (0.0, 0.0)
    for R in (P, Q), k in 1:2
        ex = R[k+1][1] - R[k][1]; ez = R[k+1][2] - R[k][2]; l = hypot(ex, ez); l < 1e-9 && continue
        nx = -ez/l; nz = ex/l
        pmin = Inf; pmax = -Inf; qmin = Inf; qmax = -Inf
        for v in P; d = v[1]*nx + v[2]*nz; pmin = min(pmin, d); pmax = max(pmax, d); end
        for v in Q; d = v[1]*nx + v[2]*nz; qmin = min(qmin, d); qmax = max(qmax, d); end
        o = min(pmax, qmax) - max(pmin, qmin)
        o <= 0.0 && return nothing
        if o < best
            best = o
            cp = (sum(v[1] for v in P) - sum(v[1] for v in Q))*nx + (sum(v[2] for v in P) - sum(v[2] for v in Q))*nz
            bn = cp > 0 ? (-nx, -nz) : (nx, nz)
        end
    end
    (best, bn[1], bn[2])
end

"""Contact between car A at (ax, az, θa) and car B (bx, bz, θb): nothing, or (px, pz, nx, nz, depth, tyre) -- the contact
point, the normal from A toward B, the overlap and whether a tyre is in the touching pair (the deepest pair)."""
function car_contact_geom(ax, az, θa, bx, bz, θb; margin = 0.0)
    best = nothing
    for pa in CAR_PARTS
        P = _part_corners(ax, az, θa, pa, margin/2)
        for pb in CAR_PARTS
            Q = _part_corners(bx, bz, θb, pb, margin/2)
            r = _sat(P, Q); r === nothing && continue
            (best === nothing || r[1] > best[5]) || continue

            sx = 0.0; sz = 0.0; n = 0
            for (S, T) in ((P, Q), (Q, P)), v in S
                _inrect(v, T) && (sx += v[1]; sz += v[2]; n += 1)
            end
            if n == 0
                cpx = (sum(v[1] for v in P) + sum(v[1] for v in Q))/8; cpz = (sum(v[2] for v in P) + sum(v[2] for v in Q))/8
            else
                cpx = sx/n; cpz = sz/n
            end
            best = (cpx, cpz, r[2], r[3], r[1], pa[5] || pb[5], pa, pb)
        end
    end
    best
end
function _inrect(v, T)
    pos = false; neg = false
    for k in 1:4
        a = T[k]; b = T[k % 4 + 1]
        c = (b[1] - a[1])*(v[2] - a[2]) - (b[2] - a[2])*(v[1] - a[1])
        c > 0 && (pos = true); c < 0 && (neg = true)
    end
    !(pos && neg)
end

"""Impulse on car A (at (px,pz) heading θ, world velocity (pvx,pvz), yaw rate r) from a contact with car B (centre
(qx,qz), velocity (avx,avz), heading θb, yaw rate rb). Returns (dvx, dvz, dr, jn) -- A's world velocity and yaw-rate
changes and the normal impulse [N·s], then (S3) its heave velocity, left-up roll rate and nose-up pitch rate changes
(Car3D's bump3d! dvy, dpp, dq) -- or nothing when the cars are apart or separating. B takes the opposite impulse at
the same point (car_contact_impulse with the cars swapped gives it). `b_fixed`: B cannot answer within this frame (a
REMOTE car, moved by the other machine after the network latency) -- A then takes the whole stop, as from a wall; with
equal masses A kept ~44 % of its closing speed for the latency's frames and ran 0.5 m into the other car
(netcollide_smoke, S2)."""
function car_contact_impulse(px, pz, θ, pvx, pvz, r, qx, qz, avx, avz; θb = θ, rb = 0.0, b_fixed = false)
    CARCONTACT_OLD && return _car_contact_impulse_old(px, pz, θ, pvx, pvz, r, qx, qz, avx, avz)

    margin = clamp(hypot(pvx - avx, pvz - avz)*CONTACT_DT, 0.02, 0.8)
    g = car_contact_geom(px, pz, θ, qx, qz, θb; margin)
    g === nothing && return nothing
    (cx, cz, nx, nz, ov, tyre, pa, pb) = g
    depth = ov - margin
    rx = cx - px; rz = cz - pz; sx = cx - qx; sz = cz - qz

    vcx = (pvx - r*rz) - (avx - rb*sz); vcz = (pvz + r*rx) - (avz + rb*sx)
    vn = vcx*nx + vcz*nz
    ra = rx*nz - rz*nx; rbn = sx*nz - sz*nx
    meff = b_fixed ? 1.0/(1.0/CAR_M + ra^2/CAR_IZZ) : 1.0/(2.0/CAR_M + ra^2/CAR_IZZ + rbn^2/CAR_IZZ)
    if depth > 0.02

        vsep = min(10.0*depth, 3.0)
        vn + vsep <= 0.0 && return nothing
        jn = (vn + vsep)*meff
    else
        vgap = max(-depth, 0.0)/CONTACT_DT
        vn - vgap <= 0.2 && return nothing
        jn = (1 + (tyre ? TYRE_E : CAR_E))*(vn - vgap)*meff
    end
    tx = -nz; tz = nx; vt = vcx*tx + vcz*tz
    rat = rx*tz - rz*tx; rbt = sx*tz - sz*tx
    mt = b_fixed ? 1.0/(1.0/CAR_M + rat^2/CAR_IZZ) : 1.0/(2.0/CAR_M + rat^2/CAR_IZZ + rbt^2/CAR_IZZ)
    μ = tyre ? TYRE_MU : CAR_MU
    jt = -sign(vt)*min(μ*jn, abs(vt)*mt)
    jx = -jn*nx + jt*tx; jz = -jn*nz + jt*tz
    CONTACT_VERT || return (jx/CAR_M, jz/CAR_M, (rx*jz - rz*jx)/CAR_IZZ, jn, 0.0, 0.0, 0.0)

    c = cos(θ); s_ = sin(θ)
    xl = rx*c + rz*s_; yl = -rx*s_ + rz*c
    jxb = jx*c + jz*s_; jyb = -jx*s_ + jz*c
    zc = (pa[5] ? (pa[1] > 0 ? 0.30 : 0.334) : xl > 1.0 ? NOSE_H : BODY_H) - CAR_HCG
    jv = 0.0
    if pa[5] && pb[5] && TREAD_MU > 0
        ua = pvx*c + pvz*s_
        cb = cos(θb); sb = sin(θb); ub = avx*cb + avz*sb
        an = nx*c + nz*s_; bn = -(nx*cb + nz*sb)
        if abs(an) > 0.7 && abs(bn) > 0.7
            wa = (an > 0 ? -1.0 : 1.0)*ua
            wb = (bn > 0 ? -1.0 : 1.0)*ub
            slip = wa - wb
            mv = 1.0/(2.0/CAR_M + xl^2/CAR_IYY + (pb[1])^2/CAR_IYY)
            jv = -sign(slip)*min(TREAD_MU*jn, abs(slip)*mv)
        end
    end
    dq  = (xl*jv - zc*jxb)/CAR_IYY
    dpp = (yl*jv - zc*jyb)/CAR_IXX
    (jx/CAR_M, jz/CAR_M, (rx*jz - rz*jx)/CAR_IZZ, jn, jv/CAR_M, dpp, dq)
end

function _car_contact_impulse_old(px, pz, θ, pvx, pvz, r, qx, qz, avx, avz)
    dx = qx - px; dz = qz - pz; d = hypot(dx, dz); d < 1e-6 && return nothing
    nx = dx/d; nz = dz/d
    c = cos(θ); s_ = sin(θ)
    lx = clamp(dx*c + dz*s_, -2.0, 2.0); ly = clamp(-dx*s_ + dz*c, -0.9, 0.9)
    rx = lx*c - ly*s_; rz = lx*s_ + ly*c
    vcx = pvx - r*rz - avx; vcz = pvz + r*rx - avz
    vn = vcx*nx + vcz*nz
    vn <= 0.2 && return nothing
    rxn = rx*nz - rz*nx
    meff = 1.0/(2.0/CAR_M + rxn^2/CAR_IZZ)
    jn = (1 + CAR_E)*vn*meff
    tx = -nz; tz = nx; vt = vcx*tx + vcz*tz
    rxt = rx*tz - rz*tx
    mt = 1.0/(2.0/CAR_M + rxt^2/CAR_IZZ)
    jt = -sign(vt)*min(CAR_MU*jn, abs(vt)*mt)
    jx = -jn*nx + jt*tx; jz = -jn*nz + jt*tz
    (jx/CAR_M, jz/CAR_M, (rx*jz - rz*jx)/CAR_IZZ, jn, 0.0, 0.0, 0.0)
end
