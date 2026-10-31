

module SuspPose
export rsfix, RS_ROLL_DEG

const RS_ROLL_DEG = parse(Float32, get(ENV, "JM_RS_ROLL", "0"))

"4x4 corrective matrix for one rear half. `side` = +1 (z>0 half) / -1. Uses `T`(translate),
`RX`(rotx), `S`(scalexyz) supplied by the caller (Render's helpers), so this file needs no GL."
function rsfix(side, T, RX, S; roll = RS_ROLL_DEG)
    ax, ay = -1.05f0, 0.31f0
    sx = parse(Float32, get(ENV,"JM_RS_SX","1.0")); sy = parse(Float32, get(ENV,"JM_RS_SY","1.0")); sz = parse(Float32, get(ENV,"JM_RS_SZ","1.0"))
    dx, dy = parse(Float32, get(ENV,"JM_RS_DX","0.0")), parse(Float32, get(ENV,"JM_RS_DY","0.0"))

    dz = parse(Float32, get(ENV,"JM_RS_DZ","0.0"))
    y0, z0 = parse(Float32, get(ENV,"JM_RS_Y0","0.02")), parse(Float32, get(ENV,"JM_RS_Z0","0.772"))
    r = deg2rad(Float32(roll))
    T(Float32[dx, dy, side*dz]) * T(Float32[ax, ay, 0]) * S(sx, sy, sz) * T(Float32[-ax, -ay, 0]) *
        T(Float32[0, y0, side*z0]) * RX(Float32(-side*r)) * T(Float32[0, -y0, -side*z0])
end
end
