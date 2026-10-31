

"""
Drivable track surface built from a centreline ribbon, with a uniform-grid
spatial index over the segments for O(1) point queries.
"""
struct TrackSurface
    pos::Vector{NTuple{3,Float64}}
    perp::Vector{NTuple{3,Float64}}
    normal::Vector{NTuple{3,Float64}}
    halfwidth::Vector{NTuple{2,Float64}}
    lapdist::Vector{Float64}
    seg::Vector{Int}

    cell::Float64
    x0::Float64
    z0::Float64
    nx::Int
    nz::Int
    buckets::Vector{Vector{Int}}
    lap_length::Float64
end

Base.show(io::IO, t::TrackSurface) =
    print(io, "TrackSurface(", length(t.seg), " segments, lap ",
          round(t.lap_length; digits=1), " m, grid ", t.nx, "×", t.nz, ")")

unit(v) = (n = sqrt(v[1]^2 + v[2]^2 + v[3]^2); n > 0 ? v ./ n : v)

"""
    TrackSurface(pts::Vector{NTuple{3,Float64}}; halfwidth=8.0, cell=20.0)

Build the ribbon from a raw ordered centreline (closed loop) -- the Classic .trk.  `pts` are (x, y, z) with y the surface height.  perp = left
of the horizontal tangent, normal = up, lapdist = cumulative horizontal distance.
"""
function TrackSurface(pts::Vector{NTuple{3,Float64}}; halfwidth::Real=8.0, cell::Real=20.0)
    n = length(pts); n >= 2 || throw(ArgumentError("need ≥2 centreline points"))
    pos = pts
    perp = NTuple{3,Float64}[]; nrm = NTuple{3,Float64}[]
    hw = NTuple{2,Float64}[]; ld = Float64[]; d = 0.0
    for i in 1:n
        a = pos[i]; bn = pos[mod1(i+1, n)]
        tx = bn[1]-a[1]; tz = bn[3]-a[3]; tl = hypot(tx, tz); tl < 1e-6 && (tl = 1.0)
        push!(perp, (-tz/tl, 0.0, tx/tl)); push!(nrm, (0.0,1.0,0.0))
        push!(hw, (Float64(halfwidth), Float64(halfwidth))); push!(ld, d)
        d += hypot(bn[1]-a[1], bn[3]-a[3])
    end
    segs = collect(1:n)
    xs = [p[1] for p in pos]; zs = [p[3] for p in pos]
    x0, z0 = minimum(xs)-cell, minimum(zs)-cell
    nx = ceil(Int, (maximum(xs)+cell-x0)/cell); nz = ceil(Int, (maximum(zs)+cell-z0)/cell)
    buckets = [Int[] for _ in 1:nx*nz]
    cellof(x,z) = (clamp(floor(Int,(x-x0)/cell),0,nx-1), clamp(floor(Int,(z-z0)/cell),0,nz-1))
    for s in segs
        a = pos[s]; b = pos[mod1(s+1,n)]; (ax,az),(bx,bz) = cellof(a[1],a[3]), cellof(b[1],b[3])
        for cx in min(ax,bx):max(ax,bx), cz in min(az,bz):max(az,bz)
            push!(buckets[cz*nx + cx + 1], s)
        end
    end
    TrackSurface(pos, perp, nrm, hw, ld, segs, Float64(cell), x0, z0, nx, nz, buckets, d)
end

"""Result of a HAT query at a world (x, z)."""
struct HATResult
    height::Float64
    normal::NTuple{3,Float64}
    on_track::Bool
    lateral::Float64
    lapdist::Float64
    found::Bool
    perp::NTuple{2,Float64}
end

"""
    hat(ts, x, z) -> HATResult

Surface height, normal, on-track flag, lateral offset and lap distance at
world position (x, z).  Searches the query cell and its 8 neighbours.
"""

const HAT_CALLS = Ref(0)
const HAT_NS    = Ref(0)
const HAT_COUNT_ON = Ref(false)
const HAT_TIME_ON  = Ref(false)
function hat_stats()
    c = HAT_CALLS[]; ns = HAT_NS[]
    (calls = c, total_s = ns/1e9, per_call_us = c == 0 ? 0.0 : ns/1e3/c)
end
hat_reset!() = (HAT_CALLS[] = 0; HAT_NS[] = 0; nothing)

@inline _cr(p0, p1, p2, p3, t) =
    0.5 * (2p1 + (-p0 + p2)*t + (2p0 - 5p1 + 4p2 - p3)*t^2 + (-p0 + 3p1 - 3p2 + p3)*t^3)
@inline _crd(p0, p1, p2, p3, t) =
    0.5 * ((-p0 + p2) + 2t*(2p0 - 5p1 + 4p2 - p3) + 3t*t*(-p0 + 3p1 - 3p2 + p3))
@inline _crdd(p0, p1, p2, p3, t) =
    0.5 * (2*(2p0 - 5p1 + 4p2 - p3) + 6t*(-p0 + 3p1 - 3p2 + p3))

const HAT_POLYLINE = Ref(false)
const HAT_POLY_SEEN = Ref(false)
@inline function hat_polyline()
    if !HAT_POLY_SEEN[]
        HAT_POLYLINE[] = get(ENV, "JM_HAT_POLYLINE", "0") != "0"
        HAT_POLY_SEEN[] = true
    end
    HAT_POLYLINE[]
end
"""Project onto the polyline instead of the C1 curve (SEAM-1 A/B control)."""
hat_polyline!(on::Bool) = (HAT_POLYLINE[] = on; HAT_POLY_SEEN[] = true; on)

"""Foot point of (x,z) on the Catmull-Rom ribbon curve, refined from segment `seg`, parameter `t`.
Returns `(seg, t)` with t in [0,1]; the search MIGRATES across nodes so it follows one foot point
rather than one segment (that migration is what makes the frame continuous at a node)."""
@inline function _ribbon_foot(ts::TrackSurface, seg::Int, t::Float64, x::Float64, z::Float64)
    n = length(ts.pos)
    n < 4 && return (seg, clamp(t, 0.0, 1.0))
    @inbounds for _ in 1:3
        i = seg; h = mod1(i-1, n); j = mod1(i+1, n); k = mod1(i+2, n)
        ax, bx, cx2, dx2 = ts.pos[h][1], ts.pos[i][1], ts.pos[j][1], ts.pos[k][1]
        az, bz, cz2, dz2 = ts.pos[h][3], ts.pos[i][3], ts.pos[j][3], ts.pos[k][3]
        ex = _cr(ax, bx, cx2, dx2, t) - x;  ez = _cr(az, bz, cz2, dz2, t) - z
        d1x = _crd(ax, bx, cx2, dx2, t);    d1z = _crd(az, bz, cz2, dz2, t)
        d2x = _crdd(ax, bx, cx2, dx2, t);   d2z = _crdd(az, bz, cz2, dz2, t)
        g1 = ex*d1x + ez*d1z
        g2 = d1x*d1x + d1z*d1z + ex*d2x + ez*d2z

        g2 <= 1e-9 && break
        step = clamp(-g1/g2, -0.5, 0.5)
        t += step
        if t < 0.0
            seg = mod1(seg - 1, n); t += 1.0
        elseif t > 1.0
            seg = mod1(seg + 1, n); t -= 1.0
        end
        abs(step) < 1e-6 && break
    end
    (seg, clamp(t, 0.0, 1.0))
end

function hat(ts::TrackSurface, x::Real, z::Real)
    if HAT_COUNT_ON[]
        HAT_CALLS[] += 1
        if HAT_TIME_ON[]
            t0 = time_ns()
            r = _hat_impl(ts, x, z)
            HAT_NS[] += Int(time_ns() - t0)
            return r
        end
    end
    return _hat_impl(ts, x, z)
end

function _hat_impl(ts::TrackSurface, x::Real, z::Real)
    cx = clamp(floor(Int, (x - ts.x0) / ts.cell), 0, ts.nx - 1)
    cz = clamp(floor(Int, (z - ts.z0) / ts.cell), 0, ts.nz - 1)
    n = length(ts.pos)
    best = 0; bestd = Inf; bestt = 0.0
    for dz in -1:1, dx in -1:1
        gx, gz = cx + dx, cz + dz
        (0 <= gx < ts.nx && 0 <= gz < ts.nz) || continue
        for s in ts.buckets[gz*ts.nx + gx + 1]
            a = ts.pos[s]; b = ts.pos[mod1(s + 1, n)]
            abx, abz = b[1] - a[1], b[3] - a[3]
            len2 = abx^2 + abz^2
            len2 < 1e-9 && continue
            t = clamp(((x - a[1]) * abx + (z - a[3]) * abz) / len2, 0.0, 1.0)
            px, pz = a[1] + t * abx, a[3] + t * abz
            d = (x - px)^2 + (z - pz)^2
            if d < bestd
                bestd = d; best = s; bestt = t
            end
        end
    end
    best == 0 && return HATResult(0.0, (0.0, 1.0, 0.0), false, 0.0, 0.0, false, (1.0, 0.0))

    poly = hat_polyline()
    s, t = poly ? (best, bestt) : _ribbon_foot(ts, best, bestt, Float64(x), Float64(z))
    s2 = mod1(s + 1, n)
    a, b = ts.pos[s], ts.pos[s2]
    pp = unit((ts.perp[s][1] + t * (ts.perp[s2][1] - ts.perp[s][1]),
               ts.perp[s][2] + t * (ts.perp[s2][2] - ts.perp[s][2]),
               ts.perp[s][3] + t * (ts.perp[s2][3] - ts.perp[s][3])))
    nn = unit((ts.normal[s][1] + t * (ts.normal[s2][1] - ts.normal[s][1]),
               ts.normal[s][2] + t * (ts.normal[s2][2] - ts.normal[s][2]),
               ts.normal[s][3] + t * (ts.normal[s2][3] - ts.normal[s][3])))

    local cx0, cz0, cy, ppx, ppz
    if poly
        cx0 = a[1] + t * (b[1] - a[1]); cz0 = a[3] + t * (b[3] - a[3])
        cy  = a[2] + t * (b[2] - a[2])
        ppx = pp[1]; ppz = pp[3]
    else
        h4 = mod1(s - 1, n); k4 = mod1(s + 2, n)
        p0, p1, p2, p3 = ts.pos[h4], ts.pos[s], ts.pos[s2], ts.pos[k4]
        cx0 = _cr(p0[1], p1[1], p2[1], p3[1], t); cz0 = _cr(p0[3], p1[3], p2[3], p3[3], t)
        cy  = _cr(p0[2], p1[2], p2[2], p3[2], t)
        tx  = _crd(p0[1], p1[1], p2[1], p3[1], t); tz = _crd(p0[3], p1[3], p2[3], p3[3], t)
        tl  = hypot(tx, tz)

        ppx, ppz = tl < 1e-9 ? (pp[1], pp[3]) : (-tz/tl, tx/tl)
    end

    lat = (x - cx0) * ppx + (z - cz0) * ppz

    height = cy + lat * pp[2]
    hwl = ts.halfwidth[s][1] + t * (ts.halfwidth[s2][1] - ts.halfwidth[s][1])
    hwr = ts.halfwidth[s][2] + t * (ts.halfwidth[s2][2] - ts.halfwidth[s][2])
    ontrack = -hwr <= lat <= hwl

    ld2 = s2 == 1 ? ts.lap_length : ts.lapdist[s2]

    ld = ts.lapdist[s] + t * (ld2 - ts.lapdist[s])
    ld >= ts.lap_length && (ld -= ts.lap_length)
    HATResult(height, nn, ontrack, lat, ld, true, (ppx, ppz))
end
