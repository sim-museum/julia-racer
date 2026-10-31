

module HoleFill
using ..Render
export holefill_parts

const NRM = Ref(Tuple(parse.(Float32, split(get(ENV, "JM_HOLEFILL_NRM", "0,1,0"), ","))))
const EDGEFILL = get(ENV, "JM_HOLEFILL_EDGE", "1") != "0"
const NANFIX = get(ENV, "JM_HOLEFILL_NANFIX", "1") != "0"
const ROADEDGE = get(ENV, "JM_HOLEFILL_ROADEDGE", "1") != "0"
const SEAMFIX = get(ENV, "JM_HOLEFILL_SEAMFIX", "1") != "0"
const STRIDE = 11

struct TriIndex
    cell::Float64
    tris::Vector{NTuple{9,Float32}}
    part::Vector{Int}
    b::Dict{Tuple{Int,Int},Vector{Int}}
end

function TriIndex(parts, keep; cell = 6.0)
    tris = NTuple{9,Float32}[]; part = Int[]; b = Dict{Tuple{Int,Int},Vector{Int}}()
    for (pi, p) in enumerate(parts)
        keep(p) || continue
        v = p.verts
        for k in 1:3STRIDE:length(v)-3STRIDE+1
            t = (v[k], v[k+1], v[k+2], v[k+STRIDE], v[k+STRIDE+1], v[k+STRIDE+2], v[k+2STRIDE], v[k+2STRIDE+1], v[k+2STRIDE+2])

            ax, ay, az = t[4]-t[1], t[5]-t[2], t[6]-t[3]; bx, by, bz = t[7]-t[1], t[8]-t[2], t[9]-t[3]
            nx = ay*bz - az*by; ny = az*bx - ax*bz; nz = ax*by - ay*bx; nn = sqrt(nx^2 + ny^2 + nz^2)
            (nn > 1e-9 && abs(ny)/nn > 0.3) || continue
            push!(tris, t); push!(part, pi); i = length(tris)
            xs = (t[1], t[4], t[7]); zs = (t[3], t[6], t[9])
            for cx in floor(Int, minimum(xs)/cell):floor(Int, maximum(xs)/cell), cz in floor(Int, minimum(zs)/cell):floor(Int, maximum(zs)/cell)
                push!(get!(b, (cx, cz), Int[]), i)
            end
        end
    end
    TriIndex(cell, tris, part, b)
end

function column(ix::TriIndex, x, z)
    out = Tuple{Float64,Int}[]
    for i in get(ix.b, (floor(Int, x/ix.cell), floor(Int, z/ix.cell)), Int[])
        t = ix.tris[i]
        den = (t[6]-t[9])*(t[1]-t[7]) + (t[7]-t[4])*(t[3]-t[9]); abs(den) < 1e-12 && continue
        l1 = ((t[6]-t[9])*(x-t[7]) + (t[7]-t[4])*(z-t[9]))/den; l2 = ((t[9]-t[3])*(x-t[7]) + (t[1]-t[7])*(z-t[9]))/den
        (l1 >= -1e-4 && l2 >= -1e-4 && l1 + l2 <= 1 + 1e-4) || continue
        push!(out, (l1*t[2] + l2*t[5] + (1 - l1 - l2)*t[8], ix.part[i]))
    end
    out
end

"""The source part's texture scale (metres per UV unit): the MEDIAN over its edges (TD261017-1 (g1): one triangle's edge,
as before, could be skinny or skewed -- at Spa's Stavelot strip the filler came out coarser and darker than the grass
round it -- tried 2026-10-18: no visible change, so the first edge stays the default; JM_HOLEFILL_TILE1=0 = the median)."""
function tile_of(src)
    v = src.verts; nt = length(v) ÷ (3*STRIDE)
    if get(ENV, "JM_HOLEFILL_TILE1", "1") != "0" || nt == 0
        L = hypot(v[1+STRIDE]-v[1], v[3+STRIDE]-v[3]); U = hypot(v[10+STRIDE]-v[10], v[11+STRIDE]-v[11])
        return (L > 1e-3 && U > 1e-4) ? L/U : 8.0
    end
    r = Float64[]
    for t in 0:min(nt, 400)-1, (i, j) in ((0, 1), (1, 2), (2, 0))
        a = 3*STRIDE*t + i*STRIDE; b = 3*STRIDE*t + j*STRIDE
        L = hypot(v[b+1]-v[a+1], v[b+3]-v[a+3]); U = hypot(v[b+10]-v[a+10], v[b+11]-v[a+11])
        (L > 0.5 && U > 1e-4) && push!(r, L/U)
    end
    isempty(r) ? 8.0 : sort(r)[cld(length(r), 2)]
end

"""
    holefill_parts(parts, isroad, isground; near = 10.0, step = 2.0, reach = 6.0, sink = 0.05) -> (Vector{TrackPart}, quads)

Ground quads for the uncovered cells within `near` m of a road triangle. `isroad(tex)` picks the tarmac, `isground(tex)`
the parts a filler may copy its look and height from.
"""
function holefill_parts(parts, isroad, isground; near = 10.0, step = 2.0, reach = 6.0, sink = 0.05)
    ix = TriIndex(parts, p -> true)

    nr = ceil(Int, near/step); cand = Set{Tuple{Int,Int}}()
    for p in parts
        isroad(lowercase(p.tex)) || continue
        v = p.verts
        for k in 1:STRIDE:length(v)-STRIDE+1, dx in -nr:nr, dz in -nr:nr
            push!(cand, (round(Int, v[k]/step) + dx, round(Int, v[k+2]/step) + dz))
        end
    end
    holes0 = [c for c in cand if isempty(column(ix, step*c[1], step*c[2]))]
    isempty(holes0) && return Render.TrackPart[], 0
    holeset = Set(holes0)

    onroad(c) = any(hp -> isroad(lowercase(parts[hp[2]].tex)), column(ix, step*c[1], step*c[2]))

    ring = Set((c[1] + dx, c[2] + dz) for c in holes0 for dx in -1:1 for dz in -1:1)
    roadcells = Set(q for q in ring if !(q in holeset) && onroad(q))

    if ROADEDGE
        for c in cand
            (c in ring || c in holeset) && continue
            onroad(c) || continue
            any(o -> isempty(column(ix, step*(c[1] + o[1] - 0.5), step*(c[2] + o[2] - 0.5))), ((0, 0), (1, 0), (1, 1), (0, 1))) || continue
            push!(ring, c); push!(roadcells, c)
        end
    end
    holes = collect(ring)
    if haskey(ENV, "JM_HOLEFILL_CELLDIAG")
        q = parse.(Float64, split(ENV["JM_HOLEFILL_CELLDIAG"], ",")); rr = ceil(Int, q[3]/step)
        c0 = (round(Int, q[1]/step), round(Int, q[2]/step))
        for dz in -rr:rr
            row = ""
            for dx in -rr:rr
                c = (c0[1] + dx, c0[2] + dz); col = column(ix, step*c[1], step*c[2])
                row *= !(c in cand) ? " " : (c in holeset) ? "H" : (c in roadcells) ? "R" : (c in ring) ? "r" :
                       any(hp -> isroad(lowercase(parts[hp[2]].tex)), col) ? "=" : "#"
            end
            println("  [holefill cells] z ", lpad(step*(c0[2] + dz), 6), "  ", row, "   (x from ", step*(c0[1] - rr), ")")
        end
        for dx in -rr:rr
            c = (c0[1] + dx, c0[2]); col = column(ix, step*c[1], step*c[2])
            println("  [holefill col] x ", step*c[1], " z ", step*c[2], ": ", join(unique([lowercase(parts[hp[2]].tex) * "@" * string(round(hp[1], digits=2)) for hp in col]), " "))
        end
    end

    function ground_at(x, z)
        best = nothing
        for (h, pi) in column(ix, x, z)
            isground(lowercase(parts[pi].tex)) || continue
            (best === nothing || h < best[1]) && (best = (h, pi))
        end
        best
    end

    rr = ceil(Int, reach/step); cornerh = Dict{Tuple{Int,Int},Float64}(); cornerp = Dict{Tuple{Int,Int},Int}()
    function corner(ci, cj)
        haskey(cornerh, (ci, cj)) && return cornerh[(ci, cj)]

        col0 = column(ix, step*(ci - 0.5), step*(cj - 0.5))
        if !isempty(col0)
            g = ground_at(step*(ci - 0.5), step*(cj - 0.5))
            g === nothing || (cornerp[(ci, cj)] = g[2])
            return cornerh[(ci, cj)] = minimum(first.(col0))
        end
        hs = Float64[]; votes = Dict{Int,Int}()
        for d in 1:rr
            for dx in -d:d, dz in -d:d
                max(abs(dx), abs(dz)) == d || continue
                q = (ci + dx, cj + dz); q in holeset && continue
                g = ground_at(step*(q[1] - 0.5), step*(q[2] - 0.5)); g === nothing && continue
                push!(hs, g[1]); votes[g[2]] = get(votes, g[2], 0) + 1
            end
            length(hs) >= 3 && break
        end
        h = isempty(hs) ? NaN : sort(hs)[cld(length(hs), 2)]
        cornerh[(ci, cj)] = h
        isempty(votes) || (cornerp[(ci, cj)] = first(sort(collect(votes), by = x -> -x[2]))[1])
        h
    end
    byp = Dict{Int,Vector{Float32}}(); nq = 0; sharedh = Dict{Tuple{Int,Int},Float64}()
    sort!(holes, by = c -> c in roadcells ? 0 : 1)
    for (ci, cj) in holes
        cs = ((ci, cj), (ci + 1, cj), (ci + 1, cj + 1), (ci, cj + 1))
        hs = if (ci, cj) in roadcells
            h0 = minimum(first.(column(ix, step*ci, step*cj)))
            [(cc = column(ix, step*(c[1] - 0.5), step*(c[2] - 0.5)); isempty(cc) ? h0 : minimum(first.(cc))) for c in cs]
        else
            [corner(c...) for c in cs]
        end

        if NANFIX && all(isnan, hs)
            for d in 1:3, dx in -d:d, dz in -d:d
                max(abs(dx), abs(dz)) == d || continue
                h_ = get(cornerh, (ci + dx, cj + dz), NaN); isnan(h_) || (hs = [h_, h_, h_, h_]; break)
            end
        end
        if NANFIX && any(isnan, hs) && !all(isnan, hs)
            m_ = sum(h for h in hs if !isnan(h)) / count(!isnan, hs); hs = [isnan(h) ? m_ : h for h in hs]
        end
        any(isnan, hs) && continue

        if SEAMFIX
            for (k, c) in enumerate(cs)
                haskey(sharedh, c) ? (hs[k] = sharedh[c]) : (sharedh[c] = hs[k])
            end
        end
        pv = Dict{Int,Int}(); for c in cs; haskey(cornerp, c) && (pv[cornerp[c]] = get(pv, cornerp[c], 0) + 1); end

        if isempty(pv) && EDGEFILL
            for d in 1:3, dx in -d:d, dz in -d:d
                max(abs(dx), abs(dz)) == d || continue
                q = (ci + dx, cj + dz); haskey(cornerp, q) && (pv[cornerp[q]] = get(pv, cornerp[q], 0) + 1)
                isempty(pv) || break
            end
        end
        isempty(pv) && continue
        pi = first(sort(collect(pv), by = x -> -x[2]))[1]; src = parts[pi]
        col = (src.verts[7], src.verts[8], src.verts[9])
        tile = tile_of(src)
        out = get!(byp, pi, Float32[])
        pts = [(step*(c[1] - 0.5), h - sink, step*(c[2] - 0.5)) for (c, h) in zip(cs, hs)]
        for (a, b, d) in ((1, 3, 2), (1, 4, 3))
            for k in (a, b, d)
                (x, y, z) = pts[k]
                append!(out, Float32[x, y, z, NRM[][1], NRM[][2], NRM[][3], col[1], col[2], col[3], x/tile, z/tile])
            end
        end
        nq += 1
    end
    [Render.TrackPart(v, parts[pi].tex, parts[pi].col) for (pi, v) in byp], nq
end

end
