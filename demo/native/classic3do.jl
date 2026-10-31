

module CLASSIC3DO

const HIDE_GROUPS = Ref(Set{Int}())
using LinearAlgebra

export Mesh3DO, parse_3do, classic_placements

const LOD_ALL = get(ENV,"JM_LOD_ALL","0") != "0"

track_slots() = Set(parse.(Int, filter(!isempty, split(get(ENV, "JM_TRACK_SLOTS", ""), ","))))

const BANDTAG = Dict{NTuple{3,Int32},Int32}()
const BANDSHIFT = 40
const BANDSTAT = Ref((trees = 0, tris = 0, keys = 0, shared = 0, untagged = 0))
bandkey(p1, p2, p3) = ntuple(j -> round(Int32, 100*(Float64(p1[j]) + Float64(p2[j]) + Float64(p3[j]))), 3)

function switch_spec()
    sp = [split(e, "=") for e in filter(!isempty, split(get(ENV, "JM_SWITCH_VARS", ""), ","))]
    Set{Int}(parse(Int, e[1]) for e in sp), Dict{Int,Int}(parse(Int, e[1]) => parse(Int, e[2]) for e in sp if length(e) == 2)
end
const POSTPROC = Ref{Any}(nothing)

function posmat(d, m, s)
    cx,sx = cos(m[1]),sin(m[1]); cy,sy = cos(m[2]),sin(m[2]); cz,sz = cos(m[3]),sin(m[3])
    Rx = [1.0 0 0; 0 cx -sx; 0 sx cx]; Ry = [cy 0 sy; 0 1.0 0; -sy 0 cy]; Rz = [cz -sz 0; sz cz 0; 0 0 1.0]
    R = (Rz*Ry*Rx) .* s

    cl(x) = abs(x) > 5.0 ? 0.0 : Float64(x)
    [R[1,1] R[1,2] R[1,3] cl(d[1]); R[2,1] R[2,2] R[2,3] cl(d[2]); R[3,1] R[3,2] R[3,3] cl(d[3]); 0 0 0 1.0]
end

struct Tri
    p::NTuple{3,NTuple{3,Float32}}
    n::NTuple{3,NTuple{3,Float32}}
    uv::NTuple{3,NTuple{2,Float32}}
    tex::String
    col::NTuple{3,Float32}

    flat::Bool

    ptype::UInt32
end

struct Mesh3DO
    tris::Vector{Tri}
    textures::Vector{String}
    groups::Vector{Int}
end

@inline u32(b, o) = (o < 0 || o+4 > length(b)) ? UInt32(0) :
    UInt32(b[o+1]) | (UInt32(b[o+2])<<8) | (UInt32(b[o+3])<<16) | (UInt32(b[o+4])<<24)
@inline i32(b, o) = reinterpret(Int32, u32(b,o))
@inline f32(b, o) = reinterpret(Float32, u32(b,o))
tag(b, o) = String(b[o+1:o+4])

"""Parse a Classic .3do file into a `Mesh3DO` (triangulated, textured)."""
function parse_3do(path::AbstractString; textable::Union{Nothing,Vector{String}}=nothing, Mroot=nothing, extdepth::Int=0, lappos::Float64=NaN, laplen::Float64=NaN)
    TRACK_SLOTS = track_slots()
    b = read(path)
    String(b[1:4]) == "4OD3" || error("not a 3DO4 file: $path (magic=$(String(b[1:4])))")

    xyz_off = norm_off = strn_off = prim_off = 0
    strn_size = 0; nverts = 0
    o = 12
    while o + 12 <= length(b)
        t = tag(b, o); sz = Int(u32(b, o+8)); data = o + 12
        if     t == "SZYX"; xyz_off  = data; nverts = sz ÷ 16
        elseif t == "MRON"; norm_off = data
        elseif t == "NRTS"; strn_off = data; strn_size = sz
        elseif t == "MIRP"; prim_off = data
        end
        o = data + sz; o += (4 - o % 4) % 4
    end
    prim_off == 0 && error("no PRIM section in $path")

    strings = String[]
    if strn_off != 0
        cur = UInt8[]
        for i in strn_off : strn_off + strn_size - 1
            c = b[i+1]
            if c == 0xFF; break
            elseif c == 0x00; push!(strings, String(copy(cur))); empty!(cur)
            else push!(cur, c); end
        end
        !isempty(cur) && push!(strings, String(copy(cur)))
    end

    stroff = Dict{Int,String}()
    let p = 0
        for s in strings
            stroff[p] = s
            p += length(s) + 1
        end
    end

    vert(n) = (f32(b, xyz_off + n*16 + 4),  f32(b, xyz_off + n*16 + 8),  f32(b, xyz_off + n*16 + 12))
    nrm(n)  = norm_off == 0 ? (0f0,1f0,0f0) :
              (f32(b, norm_off + n*16 + 4), f32(b, norm_off + n*16 + 8), f32(b, norm_off + n*16 + 12))

    tris = Tri[]
    groups = Int[]
    CURTAG = Ref(0); TREEID = Dict{Int,Int}()
    used_tex = Set{String}()

    slotdiag = get(ENV, "JM_SLOTDIAG", "") != "" &&
               occursin(lowercase(ENV["JM_SLOTDIAG"]), lowercase(basename(path)))
    slotrows = NTuple{6,Int}[]
    selrows  = NTuple{3,Int}[]

    ok(c) = (0 < c <= 64) ? Int(c) : 0
    tdcount = Ref(0)
    rv(base, cnt) = [Int(u32(b, base + (k-1)*4)) for k in 1:cnt]
    ru(base, cnt) = [(f32(b, base + (k-1)*8), f32(b, base + (k-1)*8 + 4)) for k in 1:cnt]

    rgb(c) = (Float32(((c>>16)&0xff)/255), Float32(((c>>8)&0xff)/255), Float32((c&0xff)/255))
    function emit(verts, uvs, norms, tex, M, grp, col, ptype::UInt32 = UInt32(0))
        (length(verts) < 3 || length(verts) > 64) && return
        any(v -> v < 0 || v >= nverts, verts) && return
        push!(used_tex, tex)
        txp(p) = (Float32(M[1,1]*p[1]+M[1,2]*p[2]+M[1,3]*p[3]+M[1,4]),
                  Float32(M[2,1]*p[1]+M[2,2]*p[2]+M[2,3]*p[3]+M[2,4]),
                  Float32(M[3,1]*p[1]+M[3,2]*p[2]+M[3,3]*p[3]+M[3,4]))
        txn(n) = begin
            x=M[1,1]*n[1]+M[1,2]*n[2]+M[1,3]*n[3]; y=M[2,1]*n[1]+M[2,2]*n[2]+M[2,3]*n[3]; z=M[3,1]*n[1]+M[3,2]*n[2]+M[3,3]*n[3]
            l=sqrt(x*x+y*y+z*z); l<1e-9 && (l=1.0); (Float32(x/l),Float32(y/l),Float32(z/l))
        end
        P = [txp(vert(v)) for v in verts]

        if tex in TEXVERT_WANT && get(TEXVERT_N, tex, 0) < 3
            TEXVERT_N[tex] = get(TEXVERT_N, tex, 0) + 1
            println("  [texverts] ", tex, " grp=", grp,
                    "  M.translation=(", round(M[1,4], digits=3), ", ", round(M[2,4], digits=3), ", ", round(M[3,4], digits=3), ")")
            for (i, v) in enumerate(verts)
                i > 4 && break
                lv = vert(v); wv = txp(lv)
                println("     local=(", round(lv[1], digits=3), ", ", round(lv[2], digits=3), ", ", round(lv[3], digits=3),
                        ")  ->  world=(", round(wv[1], digits=3), ", ", round(wv[2], digits=3), ", ", round(wv[3], digits=3), ")")
            end
        end

        fn = (0f0,0f0,0f0)
        let nx=0f0,ny=0f0,nz=0f0
            for k in 1:length(P)
                a = P[k]; c = P[k % length(P) + 1]
                nx += (a[2]-c[2])*(a[3]+c[3]); ny += (a[3]-c[3])*(a[1]+c[1]); nz += (a[1]-c[1])*(a[2]+c[2])
            end
            l = sqrt(nx*nx+ny*ny+nz*nz); l < 1f-9 && (l = 1f0)
            fn = (nx/l, ny/l, nz/l)
        end
        N = isempty(norms) ? [fn for _ in P] : [txn(nrm(n)) for n in norms]
        isflat = isempty(uvs)
        UV = isflat ? [(0f0,0f0) for _ in P] : uvs

        hidegrp = get(ENV, "JM_HIDE_GROUP", "")
        if !(get(ENV, "JM_HIDE_PARKED", "0") != "0" && PARKDEPTH[] > 0) && !(hidegrp != "" && string(grp) in split(hidegrp, ",")) && !(grp in HIDE_GROUPS[])
            for k in 2:length(P)-1
                push!(tris, Tri((P[1],P[k],P[k+1]), (N[1],N[k],N[k+1]), (UV[1],UV[k],UV[k+1]), tex, col, isflat, ptype))
                push!(groups, grp + (CURTAG[] << BANDSHIFT))
            end
        end
        push!(ALL_GROUPS, grp); PARKDEPTH[] > 0 && push!(PARKED_GROUPS, grp)
        if get(ENV, "JM_POSDIAG", "") != ""; ps = join(string.(PATH, base=16), ">"); for _ in 2:length(P)-1; push!(TRIPATH, ps); TRIPATH_PUSHES[] += 1; end; end
    end

    I4 = Matrix{Float64}(I,4,4)
    visited = Set{Int}()

    PRIMTALLY = Dict{UInt32,Int}()
    PARKDEPTH = Ref(0); PARKED_GROUPS = Set{Int}(); ALL_GROUPS = Set{Int}()
    TEXVERT_WANT = Set(filter(!isempty, split(get(ENV, "JM_TEXVERTS", ""), ',')))
    TEXVERT_N = Dict{String,Int}()
    SWITCH_VARS, SWITCH_FORCE = switch_spec()
    SWV = copy(SWITCH_FORCE)
    PATH = UInt32[]; TRIPATH = String[]

    TRIPATH_PUSHES = Ref(0)
    function walk(off::Int, curtex::String, depth::Int, M, grp::Int)
        n0 = length(PATH); _walk(off, curtex, depth, M, grp); resize!(PATH, n0)
    end
    function _walk(off::Int, curtex::String, depth::Int, M, grp::Int)
        (off < 0 || off in visited || depth > 220) && return
        push!(visited, off)
        p = prim_off + off
        p + 4 > length(b) && return
        typ = u32(b, p)

        if get(ENV, "JM_NODEDIAG", "") != "" && string(off) in split(ENV["JM_NODEDIAG"], ",")
            sc = sqrt(M[1,1]^2 + M[2,1]^2 + M[3,1]^2)
            println("   [nodediag] node ", off, " type 0x", string(typ, base=16), " depth ", depth,
                    "  accumulated translation (", round(M[1,4],digits=3), ", ", round(M[2,4],digits=3),
                    ", ", round(M[3,4],digits=3), ")  scale ", round(sc,digits=3))

            print("   [nodediag]   raw:")
            for w in 0:15
                q = p + 4*w
                q + 4 > length(b) && break
                print("  [", w, "]=", i32(b, q), "/", round(f32(b, q), digits=4))
            end
            println()
        end
        PRIMTALLY[typ] = get(PRIMTALLY, typ, 0) + 1
        push!(PATH, typ)
        if typ == 0x04
            cnt = Int(u32(b, p+4))

            segtree = cnt == 8 && (c8 = Int(i32(b, p + 36)); c8 >= 0 && u32(b, prim_off + c8) == 0x10)
            tid = (segtree && CURTAG[] == 0) ? get!(() -> length(TREEID) + 1, TREEID, off) : 0
            for k in 1:cnt
                (segtree && !isempty(TRACK_SLOTS) && k <= 7 && !(k in TRACK_SLOTS)) && continue
                t0 = CURTAG[]; tid > 0 && (CURTAG[] = k <= 4 ? 4tid + 1 : k <= 6 ? 4tid + 2 : k == 7 ? 4tid + 3 : 0)
                walk(Int(i32(b, p + 4 + k*4)), curtex, depth+1, M, grp)
                CURTAG[] = t0
            end
        elseif typ == 0x05
            child = Int(i32(b, p+4)); sub = u32(b, p+8)
            tex = curtex

            if sub in (0x1, 0x5, 0xD)
                tex = get(stroff, Int(u32(b, p+12)), curtex)
            end
            slotdiag && push!(selrows, (Int(sub), Int(u32(b, p+12)), Int(u32(b, p+16))))

            if sub == 0x11
                cnt = Int(u32(b, p+12))
                if 0 < cnt <= 64
                    tbl = String[get(stroff, Int(u32(b, p+16+(k-1)*4)), "") for k in 1:cnt]
                    curtable[] = tbl
                end
            elseif sub in (0x20, 0x24, 0x2C)

                slot = Int(u32(b, p+12))
                tt = curtable[]
                if tt !== nothing && 0 <= slot < length(tt) && tt[slot+1] != ""
                    tex = tt[slot+1]
                end
            end
            walk(child, tex, depth+1, M, grp)
        elseif typ == 0x14
            (Int(u32(b, p+4)) & 0x3ff) in SWITCH_VARS && !haskey(SWITCH_FORCE, Int(u32(b, p+4)) & 0x3ff) && (SWV[Int(u32(b, p+4)) & 0x3ff] = Int(i32(b, p+8)))
        elseif typ == 0x0E

            if extdepth < 2
                nm = get(stroff, Int(u32(b, p+4)), "")
                if nm != ""
                    dir = dirname(path)
                    hit = ""
                    for f in readdir(dir)
                        lowercase(f) == lowercase(nm * ".3do") && (hit = joinpath(dir, f); break)
                    end
                    if hit != ""
                        sub3 = parse_3do(hit; textable=curtable[], Mroot=M, extdepth=extdepth+1)
                        append!(tris, sub3.tris)

                        append!(groups, sub3.groups)
                        for t in sub3.textures; push!(used_tex, t); end
                    end
                end
            end
        elseif typ in (0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B)
            nchild = typ==0x06 ? 1 : typ in (0x07,0x0B) ? 2 : typ==0x08 ? 4 : 3
            for k in 1:nchild
                walk(Int(i32(b, p + 8 + (k-1)*4)), curtex, depth+1, M, grp)
            end
        elseif typ in (0x0D, 0x13, 0x16)
            d = (f32(b,p+4), f32(b,p+8), f32(b,p+12)); mm = (f32(b,p+16), f32(b,p+20), f32(b,p+24))
            s = f32(b,p+28); s = (s <= 0f0 ? 1.0 : Float64(s))

            if get(ENV, "JM_POSDIAG", "") != "" && (ENV["JM_POSDIAG"] == "all" || string(off) in split(ENV["JM_POSDIAG"], ","))
                sc = sqrt(M[1,1]^2 + M[2,1]^2 + M[3,1]^2)
                println("   [posdiag] node ", off, " type 0x", string(typ, base=16), " depth ", depth, "  d=", d, "  rot=", mm, "  s=", s,
                        "  parent: translation (", round(M[1,4],digits=3), ",", round(M[2,4],digits=3), ",", round(M[3,4],digits=3), ") scale ", round(sc,digits=3))
            end

            parked = any(abs(x) > 5.0 for x in d)
            parked && (PARKDEPTH[] += 1)

            pm = (parked && get(ENV, "JM_PARK_IDENT", "0") != "0") ? posmat(d, (0.0, 0.0, 0.0), 1.0) : posmat(d, mm, s)
            walk(Int(i32(b, p + 8*4)), curtex, depth+1, M * pm, off)
            parked && (PARKDEPTH[] -= 1)
        elseif typ == 0x19
            walk(Int(i32(b, p + 9*4)), curtex, depth+1, M, grp)
        elseif typ == 0x11

            cnt = Int(u32(b, p+16))
            if get(ENV,"JM_PRIMDIAG","") != "" && occursin(lowercase(ENV["JM_PRIMDIAG"]), lowercase(basename(path)))
                println("     [LOD node] count = ", cnt, "  source ", u32(b, p+4), " param ", u32(b, p+8), " op ", u32(b, p+12),
                        "  (walked only when 0 < count <= 4096)")
                for kk in 1:min(cnt,6)
                    println("        child ", kk, ": dist ", f32(b, p + 20 + (kk-1)*8),
                            " off ", i32(b, p + 20 + (kk-1)*8 + 4))
                end
            end
            if 0 < cnt <= 4096
                ds = [f32(b, p + 20 + (k-1)*8) for k in 1:cnt]
                offs = [Int(i32(b, p + 20 + (k-1)*8 + 4)) for k in 1:cnt]

                live = get(ENV,"JM_LOD_NULLS","0") != "0" ? collect(1:cnt) : [k for k in 1:cnt if offs[k] >= 0]

                _lodmax = get(ENV,"JM_LOD_PICK","min") == "max"

                _src = u32(b, p+4); _op = u32(b, p+12); _var = Int(u32(b, p+8)) & 0x3ff
                if !LOD_ALL && _src == 4 && _op == 5 && haskey(SWV, _var)
                    k = SWV[_var] + 1
                    1 <= k <= cnt && offs[k] >= 0 && walk(offs[k], curtex, depth+1, M, grp)
                elseif !LOD_ALL && _src == 1 && 1 <= _op <= 3 && isfinite(lappos) && get(ENV, "JM_LOD_LAPPOS", "1") != "0"
                    cands = Int[]
                    for dv in -18.0e6:0.5e6:18.0e6
                        v = lappos + dv
                        isfinite(laplen) && laplen > 0 && (v = mod(v, laplen))
                        v < 0 && continue
                        for k in 1:cnt
                            hit = _op == 1 ? v <= ds[k] : _op == 2 ? v >= ds[k] : v == ds[k]
                            hit || continue
                            offs[k] >= 0 && !(k in cands) && push!(cands, k)
                            break
                        end
                    end
                    best = 0; bestn = -1
                    if length(cands) == 1
                        best = cands[1]
                    else
                        for k in cands
                            nt = length(tris); ng = length(groups); vis = copy(visited)
                            walk(offs[k], curtex, depth+1, M, grp)
                            n = length(tris) - nt
                            resize!(tris, nt); resize!(groups, ng); empty!(visited); union!(visited, vis)
                            n > bestn && (bestn = n; best = k)
                        end
                    end
                    best > 0 && walk(offs[best], curtex, depth+1, M, grp)
                elseif !LOD_ALL && length(unique(ds)) > 1 && !isempty(live)
                    k = live[_lodmax ? argmax([ds[k] for k in live]) : argmin([ds[k] for k in live])]
                    walk(offs[k], curtex, depth+1, M, grp)
                else
                    for k in 1:cnt
                        walk(Int(i32(b, p + 20 + (k-1)*8 + 4)), curtex, depth+1, M, grp)
                    end
                end
            end

        elseif typ == 0x81B
            cnt = ok(u32(b,p+8)); vs = rv(p+12, cnt); emit(vs, [], [], curtex, M, grp, rgb(u32(b,p+4)), UInt32(0x81B))
        elseif typ == 0x81C
            cnt = ok(u32(b,p+8)); vs = rv(p+12, cnt); emit(vs, [], [], curtex, M, grp, rgb(u32(b,p+4)), UInt32(0x81C))
        elseif typ == 0x81D

            if get(ENV,"JM_TYPEDUMP","") == "0x81D" && tdcount[] < 6
                tdcount[] += 1
                println("  [0x81D #", tdcount[], "] p+4=0x", string(u32(b,p+4), base=16, pad=8),
                        "  p+8=", u32(b,p+8), " (count)",
                        "  p+12=0x", string(u32(b,p+12), base=16, pad=8),
                        "  p+16=0x", string(u32(b,p+16), base=16, pad=8),
                        "  tex='", curtex, "'")
                flush(stdout)
            end
            if get(ENV,"JM_TYPEDUMP","") == "0x81D" && tdcount[] <= 6

                let c = Int(ok(u32(b,p+8))), q = p + 12 + 8*c
                    println("      after vert+norm (", c, " each): u32 ", join([string(u32(b, q + 4i)) for i in 0:2c+3], " "))
                    println("      as f32: ", join([string(round(f32(b, q + 4i), digits=3)) for i in 0:2c+3], " "))
                end
            end
            cnt = ok(u32(b,p+8)); emit(rv(p+12,cnt), [], rv(p+12+cnt*4,cnt), curtex, M, grp, rgb(u32(b,p+4)), UInt32(0x81D))
        elseif typ == 0x81E
            cnt = ok(u32(b,p+8)); emit(rv(p+12,cnt), [], rv(p+12+2*cnt*4,cnt), curtex, M, grp, rgb(u32(b,p+4)), UInt32(0x81E))
        elseif typ == 0x81F
            cnt = ok(u32(b,p+12))
            slotdiag && curtex == "" && push!(slotrows, (length(tris), Int(typ), Int(u32(b,p+4)), Int(u32(b,p+8)), Int(u32(b,p+12)), Int(u32(b,p+16))))
            emit(rv(p+16+cnt*8,cnt), ru(p+16,cnt), [], curtex, M, grp, rgb(u32(b,p+8)), UInt32(0x81F))
        elseif typ == 0x820
            cnt = ok(u32(b,p+12)); emit(rv(p+16+cnt*8,cnt), ru(p+16,cnt), [], curtex, M, grp, rgb(u32(b,p+8)), UInt32(0x820))
        elseif typ == 0x821
            cnt = ok(u32(b,p+12))
            slotdiag && curtex == "" && push!(slotrows, (length(tris), Int(typ), Int(u32(b,p+4)), Int(u32(b,p+8)), Int(u32(b,p+12)), Int(u32(b,p+16))))
            emit(rv(p+16+cnt*8,cnt), ru(p+16,cnt), rv(p+16+cnt*12,cnt), curtex, M, grp, rgb(u32(b,p+8)), UInt32(0x821))
        elseif typ == 0x1F
            cnt = ok(u32(b,p+12))
            slotdiag && curtex == "" && push!(slotrows, (length(tris), Int(typ), Int(u32(b,p+4)), Int(u32(b,p+8)), Int(u32(b,p+12)), Int(u32(b,p+16))))
            emit(rv(p+16+cnt*8,cnt), ru(p+16,cnt), [], curtex, M, grp, rgb(u32(b,p+8)), UInt32(0x1F))
        elseif typ == 0x20
            cnt = ok(u32(b,p+16)); emit(rv(p+20+cnt*8,cnt), ru(p+20,cnt), [], curtex, M, grp, (1f0,1f0,1f0), UInt32(0x20))
        elseif typ == 0x21
            cnt = ok(u32(b,p+12))
            slotdiag && curtex == "" && push!(slotrows, (length(tris), Int(typ), Int(u32(b,p+4)), Int(u32(b,p+8)), Int(u32(b,p+12)), Int(u32(b,p+16))))
            emit(rv(p+16+cnt*8,cnt), ru(p+16,cnt), rv(p+16+cnt*12,cnt), curtex, M, grp, (1f0,1f0,1f0), UInt32(0x21))
        end

    end

    curtable = Ref{Union{Nothing,Vector{String}}}(textable)
    root = Int(u32(b, prim_off))
    walk(root, "", 0, Mroot === nothing ? I4 : Mroot, 0)
    if slotdiag && !isempty(selrows)
        tal = Dict{NTuple{3,Int},Int}()
        for r in selrows; tal[r] = get(tal, r, 0) + 1; end
        println("  [slotdiag] selectors walked:")
        for (k,c) in sort(collect(tal))
            println("     sub 0x", string(k[1], base=16), "  w3=", k[2], " w4=", k[3], "  x", c)
        end
    end
    if slotdiag && !isempty(slotrows)
        println("  [slotdiag] ", basename(path), ": ", length(slotrows), " untextured poly headers")
        shown = Dict{NTuple{4,Int},Int}()
        for r in slotrows
            k = (r[2], r[3], r[4], 0)
            shown[k] = get(shown, k, 0) + 1
        end
        for (k, c) in sort(collect(shown))
            println("     typ 0x", string(k[1], base=16), "  w1=", k[2], " w2=", k[3], "  x", c)
        end
        flush(stdout)
    end
    if get(ENV,"JM_PRIMDIAG","") != "" && occursin(lowercase(ENV["JM_PRIMDIAG"]), lowercase(basename(path)))
        handled = Set{UInt32}([0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0D,0x13,0x16,0x19,0x11,
                               0x81B,0x81C,0x81D,0x81E,0x81F,0x820,0x821,0x822])
        println("  [primdiag] ", basename(path), ": ", length(tris), " tris from ",
                sum(values(PRIMTALLY)), " nodes")
        for (t,c) in sort(collect(PRIMTALLY), by=x->-x[2])
            println("     type 0x", string(t, base=16, pad=3), "  x", rpad(c,6),
                    t in handled ? "handled" : "*** NOT HANDLED ***")
        end
        flush(stdout)
    end
    if get(ENV, "JM_POSDIAG", "") != ""
        println("   [posdiag] placing groups: ", length(ALL_GROUPS), " total, ", length(PARKED_GROUPS), " under a PARK (|d|>5) node")
        if get(ENV, "JM_POSDIAG", "") == "groups"
            gt = Dict{Int,Dict{String,Int}}()
            for (t, g) in zip(tris, groups); d = get!(gt, g, Dict{String,Int}()); d[t.tex] = get(d, t.tex, 0) + 1; end
            for g in sort(collect(keys(gt)))
                println("   [posdiag]   group ", g, (g in PARKED_GROUPS ? " PARKED" : ""), ": ", join([string(k == "" ? "(untex)" : k, "=", v) for (k, v) in sort(collect(gt[g]))], " "))
            end
        end
        for g in sort(collect(PARKED_GROUPS))
            texs = Dict{String,Int}(); for (k,gg) in enumerate(groups); gg == g && (texs[tris[k].tex] = get(texs, tris[k].tex, 0) + 1); end
            println("   [posdiag]   parked group ", g, ": ", join([string(k == "" ? "(untex)" : k, "=", v) for (k,v) in sort(collect(texs))], " "))
        end
    end

    if get(ENV, "JM_POSDIAG", "") != "" && length(TRIPATH) != length(tris)
        println("   [posdiag] path report SKIPPED: TRIPATH=", length(TRIPATH),
                " (pushes counted=", TRIPATH_PUSHES[], ") but tris=", length(tris),
                " -- the node-path buckets below would be wrong, so they are not printed.",
                " This is the chassis case; the per-group counts above are still valid.")
    end
    if get(ENV, "JM_POSDIAG", "") != "" && length(TRIPATH) == length(tris)

        let seen = sort(collect(Set(groups)))
            println("   [posdiag] path report over ", length(tris), " tris in ", length(seen),
                    " group(s); front groups present: 3560=", (3560 in seen), " 6600=", (6600 in seen),
                    seen == Int[] ? "" : string("  (ids ", first(seen), "..", last(seen), ")"))
        end

        for g in (3560, 6600)
            long = Dict{String,Int}(); short = Dict{String,Int}()
            for (k,t) in enumerate(tris)
                groups[k] == g || continue
                e(a,b) = sqrt(sum((a[i]-b[i])^2 for i in 1:3)); L = max(e(t.p[1],t.p[2]), e(t.p[2],t.p[3]), e(t.p[1],t.p[3]))
                d = L > 1.4 ? long : short; d[TRIPATH[k]] = get(d, TRIPATH[k], 0) + 1
            end
            println("   [posdiag] group $g LONG-strip paths: ", join(["$v x [$k]" for (k,v) in sort(collect(long), by=x->-x[2])], "  "))
            println("   [posdiag] group $g short paths:      ", join(["$v x [$k]" for (k,v) in sort(collect(short), by=x->-x[2])][1:min(end,3)], "  "))
        end
    end
    m_out = Mesh3DO(tris, sort(collect(used_tex)), groups)

    m_out = (extdepth == 0 && POSTPROC[] !== nothing) ? POSTPROC[](path, m_out) : m_out
    if !isempty(TREEID)
        if extdepth == 0
            empty!(BANDTAG)
            for (t, g) in zip(m_out.tris, m_out.groups)
                k = bandkey(t.p...); g2 = g >> BANDSHIFT
                tg = g2 == 0 ? Int32(0) : Int32(((g2 >> 2) << 3) | (1 << ((g2 & 3) - 1)))
                old = get(BANDTAG, k, nothing)
                BANDTAG[k] = old === nothing ? tg : (old != 0 && tg != 0 && old >> 3 == tg >> 3) ? (old | tg) : Int32(0)
            end
            BANDSTAT[] = (trees = length(TREEID), tris = count(g -> g >> BANDSHIFT > 0, m_out.groups), keys = length(BANDTAG),
                          shared = count(x -> count_ones(x & 7) > 1, values(BANDTAG)), untagged = count(==(0), values(BANDTAG)))
        end
        m_out = Mesh3DO(m_out.tris, m_out.textures, [g & ((1 << BANDSHIFT) - 1) for g in m_out.groups])
    end
    m_out
end

"""
    classic_placements(path) -> Vector{Tuple{String, NTuple{7,Float64}}}

Classic places `.dat` scenery meshes (corner terrain sections, trees, signs, buildings)
into a track via PRIM node type 0x0E — a "named external sub-object reference":
`[14, string-offset, 0,  19, dx,dy,dz, rx,ry,rz, scale, child]` (an inline 0x13
positioner).  The geometry lives in a separate `.dat` mesh named by the string; the
positioner gives its WORLD transform.  These nodes hang under unhandled parents (so the
main walk never reaches them) and their translations are huge (the regular positioner
clamp would zero them), so we recover them by scanning PRIM for the fixed signature.
Returns (sub-object name, (dx,dy,dz, rx,ry,rz, scale)) per placement.
"""
function classic_placements(path::AbstractString)
    b = read(path)
    String(b[1:4]) == "4OD3" || error("not a 3DO4 file: $path")
    strn_off = strn_sz = prim_off = prim_sz = 0
    o = 12
    while o + 12 <= length(b)
        t = tag(b, o); sz = Int(u32(b, o+8)); data = o + 12
        t == "NRTS" && (strn_off = data; strn_sz = sz)
        t == "MIRP" && (prim_off = data; prim_sz = sz)
        o = data + sz; o += (4 - o % 4) % 4
    end

    strings = String[]; cur = UInt8[]
    for i in strn_off : strn_off + strn_sz - 1
        c = b[i+1]
        if c == 0xFF; break
        elseif c == 0x00; push!(strings, String(copy(cur))); empty!(cur)
        else push!(cur, c); end
    end
    stroff = Dict{Int,String}(); let q = 0
        for s in strings; stroff[q] = s; q += length(s) + 1; end
    end
    res = Tuple{String,NTuple{7,Float64}}[]
    for N in prim_off : 4 : prim_off + prim_sz - 44
        (u32(b,N) == 14 && u32(b,N+8) == 0 && u32(b,N+12) == 19) || continue
        nm = get(stroff, Int(u32(b,N+4)), ""); nm == "" && continue
        res = push!(res, (nm, (Float64(f32(b,N+16)), Float64(f32(b,N+20)), Float64(f32(b,N+24)),
                               Float64(f32(b,N+28)), Float64(f32(b,N+32)), Float64(f32(b,N+36)),
                               Float64(f32(b,N+40)))))
    end
    res
end

end
