

function dx_roadtris()
    rt = [t for t in TRACKMESH.tris if ROAD_TEX(lowercase(t.tex))]
    cnt = Dict{Int,Int}(); len = Dict{Int,Float64}(); texb = Dict{Int,Dict{String,Int}}()
    for t in rt
        cx = (Float64(t.p[1][1]) + Float64(t.p[2][1]) + Float64(t.p[3][1]))/3
        cy = (Float64(t.p[1][2]) + Float64(t.p[2][2]) + Float64(t.p[3][2]))/3
        hr = JRTrack.hat(TRKSURF, cx, cy); (hr.found && abs(hr.lateral) < 12) || continue
        b = floor(Int, hr.lapdist/100)
        cnt[b] = get(cnt, b, 0) + 1
        tb = get!(texb, b, Dict{String,Int}()); tb[lowercase(t.tex)] = get(tb, lowercase(t.tex), 0) + 1
        emax = maximum(hypot(Float64(t.p[i][1]) - Float64(t.p[j][1]), Float64(t.p[i][2]) - Float64(t.p[j][2])) for (i, j) in ((1,2),(2,3),(3,1)))
        len[b] = get(len, b, 0.0) + emax
    end
    println("\n==== JM_ROADTRIS ", uppercasefirst(TRACKSEL), "  (", length(rt), " road tris, lap ", round(Int, TRKSURF.lap_length), " m) ====")
    println("  lapdist   tris/100m   mean longest edge (m)")
    for b in sort(collect(keys(cnt)))
        println("  ", lpad(b*100, 6), "   ", lpad(cnt[b], 6), "      ", round(len[b]/cnt[b], digits = 1))
    end
    coarse = sort(collect(keys(cnt)); by = b -> len[b]/cnt[b], rev = true)[1:min(end, 8)]
    println("  coarsest (longest mean edge): ", join(["$(b*100)m:$(round(len[b]/cnt[b],digits=1))m/$(cnt[b])" for b in coarse], "  "))

    for b in coarse
        tb = sort(collect(texb[b]); by = kv -> -kv[2])
        println("    ", lpad(b*100, 6), "m textures: ", join(["$(k)=$(v)" for (k, v) in tb[1:min(end, 6)]], " "))
    end
    fine = sort(collect(keys(cnt)); by = b -> len[b]/cnt[b])[1:min(end, 3)]
    for b in fine
        tb = sort(collect(texb[b]); by = kv -> -kv[2])
        println("    fine ", lpad(b*100, 6), "m textures: ", join(["$(k)=$(v)" for (k, v) in tb[1:min(end, 6)]], " "))
    end
    println("ROADTRIS_RESULT track=", TRACKSEL, " buckets=", length(cnt))
    flush(stdout); exit(0)
end

function dx_roadedge()

    a_, b_ = parse.(Float64, split(get(ENV, "JM_ROADEDGE", "550:700"), ":"))
    rt = [t for t in TRACKMESH.tris if ROAD_TEX(lowercase(t.tex))]
    key(p) = (round(Float64(p[1]), digits = 2), round(Float64(p[2]), digits = 2), round(Float64(p[3]), digits = 2))
    ecount = Dict{Tuple{Any,Any},Int}(); etex = Dict{Tuple{Any,Any},String}(); inwin = 0
    for t in rt
        cx = (Float64(t.p[1][1]) + Float64(t.p[2][1]) + Float64(t.p[3][1]))/3
        cy = (Float64(t.p[1][2]) + Float64(t.p[2][2]) + Float64(t.p[3][2]))/3
        hr = JRTrack.hat(TRKSURF, cx, cy); (hr.found && abs(hr.lateral) < 12 && a_ <= hr.lapdist <= b_) || continue
        inwin += 1
        for (i, j) in ((1,2),(2,3),(3,1))
            k1 = key(t.p[i]); k2 = key(t.p[j]); e = k1 < k2 ? (k1, k2) : (k2, k1)
            ecount[e] = get(ecount, e, 0) + 1; etex[e] = lowercase(t.tex)
        end
    end
    bnd = [e for (e, c) in ecount if c == 1]
    println("\n==== JM_ROADEDGE ", uppercasefirst(TRACKSEL), " lapdist ", a_, "-", b_, ": ", inwin, " road tris, ", length(ecount), " edges, ", length(bnd), " boundary edges ====")
    tex = Dict{String,Int}(); for e in bnd; tex[etex[e]] = get(tex, etex[e], 0) + 1; end
    println("  boundary-edge textures: ", join(["$(k)=$(v)" for (k, v) in sort(collect(tex); by = kv -> -kv[2])], " "))
    verts = Dict{Any,Tuple{Float64,Float64}}()
    for e in bnd, k in e
        haskey(verts, k) && continue
        hr = JRTrack.hat(TRKSURF, k[1], k[2]); hr.found || continue
        verts[k] = (hr.lapdist, hr.lateral)
    end
    stats(pts) = begin
        n = length(pts); n < 3 && return (n, 0.0, 0.0, 0.0, Int[])
        seg = [hypot(pts[i+1][1] - pts[i][1], pts[i+1][2] - pts[i][2]) for i in 1:n-1]
        hd = [atan(pts[i+1][2] - pts[i][2], pts[i+1][1] - pts[i][1]) for i in 1:n-1]
        st = [abs(rad2deg(rem(hd[i+1] - hd[i], 2pi, RoundNearest))) for i in 1:n-2]
        (n, sum(seg)/length(seg), maximum(seg), maximum(st), st)
    end

    minlat = parse(Float64, get(ENV, "JM_ROADEDGE_MINLAT", "3.3"))
    for (side, sel) in (("left (lateral>0)", v -> v[2] > 0), ("right (lateral<0)", v -> v[2] < 0))
        seen = Set{Int}()
        ks = sort([k for (k, v) in verts if sel(v) && abs(v[2]) >= minlat && a_ <= v[1] <= b_]; by = k -> verts[k][1])
        ks = [k for k in ks if (b = round(Int, verts[k][1]*5); b in seen ? false : (push!(seen, b); true))]
        pts = [(Float64(k[1]), Float64(k[2])) for k in ks]
        n, mseg, xseg, xst, st = stats(pts)
        big = count(s -> s > 8, st)
        println("  ", rpad(side, 18), " verts=", n, "  mean seg=", round(mseg, digits = 1), " m  longest=", round(xseg, digits = 1), " m  max heading step=", round(xst, digits = 1), " deg  steps>8deg=", big)
        for (k, kk) in enumerate(ks)
            k > 40 && (println("     ..."); break)
            v = verts[kk]; s = k <= length(st) + 1 && k >= 2 ? round(st[k-1], digits = 1) : 0.0
            println("     lap=", lpad(round(v[1], digits = 1), 7), "  lat=", lpad(round(v[2], digits = 2), 6), "  step=", s)
        end
    end

    ci = [i for i in eachindex(TRKSURF.lapdist) if a_ <= TRKSURF.lapdist[i] <= b_]
    n, mseg, xseg, xst, st = stats([(TRKSURF.pos[i][1], TRKSURF.pos[i][3]) for i in ci])
    println("  centreline waypoints (", n, ", mean spacing ", round(mseg, digits = 1), " m): max heading step=", round(xst, digits = 1), " deg  (", round(sum(st)/max(1,length(st)), digits = 2), " mean)")
    println("ROADEDGE_RESULT track=", TRACKSEL, " tris=", inwin, " boundary=", length(bnd))
    flush(stdout); exit(0)
end

function dx_objcensus()
    rows = NTuple{7,Any}[]
    for (nm, ox, oz, oy, kind, issolid) in OBJINSTS
        kind === :dropped && continue
        hr = JRTrack.hat(TRKSURF, Float64(ox), Float64(oz)); hr.found || continue

        g = JRTrack.hat3d(TERRAIN0, Float64(ox), Float64(oz); ref = Inf)
        base = Float64(oy) + Float64(get(OBJ_YMN, nm, 0f0))
        dy = g[3] ? Float64(oy) - Float64(g[1]) : NaN
        onroad = ROADHAT !== TERRAIN0 && (JRTrack.hat3d(ROADHAT, Float64(ox), Float64(oz); ref = Inf)[3] && hr.on_track)
        push!(rows, (nm, kind, hr.lapdist, hr.lateral, dy, onroad, Float64(ox)))
    end
    nbb = count(r -> r[2] === :bb, rows); nm_ = count(r -> r[2] === :mesh, rows)
    println("\n==== JM_OBJCENSUS ", uppercasefirst(TRACKSEL), ": ", nm_, " meshes + ", nbb, " billboards rendered ====")
    onr = filter(r -> r[6] || abs(r[4]) < 4.5, rows)
    flo = filter(r -> !isnan(r[5]) && r[5] > 1.5, rows)
    bur = filter(r -> !isnan(r[5]) && r[5] < -1.5, rows)
    println("  on/at the road (tarmac under origin or |lat| < 4.5): ", length(onr), "   floating (placement > 1.5 m above the bare terrain): ", length(flo), "   sunk (< -1.5 m): ", length(bur))
    ndis = count(((nm, ox, oz, oy, kind, _),) -> (rz = get(OBJ_RECZ, (round(Float64(ox), digits=2), round(Float64(oz), digits=2)), NaN); !isnan(rz) && abs(Float64(oy) - rz) > 0.5), OBJINSTS)
    println("  placement height vs the record: ", ndis, " of ", length(OBJINSTS), " differ by > 0.5 m (JM_OBJ_RECZ=", OBJ_USE_RECZ ? "1, record used" : "0, HAT used", ")")
    hist(v) = join(["$(k)=$(c)" for (k, c) in sort(collect(Dict(k => count(==(k), v) for k in unique(v))); by = kv -> -kv[2])[1:min(end, 12)]], " ")
    println("  on-road names: ", hist([r[1] for r in onr]))
    println("  floating names: ", hist([r[1] for r in flo]))
    println("  --- on-road, by lapdist (first 40) ---")
    for r in sort(onr; by = r -> r[3])[1:min(end, 40)]
        println("   s=", lpad(round(Int, r[3]), 6), "  lat=", lpad(round(r[4], digits = 1), 6), "  dy=", lpad(round(r[5], digits = 1), 5), "  ", r[2], "  ", r[1])
    end
    println("  --- floating, by height (first 30) ---")
    for r in sort(flo; by = r -> -r[5])[1:min(end, 30)]
        println("   s=", lpad(round(Int, r[3]), 6), "  lat=", lpad(round(r[4], digits = 1), 6), "  dy=", lpad(round(r[5], digits = 1), 5), "  ", r[2], "  ", r[1])
    end

    sites = [tryparse(Float64, t) for t in split(get(ENV, "JM_OBJCENSUS", ""), ",")]
    for s0 in sites
        s0 === nothing && continue
        win = parse(Float64, get(ENV, "JM_OBJCENSUS_WIN", "150"))
        println("  --- placements within ", win, " m of s=", s0, " (name kind lat dy yaw extent_x extent_z height) ---")
        near = sort(filter(r -> abs(r[3] - s0) < win, rows); by = r -> r[3])
        for r in near
            k = (round(r[7], digits = 2), round(Float64(first(filter(t -> t[1] == r[1] && Float64(t[2]) == r[7], OBJINSTS))[3]), digits = 2))
            yaw = get(OBJ_YAW, k, NaN)
            vs = get(OBJ_LVERTS, r[1], nothing)
            ex = vs === nothing || isempty(vs) ? "-" : string(round(maximum(getindex.(vs, 1)) - minimum(getindex.(vs, 1)), digits = 1), "x", round(maximum(getindex.(vs, 2)) - minimum(getindex.(vs, 2)), digits = 1))

            latr = "-"
            if vs !== nothing && !isempty(vs) && !isnan(yaw)
                c_, s_ = cos(yaw), sin(yaw); lo = Inf; hi = -Inf
                for (lx, lz) in vs
                    h_ = JRTrack.hat(TRKSURF, r[7] + lx*c_ + lz*s_, k[2] - (-lx*s_ + lz*c_))
                    h_.found || continue; lo = min(lo, h_.lateral); hi = max(hi, h_.lateral)
                end
                latr = isfinite(lo) ? string(round(lo, digits = 1), "..", round(hi, digits = 1)) : "-"
            end
            oy_ = Float64(first(filter(t -> t[1] == r[1] && Float64(t[2]) == r[7], OBJINSTS))[4])
            recz = get(OBJ_RECZ, k, NaN)
            println("   s=", lpad(round(Int, r[3]), 6), "  ", rpad(r[1], 10), " ", r[2], "  lat=", lpad(round(r[4], digits = 1), 6), "  dy=", lpad(round(r[5], digits = 1), 5),
                    "  yaw=", isnan(yaw) ? "  ?" : lpad(round(Int, rad2deg(yaw)), 4), "°  ext=", ex, "  h=", round(Float64(get(OBJ_YMX, r[1], 0f0) - get(OBJ_YMN, r[1], 0f0)), digits = 1), "  drawn lat=", latr,
                    "  ours_y=", round(oy_, digits = 1), " rec_y=", isnan(recz) ? "?" : round(recz, digits = 1), " (ours-rec=", isnan(recz) ? "?" : round(oy_ - recz, digits = 1), ")")
        end
    end

    vn = lowercase(get(ENV, "JM_OBJVERTS", ""))
    if vn != ""
        ninst = 0
        for (nm_, ox_, oz_, oy_, kind_, _) in OBJINSTS
            lowercase(nm_) == vn || continue; ninst += 1; ninst > 2 && break
            pv = get(OBJ_VERTS, nm_, nothing); pv === nothing && (println("  no parts for ", nm_); continue)
            yaw_ = get(OBJ_YAW, (round(Float64(ox_), digits=2), round(Float64(oz_), digits=2)), 0.0)
            M = Render.translate(Float32[ox_, oy_, -oz_]) * Render.roty(Float32(yaw_))
            hr0 = JRTrack.hat(TRKSURF, Float64(ox_), Float64(oz_))
            println("  --- ", nm_, " #", ninst, " origin (", round(Float64(ox_), digits=1), ", ", round(Float64(oz_), digits=1), ") placed y=", round(Float64(oy_), digits=1),
                    " yaw=", round(rad2deg(yaw_), digits=1), "°  s=", hr0.found ? round(hr0.lapdist, digits=0) : -1, " lat=", hr0.found ? round(hr0.lateral, digits=1) : "?")
            println("     world_x   world_z   vert_y  terrain_y   delta    s     lat")
            n = 0
            for pp in pv, k in 1:11:length(pp.verts)
                q = M * Float32[pp.verts[k], pp.verts[k+1], pp.verts[k+2], 1f0]
                px = Float64(q[1]); py = Float64(q[2]); pz = -Float64(q[3])
                g = JRTrack.hat3d(TERRAIN0, px, pz; ref = Inf); hr = JRTrack.hat(TRKSURF, px, pz)
                gl = JRTrack.hat3d(TERRAIN, px, pz; ref = Inf)
                println("     ", lpad(round(px, digits=1), 8), "  ", lpad(round(pz, digits=1), 8), "  ", lpad(round(py, digits=1), 6), "  ",
                        lpad(g[3] ? round(Float64(g[1]), digits=1) : "-", 8), "  ", lpad(g[3] ? round(py - Float64(g[1]), digits=1) : "-", 6), "  land=",
                        lpad(gl[3] ? round(Float64(gl[1]), digits=1) : "-", 7), " d=", lpad(gl[3] ? round(py - Float64(gl[1]), digits=1) : "-", 5), "  ",
                        lpad(hr.found ? round(Int, hr.lapdist) : -1, 5), "  ", lpad(hr.found ? round(hr.lateral, digits=1) : "?", 6))
                n += 1; n >= 40 && break
            end
        end
    end
    println("OBJCENSUS_RESULT track=", TRACKSEL, " onroad=", length(onr), " floating=", length(flo), " buried=", length(bur))
    flush(stdout); exit(0)
end

function dx_holecensus(CLINE)
    halfw   = parse(Float64, get(ENV,"JM_HOLECENSUS","12"))
    step_s  = parse(Float64, get(ENV,"JM_HOLECENSUS_DS","10"))
    latstep = 1.5
    nlat = Int(floor(2*halfw/latstep)) + 1
    println("== JM_HOLECENSUS ", TRACKSEL, ": ground coverage ±", halfw, " m of the centreline, every ", step_s, " m ==")
    total = 0; holes = 0; worst_s = -1.0; worst_n = -1; badstations = 0
    holelist = Tuple{Float64,Int}[]
    sacc = 0.0
    while sacc < CLINE.total
        nhole = 0
        for k in 0:nlat-1
            lat = -halfw + k*latstep

            pp = RaceAI.pose_at(CLINE, sacc, lat)
            h = JRTrack.hat3d(TERRAIN, Float64(pp[1]), Float64(pp[3]); ref=Inf)
            total += 1
            h[3] || (holes += 1; nhole += 1)
        end
        nhole > 0 && (badstations += 1)
        nhole > worst_n && (worst_n = nhole; worst_s = sacc)
        nhole > 0 && push!(holelist, (sacc, nhole))
        sacc += step_s
    end
    println("   samples=", total, "  holes=", holes, " (", round(100*holes/max(total,1), digits=2), "%)",
            "   stations with any hole: ", badstations)
    println("   worst station: s=", round(worst_s, digits=1), " m with ", worst_n, " of ", nlat, " lateral samples missing")

    sort!(holelist, by = x -> -x[2])
    println("   worst ten stations (s → missing/", nlat, "):")
    for (ss, nn) in holelist[1:min(10, end)]
        println("      s=", lpad(round(ss, digits=1), 9), "  ", lpad(nn, 3), " missing")
    end
    flush(stdout)
end

function dx_bridgeprobe(CLINE)
    println("== JM_BRIDGEPROBE: surfaces over the racing line ==")
    for tok in split(get(ENV,"JM_BRIDGEPROBE",""), ",")
        sv = tryparse(Float64, strip(String(tok))); sv === nothing && continue
        pz = RaceAI.pose_at(CLINE, sv, 0.0); qx = Float64(pz[1]); qz = Float64(pz[3])
        hits = Tuple{Float64,String}[]

        for t in TRACKMESH.tris
            ax,az = Float64(t.p[1][1]), Float64(t.p[1][2])
            bx,bz = Float64(t.p[2][1]), Float64(t.p[2][2])
            cx,cz = Float64(t.p[3][1]), Float64(t.p[3][2])
            d = (bz-cz)*(ax-cx) + (cx-bx)*(az-cz); abs(d) < 1e-9 && continue
            wa = ((bz-cz)*(qx-cx) + (cx-bx)*(qz-cz))/d
            wb = ((cz-az)*(qx-cx) + (ax-cx)*(qz-cz))/d
            wc = 1 - wa - wb
            (wa >= -0.02 && wb >= -0.02 && wc >= -0.02) || continue
            h = wa*Float64(t.p[1][3]) + wb*Float64(t.p[2][3]) + wc*Float64(t.p[3][3])
            push!(hits, (h, lowercase(t.tex)))
        end
        sort!(hits, by=first)
        print("   s=", round(Int,sv), " at (", round(qx,digits=1), ",", round(qz,digits=1), "): ")
        if isempty(hits)
            println("NO surface (hole)")
        else
            println(length(hits), " surfaces, low..high:")
            for (h,tx) in hits
                println("      h=", rpad(round(h,digits=2),9), " tex=", tx)
            end
        end
    end
    flush(stdout); exit(0)
end

function dx_roadsweep(CLINE)
    step = (v = tryparse(Float64, get(ENV, "JM_ROADSWEEP", "")); v === nothing || v <= 0 ? 2.0 : v)
    println("\n==== JM_ROADSWEEP ", uppercasefirst(TRACKSEL), "  (step=", step, " m, lateral 0.5 m, ",
            length(SOLIDS), " solids, ", count(!isnothing, SOLIDBOX), " boxed, car=",
            CAR_CIRCLE ? "circle $(CARHALF)" : "capsule $(CARW)x±$(CARLF)", ") ====")
    ctrl = if isempty(SOLIDS); false
           else (g, _, _) = car_gap(SOLIDS[1][1], SOLIDS[1][2], 0.0, 1); g < 0.0 end
    println("  control: a probe at solid #1's centre reads as a hit -> ", ctrl ? "yes" : "NO (instrument blind)")

    tarmac_hat = ROADHAT !== TERRAIN0
    println("  tarmac predicate: ", tarmac_hat ? "road-only HAT (road-textured triangles)" :
            "NONE for this track -- falling back to the 9 m corridor (verge included)")
    nprobe = 0; nroad = 0; ntar = 0
    hits  = Dict{String,Tuple{Int,Float64,Float64,Float64}}()
    verge = Dict{String,Int}()
    edge  = Dict{String,Int}()
    inside = Dict{String,Tuple{Int,Float64}}()
    s = 0.0
    while s <= CLINE.total
        for lat in -14.0:0.5:14.0
            q = RaceAI.pose_at(CLINE, s, lat); qx, qz = q[1], q[3]
            hr = JRTrack.hat(TRKSURF, qx, qz)
            nprobe += 1
            (hr.found && hr.on_track) || continue
            nroad += 1
            ontar = tarmac_hat ? JRTrack.hat3d(ROADHAT, qx, qz; ref = Inf)[3] : true
            ontar && (ntar += 1)

            fits = ontar && (!tarmac_hat || begin
                ql = RaceAI.pose_at(CLINE, s, lat - 0.9); qr = RaceAI.pose_at(CLINE, s, lat + 0.9)
                JRTrack.hat3d(ROADHAT, ql[1], ql[3]; ref = Inf)[3] && JRTrack.hat3d(ROADHAT, qr[1], qr[3]; ref = Inf)[3]
            end)
            @inbounds for k in eachindex(SOLIDS)
                (ox, oz, r, _) = SOLIDS[k]
                hypot(ox - qx, oz - qz) > r + 40.0 && continue
                (gapc, _, _) = solid_gap(qx, qz, k)
                (gap, _, _)  = car_gap(qx, qz, q[4], k)
                gap >= 0.0 && continue
                ontar = fits
                nm = k <= length(SOLIDNAMES) ? SOLIDNAMES[k] : "solid#$k"
                key = string(nm, "@", round(Int, ox), ",", round(Int, oz))
                bk = k <= length(SOLIDBOX) ? SOLIDBOX[k] : nothing
                thin = bk !== nothing && min(bk[1], bk[2]) < 0.5
                if ontar && thin
                    edge[key] = get(edge, key, 0) + 1
                elseif ontar
                    old = get(hits, key, (0, Inf, 0.0, 0.0))
                    hits[key] = (old[1] + 1, min(old[2], gap), gap < old[2] ? s : old[3], gap < old[2] ? lat : old[4])

                    gapc < 0.0 && (oc = get(inside, key, (0, Inf)); inside[key] = (oc[1] + 1, min(oc[2], gapc)))
                else
                    verge[key] = get(verge, key, 0) + 1
                end
            end
        end
        s += step
    end
    println("  ", nprobe, " probes, ", nroad, " in the 9 m corridor, ", ntar, " on tarmac; ",
            length(hits), " solid(s) reachable with the whole car on tarmac (", length(verge), " more from the verge/edge only):")
    for (key, (n, g, hs, hl)) in sort(collect(hits); by = kv -> kv[2][2])
        println("    ", rpad(key, 28), " hits=", lpad(n, 4), "  worst gap=", lpad(round(g, digits = 2), 6),
                " m  at s=", round(Int, hs), " lat=", hl)
    end
    println("  edge barriers (thin, scrape-able, not counted): ", length(edge))
    ninside = length(inside)
    println("  objects standing ON the road (car centre inside, ", ninside, "):")
    for (key, (n, g)) in sort(collect(inside); by = kv -> kv[2][2])
        println("    ", rpad(key, 28), " hits=", lpad(n, 4), "  centre gap=", round(g, digits = 2), " m")
    end
    println("ROADSWEEP_RESULT track=", TRACKSEL, " solids_on_road=", length(hits), " inside=", ninside, " edge_barriers=", length(edge), " verge_only=", length(verge),
            " tarmac_pred=", tarmac_hat ? "roadhat" : "corridor", " control=", ctrl ? "ok" : "blind")
    flush(stdout); exit(0)
end

function dx_pacediag(AILINE)
    println("\n==== JM_PACEDIAG  $(TRACKSEL)  (lap $(round(AILINE.total, digits=0)) m) ====")
    base = RaceAI.natural_laptime(AILINE)
    println("  default (amax=11.0, vmax=74.0):  ", round(base, digits=1), " s")
    for vm in (74.0, 80.0, 85.0, 90.0, 100.0)
        t = RaceAI.natural_laptime(AILINE; vmax = vm)
        println("    vmax=", lpad(round(Int,vm),3), " m/s (", lpad(round(Int,vm*3.6),3), " km/h): ",
                lpad(round(t, digits=1), 6), " s   (", lpad(round(100*(base-t)/base, digits=1), 5), " % faster)")
    end
    for am in (11.0, 14.0, 18.0)
        t = RaceAI.natural_laptime(AILINE; amax = am)
        println("    amax=", lpad(round(Int,am),3), " m/s2            : ",
                lpad(round(t, digits=1), 6), " s   (", lpad(round(100*(base-t)/base, digits=1), 5), " % faster)")
    end
    t_both = RaceAI.natural_laptime(AILINE; vmax = 90.0, amax = 14.0)
    println("    vmax=90 + amax=14            : ", round(t_both, digits=1), " s   (",
            round(100*(base-t_both)/base, digits=1), " % faster)")
    println("  target (REF_LAP) = ", round(AI_REFLAP, digits=1), " s;  gold .rpy fastest is quicker still")

    let n = 600, amax = 11.0, vmax = 74.0
        ks = Float64[]; vloc = Float64[]; vhor = Float64[]
        for i in 0:n-1
            sdist = AILINE.total * i / n
            kloc = max(AILINE.κ[RaceAI._locate(AILINE, sdist)[1]], 1e-4)
            push!(ks, kloc); push!(vloc, clamp(sqrt(amax/kloc), 12.0, vmax))
            kh = kloc; off = 5.0
            while off <= 150.0
                kh = max(kh, AILINE.κ[RaceAI._locate(AILINE, sdist+off)[1]]); off += 6.0
            end
            push!(vhor, clamp(sqrt(amax/kh), 12.0, vmax))
        end
        q(v,p) = sort(v)[clamp(round(Int, p*length(v)), 1, length(v))]

        let X = AILINE.x, Z = AILINE.z, nn = length(AILINE.x)
            dss = [hypot(X[mod1(i+1,nn)]-X[i], Z[mod1(i+1,nn)]-Z[i]) for i in 1:nn]
            qq(v,p) = sort(v)[clamp(round(Int, p*length(v)), 1, length(v))]
            ordk = sortperm(AILINE.κ, by=abs, rev=true)[1:min(20,nn)]
            println("\n  ---- E84-S5: centreline NODE SPACING (", nn, " nodes) ----")
            println("    ds m: p01 ", round(qq(dss,0.01),digits=3), "  p10 ", round(qq(dss,0.10),digits=3),
                    "  p50 ", round(qq(dss,0.50),digits=3), "  min ", round(minimum(dss),digits=4))
            println("    ds at the 20 highest-|kappa| nodes: median ",
                    round(sort([dss[i] for i in ordk])[10], digits=3), " m   min ",
                    round(minimum(dss[i] for i in ordk), digits=4), " m")
        end
        dvl = [abs(vloc[i+1]-vloc[i]) for i in 1:length(vloc)-1]
        dvh = [abs(vhor[i+1]-vhor[i]) for i in 1:length(vhor)-1]
        spacing = AILINE.total / n
        println("\n  ---- E89: vtarget STEP-TO-STEP change (sample spacing ", round(spacing,digits=1), " m) ----")
        println("    |dv| local  m/s: p50 ", round(q(dvl,0.50),digits=3), "  p90 ", round(q(dvl,0.90),digits=3),
                "  max ", round(maximum(dvl),digits=2), "   >10 m/s: ", count(>(10.0), dvl), "/", length(dvl))
        println("    |dv| horizon m/s: p50 ", round(q(dvh,0.50),digits=3), "  p90 ", round(q(dvh,0.90),digits=3),
                "  max ", round(maximum(dvh),digits=2), "   >10 m/s: ", count(>(10.0), dvh), "/", length(dvh))
        println("\n  ---- kappa / vtarget profile (", n, " samples round the lap) ----")
        println("    kappa  1/m : p10 ", round(q(ks,0.10),digits=5), "  p50 ", round(q(ks,0.50),digits=5),
                "  p90 ", round(q(ks,0.90),digits=5), "  max ", round(maximum(ks),digits=5))
        println("    radius   m : p10 ", round(Int,1/q(ks,0.90)), "  p50 ", round(Int,1/q(ks,0.50)),
                "  p90 ", round(Int,1/q(ks,0.10)), "   (p10 radius = the TIGHTEST decile)")
        println("    vtarget LOCAL  kappa only : p10 ", round(q(vloc,0.10),digits=1),
                "  p50 ", round(q(vloc,0.50),digits=1), "  p90 ", round(q(vloc,0.90),digits=1),
                "   at vmax: ", round(100*count(>=(vmax-0.1), vloc)/n, digits=1), " %")
        println("    vtarget HORIZON max (150m): p10 ", round(q(vhor,0.10),digits=1),
                "  p50 ", round(q(vhor,0.50),digits=1), "  p90 ", round(q(vhor,0.90),digits=1),
                "   at vmax: ", round(100*count(>=(vmax-0.1), vhor)/n, digits=1), " %")
        println("    => median speed cost of the horizon rule: ",
                round(q(vloc,0.50)-q(vhor,0.50), digits=1), " m/s")
        println("    (a clean 1967 Monza line should be near-straight over most of the lap:",
                " median radius >> 500 m, median vtarget at vmax)")

        for rlim in (80.0, 40.0, 20.0, 10.0)
            klim = 1/rlim; c = count(>=(klim), ks)
            println("    radius < ", lpad(round(Int,rlim),3), " m: ", lpad(c,4), " / ", n,
                    " samples (", lpad(round(100*c/n, digits=1),5), " %)  -> vtarget ",
                    round(clamp(sqrt(11.0/klim),12.0,74.0), digits=1), " m/s")
        end
        worst = sortperm(ks, rev=true)[1:min(8,length(ks))]
        println("    worst spikes (lap distance m -> radius m):")
        for w in worst
            println("      s=", lpad(round(Int, AILINE.total*(w-1)/n),5), "  r=",
                    round(1/ks[w], digits=1), " m")
        end
    end
    exit(0)
end

function check_diag_globals(fs = (dx_roadtris, dx_roadedge, dx_objcensus, dx_holecensus, dx_bridgeprobe, dx_roadsweep, dx_pacediag,
                                  frameprof_report!, fpsdiag_report!, allocprof_report!, mirror_render!, texthud_draw!,
                                  gc_step!, pace_step!, wreck_notice_draw!, replay_tyres_draw!, countdown_step!, replay_record!, ai_cooldown_step!, shots_step!, stall_log!, fuel_step!, joytrace_report!))
    bad = String[]
    for f in fs, ci in code_lowered(f)
        walk(x) = x isa GlobalRef ? (isdefined(x.mod, x.name) || push!(bad, "$(nameof(f)): $(x.name)")) :
                  x isa Expr ? foreach(walk, x.args) : nothing
        foreach(walk, ci.code)
    end
    println("  [check_diag] ", isempty(bad) ? "every global the moved diagnostics name exists" : "UNDEFINED: " * join(unique(bad), ", "))
    isempty(bad)
end

"""FRAMEPROF: the per-pass frame-time breakdown, every FRAMEPROF frames. (LOOPSPLIT-1 S4: moved unchanged from main())"""
function frameprof_report!()
        if FRAMEPROF > 0
            PROF_N[] += 1
            if PROF_N[] >= FRAMEPROF
                nn = PROF_N[]

                tot = PROF_TOT[] > 0 ? PROF_TOT[]/nn*1000 : NaN
                println("  [frameprof] ", nn, " frames:  world ", round(1000*PROF_WORLD[]/nn, digits=2),
                        " ms   hud ", round(1000*PROF_HUD[]/nn, digits=2), " ms",
                        "   [depth ", round(1000*PROF_DEPTH[]/nn, digits=2), " track ", round(1000*PROF_TRACK[]/nn, digits=2),
                        " objects ", round(1000*PROF_OBJ[]/nn, digits=2), " billboards ", round(1000*PROF_BB[]/nn, digits=2),
                        " cars ", round(1000*PROF_CARS[]/nn, digits=2), "]",
                        isnan(tot) ? "   (total not measured -- set JM_FPSDIAG too)" :
                                     "   of " * string(round(tot, digits=2)) * " ms total")
                flush(stdout)
                PROF_WORLD[] = 0.0; PROF_HUD[] = 0.0; PROF_N[] = 0; PROF_TOT[] = 0.0
                PROF_DEPTH[] = 0.0; PROF_TRACK[] = 0.0; PROF_OBJ[] = 0.0; PROF_BB[] = 0.0; PROF_CARS[] = 0.0
            end
        end
    nothing
end

"""FPSDIAG: frame time per view, every FPSDIAG frames (E80). (LOOPSPLIT-1 S4: moved unchanged from main())"""
function fpsdiag_report!(cs)
        if FPSDIAG > 0
            _now = time()
            if FPS_T0[] > 0
                FPS_ACC[] += _now - FPS_T0[]; FPS_N[] += 1; FRAMEPROF > 0 && (PROF_TOT[] += _now - FPS_T0[])
                push!(FPS_DTS, _now - FPS_T0[])
                if FPS_N[] >= FPSDIAG
                    _ms = 1000*FPS_ACC[]/FPS_N[]

                    _hr = JRTrack.hat(TRKSURF, cs.x, cs.z)
                    _nobj = count(o -> (o[4][1] - cs.x)^2 + (-o[4][3] - cs.z)^2 < OBJ_CULL2, OBJECTS)
                    println("  [fps] view=", CTL.view == 0 ? "cockpit" : "chase  ",
                            "  ", round(1000/_ms, digits=1), " fps  (", round(_ms, digits=1), " ms/frame)",
                            "  s=", _hr.found ? round(Int, _hr.lapdist) : -1, "  objs_in_range=", _nobj, "/", length(OBJECTS),
                            "  billboards=", length(BILLBOARDS), "  trees=", length(STATICTREES))
                    if !isempty(FPS_DTS)
                        _d = sort(FPS_DTS) .* 1000
                        println("  [pace] frame interval ms: p10 ", round(_d[max(1, length(_d)÷10)], digits=1), "  p50 ", round(_d[max(1, length(_d)÷2)], digits=1),
                                "  p90 ", round(_d[max(1, 9*length(_d)÷10)], digits=1), "  max ", round(_d[end], digits=1), "  >25ms ", count(>(25.0), _d), "/", length(_d), "  (interval ", PACE_INT[], ")")
                        empty!(FPS_DTS)
                    end
                    flush(stdout); FPS_ACC[] = 0.0; FPS_N[] = 0
                end
            end
            FPS_T0[] = _now
        end
    nothing
end

"""ALLOCPROF: the per-pass allocation tally, printed every ALLOCPROF frames. (LOOPSPLIT-1 S4: moved unchanged from main())"""
function allocprof_report!()
        if ALLOCPROF > 0
            ALLOC_MARK[5] = Base.gc_bytes()
            for k in 1:4; ALLOC_ACC[k] += ALLOC_MARK[k+1] - ALLOC_MARK[k]; end
            ALLOC_N[] += 1
            if ALLOC_N[] >= ALLOCPROF
                println("  [alloc] KB/frame: sim ", round(Int, ALLOC_ACC[1]/ALLOC_N[]/1024), "  shadow+mirror ", round(Int, ALLOC_ACC[2]/ALLOC_N[]/1024),
                        "  world ", round(Int, ALLOC_ACC[3]/ALLOC_N[]/1024), "  hud ", round(Int, ALLOC_ACC[4]/ALLOC_N[]/1024)); flush(stdout)
                fill!(ALLOC_ACC, 0); ALLOC_N[] = 0
            end
        end
    nothing
end

"""E59 multi-shot (JM_SHOTS): dump the settled frame for the current shot, then teleport to the next. (LOOPSPLIT-1 cycle 2 S3: moved unchanged from main())"""
function shots_step!(frames, vp, eye, W, H, place_at_s!, shot_idx, shot_t0, shots_done)
        if SMOKE && !isempty(SHOTS) && !shots_done[] && frames - shot_t0[] == SHOT_SETTLE
            sh = SHOTS[shot_idx[]]
            buf=Vector{UInt8}(undef,W*H*3); glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE,buf)
            open(joinpath(SHOTS_DIR, sh.name * ".ppm"),"w") do io; write(io,"P6\n$W $H\n255\n")
                for y in H:-1:1, x in 1:W; o=((y-1)*W+(x-1))*3; write(io,buf[o+1],buf[o+2],buf[o+3]); end; end
            println("  JM_SHOTS: dumped ", sh.name, " (", shot_idx[], "/", length(SHOTS), ")"); flush(stdout)
            isempty(PICKS) || pick_report(vp, eye, sh.name)
            HUDBAND_ON && bandmir_pick_report(sh.name)
            if shot_idx[] < length(SHOTS)
                shot_idx[] += 1; shot_t0[] = frames + 1
                nxt = SHOTS[shot_idx[]]; CTL.view = nxt.view
                for (k, v) in nxt.env; ENV[k] = v; end
                place_at_s!(nxt.s, parse(Float64, get(ENV, "JM_SHOT_LAT", "0")))
            else
                shots_done[] = true
            end
        end
    nothing
end

"""PERF-4: name every stall over STALL_MS (compile / GC / other) and keep FRAME_FPS. (LOOPSPLIT-1 cycle 2 S3: moved unchanged from main())"""
function stall_log!(cs, frames, inp, win_shown)
        if STALLLOG
            _now = time_ns(); _ct = UInt64(Base.cumulative_compile_time_ns()[1]); _gt = UInt64(Base.gc_time_ns())
            if STALL_PREV[1] > 0
                _wall = (_now - STALL_PREV[1]) / 1e6
                FRAME_FPS[] = FRAME_FPS[] == 0.0 ? 1000 / _wall : FRAME_FPS[] + (1000 / _wall - FRAME_FPS[]) * 0.1
                if _wall > STALL_MS && win_shown[]
                    _cm = (_ct - STALL_PREV[2]) / 1e6; _gm = (_gt - STALL_PREV[3]) / 1e6
                    STALL_N[] += 1; STALL_COMP_MS[] += _cm
                    println("  [stall] frame ", frames, " t=", round(cs.t, digits = 2), "s  ", round(_wall, digits = 1),
                            " ms (compile ", round(_cm, digits = 1), ", gc ", round(_gm, digits = 1), ")  gear ", cs.gear,
                            " thr ", round(inp.throttle, digits = 2), " clu ", round(inp.clutch, digits = 2),
                            " brk ", round(inp.brake, digits = 2), " v ", round(cs.v, digits = 1)); flush(stdout)
                end
            end
            STALL_PREV[1] = _now; STALL_PREV[2] = _ct; STALL_PREV[3] = _gt
        end
    nothing
end

"""JM_JOYTRACE: what the car is asked to do, once a second (PO 2026-08-27c). (LOOPSPLIT-1 cycle 2 S4: moved unchanged.)"""
function joytrace_report!(cs, inp, race_go)
        if _DX_JOYTRACE && cs.t - JOYTRACE_T[] >= 1.0
            JOYTRACE_T[] = cs.t
            _js = GLFW.GetJoystickAxes(GLFW.JOYSTICK_1)
            println("  [trace] thr=", round(inp.throttle,digits=2), " brk=", round(inp.brake,digits=2),
                    " clu=", round(inp.clutch,digits=2), " steer=", round(inp.steer,digits=2),
                    " | gear=", cs.gear, " rpm=", round(Int,cs.rpm), " v=", round(cs.v*3.6,digits=1), "km/h",
                    " | raw=", _js === nothing ? "nil" : join(round.(_js,digits=2), ","),
                    " | auto=", CTL.auto, " race_go=", race_go[])
            flush(stdout)
        end
    nothing
end

"""JOLT_MS (SPATD-1): name every impulse (LOOPSPLIT-1 cycle 3 S5: moved unchanged from the frame loop)."""
function jolt_report!(cs, frames, CLINE, STEP_GUARD_HITS, dt, rst)
    if JOLT_MS > 0 && CAR3D && !SKIDPAD && !rst && JOLT_N[] < 60

        (_jvx, _jvz) = WBODY[] === nothing ? DriveRT3D.world_velocity(cs) : (WVX[], WVZ[]); _jp = JOLT_PRE[]
        _jd = hypot(_jvx - _jp[1], _jvz - _jp[2])
        if _jd > JOLT_MS
            JOLT_N[] += 1
            _js = CLINE === nothing ? NaN : RaceAI.project(CLINE, cs.x, cs.z)
            println("  [jolt] t=", round(cs.t, digits = 2), " s=", round(_js[1], digits = 1), " lat=", round(_js[2], digits = 2),
                    " v=", round(cs.v, digits = 1), "  dv ", round(_jd, digits = 2), " m/s in ", round(dt*1000, digits = 1),
                    " ms  | solids F=(", round(_jp[3], digits = 0), ",", round(_jp[4], digits = 0), ") pk ", round(_jp[5], digits = 0),
                    "  wall hits +", CLASSICW_STAT[].hits - _jp[6], " clamps +", CLASSICW_STAT[].clamps - _jp[7],
                    "  obstacle hits +", OBS_STAT[].hits - _jp[8], " clamps +", OBS_STAT[].clamps - _jp[9],
                    "  step-guard +", STEP_GUARD_HITS[] - _jp[10], "  rh (", round(cs.rh[1], digits = 3), ", ", round(cs.rh[2], digits = 3),
                    ", ", round(cs.rh[3], digits = 3), ", ", round(cs.rh[4], digits = 3), ")")

            _jg = Inf; _jk = 0
            for _k in eachindex(SOLIDS)
                SOLIDS[_k][3] < -100 && continue
                _g = car_gap(cs.x, cs.z, cs.θ, _k)[1]; _g < _jg && (_jg = _g; _jk = _k)
            end
            println("         at (", round(cs.x, digits = 1), ", ", round(cs.z, digits = 1), ")  deepest solid: ",
                    _jk == 0 ? "none" : string(_jk <= length(SOLIDNAMES) ? SOLIDNAMES[_jk] : "?", " kind=", SOLIDS[_jk][4],
                    " r=", round(SOLIDS[_jk][3], digits = 1), (_jk <= length(SOLIDBOX) && SOLIDBOX[_jk] !== nothing) ? " (box)" : "",
                    " at (", round(SOLIDS[_jk][1], digits = 1), ", ", round(SOLIDS[_jk][2], digits = 1), ") gap ", round(_jg, digits = 2), " m"))
            if CLASSICW_PHYS
                (_sec, _f, _lat, _, _) = classicw_locate(cs.x, cs.z; update = false)
                (_lo, _hi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, _sec, _f, _lat, _lat; hmin = CLASSICW_HMIN)
                _L = CLASSICWALLS.seclen[_sec]
                println("         Classic: section ", _sec, " at ", round(_f*_L, digits = 1), "/", round(_L, digits = 1), " m, lat ",
                        round(_lat, digits = 2), ", free [", round(_lo, digits = 2), ", ", round(_hi, digits = 2), "]")
            end
            flush(stdout)
        end
    end
    nothing
end

"""_DX_SHOTS: the diagnostic shot sequence (LOOPSPLIT-1 cycle 3 S5: moved unchanged from the frame loop)."""
function dx_shots!(cs, frames, REPLAY, eye, race_done, rep_rt, vp)
    if _DX_SHOTS
        _tnow = REPLAY ? rep_rt[] : cs.t
        for (k, ts) in enumerate(SHOTS_AT)
            SHOTS_AT_ARMED[k] == -2 && continue
            if SHOTS_AT_ARMED[k] < 0
                fire = startswith(ts.when, "finish+") ? (race_done && RACEEND_T0[] >= 0 && cs.t - RACEEND_T0[] >= parse(Float64, ts.when[8:end])) :
                                                      (_tnow >= parse(Float64, ts.when))
                if fire
                    SHOTS_AT_ARMED[k] = frames; ts.view >= 0 && (CTL.view = ts.view)
                    println("  JM_SHOT_AT: armed ", ts.name, " at t=", round(_tnow, digits=2)); flush(stdout)
                end
            elseif frames - SHOTS_AT_ARMED[k] >= SHOT_SETTLE
                buf=Vector{UInt8}(undef,W*H*3); glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE,buf)
                open(joinpath(SHOTS_DIR, ts.name * ".ppm"),"w") do io; write(io,"P6\n$W $H\n255\n")
                    for y in H:-1:1, x in 1:W; o=((y-1)*W+(x-1))*3; write(io,buf[o+1],buf[o+2],buf[o+3]); end; end
                println("  JM_SHOT_AT: dumped ", ts.name); flush(stdout)
                isempty(PICKS) || pick_report(vp, eye, ts.name)
                HUDBAND_ON && bandmir_pick_report(ts.name)
                SHOTS_AT_ARMED[k] = -2
                all(==(-2), SHOTS_AT_ARMED) && haskey(ENV, "JM_SHOT_AT_EXIT") && break
            end
        end
    end
    nothing
end

"""ALLOCSITES: the frame's allocation sites (LOOPSPLIT-1 cycle 3 S5: moved unchanged from the frame loop)."""
function allocsites_report!(cs, frames)
    if ALLOCSITES
        if frames == 300
            @eval Main using Profile; Base.invokelatest(() -> (Profile.Allocs.clear(); Profile.Allocs.start(sample_rate = 0.02)))
        elseif frames == 600
            Base.invokelatest() do
                Profile.Allocs.stop(); r = Profile.Allocs.fetch()
                cnt = Dict{String,Tuple{Int,Int}}()
                for a in r.allocs
                    site = "?"
                    for fr in a.stacktrace
                        f = string(fr.file)
                        (occursin("drive_native_mtk", f) || occursin("main_loop", f) || occursin("render.jl", f) || occursin("ai.jl", f) || occursin("drive_rt", f)) || continue
                        site = string(basename(f), ":", fr.line, " ", fr.func); break
                    end
                    c = get(cnt, site, (0, 0)); cnt[site] = (c[1] + 1, c[2] + a.size)
                end
                println("  [allocsites] top sites (sampled 2%, frames 300-600):")
                for (k, v) in sort(collect(cnt), by = x -> -x[2][2])[1:min(end, 25)]
                    println("     ", lpad(round(Int, v[2]/0.02/300/1024), 6), " KB/frame  ", lpad(v[1], 6), " samples  ", k)
                end
                flush(stdout)
            end
        end
    end
    nothing
end

"""JM_AIPOSE_TRACE=<i>|all: AI car i's drawn pose every frame (LOOPSPLIT-1 cycle 5 S3: moved unchanged from the frame loop)."""
function aipose_trace(cs, ai_poses, AICARS)
    if AIPOSE_TRACE != 0
        for k in (AIPOSE_TRACE > 0 ? (AIPOSE_TRACE:AIPOSE_TRACE) : (1:min(length(ai_poses), length(AICARS))))
            (k <= length(ai_poses) && k <= length(AICARS)) || continue
            p = ai_poses[k]; c = AICARS[k]
            println("   [aipose] ", round(cs.t, digits = 4), " ", round(c.s, digits = 3), " ", round(c.lane, digits = 4), " ",
                    join((round(Float64(p[j]), digits = 4) for j in 1:6), " "), " ", k)
        end
    end
    nothing
end

"""PERF-4 S3 GPU queue fence ([gpuq]) + PERF-2 per-pass draw stats ([drawstats]), after the swap (LOOPSPLIT-1 cycle 5 S4: moved unchanged from the frame loop)."""
function frame_stats_step!()
    if GPUQ > 0
        _tq = time()
        if GPUQ_FENCE[] != C_NULL
            glClientWaitSync(GPUQ_FENCE[], GL_SYNC_FLUSH_COMMANDS_BIT, UInt64(100_000_000)); glDeleteSync(GPUQ_FENCE[])
        end
        GPUQ_FENCE[] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
        _w = time() - _tq; GPUQ_WAIT[1] += _w; GPUQ_WAIT[2] += 1; GPUQ_WAIT[3] = max(GPUQ_WAIT[3], _w)
        if GPUQ_DIAG > 0 && GPUQ_WAIT[2] >= GPUQ_DIAG
            println("  [gpuq] fence wait mean ", round(1000GPUQ_WAIT[1]/GPUQ_WAIT[2], digits = 2), " ms  max ",
                    round(1000GPUQ_WAIT[3], digits = 1), " ms  fps ", round(FRAME_FPS[], digits = 1)); flush(stdout)
            fill!(GPUQ_WAIT, 0.0)
        end
    end
    if DSTAT
        DS_T[6] = time()
        for k in 1:5; DS_ACC[k] += DS_T[k+1] - DS_T[k]; end
        for k in 1:6; DS_NA[k] += DS_N[k]; end
        DS_CNT[] += 1
        if DS_CNT[] >= 240
            c = DS_CNT[]; f(x) = round(1000x/c, digits=2)
            println("  [drawstats] FRAME ", f(DS_LOOP[]), " ms  = SIM ", f(DS_SIMACC[1]), " (player phys ", f(DS_SIMACC[2]), ", AI phys ", f(DS_SIMACC[3]), ")  shadow ", f(DS_ACC[1]), "  mirror ", f(DS_ACC[2]), "  main ", f(DS_ACC[3]), "  hud ", f(DS_ACC[4]), "  swap ", f(DS_ACC[5]),
                    "   draws: shadow ", DS_NA[1]÷c, " mirror ", (DS_NA[2]-DS_NA[1])÷c, " main ", (DS_NA[3]-DS_NA[2])÷c,
                    "   ktris: shadow ", DS_NA[4]÷c÷1000, " mirror ", (DS_NA[5]-DS_NA[4])÷c÷1000, " main ", (DS_NA[6]-DS_NA[5])÷c÷1000, GPUSYNC ? "  (GPU-synced)" : "")
            flush(stdout); fill!(DS_ACC, 0.0); fill!(DS_NA, 0); fill!(DS_SIMACC, 0.0); DS_LOOP[] = 0.0; DS_CNT[] = 0
        end
    end
    nothing
end

"""JM_FRAMEDUMP consecutive frames + the smoke self-test frame (frame 38) (LOOPSPLIT-1 cycle 5 S4: moved unchanged from the frame loop)."""
function frame_dumps!(frames)
    if !isempty(FRAMEDUMP) && FRAMEDUMP[1] <= frames < FRAMEDUMP[1] + FRAMEDUMP[2]
        buf=Vector{UInt8}(undef,W*H*3); glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE,buf)
        fn = joinpath(FRAMEDUMP_DIR, "jmfd_" * lpad(frames, 5, '0') * ".ppm")
        open(fn,"w") do io; write(io,"P6\n$W $H\n255\n")
            for y in H:-1:1, x in 1:W; o=((y-1)*W+(x-1))*3; write(io,buf[o+1],buf[o+2],buf[o+3]); end; end
        frames == FRAMEDUMP[1] + FRAMEDUMP[2] - 1 && (println("  JM_FRAMEDUMP: ", FRAMEDUMP[2],
            " consecutive frames -> ", FRAMEDUMP_DIR); flush(stdout))
    end
    if SMOKE && frames == 38 && isempty(SHOTS)
        buf=Vector{UInt8}(undef,W*H*3); glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE,buf)
        open(get(ENV,"JM_DUMP","/tmp/zand_hud.ppm"),"w") do io; write(io,"P6\n$W $H\n255\n")
            for y in H:-1:1, x in 1:W; o=((y-1)*W+(x-1))*3; write(io,buf[o+1],buf[o+2],buf[o+3]); end; end
    end
    nothing
end

"""JM_AICAM_SEQ: the smoke run's AI-camera sequence and its shots (LOOPSPLIT-1 cycle 5 S4: moved unchanged from the frame loop)."""
function aicam_seq_step!(frames)
    if SMOKE && !isempty(AICAM_SEQ) && frames >= 40
        seg = (frames - 40) ÷ AICAM_SEG + 1
        if seg <= length(AICAM_SEQ)
            (AICAM_R[], AIVIEW_R[]) = AICAM_SEQ[seg]
            if (frames - 40) % AICAM_SEG == AICAM_SEG - 1
                buf=Vector{UInt8}(undef,W*H*3); glReadPixels(0,0,W,H,GL_RGB,GL_UNSIGNED_BYTE,buf)
                open(joinpath(SHOTS_DIR, "aiseq_$(seg)_$(AICAM_R[])_$(AIVIEW_R[]).ppm"),"w") do io; write(io,"P6\n$W $H\n255\n")
                    for y in H:-1:1, x in 1:W; o=((y-1)*W+(x-1))*3; write(io,buf[o+1],buf[o+2],buf[o+3]); end; end
            end
        end
    end
    nothing
end

"""SPA-FPS-1 S6: the frame-time EMA the adaptive mirror reads, + the JM_FPSDIAG [mirror] report (LOOPSPLIT-1 cycle 5 S4: moved unchanged from the frame loop)."""
function mirror_ema_step!(frames)
    let _nowt = time()
        if MIRROR_T0[] > 0
            dtf = _nowt - MIRROR_T0[]
            MIRROR_EMA[] = MIRROR_EMA[] == 0.0 ? dtf : (0.9 * MIRROR_EMA[] + 0.1 * dtf)
        end
        MIRROR_T0[] = _nowt
    end

    if FPSDIAG > 0 && frames % FPSDIAG == 0
        println("  [mirror] adapt=", MIRROR_ADAPT ? 1 : 0,
                "  frame EMA ", round(MIRROR_EMA[]*1000, digits=1), " ms   marks ", MIRROR_RESUME_MS, "/", MIRROR_ADAPT_MS, " ms   -> every ",
                (MIRROR_ADAPT && MIRROR_STARVED[] ? MIRROR_ADAPT_N : MIRROR_EVERY),
                " frame(s);  mirror renders skipped ", MIRROR_SKIPPED[]); flush(stdout)
    end
    nothing
end

"""The end-of-run reports: JM_AI_OFFROAD's census of AI car-frames off the drawn road, JM_DRIVECHECK's driveability
verdict (LOOPSPLIT-1 cycle 5 S5: moved unchanged from main(), after the frame loop)."""
function end_reports()
    if AI_OFFROAD_DIAG && AI_OFF[2] > 0
        println("  [ai offroad] ", AI_OFF[1], " of ", AI_OFF[2], " AI car-frames had the car centre off the drawn road (",
                round(100*AI_OFF[1]/AI_OFF[2], digits = 2), " %); worst 100 m bins (s: frames): ",
                join(["$(100k)-$(100k+100): $(v)" for (k, v) in sort(collect(AI_OFFBIN), by = x -> -x[2])[1:min(end, 8)]], ", "))
        AI_OFF_POS === nothing || println("  [ai offroad] CONTROL: the cars moved ", AI_OFF_POS, " m sideways: ", AI_OFF[4], " of ", AI_OFF[2],
                                          " car-frames with an edge off the road (", round(100*AI_OFF[4]/AI_OFF[2], digits = 2), " %)")
        println("  [ai offroad] ", AI_OFF[3], " car-frames with a car EDGE (centre ± 0.95 m) off the drawn road (",
                round(100*AI_OFF[3]/AI_OFF[2], digits = 2), " %); worst 100 m bins: ",
                join(["$(100k)-$(100k+100): $(v)" for (k, v) in sort(collect(AI_EDGEBIN), by = x -> -x[2])[1:min(end, 8)]], ", "))
    end
    if DRIVECHECK
        d = DC[]
        println("\n  ══ DRIVEABILITY (", TRACKSEL, ") — autodrive reached s=", round(d.maxs, digits=1), " m ══")
        println("    max height above ground : ", round(d.airmax, digits=2), " m at s=", round(d.air_s, digits=1),
                "   (", d.air_bad, " frames over ", DC_AIR_MAX, " m)")
        println("    max upward velocity     : ", round(d.vzmax, digits=2), " m/s at s=", round(d.vz_s, digits=1),
                "   (", d.vz_bad, " frames over ", DC_VZ_MAX, " m/s)")
        println("    longest slow spell      : ", round(d.stuckmax, digits=1), " s at s=", round(d.stuck_s, digits=1),
                "   (stuck threshold ", DC_STUCK_S, " s under ", DC_STUCK_V, " m/s)")

        println("    lowest groundz seen     : ", round(d.ghmin, digits=2),
                d.ghmin > -900 ? "   (the -999 off-HAT sentinel NEVER reached the physics)" : "   (sentinel seen)")
        println("    frames OFF the mesh     : ", d.off_bad,
                d.off_bad == 0 ? "   (the car never left the ground mesh)" :
                string("   highest y=", round(d.off_y, digits=2), " m at s=", round(d.off_s, digits=1),
                       "  — height there is measured against a STALE held reference, not real ground"))
        if !isempty(d.offevents)
            println("    off-mesh sites (s, y, km/h):")
            for (es, ey, ekmh) in d.offevents
                println("      s=", round(es, digits=1), "  y=", round(ey, digits=2), " m  ",
                        round(ekmh, digits=1), " km/h")
            end
        end
        if !isempty(d.events)
            println("    launch SITES (each distinct excursion above ", DC_AIR_MAX, " m; height = that excursion's PEAK):")
            for (es, eair, ekmh, ex, ez) in d.events
                println("      s=", lpad(round(es, digits=1), 9), " m   air ", lpad(round(eair, digits=2), 6),
                        " m   at ", lpad(round(ekmh, digits=0), 4), " km/h   world (",
                        round(ex, digits=1), ", ", round(ez, digits=1), ")")
            end
        end
        bad = (d.airmax > DC_AIR_MAX) + (d.vzmax > DC_VZ_MAX) + (d.stuckmax > DC_STUCK_S)
        println(bad == 0 ? "    VERDICT: CLEAN — no levitation, no bounce, never stuck" :
                           "    VERDICT: $(bad) FAILURE MODE(S) SEEN — see the lines above")
        flush(stdout)
    end
    nothing
end

function wheelgap_probe(cs, ai_poses)

    wgap(y, x, z) = (h = JRTrack.hat3d(TERRAIN, x, z; ref=Inf);
                     h[3] ? round(y - Float64(h[1]), digits=3) : NaN)
    println("  [wheelgap] t=", round(cs.t, digits=1), "  player ", wgap(cs.y, cs.x, cs.z), " m",
            isempty(ai_poses) ? "   (no AI on track yet)" : "")
    for (k, p) in enumerate(ai_poses)
        println("  [wheelgap]      AI ", k, "  ", wgap(p[2], p[1], p[3]), " m")
    end
    flush(stdout)
    nothing
end
