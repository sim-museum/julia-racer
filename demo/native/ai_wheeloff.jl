

const AI_WHEELOFF_ON = get(ENV, "JM_AI_WHEELOFF", "1") != "0"
const AI_WHEELOFF_CLOSE = parse(Float64, get(ENV, "JM_AI_WHEELOFF_CLOSE", string(1.5*WRECK_CLOSE)))
const AI_WHEELS_OFF = Dict{Int,Vector{String}}()
const AI_LOOSE_WHEELS = Vector{LooseWheel}()
const AI_LOOSE_OWNER = Int[]
mutable struct AIWreck; b::WreckBody.Body; pl::WreckBody.Plane; walls::Vector{WreckBody.WallPlane}; end
const AI_WRECK = Dict{Int,AIWreck}()

ai_wheel_off(k::Int, nm::AbstractString) = (v = get(AI_WHEELS_OFF, k, nothing); v !== nothing && nm in v)

const WRECK_CLEAR = parse(Float64, get(ENV, "JM_WRECK_CLEAR", "5"))
const YELLOW_AFTER = 2.0
const AI_GONE_AT = Dict{Int,Float64}()
const AI_GONE = Set{Int}()
const YELLOW_T = Ref((-Inf, -Inf))
const PLAYER_YELLOWED = Ref(false)

function ai_wheeloff_reset!()
    empty!(AI_WHEELS_OFF); empty!(AI_LOOSE_WHEELS); empty!(AI_LOOSE_OWNER); empty!(AI_WRECK)
    empty!(AI_GONE_AT); empty!(AI_GONE); empty!(CLASSICAI.GONE); empty!(CLASSICAI.YZ); empty!(CLASSICAI.SOFTEDGE); YELLOW_T[] = (-Inf, -Inf); PLAYER_YELLOWED[] = false
    nothing
end

yellow_on(t) = YELLOW_T[][1] <= t < YELLOW_T[][2]

function yellow_start!(t::Float64, x::Float64, z::Float64, who::String, CLASSICAI_T)
    (t0, t1) = YELLOW_T[]
    YELLOW_T[] = (t1 > t ? t0 : t, max(t1, t + WRECK_CLEAR + YELLOW_AFTER))
    gs = NaN
    CLASSICAI_T === nothing || ((gs, _, _) = CLASSICAI.tlocate(CLASSICAI_T, x, z, 0); push!(CLASSICAI.YZ, gs))
    println("  [yellow] t=", round(t, digits = 2), " ", who, isnan(gs) ? "" : string(" at s ", round(gs, digits = 0)),
            ": yellow flag until t=", round(YELLOW_T[][2], digits = 1)); flush(stdout)
    nothing
end

"""TD261030-1 (2)+(7), one frame: a new AI wreck (or the player's) puts out the yellow; a wreck WRECK_CLEAR s old is taken away
with its wheels; the flag comes in YELLOW_AFTER s after the last one goes."""
function yellow_step!(t::Float64, px::Float64, pz::Float64, CLASSICAI_T)
    WRECK_CLEAR > 0 || return nothing
    for (k, w) in AI_WRECK
        haskey(AI_GONE_AT, k) && continue
        AI_GONE_AT[k] = t + WRECK_CLEAR
        yellow_start!(t, w.b.p[1], -w.b.p[3], string("AI ", k, " wrecked"), CLASSICAI_T)
    end
    if WRECKED[] && !PLAYER_YELLOWED[]
        PLAYER_YELLOWED[] = true
        yellow_start!(t, px, pz, "your car wrecked", CLASSICAI_T)
    end
    for (k, tg) in AI_GONE_AT
        (t >= tg && !(k in AI_GONE)) || continue
        push!(AI_GONE, k); push!(CLASSICAI.GONE, k); delete!(AI_WRECK, k)
        for i in reverse(eachindex(AI_LOOSE_OWNER))
            AI_LOOSE_OWNER[i] == k && (deleteat!(AI_LOOSE_OWNER, i); deleteat!(AI_LOOSE_WHEELS, i))
        end
        println("  [yellow] t=", round(t, digits = 2), " AI ", k, "'s wreck taken off the track"); flush(stdout)
    end
    if isfinite(YELLOW_T[][2]) && t >= YELLOW_T[][2]
        YELLOW_T[] = (-Inf, -Inf); empty!(CLASSICAI.YZ)
        println("  [yellow] t=", round(t, digits = 2), " yellow flag in -- the track is clear"); flush(stdout)
    end
    nothing
end

"The cars taken off the track: far away and under the ground, so nothing draws, records or touches them."
function ai_gone_poses!(ai_poses)
    for k in AI_GONE
        k <= length(ai_poses) && (ai_poses[k] = (1.0e5 + 50.0k, -1000.0, 1.0e5, 0.0, 0.0, 0.0))
    end
    ai_poses
end

"Render-frame hub of AI car `cm`'s drawn wheel at (wx, wz) for pose p."
ai_hub(p, cm, wx, wz, r) = (M = ai_wheel_mat(p, cm, wx, wz, r, 0.0); (Float64(M[1, 4]), Float64(M[2, 4]), Float64(M[3, 4])))

"""The player (at game (hx, hz), velocity (pvx, pvz)) hit AI car k (pose p, velocity (vx, vz)). Over the threshold, the
car's wheel nearest the player comes off. Returns the closing speed if a wheel came off, else NaN."""
function ai_tear_wheel!(k::Int, p, vx::Float64, vz::Float64, hx::Float64, hz::Float64, pvx::Float64, pvz::Float64, t::Float64)
    (AI_WHEELOFF_ON && k <= length(AICARMODELS::Vector{Render.CLASSICCarModel}) && !haskey(AI_WRECK, k)) || return NaN
    nx = hx - p[1]; nz = hz - p[3]; d = max(hypot(nx, nz), 1e-3)
    closing = ((vx - pvx)*nx + (vz - pvz)*nz)/d
    closing > AI_WHEELOFF_CLOSE || return NaN
    cm = AICARMODELS[k]::Render.CLASSICCarModel; best = 0; bd = Inf
    for (i, w) in enumerate(cm.wheelspec)
        ai_wheel_off(k, w[5]) && continue
        h = ai_hub(p, cm, w[1], w[2], w[4]); dd = hypot(h[1] - hx, -h[3] - hz)
        dd < bd && (bd = dd; best = i)
    end
    best == 0 && return NaN
    w = cm.wheelspec[best]; h = ai_hub(p, cm, w[1], w[2], w[4])
    push!(get!(Vector{String}, AI_WHEELS_OFF, k), w[5])

    ox = -h[3]; ax = h[1] - hx; az = ox - hz; al = max(hypot(ax, az), 1e-3)
    push!(AI_LOOSE_WHEELS, (h[1], h[2], ox, vx + 3.5ax/al, 3.0 + 0.15closing, vz + 3.5az/al, 0.0, hypot(vx, vz)/max(Float64(w[4]), 0.1), String(w[5])))
    push!(AI_LOOSE_OWNER, k)
    println(ai_tear_msg(k, String(cm.name), String(w[5]), closing)); flush(stdout)
    closing
end

ai_tear_msg(k::Int, car::String, nm::String, closing::Float64) =
    string("  [WRECK] AI ", k, " (", car, "): ", nm, " torn off -- hit at ", round(closing*3.6, digits = 0), " km/h closing; out of the race")

"""AI car k, a wheel gone, becomes a free rigid body at pose p with world velocity (vx, vz), yaw rate r, heave w and the
hit's roll and pitch rates (as bump3d! gives a knocked-loose car); the body's corner nearest each torn wheel (as drawn) is
off, so its upright scrapes."""
function ai_wreck_start!(k::Int, p, vx::Float64, vz::Float64, r::Float64, w::Float64, roll::Float64 = 0.0, pitch::Float64 = 0.0)
    WB = WreckBody; V3 = WB.V3
    b = WB.Body(WB.Geometry(m = DriveRT3D.MASS[]))
    WB.start!(b, Float64(p[1]), Float64(p[3]), Float64(p[2]), Float64(p[4]), Float64(p[5]), Float64(p[6]), V3(vx, w, -vz), V3(roll, r, pitch))
    b.brake = LOOSE_BRAKE
    cm = AICARMODELS[k]::Render.CLASSICCarModel; g = b.g
    for nm in get(AI_WHEELS_OFF, k, String[])
        i = findfirst(s -> s[5] == nm, cm.wheelspec); i === nothing && continue
        s = cm.wheelspec[i]; q = V3(ai_hub(p, cm, s[1], s[2], s[4])...)
        j = 1; dj = Inf
        for c in 1:4
            dc = WB.LinearAlgebra.norm(b.p + b.R*V3(g.wheel_x[c], g.wheel_r[c] - g.hcg, g.wheel_z[c]) - q)
            dc < dj && (j = c; dj = dc)
        end
        b.on[j] = false
    end
    AI_WRECK[k] = AIWreck(b, WB.Plane(Float64(p[1]), -Float64(p[3]), Float64(p[2]), 0.0, 0.0), WB.WallPlane[])
    nothing
end

"""Each AI wreck, one frame: its rigid body on the terrain (as loose_body_step!, never landing back), its pose into
ai_poses, and the field told it is stopped there (so the field neither moves it nor counts its laps). `groundz` = the
AI's ground (groundz_ai)."""
function ai_wrecks_step!(ai_poses, gdt::Float64, groundz::G, AICARS, AILINE, CLASSICAI_T, classicai_field!) where {G}
    WB = WreckBody; V3 = WB.V3

    hq(X, Z) = (y = Float64(groundz(X, -Z)); isfinite(y) && y > -900 ? y : NaN)
    th(X, Z, yref) = SKIDPAD ? 0.0 : (y = Float64(groundz(X, -Z)); (isfinite(y) && y > -900 && y <= yref) ? y :
                                      (r = JRTrack.hat3d(TERRAIN, X, -Z; ref = yref); r[3] ? Float64(r[1]) : objground_at(X, -Z, yref)))
    for (k, w) in AI_WRECK
        k > length(ai_poses) && continue
        b = w.b
        pl = WB.fit_plane(hq, b.p[1], b.p[3])
        (isfinite(pl.h0) && isfinite(pl.gx) && isfinite(pl.gz)) ? (w.pl = pl) : (pl = w.pl)
        WB.step!(b, gdt, WB.Terrain(th, pl); walls = ai_body_walls!(w.walls, b))
        o = b.p - b.R*V3(0.0, b.g.hcg, 0.0)

        gh = hq(b.p[1], b.p[3])
        if isfinite(gh) && o[2] < gh - 0.3
            b.p = b.p + V3(0.0, gh - o[2], 0.0); b.v = V3(b.v[1], max(b.v[2], 0.0), b.v[3]); o = V3(o[1], gh, o[3])
        end
        (θ, pitch, roll) = WB.attitude(b)
        pose = (o[1], o[2], -o[3], θ, pitch, roll)
        ai_wall_tear!(k, b, pose); ai_ground_tear!(k, b, pose)
        ai_poses[k] = pose
        if CLASSICAI_T !== nothing
            gc = classicai_field!().cars[k]
            (gs, gd, _) = CLASSICAI.tlocate(CLASSICAI_T, pose[1], pose[3], 0)
            gc.s = gs; gc.d = gd; gc.v = 0.0; gc.dv = 0.0; gc.yaw = θ; gc.yawr = 0.0; gc.pose = pose; gc.prev = pose
        end
        if AILINE !== nothing && k <= length(AICARS)
            (rs, rl) = RaceAI.project(AILINE, pose[1], pose[3]; hint = AICARS[k].s)
            AICARS[k].s = rs; AICARS[k].lane = clamp(rl, -RaceAI.LANE_MAX, RaceAI.LANE_MAX); AICARS[k].v = 0.0
        end
    end
    nothing
end

"The torn-off AI wheels, drawn with their own car's wheel mesh (no yaw, as the player's loose wheels)."
function draw_ai_loose_wheels!(drawfn, chassis)
    for j in eachindex(AI_LOOSE_WHEELS)
        k = AI_LOOSE_OWNER[j]; k > length(chassis) && continue
        cm = chassis[k]; w = AI_LOOSE_WHEELS[j]
        M = Render.translate(w[1], w[2], -w[3]) * ai_hand(cm) * Render.rotz(Float32(w[7]))
        its = get(cm.wheels, w[9], nothing)
        its === nothing && continue
        for it in its; drawfn(it, M); end
    end
    nothing
end

"""S2: the walls an AI rigid body steps against -- Classic's two faces beside its CG and the solid objects near it, as planes
(as wreck_body_step! builds the player's). Stateless: located without a section hint and judged at the body's own lateral
(classicwall_gaps keeps the player's "came from" side; an AI body starts on the road, so it has none to keep)."""
function ai_body_walls!(walls::Vector{WreckBody.WallPlane}, b::WreckBody.Body)
    WB = WreckBody; V3 = WB.V3
    empty!(walls)
    AI_WHEELOFF_ON || return walls
    gx = b.p[1]; gz = -b.p[3]
    if CLASSICW_PHYS
        (sec, f, lat, nx, nz) = classicw_locate(gx, gz; hint = 0)
        (lo, hi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, sec, f, lat, lat; hmin = CLASSICW_HMIN)
        for (d, n) in ((lat - lo, (nx, nz)), (hi - lat, (-nx, -nz)))
            (isfinite(d) && 0.0 < d < 4.0) || continue
            push!(walls, WB.WallPlane(V3(gx - d*n[1], b.p[2], -(gz - d*n[2])), V3(n[1], 0.0, -n[2])))
        end
    end
    for k in eachindex(SOLIDS)
        (ox, oz, r, kind) = SOLIDS[k]
        hypot(ox - gx, oz - gz) < r + 6.0 || continue
        (g, sx, sz) = solid_gap(gx, gz, k)
        g < 4.0 || continue
        push!(walls, WB.WallPlane(V3(gx - g*sx, b.p[2], -(gz - g*sz)), V3(sx, 0.0, -sz), kind === :soft))
    end
    walls
end

"""S2: a hard wall hit on AI car k's rigid body (pose `p`, game frame) tears off the wheels leading into that wall, as
E95 does for the player's: the body's wall push > 30 kN at > 3 m/s, each still-attached hub > 0.35 m ahead of the CG
against the wall's normal. The body loses that corner. Returns the number torn off."""
function ai_wall_tear!(k::Int, b::WreckBody.Body, p)
    WB = WreckBody; V3 = WB.V3
    (AI_WHEELOFF_ON && b.wall_peak > 3.0e4 && WB.LinearAlgebra.norm(b.v) > 3.0 && k <= length(AICARMODELS::Vector{Render.CLASSICCarModel})) || return 0
    cm = AICARMODELS[k]::Render.CLASSICCarModel; n = b.wall_n; g = b.g; c = 0
    for w in cm.wheelspec
        ai_wheel_off(k, w[5]) && continue
        h = ai_hub(p, cm, w[1], w[2], w[4])
        lead = -((h[1] - b.p[1])*n[1] + (h[3] - b.p[3])*n[3])
        lead > 0.35 || continue
        push!(get!(Vector{String}, AI_WHEELS_OFF, k), String(w[5]))
        q = V3(h...); j = 1; dj = Inf
        for cc in 1:4
            dc = WB.LinearAlgebra.norm(b.p + b.R*V3(g.wheel_x[cc], g.wheel_r[cc] - g.hcg, g.wheel_z[cc]) - q)
            dc < dj && (j = cc; dj = dc)
        end
        b.on[j] = false

        vn = b.v[1]*n[1] + b.v[3]*n[3]; tx = b.v[1] - vn*n[1]; tz = b.v[3] - vn*n[3]; out = abs(vn)*0.35
        push!(AI_LOOSE_WHEELS, (h[1], h[2], -h[3], 0.7tx + out*n[1], 2.5 + 0.1*WB.LinearAlgebra.norm(b.v), -(0.7tz + out*n[3]), 0.0,
                                WB.LinearAlgebra.norm(b.v)/max(Float64(w[4]), 0.1), String(w[5])))
        push!(AI_LOOSE_OWNER, k)
        println(ai_walltear_msg(k, String(cm.name), String(w[5]), WB.LinearAlgebra.norm(b.v))); flush(stdout)
        c += 1
    end
    c
end
"""TD261029-3: AI car k's body (pose `p`) struck a wheel into the ground or loaded it upside down (WreckBody.ground_tears):
that wheel comes off, as on a wall. Returns the number torn off."""
function ai_ground_tear!(k::Int, b::WreckBody.Body, p)
    WB = WreckBody; V3 = WB.V3
    (AI_WHEELOFF_ON && k <= length(AICARMODELS::Vector{Render.CLASSICCarModel})) || return 0
    js = WB.ground_tears(b, WHEEL_TEAR[1], WHEEL_TEAR[2]*b.g.m*9.80665); isempty(js) && return 0
    cm = AICARMODELS[k]::Render.CLASSICCarModel; g = b.g; c = 0
    for j in js
        b.on[j] = false
        q = b.p + b.R*V3(g.wheel_x[j], g.wheel_r[j] - g.hcg, g.wheel_z[j])
        best = 0; bd = Inf
        for (i, w) in enumerate(cm.wheelspec)
            ai_wheel_off(k, w[5]) && continue
            h = ai_hub(p, cm, w[1], w[2], w[4]); dd = WB.LinearAlgebra.norm(V3(h...) - q)
            dd < bd && (bd = dd; best = i)
        end
        best == 0 && continue
        w = cm.wheelspec[best]; h = ai_hub(p, cm, w[1], w[2], w[4])
        push!(get!(Vector{String}, AI_WHEELS_OFF, k), String(w[5]))
        push!(AI_LOOSE_WHEELS, (h[1], h[2], -h[3], b.v[1], max(b.v[2], 0.0) + 2.0, -b.v[3], 0.0, b.spin[j], String(w[5])))
        push!(AI_LOOSE_OWNER, k)
        println("  [WRECK] AI ", k, " (", cm.name, "): ", w[5], " torn off on the ground (struck at ", round(b.gnd_v[j], digits = 1),
                " m/s); out of the race"); flush(stdout)
        c += 1
    end
    c
end
ai_walltear_msg(k::Int, car::String, nm::String, v::Float64) =
    string("  [WRECK] AI ", k, " (", car, "): ", nm, " torn off on a wall at ", round(v*3.6, digits = 0), " km/h; out of the race")

const AIWALL_TEST = (f = split(get(ENV, "JM_AIWALL_TEST", ""), ":"); length(f) == 3 && haskey(ENV, "JM_SMOKE") ? Tuple(parse.(Float64, f)) : nothing)
const AIWALL_DONE = Ref(false)
function aiwall_test_step!(t::Float64, ai_poses, AICARS)
    (AIWALL_TEST === nothing || AIWALL_DONE[] || t < AIWALL_TEST[1] || !CLASSICW_PHYS) && return nothing
    k = Int(AIWALL_TEST[2]); k <= min(length(ai_poses), length(AICARS)) || return nothing
    AIWALL_DONE[] = true
    p = ai_poses[k]; vk = Float64(AICARS[k].v)
    (sec, f, lat, nx, nz) = classicw_locate(Float64(p[1]), Float64(p[3]); hint = 0)
    (lo, hi, _, _) = CLASSICWall.free_interval(CLASSICWALLS, sec, f, lat, lat; hmin = CLASSICW_HMIN)
    s = (lat - lo) < (hi - lat) ? -1.0 : 1.0
    vx = vk*cos(p[4]) + s*AIWALL_TEST[3]*nx; vz = vk*sin(p[4]) + s*AIWALL_TEST[3]*nz
    haskey(LOOSE_OF, k) && (LOOSE_BODY[LOOSE_OF[k]] = nothing; delete!(LOOSE_OF, k))
    ai_wreck_start!(k, p, vx, vz, 0.0, 0.0)
    println("  [aiwall] t=", round(t, digits = 2), " AI ", k, " sent ", round(min(lat - lo, hi - lat), digits = 1), " m to a wall at ",
            AIWALL_TEST[3], " m/s across, ", round(vk, digits = 1), " m/s along"); flush(stdout)
    nothing
end

const REPLAY_WHEELBUF = Float32[]
const REP_WB = Float32[]
const REP_WIDX = UnitRange{Int}[]
const REPLAY_WHEELS = get(ENV, "JM_REPLAY_WHEELS", "1") != "0"
const REPLAY_WHEELDIAG = get(ENV, "JM_REPLAY_WHEELDIAG", "0") != "0"
const REP_WDIAGN = Ref(0)

function replay_wheels_rec!(buf::Vector{Float32}, f::Int)
    for w in LOOSE_WHEELS
        i = findfirst(==(w[9]), WHEEL_NAMES); i === nothing && continue
        push!(buf, Float32(f), 0f0, Float32(i), Float32(w[1]), Float32(w[2]), Float32(w[3]), Float32(w[7]))
    end
    cms = AICARMODELS::Vector{Render.CLASSICCarModel}
    for j in eachindex(AI_LOOSE_WHEELS)
        k = AI_LOOSE_OWNER[j]; k <= length(cms) || continue
        w = AI_LOOSE_WHEELS[j]; i = findfirst(s -> s[5] == w[9], cms[k].wheelspec); i === nothing && continue
        push!(buf, Float32(f), Float32(k), Float32(i), Float32(w[1]), Float32(w[2]), Float32(w[3]), Float32(w[7]))
    end
    nothing
end

"The records of a recording's first nf frames (they are written in frame order)."
replay_wheels_upto(buf::Vector{Float32}, nf::Int) =
    (n = length(buf) ÷ 7; m = 0; while m < n && buf[m*7+1] < nf; m += 1; end; buf[1:m*7])

"Load a replay's wheel records (`wheels`, if the file has them) and index them by frame."
function replay_wheels_load!(repd)
    empty!(REP_WB); empty!(REP_WIDX)
    (REPLAY_WHEELS && hasproperty(repd, :wheels) && !isempty(repd.wheels)) || return 0
    append!(REP_WB, repd.wheels); resize!(REP_WIDX, repd.nframes); fill!(REP_WIDX, 1:0)
    n = length(REP_WB) ÷ 7; a = 1
    while a <= n
        f = round(Int, REP_WB[(a-1)*7+1]); b = a
        while b < n && round(Int, REP_WB[b*7+1]) == f; b += 1; end
        0 <= f < repd.nframes && (REP_WIDX[f+1] = a:b)
        a = b + 1
    end
    n
end

"""The torn-off wheels at replay frame position fr: the player's LOOSE_WHEELS and the AI's lists, rebuilt from the
recording; a wheel in both neighbouring frames is placed between them."""
function replay_wheels_set!(fr::Float64)
    empty!(LOOSE_WHEELS); empty!(AI_LOOSE_WHEELS); empty!(AI_LOOSE_OWNER); empty!(AI_WHEELS_OFF)
    isempty(REP_WIDX) && return nothing
    wb = REP_WB; nf = length(REP_WIDX); f = clamp(fr, 0.0, nf - 1.0); i0 = floor(Int, f); i1 = min(i0 + 1, nf - 1); g = f - i0
    cms = AICARMODELS::Vector{Render.CLASSICCarModel}
    for a in REP_WIDX[i0+1]
        o = (a - 1)*7
        own = round(Int, wb[o+2]); wi = round(Int, wb[o+3])
        x = Float64(wb[o+4]); y = Float64(wb[o+5]); z = Float64(wb[o+6]); sp = Float64(wb[o+7])
        for b in REP_WIDX[i1+1]
            q = (b - 1)*7
            if wb[q+2] == wb[o+2] && wb[q+3] == wb[o+3]
                x += g*(wb[q+4] - x); y += g*(wb[q+5] - y); z += g*(wb[q+6] - z); sp += g*(wb[q+7] - sp)
                break
            end
        end
        if own == 0
            1 <= wi <= 4 && push!(LOOSE_WHEELS, (x, y, z, 0.0, 0.0, 0.0, sp, 0.0, WHEEL_NAMES[wi]))
        elseif own <= length(cms) && 1 <= wi <= length(cms[own].wheelspec)
            nm = String(cms[own].wheelspec[wi][5])
            push!(AI_LOOSE_WHEELS, (x, y, z, 0.0, 0.0, 0.0, sp, 0.0, nm)); push!(AI_LOOSE_OWNER, own)
            push!(get!(Vector{String}, AI_WHEELS_OFF, own), nm)
        end
    end
    if REPLAY_WHEELDIAG && (REP_WDIAGN[] += 1) % 15 == 0
        println("  [repwheels] f=", round(f, digits = 1), " player loose ", length(LOOSE_WHEELS), " AI off ",
                join(sort!([string(k, ":", join(v, "+")) for (k, v) in AI_WHEELS_OFF]), " "),
                isempty(AI_LOOSE_WHEELS) ? "" : string(" first at ", round.((AI_LOOSE_WHEELS[1][1], AI_LOOSE_WHEELS[1][2], AI_LOOSE_WHEELS[1][3]), digits = 2)))
    end
    nothing
end

const YELLOW_V = Float32[]
"""TD261030-1 (7): the yellow flag, waving, at the left of the 3D view (where the chequered flag shows) while it is out."""
function yellow_flag_draw!(t::Float64, W, H)
    yellow_on(t) || return nothing
    v = empty!(YELLOW_V)
    fw = round(0.09W); fh = round(0.62fw); fx = round(0.05W); fy = band_px() + round(0.30view_h() - fh/2)
    Render.hquad!(v, fx - 10, fy - 6, 6, fh*2.2, (0.55, 0.45, 0.30))
    n = 8; sw = fw/n
    for i in 0:n-1
        dy = 0.06fh*sin(2π*(1.6t - i/n))*(i/n); sh = 0.85 + 0.15*cos(2π*(1.6t - i/n))
        Render.hquad!(v, fx + i*sw, fy + dy, sw + 1, fh, (0.98sh, 0.84sh, 0.08))
    end
    Render.hud_draw(hudprog, hudvao, hudvbo, v, W, H)
    nothing
end
