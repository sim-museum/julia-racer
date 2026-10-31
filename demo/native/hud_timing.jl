

const HUD_DELTA_ON  = get(ENV, "JM_HUD_DELTA", "1") != "0"
const HUD_TIMING_ON = get(ENV, "JM_HUD_TIMING", "1") != "0"
const HUD_MAP_ON    = get(ENV, "JM_HUD_MAP", "1") != "0"
const DELTA_RANGE   = clamp(parse(Float64, get(ENV, "JM_DELTA_RANGE", "2.0")), 0.2, 30.0)
const LT_BIN = 5.0

hud_sector_starts(track::AbstractString) =
    track == "monza"   ? [0.0, 0.342361, 0.472561, 0.655202, 0.851958] :
    track == "watglen" ? [0.0, 0.415715, 0.775550] : [0.0, 1/3, 2/3]

mutable struct LapClock
    cur::Vector{Float64}
    prev::Vector{Float64}
    hasprev::Bool
    laps::Int
    lasts::Float64; lastt::Float64
    secs::Vector{Float64}
    sec_t0::Vector{Float64}
    sec_now::Vector{Float64}
    sec_last::Vector{Float64}
    sec_best::Vector{Float64}
    sec_dnow::Vector{Float64}
    sec_dlast::Vector{Float64}
    sec_lastdone::Int
    delta::Float64; dvalid::Bool
end
LapClock() = LapClock(Float64[], Float64[], false, 0, 0.0, 0.0, Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], 0, 0.0, false)
const LAPCLOCK = LapClock()

function lapclock_reset!(c::LapClock, total::Float64, fracs::Vector{Float64})
    n = ceil(Int, total/LT_BIN) + 1
    c.cur = fill(NaN, n); c.prev = fill(NaN, n); c.hasprev = false; c.laps = 0; c.lasts = 0.0; c.lastt = 0.0
    c.secs = fracs .* total; ns = length(fracs)
    c.sec_t0 = fill(NaN, ns); c.sec_t0[1] = 0.0
    c.sec_now = fill(NaN, ns); c.sec_last = fill(NaN, ns); c.sec_best = fill(NaN, ns); c.sec_dnow = fill(NaN, ns); c.sec_dlast = fill(NaN, ns)
    c.sec_lastdone = 0; c.delta = 0.0; c.dvalid = false; c.cur[1] = 0.0
    c
end

sector_of(c::LapClock, s::Float64) = (k = 1; for j in 2:length(c.secs); s >= c.secs[j] && (k = j); end; k)

"Time into a stored lap at distance s (linear between bins); NaN where the lap has no sample."
function lap_time_at(v::Vector{Float64}, s::Float64)
    x = s/LT_BIN; k = clamp(floor(Int, x) + 1, 1, length(v) - 1); f = clamp(x - (k - 1), 0.0, 1.0)
    a = v[k]; b = v[k + 1]
    isnan(a) ? NaN : isnan(b) ? a : a + f*(b - a)
end

function lapclock_fill!(c::LapClock, s::Float64, t::Float64)
    s <= c.lasts && return
    k0 = floor(Int, c.lasts/LT_BIN) + 2; k1 = min(floor(Int, s/LT_BIN) + 1, length(c.cur))
    for k in k0:k1
        sk = (k - 1)*LT_BIN
        c.cur[k] = c.lastt + (t - c.lastt)*(sk - c.lasts)/(s - c.lasts)
    end

    for j in 2:length(c.secs)
        b = c.secs[j]
        if c.lasts < b <= s
            tb = c.lastt + (t - c.lastt)*(b - c.lasts)/(s - c.lasts)
            if !isnan(c.sec_t0[j - 1])
                c.sec_now[j - 1] = tb - c.sec_t0[j - 1]; close_sector!(c, j - 1)
            end
            c.sec_t0[j] = tb
        end
    end
    c.lasts = s; c.lastt = t
end

function close_sector!(c::LapClock, j::Int)
    d = c.sec_now[j]
    c.sec_dnow[j] = d - c.sec_best[j]
    (isnan(c.sec_best[j]) || d < c.sec_best[j]) && (c.sec_best[j] = d)
    c.sec_lastdone = j
end

"""One frame: s = distance into the current lap [m], t = time into it [s], laps = cs.laps, total = lap length,
lastlap = the lap time just banked by lap_timing_step!. Call after lap_timing_step! (it has already reset lap_t0)."""
function lapclock_step!(c::LapClock, s::Float64, t::Float64, laps::Int, total::Float64, lastlap::Float64, running::Bool, fracs::Vector{Float64})
    (isempty(c.cur) || laps < c.laps) && lapclock_reset!(c, total, fracs)
    if laps > c.laps

        lastlap > 0 && lapclock_fill!(c, total, lastlap)
        ns = length(c.secs)
        if !isnan(c.sec_t0[ns]) && lastlap > 0
            c.sec_now[ns] = lastlap - c.sec_t0[ns]; close_sector!(c, ns)
        end
        c.sec_last .= c.sec_now; c.sec_dlast .= c.sec_dnow

        if lastlap > 0
            for k in length(c.cur):-1:1
                isnan(c.cur[k]) || break
                (k - 1)*LT_BIN >= total - 1e-9 ? (c.cur[k] = lastlap) : break
            end
        end
        clean = !any(isnan, c.cur)
        clean && copyto!(c.prev, c.cur)
        c.hasprev = c.hasprev || clean
        fill!(c.cur, NaN); c.cur[1] = 0.0; c.lasts = 0.0; c.lastt = 0.0
        fill!(c.sec_t0, NaN); c.sec_t0[1] = 0.0; fill!(c.sec_now, NaN); fill!(c.sec_dnow, NaN)
        c.laps = laps
    end
    running && s > c.lasts && s - c.lasts < 400.0 && lapclock_fill!(c, s, t)
    running && s - c.lasts >= 400.0 && (c.lasts = s; c.lastt = t)
    if c.hasprev && running
        tp = lap_time_at(c.prev, s); c.dvalid = !isnan(tp); c.dvalid && (c.delta = t - tp)
    else
        c.dvalid = false
    end
    c
end

@inline function htri!(v, x1, y1, x2, y2, x3, y3, c)
    push!(v, x1, y1, c[1], c[2], c[3], x2, y2, c[1], c[2], c[3], x3, y3, c[1], c[2], c[3])
end
function hdisc!(v, cx, cy, r, c; n = 16)
    for i in 0:n-1
        a0 = 2π*i/n; a1 = 2π*(i + 1)/n
        htri!(v, cx, cy, cx + r*cos(a0), cy + r*sin(a0), cx + r*cos(a1), cy + r*sin(a1), c)
    end
end

function hpolyline!(v, xs, ys, w, c; closed = true)
    n = length(xs); h = w/2
    for i in 1:(closed ? n : n - 1)
        j = i == n ? 1 : i + 1
        dx = xs[j] - xs[i]; dy = ys[j] - ys[i]; L = hypot(dx, dy); L < 1e-6 && continue
        nx = -dy/L*h; ny = dx/L*h
        htri!(v, xs[i] + nx, ys[i] + ny, xs[j] + nx, ys[j] + ny, xs[j] - nx, ys[j] - ny, c)
        htri!(v, xs[i] + nx, ys[i] + ny, xs[j] - nx, ys[j] - ny, xs[i] - nx, ys[i] - ny, c)
        hdisc!(v, xs[j], ys[j], h, c; n = 8)
    end
end

const DELTA_RED = (0.86, 0.12, 0.10); const DELTA_GREEN = (0.15, 0.78, 0.22)
"""The delta bar centred at cx: slower = grows left in red, faster = grows right in green (PO 2026-10-26).
Pushes the bar's quads to bv and its number (e.g. "+0.53", right of the bar) to tv."""
function delta_bar!(bv, tv, font, cx, y, w, h, delta; scale = 1.0, range = DELTA_RANGE)
    Render.hquad!(bv, cx - w/2, y, w, h, (0.16, 0.16, 0.18))
    f = clamp(abs(delta)/range, 0.0, 1.0)*(w/2)
    if delta > 0
        Render.hquad!(bv, cx - f, y, f, h, DELTA_RED)
    elseif delta < 0
        Render.hquad!(bv, cx, y, f, h, DELTA_GREEN)
    end
    Render.hquad!(bv, cx - 1, y - 2, 2, h + 4, (0.9, 0.9, 0.9))
    if font !== nothing
        msg = (delta >= 0 ? "+" : "-") * fmt2(abs(delta))
        Render.text!(tv, font, cx + w/2 + 8, y + (h - scale*font.lineh)/2, msg, delta > 0 ? (1.0, 0.45, 0.40) : (0.45, 1.0, 0.45); scale = scale)
    end
    nothing
end

"Seconds with exactly two decimals (12.30, not 12.3)."
fmt2(x) = (c = round(Int, abs(x)*100); string(x < 0 && c > 0 ? "-" : "", c ÷ 100, ".", lpad(c % 100, 2, '0')))
secfmt(t) = isnan(t) || t <= 0 ? "--.--" : t >= 60 ? string(floor(Int, t/60), ":", lpad(fmt2(mod(t, 60.0)), 5, '0')) : fmt2(t)
"""Sector columns from x0: per sector its number, this lap's time (or the last lap's while it is still running) and
the delta to the session's best; then Lap n/N, position and Opt. Returns the right edge used."""
function timing_cols!(tv, font, c::LapClock, x0, y0, lapn, lapN, pos, nfield; colw = 92.0)
    lh = font.lineh; ns = length(c.secs); x = x0
    cs_ = sector_of(c, c.lasts)
    for j in 1:ns
        Render.text!(tv, font, x, y0, "S$j", j == cs_ ? (1.0, 0.82, 0.35) : (0.75, 0.75, 0.75))
        done = !isnan(c.sec_now[j]); d = done ? c.sec_now[j] : c.sec_last[j]
        Render.text!(tv, font, x, y0 + lh, secfmt(d), done ? (0.95, 0.95, 0.95) : (0.55, 0.55, 0.55))
        dd = done ? c.sec_dnow[j] : c.sec_dlast[j]
        if !isnan(d) && !isnan(dd)
            Render.text!(tv, font, x, y0 + 2lh, (dd > 0.0005 ? "+" : dd < -0.0005 ? "-" : " ") * fmt2(abs(dd)),
                         dd > 0.0005 ? (1.0, 0.45, 0.40) : (0.45, 1.0, 0.45))
        end
        x += colw
    end
    opt = any(isnan, c.sec_best) ? 0.0 : sum(c.sec_best)
    Render.text!(tv, font, x, y0, lapN > 0 ? "Lap $(lapn)/$(lapN)" : "Lap $(lapn)", (0.95, 0.95, 0.95))
    pos > 0 && Render.text!(tv, font, x, y0 + lh, "P$(pos) of $(nfield)", (1.0, 0.82, 0.35))
    Render.text!(tv, font, x, y0 + 2lh, "O  " * lapfmt(opt), (0.75, 0.75, 0.95))
    x + 120.0
end

const MAP_N = 360
mutable struct TrackMap; ux::Vector{Float64}; uy::Vector{Float64}; c::Float64; s::Float64; cx::Float64; cz::Float64
    lo::Float64; wl::Float64; sc::Float64; aspect::Float64; ok::Bool
    lx::Vector{Float64}; lz::Vector{Float64}; ls::Vector{Float64}; total::Float64; end
TrackMap() = TrackMap(Float64[], Float64[], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, false, Float64[], Float64[], Float64[], 1.0)
const TRACKMAP = TrackMap()
function trackmap_build!(m::TrackMap, xs::Vector{Float64}, zs::Vector{Float64}, ss::Vector{Float64}, total::Float64)
    n = length(xs); n < 10 && return m
    m.lx = copy(xs); m.lz = copy(zs); m.ls = copy(ss); m.total = total
    idx = unique(round.(Int, range(1, n, length = min(n, MAP_N))))
    x = xs[idx]; z = zs[idx]
    mx = sum(x)/length(x); mz = sum(z)/length(z)
    sxx = sum((x .- mx).^2); szz = sum((z .- mz).^2); sxz = sum((x .- mx).*(z .- mz))
    th = 0.5*atan(2sxz, sxx - szz)
    c = cos(th); s = sin(th)

    u = (x .- mx).*c .+ (z .- mz).*s; w = (x .- mx).*s .- (z .- mz).*c
    lo = minimum(u); hi = maximum(u); wl = minimum(w); wh = maximum(w)
    m.ux = (u .- lo)./(hi - lo); m.uy = (w .- wl)./(hi - lo)
    m.c = c; m.s = s; m.cx = mx; m.cz = mz; m.lo = lo; m.wl = wl; m.sc = hi - lo
    m.aspect = (hi - lo)/max(wh - wl, 1e-6); m.ok = true
    m
end
"The line's world (x, z) at arc length s (wrapped)."
function line_xz(m::TrackMap, s::Float64)
    q = mod(s, m.total); k = clamp(searchsortedlast(m.ls, q), 1, length(m.ls))
    k == length(m.ls) && return (m.lx[k], m.lz[k])
    f = clamp((q - m.ls[k])/max(m.ls[k + 1] - m.ls[k], 1e-9), 0.0, 1.0)
    (m.lx[k] + f*(m.lx[k + 1] - m.lx[k]), m.lz[k] + f*(m.lz[k + 1] - m.lz[k]))
end

@inline function map_pt(m::TrackMap, x, z)
    u = (x - m.cx)*m.c + (z - m.cz)*m.s; w = (x - m.cx)*m.s - (z - m.cz)*m.c
    ((u - m.lo)/m.sc, (w - m.wl)/m.sc)
end
"""The map fitted into (bx, by, bw, bh): ribbon + white edge, the start tick, a disc per car (cars = (id, x, z, pos);
id 0 = the player, drawn last and larger with a white ring, as reference's)."""
function trackmap_draw!(bv, tv, font, m::TrackMap, bx, by, bw, bh, cars, sfx, sfz)
    m.ok || return nothing
    n = length(m.ux)
    sc = min(bw, bh*m.aspect); hgt = sc/m.aspect
    ox = bx + (bw - sc)/2; oy = by + (bh - hgt)/2
    xs = [ox + m.ux[i]*sc for i in 1:n]; ys = [oy + m.uy[i]*sc for i in 1:n]
    rw = clamp(min(bh/11, sc/30), 3.0, 30.0)
    hpolyline!(bv, xs, ys, rw + 3.0, (0.92, 0.92, 0.92))
    hpolyline!(bv, xs, ys, rw, (0.05, 0.04, 0.04))
    p = map_pt(m, sfx, sfz); px = ox + p[1]*sc; py = oy + p[2]*sc
    Render.hquad!(bv, px - 1.5, py - rw, 3.0, 2rw, (0.95, 0.95, 0.95))
    r = clamp(min(bh/14, sc/28), 3.5, 30.0)
    for pass in 1:2, (id, x, z, pos) in cars
        (pass == 2) == (id == 0) || continue
        q = map_pt(m, x, z); qx = ox + q[1]*sc; qy = oy + q[2]*sc
        rr = id == 0 ? 1.35r : r
        id == 0 && hdisc!(bv, qx, qy, rr + 2.0, (0.98, 0.98, 0.98))
        hdisc!(bv, qx, qy, rr, (0.98, 0.78, 0.10))
        if font !== nothing && pos > 0
            t = string(pos); tw = Render.text_width(font, t)
            Render.text!(tv, font, qx - tw/2, qy - font.lineh/2, t, (0.05, 0.05, 0.05))
        end
    end
    nothing
end

let bv = Float32[], tv = Float32[], c = LapClock()
    lapclock_reset!(c, 1000.0, [0.0, 0.5]); lapclock_step!(c, 10.0, 1.0, 0, 1000.0, 0.0, true, [0.0, 0.5])
    lapclock_step!(c, 600.0, 9.0, 0, 1000.0, 0.0, true, [0.0, 0.5]); lapclock_step!(c, 2.0, 0.1, 1, 1000.0, 15.0, true, [0.0, 0.5])
    lap_time_at(c.prev, 3.0)
    delta_bar!(bv, tv, FONT, 500.0, 6.0, 400.0, 14.0, 0.3); delta_bar!(bv, tv, FONT, 500.0, 6.0, 400.0, 14.0, -0.3)
    FONT === nothing || timing_cols!(tv, FONT, c, 200.0, 6.0, 2, 3, 1, 7)
    m = TrackMap()
    trackmap_build!(m, [100cos(a) for a in 0:0.1:6.2], [50sin(a) for a in 0:0.1:6.2], collect(0.0:7.5:7.5*62), 470.0); line_xz(m, 100.0)
    trackmap_draw!(bv, tv, FONT, m, 0.0, 0.0, 200.0, 100.0, [(0, 100.0, 0.0, 1), (1, 0.0, 50.0, 2)], 100.0, 0.0)
end
const HUD_SECFRACS = hud_sector_starts(TRACKSEL)
const HUDX_BV = Float32[]
const HUD_MAPCARS = Tuple{Int,Float64,Float64,Int}[]
const HUD_FORCE_DELTA = something(tryparse(Float64, get(ENV, "JM_HUD_FORCE_DELTA", "")), NaN)
const HUD_DCX = Ref(0.0)

const REP_STAND = Tuple{Int,Float64}[]
const REP_STANDDIAG = get(ENV, "JM_REPLAY_STANDDIAG", "0") != "0"
const REP_SDN = Ref(0)
function replay_standings!(tl::Vector{Float32}, nt::Int, kp::Int, np::Int, na::Int, ks::Int, kl::Int, ncar::Int, i::Int, total::Float64)
    empty!(REP_STAND); b = i*nt
    push!(REP_STAND, (0, Float64(tl[b+kp])))
    for k in 1:ncar-1
        o = b + np + na*(k - 1)
        push!(REP_STAND, (k, Float64(tl[o+kl]) + mod(Float64(tl[o+ks]), total)/total))
    end
    sort!(REP_STAND, by = e -> -e[2])
    REP_STANDDIAG && (REP_SDN[] += 1) % 60 == 0 &&
        println("  [repstand] f=", i, " order ", join((string(e[1], "@", round(e[2], digits = 3)) for e in REP_STAND), " "))
    nothing
end

const REPCLK_I = Ref(-1); const REPCLK_LAPS = Ref(0); const REPCLK_T0 = Ref(0.0); const REPCLK_P = Ref((0.0, 0.0))
const REPCLK_LAST = Ref(0.0); const REPCLK_BEST = Ref(0.0); const REPCLK_CUR = Ref(0.0); const REPCLK_ON = Ref(false)
function replay_lapclock!(c::LapClock, tl::Vector{Float32}, nt::Int, kp::Int, d::Vector{Float32}, st::Int, i::Int,
                          total::Float64, fracs::Vector{Float64})
    if i < REPCLK_I[]
        REPCLK_I[] = -1; empty!(c.cur)
    end
    for f in (REPCLK_I[] + 1):i
        t = Float64(d[f*st + 1]); p = Float64(tl[f*nt + kp]); laps = floor(Int, p)
        if f == 0
            REPCLK_T0[] = t; REPCLK_LAPS[] = laps; REPCLK_LAST[] = 0.0; REPCLK_BEST[] = 0.0
        elseif laps > REPCLK_LAPS[]
            (tp, pp) = REPCLK_P[]
            tc = tp + (t - tp)*clamp((laps - pp)/max(p - pp, 1e-9), 0.0, 1.0)
            REPCLK_LAST[] = tc - REPCLK_T0[]
            REPCLK_BEST[] = REPCLK_BEST[] > 0 ? min(REPCLK_BEST[], REPCLK_LAST[]) : REPCLK_LAST[]
            REPCLK_T0[] = tc; REPCLK_LAPS[] = laps
        end
        lapclock_step!(c, (p - laps)*total, t - REPCLK_T0[], laps, total, REPCLK_LAST[], true, fracs)
        REPCLK_P[] = (t, p); REPCLK_CUR[] = t - REPCLK_T0[]
    end
    REPCLK_I[] = max(i, REPCLK_I[]); REPCLK_ON[] = true
    nothing
end
