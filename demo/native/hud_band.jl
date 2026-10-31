

const BANDMIR_ON = HUDBAND_ON && get(ENV, "JM_HUDBAND_MIRROR", "1") != "0"
const MAP_SPAN = parse(Float64, get(ENV, "JM_MAP_SPAN", "2600"))
const MAP_FOLLOW = Ref(false)

const HUDBAND_DIAG = something(tryparse(Float64, get(ENV, "JM_HUDBAND_DIAG", "")), NaN)
const HUDBAND_DIAG_DONE = Ref(false)

struct BandLayout
    circ::NTuple{4,Float64}; mirror::NTuple{4,Float64}; mid::NTuple{4,Float64}; map::NTuple{4,Float64}
    pos::NTuple{4,Float64}; tach::NTuple{4,Float64}; timing::NTuple{4,Float64}
end

const BAND_TT_ON = get(ENV, "JM_HUD_TYRETEMP", "1") != "0"

const BAND_GAUGE_ON = get(ENV, "JM_HUD_CLASSICGAUGE", "1") != "0"
"The band's boxes (x, y, w, h), window pixels from the top-left."
function band_layout(W::Int, BH::Int)
    pad = 8.0; h = BH - 2pad
    circ = (pad, pad, h, h)
    g = BAND_GAUGE_ON

    pos = (circ[1] + circ[3] + pad, pad, g ? round(0.55h) : 0.0, h)
    tach = (pos[1] + pos[3] + (g ? pad : 0.0), pad, g ? round(0.80h) : 0.0, h)
    mw = g ? min(1.35h, 0.26W) : min(1.8h, 0.26W); map = (W - pad - mw, pad, mw, h)
    tw = g ? round(1.05h) : 0.0; timing = (map[1] - (g ? pad : 0.0) - tw, pad, tw, h)
    x0 = tach[1] + tach[3] + 2pad; x1 = timing[1] - 2pad

    rowh = max(44.0, round(0.20BH))
    mirh = BH - 3pad - rowh
    mirror = (x0, pad, x1 - x0, mirh)
    mid = (x0, 2pad + mirh, x1 - x0, rowh)
    BandLayout(circ, mirror, mid, map, pos, tach, timing)
end
const BAND = band_layout(W, max(BAND_H, 120))

const BANDMIR_FBO = Ref{Tuple{UInt32,UInt32}}((UInt32(0), UInt32(0)))
const BANDMIR_PROG = Ref{UInt32}(0)
const BANDMIR_CAM = Ref{Any}(nothing)
"JM_PICK_BANDMIR: the screen pixels inside the band's mirror, as pixels of its (left-right flipped) picture."
function bandmir_pick_report(label)
    (isempty(PICKS_BANDMIR) || BANDMIR_CAM[] === nothing) && return nothing
    (x, y, w, h) = BAND.mirror; mw = round(Int, w); mh = round(Int, h)
    pk = [(mw*(1 - (px - x)/w), mh*(py - y)/h) for (px, py) in PICKS_BANDMIR]
    pick_report(BANDMIR_CAM[][1], BANDMIR_CAM[][2], label * " (band mirror)"; picks = pk, w = mw, h = mh)
end
const BANDMIR_VS = """
#version 330 core
const vec2 P[3]=vec2[3](vec2(-1,-1),vec2(3,-1),vec2(-1,3));
out vec2 uv; void main(){ vec2 p=P[gl_VertexID]; uv=p*0.5+0.5; gl_Position=vec4(p,0,1); }"""

const BANDMIR_FS = "#version 330 core\nin vec2 uv; out vec4 o; uniform sampler2D uTex; void main(){ o=vec4(texture(uTex, vec2(1.0 - uv.x, uv.y)).rgb, 1.0); }"
const BANDMIR_FOVX = deg2rad(parse(Float64, get(ENV, "JM_HUDBAND_MIRROR_FOV", "62")))
const PROJ_BANDMIR = let a = BAND.mirror[3]/BAND.mirror[4]
    Render.perspective_revz(Float32(2atan(tan(BANDMIR_FOVX/2)/a)), Float32(a), 0.35f0, 3000f0)
end

"""The band mirror's camera: the driver's eye (raised 0.25 m, clear of the helmet and roll hoop), looking straight back
along the car's heading, level -- a steady picture like reference's virtual mirror (mirrored left-right when drawn)."""
function band_mirror_camera(cs)
    θ = Float32(cs.θ); fx = cos(θ); fz = -sin(θ)
    ex = Float32(EYE_RAW[1] + BODY_OFF[1]); ey = Float32(EYE_RAW[2] + BODY_OFF[2] + 0.25)
    eye = Float32[cs.x + ex*fx, cs.y + ey, -cs.z + ex*fz]
    ctr = eye + Float32[-10f0*fx, -0.25f0, -10f0*fz]
    PROJ_BANDMIR * Render.lookat(eye, ctr, Float32[0f0, 1f0, 0f0]), eye
end

"The rear view into the band mirror's texture (before the main pass; the world and every other car, not yours)."
function band_mirror_render!(drawworld, cs, lightVP)
    BANDMIR_ON || return nothing
    mw = round(Int, BAND.mirror[3]); mh = round(Int, BAND.mirror[4])
    BANDMIR_FBO[][1] == 0 && (BANDMIR_FBO[] = Render.make_mirror_fbo(mw, mh))
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE); glDepthFunc(GL_GEQUAL); glClearDepth(0.0)
    glBindFramebuffer(GL_FRAMEBUFFER, BANDMIR_FBO[][1]); glViewport(0, 0, mw, mh)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    mvp, meye = band_mirror_camera(cs)
    BANDMIR_CAM[] = (mvp, meye)
    Render.draw_sky(skyprog, skyvao, inv(mvp), meye, LIGHTDIR; cloud = GRADE.cloud, zenith = GRADE.zenith, horizon = GRADE.horizon)
    Render.set_scene_uniforms(prog, meye; fognear = 400f0, fogfar = 2800f0, fogcol = GRADE.horizon, suncol = GRADE.suncol,
                              ambsky = GRADE.ambsky, sat = GRADE.sat)
    Render.bind_shadow(prog, shadowtex, lightVP)
    _a2c = get(ENV, "JM_BANDMIR_A2C", "1") != "0"
    _a2c || glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE)
    drawworld(mvp, meye, false)
    _a2c || glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE)
    glBindFramebuffer(GL_FRAMEBUFFER, 0)
    nothing
end

"Draw the mirror texture into its band box (a framed, full-colour picture)."
function band_mirror_blit!(W::Int, H::Int)
    (BANDMIR_ON && BANDMIR_FBO[][2] != 0) || return nothing
    if BANDMIR_PROG[] == 0
        p = glCreateProgram(); glAttachShader(p, Render.compile(BANDMIR_VS, GL_VERTEX_SHADER))
        glAttachShader(p, Render.compile(BANDMIR_FS, GL_FRAGMENT_SHADER)); glLinkProgram(p); BANDMIR_PROG[] = p
    end
    (x, y, w, h) = BAND.mirror
    glDisable(GL_DEPTH_TEST)
    glViewport(round(Int, x), H - round(Int, y + h), round(Int, w), round(Int, h))
    glUseProgram(BANDMIR_PROG[]); glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, BANDMIR_FBO[][2])
    glUniform1i(Render.uloc(BANDMIR_PROG[], "uTex"), 0)
    glBindVertexArray(hudvao); glDrawArrays(GL_TRIANGLES, 0, 3)
    glViewport(0, 0, W, H); glEnable(GL_DEPTH_TEST)
    nothing
end

"""The map in its box at MAP_SPAN metres across: whole and centred if the track fits, else the window around the car at
(cx, cz) (scissored to the box). Ribbon, start tick, a disc per car (cars = (id, x, z, pos); id 0 = you, ringed)."""
function trackmap_draw_fixed!(bv, tv, font, m::TrackMap, bx, by, bw, bh, cars, sfx, sfz, cx, cz)
    m.ok || return nothing
    ppm = bw/MAP_SPAN
    tw = m.sc*ppm; th = m.sc/m.aspect*ppm
    fits = tw <= bw && th <= bh
    MAP_FOLLOW[] = !fits
    if fits
        ox = bx + (bw - tw)/2; oy = by + (bh - th)/2
    else
        q = map_pt(m, cx, cz); ox = bx + bw/2 - q[1]*m.sc*ppm; oy = by + bh/2 - q[2]*m.sc*ppm
    end
    k = m.sc*ppm
    n = length(m.ux); xs = [ox + m.ux[i]*k for i in 1:n]; ys = [oy + m.uy[i]*k for i in 1:n]
    rw = clamp(bh/16, 3.0, 12.0)
    hpolyline!(bv, xs, ys, rw + 3.0, (0.92, 0.92, 0.92))
    hpolyline!(bv, xs, ys, rw, (0.05, 0.04, 0.04))
    p = map_pt(m, sfx, sfz); px = ox + p[1]*k; py = oy + p[2]*k
    Render.hquad!(bv, px - 1.5, py - rw, 3.0, 2rw, (0.95, 0.95, 0.95))
    r = clamp(bh/18, 3.5, 11.0)
    for pass in 1:2, (id, x, z, pos) in cars
        (pass == 2) == (id == 0) || continue
        q = map_pt(m, x, z); qx = ox + q[1]*k; qy = oy + q[2]*k
        (bx - r <= qx <= bx + bw + r && by - r <= qy <= by + bh + r) || continue
        rr = id == 0 ? 1.35r : r
        id == 0 && hdisc!(bv, qx, qy, rr + 2.0, (0.98, 0.98, 0.98))
        hdisc!(bv, qx, qy, rr, (0.98, 0.78, 0.10))
        if font !== nothing && pos > 0
            t = string(pos); w_ = Render.text_width(font, t)
            Render.text!(tv, font, qx - w_/2, qy - font.lineh/2, t, (0.05, 0.05, 0.05))
        end
    end
    nothing
end

"Surface temperature -> bar colour: blue below 45 C, green 60-90, yellow 110, red from 130."
function tt_colour(T)
    k = ((45.0, (0.20, 0.45, 1.00)), (60.0, (0.15, 0.85, 0.30)), (90.0, (0.25, 0.90, 0.25)), (110.0, (0.95, 0.85, 0.15)), (130.0, (1.0, 0.20, 0.10)))
    T <= k[1][1] && return k[1][2]
    for j in 2:length(k)
        if T <= k[j][1]
            f = (T - k[j-1][1])/(k[j][1] - k[j-1][1]); a = k[j-1][2]; b = k[j][2]
            return (a[1] + f*(b[1]-a[1]), a[2] + f*(b[2]-a[2]), a[3] + f*(b[3]-a[3]))
        end
    end
    k[end][2]
end
"The tyre's ring colour: tt_colour of its mean surface temperature (TD261029-2)."
tyre_ring(i) = tt_colour((TT_SURF[i, 1] + TT_SURF[i, 2] + TT_SURF[i, 3])/3)

const GAUGE_YEL = (1.0, 0.86, 0.10); const GAUGE_GRN = (0.25, 0.90, 0.30)
fmt_lapg(t) = (isfinite(t) && t > 0) ? (m = floor(Int, t/60); ms = round(Int, 1000*(t - 60m)); ms == 60_000 && (m += 1; ms = 0);
                                         string(m, ":", lpad(ms ÷ 1000, 2, '0'), ".", lpad(ms % 1000, 3, '0'))) : "--:--.---"
fmt_split(t) = isfinite(t) ? (t < 0 ? "-" : "+") * (ms = round(Int, 1000abs(t)); string(ms ÷ 1000, ".", lpad(ms % 1000, 3, '0'))) : "--.---"
band_textscale(h) = h/200
"""Classic's rev counter in the box (x, y, w, h): the dial, its ticks to 10,000 rpm (red from 9,000), the needle, the gear in
its yellow box, RPM, water/oil (F) and the speed (TD261029-2). Quads into bv, text into tv."""
function band_tach!(bv, tv, box, rpm, gear, kph, water, oil)
    (x, y, w, h) = box; R = min(w, h)/2; cx = x + w/2; cy = y + h/2; ts = R/80
    Render.hdisc!(bv, cx, cy, R, (0.62, 0.63, 0.66); seg = 40)
    Render.hdisc!(bv, cx, cy, 0.93R, (0.30, 0.31, 0.33); seg = 40)
    Render.hdisc!(bv, cx, cy, 0.88R, (0.07, 0.07, 0.08); seg = 40)
    a0 = deg2rad(215.0); sweep = deg2rad(250.0)
    ang(r) = a0 - sweep*clamp(r/10_000, 0.0, 1.0)
    for k in 0:20
        r = 500k; a = ang(r); major = iseven(k); r0 = major ? 0.66R : 0.73R
        c = r >= 9000 ? (0.92, 0.12, 0.10) : (0.92, 0.92, 0.92)
        Render.hline!(bv, cx + r0*cos(a), cy - r0*sin(a), cx + 0.84R*cos(a), cy - 0.84R*sin(a), major ? 2.6 : 1.4, c)
    end
    a = ang(rpm)
    Render.hline!(bv, cx - 0.10R*cos(a), cy + 0.10R*sin(a), cx + 0.80R*cos(a), cy - 0.80R*sin(a), 3.0, (0.95, 0.95, 0.95))
    Render.hdisc!(bv, cx, cy, 0.07R, (0.55, 0.55, 0.58))
    gs = gear == 0 ? "N" : gear < 0 ? "R" : string(gear)
    gw = Render.text_width(FONT, gs; scale = 1.1ts)
    Render.hquad!(bv, cx - gw/2 - 4, cy - 0.62R - 2, gw + 8, 1.1ts*FONT.lineh + 2, (0.16, 0.15, 0.10))
    Render.text!(tv, FONT, cx - gw/2, cy - 0.62R, gs, GAUGE_YEL; scale = 1.1ts)
    rl = "RPM"; Render.text!(tv, FONT, cx - Render.text_width(FONT, rl; scale = 0.55ts)/2, cy - 0.22R, rl, (0.70, 0.70, 0.72); scale = 0.55ts)
    tl = string(round(Int, water*9/5 + 32), "/", round(Int, oil*9/5 + 32))
    Render.text!(tv, FONT, cx + 0.05R, cy + 0.20R, tl, GAUGE_GRN; scale = 0.7ts)
    sp = string(round(Int, kph)); sw = Render.text_width(FONT, sp; scale = 1.15ts); kw = Render.text_width(FONT, " KPH"; scale = 0.75ts)
    Render.text!(tv, FONT, cx - (sw + kw)/2, cy + 0.50R, sp, GAUGE_YEL; scale = 1.15ts)
    Render.text!(tv, FONT, cx - (sw + kw)/2 + sw, cy + 0.50R + 0.3ts*FONT.lineh, " KPH", GAUGE_YEL; scale = 0.75ts)
    nothing
end
"Label: value rows, right-aligned labels at the colon, in a dark box (Classic's yellow text)."
function band_rows!(bv, tv, box, rows; scale = 0.9)
    (x, y, w, h) = box
    Render.hquad!(bv, x, y, w, h, (0.10, 0.10, 0.12))
    lh = scale*FONT.lineh; y0 = y + (h - length(rows)*lh)/2
    lw = maximum(Render.text_width(FONT, r[1]; scale = scale) for r in rows)
    for (k, (lab, val)) in enumerate(rows)
        yy = y0 + (k - 1)*lh
        isempty(lab) || Render.text!(tv, FONT, x + 8 + lw - Render.text_width(FONT, lab; scale = scale), yy, lab, GAUGE_YEL; scale = scale)
        Render.text!(tv, FONT, x + 14 + lw, yy, val, GAUGE_YEL; scale = scale)
    end
    nothing
end

const BAND_NUMSC = parse(Float64, get(ENV, "JM_HUD_DELTA_SCALE", "1.6"))
const SECD_RANGE = parse(Float64, get(ENV, "JM_HUD_SECDELTA_RANGE", "1.0"))
const MAP_NUMS = get(ENV, "JM_HUD_MAPNUM", "0") != "0"
"""The current section and when it began: the driving car's lap position, or in a replay the recorded player's (the
replay's lap clock; a replay without one shows no name -- the live car sits on the grid, which read as Parabolica)."""
function seg_step!(cs)
    (isempty(SECTIONS) || SEGNAME_SECS <= 0) && return nothing
    rep = !isempty(REPLAY_FILE_R[])
    rep && !REPCLK_ON[] && return nothing
    s, t = rep ? (mod(REPCLK_P[][2], 1.0)*LAPLEN, REPCLK_P[][1]) : (cs.lapdist, cs.t)
    SEG[] = section_update(SECTIONS, s, LAPLEN, t, SEG[]...)
    SEG_T[] = t
    nothing
end
const SEG_T = Ref(0.0)
seg_showing(cs) = !isempty(SEGNAME_FORCE) || (!isempty(SECTIONS) && SEGNAME_SECS > 0 && 0 <= SEG_T[] - SEG[][3] < SEGNAME_SECS)

const SECD = Ref((-1, NaN, 0.0, -1))
"""The delta since the bar came back in this section (it is called only while the bar shows, so the first call in a new
section -- the name just gone -- is the zero). Across the start line the lap delta restarts; this one stays continuous."""
function section_delta(laps)
    (k, base, lastd, lastlap) = SECD[]; d = LAPCLOCK.delta; seg = SEG[][1]
    LAPCLOCK.dvalid || (SECD[] = (seg, NaN, d, laps); return NaN)
    if seg != k || isnan(base)
        base = d
    elseif laps != lastlap
        base = d - (lastd - base)
    end
    SECD[] = (seg, base, d, laps)
    d - base
end

const BAND_BV = Float32[]; const BAND_MV = Float32[]; const BAND_TV = Float32[]; const BAND_MT = Float32[]; const BAND_GT = Float32[]
"""The whole band: black ground, mirror, traction circles, lap rows + sectors, delta bar, race order, map."""
function hudband_draw!(cs, W, H, AICARS, best_lap, last_lap, lap_t0, race_done, race_go, standings, texthud_name, ai_poses, tc)
    FONT === nothing && return nothing
    BH = Float64(BAND_H); L = BAND
    bv = empty!(BAND_BV); Render.hquad!(bv, 0.0, 0.0, Float64(W), BH, (0.0, 0.0, 0.0))
    (mx, my, mw, mh) = L.mirror
    Render.hquad!(bv, mx - 2, my - 2, mw + 4, mh + 4, (0.35, 0.35, 0.38))

    (cx0, cy0, cw, ch) = L.circ; cell = cw/2; baseR = cell/(2*1.75)
    ttl = BAND_TT_ON && (isempty(REPLAY_FILE_R[]) || REP_TT_LIVE[])
    for (i, (ix, iy)) in enumerate(((0, 0), (1, 0), (0, 1), (1, 1)))
        is_loose(WHEEL_NAMES[i]) && continue
        ttl ? Render.htraction!(bv, cx0 + (ix + 0.5)*cell, cy0 + (iy + 0.5)*cell, baseR, tc[i]; ring = tyre_ring(i), ringw = 3.5) :
              Render.htraction!(bv, cx0 + (ix + 0.5)*cell, cy0 + (iy + 0.5)*cell, baseR, tc[i])
    end
    gtv = empty!(BAND_GT)
    if BAND_GAUGE_ON
        band_tach!(bv, gtv, L.tach, cs.rpm, cs.gear, cs.v*3.6, DIALV.water, DIALV.oiltemp)
        st0 = (IS_RACE && !isempty(AICARS)) ? standings() : nothing
        pk = st0 === nothing ? 1 : something(findfirst(e -> e[1] == 0, st0), 1)
        nf = st0 === nothing ? 1 : length(st0)
        lapn = REPCLK_ON[] ? REPCLK_LAPS[] : cs.laps
        band_rows!(bv, gtv, L.pos, (("POS:", "$pk/$nf"), ("LAPS:", "$lapn/" * (IS_RACE ? string(RACE_LAPS) : "--"))); scale = 1.0band_textscale(L.pos[4]))
        c = LAPCLOCK; k = c.sec_lastdone; ns = length(c.sec_best)
        cur = REPCLK_ON[] ? REPCLK_CUR[] : (race_go[] && !race_done ? cs.t - lap_t0 : NaN)
        sk = 1 <= k <= ns
        stime = sk ? (isfinite(c.sec_now[k]) ? c.sec_now[k] : c.sec_last[k]) : NaN
        sdel = sk ? (isfinite(c.sec_dnow[k]) ? c.sec_dnow[k] : c.sec_dlast[k]) : NaN
        lastl = REPCLK_ON[] ? REPCLK_LAST[] : last_lap
        band_rows!(bv, gtv, L.timing, (("CURRENT:", fmt_lapg(cur)), (sk ? "SECTOR $k:" : "SECTOR 1:", fmt_lapg(stime)),
                                       ("SELF SPLIT:", fmt_split(sdel)), ("BEST SPLIT:", sk ? fmt_lapg(c.sec_best[k]) : "--:--.---"),
                                       ("", isfinite(lastl) && lastl > 0 ? fmt_lapg(lastl) : "-------")); scale = 0.95band_textscale(L.timing[4]))
    end
    Render.hud_draw(hudprog, hudvao, hudvbo, bv, W, H)
    BAND_GAUGE_ON && Render.text_draw(textprog, textvao, textvbo, FONT, gtv, W, H)
    band_mirror_blit!(W, H)

    tv = empty!(BAND_TV); xb = empty!(HUDX_BV)
    (x0, y0, wmid, hmid) = L.mid
    rc = REPCLK_ON[]
    st = (IS_RACE && !isempty(AICARS)) ? standings() : nothing
    dcx = x0 + wmid/2; HUD_DCX[] = dcx
    numw = Render.text_width(FONT, "+00.00"; scale = BAND_NUMSC) + 16.0
    dw = max(120.0, wmid - 2numw); dh = round(0.55hmid); dy = y0 + (hmid - dh)/2
    forced = !isnan(HUD_FORCE_DELTA)
    if (!forced || !isempty(SEGNAME_FORCE)) && seg_showing(cs)
        msg = isempty(SEGNAME_FORCE) ? SEG[][2] : SEGNAME_FORCE; sw0 = Render.text_width(SEGFONT, msg)
        ssc = min(1.0, (wmid - 8)/max(sw0, 1.0)); sw = ssc*sw0
        st_ = empty!(HUD_ST); Render.text!(st_, SEGFONT, clamp(dcx - sw/2, x0, x0 + wmid - sw), y0 + (hmid - ssc*SEGFONT.lineh)/2, msg, (0.95, 0.95, 0.95); scale = ssc)
        Render.text_draw(textprog, textvao, textvbo, SEGFONT, st_, W, H)
    elseif HUD_DELTA_ON && ((LAPCLOCK.dvalid && (rc || race_go[] && !race_done)) || forced)
        d = forced ? HUD_FORCE_DELTA : section_delta(rc ? REPCLK_LAPS[] : cs.laps)
        isnan(d) || delta_bar!(xb, tv, FONT, dcx, dy, dw, dh, d; scale = BAND_NUMSC, range = forced ? DELTA_RANGE : SECD_RANGE)
    end
    tend = x0
    Render.hud_draw(hudprog, hudvao, hudvbo, xb, W, H)
    Render.text_draw(textprog, textvao, textvbo, FONT, tv, W, H)
    if !isnan(HUDBAND_DIAG) && !HUDBAND_DIAG_DONE[] && cs.t >= HUDBAND_DIAG
        HUDBAND_DIAG_DONE[] = true
        rg = [is_loose(WHEEL_NAMES[i]) ? "off" : join(round.(Int, 255 .* tyre_ring(i)), ",") for i in 1:4]
        println("  [hudband] H3 ", H3, " band ", BAND_H, " circ ", round.(Int, L.circ), " tach ", round.(Int, L.tach), " pos ", round.(Int, L.pos),
                " timing ", round.(Int, L.timing), " rings ", join(rg, ";"), " mirror ", round.(Int, L.mirror), " mid ",
                round.(Int, L.mid), " map ", round.(Int, L.map), " delta ", round.(Int, (dcx, dy, dw, dh)),
                " timing_end ", round(Int, tend), " follow ", MAP_FOLLOW[]); flush(stdout)
    end

    if HUD_MAP_ON && TRACKMAP.ok
        (bx, by, bw, bh) = L.map
        cars = empty!(HUD_MAPCARS)
        if st !== nothing
            for (i, e) in enumerate(st)
                if e[1] == 0
                    push!(cars, (0, cs.x, cs.z, i))
                elseif e[1] <= length(ai_poses)
                    q = ai_poses[e[1]]; push!(cars, (e[1], Float64(q[1]), Float64(q[3]), i))
                end
            end
        else
            push!(cars, (0, cs.x, cs.z, 0))
        end
        sf = line_xz(TRACKMAP, 0.0); mv = empty!(BAND_MV); mt = empty!(BAND_MT)
        trackmap_draw_fixed!(mv, mt, MAP_NUMS ? FONT : nothing, TRACKMAP, bx, by, bw, bh, cars, sf[1], sf[2], cs.x, cs.z)
        glEnable(GL_SCISSOR_TEST); glScissor(round(Int, bx), H - round(Int, by + bh), round(Int, bw), round(Int, bh))
        Render.hud_draw(hudprog, hudvao, hudvbo, mv, W, H)
        Render.text_draw(textprog, textvao, textvbo, FONT, mt, W, H)
        glDisable(GL_SCISSOR_TEST)
    end
    nothing
end

const BAND_EVT_SECS = parse(Float64, get(ENV, "JM_HUD_EVENT_SECS", "6"))
const BAND_EVT = Ref((:none, -1e9))
const BAND_LAPS = Ref(-1)
const BAND_EVT_TEST = let f = split(get(ENV, "JM_HUD_EVENT_AT", ""), ":")
    length(f) == 2 && haskey(ENV, "JM_SMOKE") ? (parse(Float64, f[1]), Symbol(f[2])) : nothing
end
const BAND_EVT_TESTED = Ref(false)
const BAND_OV = Float32[]; const BAND_OT = Float32[]

"A race lap begins as you cross the line: the last lap -> the flag, any other -> the leaderboard."
function band_events_step!(cs, race_go, race_done)
    if BAND_LAPS[] < 0 || !race_go[]
        BAND_LAPS[] = cs.laps
    elseif cs.laps > BAND_LAPS[]
        BAND_LAPS[] = cs.laps
        if IS_RACE && !race_done
            nxt = cs.laps + 1
            nxt == RACE_LAPS ? (BAND_EVT[] = (:flag, cs.t)) : nxt < RACE_LAPS && (BAND_EVT[] = (:board, cs.t))
            BAND_EVT[][1] === :none || (println("  [hudband] ", BAND_EVT[][1], " at t=", round(cs.t, digits = 2), " (lap ", nxt, " of ", RACE_LAPS, ")"); flush(stdout))
        end
    end
    if BAND_EVT_TEST !== nothing && !BAND_EVT_TESTED[] && cs.t >= BAND_EVT_TEST[1]
        BAND_EVT_TESTED[] = true; BAND_EVT[] = (BAND_EVT_TEST[2], cs.t)
        println("  [hudband] ", BAND_EVT_TEST[2], " at t=", round(cs.t, digits = 2), " (test)"); flush(stdout)
    end
    nothing
end

"""Over the 3D view: the chequered flag (middle left) or Classic's leaderboard (top left: laps to go, your position, the
order), while the event is fresh."""
function band_overlay_draw!(cs, W, H, AICARS, standings, texthud_name)
    (what, t0) = BAND_EVT[]
    (what === :none || cs.t - t0 > BAND_EVT_SECS || cs.t < t0) && return nothing
    v = empty!(BAND_OV)
    if what === :flag
        fw = round(0.11W); fh = round(0.62fw); fx = round(0.05W); fy = band_px() + round(0.45view_h() - fh/2)
        Render.hquad!(v, fx - 10, fy - 6, 6, fh*2.2, (0.55, 0.45, 0.30))
        Render.hquad!(v, fx - 4, fy - 4, fw + 8, fh + 8, (0.08, 0.08, 0.08))
        nx, ny = 6, 4; sw = fw/nx; sh = fh/ny
        for i in 0:nx-1, j in 0:ny-1
            Render.hquad!(v, fx + i*sw, fy + j*sh, sw, sh, iseven(i + j) ? (0.97, 0.97, 0.97) : (0.06, 0.06, 0.06))
        end
        Render.hud_draw(hudprog, hudvao, hudvbo, v, W, H)
    elseif what === :board && FONT !== nothing
        st = (IS_RACE && !isempty(AICARS)) ? standings() : Tuple{Int,Float64}[(0, 0.0)]
        me = something(findfirst(e -> e[1] == 0, st), 1)
        togo = max(RACE_LAPS - cs.laps, 0); lh = FONT.lineh; sc = 1.3
        rows = String["LAP $(min(cs.laps + 1, RACE_LAPS)) OF $(RACE_LAPS)   -   $(togo) TO GO", "POSITION $(me) OF $(length(st))"]
        for (i, e) in enumerate(st)
            i > 8 && break
            push!(rows, string(i, "  ", texthud_name(e[1])))
        end
        bw = maximum(r -> Render.text_width(FONT, r; scale = sc), rows) + 32; bh = length(rows)*sc*lh + 24
        bx = 24.0; by = band_px() + 24.0
        Render.hquad!(v, bx, by, bw, bh, (0.04, 0.04, 0.06)); Render.hquad!(v, bx, by, 6, bh, (1.0, 0.82, 0.25))
        Render.hud_draw(hudprog, hudvao, hudvbo, v, W, H)
        tv = empty!(BAND_OT)
        for (k, r) in enumerate(rows)
            col = k == 1 ? (1.0, 0.82, 0.25) : k == 2 ? (0.95, 0.95, 0.95) : (k - 2 == me ? (1.0, 0.82, 0.35) : (0.80, 0.82, 0.86))
            Render.text!(tv, FONT, bx + 18, by + 12 + (k - 1)*sc*lh, r, col; scale = sc)
        end
        Render.text_draw(textprog, textvao, textvbo, FONT, tv, W, H)
    end
    nothing
end

"A replay's recorded tyres (fx, fy, gx, gy, xi per wheel; REPLAY-6) as the band's traction circles (Fx, Fy, grip)."
band_tc(t) = ntuple(i -> (Float64(t[i][1]), Float64(t[i][2]), max((Float64(t[i][3]) + Float64(t[i][4]))/2, 1e-3)), 4)

"Where a message goes: over the band's mirror (HUDBAND-1), else the old place over the 3D view."
band_msg_y(H, old) = band_shown() ? BAND.mirror[2] + 10.0 : old
