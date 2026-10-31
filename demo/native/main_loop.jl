

"""The E64 mirror pass: the rear view into the mirror RTT, one camera per disc (see the comments at the call site)."""
function mirror_render!(drawworld, cs, cam_pitch, cam_roll, frames, _mir_alt, lightVP, bodyModel, wheelmat, loosemat)
        glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE); glDepthFunc(GL_GEQUAL); glClearDepth(0.0)
        glBindFramebuffer(GL_FRAMEBUFFER, mirfbo); glViewport(0,0,MIRW,MIRH)
        _mir_half = _mir_alt ? (frames % 2) : -1
        if _mir_half >= 0
            glEnable(GL_SCISSOR_TEST); glScissor(_mir_half == 0 ? 0 : MIRW÷2, 0, MIRW÷2, MIRH)
        end
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        _mir_half >= 0 && glDisable(GL_SCISSOR_TEST)

        if MIRROR_HALVES > 0

        for (x0, side) in (_mir_half == 0 ? ((0, 1),) : _mir_half == 1 ? ((MIRW÷2, -1),) :
                           MIRROR_HALVES == 1 ? ((0, 1),) : ((0, 1), (MIRW÷2, -1)))
            glViewport(x0, 0, MIRW÷2, MIRH)
            mvp, meye = mirror_camera(cs, cam_pitch, cam_roll, side)
            Render.draw_sky(skyprog, skyvao, inv(mvp), meye, LIGHTDIR;
                            cloud = GRADE.cloud, zenith = GRADE.zenith, horizon = GRADE.horizon)
            Render.set_scene_uniforms(prog, meye; fognear=400f0, fogfar=2800f0,
                                      fogcol=GRADE.horizon, suncol=GRADE.suncol, ambsky=GRADE.ambsky, sat=GRADE.sat)
            Render.bind_shadow(prog, shadowtex, lightVP)
            drawworld(mvp, meye, !MIRROR_UNFLIP)
            for (i,it) in enumerate(carItems); _in_car_range(i) || continue; Render.draw(prog, it, mvp, bodyModel; bright=1.2, spec=0.08, ambfill=0.78); end
            for (wx,wz,steer,r,nm) in WHEELS, it in WHEELITEMS[nm]
                is_loose(nm) && continue
                Render.draw(prog, it, mvp, wheelmat(wx,wz,steer,r,nm); bright=1.0, ambfill=0.75)
            end
            for (lx,ly,lz,_,_,_,sp,_,nm) in LOOSE_WHEELS, it in loose_items(nm)
                Render.draw(prog, it, mvp, loosemat(lx,ly,lz,sp); bright=1.0, ambfill=0.75)
            end
        end
        end
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
    nothing
end

"""TEXTHUD-1 (GOLDMATCH-JR-1 S1): the gold's timing overlay -- lap rows top-left, Track Position top-right (LOOPSPLIT-1 S3;
the code is unchanged, the loop locals it reads are arguments)."""
function texthud_draw!(cs, W, H, AICARS, best_lap, fmt_lap, lap_t0, last_lap, player_finpos, race_done, race_go, standings, texthud_name, ai_poses, tc = ntuple(_ -> (0.0, 0.0, 1.0), 4))
        if FONT !== nothing

            nrows = (IS_RACE && !isempty(AICARS)) ? max(3, min(7, 1 + length(AICARS)) + 1) : 3
            fy = 6.0
            if HUDBAND_ON
                seg_step!(cs)
                band_shown() && hudband_draw!(cs, W, H, AICARS, best_lap, last_lap, lap_t0, race_done, race_go, standings, texthud_name, ai_poses, tc)
                band_events_step!(cs, race_go, race_done)
                band_overlay_draw!(cs, W, H, AICARS, standings, texthud_name)
            else
            bv = empty!(HUD_BV); Render.hquad!(bv, 0.0, 0.0, Float64(W), nrows*FONT.lineh + 12.0, (0.0, 0.0, 0.0))
            Render.hud_draw(hudprog, hudvao, hudvbo, bv, W, H)
            tv = empty!(HUD_TV); fy = 6.0; fx = 8.0
            cur = (race_go[] && !race_done) ? cs.t - lap_t0 : (SMOKE ? 23.27 : 0.0)
            Render.text!(tv, FONT, fx, fy,            "I  " * lapfmt(cur),                              (1.0, 0.45, 1.0))
            Render.text!(tv, FONT, fx, fy + FONT.lineh, "B  " * lapfmt(SMOKE ? 92.1 : best_lap),           (0.75, 0.75, 0.75))
            Render.text!(tv, FONT, fx, fy + 2FONT.lineh, "L  " * lapfmt(SMOKE ? 94.3 : last_lap),          (0.45, 0.95, 0.45))
            if IS_RACE && !isempty(AICARS)
                st = standings(); ppy = 0.0
                for e in st; e[1] == 0 && (ppy = e[2]); end
                lapsN = length(st) > 0 ? cs.laps + 1 : 1
                tx = W - 8.0 - 300.0
                Render.text!(tv, FONT, tx, fy, "Track Position ($(lapsN)L)", (0.95, 0.95, 0.95))
                for (i, e) in enumerate(st)
                    i > 7 && break
                    gapm = round(Int, (e[2] - ppy) * LAPLEN)
                    col = e[1] == 0 ? (1.0, 0.82, 0.35) : (0.95, 0.95, 0.95)
                    Render.text!(tv, FONT, tx, fy + i*FONT.lineh, string(i, " ", texthud_name(e[1])), col)
                    Render.text!(tv, FONT, tx + 190.0, fy + i*FONT.lineh, (gapm >= 0 ? "+" : "") * string(gapm) * "m", col)
                end
            end

            xb = empty!(HUDX_BV); dw = min(500.0, 0.26W)
            st2 = (IS_RACE && !isempty(AICARS)) ? standings() : nothing
            tend = 160.0
            if HUD_TIMING_ON && !isempty(LAPCLOCK.secs)
                pos = 0
                if st2 !== nothing; for (i, e) in enumerate(st2); e[1] == 0 && (pos = i); end; end
                tend = timing_cols!(tv, FONT, LAPCLOCK, 170.0, fy, IS_RACE ? min(cs.laps + 1, RACE_LAPS) : cs.laps + 1, IS_RACE ? RACE_LAPS : 0,
                                    pos, 1 + length(AICARS); colw = clamp(W/24, 60.0, 92.0)) - 10.0
            end
            dcx = max(W/2, tend + 12.0 + dw/2); HUD_DCX[] = dcx
            if HUD_DELTA_ON && ((LAPCLOCK.dvalid && race_go[] && !race_done) || !isnan(HUD_FORCE_DELTA))
                delta_bar!(xb, tv, FONT, dcx, fy, dw, FONT.lineh - 6.0, isnan(HUD_FORCE_DELTA) ? LAPCLOCK.delta : HUD_FORCE_DELTA)
            end
            if HUD_MAP_ON && st2 !== nothing && TRACKMAP.ok
                bandh = nrows*FONT.lineh + 12.0
                mr = W - 8.0 - 300.0 - 16.0; ml = dcx + dw/2 + 70.0
                mw = min(2.2*(bandh - 12.0), mr - ml)
                if mw > 60.0
                    cars = empty!(HUD_MAPCARS)
                    for (i, e) in enumerate(st2)
                        if e[1] == 0
                            push!(cars, (0, cs.x, cs.z, i))
                        elseif e[1] <= length(ai_poses)
                            q = ai_poses[e[1]]; push!(cars, (e[1], Float64(q[1]), Float64(q[3]), i))
                        end
                    end
                    sf = line_xz(TRACKMAP, 0.0)
                    trackmap_draw!(xb, tv, FONT, TRACKMAP, mr - mw, 6.0, mw, bandh - 12.0, cars, sf[1], sf[2])
                end
            end
            Render.hud_draw(hudprog, hudvao, hudvbo, xb, W, H)
            Render.text_draw(textprog, textvao, textvbo, FONT, tv, W, H)
            end

            if !HUDBAND_ON && !isempty(SECTIONS) && SEGNAME_SECS > 0
                seg_step!(cs)
                if seg_showing(cs)
                    msg = isempty(SEGNAME_FORCE) ? SEG[][2] : SEGNAME_FORCE; tw = Render.text_width(SEGFONT, msg)
                    bandh = nrows*FONT.lineh + 12.0
                    sy = HUDBAND_ON ? BAND.mid[2] + 4.3FONT.lineh : max(HUD_DELTA_ON ? fy + FONT.lineh : 2.0, (bandh - SEGFONT.lineh)/2)
                    st = empty!(HUD_ST); Render.text!(st, SEGFONT, clamp(HUD_DCX[] - tw/2, 2.0, W - tw - 2.0), sy, msg, (0.95, 0.95, 0.95))
                    Render.text_draw(textprog, textvao, textvbo, SEGFONT, st, W, H)
                end
            end

            netmsg = NETLINK === nothing ? "" : NetPlay.status_text(NETLINK)
            (isempty(netmsg) && time() - NET_NOTE[][2] < 8.0) && (netmsg = NET_NOTE[][1])
            if !isempty(netmsg)
                lines = split(replace(netmsg, ". " => ".\n"), '\n'); sc = 1.6
                lw = maximum(l -> Render.text_width(FONT, l; scale=sc), lines); lh = sc*FONT.lineh + 4
                bx = (W - lw)/2 - 20; by = band_msg_y(H, H*0.40)
                bg = Float32[]; Render.hquad!(bg, bx, by - 10, lw + 40, length(lines)*lh + 20, (0.0, 0.0, 0.0))
                Render.hud_draw(hudprog, hudvao, hudvbo, bg, W, H)
                fv = Float32[]
                for (k, l) in enumerate(lines)
                    Render.text!(fv, FONT, (W - Render.text_width(FONT, l; scale=sc))/2, by + (k - 1)*lh, l, (1.0, 0.7, 0.15); scale=sc)
                end
                Render.text_draw(textprog, textvao, textvbo, FONT, fv, W, H)
            end

            if IS_RACE && race_done && RACEEND_SECS > 0
                RACEEND_T0[] < 0 && (RACEEND_T0[] = cs.t)
                if !HUDBAND_ON
                sq = 28.0; y0 = HUDBAND_ON ? Float64(BAND_H) : nrows*FONT.lineh + 12.0
                cv = Float32[]
                for (r, yy) in enumerate((y0, y0 + sq))
                    for (c, xx) in enumerate(0.0:sq:Float64(W))
                        Render.hquad!(cv, xx, yy, sq, sq, iseven(r + c) ? (0.05, 0.05, 0.05) : (0.97, 0.97, 0.97))
                    end
                end
                Render.hud_draw(hudprog, hudvao, hudvbo, cv, W, H)
                end
                if cs.t - RACEEND_T0[] < RACEEND_SECS
                    msg = "RACE FINISHED" * (isempty(AICARS) ? "" : " - P$(player_finpos[]) of $(length(AICARS)+1)")
                    sub = "best lap $(fmt_lap(best_lap))   -   " * (INSTANT_ON ? "I or left arrow: instant replay   -   " : "") * "Esc to leave"
                    sc = HUDBAND_ON ? 2.0 : 3.0; tw = Render.text_width(FONT, msg; scale=sc); sw = Render.text_width(FONT, sub; scale=1.4)
                    bx = (W - tw)/2 - 24; by = band_msg_y(H, H*0.30)
                    bg = Float32[]; Render.hquad!(bg, bx, by - 10, tw + 48, sc*FONT.lineh + 1.4*FONT.lineh + 28, (0.0, 0.0, 0.0))
                    Render.hud_draw(hudprog, hudvao, hudvbo, bg, W, H)
                    fv = Float32[]
                    Render.text!(fv, FONT, (W - tw)/2, by, msg, (1.0, 0.85, 0.2); scale=sc)
                    Render.text!(fv, FONT, (W - sw)/2, by + sc*FONT.lineh + 8, sub, (0.95, 0.95, 0.95); scale=1.4)
                    Render.text_draw(textprog, textvao, textvbo, FONT, fv, W, H)
                end
            end
        end
    nothing
end

"""REPLAY-6: the per-wheel tyre panel on the replay screen (chase view, player car). (LOOPSPLIT-1 S5: moved unchanged from main())"""
function replay_tyres_draw!(W, H, REPLAY, rep_cam, rep_focus, rep_tyres, rep_tyshow)
        HUDBAND_ON && return nothing
        if REPLAY && rep_tyshow[] && rep_focus[] == 0 && REPLAY_CAMS[rep_cam[]] === :chase && rep_tyres[] !== nothing && any(t -> t[3] + t[4] > 0, rep_tyres[])
            baseR = 18.0; pitch = 2*1.9*baseR + 12; pcx = W/2; pcy = H - pitch - 18.0
            tyv = empty!(HUD_TY); Render.htyres!(tyv, pcx, pcy, baseR, rep_tyres[])
            Render.hud_draw(hudprog, hudvao, hudvbo, tyv, W, H)
            if FONT !== nothing
                tt = empty!(HUD_TYT); lc = (0.75, 0.78, 0.82)
                for (k, nm) in enumerate(("FL", "FR", "RL", "RR"))
                    lx = pcx + (isodd(k) ? -1 : 1)*pitch/2 + (isodd(k) ? -1.9baseR - 4 - Render.text_width(FONT, nm) : 1.9baseR + 4)
                    ly = pcy + (k <= 2 ? -1 : 1)*pitch/2 - FONT.lineh/2
                    Render.text!(tt, FONT, lx, ly, nm, lc)
                end
                cap = "tyres: ring = grip, dot = force"
                Render.text!(tt, FONT, pcx - Render.text_width(FONT, cap)/2, pcy - pitch - 4 - FONT.lineh, cap, lc)
                Render.text_draw(textprog, textvao, textvbo, FONT, tt, W, H)
            end
        end
    nothing
end

"""TD261013-1 (c3): say on screen that a wreck refuses shifts and throttle, with the way out. (LOOPSPLIT-1 S5: moved unchanged from main())"""
function wreck_notice_draw!(W, H, REPLAY)
        if WRECKED[] && !REPLAY && FONT !== nothing && get(ENV, "JM_WRECK_NOTICE", "1") != "0"
            m1 = "CAR WRECKED -- engine and gearbox out"
            m2 = (WHEELOFF_RULE && !isempty(LOOSE_WHEELS)) ? "A wheel has come off -- R: restart the race" : "SHIFT+R: back onto the track     R: restart the race"
            INSTANT_ON && (m2 *= "     I or left arrow: instant replay")
            my = band_msg_y(H, H*0.30)
            bw = max(Render.text_width(FONT, m1), Render.text_width(FONT, m2)) + 40; by = my - 14
            bx = empty!(HUD_TY); Render.hquad!(bx, W/2 - bw/2, by, bw, 2.4*FONT.lineh + 28, (0.06, 0.06, 0.08))
            Render.hud_draw(hudprog, hudvao, hudvbo, bx, W, H)
            tt = empty!(HUD_TYT)
            Render.text!(tt, FONT, W/2 - Render.text_width(FONT, m1)/2, my, m1, (1.0, 0.82, 0.25))
            Render.text!(tt, FONT, W/2 - Render.text_width(FONT, m2)/2, my + 1.4*FONT.lineh, m2, (0.92, 0.92, 0.92))
            Render.text_draw(textprog, textvao, textvbo, FONT, tt, W, H)
        end
    nothing
end

"""PACE-1: present every 2nd refresh while the work time stays over JM_PACE_HI ms. (LOOPSPLIT-1 S5: moved unchanged from main())"""
function pace_step!(frames, now)
        if PACE_ON && (!SMOKE || haskey(ENV, "JM_PACE_TEST"))

            _wk = min(time() - now, 1.5 * PACE_HI)
            if frames > PACE_WARM
                PACE_EMA[] = PACE_EMA[] == 0.0 ? _wk : 0.95*PACE_EMA[] + 0.05*_wk
            end
            if PACE_INT[] == 1 && PACE_EMA[] > PACE_HI
                PACE_INT[] = 2; GLFW.SwapInterval(2); println("  [pace] work ", round(1000*PACE_EMA[], digits=1), " ms -> steady 30 Hz"); flush(stdout)
            elseif PACE_INT[] == 2 && PACE_EMA[] < PACE_LO
                PACE_INT[] = 1; GLFW.SwapInterval(1); println("  [pace] work ", round(1000*PACE_EMA[], digits=1), " ms -> 60 Hz"); flush(stdout)
            end
        end
    nothing
end

"""PERF-3: a young-generation GC every GC_EVERY frames, while the GPU finishes the frame. (LOOPSPLIT-1 S5: moved unchanged from main())"""
function gc_step!(frames)
        if GC_EVERY > 0 && frames % GC_EVERY == 0 && frames > 0
            _tg = time(); GC.gc(false); _tg = time() - _tg
            GC_MS[1] += _tg; GC_MS[2] = max(GC_MS[2], _tg); GC_MS[3] += 1
            if FPSDIAG > 0 && GC_MS[3] >= 20
                println("  [gc] young collections: mean ", round(1000GC_MS[1]/GC_MS[3], digits=2), " ms  max ", round(1000GC_MS[2], digits=2), " ms  (every ", GC_EVERY, " frames)"); flush(stdout)
                fill!(GC_MS, 0.0)
            end
        end
    nothing
end

"""COUNTDOWN start (PO 2026-08-31): the lights, the held field, the green. Returns `lap_t0` (set at the green).
(LOOPSPLIT-1 S6: moved unchanged from main(); the Refs are shared, lap_t0 comes back.)"""
function countdown_step!(cs, AICARS, HOLD_START, ai_release, cd_armed, cd_left, cd_said, cd_t0, phase, race_go, lap_t0)
        if COUNTDOWN > 0 && HOLD_START && !race_go[] && phase[] == :race

            if ARM_AFTER > 0 && !cd_armed[] && cs.t >= ARM_AFTER
                cd_armed[] = true; cd_t0[] = cs.t
                println("  → COUNTDOWN ARMED (JM_ARM_AFTER)"); flush(stdout)
            end
            if START_ARM && !cd_armed[]
                if !cd_said[]
                    cd_said[] = true
                    println("  ★ PRESS ANY KEY TO START COUNTDOWN"); flush(stdout)
                end
                (ARM_KEY > 0 && cs.t >= ARM_KEY && ARM_KEY_T[] == 0) && (ARM_KEY_T[] = time())
                if anykey()
                    ctrace_start!("countdown armed")
                    cd_armed[] = true
                    cd_t0[] = cs.t
                    println("  → COUNTDOWN ARMED"); flush(stdout)
                end
            elseif cd_t0[] < 0
                cd_t0[] = cs.t
            end
            if !(START_ARM && !cd_armed[])
                cd_left[] = COUNTDOWN - (cs.t - cd_t0[])
            end
            if cd_left[] <= 0 && !(START_ARM && !cd_armed[])
                cd_left[] = 0.0
                ctrace_start!("green", CTRACE_RACE)
                race_go[] = true; lap_t0 = cs.t
                ai_release[] = cs.t + AI_HEADSTART
                println("  → GREEN"); flush(stdout)
                for c in AICARS
                    rand() < 0.45 && (c.mishap = 0.4 + 1.4*rand())
                end
            end
        elseif race_go[] && cd_left[] >= 0

            cd_left[] = (cs.t - cd_t0[] - COUNTDOWN) < 1.5 ? 0.0 : -1.0
        end
    lap_t0
end

function att_quat(θ::Float64, p::Float64, r::Float64)
    cy, sy = cos(θ/2), sin(θ/2); cz, sz = cos(p/2), sin(p/2); cx, sx = cos(r/2), sin(r/2)
    (w, x, y, z) = (cy*cz, sy*sz, sy*cz, cy*sz)
    (w*cx - x*sx, w*sx + x*cx, y*cx + z*sx, z*cx - y*sx)
end
function replay_att_lerp(a0::NTuple{3,Float64}, a1::NTuple{3,Float64}, g::Float64)
    q0 = att_quat(a0...); q1 = att_quat(a1...)
    d = sum(q0 .* q1); d < 0 && (q1 = .-q1; d = -d)
    q = d > 0.9995 ? q0 .+ g .* (q1 .- q0) : (ω = acos(min(d, 1.0)); (sin((1 - g)*ω) .* q0 .+ sin(g*ω) .* q1) ./ sin(ω))
    (w, x, y, z) = q ./ sqrt(sum(q .^ 2))
    f1 = 1 - 2(y*y + z*z); f2 = 2(x*y + w*z); f3 = 2(x*z - w*y); u2 = 1 - 2(x*x + z*z); r2 = 2(y*z - w*x)
    (atan(-f3, f1), asin(clamp(f2, -1.0, 1.0)), atan(-r2, u2))
end

"""E18 / REPLAY-2 / REPLAY-6: record every car's pose and the analysis channels at ~15 Hz once the race is green, with the
crash-safe autosave. (LOOPSPLIT-1 cycle 2 S1: moved unchanged from main(); its buffers and Refs are shared.)"""
function replay_record!(cs, inp, AICARS, ai_poses, CLINE, REPLAY_AUTOSAVE, REPLAY_NCAR, phase, race_go, rst, replay_buf, replay_names, replay_out, replay_saved, replay_t, tele_buf, tyre_lp, prog::Float64 = 0.0)
        if replay_buf !== nothing && race_go[] && !rst && length(ai_poses) == length(AICARS) && (cs.t - replay_t[]) >= 1/15
            replay_t[] = cs.t
            b = WBODY[]
            bp, br = 0.0, 0.0
            if b === nothing
                push!(replay_buf, Float32(cs.t), Float32(cs.x), Float32(cs.y), Float32(cs.z), Float32(cs.θ))
            else
                o = b.p - b.R * WreckBody.V3(0.0, b.g.hcg, 0.0); (bθ, bp, br) = WreckBody.attitude(b)
                push!(replay_buf, Float32(cs.t), Float32(o[1]), Float32(o[2]), Float32(-o[3]), Float32(bθ))
            end
            for p in ai_poses; push!(replay_buf, Float32(p[1]), Float32(p[2]), Float32(p[3]), Float32(p[4])); end

            push!(tele_buf, Float32(cs.laps), Float32(cs.lapdist), Float32(cs.v), Float32(inp.throttle), Float32(inp.brake),
                  Float32(inp.steer), Float32(inp.clutch), Float32(cs.gear), Float32(cs.rpm), Float32(cs.lateral),
                  Float32(cs.ontrack ? 1 : 0), Float32(phase[] == :race ? 1 : 0))
            for w in tyre_lp[], q in w; push!(tele_buf, Float32(q)); end
            push!(tele_buf, Float32(bp), Float32(br), Float32(b === nothing ? 0 : 1))
            push!(tele_buf, Float32(prog))
            tt_record!(tele_buf)
            for (k, a) in enumerate(AICARS)
                push!(tele_buf, Float32(a.s), Float32(a.v), Float32(a.lap), Float32(a.lane), Float32(ai_poses[k][5]), Float32(ai_poses[k][6]))
            end
            replay_wheels_rec!(REPLAY_WHEELBUF, length(replay_buf) ÷ (1 + 4*REPLAY_NCAR) - 1)
            if time() - replay_saved[] > REPLAY_AUTOSAVE
                replay_saved[] = time(); write_replay(replay_out[], replay_buf, tele_buf, REPLAY_NCAR, replay_names, CLINE; final = false)
            end
        end
    nothing
end

"""WGTD-1 (e) / TD261013-1 (a3): after the flag each AI car drives one cool-down lap and stops in the pit lane.
(LOOPSPLIT-1 cycle 2 S2: moved unchanged from main(); the field `f` and AI state are shared.)"""
function ai_cooldown_step!(f, AICARS, CLASSICAI_T, crossed, ent_name, phase, race_done, race_go)
        if AI_COOLDOWN && IS_RACE && phase[] == :race && race_go[]
            flag = race_done || any(a -> a.lap >= RACE_LAPS, AICARS)
            for (i, gc) in enumerate(f.cars)
                i > length(AICARS) && break
                st = get(AI_COOL, i, nothing)
                if st === nothing
                    if flag && crossed[i]
                        AI_COOL[i] = (:cool, gc.pace, 0.0); gc.pace = AI_COOL_PACE*gc.pace
                        AI_COOL_DIAG && println("  [cooldown] ", ent_name(i), " finished (lap ", AICARS[i].lap, "): cool-down lap")
                    end
                elseif st[1] === :cool && isfinite(PIT_IN[]) && haskey(CLASSICAI_T.lines, CLASSICAI.PIT) &&
                       PIT_IN[] - 60.0 <= gc.s < PIT_BOX[]

                    nb = count(v -> v[1] === :pit, values(AI_COOL))
                    if PITPARK && isfinite(PIT_END[]) && PIT_END[] - PIT_BOX[] > 30.0
                        sp = min(15.0, (PIT_END[] - PIT_BOX[] - 20.0) / max(length(AICARS) - 1, 1))
                        box = max(PIT_END[] - 10.0 - sp*nb, PIT_BOX[] + 10.0)
                    else
                        box = PIT_BOX[] + 30.0 - 15.0*nb
                    end
                    CLASSICAI.join_pit!(CLASSICAI_T, gc); gc.hold = typemax(Int) ÷ 4
                    AI_COOL[i] = (:pit, st[2], box)
                    AI_COOL_DIAG && println("  [cooldown] ", ent_name(i), " into the pits, box at ", round(Int, box), " m")
                elseif st[1] === :pit
                    rem = st[3] - gc.s
                    gc.pace = st[2] * clamp(rem / 40.0, 0.0, 1.0); gc.hold = typemax(Int) ÷ 4
                    if AI_COOL_DIAG && gc.v < 0.3 && !(i in AI_COOL_STOPPED)
                        push!(AI_COOL_STOPPED, i)
                        println("  [cooldown] ", ent_name(i), " stopped at s ", round(Int, gc.s), ", ", round(gc.d, digits = 1),
                                " m off Classic's centre (pit line ", round(CLASSICAI.dlat(CLASSICAI_T.lines[CLASSICAI.PIT], gc.s), digits = 1),
                                ", corridor ", round.(CLASSICAI.corridor(CLASSICAI_T, gc.s), digits = 1), ")")
                    end
                elseif st[1] === :cool && crossed[i]

                    q = 25.0*count(v -> v[1] === :stop, values(AI_COOL))
                    AI_COOL[i] = (:stop, st[2], AICARS[i].s + q)

                    sst = mod(gc.s + 300.0 + q, CLASSICAI_T.lap); lo, hi = CLASSICAI.corridor(CLASSICAI_T, sst)
                    CLASSICAI.join!(CLASSICAI_T, gc, hi - 1.2; at = sst, why = :cooldown); gc.hold = typemax(Int) ÷ 4
                    AI_COOL_DIAG && println("  [cooldown] ", ent_name(i), " pulls aside to stop")
                elseif st[1] === :stop
                    frac = clamp((AICARS[i].s - st[3]) / 300.0, 0.0, 1.0)
                    gc.pace = st[2] * 0.6 * (1.0 - frac); gc.hold = typemax(Int) ÷ 4
                end
            end
        end
    nothing
end

"""E10 / FUEL-1: burn fuel once the race is green (the gold's pedal x rpm law), the burnt mass leaving the car; an empty
tank cuts the throttle. Returns `inp`. (LOOPSPLIT-1 cycle 2 S4: moved unchanged from main().)"""
function fuel_step!(cs, inp, dt, FUEL1, FUEL_ON, fuel, fuel_mass_t, race_go, rst)
        if FUEL_ON && race_go[] && !rst
            if FUEL1
                fuel[] = max(0.0, fuel[] - DriveRT3D.fuel_rate_kgh(inp.throttle, cs.rpm)/3600*(dt > 1e-4 ? dt : 1/60)/DriveRT3D.FUEL_KG_PER_L)
                if CAR3D && cs.t - fuel_mass_t[] >= 0.5
                    fuel_mass_t[] = cs.t; DriveRT3D.set_fuel_burnt!(cs, (FUEL_START_L[] - fuel[])*DriveRT3D.FUEL_KG_PER_L)
                end
            else
                fuel[] = max(0.0, fuel[] - FUEL_LPK * cs.v * (dt > 1e-4 ? dt : 1/60) / 1000)
            end
            fuel[] <= 0 && (inp = DriveInput(throttle=0.0, brake=inp.brake, steer=inp.steer,
                            clutch=inp.clutch, shift_up=inp.shift_up, shift_down=inp.shift_down, autoshift=inp.autoshift))
        end
    inp
end

"""CTRL+R: the session restarts on the current track (LOOPSPLIT-1 cycle 2, moved unchanged from the frame loop; the
per-session state that main() holds in plain locals comes back as a NamedTuple). restart_smoke reads THIS body."""
function restart_session!(cs, cs0, groundz, groundz_phys, AICARS, AILINE, CLINE, DO_QUAL, HOLD_START, FUEL1, FUEL_ON, CLASSICAI_F, PLAYER_G, REPLAY_NCAR, ai_best, ai_lap_prev, ai_lapt0, burn_lap, fuel_laps, fuel, cd_armed, cd_left, cd_said, cd_t0, flush_ibt!, form_grid!, ibt_samples, launch_done, new_replay_path, phase, player_finpos, player_grid, player_laps, race_go, replay_buf, replay_names, replay_out, replay_saved, replay_t, tele_buf, player_s_prev)
    t_restart = time()

    if replay_buf !== nothing && !isempty(replay_buf)
        write_replay(replay_out[], replay_buf, tele_buf, REPLAY_NCAR, replay_names, CLINE; final = true)
        println("  [session] replay saved: ", basename(replay_out[])); flush(stdout)
        empty!(replay_buf); empty!(tele_buf); empty!(REPLAY_WHEELBUF); tt_reset!(); replay_t[] = -1.0; replay_saved[] = time()
        replay_out[] = new_replay_path()
    end
    flush_ibt!(ibt_samples)

    groundz(cs0.x, cs0.z; acquire = true)
    PLAYER_G[] = NaN; respawnX!(cs; groundz = groundz_phys); DriveRT3D.damage_reset!()
    DC[].lastz = 0.0
    WRECKED[] = false; WRECK_FROZEN[] = false; AIRBORNE[] = false; empty!(LOOSE_WHEELS)
    CLUTCH_GATE[] = -1.0
    TELLTALE[] = 0.0
    empty!(AI_COOL)

    cs.laps = 0; lap_t0 = cs.t; last_lap = 0.0; best_lap = 0.0; prev_laps = 0
    race_done = false; empty!(player_laps)
    player_finpos[] = 0; player_grid[] = 0
    launch_done[] = false

    player_prog = 0.0
    CLINE !== nothing && (player_s_prev = RaceAI.project(CLINE, cs.x, cs.z)[1])

    phase[] = DO_QUAL ? :practice : :race

    race_go[] = !HOLD_START
    cd_t0[] = -1.0; cd_left[] = -1.0

    cd_armed[] = false; cd_said[] = false

    if !isempty(AICARS)
        AICARS .= RaceAI.init_cars(AILINE, length(AICARS); start_s = 30.0)
        fill!(ai_lapt0, cs.t); fill!(ai_lap_prev, 0); fill!(ai_best, Inf)
        RaceAI.aistat_reset!()

        (!DO_QUAL && HOLD_START) && (player_grid[] = form_grid!(Inf))
        CLASSICAI_F[] = nothing
    end

    FUEL_ON && (fuel[] = FUEL1 ? FUEL_START_L[] : burn_lap * fuel_laps; CAR3D && FUEL1 && DriveRT3D.set_fuel_burnt!(cs, 0.0))
    println("  [restart] session reset on ", TRACKSEL, " in ",
            round((time() - t_restart)*1000, digits=1), " ms (track NOT reloaded)")
    flush(stdout)
    (lap_t0 = lap_t0, last_lap = last_lap, best_lap = best_lap, prev_laps = prev_laps, race_done = race_done,
     player_prog = player_prog, player_s_prev = player_s_prev)
end

"""INSTANTREPLAY-1: the race's wreck, airborne and loose-wheel state back to a fresh launch's before main() runs again
as the session's replay (a JM_REPLAY process starts from these; the wheels lost in the race would be missing from its
first frame on)."""
function instant_replay_reset!()
    WRECKED[] = false; WRECK_FROZEN[] = false; AIRBORNE[] = false; WBODY[] = nothing; empty!(LOOSE_WHEELS); empty!(LOOSE_OF)
    ai_wheeloff_reset!()
    CLUTCH_GATE[] = -1.0; TELLTALE[] = 0.0; STUCK_T0[] = -1.0; RACEEND_T0[] = -1.0; WHEELOFF_MSG[] = 0.0; REP_BODY[] = (NaN, NaN)
    empty!(AI_COOL); DriveRT3D.damage_reset!()
    nothing
end

"""SHIFT+R: back onto the road at the current lap position, upright and stopped -- a new car (the wreck cleared).
LOOPSPLIT-1 cycle 4 S5: moved unchanged from the frame loop. In the loop its θ0 / nLx / nLz were main() locals shared
with the JM_OFFTRACK_PROBE block (nothing read them after either)."""
function recover_step!(cs, CLINE, groundz, PLAYER_G)

    sR = RaceAI.project(CLINE, cs.x, cs.z)[1]; pR = RaceAI.pose_at(CLINE, sR, 0.0)

    cx0, cz0, θ0 = pR[1], pR[3], pR[4]
    nLx = -sin(θ0); nLz = cos(θ0)
    hiR = 0.0; while hiR < 10.0 && onroad_at(cx0 + nLx*(hiR+0.5), cz0 + nLz*(hiR+0.5)); hiR += 0.5; end
    loR = 0.0; while loR < 10.0 && onroad_at(cx0 - nLx*(loR+0.5), cz0 - nLz*(loR+0.5)); loR += 0.5; end
    offR = clamp((hiR - loR)/2, -5.0, 5.0)
    cxR = cx0 + nLx*offR; czR = cz0 + nLz*offR
    println("  [recover] placed at s ", round(sR, digits = 1), ": road ", -loR, " .. +", hiR, " m about the line, offset ",
            round(offR, digits = 2), " m"); flush(stdout)
    DriveRT3D.place3d!(cs, cxR, czR, θ0; v=0.0)
    pR = (cxR, pR[2], czR, θ0)
    cs.s_vreset(cs.integ, zeros(14))
    hR = groundz(pR[1], pR[3]; acquire=true); isfinite(hR) && (cs.zref = Float64(hR))
    cs.heave = 0.0; cs.pitch = 0.0; cs.roll = 0.0; cs.y = cs.zref

    if get(ENV, "JM_RECOVER_KEEPWRECK", "0") == "0"
        PLAYER_G[] = NaN; DriveRT3D.damage_reset!(); DC[].lastz = 0.0
        WRECKED[] && println("  [recover] SHIFT+R: the wreck is cleared -- engine and gearbox back"); flush(stdout)
        WRECKED[] = false; WRECK_FROZEN[] = false; AIRBORNE[] = false; empty!(LOOSE_WHEELS); CLUTCH_GATE[] = -1.0
    end
    nothing
end

"""The player's driving step, one frame: the wreck / launched car as the free rigid body, or the trackside contact
forces, the chassis step, the launch hand-over and the Classic-wall / drawn-obstacle clamps. LOOPSPLIT-1 cycle 4 S6: moved
unchanged from the frame loop; returns (ffb_jolt, _tgc) -- `_tgc` = when the wall work began (the CLASSICWALL cost line)."""
function player_drive_step!(cs, inp, dt, rst, groundz, groundz_phys, roll_ter, ffb_jolt, race_go, BND_FX, BND_FY, BND_MZ, OFFDIST, OFFTRACK_PROBE, ONTRACK, PLAYER_CDA, PLAYER_HDG, STEP_GUARD_HITS)
    local cfx, cfy, cpk, _tgc
    (WRECKED[] || AIRBORNE[]) || (WBODY[] = nothing)
    if CAR3D && CRASHBODY && (WRECKED[] || AIRBORNE[])

        WBODY[] === nothing && ((WVX[], WVZ[]) = DriveRT3D.world_velocity(cs))
        (JOLT_MS > 0 && !SKIDPAD && !rst) && (JOLT_PRE[] = (WVX[], WVZ[], 0.0, 0.0, 0.0, CLASSICW_STAT[].hits, CLASSICW_STAT[].clamps,
                                      OBS_STAT[].hits, OBS_STAT[].clamps, STEP_GUARD_HITS[]))
        wreck_body_step!(cs, inp, dt, groundz_phys, roll_ter)
        _tgc = time()
    else
    if CAR3D
        (cfx, cfy, cpk, ffb_jolt) = player_step3d!(cs, dt, rst, inp, groundz, ffb_jolt, groundz_phys, race_go, BND_FX, BND_FY, BND_MZ,
                                                   OFFDIST, OFFTRACK_PROBE, ONTRACK, PLAYER_CDA)
    end

    PLAYER_HDG[] = cs.θ
    (JOLT_MS > 0 && CAR3D && !SKIDPAD && !rst) && (JOLT_PRE[] = (WVX[], WVZ[], cfx, cfy, cpk, CLASSICW_STAT[].hits, CLASSICW_STAT[].clamps,
                                  OBS_STAT[].hits, OBS_STAT[].clamps, STEP_GUARD_HITS[]))
    _tps = time()
    step_carX!(cs, inp.throttle * DriveRT3D.engine_power(), inp.brake, inp.steer, dt > 1e-4 ? dt : 1/60; lock = STEER_LIM[],
                clutch=inp.clutch, up=inp.shift_up, dn=inp.shift_down, manual=!inp.autoshift,
                groundz=groundz_phys)
    DS_SIM[1] += time() - _tps

    (WRECKED[] && WRECK_FROZEN[] && WRECK_PIN && CAR3D) && DriveRT3D.pin3d!(cs, WRECK_PX[], WRECK_PZ[], WRECK_PT[], WRECK_PH[])

    if AIRBODY && CRASHBODY && CAR3D && !WRECKED[] && !rst && !SKIDPAD && maximum(cs.rh) > AIRBODY_RH
        AIRBORNE[] = true; AIR_LAND[] = 0.0; AIR_PEAK[] = (0.0, 0.0); ctrace_start!("launch")
        println(launch_msg(Float64(cs.v), cs.rh)); flush(stdout)
    end
    if VDIAG && CAR3D && cs.t - VDIAG_T[] >= 0.5
        VDIAG_T[] = cs.t
        println("  [vdiag] t ", round(cs.t, digits = 1), " s ", round(cs.lapdist, digits = 0), " v ", round(cs.v*3.6, digits = 0),
            "  y-road ", round(cs.y - groundz_phys(cs.x, cs.z), digits = 2), "  zref-road ", round(cs.zref - groundz_phys(cs.x, cs.z), digits = 2),
            "  heave ", round(cs.heave, digits = 3), "  vref ", round(get(DriveRT3D._VREF, cs, NaN), digits = 2),
            "  yaw ", round(rad2deg(cs.θ), digits = 2), " vmm ", round(1000cs.v, digits = 1), " brk ", round(inp.brake, digits = 2), " st ", round(inp.steer, digits = 3),
            "  y-mesh ", (let r = JRTrack.hat3d(ROADHAT, cs.x, cs.z; ref = cs.y + 3.0); r[3] ? round(cs.y - r[1], digits = 2) : NaN end)); flush(stdout)
    end
    _tgc = time()
    if CLASSICW_PHYS && CAR3D && !rst && CLASSICW_NEAR[] && !(WRECK_FROZEN[] && WRECK_NOCLAMP)
        for _ in 1:2
            classicwall_clamp!(cs)
            OBS_ON && obs_clamp!(cs)
        end
    end

    (CLASSICW_PHYS && CAR3D && !rst && !CLASSICW_NEAR[] && OBS_ON) && (OBS_PREV[] = (cs.x, cs.z))
    end

    step_loose_wheels!(dt > 1e-4 ? dt : 1/60, (gx, gzz) -> (h = groundz(gx, gzz); h > -900 ? Float64(h) : 0.0))
    isempty(AI_LOOSE_WHEELS) || step_loose_wheels!(dt > 1e-4 ? dt : 1/60, (gx, gzz) -> (h = groundz(gx, gzz); h > -900 ? Float64(h) : 0.0), AI_LOOSE_WHEELS)
    (ffb_jolt, _tgc)
end

"""The window title, every 0.25 s: the replay's camera / car / clock, or the live speed, gear, race state, position and
fuel. LOOPSPLIT-1 cycle 5 S1: moved unchanged from the frame loop; returns titleT."""
function title_step!(cs, win, now, titleT, REPLAY, FUEL_ON, repd, rep_focus, rep_cam, rep_rt, rep_dur, rep_play, rep_speed, AICARS, standings, race_go, phase, cd_armed, cd_left, race_done, player_finpos, best_lap, fmt_lap, player_grid, fuel, burn_lap)
    if now - titleT > 0.25 && REPLAY
        carname = rep_focus[] == 0 ? (isempty(repd.names) ? "Player" : repd.names[1]) :
                  (rep_focus[] < length(repd.names) ? repd.names[rep_focus[]+1] : "AI $(rep_focus[])")
        GLFW.SetWindowTitle(win, "▶ REPLAY — $(uppercasefirst(TRACKSEL)) — 📷 $(REPLAY_CAM_LABEL[rep_cam[]]) — 🏎 $carname [$(rep_focus[]+1)/$(repd.ncar)]" *
            "  —  $(round(rep_rt[],digits=1))/$(round(rep_dur,digits=1))s  $(rep_play[] ? "▶" : "⏸")×$(rep_speed[])   ·  V angle · C car · SPACE play · ←/→ seek · ↑/↓ speed")
        titleT = now
    elseif now - titleT > 0.25

        if TRACE_POS && IS_RACE && race_go[] && !isempty(AICARS)
            _p = findfirst(e -> e[1] == 0, standings())
            if _p !== nothing && _p != POS_LAST[]
                println("  [pos] t=", round(cs.t, digits=2), "s  lap=", cs.laps + 1,
                        "  title shows P", _p, "/", length(AICARS) + 1,
                        POS_LAST[] < 0 ? "   (first sample)" : "   (was P$(POS_LAST[]))")
                flush(stdout)
                POS_LAST[] = _p
            end
        end
        GLFW.SetWindowTitle(win, "Julia Racer — $(uppercasefirst(TRACKSEL)) — $(round(Int,cs.v*3.6)) km/h — gear $(cs.gear == 0 ? "N" : cs.gear < 0 ? "R" : string(cs.gear)) ($(CTL.auto ? "AUTO" : "MANUAL")) — $(round(Int,cs.rpm)) rpm" *
            (phase[] == :qual ? "  ⏱ QUALIFYING — drive a lap, then press ENTER to start the race" :
             (!race_go[]) ? ((START_ARM && !cd_armed[] && COUNTDOWN > 0) ?
                             "  🏁 PRESS ANY KEY TO START COUNTDOWN" :
                             cd_left[] > 0 ? "  🏁 $(ceil(Int, cd_left[]))..." :
                             "  🏁 GET READY — floor the throttle to start (the field launches with you)") :
             IS_RACE ? (race_done ? "  ✦ FINISHED P$(player_finpos[])/$(length(AICARS)+1) — best $(fmt_lap(best_lap)) (started P$(player_grid[]))" :
                        "  — lap $(min(cs.laps+1,RACE_LAPS))/$RACE_LAPS" *
                        (isempty(AICARS) ? "" : "  Pos P$(findfirst(e->e[1]==0, standings()))/$(length(AICARS)+1)")) :
                       "  [$(uppercasefirst(MODE))]") *
            (FUEL_ON ? "  — fuel $(round(Int,fuel[]))L ($(round(burn_lap>0 ? fuel[]/burn_lap : 0,digits=1)) laps)" : "") *
            (cs.ontrack ? "" : "  [OFF TRACK]"))
        titleT = now
    end
    titleT
end

"""LOOPSPLIT-1 cycle 5 S2: the player car + cockpit draw (body/interior skin, a replayed team's cockpit, tach and
dials, pipes, axles, suspension, driver figure, gauges, road and loose wheels, mirrors, steering wheel, gear lever,
hands, perspex and windscreen) -- moved unchanged from the frame loop. Assigns nothing the loop reads."""
function player_car_draw!(cs, inp, vp, bodyModel, wheelmat, loosemat, REPLAY, rep_team, mirror_live, frames, TEAM_LEVER_AT_LOTUS)
    _tm = (REPLAY || PLAYER_CM[] !== nothing) && CTL.view == 0 && rep_team[] !== nothing
    let _items = _tm ? rep_team[].body : (CTL.view == 0 && !isempty(carItemsIn)) ? carItemsIn : carItems
        for (i,it) in enumerate(_items); _in_car_range(i) || continue; Render.draw(prog, it, vp, bodyModel; bright=1.2, spec=0.08, ambfill=0.78); end
    end
    if _tm && !isempty(rep_team[].dash)
        glDisable(GL_DEPTH_TEST)
        for it in rep_team[].dash; Render.draw(prog, it, vp, bodyModel; bright=1.2, spec=0.08, ambfill=0.78); end
        glEnable(GL_DEPTH_TEST)
        for it in rep_team[].arms; Render.draw(prog, it, vp, bodyModel; bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3]); end
    end

    _thoff = (0f0, 0f0, 0f0); _tside = 0
    if _tm && TEAM_SHIFT_HAND && TEAM_LEVER_LM[] !== nothing && !isempty(rep_team[].handsLR[1]) && !isempty(rep_team[].handsLR[2])
        let T = rep_team[], st = Float32(HSIGN*inp.steer*2.5)
            k4 = TEAM_LEVER_LM[] * Float32[T.shtip; 1]; _tside = k4[3] > 0 ? 1 : -1
            u = Float32(shift_hand_u(lever_gear(cs.gear, cs.t)))
            if u > 0f0
                wrel = Render.translate(T.swc) * Render.rotaxis(T.swa, st) * Render.translate(-T.swc[1], -T.swc[2], -T.swc[3])
                g = wrel * Float32[T.fistc[_tside > 0 ? 2 : 1]; 1]
                _thoff = (u*(k4[1] + SHIFT_HAND_UP[1] - g[1]), u*(k4[2] + SHIFT_HAND_UP[2] - g[2]), u*(k4[3] + SHIFT_HAND_UP[3] - g[3]))
            end
            if TEAM_HANDDIAG && TEAM_HANDDIAG_N[] < 40
                TEAM_HANDDIAG_N[] += 1
                wr = Render.translate(T.swc) * Render.rotaxis(T.swa, st) * Render.translate(-T.swc[1], -T.swc[2], -T.swc[3])
                f4 = vp * bodyModel * Render.translate(_thoff...) * wr * Float32[T.fistc[_tside > 0 ? 2 : 1]; 1]
                q4 = vp * bodyModel * (k4 .+ Float32[SHIFT_HAND_UP; 0])
                px(q) = (round(Int, W*(q[1]/q[4] + 1)/2), round(Int, view_h()*(1 - q[2]/q[4])/2))
                println("  [teamhand] t ", round(cs.t; digits = 2), " u ", round(u; digits = 2), " side ", _tside, " glove px ", px(f4), " knob px ", px(q4)); flush(stdout)
            end
        end
    end
    if _tm && !isempty(rep_team[].sleeves)
        let T = rep_team[], st = Float32(HSIGN*inp.steer*2.5)
            wrel = Render.translate(T.swc) * Render.rotaxis(T.swa, st) * Render.translate(-T.swc[1], -T.swc[2], -T.swc[3])
            Rw = Render.rotaxis(T.swa, st)
            for q in T.sleeves

                h = TEAM_SLEEVE_RIGID ? Float32[q.cuff[1], q.cuff[2], q.cuff[3], 1f0] : wrel * Float32[q.cuff[1], q.cuff[2], q.cuff[3], 1f0]
                u = (TEAM_SLEEVE_LOCK && !TEAM_SLEEVE_RIGID) ? clamp((abs(st) - deg2rad(15)) / deg2rad(45), 0.0, 1.0) : 0.0
                S = (q.S[1], q.S[2] - 0.08u, q.S[3] + u*(sign(q.S[3])*0.20 - q.S[3])); th = q.thick + u*(1f0 - q.thick)
                d = (h[1] - S[1], h[2] - S[2], h[3] - S[3]); ld = sqrt(d[1]^2 + d[2]^2 + d[3]^2)
                h = ld > 1e-6 ? (h[1] + TEAM_CUFF_INTO*d[1]/ld, h[2] + TEAM_CUFF_INTO*d[2]/ld, h[3] + TEAM_CUFF_INTO*d[3]/ld) : (h[1], h[2], h[3])
                sign(q.S[3]) == _tside && (h = (h[1] + _thoff[1], h[2] + _thoff[2], h[3] + _thoff[3]))
                M = TEAM_SLEEVE_RIGID ? wrel * sleeve_mat(q.W, q.E, h, S, q.nf; target = q.nref, thick = th) :
                                        sleeve_mat(q.W, q.E, h, S, q.nf; target = (Rw * Float32[q.nref..., 0f0])[1:3], thick = th)
                for it in q.items; Render.draw(prog, it, vp, bodyModel * M; bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3]); end
            end
        end
    end

    if CTL.view == 0 && _tm && rep_team[].tach !== nothing
        let t = rep_team[].tach
            glDisable(GL_DEPTH_TEST)
            Render.draw(prog, t.item, vp, bodyModel * dial_model(t.f, t.d0 + t.dk * clamp(cs.rpm, t.lo, t.hi) / 1000); unlit=true)
            if !isempty(rep_team[].dials)
                dv = dials_update!(cs.rpm)
                for d in rep_team[].dials
                    Render.draw(prog, d.item, vp, bodyModel * dial_model(d.f, dial_deg(d.scale, getfield(dv, d.sym))); unlit=true)
                end
            end
            glEnable(GL_DEPTH_TEST)
        end
    end
    if _tm
        let T = rep_team[]
            wm = bodyModel * Render.translate(T.swc) * Render.rotaxis(T.swa, Float32(HSIGN*inp.steer*2.5)) * Render.translate(-T.swc[1], -T.swc[2], -T.swc[3])
            l, f = shift_target(lever_gear(cs.gear, cs.t)); top = LEVER_TOP
            if TEAM_LEVER_AT_LOTUS && SHIFT_PIVOT !== nothing

                mir = TEAM_LEVER_SIDE && T.lever_left != (SHIFT_PIVOT[3] > 0)
                tipm = T.shtip .- T.shp; kg = team_lever_gain(TEAM_LEVER_STRAIGHT ? sqrt(sum(abs2, tipm)) : T.shlen)
                inb = SHIFT_PIVOT[3] > 0 ? -1f0 : 1f0

                function gate(l_, f_)
                    G = Render.translate(SHIFT_XYZ) * SHIFT_MOVE * Render.translate(SHIFT_PIVOT) * Render.rotaxis(Float32[0, 0, 1], -deg2rad(SHIFT_FA * f_ * kg)) *
                        Render.rotaxis(Float32[1, 0, 0], inb * deg2rad(SHIFT_LAT * l_ * kg + SHIFT_LEAN)) * Render.scalexyz(SHIFT_S, SHIFT_SY, SHIFT_S)
                    mir ? Render.scalexyz(1f0, 1f0, -1f0) * G * Render.scalexyz(1f0, 1f0, -1f0) : G
                end
                D = (mir ? TEAM_LEVER_D .+ TEAM_LEVER_MIRD : TEAM_LEVER_D) .+ T.lever_d
                R0 = TEAM_LEVER_STRAIGHT ? T.shR : Render.ident()
                if TEAM_LEVER_STRAIGHT
                    G0 = gate(0f0, 0f0); t4 = Float32[tipm; 1]
                    D = D .+ ((G0 * t4)[1:3] .- (G0 * R0 * t4)[1:3])
                end
                top = top || TEAM_LEVER_SIDE
                lm = Render.translate(D) * gate(l, f) * R0 * Render.translate(-T.shp[1], -T.shp[2], -T.shp[3])
                sm = bodyModel * lm
                TEAM_LEVER_SM[] = sm; TEAM_LEVER_LM[] = Matrix{Float32}(lm)
            else
                inb = T.shp[3] > 0 ? -1f0 : 1f0
                sm = bodyModel * Render.translate(T.shp) * Render.rotaxis(Float32[0, 0, 1], -deg2rad(SHIFT_FA * f)) *
                     Render.rotaxis(Float32[1, 0, 0], inb * deg2rad(SHIFT_LAT * l)) * Render.translate(-T.shp[1], -T.shp[2], -T.shp[3])
                TEAM_LEVER_LM[] = Matrix{Float32}(inv(bodyModel) * sm)
            end
            if LEVER_CYCLE > 0 && lever_gear(cs.gear, cs.t) != LEVER_DIAG_G[]
                LEVER_DIAG_G[] = lever_gear(cs.gear, cs.t); q = vp * sm * Float32[T.shtip; 1]
                println("  [leverdiag] gear ", LEVER_DIAG_G[], " knob px (", round(Int, W*(q[1]/q[4] + 1)/2), ", ", round(Int, view_h()*(1 - q[2]/q[4])/2),
                        ")  tip-foot (mesh) ", round.(T.shtip .- T.shp; digits = 3)); flush(stdout)
            end

            top && glDepthFunc(GL_ALWAYS)
            for it in T.shift; Render.draw(prog, it, vp, sm; bright=0.95, ambfill=0.45); end
            top && glDepthFunc(GL_GEQUAL)
            for it in T.wheel; Render.draw(prog, it, vp, wm; bright=1.2, spec=0.05, ambfill=0.6); end
            _hc = get(ENV, "JM_TEAM_HAND_CULL", "")
            _hw = glIsEnabled(GL_CULL_FACE) == GL_TRUE
            isempty(_hc) || (glEnable(GL_CULL_FACE); glCullFace(_hc == "front" ? GL_FRONT : GL_BACK))
            if TEAM_SHIFT_HAND && !isempty(T.handsLR[1]) && !isempty(T.handsLR[2])
                wr = Render.translate(T.swc) * Render.rotaxis(T.swa, Float32(HSIGN*inp.steer*2.5)) * Render.translate(-T.swc[1], -T.swc[2], -T.swc[3])
                for (i, s) in ((1, -1), (2, 1))
                    hm = s == _tside ? bodyModel * Render.translate(_thoff...) * wr : wm
                    for it in T.handsLR[i]; Render.draw(prog, it, vp, hm; bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3]); end
                end
            else
                for it in T.hands; Render.draw(prog, it, vp, wm; bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3]); end
            end
            isempty(_hc) || (glCullFace(GL_BACK); _hw || glDisable(GL_CULL_FACE))
        end
    end

    TELLTALE[] = max(TELLTALE[], cs.rpm)
    if CTL.view == 0 && !isempty(TACH_ITEMS) && !_tm
        Render.draw(prog, TACH_ITEMS[1], vp, bodyModel * tach_model(tach_deg(TELLTALE[])); unlit=true)
        Render.draw(prog, TACH_ITEMS[2], vp, bodyModel * tach_model(tach_deg(cs.rpm)); unlit=true)
    end
    if CTL.view == 0 && !isempty(DIAL_ITEMS) && !_tm
        dv = dials_update!(cs.rpm)
        for (d, it) in zip(DIALS, DIAL_ITEMS)
            Render.draw(prog, it, vp, bodyModel * dial_model(d.f, dial_deg(d.spec.scale, getfield(dv, d.sym))); unlit=true)
        end
    end

    let pm = bodyModel * Render.translate(0, PIPE_LIFT, 0)
        _tm || for it in pipeItems; Render.draw(prog, it, vp, pm; bright=1.15, spec=0.25, ambfill=0.6); end
    end

    _tm || for it in axleItems; Render.draw(prog, it, vp, bodyModel; bright=1.1, spec=0.3, ambfill=0.55); end
    _tm || for it in fsuspSynItems; Render.draw(prog, it, vp, bodyModel; bright=1.1, spec=0.3, ambfill=0.55); end
    if CTL.view != 0
        if get(ENV,"JM_RSUSP2","0") == "1"
            for it in rsusp2Items; Render.draw(prog, it, vp, bodyModel; bright=1.15, spec=RS_SPEC, ambfill=0.55); end
        end
        if RSUSP_ON
            let mA = RS_BAKECLIP ? bodyModel : bodyModel*RSFIX_A,
                mB = RS_BAKECLIP ? bodyModel : bodyModel*RSFIX_B
                for it in rsuspItemsA; Render.draw(prog, it, vp, mA; bright=1.15, spec=RS_SPEC, ambfill=0.55); end
                for it in rsuspItemsB; Render.draw(prog, it, vp, mB; bright=1.15, spec=RS_SPEC, ambfill=0.55); end
            end
        end
        for it in driverItems; Render.draw(prog, it, vp, bodyModel; bright=1.2, spec=0.10, ambfill=0.55); end
        helmModel = bodyModel * Render.translate(HELM_OFF[1],HELM_OFF[2],HELM_OFF[3])
        for it in helmItems; Render.draw(prog, it, vp, helmModel; bright=1.2, spec=0.12, ambfill=0.60); end
    end

    dash_b = parse(Float64, get(ENV,"JM_DASH_B","1.2")); dash_a = parse(Float64, get(ENV,"JM_DASH_A","0.9"))
    if CTL.view == 0
        glDisable(GL_DEPTH_TEST)
        _tm || for it in gaugeItems; Render.draw(prog, it, vp, bodyModel*GAUGEFLIP; bright=dash_b, spec=0.0, ambfill=dash_a); end
        glEnable(GL_DEPTH_TEST)
    else
        for it in gaugeItems; Render.draw(prog, it, vp, bodyModel*GAUGEFLIP; bright=dash_b, spec=0.0, ambfill=dash_a); end
    end

    for (wx,wz,steer,r,nm) in WHEELS, it in WHEELITEMS[nm]
        is_loose(nm) && continue
        PLAYER_MINE[] && continue
        get(ENV,"JM_NO_WHEELS","0") != "0" && continue
        Render.draw(prog, it, vp, wheelmat(wx,wz,steer,r,nm); bright=1.0, ambfill=0.75)
    end
    for (lx,ly,lz,_,_,_,sp,_,nm) in LOOSE_WHEELS, it in loose_items(nm)
        Render.draw(prog, it, vp, loosemat(lx,ly,lz,sp); bright=1.0, ambfill=0.75)
    end

    _tm || for it in fsuspItems; Render.draw(prog, it, vp, bodyModel; bright=1.1, spec=0.15, ambfill=0.5); end

    _tm || for it in mirrorItems; Render.draw(prog, it, vp, bodyModel*MIRRORMAT; bright=1.25, spec=0.30, ambfill=0.75, depthbias=true); end
    if mirror_live && !_tm
        for it in mirGlassItems; Render.draw(prog, it, vp, bodyModel*MIRRORMAT; mirrorglass=true, depthbias=true); end
    end
    if _tm && MIRROR_RTT && CTL.view == 0
        for it in rep_team[].mglass; Render.draw(prog, it, vp, bodyModel; mirrorglass=true, depthbias=true); end
    end

    swModel = bodyModel * Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(HSIGN*inp.steer*2.5 + SW_ROT)) * Render.translate(-SWCENTER[1], -SWCENTER[2], -SWCENTER[3])
    _tm || for it in swItems; Render.draw(prog, it, vp, swModel; bright=1.2, ambfill=0.34); end
    if CTL.view == 0 && !_tm && !isempty(shiftItems) && SHIFT_PIVOT !== nothing
        _tn = time(); _dt = SHIFT_TW[] == 0.0 ? 1f0 : Float32(clamp(_tn - SHIFT_TW[], 0.0, 0.1)); SHIFT_TW[] = _tn
        SHIFT_LASTM[] = shift_model(lever_gear(cs.gear, cs.t), _dt); shm = bodyModel * SHIFT_LASTM[]

        SHIFT_ONTOP && glDisable(GL_DEPTH_TEST)
        for it in shiftItems; Render.draw(prog, it, vp, shm; bright=0.95, ambfill=0.45); end
        SHIFT_ONTOP && glEnable(GL_DEPTH_TEST)
    end

    frames == 1 && precompile(hands_draw!, typeof.((cs, inp, vp, bodyModel, swModel)))
    if HANDS && CTL.view == 0 && !_tm
        hands_draw!(cs, inp, vp, bodyModel, swModel)
    end

    if SHIFT_OVER_ARMS && CTL.view == 0 && !_tm && !isempty(shiftItems) && SHIFT_PIVOT !== nothing
        glDisable(GL_DEPTH_TEST)
        for it in shiftItems; Render.draw(prog, it, vp, bodyModel * SHIFT_LASTM[]; bright=0.95, ambfill=0.45); end
        glEnable(GL_DEPTH_TEST)
    end

    if _tm && VISOR_ALPHA > 0 && !(isempty(rep_team[].glass) && isempty(rep_team[].wglass))
        glDepthMask(GL_FALSE)
        for it in rep_team[].glass; Render.draw(prog, it, vp, bodyModel; bright=1.0, spec=0.35, ambfill=0.7, alpha=VISOR_ALPHA, depthbias=true); end
        for it in rep_team[].wglass; Render.draw(prog, it, vp, bodyModel; bright=1.0, spec=0.35, ambfill=0.7, alpha=TEAM_WIN_ALPHA, depthbias=true); end
        glDepthMask(GL_TRUE)

        if TEAM_LEVER_SIDE && TEAM_LEVER_AT_LOTUS && SHIFT_PIVOT !== nothing
            SHIFT_OVER_ARMS && glDisable(GL_DEPTH_TEST)
            for it in rep_team[].shift; Render.draw(prog, it, vp, TEAM_LEVER_SM[]; bright=0.95, ambfill=0.45); end
            SHIFT_OVER_ARMS && glEnable(GL_DEPTH_TEST)
        end
    end
    if !isempty(visorItems) && VISOR_ALPHA > 0 && !_tm
        glDepthMask(GL_FALSE)
        for it in visorItems; Render.draw(prog, it, vp, bodyModel; bright=1.0, spec=0.35, ambfill=0.7, alpha=VISOR_ALPHA, depthbias=true); end
        glDepthMask(GL_TRUE)
    end
    if WIND_ALPHA > 0 && !_tm
        glDepthMask(GL_FALSE)

        for it in windItems; Render.draw(prog, it, vp, bodyModel; bright=WIND_B, spec=0.02, ambfill=WIND_A, alpha=WIND_ALPHA, depthbias=true); end
        glDepthMask(GL_TRUE)
    end
    nothing
end

"""AI lap timing (both the physics and kinematic fields): each AI lap as it completes -> ai_best; JM_AI_LAPDIAG prints it.
LOOPSPLIT-1 cycle 5 S3: moved unchanged from the frame loop."""
function ai_lap_step!(cs, phase, race_go, AICARS, ai_lap_prev, ai_lapt0, ai_best, ent_name, fmt_lap)
    if phase[] == :race && race_go[]
        for i in 1:length(AICARS)
            if AICARS[i].lap > ai_lap_prev[i]
                lt = cs.t - ai_lapt0[i]; ai_lapt0[i] = cs.t
                ai_lap_prev[i] > 0 && (ai_best[i] = min(ai_best[i], lt))

                if AI_LAPDIAG && ai_lap_prev[i] > 0
                    println("  [ailap] ", rpad(ent_name(i), 9), " lap ", AICARS[i].lap,
                            "  ", fmt_lap(lt), "   best ", fmt_lap(ai_best[i]))
                    flush(stdout)
                end
                ai_lap_prev[i] = AICARS[i].lap
            end
        end
    end
    nothing
end

"""Hidden warm-up frame 1: compile the AI-field functions first called at the GREEN LIGHT, from the real types (LOOPSPLIT-1
cycle 4 S1). LOOPSPLIT-1 cycle 5 S6: moved unchanged from the frame loop."""
function warm_ai_compile!(cs, ddt, ai_hit, AICARS, AILINE, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, CLASSICAI_T, ent_name, classicai_field!, phase, race_done, race_go, ai_poses, AIPHYS, PLAYER_CDA, ffb_jolt, ai_onroad, ai_stuck, groundz_ai, aibankP, groundz)
    precompile(classicai_poses!, typeof.((cs, ddt, ai_hit, AICARS, AILINE, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, CLASSICAI_T, ent_name, classicai_field!, phase, race_done, race_go)))
    precompile(ai_cooldown_step!, (CLASSICAI.Field, typeof(AICARS), typeof(CLASSICAI_T), BitVector, typeof(ent_name), typeof(phase),
                                   typeof(race_done), typeof(race_go)))
    precompile(ai_contact_step!, typeof.((cs, ai_poses, AICARS, AIPHYS, PLAYER_CDA, ffb_jolt)))
    AI_PHYSICS && precompile(aiphys_poses!, typeof.((cs, ddt, ai_hit, AICARS, AILINE, AIPHYS, ai_onroad, ai_stuck, groundz_ai, aibankP, groundz)))

    precompile(aibankP, (typeof(cs),))
    precompile(Core.kwcall, (NamedTuple{(:manual, :groundz), Tuple{Bool, typeof(groundz_ai)}}, typeof(DriveRT3D.step_car3d!),
                             typeof(cs), Float64, Float64, Float64, Float64))
    nothing
end

"""CARCHOICE-1 S1: the player's car drawn as the chosen '67 chassis -- the AI loader's model in the player's frames: the body
on `tiltModel` (dive, squat and roll, as the Lotus body), the wheels on `carModel` (upright, as the Lotus's), the fronts
steered by δ, all turned by the drive spin. `body = false`: the wheels alone (the cockpit view draws the team's cockpit)."""
function player_chassis_draw!(cm::Render.CLASSICCarModel, vp, tiltModel, carModel, δ::Float32, spin::Float64; body::Bool = true)
    Hm = ai_hand(cm)
    if body
        bm = tiltModel * Hm * Render.translate(cm.body_off[1], cm.body_off[2], cm.body_off[3])
        for it in cm.body; Render.draw(prog, it, vp, bm; bright=AI_BRIGHT, spec=0.10, ambfill=AI_AMB); end
        for it in get(AI_AXLES, cm.name, AI_NOAXLE); Render.draw(prog, it, vp, carModel * Hm; bright=1.1, spec=0.3, ambfill=0.55); end
    end
    for (i, (wx, wz, front, r, nm)) in enumerate(cm.wheelspec)
        i <= 4 && is_loose(WHEEL_NAMES[i]) && continue
        for it in cm.wheels[nm]
            M = carModel * Hm * Render.translate(wx, r, wz) * (front ? Render.roty(Float32(HSIGN)*δ) : Render.ident()) * Render.rotz(Float32(spin))
            Render.draw(prog, it, vp, M)
        end
    end
    nothing
end

"The mesh of the player's torn-off wheel `nm` (a Lotus name): the chosen chassis' own wheel at that corner (CARCHOICE-1)."
function loose_items(nm::String)
    cm = PLAYER_CM[]
    (cm === nothing || !PLAYER_MINE[]) && return WHEELITEMS[nm]
    i = findfirst(==(nm), WHEEL_NAMES); (i === nothing || i > length(cm.wheelspec)) && return WHEELITEMS[nm]
    get(cm.wheels, cm.wheelspec[i][5], WHEELITEMS[nm])
end

"""Hidden warm-up frame 1: compile what first runs MID-race -- a car knocked loose, SHIFT+R, a wreck's rigid body and launch,
a loose AI car's flight -- from the real types (restart_session!'s precompile stays in the loop: it names 45 main() locals).
LOOPSPLIT-1 cycle 5 S6: moved unchanged from the frame loop."""
function warm_midrace_compile!(cs, inp, dt, ai_poses, ddt, AICARS, AILINE, CLASSICAI_T, PLAYER_G, aibankP, frames, classicai_field!, groundz, groundz_ai, groundz_phys, roll_ter, CLINE)
    precompile(loose_cars_step!, typeof.((cs, ai_poses, ddt, AICARS, AILINE, CLASSICAI_T, PLAYER_G, aibankP, frames, classicai_field!, groundz, groundz_ai)))
    CLINE !== nothing && precompile(recover_step!, typeof.((cs, CLINE, groundz, PLAYER_G)))

    (CRASHBODY && CAR3D) && wreck_body_step!(cs, inp, dt, groundz_phys, roll_ter; dry = true)
    (CRASHBODY && CAR3D) && attrates3d(cs)
    (CRASHBODY && CAR3D) && WreckBody.attitude(WreckBody.Body(WreckBody.Geometry()))
    (CRASHBODY && CAR3D) && (precompile(launch_msg, (Float64, NTuple{4,Float64}));
                             precompile(body_msg, (Bool, Float64, Int, Float64, Float64, Float64));
                             precompile(landed_msg, (Float64, Float64, Float64, Tuple{Float64,Float64})))
    (CRASHBODY && CAR3D) && precompile(airbody_land!, (typeof(cs), WreckBody.Body, typeof(groundz_phys), Int, Int, WreckBody.V3, Float64))
    (CRASHBODY && CAR3D) && (precompile(WreckBody.kick!, (WreckBody.Body, WreckBody.V3, Float64));
                             precompile(player_bump!, (typeof(cs), ntuple(_ -> Float64, 6)...)))
    (CRASHBODY && CAR3D) && precompile(wreck_body_step!, typeof.((cs, inp, dt, groundz_phys, roll_ter)))

    CRASHBODY && let V = WreckBody.V3
        precompile(WreckBody._contact, (WreckBody.Body, V, Float64, V, Float64, Float64, Float64))
        precompile(WreckBody.LinearAlgebra.cross, (V, V)); precompile(WreckBody.LinearAlgebra.norm, (V,))
        precompile(WreckBody.LinearAlgebra.dot, (V, V)); precompile(*, (V, Float64))
        precompile(setindex!, (WreckBody.StaticArrays.MArray{Tuple{4},Float64,1,4}, Float64, Int))

        precompile(Render.M4, ntuple(_ -> Float32, 16)); precompile(WreckBody.StaticArrays.Args, (NTuple{16,Float32},))
    end
    if CRASHBODY && !isempty(LOOSE_CARS)
        let pc = LOOSE_CARS[1]
            loose_body_step!(1, 1, pc, 1/60, groundz; dry = true)
            attrates3d(pc); loose_wvel(1); loose_yawrate(1)
            precompile(loose_body_step!, (Int, Int, typeof(pc), Float64, typeof(groundz)))
            precompile(loose_body_start!, (Int, Int, typeof(pc), Float64))
            precompile(ai_launch_msg, (Int, Float64, Float64, Float64, Float64))
            precompile(ai_landed_msg, (Int, Bool, Float64, Float64, Float64, Tuple{Float64,Float64}))
            precompile(DriveRT3D.pin3d!, (typeof(pc), Float64, Float64, Float64, Float64))

            precompile(maximum, (typeof(pc.rh),))
        end
    end

    precompile(ai_wrecks_step!, typeof.((ai_poses, 1/60, groundz_ai, AICARS, AILINE, CLASSICAI_T, classicai_field!)))
    isempty(ai_poses) || precompile(ai_tear_wheel!, (Int, eltype(ai_poses), ntuple(_ -> Float64, 7)...))
    isempty(ai_poses) || precompile(ai_wreck_start!, (Int, eltype(ai_poses), Float64, Float64, Float64, Float64, Float64, Float64))
    precompile(ai_tear_msg, (Int, String, String, Float64))
    precompile(ai_walltear_msg, (Int, String, String, Float64))
    isempty(ai_poses) || precompile(ai_wall_tear!, (Int, WreckBody.Body, eltype(ai_poses)))
    precompile(ai_body_walls!, (Vector{WreckBody.WallPlane}, WreckBody.Body))
    precompile(replay_wheels_rec!, (Vector{Float32}, Int))
    precompile(loose_items, (String,))

    CAR3D && precompile(Core.kwcall, (NamedTuple{(:groundz,), Tuple{typeof(groundz_phys)}}, typeof(DriveRT3D.respawn3d!), typeof(cs)))
    precompile(Render.translate, (NTuple{3,Float32},))

    precompile(*, (Float64, Float32))

    precompile(isempty, (Vector{NTuple{6,Float64}},))
    nothing
end

"""LOOPSPLIT-1 cycle 6 S1: the drawn car's tilt (terrain follow + dynamic dive/squat/roll, ±6° cap), the cockpit
camera's low-pass tilt, the view (the player, a replay's focus car or an AI camera) and the car's model matrices --
moved unchanged from the frame loop. The tilt state the loop carries from frame to frame comes back with the view and
the matrices. `rep_ai_raw` is the replay's AI poses (`nothing` outside a replay)."""
function body_camera_step!(cs, inp, dt, v_prev, pitch_ter, roll_ter, pitch_dyn, cam_pitch, cam_roll, CLINE, REPLAY,
                           rep_focus, rep_swap, rep_ai_raw, rep_cam, aicam_pose)

    acc = clamp((cs.v - v_prev)/max(dt,1e-3), -15.0, 15.0); v_prev = cs.v
    pitch_ter += (terrain_pitch(cs) - pitch_ter) * min(1.0, dt*6)
    roll_ter  += (terrain_roll(cs)  - roll_ter)  * min(1.0, dt*6)
    if TILTDIAG && (abs(roll_ter) > deg2rad(10) || abs(pitch_ter) > deg2rad(10)) && TILT_N[] < 40
        TILT_N[] += 1; _ts = CLINE === nothing ? (NaN, NaN) : RaceAI.project(CLINE, cs.x, cs.z)
        println("  [tilt] t=", round(cs.t, digits = 2), " s=", round(_ts[1], digits = 1), " lat=", round(_ts[2], digits = 2),
                "  drawn roll ", round(rad2deg(roll_ter), digits = 1), " deg  pitch ", round(rad2deg(pitch_ter), digits = 1),
                " deg  (physics roll ", round(rad2deg(cs.roll), digits = 1), ")"); flush(stdout)
    end
    rollv = 0.0
    if ROLLDIAG && CAR3D && (ROLLDIAG_N[] += 1) % 30 == 0
        println("  [rolldiag] t=", round(cs.t, digits = 2), " v ", round(cs.v*3.6, digits = 0), "  terrain ", round(rad2deg(terrain_roll(cs)), digits = 2),
                "  physics ", round(rad2deg(cs.roll), digits = 2), " deg  ay ", round(hasproperty(cs, :ay) ? cs.ay/9.81 : NaN, digits = 2), " g"); flush(stdout)
    end
    if CAR3D
        pitch_dyn = (cs.pitch - pitch_ter) * SUSP_GAIN

        rollv = (ROLL_TWICE ? cs.roll : cs.roll - roll_ter) * SUSP_GAIN
    else

        pitch_dyn += (clamp(-0.010*acc, -0.06, 0.06)*SUSP_GAIN - pitch_dyn) * min(1.0, dt*4.0)
        rollv     += (clamp(-0.05*inp.steer*min(cs.v/20,1.0), -0.05, 0.05)*SUSP_GAIN - rollv) * min(1.0, dt*4.0)
    end
    pitch_dyn = clamp(pitch_dyn, -0.11, 0.11); rollv = clamp(rollv, -0.10, 0.10)
    REPLAY && (pitch_dyn = 0.0; rollv = 0.0)

    αcam = 1.0 - exp(-(dt > 1e-4 ? dt : 1/60)/CAM_TILT_TAU)
    cam_pitch += ((pitch_ter + pitch_dyn) - cam_pitch) * αcam
    cam_roll  += ((roll_ter  + rollv    ) - cam_roll ) * αcam
    vp, eye = camera(cs, cam_pitch, cam_roll)
    if REPLAY
        fp = (rep_focus[] == 0 || rep_swap[]) ? (cs.x, cs.y, cs.z, cs.θ) :
             (rf = rep_ai_raw[rep_focus[]]; (rf[1], rf[2], rf[3], rf[4]))
        _tilt = (rep_focus[] == 0 || rep_swap[]) && REPCAM_TILT
        vp, eye = replay_camera(REPLAY_CAMS[rep_cam[]], fp[1], fp[2], fp[3], fp[4], REPCAM_HEAD[];
                                pitch = _tilt ? pitch_ter : 0.0, roll = _tilt ? roll_ter : 0.0)
    end
    AICAM_R[] > 0 && aicam_pose[] !== nothing && ((vp, eye) = AIVIEW_R[] === :classicchase ? aicam_camera(aicam_pose[]) :
                                                   replay_camera(AIVIEW_R[], aicam_pose[][1], aicam_pose[][2], aicam_pose[][3], aicam_pose[][4]))
    carModel = Render.translate(cs.x, cs.y, -cs.z) * Render.roty(Float32(cs.θ)) *
               Render.rotz(Float32(pitch_ter)) * Render.rotx(Float32(roll_ter))
    tiltModel = carModel * Render.rotz(Float32(pitch_dyn)) * Render.rotx(Float32(rollv))
    if WBODY[] !== nothing
        carModel = Render.M4(WreckBody.model_matrix(WBODY[])...); tiltModel = carModel
    elseif isfinite(REP_BODY[][1])
        carModel = Render.pose_matrix(cs.x, cs.y, -cs.z, cs.θ, REP_BODY[][1], REP_BODY[][2]); tiltModel = carModel
    end
    REPLAY_BODYDIAG && (REP_DIAGN[] += 1) % 15 == 1 && (println("  [replaybody] frame ", REP_DIAGN[], " up_y ", round(carModel[2, 2], digits = 3),
                                                               " at (", round(cs.x, digits = 1), ", ", round(cs.z, digits = 1), ") y ", round(cs.y, digits = 2)); flush(stdout))
    bodyModel = tiltModel * Render.translate(BODY_OFF) * CARHAND
    (; v_prev, pitch_ter, roll_ter, pitch_dyn, rollv, cam_pitch, cam_roll, vp, eye, carModel, tiltModel, bodyModel)
end

function shadow_pass_step!(cs, depthprog, shadowfbo, bodyModel, wheelmat, loosemat, ai_poses, AICHASSIS, spin)
    lightVP = Render.light_vp(Float32[cs.x, cs.y, -cs.z], LIGHTDIR)
    Render.shadow_pass(depthprog, shadowfbo, lightVP) do dp

        TRACK_SHADOW && for (ti, it) in enumerate(trackItems)
            TRACKCULL_SH && !inlightbox(lightVP, TRACKBOUND[ti][1], TRACKBOUND[ti][2]) && continue
            Render.draw_depth(dp, it, Render.IDENT)
        end
        for it in carItems; Render.draw_depth(dp, it, bodyModel); end
        for (wx,wz,steer,r,nm) in WHEELS, it in WHEELITEMS[nm]
            is_loose(nm) && continue
            Render.draw_depth(dp, it, wheelmat(wx,wz,steer,r,nm))
        end
        for (lx,ly,lz,_,_,_,sp,_,nm) in LOOSE_WHEELS, it in loose_items(nm)
            Render.draw_depth(dp, it, loosemat(lx,ly,lz,sp))
        end
        draw_ai_depth!(dp, ai_poses, AICHASSIS, spin)
    end
    lightVP
end

function mirror_gate(REPLAY, frames)
    if MIRROR_ADAPT
        _ms = MIRROR_EMA[] * 1000
        MIRROR_STARVED[] = MIRROR_STARVED[] ? (_ms > MIRROR_RESUME_MS) : (_ms > MIRROR_ADAPT_MS)
    end
    _mir_every = (MIRROR_ADAPT && MIRROR_STARVED[]) ? MIRROR_ADAPT_N : MIRROR_EVERY
    _mir_noreplay = !REPLAY || get(ENV, "JM_MIRROR_IN_REPLAY", "1") != "0"
    _mir_alt = MIRROR_ADAPT && MIRROR_STARVED[] && get(ENV, "JM_MIRROR_ALT", "1") != "0"
    mirror_live = MIRROR_RTT && CTL.view == 0 && _mir_noreplay &&
                  (_mir_alt || _mir_every <= 1 || (frames % _mir_every) == 0)
    (MIRROR_RTT && CTL.view == 0 && _mir_noreplay && !mirror_live) && (MIRROR_SKIPPED[] += 1)
    (mirror_live, _mir_alt)
end

function main_pass_begin!(vp, eye, lightVP)
    glViewport(0,0,W,view_h())
    glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE); glDepthFunc(GL_GEQUAL); glClearDepth(0.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    Render.draw_sky(skyprog, skyvao, inv(vp), eye, LIGHTDIR;
                    cloud = GRADE.cloud, zenith = GRADE.zenith, horizon = GRADE.horizon)
    Render.set_scene_uniforms(prog, eye; fognear=400f0, fogfar=2800f0,
                              fogcol=GRADE.horizon, suncol=GRADE.suncol, ambsky=GRADE.ambsky, sat=GRADE.sat)
    Render.bind_shadow(prog, shadowtex, lightVP)
    nothing
end

function hud_draw_step!(cs, inp, tc_hud, last_lap, best_lap, cd_armed, cd_left, race_go, phase, race_done, player_finpos, HOLD_START)
    glViewport(0,0,W,H)
    Render.hud_draw(hudprog, hudvao, hudvbo,
        Render.compose_hud(W, H, cs.v*3.6, cs.gear, cs.rpm, 9500.0, inp.throttle, inp.brake, inp.clutch, tc_hud;
                           band=BAND_H,
                           lastlap=(FONT === nothing ? (SMOKE ? 94.3 : last_lap) : 0.0),
                           bestlap=(FONT === nothing ? (SMOKE ? 92.1 : best_lap) : 0.0), manual=!CTL.auto,
                           countdown=cd_left[],

                           startprompt=(START_ARM && !cd_armed[] && HOLD_START && !race_go[] &&
                                        phase[] == :race && COUNTDOWN > 0) ? mod(cs.t, 1.0) : -1.0,
                           clutchgate=(CLUTCH_GATE[] > 0 ? inp.clutch : -1.0),

                           finished=(race_done ? player_finpos[] : 0)), W, H)
    nothing
end

function replay_tyre_lp_step!(cs, dt, replay_buf, tyre_lp)
    if replay_buf !== nothing && CAR3D
        tg = DriveRT3D.tyregrip3d(cs)
        if tg !== nothing
            α_ty = clamp(dt/0.10, 0.0, 1.0)
            tyre_lp[] = ntuple(i -> ntuple(j -> tyre_lp[][i][j] + (tg[i][j] - tyre_lp[][i][j])*α_ty, 5), 4)
        end
    end
    nothing
end

function warm_replay_record!(cs, inp, AICARS, ai_poses, CLINE, REPLAY_AUTOSAVE, REPLAY_NCAR, phase, replay_names, tyre_lp)
    _rb = Float32[]; _ro = Ref(joinpath(mktempdir(), "warm.jmr"))
    replay_record!(cs, inp, AICARS, fill!(similar(ai_poses, length(AICARS)), ntuple(_ -> 0.0, 6)), CLINE, REPLAY_AUTOSAVE,
                   REPLAY_NCAR, phase, Ref(true), false, _rb, replay_names, _ro, Ref(-Inf), Ref(-1.0), Float32[], tyre_lp)
    println("  [warm] replay recorder: dry frame of ", length(_rb), " values, autosave ", isfile(_ro[]) ? "written" : "MISSING")
    flush(stdout); rm(dirname(_ro[]); recursive = true, force = true)
    nothing
end

function input_hooks_step!(cs, dt, rst, recover, restart, race_go, LASTZ)
    if SMOKE && LASTZ_AT !== nothing && !LASTZ_SET[] && cs.t >= LASTZ_AT[1]
        LASTZ[] = LASTZ_AT[2]; LASTZ_SET[] = true; println("  [test] LASTZ set to ", LASTZ_AT[2], " m at t=", round(cs.t, digits = 2)); flush(stdout)
    end
    if SMOKE && !isempty(RESTART_AT) && RESTART_I[] <= length(RESTART_AT) && cs.t >= RESTART_AT[RESTART_I[]]
        restart = true; RESTART_I[] += 1; println("  [test] restart (R) at t=", round(cs.t, digits = 2)); flush(stdout)
    end
    if SMOKE && !isempty(RECOVER_AT) && RECOVER_I[] <= length(RECOVER_AT) && cs.t >= RECOVER_AT[RECOVER_I[]]
        recover = true; rst = true; RECOVER_I[] += 1; println("  [test] recover (SHIFT+R) at t=", round(cs.t, digits = 2)); flush(stdout)
    end
    if AUTODRIVE && AUTODRIVE_RECOVER > 0 && race_go[] && cs.t > 3.0 && !WRECKED[]
        AD_SLOW_T[] = abs(cs.v) < 0.5 ? AD_SLOW_T[] + max(dt, 0.0) : 0.0
        if AD_SLOW_T[] >= AUTODRIVE_RECOVER
            recover = true; rst = true; AD_SLOW_T[] = 0.0
            println("  [auto] stuck ", AUTODRIVE_RECOVER, " s at lapdist ", round(cs.lapdist, digits = 1), " -- recover (SHIFT+R) at t=",
                    round(cs.t, digits = 2)); flush(stdout)
        end
    end
    if SMOKE && WHEELOFF_AT !== nothing && cs.t >= WHEELOFF_AT[1] && !WHEELOFF_AT_DONE[]
        WHEELOFF_AT_DONE[] = true; push!(LOOSE_WHEELS, (cs.x, 0.33, cs.z, 0.0, 2.0, 0.0, 0.0, 0.0, WHEELOFF_AT[2]))
        println("  [test] ", WHEELOFF_AT[2], " torn off at t=", round(cs.t, digits = 2)); flush(stdout)
    end
    if SMOKE && WRECK_AT > 0 && cs.t >= WRECK_AT && !WRECK_AT_DONE[]
        WRECK_AT_DONE[] = true; wreck!(max(cs.v, 20.0); closing = 20.0, x = cs.x, z = cs.z)
        haskey(ENV, "JM_WRECK_LOOSE") && push!(LOOSE_WHEELS, (cs.x, cs.y, cs.z, 0.0, 0.0, 0.0, 0.0, 0.0, "lotwlf"))
    end

    if recover && WRECKED[] && !isempty(LOOSE_WHEELS) && WHEELOFF_RULE
        recover = false; rst = false; WHEELOFF_MSG[] = 3.0
        println("  [recover] SHIFT+R refused: a wheel has come off -- R restarts the race"); flush(stdout)
    end
    (rst || restart) && CLASSICW_ON && classicw_teleported!()
    (rst, recover, restart)
end

function input_overrides(cs, inp, cd_armed, phase, race_go, HOLD_START)
    CRASH_ON && (inp = JRTrack.DriveInput(throttle = 0.0, brake = 0.0, steer = 0.0, clutch = inp.clutch,
                                             shift_up = false, shift_down = false, autoshift = true))

    if SMOKE && haskey(ENV, "JM_SHOT_STEER")
        inp = JRTrack.DriveInput(throttle = 0.0, brake = 1.0, steer = parse(Float64, ENV["JM_SHOT_STEER"]), clutch = inp.clutch,
                                    shift_up = false, shift_down = false, autoshift = true)
    end

    if RAM_AI !== nothing && RAM_STATE[].done && !(length(RAM_AI) >= 6 && RAM_AI[6] == 1)
        tr = cs.t - RAM_STATE[].t0 - RAM_AI[4]; ram = !(length(RAM_AI) >= 5 && RAM_AI[5] > 0)
        th = RAM_HIT[]; coast = ram && isnan(th) && tr < 3.0
        brake = ram ? (!isnan(th) && cs.t - th < 3.0) : tr < 3.0
        (coast || brake) && (inp = JRTrack.DriveInput(throttle = 0.0, brake = brake ? 1.0 : 0.0, steer = 0.0, clutch = inp.clutch,
                                                         shift_up = false, shift_down = false, autoshift = true))
    end

    if START_ARM && HOLD_START && !cd_armed[] && phase[] == :race && !race_go[]
        inp = JRTrack.DriveInput(throttle = 0.0, brake = 1.0, steer = inp.steer, clutch = inp.clutch,
                                    shift_up = inp.shift_up, shift_down = inp.shift_down, autoshift = inp.autoshift)
    end
    inp
end

function skiddrive_input(cs, frames)
    sd_t = Float64(cs.t); sd_i = 1; sd_t0 = 0.0
    while sd_i < length(SKIDDRIVE) && sd_t >= sd_t0 + SKIDDRIVE[sd_i].dur
        sd_t0 += SKIDDRIVE[sd_i].dur; sd_i += 1
    end
    sd_prev = sd_i == 1 ? 0.0 : SKIDDRIVE[sd_i-1].steer
    sd_k = clamp((sd_t - sd_t0) / SKIDRAMP, 0.0, 1.0)
    inp = DriveInput(throttle = SKIDDRIVE[sd_i].throttle, brake = 0.0,
                     steer = sd_prev + (SKIDDRIVE[sd_i].steer - sd_prev) * sd_k,
                     clutch = 0.0, shift_up = false, shift_down = false, autoshift = true)
    if AUTODRIVE_DIAG > 0 && (frames % AUTODRIVE_DIAG) == 0
        println("  [skid] seg ", sd_i, "/", length(SKIDDRIVE), "  t=", round(cs.t,digits=1),
                " v=", round(cs.v*3.6,digits=1), " km/h  thr=", round(inp.throttle,digits=2),
                " steer=", round(inp.steer,digits=3))
        flush(stdout)
    end
    inp
end

function practice_end_step!(cs, accelNow, enterPrev, last_lap, best_lap, race_done, lap_t0, phase, player_grid, cd_t0, launch_done, FUEL_ON, form_grid!, refuel!)
    if phase[] == :practice && ((accelNow && !enterPrev) || cs.t >= PRACTICE_SEC)
        qt = best_lap > 0.0 ? best_lap : Inf
        player_grid[] = form_grid!(qt)
        phase[] = :race
        cd_t0[] = cs.t
        cs.laps = 0; last_lap = 0.0; best_lap = 0.0; race_done = false; lap_t0 = cs.t
        launch_done[] = false
        FUEL_ON && refuel!()
    end
    (; last_lap, best_lap, race_done, lap_t0)
end

function green_light_step!(cs, inp, AICARS, HOLD_START, ai_release, phase, race_go, lap_t0, SMOKE_HOLD)
    if SMOKE_HOLD > 0 && !race_go[] && cs.t >= SMOKE_HOLD
        race_go[] = true; lap_t0 = cs.t; ai_release[] = cs.t + AI_HEADSTART
        println("  [smoke] green light at t=", round(cs.t, digits = 2)); flush(stdout)
    end

    if COUNTDOWN <= 0 && HOLD_START && !race_go[] && phase[] == :race && inp.throttle > 0.15
        race_go[] = true; lap_t0 = cs.t
        ai_release[] = cs.t + AI_HEADSTART
        AI_HEADSTART > 0 && println("  → HEAD START: you go now, the field launches in ",
                                    round(AI_HEADSTART, digits=1), " s")

        for c in AICARS
            rand() < 0.45 && (c.mishap = 0.4 + 1.4*rand())
        end
    end
    lap_t0
end

function launch_assist(cs, inp, rst, race_go, launch_done)
    race_go[] && cs.v > 12.0 && (launch_done[] = true)

    if race_go[] && !rst && !CTL.auto && !launch_done[] && inp.throttle > 0.05 &&
       get(ENV, "JM_LAUNCH_ASSIST", "0") != "0"
        inp = DriveInput(throttle=inp.throttle, brake=inp.brake, steer=inp.steer,
                         clutch=0.0, shift_up=false, shift_down=false, autoshift=true)
    end
    inp
end

function respawn_step!(cs, cs0, groundz, groundz_phys, PLAYER_G)
    groundz(cs0.x, cs0.z; acquire = true)
    PLAYER_G[] = NaN; respawnX!(cs; groundz=groundz_phys); DriveRT3D.damage_reset!(); DC[].lastz = 0.0
    WRECKED[] = false; WRECK_FROZEN[] = false; AIRBORNE[] = false; empty!(LOOSE_WHEELS); CLUTCH_GATE[] = -1.0

    nothing
end

function ai_poses_step!(cs, ddt, ai_hit, rep_ai_raw, AICARS, AIPHYS, CLASSICAI_T, phase, race_go, race_done, ai_release, aibankP, aibankK, ai_ground, groundz, groundz_ai, ai_onroad, ai_stuck, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, ent_name, classicai_field!, REPLAY, AILINE, AI_SCALE)
    ai_poses = if REPLAY
        NTuple{6,Float64}[(a[1], a[2], a[3], a[4], isfinite(a[5]) ? a[5] : 0.0, isfinite(a[6]) ? a[6] : 0.0) for a in rep_ai_raw]
    elseif AILINE === nothing || phase[] != :race
        NTuple{6,Float64}[]
    elseif !race_go[] || (AI_HEADSTART > 0 && ai_release[] >= 0.0 && cs.t < ai_release[])

        AI_PHYSICS ? [(b=aibankP(pc); (pc.x, groundz(pc.x, pc.z), pc.z, pc.θ, b[1], b[2])) for pc in AIPHYS] :
        CLASSICAI_T !== nothing ? NTuple{6,Float64}[c.pose for c in classicai_field!().cars] :
                     [(p=RaceAI.pose_at(AILINE, c.s, c.lane); b=aibankK(p); ai_ground((p[1],p[2],p[3],p[4],b[1],b[2]))) for c in AICARS]
    elseif AI_PHYSICS
        aiphys_poses!(cs, ddt, ai_hit, AICARS, AILINE, AIPHYS, ai_onroad, ai_stuck, groundz_ai, aibankP, groundz)
    elseif CLASSICAI_T !== nothing
        classicai_poses!(cs, ddt, ai_hit, AICARS, AILINE, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, CLASSICAI_T, ent_name, classicai_field!, phase, race_done, race_go)
    else
        pp = RaceAI.project(AILINE, cs.x, cs.z)
        poses, hit = RaceAI.step_field!(AICARS, AILINE, ddt; scale = AI_SCALE, player = (pp[1], pp[2], cs.v), rel = AI_REL)
        ai_hit[] = hit
        [(b=aibankK(p); ai_ground((p[1],p[2],p[3],p[4],b[1],b[2]))) for p in poses]
    end
    ai_poses
end

function ai_poses_post(cs, dt, ai_poses, AICARS, REPLAY, ai_poses_prev, CLASSICAI_T)
    if AIJIT_ON && !REPLAY && !isempty(ai_poses) && CLASSICAI_T === nothing
        ai_poses = NTuple{6,Float64}[ai_smooth(k, ai_poses[k], dt > 1e-4 ? dt : 1/60) for k in eachindex(ai_poses)]
    end
    AICAM_R[] > 0 && (ai_poses_prev[] = NTuple{6,Float64}[ntuple(j -> Float64(p[j]), 6) for p in ai_poses])
    AUTODRIVE_TRAFFIC && (AD_TRAFFIC[] = NTuple{3,Float64}[(Float64(p[1]), Float64(p[3]), k <= length(AICARS) ? Float64(AICARS[k].v) : 0.0)
                                                         for (k, p) in enumerate(ai_poses)])
    ai_poses
end

"""AUTODRIVE-MONZA-1 (2026-10-23): the autodriver follows CLINE, the centreline re-centred on the visible road (each node
moved at most 4 m). At Monza's pit exit (s 345-440) that leaves CLINE in the walled pit-exit channel beside Classic's road
(road [+3.8, +20] m, CLINE at -1.4..+2 in Classic's frame), so the autodriver hit its walls at s 409/437 every run.
* Classic's ROAD at each point is the free interval (between Classic's walls) holding Classic's own racing line (race.lp) -- neither
  the widest interval (the pit lane is wider in places) nor continuity (before the split CLINE is already on the pit-exit
  side) picks it; race.lp's lateral sign is the one under which the line and CLINE share an interval most often.
* Marked: CLINE off Classic's road in a channel narrower than `narrow` (a lane that closes) and the wider part of the same
  lane before it, or on the road within `inner` of its walls. A wide lane (Monza's pit lane before the line) is driven
  as it is: an offset there carried over the line and started the car with a lane change that ended in a spin.
* The TARGET is built in Classic's frame, not as an offset smoothed along CLINE (CLINE kinks 5.5 m in 10 m into the fork;
  a smooth offset on a kinked line is a kinked path): `margin` inside Classic's road on the marked points, that lateral held
  `hold` metres before and after while the road allows it, then averaged +-`ramp` metres back onto CLINE.
Returns (step, offsets in CLINE's frame); all zero without a race.lp or where nothing is marked (every other track)."""
function autodrive_offsets(CLINE; step = 2.0, margin = 2.5, inner = 1.0, narrow = 9.0, hold = 120.0, ramp = 120.0)
    n = max(1, floor(Int, CLINE.total / step)); off = zeros(n)
    lpf = joinpath(ZD, "race.lp"); isfile(lpf) || return (step, off)
    dl = Float64.(CLASSICLP.read_lp(lpf).dlat); N = length(dl)
    L = CLASSICWALLS.seclen; c0 = cumsum(L) .- L; lap = sum(L)
    racelat(sec, f) = dl[mod1(round(Int, (c0[sec] + f*L[sec]) / lap * N) + 1, N)]
    fi(sec, f, lat) = CLASSICWall.free_interval(CLASSICWALLS, sec, f, lat, lat; hmin = CLASSICW_HMIN)
    lat = fill(NaN, n); sg = zeros(n); sec_ = zeros(Int, n); f_ = zeros(n)
    for k in 1:n
        s = (k - 1)*step
        p0 = RaceAI.pose_at(CLINE, s, 0.0); p1 = RaceAI.pose_at(CLINE, s, 1.0)
        l0 = classicw_locate_any(p0[1], p0[3]); l1 = classicw_locate_any(p1[1], p1[3])
        (l0 === nothing || l1 === nothing || l0[1] != l1[1]) && continue
        g = l1[3] - l0[3]; abs(g) < 0.5 && continue
        lat[k] = l0[3]; sg[k] = g; sec_[k] = l0[1]; f_[k] = l0[2]
    end
    ok = findall(isfinite, lat); isempty(ok) && return (step, off)
    same(sgn) = count(k -> fi(sec_[k], f_[k], lat[k])[1:2] == fi(sec_[k], f_[k], sgn*racelat(sec_[k], f_[k]))[1:2], ok)
    sgn = same(1.0) >= same(-1.0) ? 1.0 : -1.0
    road = Dict{Int,Tuple{Float64,Float64}}(); kind = Dict{Int,Symbol}()
    for k in ok
        (lo, hi, _, _) = fi(sec_[k], f_[k], sgn*racelat(sec_[k], f_[k]))
        (isfinite(lo) && isfinite(hi) && hi - lo > 2margin + 1.0) || continue
        road[k] = (lo, hi); (wlo, whi, _, _) = fi(sec_[k], f_[k], lat[k])
        kind[k] = lo + inner <= lat[k] <= hi - inner ? :road : (wlo, whi) == (lo, hi) ? :near : whi - wlo >= narrow ? :lane : :narrow
    end
    want(k) = clamp(lat[k], road[k][1] + margin, road[k][2] - margin)
    tg = Dict{Int,Float64}()
    for k in 1:n
        get(kind, k, :none) in (:near, :narrow) || continue
        tg[k] = want(k); j = mod1(k - 1, n)
        while get(kind, j, :none) === :lane && !haskey(tg, j); tg[j] = want(j); j = mod1(j - 1, n); end
    end
    isempty(tg) && return (step, off)
    nh = round(Int, hold/step)
    for k in collect(keys(tg)), d in (-1, 1)
        haskey(tg, mod1(k + d, n)) && continue
        v = tg[k]; j = mod1(k + d, n)
        for _ in 1:nh
            (haskey(road, j) && !haskey(tg, j) && road[j][1] + margin <= v <= road[j][2] - margin) || break
            tg[j] = v; j = mod1(j + d, n)
        end
    end
    nr = round(Int, ramp/step)
    for k in ok
        any(d -> haskey(tg, mod1(k + d, n)), -nr:nr) || continue
        m = 0.0; c = 0
        for d in -nr:nr
            j = mod1(k + d, n); isfinite(lat[j]) || continue
            m += get(tg, j, lat[j]); c += 1
        end
        off[k] = (m/c - lat[k]) / sg[k]
    end
    (step, off)
end
"""AUTODRIVE-MONZA-1: the autodriver's own line -- CLINE moved by autodrive_offsets (RaceAI's lane is capped at +-3.8 m,
so the shift cannot be a lane); `nothing` where every offset is 0 (the autodriver then follows CLINE exactly)."""
function autodrive_line(CLINE, off, groundfn)
    any(!iszero, off[2]) || return nothing
    st, o = off
    RaceAI.build_line([(p = RaceAI.pose_at(CLINE, (k - 1)*st, o[k]); (p[1], p[3])) for k in eachindex(o)], groundfn)
end
ad_offset(s) = (o = AD_OFF[]; o === nothing ? 0.0 : (n = length(o[2]); o[2][mod1(round(Int, s / o[1]) + 1, n)]))
"the controller on the autodriver's own line (a function barrier: ADLINE is a Ref{Any})"
ad_control(L, cs, vt, yawrate) = (p = RaceAI.project(L, cs.x, cs.z);
    RaceAI.controller(L, p[1], p[2], AUTODRIVE_LAT, vt, cs.x, cs.z, cs.θ, cs.v, yawrate; power = 1.0))

"""E85-S6 HEADLESS AUTODRIVE (JM_AUTODRIVE=1): the AI controller drives the player on the centreline; returns the
frame's input. LOOPSPLIT-1 cycle 2 S2: moved unchanged from the frame loop -- in the loop its `let` wrote main()'s own
`ps` and `step` (and the loop's `up` / `dn` / `thr` / `brk` / `st`) as a side effect; here they are its own."""
function autodrive_step!(cs, CLINE, frames, dt, groundz, inp, ONTRACK)
    let (s0, lat0) = RaceAI.project(CLINE, cs.x, cs.z)

        if TRACE_DS
            DS_PREV[] >= 0.0 && ds_note!(s0 - DS_PREV[], cs.v, dt, CLINE.total)
            DS_PREV[] = s0
        end

        yawrate = try; y = AIyaw(cs); isfinite(y) ? y : 0.0; catch; 0.0; end

        _vt = AUTODRIVE_V
        if AUTODRIVE_PROF_PACE > 0
            AUTODRIVE_PROF[] === nothing && (AUTODRIVE_PROF[] = RaceAI.free_speed_profile(CLINE; amax = AI_AMAX))
            (ps, pv) = AUTODRIVE_PROF[]
            _k = clamp(floor(Int, mod(s0 + 2.0*cs.v, CLINE.total) / (ps[2] - ps[1])) + 1, 1, length(pv))
            _vt = min(_vt, AUTODRIVE_PROF_PACE * pv[_k])
        end

        _vt *= minimum(TT_MU)^AD_TT_EXP
        if AUTODRIVE_TRAFFIC
            for (ax, az, av) in AD_TRAFFIC[]
                (as_, al) = RaceAI.project(CLINE, ax, az)
                gap = mod(as_ - s0 + CLINE.total/2, CLINE.total) - CLINE.total/2
                (0.0 < gap < 40.0 && abs(al - lat0) < 2.2) || continue
                _vt = min(_vt, max(0.0, av + (gap - 12.0)/2.0))
            end
        end

        _adl = ADLINE[]
        thr, brk, st = _adl === nothing ?
            RaceAI.controller(CLINE, s0, lat0, AUTODRIVE_LAT, _vt, cs.x, cs.z, cs.θ, cs.v, yawrate; power = 1.0) :
            ad_control(_adl, cs, _vt, yawrate)

        if DRIVECHECK

            step = max(Float64(cs.t) - DC[].lastt, 1e-4)
            DC[].lastt = Float64(cs.t)
            gh = groundz(cs.x, cs.z)

            if !ONTRACK[]
                DC[].off_bad += 1
                if DC[].off_bad == 1 || Float64(cs.y) > DC[].off_y
                    DC[].off_s = s0; DC[].off_y = Float64(cs.y)
                end
                length(DC[].offevents) < 24 && (isempty(DC[].offevents) ||
                    abs(DC[].offevents[end][1] - s0) > 20.0) &&
                    push!(DC[].offevents, (s0, Float64(cs.y), Float64(cs.v)*3.6))
            end
            gh < DC[].ghmin && (DC[].ghmin = Float64(gh))
            if gh > -900f0
                air = Float64(cs.y) - Float64(gh)
                if air > DC[].airmax; DC[].airmax = air; DC[].air_s = s0; end
                if air > DC_AIR_MAX
                    DC[].air_bad += 1

                    if !DC[].inair && length(DC[].events) < 24
                        push!(DC[].events, (s0, air, cs.v*3.6, Float64(cs.x), Float64(cs.z)))
                    elseif DC[].inair && !isempty(DC[].events)

                        (es, epk, ekmh, ex, ez) = DC[].events[end]
                        air > epk && (DC[].events[end] = (es, air, ekmh, ex, ez))
                    end
                    DC[].inair = true
                elseif air < DC_AIR_MAX*0.5
                    DC[].inair = false
                end
            end

            if DC[].lastz != 0.0 && cs.t > DC_SETTLE_S
                vz = (Float64(cs.y) - DC[].lastz) / step
                if vz > DC[].vzmax; DC[].vzmax = vz; DC[].vz_s = s0; end
                vz > DC_VZ_MAX && (DC[].vz_bad += 1)
            end
            DC[].lastz = Float64(cs.y)
            if cs.v < DC_STUCK_V
                DC[].stuck += step
                if DC[].stuck > DC[].stuckmax; DC[].stuckmax = DC[].stuck; DC[].stuck_s = s0; end
            else
                DC[].stuck = 0.0
            end
            DC[].maxs = max(DC[].maxs, s0)
        end
        if AUTODRIVE_DIAG > 0 && (frames % AUTODRIVE_DIAG) == 0
            println("  [auto] t=", round(cs.t,digits=1), " v=", round(cs.v*3.6,digits=1),
                    " km/h  thr=", round(thr,digits=2), " brk=", round(brk,digits=2),
                    " steer=", round(st,digits=2), " s=", round(s0,digits=1), " lat=", round(lat0,digits=2),
                    " off=", round(ad_offset(s0),digits=2))
            flush(stdout)
        end
        if AUTODRIVE_MANUAL && !CTL.auto
            h = ADM[].hold
            if h == 0
                up = cs.gear == 0 || (cs.rpm > 8700 && cs.gear < 5)
                dn = !up && cs.gear > 1 && cs.rpm < 4300 && brk > 0.1
                (up || dn) && (ADM[] = (hold = 8, up = up, dn = dn))
            end
            h = ADM[].hold; ADM[] = (hold = max(h - 1, 0), up = ADM[].up, dn = ADM[].dn)

            launch = cs.v < 7.0 ? clamp(0.7 - cs.v / 10, 0.0, 1.0) : 0.0
            inp = DriveInput(throttle = h > 0 ? 0.0 : (launch > 0 ? max(clamp(thr, 0, 1), 0.5) : clamp(thr, 0, 1)),
                             brake = clamp(brk, 0, 1), steer = clamp(st, -1, 1), clutch = h > 0 ? 1.0 : launch,
                             shift_up = h == 4 && ADM[].up, shift_down = h == 4 && ADM[].dn, autoshift = false)
        else
            inp = DriveInput(throttle = clamp(thr, 0, 1), brake = clamp(brk, 0, 1),
                             steer = clamp(st, -1, 1), clutch = inp.clutch,
                             shift_up = false, shift_down = false, autoshift = true)
        end
    end
    inp
end

"""E18 replay PLAYBACK, one frame: the VCR keys, then the player's (or the swapped AI's) pose, telemetry and input from the
recording. Returns (inp, rep_ai_raw); the loop then skips the sim. LOOPSPLIT-1 cycle 2 S3: moved unchanged."""
function replay_step!(cs, inp, dt, REP_AICOCKPIT, REP_NT, REP_TEAMCOCKPIT, rep_ai_inputs, rep_ai_lotus, rep_cam, rep_dur, rep_focus, rep_pdn, rep_pend, rep_phome, rep_play, rep_pn, rep_ppd, rep_ppu, rep_psp, rep_pup, rep_pv, rep_pw, rep_rt, rep_speed, rep_swap, rep_team, rep_team_items, rep_tyres, rep_tyshow, repd, replay_poses, replay_tele)

    sp = key(GLFW.KEY_SPACE)
    if sp && !rep_psp[]
        rep_play[] = !rep_play[]
        (rep_play[] && rep_rt[] >= rep_dur - 1e-6) && (rep_rt[] = 0.0)
    end
    rep_psp[] = sp
    kh = key(GLFW.KEY_HOME); (kh && !rep_phome[]) && (rep_rt[] = 0.0); rep_phome[] = kh
    ke = key(GLFW.KEY_END);  (ke && !rep_pend[]) && (rep_rt[] = rep_dur; rep_play[] = false); rep_pend[] = ke
    kpu = key(GLFW.KEY_PAGE_UP);   (kpu && !rep_ppu[]) && (rep_rt[] = clamp(rep_rt[] - 10.0, 0.0, rep_dur)); rep_ppu[] = kpu
    kpd = key(GLFW.KEY_PAGE_DOWN); (kpd && !rep_ppd[]) && (rep_rt[] = clamp(rep_rt[] + 10.0, 0.0, rep_dur)); rep_ppd[] = kpd
    up = key(GLFW.KEY_UP);   (up && !rep_pup[]) && (rep_speed[] = clamp(rep_speed[]*2, 0.25, 8.0)); rep_pup[] = up
    dn = key(GLFW.KEY_DOWN); (dn && !rep_pdn[]) && (rep_speed[] = clamp(rep_speed[]/2, 0.25, 8.0)); rep_pdn[] = dn
    key(GLFW.KEY_RIGHT) && (rep_rt[] = clamp(rep_rt[] + 8*dt, 0.0, rep_dur))
    key(GLFW.KEY_LEFT)  && (rep_rt[] = clamp(rep_rt[] - 8*dt, 0.0, rep_dur))
    rep_play[] && (rep_rt[] = clamp(rep_rt[] + dt*rep_speed[], 0.0, rep_dur))
    (rep_play[] && rep_rt[] >= rep_dur - 1e-6) && (rep_play[] = false)
    cf = key(GLFW.KEY_C); (cf && !rep_pn[]) && (rep_focus[] = mod(rep_focus[]+1, repd.ncar)); rep_pn[] = cf
    cvv = key(GLFW.KEY_V); (cvv && !rep_pv[]) && (rep_cam[] = mod1(rep_cam[]+1, length(REPLAY_CAMS))); rep_pv[] = cvv
    (pp, rep_ai_raw) = replay_poses(rep_rt[])
    REPCAM_HEAD[] = (REPCAM_EARTH && REPLAY_CAMS[rep_cam[]] in (:chase, :tv, :f10, :nose)) ?
                    rep_travel_heading(replay_poses, rep_rt[], rep_focus[], rep_focus[] == 0 ? pp[4] : rep_ai_raw[rep_focus[]][4]) : NaN
    isempty(REP_WIDX) || replay_wheels_set!(rep_rt[]*repd.fps)
    rep_team[] = (REP_TEAMCOCKPIT && REPLAY_CAMS[rep_cam[]] === :cockpit &&
                  ((rep_focus[] >= 1 && !rep_ai_lotus(rep_focus[])) || (rep_focus[] == 0 && PLAYER_CM[] !== nothing))) ?
                 rep_team_items(rep_focus[]) : nothing
    rep_swap[] = REP_AICOCKPIT && REPLAY_CAMS[rep_cam[]] === :cockpit && rep_focus[] >= 1 && (rep_ai_lotus(rep_focus[]) || rep_team[] !== nothing)
    if rep_swap[]
        k_ = rep_focus[]; rep_ai_raw = copy(rep_ai_raw); (pp, rep_ai_raw[k_]) = (rep_ai_raw[k_], pp)
    end
    cs.x = pp[1]; cs.y = pp[2]; cs.z = pp[3]; cs.θ = pp[4]; cs.v = 0.0; cs.pitch = 0.0; cs.roll = 0.0
    REP_BODY[] = (!rep_swap[] && isfinite(pp[5])) ? (pp[5], pp[6]) : (NaN, NaN)
    if REP_NT > 0
        rt_ = replay_tele(rep_rt[])
        cs.v = rt_.speed; cs.rpm = rt_.rpm; cs.gear = rt_.gear
        inp = DriveInput(throttle = rt_.throttle, brake = rt_.brake, steer = rt_.steer, clutch = rt_.clutch)
        rep_tyres[] = rt_.tyres
        REP_TT_LIVE[] = rt_.tt !== nothing
        if rt_.tt !== nothing
            for w in 1:4, z in 1:3; TT_SURF[w, z] = rt_.tt[3(w-1)+z]; TT_CARC[w, z] = rt_.tt[12+3(w-1)+z]; end
            REPLAY_TTDIAG && (REP_DIAGN[] += 1) % 60 == 0 &&
                println("  [reptt] t ", rep_rt[], " carcass FL m ", round(TT_CARC[1, 2]; digits = 2), " surface RR o ", round(TT_SURF[4, 3]; digits = 2))
        end
    end
    if rep_swap[]
        ai_ = rep_ai_inputs(rep_rt[], rep_focus[])
        cs.v = ai_.v; cs.rpm = ai_.rpm; cs.gear = ai_.gear; rep_tyres[] = nothing; REP_TT_LIVE[] = false
        inp = DriveInput(throttle = 0.0, brake = 0.0, steer = ai_.steer, clutch = 0.0)
    end
    kw = key(GLFW.KEY_W); (kw && !rep_pw[]) && (rep_tyshow[] = !rep_tyshow[]); rep_pw[] = kw

    CTL.view = (REPLAY_CAMS[rep_cam[]] === :cockpit && (rep_focus[] == 0 || rep_swap[])) ? 0 : 1
    (inp, rep_ai_raw)
end

"""Lap timing, one frame: a completed lap is timed, banked and logged; at race distance the classification, the result
file and the recent-average bank. Returns the five lap-state locals main() keeps. LOOPSPLIT-1 cycle 2 S4: moved unchanged."""
function lap_timing_step!(cs, prev_laps, lap_t0, last_lap, best_lap, race_done, phase, player_laps, telem, AICARS, standings, player_finpos, player_grid, AI_TGT, player_prog, CLINE, FUEL_ON, AILINE, ai_best, ent_name, fmt_lap, save_human_best, banking_allowed, plausible_lap)
    if cs.laps > prev_laps
        last_lap = cs.t - lap_t0; lap_t0 = cs.t
        improved = best_lap == 0.0 || last_lap < best_lap
        improved && (best_lap = last_lap)
        phase[] == :race && push!(player_laps, last_lap)

        improved && cs.laps >= 1 && save_human_best(TRACKSEL, best_lap)
        telem !== nothing && (write(telem, "# LAP $(cs.laps)  $(fmt_lap(last_lap))\n"); flush(telem))
        println(phase[] == :practice ? "  practice lap: $(fmt_lap(last_lap))" : "  lap $(cs.laps): $(fmt_lap(last_lap))",
                last_lap==best_lap ? "  (best)" : "")
        begin
        if phase[] == :race && cs.laps >= RACE_LAPS && !race_done
            race_done = true
            println("\n  ═══════ RACE FINISHED — $RACE_LAPS laps ═══════")
            order = isempty(AICARS) ? [(0, 0.0)] : standings()
            player_finpos[] = findfirst(e -> e[1] == 0, order)
            println("\n  ══════ YOU FINISHED — P$(player_finpos[]) of $(length(order)) ══════")
            println("  started P$(player_grid[])   best lap $(fmt_lap(best_lap))   total $(fmt_lap(cs.t))")

            println("  ── R1 DIAG  (AI pace ", round(Int,AI_PCT), "% → target ", round(AI_TGT,digits=1),
                    "s/lap;  YOU laps=", cs.laps, " prog=", round(player_prog/(CLINE===nothing ? 1 : CLINE.total), digits=2), ") ──")

            let _old = cs.laps + (FUEL_ON && LAPLEN > 0 ? clamp(cs.lapdist/LAPLEN, 0.0, 1.0) : 0.0),
                _new = CLINE === nothing ? Float64(cs.laps) : player_prog/CLINE.total
                println("     YOU     ranked pp=", round(standings()[findfirst(e -> e[1] == 0, standings())][2], digits=3),
                        "   [old formula ", round(_old, digits=3),
                        " = laps ", cs.laps, " + lapdist ", round(cs.lapdist, digits=1),
                        "/", round(LAPLEN, digits=1), "]",
                        "   [accumulator ", round(_new, digits=3), "]")
            end
            for (i,c) in enumerate(AICARS)

                println("     ", rpad(AISPECS[i][1],8), " lap=", c.lap, " prog=", round(c.lap + mod(c.s, AILINE.total)/AILINE.total, digits=2),
                        " v=", round(Int,c.v*3.6), "km/h pace=", round(Int,c.pace*100), "%",
                        "  best=", isfinite(ai_best[i]) ? fmt_lap(ai_best[i]) : "-")
            end
            println("  ── final classification ──")
            for (p, (id, _)) in enumerate(order)
                println("   P$p  ", ent_name(id), id==0 ? "  ← YOU (best $(fmt_lap(best_lap)))" : "")
            end
            println()

            lead_prog = order[1][2]
            est_time(prog) = cs.t * RACE_LAPS / max(prog, 0.01)
            lead_est = est_time(lead_prog)

            best_any = best_lap > 0 ? best_lap : Inf; best_any_who = best_lap > 0 ? "You" : "-"
            for i in 1:length(AICARS)
                if ai_best[i] < best_any; best_any = ai_best[i]; best_any_who = ent_name(i); end
            end

            try

                open(SMOKE ? "/tmp/jm_smoke_last_race_result.txt" : joinpath(@__DIR__, "last_race_result.txt"), "w") do io
                    println(io, "track\t$(TRACKSEL)"); println(io, "laps\t$RACE_LAPS")
                    println(io, "you_pos\t$(player_finpos[])"); println(io, "field\t$(length(order))")
                    println(io, "you_start\t$(player_grid[])")
                    println(io, "you_best\t$(best_lap > 0 ? fmt_lap(best_lap) : "-")"); println(io, "you_total\t$(fmt_lap(cs.t))")
                    println(io, "best_any\t$(isfinite(best_any) ? fmt_lap(best_any) : "-")\t$best_any_who")
                    println(io, "you_laps\t", join((fmt_lap(t) for t in player_laps), ","))
                    println(io, "win_total\t$(fmt_lap(lead_est))")
                    for (p, (id, prog)) in enumerate(order)
                        laps_down = floor(Int, lead_prog - prog)

                        gap = p == 1 ? fmt_lap(lead_est) :
                              (laps_down >= 1 ? "+$laps_down lap$(laps_down>1 ? "s" : "")" :
                                                "+$(fmt_lap(est_time(prog) - lead_est))")
                        println(io, "P$p\t$(ent_name(id))\t$gap")
                    end
                end
            catch e; @warn "result write failed" e; end

            if !isempty(player_laps) && banking_allowed()
                avg = sum(player_laps)/length(player_laps)
                if !plausible_lap(TRACKSEL, avg)
                    println("  ⚠ not banking an implausible race average for ", TRACKSEL, ": ",
                            round(avg, digits=3), " s")
                else
                try
                    rp = joinpath(@__DIR__, "human_recent.txt"); recents = Dict{String,Float64}()
                    isfile(rp) && for ln in eachline(rp)
                        sp = split(strip(ln), '\t'); length(sp)==2 && (v=tryparse(Float64,sp[2]))!==nothing && (recents[sp[1]]=v)
                    end
                    recents[TRACKSEL] = avg
                    open(rp, "w") do io; for (k,v) in recents; println(io, "$k\t$(round(v,digits=3))"); end; end
                catch e; @warn "human_recent write failed" e; end
                end
            end
        end
        end
    elseif cs.laps < prev_laps
        lap_t0 = cs.t
    end
    prev_laps = cs.laps
    (prev_laps = prev_laps, lap_t0 = lap_t0, last_lap = last_lap, best_lap = best_lap, race_done = race_done)
end

"""One reference .ibt telemetry row for this frame (the channels the gold tools read). LOOPSPLIT-1 cycle 2 S5: moved unchanged."""
function ibt_sample!(ibt_samples, cs, inp, fuel)
    tl = telemetryX(cs); δw = inp.steer * (CAR3D ? DriveRT3D.MAXSTEER : SKIDPAD ? 0.30 : CAR.max_steer)
    dials_update!(cs.rpm)
    row = Dict{String,Float64}(
        "SessionTime"=>cs.t, "SessionTick"=>Float64(length(ibt_samples)+1),
        "IsOnTrack"=>cs.ontrack ? 1.0 : 0.0,
        "Speed"=>cs.v, "RPM"=>cs.rpm, "Gear"=>Float64(cs.gear),
        "Throttle"=>inp.throttle, "Brake"=>inp.brake, "Clutch"=>1.0-inp.clutch,

        "SteeringWheelAngle"=>δw*STEER_RATIO[], "SteeringWheelAngleMax"=>deg2rad(DriveRT3D.WHEEL_LOCK_DEG), "FrameRate"=>FRAME_FPS[],
        "Lap"=>Float64(cs.laps+1), "LapCompleted"=>Float64(cs.laps),
        "LapDist"=>cs.lapdist, "LapDistPct"=>(SKIDPAD ? 0.0 : (LAPLEN>0 ? cs.lapdist/LAPLEN : 0.0)),
        "Yaw"=>cs.θ, "YawRate"=>tl.r,
        "VelocityX"=>tl.u, "VelocityY"=>tl.v, "VelocityZ"=>0.0,

        "LongAccel"=>tl.ax, "LatAccel"=>tl.ay, "VertAccel"=>(CAR3D ? tl.vacc : 9.80665),
        "LFspeed"=>tl.ωf*0.30, "RFspeed"=>tl.ωf*0.30, "LRspeed"=>tl.ωRL*DriveRT3D.RW_R, "RRspeed"=>tl.ωRR*DriveRT3D.RW_R,
        "Alt"=>cs.y,

        "FuelLevel"=>fuel[], "FuelLevelPct"=>fuel[]/DriveRT3D.FUEL_MAX_L,

        "TrackWetness"=>WETNESS, "WeatherDeclaredWet"=>(WETNESS >= 6 ? 1 : 0),
        "ManifoldPress"=>manifold_bar(inp.throttle, cs.rpm),
        "FuelUsePerHour"=>DriveRT3D.fuel_rate_kgh(inp.throttle, cs.rpm),
        "OilPress"=>oil_bar(cs.rpm), "FuelPress"=>(cs.rpm > 200 ? FUEL_BAR : 0.0), "Voltage"=>12.0,
        "OilTemp"=>DIALV.oiltemp, "WaterTemp"=>DIALV.water, "OilLevel"=>6.0, "WaterLevel"=>4.0,
        "WindVel"=>abs(WIND_V), "WindDir"=>WIND_DIR,
        "EngineWarnings"=>Float64(engine_warnings(cs.rpm, 9500.0*DriveRT3D.CAR[].kR)))

    for (i, w) in enumerate(("LF", "RF", "LR", "RR")), (z, zn) in enumerate(("L", "M", "R"))
        row[w * "temp" * zn] = TT_SURF[i, z]; row[w * "tempC" * zn] = TT_CARC[i, z]; row[w * "wear" * zn] = TT_WEAR[i, z]
    end
    if CAR3D

        _rhok = get(ENV,"JM_RH_RAW","") != "" || getproperty(tl, :grounded)
        row["LFrideHeight"]= _rhok ? tl.rh[1] : NaN
        row["RFrideHeight"]= _rhok ? tl.rh[2] : NaN
        row["LRrideHeight"]= _rhok ? tl.rh[3] : NaN
        row["RRrideHeight"]= _rhok ? tl.rh[4] : NaN
        row["Pitch"]=tl.pitch; row["Roll"]=tl.roll
        bp = DriveRT3D.brakepress3d(cs)
        if bp !== nothing
            row["LFbrakeLinePress"] = bp[1]; row["RFbrakeLinePress"] = bp[1]
            row["LRbrakeLinePress"] = bp[2]; row["RRbrakeLinePress"] = bp[2]
        end
    end
    if WBODY[] !== nothing
        w = WB_TELE[]
        row["YawRate"] = w.r; row["LongAccel"] = w.ax; row["LatAccel"] = w.ay; row["VertAccel"] = w.az
    end
    push!(ibt_samples, row)
    nothing
end

"""Force feedback, one frame: the rim torque (or the legacy road term), de-spiked, squelched, low-passed, plus any jolt.
Returns the two filter states (fy_lp, ffb_f). LOOPSPLIT-1 cycle 2 S5: moved unchanged."""
function ffb_step!(ffb, cs, inp, dt, fy_lp, ffb_f, ffb_jolt)
    τrim = (FFB_PHYS && CAR3D) ? DriveRT3D.rimtorque3d(cs) : nothing
    if τrim !== nothing
        fy = -τrim / FFB_TREF; trail = 1.0
    else
        tl  = telemetryX(cs)
        αf  = atan(tl.v + FFB_AF*tl.r, max(tl.u, 1.0)) - clamp(inp.steer, -1, 1)*FFB_DELTA
        trail = FFB_TFLOOR + (1 - FFB_TFLOOR) * clamp(1 - abs(αf)/FFB_ATRAIL, 0.0, 1.0)
        fy = cs.tc[1][2] + cs.tc[2][2]
    end
    fy_lp += (fy - fy_lp) * clamp(dt/FFB_DESPIKE, 0.0, 1.0)
    fy = fy_lp * fy_lp*fy_lp / (fy_lp*fy_lp + FFB_SQ*FFB_SQ)
    mz  = fy * trail

    (CRASHBODY && (WRECKED[] || AIRBORNE[])) && (mz = 0.0; τrim = nothing)
    spd = clamp(cs.v/2.5, 0.0, 1.0)

    spr = FFB_SPRING * clamp(STEER_GAIN[] > 0 ? inp.steer / STEER_GAIN[] : inp.steer, -1, 1)

    target = if FFB_IRMODE && τrim !== nothing
        clamp(FFB_SIGN * (mz * spd * FFB_TREF / FFB_MAXNM) + FFB_SIGN * FFB_SPRING_IR * clamp(STEER_GAIN[] > 0 ? inp.steer / STEER_GAIN[] : inp.steer, -1, 1), -1.0, 1.0)
    else
        tanh(FFB_SIGN * (FFB_GAIN * mz * spd + spr))
    end
    ffb_f += (target - ffb_f) * clamp(dt/FFB_LP, 0.0, 1.0)

    FFB.set_force!(ffb, clamp(ffb_f*(1.0 - min(abs(ffb_jolt), 1.0)) + ffb_jolt, -1.0, 1.0))
    (fy_lp, ffb_f)
end

"""The player against the local AI field, one frame: the draft tow, then each touching car's rigid-body contact
(the player's kick + damage, the AI's response -- loose Car3D, physics AI or the field's nudge). Returns ffb_jolt.
LOOPSPLIT-1 cycle 2 S6: moved unchanged."""
function ai_contact_step!(cs, ai_poses, AICARS, AIPHYS, PLAYER_CDA, ffb_jolt)

    if CAR3D && (ph = JRTrack.hat(TRKSURF, cs.x, cs.z); ph.found && abs(ph.lateral) <= ROAD_HALFW)
        plds = [(p[1], p[3], p[4], AICARS[k].v) for (k,p) in enumerate(ai_poses)]
        ptw = draft_tow(cs.x, cs.z, cs.θ, cs.v, plds)
        ptw > 0.0 && (PLAYER_CDA[] = 1.0 - DRAFT_DRAG_CUT * clamp(ptw/TOW_MAX, 0.0, 1.0))
    end
    pm = 560.0; am = 560.0

    pvx = WVX[]; pvz = WVZ[]
    for (k, p) in enumerate(ai_poses)
        dx = p[1] - cs.x; dz = p[3] - cs.z; d = hypot(dx, dz)
        (d < 1e-3 || !car_near(dx, dz, d, cs.θ, p[4])) && continue
        nx = dx/d; nz = dz/d
        ac = AICARS[k]; aθ = p[4]
        lat = -dx*sin(cs.θ) + dz*cos(cs.θ)

        ffb_jolt = clamp(-sign(lat)*0.6, -1.0, 1.0)
        avx = ac.v*cos(aθ); avz = ac.v*sin(aθ)
        if haskey(LOOSE_OF, k)
            (avx, avz) = loose_wvel(LOOSE_OF[k])
        end

        imp = car_contact_impulse(cs.x, cs.z, cs.θ, pvx, pvz, player_yawrate(cs), p[1], p[3], avx, avz; θb = aθ)
        imp === nothing && continue
        (pdvx, pdvz, pdr, j) = imp
        (RAM_AI !== nothing && isnan(RAM_HIT[])) && (RAM_HIT[] = cs.t)
        if CARHIT_TRACE
            g = car_contact_geom(cs.x, cs.z, cs.θ, p[1], p[3], aθ; margin = 0.1)
            pp = g === nothing ? "-" : string(something(findfirst(==(g[7]), CAR_PARTS), 0), "-", something(findfirst(==(g[8]), CAR_PARTS), 0))
            println("  [carhit-trace] t ", round(cs.t, digits = 3), " AI ", k, " lat ", round(lat, digits = 2), " parts ", pp,
                    " jn ", round(j), " dv ", round(hypot(pdvx, pdvz), digits = 2), " dr ", round(pdr, digits = 2),
                    " heave ", round(imp[5], digits = 2), " roll ", round(imp[6], digits = 2), " pitch ", round(imp[7], digits = 2),
                    " airborne ", AIRBORNE[], " wrecked ", WRECKED[])
        end
        player_bump!(cs, pdvx, pdvz, pdr, imp[5], imp[6], imp[7])
        carhit_damage!(cs, pdvx, pdvz, j)
        ffb_jolt = clamp(ffb_jolt - sign(lat)*(j/pm)*0.4 - 0.5, -1.0, 1.0)
        if !AI_PHYSICS && loose_hit!(k, p, aθ, avx, avz, cs, pvx, pvz, player_yawrate(cs), cs.t)

        elseif AI_PHYSICS
            alat = -dx*sin(aθ) + dz*cos(aθ)

            AIbump!(AIPHYS[k], (j/am)*nx, (j/am)*nz, clamp(sign(alat)*(j/am)*0.05, -1.0, 1.0))
        else

            along  = nx*cos(aθ) + nz*sin(aθ); across = -nx*sin(aθ) + nz*cos(aθ)
            ac.v   = max(0.0, ac.v + clamp((j/am)*along, -8.0, 2.5))
            ac.lane = clamp(ac.lane + clamp((j/am)*across*0.04, -0.7, 0.7), -RaceAI.LANE_MAX, RaceAI.LANE_MAX)
            ac.spin += clamp((j/am)*across*0.06, -0.8, 0.8)
        end
    end
    ffb_jolt
end

"""JM_CRASH harness, one frame (CLASSICWALL-1): teleports the car into the next scripted crash, watches it for going THROUGH
a wall, scores the penetration, and prints one line per scenario. LOOPSPLIT-1 cycle 3 S1: moved unchanged."""
function crash_step!(cs, frames, place_at_s!, crash_i, crash_invis, crash_jit, crash_ninv, crash_plat, crash_rec, crash_sum, crash_t0, crash_thru)
    if crash_i[] == 0 || frames - crash_t0[] >= CRASH_FRAMES
        if crash_i[] > 0
            (s0, sd, ang, vv) = CRASHES[crash_i[]]
            r = classicw_locate_any(cs.x, cs.z)
            (lo, hi) = r === nothing ? (NaN, NaN) : classicw_road_interval(r[1], r[2])
            ok = !crash_thru[]
            crash_invis[] > 0 && (crash_ninv[] += 1)
            (wp, sp, bp) = crash_rec[]
            crash_sum[] = (crash_sum[][1] + 1, crash_sum[][2] + (ok ? 0 : 1), max(crash_sum[][3], wp), max(crash_sum[][4], sp))
            println("  [crash] ", crash_i[], "/", length(CRASHES), " s=", round(Int, s0), " ", sd > 0 ? "L" : "R", " ",
                    round(Int, ang), "deg ", round(Int, vv), " m/s: wall pen max ", round(wp, digits = 2),
                    " m, drawn-face pen max ", round(sp, digits = 2), " m, old-box pen max ", round(bp, digits = 2), " m, end lat ", r === nothing ? "?" : round(r[3], digits = 2),
                    " in [", round(lo, digits = 1), ", ", round(hi, digits = 1), "] -> ", ok ? "CONTAINED" : "THROUGH",
                    WRECKED[] ? "  (wrecked)" : "", "  v_end ", round(cs.v, digits = 1),
                    isfinite(crash_jit[][1]) ? string("  REST JITTER y ", round(crash_jit[][2] - crash_jit[][1], digits = 3), " m, heading ",
                        round(rad2deg(crash_jit[][4] - crash_jit[][3]), digits = 2), " deg") : "",
                    crash_invis[] > 0 ? "  INVISIBLE-CONTACT frames $(crash_invis[])" : ""); flush(stdout)
        end
        crash_i[] += 1
        if crash_i[] <= length(CRASHES)
            (s0, sd, ang, vv) = CRASHES[crash_i[]]
            place_at_s!(s0)
            DriveRT3D.damage_reset!(); WRECKED[] = false; WRECK_FROZEN[] = false; AIRBORNE[] = false; empty!(LOOSE_WHEELS); DC[].lastz = 0.0
            r = classicw_locate_any(cs.x, cs.z)
            if r !== nothing
                (_, _, _, gx, gz) = r; a = deg2rad(ang)
                dx = cos(cs.θ)*cos(a) + sd*gx*sin(a); dz = sin(cs.θ)*cos(a) + sd*gz*sin(a)
                DriveRT3D.place3d!(cs, cs.x, cs.z, atan(dz, dx); v = vv)
            end
            classicw_teleported!(); OBS_STAT[] = (hits = 0, clamps = 0, maxpen = 0.0)
            crash_t0[] = frames; crash_rec[] = (0.0, 0.0, 0.0)
            crash_plat[] = NaN; crash_thru[] = false; crash_invis[] = 0; crash_jit[] = (Inf, -Inf, Inf, -Inf)
        else
            println("  [crash] SUMMARY ", TRACKSEL, ": ", crash_sum[][1], " scenarios, THROUGH ", crash_sum[][2],
                    ", deepest Classic-wall penetration ", round(crash_sum[][3], digits = 2), " m, deepest drawn-face penetration ",
                    round(crash_sum[][4], digits = 2), " m; scenarios with an INVISIBLE-wall contact ", crash_ninv[],
                    "; wall+obstacle cost ", round(1e6*CLASSICW_T[][1]/max(CLASSICW_T[][2], 1), digits = 1), " us/frame"); flush(stdout)
        end
    else
        WRECK_FROZEN[] && (j_ = crash_jit[]; crash_jit[] = (min(j_[1], cs.y), max(j_[2], cs.y), min(j_[3], cs.θ), max(j_[4], cs.θ)))
        (_, _, _, glo, ghi, _, _, _, _, gend_) = classicwall_gaps(cs.x, cs.z, cs.θ)
        glo = min(glo, gend_)

        rr = classicw_locate(cs.x, cs.z)
        if rr !== nothing
            (sc_, fc_, lc_) = rr
            Icur = CLASSICWall.free_interval(CLASSICWALLS, sc_, fc_, lc_, lc_; hmin = CLASSICW_HMIN)
            inwall = !(Icur[1] <= lc_ <= Icur[2])
            crossed = false
            if !isnan(crash_plat[])
                Iprev = CLASSICWall.free_interval(CLASSICWALLS, sc_, fc_, crash_plat[], crash_plat[]; hmin = CLASSICW_HMIN)
                crossed = (Iprev[1] <= crash_plat[] <= Iprev[2]) && !inwall && Iprev[1:2] != Icur[1:2] &&
                          abs(crash_plat[] - lc_) < 5.0
            end

            (inwall && min(abs(lc_ - Icur[1]), abs(lc_ - Icur[2])) > CARW) && (crash_thru[] = true)
            crossed && (crash_thru[] = true)
            if get(ENV, "JM_CRASH_TRACE", "0") != "0" && frames - crash_t0[] <= parse(Int, get(ENV, "JM_CRASH_TRACE_N", "90"))
                (_, _, _, glo2, ghi2) = classicwall_gaps(cs.x, cs.z, cs.θ)
                println("   [ctrace] f", frames - crash_t0[], " x ", round(cs.x, digits = 1), " z ", round(cs.z, digits = 1),
                        " sec ", sc_, " f ", round(fc_, digits = 2), " lat ", round(lc_, digits = 2), " I ", round.(Icur[1:2], digits = 2),
                        " prev ", round(CLASSICW_PREV[], digits = 2), " glo ", round(glo2, digits = 2), " ghi ", round(ghi2, digits = 2),
                        " v ", round(cs.v, digits = 1), inwall ? " INWALL" : "", crossed ? " CROSSED" : "")
            end
            crash_plat[] = lc_
        end

        if min(glo, ghi) < 0.0
            near = false
            ci = floor(Int, cs.x/OBS_C); cj = floor(Int, cs.z/OBS_C)
            for gi in ci-1:ci+1, gj in cj-1:cj+1, (ax, az, bx, bz) in get(DRAWN_GRID, (gi, gj), NTuple{4,Float64}[])
                seg_seg(cs.x + CARLF*cos(cs.θ), cs.z + CARLF*sin(cs.θ), cs.x - CARLF*cos(cs.θ), cs.z - CARLF*sin(cs.θ),
                        ax, az, bx, bz)[1] < CARW + 1.0 && (near = true; break)
            end
            near = near || any(k -> car_gap(cs.x, cs.z, cs.θ, k)[1] < 1.0, eachindex(SOLIDS))
            if !near
                (_, _, _, _, _, gx_, gz_) = classicwall_gaps(cs.x, cs.z, cs.θ)
                sgn_ = glo < ghi ? -1.0 : 1.0
                h0 = JRTrack.hat3d(TERRAIN, cs.x, cs.z; ref = Inf)
                for dd in 0.5:0.5:(CARW + 2.0)
                    h = JRTrack.hat3d(TERRAIN, cs.x + sgn_*gx_*dd, cs.z + sgn_*gz_*dd; ref = Inf)
                    (h0[3] && h[3] && h[1] - h0[1] >= 0.25) && (near = true; break)
                end
            end
            near || (crash_invis[] += 1)
        end
        bpn = 0.0
        for k in eachindex(SOLIDS); bpn = max(bpn, -car_gap(cs.x, cs.z, cs.θ, k)[1]); end
        opn = 0.0
        if OBS_ON
            (ppx, ppz) = OBS_PREV[]; wi = 0
            for i in obs_near(cs.x, cs.z)
                OBS_KIND[i] === :soft && continue
                g_ = -obs_gap(i, cs.x, cs.z, cs.θ, ppx, ppz)[1]
                g_ > opn && (opn = g_; wi = i)
            end
            if wi > 0 && opn > 0.35 && get(ENV, "JM_CRASH_TRACE", "0") != "0"
                sg = OBS_SEG[wi]
                println("   [otrace] f", frames - crash_t0[], " obstacle ", wi, " ", OBS_NAME[wi], " len ", round(hypot(sg[3]-sg[1], sg[4]-sg[2]), digits = 2),
                        " r ", sg[5], " pen ", round(opn, digits = 2), " car (", round(cs.x, digits = 2), ",", round(cs.z, digits = 2), ") θ ",
                        round(rad2deg(cs.θ), digits = 1), " seg (", round.(sg[1:4], digits = 2), ")")
            end
        end
        crash_rec[] = (max(crash_rec[][1], -min(glo, ghi)), max(crash_rec[][2], opn), max(crash_rec[][3], bpn))
    end
    nothing
end

"""The player car's physics step inputs, one frame (CAR3D): object/fence/wreck forces, loose wheels, per-wheel surface
grip, then the step itself. Returns (cfx, cfy, cpk, ffb_jolt). LOOPSPLIT-1 cycle 3 S2: moved unchanged."""
function player_step3d!(cs, dt, rst, inp, groundz, ffb_jolt, groundz_phys, race_go, BND_FX, BND_FY, BND_MZ, OFFDIST, OFFTRACK_PROBE,
                        ONTRACK, PLAYER_CDA)
    cfx = cfy = cmz = 0.0
    cpk = 0.0
    if !SKIDPAD && !rst
        update_world_velocity!(cs, cs.x, cs.z, dt > 1e-4 ? dt : 1/60)
        (cfx, cfy, cmz, cpk, chard, cclose) = solid_contact(cs.x, cs.z, cs.θ, cs.v, dt > 1e-4 ? dt : 1/60)

        if CLASSICW_PHYS && CAR3D && !(WRECK_FROZEN[] && WRECK_NOCLAMP)
            _tgw = time()
            (gfx, gfy, gmz, gpk, ghd, gcl) = classicwall_contact(cs.x, cs.z, cs.θ, dt > 1e-4 ? dt : 1/60)
            if OBS_ON
                (ofx, ofy, omz, opk, ohd, ocl) = obs_contact(cs.x, cs.z, cs.θ, dt > 1e-4 ? dt : 1/60)
                gfx += ofx; gfy += ofy; gmz += omz; gpk = max(gpk, opk); ghd = max(ghd, ohd); gcl = max(gcl, ocl)
            end
            CLASSICW_T[] = (CLASSICW_T[][1] + time() - _tgw, CLASSICW_T[][2] + 1)
            if gpk > 0.0
                cθ_ = cos(cs.θ); sθ_ = sin(cs.θ)
                (cfx, cfy, sc_) = DriveRT3D.cap_total_contact(cfx + gfx, cfy + gfy, WVX[]*cθ_ + WVZ[]*sθ_,
                                                             -WVX[]*sθ_ + WVZ[]*cθ_; dt = dt > 1e-4 ? dt : 1/60)
                cmz = (cmz + gmz)*sc_; cpk = max(cpk, gpk); chard = max(chard, ghd); cclose = max(cclose, gcl)
            end
        end

        hitpk = max(cpk, BND_PK[])

        if DriveRT3D.wrecks(cclose, BND_PK[], cs.v;
                            close_ms = WRECK_CLOSE, bnd_peak_max = 1.0e3,
                            vmin_ms = WRECK_MS)
            wreck!(abs(cs.v); closing = cclose, bnd_peak = BND_PK[],
                   x = cs.x, z = cs.z)
        end

        STALL_T[], _stallfire = DriveRT3D.stall_step(STALL_T[], dt;
            auto = CTL.auto, wrecked = WRECKED[], rpm = cs.rpm, clutch = CLU_NOW[],
            rpm_floor = STALL_RPM, secs = STALL_SECS, speed = abs(cs.v), vmax = 5.0)
        if _stallfire
            CTL.auto = true
            println("  [gearbox] ENGINE STALLED (", round(Int, cs.rpm),
                    " rpm, clutch engaged) -- switched to AUTO")
            flush(stdout)
        end

        if WRECKED[] && hitpk > 1.0e3 && abs(cs.v) > 1.0
            detach_hit_wheels!(cs.x, cs.z, cs.θ, cs.v;
                               fence = BND_PK[] > 1.0e4,
                               fence_nx = BND_NX[], fence_nz = BND_NZ[])
        end

        if cpk > 1.0e3 && !WRECKED[] && cclose > 0.0
            DriveRT3D.damage_impact!(cfx, cfy, cclose)
            if DriveRT3D.damaged()

                DMG_RUN_N[] += 1
                if DMG_EVERY <= 0.0 || cs.t - DMG_LAST_T[] >= DMG_EVERY
                    println("  [damage] contact at ", round(cclose, digits=1),
                            " m/s closing, car ", round(abs(cs.v), digits=1), " m/s",
                            DMG_RUN_N[] > 1 ? string("  (x", DMG_RUN_N[], " since the last line)") : "",
                            " -- corner grip now ",
                            round(100*DriveRT3D.damage_mu(1)), "%")
                    flush(stdout)
                    DMG_LAST_T[] = cs.t
                    DMG_RUN_N[]  = 0
                end
            end
        end
        cpk > 1.0e3 && (ffb_jolt = clamp(sign(cmz != 0 ? cmz : 1.0) * min(cpk/4.0e4, 1.0), -1.0, 1.0))

        if STUCK_SECS > 0 && cpk > 1.0e3 && race_go[] && !WRECKED[] && abs(cs.v) < 0.5
            if STUCK_T0[] < 0.0
                STUCK_T0[] = cs.t

                haskey(ENV,"JM_TRACE_STUCK") && println("  [stuck?] armed at t=",
                    round(cs.t, digits=2), "s  |v|=", round(abs(cs.v), digits=2),
                    " m/s at (", round(cs.x, digits=1), ", ", round(cs.z, digits=1), ")")
            elseif cs.t - STUCK_T0[] >= STUCK_SECS
                println("\n  ═══════ STUCK — not wrecked, pinned against something ═══════")
                println("  [STUCK] ", round(cs.t - STUCK_T0[], digits=1),
                        " s at |v| < 0.5 m/s with a contact every frame")
                println("  [STUCK]   at world (", round(cs.x, digits=1), ", ",
                        round(cs.z, digits=1), ")  lap ", cs.laps,
                        "  lapdist ", round(cs.lapdist, digits=1))
                println("  [STUCK]   JM_STUCK_SECS=0 disables this watchdog")
                flush(stdout)
                GLFW.SetWindowShouldClose(win, true)
            end
        elseif STUCK_T0[] >= 0.0 && abs(cs.v) >= 0.5
            haskey(ENV,"JM_TRACE_STUCK") && println("  [stuck?] cleared after ",
                round(cs.t - STUCK_T0[], digits=2), " s -- moving again")
            STUCK_T0[] = -1.0
        end
    end

    if WRECKED[]
        cfx = 0.0; cfy = 0.0; cmz = 0.0
        BND_FX[] = 0.0; BND_FY[] = 0.0; BND_MZ[] = 0.0
    end

    wfx = 0.0; wfy = 0.0
    if WRECKED[]
        if haskey(ENV, "JM_WRECK_DAMP_OLD")
            wfx = -617.0 * WRECK_DAMP * cs.v
        else
            cθw, sθw = cos(cs.θ), sin(cs.θ)
            vbx =  WVX[]*cθw + WVZ[]*sθw
            vby = -WVX[]*sθw + WVZ[]*cθw
            wfx = -617.0 * WRECK_DAMP * vbx
            wfy = -617.0 * WRECK_DAMP * vby
        end
    end
    if GRASS_N[] > 0 && !WRECKED[]
        cθg2, sθg2 = cos(cs.θ), sin(cs.θ)
        vbx = WVX[]*cθg2 + WVZ[]*sθg2; vby = -WVX[]*sθg2 + WVZ[]*cθg2
        kd = GRASS_N[]/4 * DriveRT3D.car_mass() * 9.80665 * GRASS_DRAG_K
        wfx -= kd*vbx; wfy -= kd*vby
        if GRASS_DIAG > 0 && cs.t >= GRASS_DIAG_T[] + GRASS_DIAG
            GRASS_DIAG_T[] = cs.t
            println("  [grassdrag] t ", round(cs.t; digits = 1), " v ", round(hypot(vbx, vby); digits = 1), " wheels ", GRASS_N[],
                    " drag ", round(kd*hypot(vbx, vby)/(DriveRT3D.car_mass()*9.80665); digits = 3), " g")
        end
    end
    if WIND_V != 0 && !WRECKED[]
        cθa, sθa = cos(cs.θ), sin(cs.θ)
        wx_, wz_ = -WIND_V*cos(WIND_DIR), -WIND_V*sin(WIND_DIR)
        ub = WVX[]*cθa + WVZ[]*sθa; ua = (WVX[] - wx_)*cθa + (WVZ[] - wz_)*sθa
        q = 0.5*DriveRT3D.air_density(TRACKSEL)*DriveRT3D.CDA_IBT
        fw = -q*(ua*abs(ua) - ub*abs(ub)); wfx += fw
        if WIND_DIAG > 0 && cs.t >= WIND_DIAG_T[] + WIND_DIAG
            WIND_DIAG_T[] = cs.t
            println("  [wind] t ", round(cs.t; digits = 1), " v ", round(ub; digits = 2), " air ", round(ua; digits = 2), " force ", round(fw; digits = 1), " N")
        end
    end
    DriveRT3D.extforce3d!(cs; Fx = cfx + BND_FX[] + wfx, Fy = cfy + BND_FY[] + wfy, Mz = cmz + BND_MZ[],
                          CdA_scale = PLAYER_CDA[])
    if OFFTRACK_PROBE[] > 0
        OFFTRACK_PROBE[] -= 1
        println("  [offtrack] t=", round(cs.t, digits=3), " v=", round(cs.v*3.6, digits=1), " x=", round(cs.x, digits=2),
                " z=", round(cs.z, digits=2), " y=", round(cs.y, digits=3), " heave=", round(cs.heave, digits=3),
                " ontrack=", ONTRACK[], " wrecked=", WRECKED[], " bndpk=", round(BND_PK[], digits=0),
                " bndF=(", round(BND_FX[], digits=0), ",", round(BND_FY[], digits=0), ") wfx=", round(wfx, digits=0),
                " cF=(", round(cfx, digits=0), ",", round(cfy, digits=0), ") off=", round(OFFDIST[], digits=1),
                " nl=", round(BND_NL[], digits=2))
        OFFTRACK_PROBE[] == 0 && haskey(ENV, "JM_OFFTRACK_EXIT") && (flush(stdout); exit(0))
    end

    if WRECKED[]
        if !WRECK_FROZEN[] && abs(cs.v) < 2.0
            WRECK_FROZEN[] = true
            WRECK_PX[] = cs.x; WRECK_PZ[] = cs.z; WRECK_PT[] = cs.θ
            println("  [WRECK] car came to rest"); flush(stdout)
            WRECK_PH[] = Float64(groundz_phys(cs.x, cs.z))
        end
        if WRECK_FROZEN[]

            WRECK_PIN && CAR3D ? DriveRT3D.pin3d!(cs, WRECK_PX[], WRECK_PZ[], WRECK_PT[], WRECK_PH[]) :
                                 DriveRT3D.place3d!(cs, WRECK_PX[], WRECK_PZ[], WRECK_PT[]; v=0.0)
        end
    end

    μFL = μFR = μRL = μRR = 1.0
    if !SKIDPAD && !rst
        hc = JRTrack.hat(TRKSURF, cs.x, cs.z)
        if hc.found && abs(hc.lateral) > ROAD_HALFW - 1.2
            cθg = cos(cs.θ); sθg = sin(cs.θ)
            cx_ = Float64(cs.x); cz_ = Float64(cs.z)
            μFL = wheel_mu(cx_, cz_, cθg, sθg,  1.314,  0.75); μFR = wheel_mu(cx_, cz_, cθg, sθg,  1.314, -0.75)
            μRL = wheel_mu(cx_, cz_, cθg, sθg, -1.096,  0.75); μRR = wheel_mu(cx_, cz_, cθg, sθg, -1.096, -0.75)
        end
    end
    GRASS_N[] = count(<(1.0), (μFL, μFR, μRL, μRR))

    if WET_A != 1.0
        _w = wet_mu(cs.v); μFL *= _w; μFR *= _w; μRL *= _w; μRR *= _w
    end

    rst || tt_step!(cs.tc, Float64(cs.v), Float64(dt), WETNESS)
    μFL *= TT_MU[1]; μFR *= TT_MU[2]; μRL *= TT_MU[3]; μRR *= TT_MU[4]
    tt_diag(Float64(cs.t))
    rst || tt_rest_step!(Float64(cs.v), Float64(cs.t), Float64(dt))
    DriveRT3D.wheelmu3d!(cs, μFL*DriveRT3D.damage_mu(1), μFR*DriveRT3D.damage_mu(2),
                             μRL*DriveRT3D.damage_mu(3), μRR*DriveRT3D.damage_mu(4))
    (cfx, cfy, cpk, ffb_jolt)
end

"""The shared world draw (main pass and E64 mirror pass): track, objects, billboards, AI and remote cars. LOOPSPLIT-1
cycle 3 S3: the per-frame `drawworld` closure's body, moved unchanged; the loop's closure now only forwards to it."""
function drawworld_impl!(vp_, eye_, flip::Bool, cs, AICHASSIS, _tp_d, aiBody, aiWheel, ai_poses, spin)
    (HORIZON_RING === nothing || LAYOFF_HZ) || Render.draw_horizon(prog, HORIZON_RING, vp_, eye_; tint=GRADE.ringtint)

    glUniform1i(Render.uloc(prog,"uBackFlip"), BACKFLIP_MODE)
    glUniform1i(Render.uloc(prog,"uMirrorPass"), flip ? 1 : 0)
    secfrom = (@isdefined SEC_FROM) ? SEC_FROM : typemax(Int)
    PROF_DEPTH[] += time() - _tp_d; _tp_t = time()
    glUniform1f(Render.uloc(prog,"uMaxDist"), TRACK_MAXD)
    _clap = _LW === nothing ? 0.0 : lapnear(Float64(eye_[1]), -Float64(eye_[3]))
    _late = false
    for ti in (length(TRACK_ORDER) == length(trackItems) ? TRACK_ORDER : eachindex(trackItems))
        it = trackItems[ti]
        if GROOVE_LATE && !_late && TRACK_GROOVE[ti]
            _late = true; glDepthMask(GL_FALSE); glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE)
        end

        if (@isdefined ROADTESS) && ROADTESS && ti <= length(TRACKMAIN) && TRACK_ASPGROOVE[ti]
            continue
        end

        if ti == secfrom && SEC_TWOSIDED; glDisable(GL_CULL_FACE)
        elseif ti == secfrom; glEnable(GL_CULL_FACE); glCullFace(xor(OBJ_FF_CW, flip) ? GL_FRONT : GL_BACK)
        elseif ti < secfrom && (@isdefined TRACK_RAILCULL) && ti <= length(TRACK_RAILCULL)
            if TRACK_RAILCULL[ti]; glEnable(GL_CULL_FACE); glCullFace(xor(OBJ_FF_CW, flip) ? GL_FRONT : GL_BACK)
            elseif TRACK_BOARDCULL[ti]; glEnable(GL_CULL_FACE); glCullFace(flip ? GL_FRONT : GL_BACK)
            else; glDisable(GL_CULL_FACE); end
        end
        TRACKCULL && !infrustum(vp_, TRACKBOUND[ti][1], TRACKBOUND[ti][2]) && continue
        if MONZA
            cat = TRACKCAT[ti]
            b, a = cat === :road ? (MZ_ROAD_B, MZ_ROAD_A) : cat === :dark ? (MZ_DARK_B, MZ_DARK_A) :
                   cat === :bank ? (MZ_BANK_B, MZ_BANK_A) : (MZ_OTHER_B, MZ_OTHER_A)
            gg = ti <= length(TRACKGAIN) ? TRACKGAIN[ti] : 1f0
            if _LW !== nothing && ti <= length(TRACKLAPOFF) && length(TRACKLAPOFF[ti]) > LAPNB + 1
                for (f0, n0) in bandranges!(BANDR, TRACKLAPOFF[ti], _clap, true)
                    n0 > 0 && Render.draw(prog, it, vp_, Render.IDENT; wet=(ti <= length(TRACKWET) ? TRACKWET[ti] : 0f0), bright=b*gg, ambfill=a*gg, first=f0, count=n0)
                end
            else
                Render.draw(prog, it, vp_, Render.IDENT; wet=(ti <= length(TRACKWET) ? TRACKWET[ti] : 0f0), bright=b*gg, ambfill=a*gg)
            end
        else

            gg = ti <= length(TRACKGAIN) ? TRACKGAIN[ti] : 1f0
            if _LW !== nothing && ti <= length(TRACKLAPOFF) && length(TRACKLAPOFF[ti]) > LAPNB + 1
                for (f0, n0) in bandranges!(BANDR, TRACKLAPOFF[ti], _clap)
                    n0 > 0 && Render.draw(prog, it, vp_, Render.IDENT; wet=(ti <= length(TRACKWET) ? TRACKWET[ti] : 0f0), bright=TRACK_BRIGHT*gg*TERR_B, ambfill=TRACK_AMB*gg*TERR_A, first=f0, count=n0)
                end
            elseif _LW !== nothing && ti <= length(TRACKLAPOFF)
                for (f0, n0) in lapranges(TRACKLAPOFF[ti], _clap)
                    n0 > 0 && Render.draw(prog, it, vp_, Render.IDENT; wet=(ti <= length(TRACKWET) ? TRACKWET[ti] : 0f0), bright=TRACK_BRIGHT*gg*TERR_B, ambfill=TRACK_AMB*gg*TERR_A, first=f0, count=n0)
                end
            else
                Render.draw(prog, it, vp_, Render.IDENT; wet=(ti <= length(TRACKWET) ? TRACKWET[ti] : 0f0), bright=TRACK_BRIGHT*gg*TERR_B, ambfill=TRACK_AMB*gg*TERR_A)
            end
        end
    end
    _late && (glDepthMask(GL_TRUE); glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE))
    glUniform1f(Render.uloc(prog,"uMaxDist"), 0f0)
    (@isdefined SEC_FROM) && length(trackItems) >= SEC_FROM && glDisable(GL_CULL_FACE)
    if OBJ_CULLFACE

        glEnable(GL_CULL_FACE); glCullFace(xor(OBJ_FF_CW, flip) ? GL_FRONT : GL_BACK)
        glUniform1i(Render.uloc(prog,"uBackFlip"), 0)
    end
    PROF_TRACK[] += time() - _tp_t; _tp_o = time()

    (function (_OBJS, _BND)
    _cseg = classicvis_seg(eye_[1], eye_[3])
    _olap = _LW === nothing ? 0.0 : lapnear(Float64(eye_[1]), -Float64(eye_[3]))
    if _LW !== nothing && length(OBJLAP) != length(_OBJS)
        resize!(OBJLAP, length(_OBJS)); resize!(OBJLAP2, length(_OBJS))
        for (k, o) in enumerate(_OBJS)
            if isbackdrop(lowercase(String(o[5])))
                OBJLAP[k] = NaN; OBJLAP2[k] = NaN
            else
                sl, dsq = lapnear_d(Float64(o[4][1]), -Float64(o[4][3]))
                OBJLAP[k] = sl; OBJLAP2[k] = lapnear2(Float64(o[4][1]), -Float64(o[4][3]), sl, dsq)
            end
        end
    end
    for (oi,(items,mat,grz,opos,onm)) in enumerate(_OBJS)
        LAYOFF_OBJ && continue
        _ocl = oi <= length(OBJCLIP) ? OBJCLIP[oi] : nothing
        (_ocl === nothing && _cseg > 0 && oi <= length(OBJVIS) && OBJVIS[oi] !== nothing && !OBJVIS[oi][_cseg]) && continue
        (OBJ_SCENEDUP[oi] && _ocl === nothing && !(oi <= length(OBJVIS) && OBJVIS[oi] !== nothing)) && continue
        bc, br = oi <= length(_BND) ? _BND[oi] : ((opos[1], opos[2], opos[3]), 80f0)
        (_LW !== nothing && oi <= length(OBJLAP) && !isnan(OBJLAP[oi]) && !isnan(_olap) && !lapwin_in(OBJLAP[oi], _olap) &&
         !(oi <= length(OBJLAP2) && !isnan(OBJLAP2[oi]) && lapwin_in(OBJLAP2[oi], _olap))) && continue
        max(sqrt((eye_[1]-bc[1])^2+(eye_[2]-bc[2])^2+(eye_[3]-bc[3])^2) - br, 0f0)^2 > (flip ? MIR_OBJ_CULL2 : OBJ_CULL2) && continue
        FRUSTUM_CULL && !infrustum(vp_, bc, br) && continue

        br < (flip ? MINPIX_K_MIR : MINPIX_K) * sqrt((eye_[1]-bc[1])^2+(eye_[2]-bc[2])^2+(eye_[3]-bc[3])^2) && continue
        ob, oa = OBJ_BRIGHT, OBJ_AMB

        _ovg = OBJ_VEG_GRADE ? get(OBJVEG, onm, nothing) : nothing
        if MONZA
            g = monza_obj_grade(onm)
            g === :road && ((ob, oa) = (Float32(MZ_ROAD_B), Float32(MZ_ROAD_A))); g === :bank && ((ob, oa) = (Float32(MZ_BANK_B), Float32(MZ_BANK_A)))
        end
        otint = is_crowd_obj(onm) ? CROWD_TINT : (1f0,1f0,1f0)

        _twin = OBJ_CULLFACE ? nothing : get(TWINBOARD, onm, nothing)
        _sbcull = (STARTBOX_CULL && !OBJ_CULLFACE && onm == "startbox") || _twin !== nothing
        if _sbcull
            glEnable(GL_CULL_FACE); glCullFace(_twin === nothing ? (xor(OBJ_FF_CW, flip) ? GL_BACK : GL_FRONT) : (xor(_twin, flip) ? GL_BACK : GL_FRONT))
            glUniform1i(Render.uloc(prog,"uBackFlip"), 0)
        end
        _obd = BACKDROP_ON ? get(OBJBACKDROP, onm, nothing) : nothing
        _bdfar = _obd !== nothing && hypot(eye_[1] - opos[1], eye_[3] - opos[3]) > backdrop_r(onm)
        for _cpl in (_ocl === nothing ? (nothing,) : _ocl)
        _cpl === nothing || Render.clip_planes!(prog, _cpl)
        for (_ii, it) in enumerate(items)
            _fol = _ovg !== nothing && _ii <= length(_ovg) && _ovg[_ii]
            _bd = _obd !== nothing && _ii <= length(_obd) && _obd[_ii]
            (_bd && _bdfar) && continue
            Render.draw(prog, it, vp_, mat; bright = _fol ? TRACK_BRIGHT*VEG_GAIN*TERR_B : (OBJ_FLAT[1] > 0f0 ? 1f0 : ob),
                        ambfill = _fol || OBJ_FLAT[1] > 0f0 ? (_fol ? TRACK_AMB*VEG_GAIN*TERR_A : 0f0) : oa, graze=grz || _bd, tint=otint,
                        flat = _fol ? (0f0, 0f0) : OBJ_FLAT)
        end
        end
        _ocl === nothing || Render.clip_planes!(prog, nothing)
        if _sbcull; glDisable(GL_CULL_FACE); glUniform1i(Render.uloc(prog,"uBackFlip"), BACKFLIP_MODE); end
    end
    end)(OBJECTS, OBJBOUND)
    PROF_OBJ[] += time() - _tp_o; _tp_b = time()

    _blap = (_LW === nothing || !LAPWIN_SPRITES) ? NaN : lapnear(Float64(eye_[1]), -Float64(eye_[3]))
    if !isnan(_blap) && length(BBLAP) != length(BILLBOARDS) + length(STATICTREES)
        empty!(BBLAP); empty!(BBLAP2)
        for (_, pos) in Iterators.flatten((((x[1], x[2]) for x in STATICTREES), ((x[1], x[2]) for x in BILLBOARDS)))
            sl, dsq = lapnear_d(Float64(pos[1]), -Float64(pos[3])); push!(BBLAP, sl); push!(BBLAP2, lapnear2(Float64(pos[1]), -Float64(pos[3]), sl, dsq))
        end
    end
    _bbin(k) = isnan(_blap) || k > length(BBLAP) || isnan(BBLAP[k]) || lapwin_in(BBLAP[k], _blap) || (!isnan(BBLAP2[k]) && lapwin_in(BBLAP2[k], _blap))
    for (_k, (it,pos,w,h,yaw)) in enumerate(STATICTREES)
        _bbin(_k) || continue
        (eye_[1]-pos[1])^2+(eye_[2]-pos[2])^2+(eye_[3]-pos[3])^2 > BB_CULL2 && continue
        FRUSTUM_CULL && !infrustum(vp_, pos, max(w, h) + 5f0) && continue
        Render.draw(prog, it, vp_, Render.translate(pos[1],pos[2],pos[3])*Render.roty(yaw)*Render.scalexyz(w,h,1f0); bright=1.3, ambfill=0.8, graze=true, unlit=!BB_LIT)
    end
    OBJ_CULLFACE && glDisable(GL_CULL_FACE)
    glUniform1i(Render.uloc(prog,"uBackFlip"), 0)
    (function (_BBS, _nst, _bbin)
    for (_k, (it,pos,w,h)) in enumerate(_BBS)
        LAYOFF_BB && continue
        _bbin(_nst + _k) || continue
        _d2 = (eye_[1]-pos[1])^2+(eye_[2]-pos[2])^2+(eye_[3]-pos[3])^2
        _d2 > (flip ? MIR_BB_CULL2 : BB_CULL2) && continue
        max(w, h) < 2f0 * (flip ? MINPIX_K_MIR : MINPIX_K) * sqrt(_d2) && continue
        FRUSTUM_CULL && !infrustum(vp_, pos, max(w, h) + 5f0) && continue
        Render.draw(prog, it, vp_, Render.billboard_model(pos,w,h,eye_); bright=BB_BRIGHT, ambfill=BB_AMB, unlit=!BB_LIT)
    end
    end)(BILLBOARDS, length(STATICTREES), _bbin)
    PROF_BB[] += time() - _tp_b; _tp_c = time()

    draw_ai_main!(prog, vp_, ai_poses, AICHASSIS, spin, AI_NOWHEELS)
    spray_draw!(prog, vp_, eye_, cs, ai_poses)
    PROF_CARS[] += time() - _tp_c

    if !isempty(NETPOSES[]) && !isempty(AICARMODELS)
        for (nid, p) in NETPOSES[]
            ct = get(NETCARS[], nid, 0xff)
            cm = ct < length(ALL67) ? get(NET_BY_TYPE, ALL67[ct + 1][1], AICARMODELS[NetPlay.chassis_slot(nid, length(AICARMODELS))]) :
                                      AICARMODELS[NetPlay.chassis_slot(nid, length(AICARMODELS))]
            let _bm = aiBody(p, cm); for it in cm.body; Render.draw(prog, it, vp_, _bm; bright=AI_BRIGHT, spec=0.10, ambfill=AI_AMB); end; end
            for (wx,wz,_,r,nm) in cm.wheelspec, it in cm.wheels[nm]; Render.draw(prog, it, vp_, aiWheel(p,cm,wx,wz,r)); end
        end
    end
end

"""Cockpit view: the gloved hands on the wheel and the sleeves laid from glove to shoulder (ARMS2), one frame.
LOOPSPLIT-1 cycle 3 S4: moved unchanged."""
function hands_draw!(cs, inp, vp, bodyModel, swModel)
    _hcull = glIsEnabled(GL_CULL_FACE) == GL_TRUE

    _hoff = (0f0, 0f0, 0f0)
    if SHIFT_HAND && SHIFT_KNOB !== nothing && !isempty(shiftItems) && length(handLR) == 2
        _u = Float32(shift_hand_u(cs.gear))
        if _u > 0f0
            _swr = Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(HSIGN*inp.steer*2.5 + SW_ROT)) * Render.translate(-SWCENTER[1], -SWCENTER[2], -SWCENTER[3])
            _g = _swr * gripmat(SHIFT_HAND_K == 1 ? 1 : -1) * Float32[FIST_C[SHIFT_HAND_K]..., 1f0]
            _k = SHIFT_LASTM[] * Float32[SHIFT_KNOB..., 1f0]
            _hoff = (_u*(_k[1] + SHIFT_HAND_UP[1] - _g[1]), _u*(_k[2] + SHIFT_HAND_UP[2] - _g[2]), _u*(_k[3] + SHIFT_HAND_UP[3] - _g[3]))
        end
    end
    HAND_CULL || glDisable(GL_CULL_FACE)
    if ARMS

        if ARMS2 && length(armLR) == 2 && SLEEVE_ENDS[1] !== nothing && SLEEVE_ENDS[2] !== nothing
            swRel = Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(HSIGN*inp.steer*2.5 + SW_ROT)) * Render.translate(-SWCENTER[1], -SWCENTER[2], -SWCENTER[3])
            for (k, sgn) in ((1, 1), (2, -1))
                W, E, nf = SLEEVE_ENDS[k]
                Wt = (CUFF && CUFF_RAW[k] !== nothing) ? CUFF_RAW[k] : W
                h = swRel * gripmat(sgn) * Float32[Wt[1], Wt[2], Wt[3], 1f0]
                k == SHIFT_HAND_K && (h = Float32[h[1] + _hoff[1], h[2] + _hoff[2], h[3] + _hoff[3], 1f0])
                S = (SHOULDER[1], SHOULDER[2], sgn*SHOULDER[3])

                if ARM_LOCK_S !== nothing
                    _u = clamp((abs(HSIGN*inp.steer*2.5) - deg2rad(ARM_LOCK_DEG[1])) / deg2rad(ARM_LOCK_DEG[2] - ARM_LOCK_DEG[1]), 0.0, 1.0)
                    S = (S[1] + _u*(ARM_LOCK_S[1] - S[1]), S[2] + _u*(ARM_LOCK_S[2] - S[2]), S[3] + _u*(sgn*ARM_LOCK_S[3] - S[3]))
                end
                if SLEEVE_HOLD && SLEEVE_NREF[k] === nothing
                    sw0 = Render.translate(SWCENTER) * Render.rotaxis(SWAXIS, Float32(SW_ROT)) * Render.translate(-SWCENTER[1], -SWCENTER[2], -SWCENTER[3])
                    h0 = sw0 * gripmat(sgn) * Float32[Wt[1], Wt[2], Wt[3], 1f0]
                    sleeve_mat(W, E, (h0[1], h0[2], h0[3]), S, nf); SLEEVE_NREF[k] = SLEEVE_LASTR[] * collect(nf)
                end

                _ul = clamp((abs(HSIGN*inp.steer*2.5) - deg2rad(ARM_LOCK_DEG[1])) / deg2rad(ARM_LOCK_DEG[2] - ARM_LOCK_DEG[1]), 0.0, 1.0)
                _into = haskey(ENV, "JM_ARM_INTO") ? parse(Float64, ENV["JM_ARM_INTO"]) :
                        haskey(ENV, "JM_CUFF_INTO") && CUFF ? parse(Float64, ENV["JM_CUFF_INTO"]) :
                        (CUFF && CUFF_RAW[k] !== nothing) ? CUFF_INTO[1] + clamp((abs(HSIGN*inp.steer*2.5) - deg2rad(CUFF_LOCK_DEG[1])) /
                                                              deg2rad(CUFF_LOCK_DEG[2] - CUFF_LOCK_DEG[1]), 0.0, 1.0)*(CUFF_INTO[2] - CUFF_INTO[1]) :
                        ARM_INTO_STRAIGHT + _ul*(ARM_INTO - ARM_INTO_STRAIGHT)
                _d = (h[1] - S[1], h[2] - S[2], h[3] - S[3]); _l = sqrt(_d[1]^2 + _d[2]^2 + _d[3]^2)
                h = _l > 1e-6 ? (h[1] + _into*_d[1]/_l, h[2] + _into*_d[2]/_l, h[3] + _into*_d[3]/_l) : h

                _tgt = SLEEVE_HOLD ? (ARM_ROLLWHEEL && SLEEVE_NREF[k] !== nothing ?
                       (Render.rotaxis(SWAXIS, Float32(HSIGN*inp.steer*2.5)) * Float32[SLEEVE_NREF[k]..., 0f0])[1:3] : SLEEVE_NREF[k]) : nothing
                Render.draw(prog, armLR[k], vp, bodyModel * sleeve_mat(W, E, (h[1], h[2], h[3]), S, nf; target = _tgt);
                            bright=1.35, spec=0.05, ambfill=0.85)
            end
        else
            _amod = ARMS_ON_WHEEL ? swModel : bodyModel*ARMFIX
            for it in armItems;  Render.draw(prog, it, vp, _amod; bright=1.15, spec=0.05, ambfill=0.60); end
        end
    end
    if length(handLR) == 2
        _hm(k) = (k == SHIFT_HAND_K && _hoff != (0f0, 0f0, 0f0)) ? bodyModel * Render.translate(_hoff) * inv(bodyModel) : Render.ident()
        Render.draw(prog, handLR[1], vp, _hm(1)*swModel*gripmat(+1)*fistrot(1); bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3])
        Render.draw(prog, handLR[2], vp, _hm(2)*swModel*gripmat(-1)*fistrot(2); bright=GLOVE_LIGHT[1], spec=GLOVE_LIGHT[2], ambfill=GLOVE_LIGHT[3])
    else
        for it in handItems; Render.draw(prog, it, vp, swModel; bright=1.15, spec=0.05, ambfill=0.60); end
    end
    (!HAND_CULL && _hcull) && glEnable(GL_CULL_FACE)
    nothing
end

"""JM_RAM_AI test hook: place the player to ram AI car k, then brake as a driver would, one frame. LOOPSPLIT-1 cycle 3 S6: moved unchanged."""
function ram_ai_step!(cs, ai_poses, ddt, AICARS, PLAYER_G, groundz)
    isnan(RAM_STATE[].t0) && (RAM_STATE[] = (t0 = cs.t, done = false))
    if cs.t - RAM_STATE[].t0 >= RAM_AI[4]
        k = Int(RAM_AI[1]); q = ai_poses[k]; qθ = q[4]; vk = AICARS[k].v

        ahead = length(RAM_AI) >= 5 && RAM_AI[5] != 0 ? RAM_AI[5] : -4.7

        SIDE = length(RAM_AI) >= 6 && RAM_AI[6] == 1
        SIDE && (ahead = 0.0)
        rx = q[1] + ahead*cos(qθ) - RAM_AI[3]*sin(qθ); rz = q[3] + ahead*sin(qθ) + RAM_AI[3]*cos(qθ)
        PLAYER_G[] = NaN; CLASSICW_ON && classicw_teleported!()
        DriveRT3D.place3d!(cs, rx, rz, qθ; v = (ahead > 0 && !SIDE) ? 0.0 : vk + RAM_AI[2])
        (ahead > 0 && !SIDE) && (WRECKED[] = true)
        h = groundz(rx, rz; acquire = true); isfinite(h) && (cs.zref = Float64(h); cs.y = cs.zref)
        RAM_STATE[] = (t0 = RAM_STATE[].t0, done = true)
        println("  [ram] t=", round(cs.t, digits = 2), SIDE ? " player placed ALONGSIDE AI " : ahead > 0 ? " a wreck placed $(ahead) m ahead of AI " : " player placed $(-ahead) m behind AI ", k, " (", round(vk*3.6), " km/h), lat ",
                RAM_AI[3], " m, closing ", RAM_AI[2], " m/s"); flush(stdout)
    end
    nothing
end

"""CARCOLLIDE-2 S4: each AI car knocked loose steps as its pool Car3D (tumble, locked brakes, recovery, rejoin), one frame. LOOPSPLIT-1 cycle 3 S6: moved unchanged."""
function loose_cars_step!(cs, ai_poses, ddt, AICARS, AILINE, CLASSICAI_T, PLAYER_G, aibankP, frames, classicai_field!, groundz, groundz_ai)
    LOOSE_DROP !== nothing && loose_drop_step!(cs, AICARS, AILINE, groundz, frames)

    if RAM_SOLO && !RAM_SOLO_DONE[]
        RAM_SOLO_DONE[] = true; PLAYER_G[] = NaN; CLASSICW_ON && classicw_teleported!()
        DriveRT3D.place3d!(cs, cs.x - 30*sin(cs.θ), cs.z + 30*cos(cs.θ), cs.θ; v = 0.0)
    end
    for (k, sl) in collect(LOOSE_OF)
        if k > length(ai_poses) || k > length(AICARS)
            delete!(LOOSE_OF, k); continue
        end
        pc = LOOSE_CARS[sl]

        (lrs, _) = RaceAI.project(AILINE, pc.x, pc.z; hint = AICARS[k].s)
        lherr = RaceAI.wrapπ(pc.θ - RaceAI.pose_at(AILINE, lrs, 0.0)[4])
        flying = LOOSE_BODY[sl] !== nothing
        if !flying && !LOOSE_REC[sl] && ((pc.v < 4.0 && abs(DriveRT3D.yawrate3d(pc)) < 0.5) || cs.t - LOOSE_T0[sl] > 4.0)
            LOOSE_REC[sl] = true; LOOSE_T0[sl] = cs.t
        end
        if flying
            ai_poses[k] = loose_body_step!(k, sl, pc, ddt > 1e-4 ? Float64(ddt) : 1/60, groundz)
        else
            if LOOSE_REC[sl]
                (lthr, lbrk, lst) = RaceAI.controller(AILINE, lrs, 0.0, 0.0, 6.0, pc.x, pc.z, pc.θ, pc.v, DriveRT3D.yawrate3d(pc))
                DriveRT3D.step_car3d!(pc, lthr, lbrk, lst, ddt; manual = false, groundz = groundz_ai)
            else
                DriveRT3D.step_car3d!(pc, 0.0, LOOSE_BRAKE, 0.0, ddt; manual = true, groundz = groundz_ai)
            end
            LOOSE_DIAG && (frames % 6 == 0) && println("   [loosediag] t ", round(cs.t, digits = 2), " v ", round(pc.v, digits = 2),
                " r ", round(DriveRT3D.yawrate3d(pc), digits = 2), " θ ", round(pc.θ, digits = 2), " grounded ", pc.grounded,
                " rh ", round.(pc.rh, digits = 3), " heave ", round(pc.heave, digits = 3), " ddt ", round(ddt, digits = 4), " rec ", LOOSE_REC[sl])
            bk = aibankP(pc)
            gy = isfinite(pc.y) ? pc.y : groundz(pc.x, pc.z)
            ai_poses[k] = (pc.x, gy, pc.z, pc.θ, bk[1], bk[2])

            if AIRBODY && CRASHBODY && maximum(pc.rh) > AIRBODY_RH
                loose_body_start!(k, sl, pc, ai_poses[k][6])
                flying = true
            end
        end
        done = (!flying && LOOSE_REC[sl] && ((abs(lherr) < 0.35 && pc.v > 2.0) || cs.t - LOOSE_T0[sl] > 10.0)) || !isfinite(pc.x)
        if CLASSICAI_T !== nothing
            gc = classicai_field!().cars[k]
            (gs, gd, _) = CLASSICAI.tlocate(CLASSICAI_T, pc.x, pc.z, 0)
            (lo, hi) = CLASSICAI.corridor(CLASSICAI_T, gs)
            gc.s = gs; gc.d = gd; gc.v = pc.v; gc.dv = 0.0

            (done && !(lo <= gd <= hi)) && (CLASSICAI.SOFTEDGE[k] = 36*20; CLASSICAI.join!(CLASSICAI_T, gc, clamp(gd, lo + 0.5, hi - 0.5); why = :rejoin))
            gc.yaw = pc.θ; gc.yawr = loose_yawrate(sl); gc.pose = ai_poses[k]; gc.prev = gc.pose
            (rs, rl) = RaceAI.project(AILINE, pc.x, pc.z; hint = AICARS[k].s); AICARS[k].s = rs; AICARS[k].lane = rl; AICARS[k].v = pc.v
        else
            (rs, rl) = RaceAI.project(AILINE, pc.x, pc.z; hint = AICARS[k].s)
            AICARS[k].s = rs; AICARS[k].lane = clamp(rl, -RaceAI.LANE_MAX, RaceAI.LANE_MAX); AICARS[k].v = pc.v
            done && (AICARS[k].spin = RaceAI.wrapπ(pc.θ - RaceAI.pose_at(AILINE, rs, 0.0)[4]))
        end
        if done
            delete!(LOOSE_OF, k)
            println("  [loose] t=", round(cs.t, digits = 2), " AI ", k, " rejoins at ", round(pc.v*3.6), " km/h after ", LOOSE_HITS[sl], " more hits, heading off the line ",
                    round(rad2deg(RaceAI.wrapπ(pc.θ - RaceAI.pose_at(AILINE, RaceAI.project(AILINE, pc.x, pc.z)[1], 0.0)[4]))), " deg"); flush(stdout)
        end
    end
    nothing
end

"""AISINK-1 test hook, smoke only: JM_LOOSE_DROP="t:s:lat:h" -- at race time t AI 1 becomes a loose rigid body h m over the
ground at (s, lat) on its rail, at rest; every 0.5 s its pose height against the ground under it (TERRAIN, else the placed
objects' grass) is printed, so a car resting under (or over) the drawn ground shows as a gap."""
function loose_drop_step!(cs, AICARS, AILINE, groundz, frames)
    (t, s, lat, h) = LOOSE_DROP
    if !LOOSE_DROP_DONE[] && cs.t >= t && !isempty(AICARS) && !isempty(LOOSE_CARS) && !haskey(LOOSE_OF, 1)
        LOOSE_DROP_DONE[] = true
        p = RaceAI.pose_at(AILINE, s, lat)
        r = JRTrack.hat3d(TERRAIN, p[1], p[3]; ref = Inf); og = objground_at(p[1], p[3])
        g = r[3] ? Float64(r[1]) : isfinite(og) ? og : Float64(groundz(p[1], p[3]))
        pc = LOOSE_CARS[1]
        DriveRT3D.pin3d!(pc, p[1], p[3], p[4], g + h); pc.s_vel(pc.integ, [0.0, 0.0]); pc.v = 0.0
        LOOSE_OF[1] = 1; LOOSE_T0[1] = cs.t; LOOSE_REC[1] = false; LOOSE_HITS[1] = 0
        loose_body_start!(1, 1, pc, 0.0)
        println("  [loosedrop] t=", round(cs.t, digits = 2), " AI 1 dropped at s ", s, " lat ", lat, " (", round(p[1], digits = 1), ", ",
                round(p[3], digits = 1), "), ground ", round(g, digits = 2), r[3] ? " (TERRAIN)" : isfinite(og) ? " (object grass)" : " (none: groundz)")
    end
    if LOOSE_DROP_DONE[] && frames % 30 == 0 && haskey(LOOSE_OF, 1)
        pc = LOOSE_CARS[1]; b = LOOSE_BODY[1]
        (x, y, z) = b === nothing ? (pc.x, pc.y, pc.z) : (o = b.p - b.R * WreckBody.V3(0.0, b.g.hcg, 0.0); (o[1], o[2], -o[3]))
        r = JRTrack.hat3d(TERRAIN, x, z; ref = Inf); og = objground_at(x, z); dg = r[3] ? Float64(r[1]) : og
        println("  [loosedrop] t=", round(cs.t, digits = 2), b === nothing ? " car " : " body ", "y ", round(y, digits = 3), " drawn ground ",
                round(dg, digits = 3), " gap ", round(y - dg, digits = 3), " v ", round(pc.v, digits = 2))
    end
    nothing
end

"""CARLAUNCH-2: loose AI car k (pool slot sl, its Car3D `pc`, drawn roll `roll`) had a corner thrown up -- from here it is
a free rigid body, taking the driving model's pose, velocity and roll, pitch, heave and yaw rates."""
function loose_body_start!(k::Int, sl::Int, pc, roll::Float64)
    WB = WreckBody; V3 = WB.V3
    b = WB.Body(WB.Geometry(m = DriveRT3D.MASS[]))
    (pp, q, w) = attrates3d(pc)
    (vx, vz) = DriveRT3D.world_velocity(pc)
    WB.start!(b, pc.x, pc.z, pc.y, pc.θ, pc.pitch, roll, V3(vx, w, -vz), V3(pp, Float64(DriveRT3D.yawrate3d(pc)), q))
    b.brake = LOOSE_BRAKE
    LOOSE_BODY[sl] = b; LOOSE_BPLANE[sl] = WB.Plane(pc.x, -pc.z, pc.y, 0.0, 0.0)
    LOOSE_LAND[sl] = 0.0; LOOSE_DOWN[sl] = 0.0; LOOSE_PEAK[sl] = (0.0, 0.0); ctrace_start!("ai launch")
    println(ai_launch_msg(k, Float64(pc.v), pp, q, w)); flush(stdout)
    nothing
end

"""CARLAUNCH-2: a loose AI car's flight, one frame -- the rigid body steps on the terrain under each point (as the player's
does, CRASHBODY-1), and once it stands on its four tyres, upright and settled, for 0.15 s, it goes back onto its pool
Car3D at its speed, slip and yaw rate. At rest off its wheels it lies LOOSE_UPRIGHT s and is set back on them where it lies
(stopped). Returns the drawn pose (x, y, z, heading, pitch, roll)."""
function loose_body_step!(k::Int, sl::Int, pc, gdt::Float64, groundz::G; dry::Bool = false) where {G}
    WB = WreckBody; V3 = WB.V3
    b = dry ? WB.Body(WB.Geometry(m = DriveRT3D.MASS[])) : LOOSE_BODY[sl]
    dry && WB.start!(b, pc.x, pc.z, pc.y, pc.θ, 0.0, 0.0, V3(pc.v*cos(pc.θ), 0.0, -pc.v*sin(pc.θ)), V3(0.0, 0.0, 0.0))

    hq(X, Z) = (og = objground_at(X, -Z); isfinite(og) ? og : (y = groundz(X, -Z); y > -900 ? Float64(y) : NaN))
    pl = WB.fit_plane(hq, b.p[1], b.p[3])
    (isfinite(pl.h0) && isfinite(pl.gx) && isfinite(pl.gz)) ? (dry || (LOOSE_BPLANE[sl] = pl)) : (pl = LOOSE_BPLANE[sl])

    th(X, Z, yref) = SKIDPAD ? 0.0 : (r = JRTrack.hat3d(TERRAIN, X, -Z; ref = yref); r[3] ? Float64(r[1]) : objground_at(X, -Z, yref))
    ter = WB.Terrain(th, pl)
    WB.step!(b, gdt, ter; walls = dry ? LOOSE_WALLS : ai_body_walls!(LOOSE_WALLS, b))
    (nt, nh) = WB.touching(b, ter)
    (θ, pitch, roll) = WB.attitude(b)
    o = b.p - b.R * V3(0.0, b.g.hcg, 0.0)
    pose = (o[1], o[2], -o[3], θ, pitch, roll)
    dry && return pose
    pc.x = o[1]; pc.z = -o[3]; pc.y = o[2]; pc.θ = θ; pc.v = hypot(b.v[1], b.v[3]); pc.pitch = pitch; pc.roll = roll
    if ai_wall_tear!(k, b, pose) + ai_ground_tear!(k, b, pose) > 0
        AI_WRECK[k] = AIWreck(b, LOOSE_BPLANE[sl], WB.WallPlane[]); delete!(LOOSE_OF, k); LOOSE_BODY[sl] = nothing
        return pose
    end
    pk = LOOSE_PEAK[sl]
    LOOSE_PEAK[sl] = (abs(roll) > abs(pk[1]) ? roll : pk[1], abs(pitch) > abs(pk[2]) ? pitch : pk[2])
    n = WB.plane_n(pl); up = dot(b.R[:, 2], n)
    settled = nt == 4 && nh == 0 && up > cos(deg2rad(6.0)) && abs(b.ω[1]) < 0.6 && abs(b.ω[3]) < 0.6 && abs(dot(b.v, n)) < 0.6
    LOOSE_LAND[sl] = settled ? LOOSE_LAND[sl] + gdt : 0.0
    LOOSE_DOWN[sl] = (b.rest && !settled) ? LOOSE_DOWN[sl] + gdt : 0.0
    landed = LOOSE_LAND[sl] >= 0.15 || (b.rest && settled)
    (landed || LOOSE_DOWN[sl] >= LOOSE_UPRIGHT) || return pose

    wvx = landed ? b.v[1] : 0.0; wvz = landed ? -b.v[3] : 0.0
    u = wvx*cos(θ) + wvz*sin(θ); vl = -wvx*sin(θ) + wvz*cos(θ)
    r = landed ? (b.R * b.ω)[2] : 0.0
    og = objground_at(o[1], -o[3]); h = isfinite(og) ? og : groundz(o[1], -o[3])
    DriveRT3D.pin3d!(pc, o[1], -o[3], θ, h > -900 ? Float64(h) : o[2])
    pc.s_vel(pc.integ, [u, vl]); pc.v = hypot(u, vl)
    DriveRT3D._RSET3D[pc.sys][2](pc.integ, r)
    DriveRT3D.OrdinaryDiffEq.SciMLBase.u_modified!(pc.integ, true)
    LOOSE_BODY[sl] = nothing
    println(ai_landed_msg(k, landed, b.t, b.peak_h, Float64(pc.v), LOOSE_PEAK[sl])); flush(stdout)
    (pc.x, pc.y, pc.z, θ, 0.0, 0.0)
end

ai_launch_msg(k::Int, v::Float64, pp::Float64, q::Float64, w::Float64) = string("  [airbody] AI ", k, " thrown up: a free rigid body at ",
    round(v*3.6, digits = 0), " km/h: roll rate ", round(pp, digits = 2), " pitch rate ", round(q, digits = 2), " heave ", round(w, digits = 2),
    " -- until it stands on its wheels")
ai_landed_msg(k::Int, landed::Bool, t::Float64, h::Float64, v::Float64, pk::Tuple{Float64,Float64}) = string("  [airbody] AI ", k,
    landed ? " landed on its wheels after " : " lay off its wheels and was set back on them after ", round(t, digits = 2),
    " s (highest CG ", round(h, digits = 2), " m, roll to ", round(rad2deg(pk[1]), digits = 0), " deg, pitch to ",
    round(rad2deg(pk[2]), digits = 0), " deg): back on its driving model at ", round(v*3.6, digits = 0), " km/h")

"""The Classic AI field, one frame (AICLASSIC-2): step the field with the player's position, the off-road diagnostic, the race
bookkeeping (laps, order) in our frame, the cool-down. Returns the field's poses. LOOPSPLIT-1 cycle 4 S1: moved unchanged
(in the loop its `(ps, ...) =` wrote main()'s own `ps`)."""
function classicai_poses!(cs, ddt, ai_hit, AICARS, AILINE, CLASSICAI_PD, CLASSICAI_PH, CLASSICAI_SC, CLASSICAI_T, ent_name, classicai_field!, phase, race_done, race_go)

    f = classicai_field!()
    (ps, pdl, CLASSICAI_PH[]) = CLASSICAI.tlocate(CLASSICAI_T, cs.x, cs.z, CLASSICAI_PH[])
    pdv = ddt > 1e-4 ? (pdl - CLASSICAI_PD[][2])/ddt : 0.0
    abs(ps - CLASSICAI_PD[][1]) > 30 && (pdv = 0.0)
    CLASSICAI_PD[] = (ps, pdl)
    poses, hit = CLASSICAI.step!(f, ddt; player = (ps, pdl, clamp(pdv, -15.0, 15.0), cs.v), scale = CLASSICAI_SC,
                             vrel = isfinite(AI_REL) ? max(cs.v*AI_REL, 6.0) : Inf)
    ai_hit[] = hit
    if AI_OFFROAD_DIAG && race_go[]
        for gc in f.cars
            (wx, wz, _) = CLASSICAI.tworld(CLASSICAI_T, gc.s, gc.d); AI_OFF[2] += 1
            if !JRTrack.hat3d(ROADHAT, wx, wz; ref = Inf)[3]
                AI_OFF[1] += 1; b = floor(Int, gc.s/100); AI_OFFBIN[b] = get(AI_OFFBIN, b, 0) + 1
            end

            (lx, lz, _) = CLASSICAI.tworld(CLASSICAI_T, gc.s, gc.d + 0.95); (rx, rz, _) = CLASSICAI.tworld(CLASSICAI_T, gc.s, gc.d - 0.95)
            if !(JRTrack.hat3d(ROADHAT, lx, lz; ref = Inf)[3] && JRTrack.hat3d(ROADHAT, rx, rz; ref = Inf)[3])
                AI_OFF[3] += 1; b = floor(Int, gc.s/100); AI_EDGEBIN[b] = get(AI_EDGEBIN, b, 0) + 1
            end
            if AI_OFF_POS !== nothing
                (lx2, lz2, _) = CLASSICAI.tworld(CLASSICAI_T, gc.s, gc.d + AI_OFF_POS + 0.95); (rx2, rz2, _) = CLASSICAI.tworld(CLASSICAI_T, gc.s, gc.d + AI_OFF_POS - 0.95)
                (JRTrack.hat3d(ROADHAT, lx2, lz2; ref = Inf)[3] && JRTrack.hat3d(ROADHAT, rx2, rz2; ref = Inf)[3]) || (AI_OFF[4] += 1)
            end
        end
    end
    crossed = falses(length(AICARS))
    for (i, gc) in enumerate(f.cars)
        i > length(AICARS) && break
        prevs = AICARS[i].s
        s, lat = RaceAI.project(AILINE, poses[i][1], poses[i][3]; hint = prevs)
        AICARS[i].s = s; AICARS[i].lane = lat; AICARS[i].v = gc.v
        (prevs > AILINE.total*0.7 && s < AILINE.total*0.3) && (AICARS[i].lap += 1; crossed[i] = true)
    end

    ai_cooldown_step!(f, AICARS, CLASSICAI_T, crossed, ent_name, phase, race_done, race_go)
    poses
end

"""The physics AI field (JM_AI_PHYSICS), one frame: plan, drive and step each AI Car3D, grass and solids, car-car
contact; returns the field's poses. LOOPSPLIT-1 cycle 4 S2: moved unchanged."""
function aiphys_poses!(cs, ddt, ai_hit, AICARS, AILINE, AIPHYS, ai_onroad, ai_stuck, groundz_ai, aibankP, groundz)

    for (i, pc) in enumerate(AIPHYS)

        prevs = AICARS[i].s
        s, lat = RaceAI.project(AILINE, pc.x, pc.z; hint = prevs)
        AICARS[i].s = s; AICARS[i].lane = lat; AICARS[i].v = pc.v
        (prevs > AILINE.total*0.7 && s < AILINE.total*0.3) && (AICARS[i].lap += 1)
        TRACE_DS && ds_note!(s - prevs, pc.v, ddt, AILINE.total)

        offhat = !JRTrack.hat3d(TERRAIN, pc.x, pc.z; ref=Inf)[3]
        railθ = RaceAI.pose_at(AILINE, s, 0.0)[4]
        spun = abs(atan(sin(pc.θ - railθ), cos(pc.θ - railθ))) > SPIN_LIM
        off = pc.v < 1.4 || abs(lat) > 9.0 || offhat || spun
        if off
            ai_stuck[i] += 1
            lim = (offhat || abs(lat) > 12.0 || spun) ? 6 : (abs(lat) > 9.0 ? 18 : 90)
            if ai_stuck[i] > lim
                rp = RaceAI.pose_at(AILINE, s + 10.0, 0.0); AIplace!(pc, rp[1], rp[3], rp[4]; v = max(8.0, pc.v*0.6)); ai_stuck[i] = 0
            end
        else; ai_stuck[i] = 0; end
    end

    ppr = RaceAI.project(AILINE, cs.x, cs.z)
    vts = RaceAI.plan!(AICARS, AILINE; scale = 1.0, amax = AI_AMAX, dt = ddt,
                       player = (ppr[1], ppr[2], cs.v))
    for (i, pc) in enumerate(AIPHYS)
        thr, brk, st = RaceAI.controller(AILINE, AICARS[i].s, AICARS[i].lane, AICARS[i].tlane, vts[i],
                                         pc.x, pc.z, pc.θ, pc.v, AIyaw(pc); power = AI_POWER)
        _tas = time(); DriveRT3D.step_car3d!(pc, thr, brk, st, ddt; manual=false, groundz=groundz_ai); DS_SIM[2] += time() - _tas

        hs = JRTrack.hat(TRKSURF, pc.x, pc.z)
        ai_onroad[i] = !(hs.found && abs(hs.lateral) > ROAD_HALFW)
        if !ai_onroad[i] && pc.v > 2.5
            AIbump!(pc, -GRASS_DRAG*pc.v*cos(pc.θ)*ddt, -GRASS_DRAG*pc.v*sin(pc.θ)*ddt, (2*rand()-1)*GRASS_SLIP*ddt)
        end
        ho = solid_hit(pc.x, pc.z, pc.θ, pc.v)
        ho !== nothing && AIbump!(pc, ho[1], ho[2], ho[3], ho[4], ho[5], ho[6])
    end

    let plead = (cs.x, cs.z, cs.θ, cs.v)
        for (i, pc) in enumerate(AIPHYS)
            ai_onroad[i] || continue
            leads = NTuple{4,Float64}[plead]
            for j in 1:length(AIPHYS); j != i && push!(leads, (AIPHYS[j].x, AIPHYS[j].z, AIPHYS[j].θ, AIPHYS[j].v)); end
            tw = draft_tow(pc.x, pc.z, pc.θ, pc.v, leads)
            tw > 0.0 && AIbump!(pc, tw*cos(pc.θ)*ddt, tw*sin(pc.θ)*ddt, 0.0)
        end
    end

    for a in 1:length(AIPHYS)-1, b in a+1:length(AIPHYS)
        pa = AIPHYS[a]; pb = AIPHYS[b]
        dx = pb.x-pa.x; dz = pb.z-pa.z; d = hypot(dx,dz)
        (d < 1e-3 || d > contact_d(dx, dz, d, pa.θ, pb.θ)) && continue
        nx = dx/d; nz = dz/d
        vrel = (pa.v*cos(pa.θ)-pb.v*cos(pb.θ))*nx + (pa.v*sin(pa.θ)-pb.v*sin(pb.θ))*nz
        vrel <= 0.2 && continue
        imp = clamp(1.45*vrel*280.0/560, 0.0, 4.0)
        AIbump!(pa, -imp*nx, -imp*nz, clamp(-imp*0.04, -0.6, 0.6))
        AIbump!(pb,  imp*nx,  imp*nz, clamp( imp*0.04, -0.6, 0.6))
    end
    [(b=aibankP(pc); (pc.x, isfinite(pc.y) ? pc.y : groundz(pc.x, pc.z), pc.z, pc.θ, b[1], b[2])) for pc in AIPHYS]
end

function track_position_step!(cs, dt, rst, race_go, player_prog, player_s_prev, ffb_jolt, groundz_phys,
                              BND_FX, BND_FY, BND_MZ, CLINE, LASTGX, LASTGZ, OFFDIST, ONTRACK, TRACE_PROG, prog_bad, prog_t0)
    hr = JRTrack.hat(TRKSURF, cs.x, cs.z)
    if hr.found
        cs.lapdist = hr.lapdist; cs.lateral = hr.lateral; cs.along = hr.lapdist; cs.ontrack = hr.on_track
    else; cs.ontrack = false; end

    if CLINE !== nothing
        ps = RaceAI.project(CLINE, cs.x, cs.z)[1]

        ds = prog_delta(ps, player_s_prev, CLINE.total; teleport = rst)
        race_go[] && (player_prog += ds)
        player_s_prev = ps
        while cs.laps < floor(Int, player_prog/CLINE.total); cs.laps += 1; end

        if TRACE_PROG
            _oldpp = cs.laps + (LAPLEN > 0 ? clamp(cs.lapdist/LAPLEN, 0.0, 1.0) : 0.0)
            _newpp = player_prog/CLINE.total
            _bad = abs(_oldpp - _newpp) > 0.5
            if _bad && !prog_bad[]
                prog_bad[] = true; prog_t0[] = cs.t
                println("  [prog] DISAGREE from t=", round(cs.t, digits=2),
                        "s  laps=", cs.laps, " lapdist=", round(cs.lapdist, digits=1),
                        "  old=", round(_oldpp, digits=3), " new=", round(_newpp, digits=3))
            elseif !_bad && prog_bad[]
                prog_bad[] = false
                println("  [prog] agree again at t=", round(cs.t, digits=2),
                        "s  -- the readout was a lap out for ",
                        round(cs.t - prog_t0[], digits=2), " s")
            end
        end
    end

    if TRAIL_SECS > 0.0 && !WRECKED[] && cs.t - TRAIL_LAST[] >= 1.0/TRAIL_HZ
        TRAIL_LAST[] = cs.t

        _lat = hr.found ? hr.lateral : NaN
        _lap = hr.found ? hr.lapdist : NaN
        push!(TRAIL_BUF, (cs.t, cs.x, cs.z, cs.v, cs.θ, OFFDIST[], _lat, _lap))
        while length(TRAIL_BUF) > 1 && TRAIL_BUF[end][1] - TRAIL_BUF[1][1] > TRAIL_SECS
            popfirst!(TRAIL_BUF)
        end
    end
    if ONTRACK[]; LASTGX[] = cs.x; LASTGZ[] = cs.z; OFFDIST[] = 0.0
        BND_FX[] = 0.0; BND_FY[] = 0.0; BND_MZ[] = 0.0
    elseif (hr.found && abs(hr.lateral) < ROAD_HALFW) ||
           (RUNOFF_W > 0 && CLINE !== nothing && abs(RaceAI.project(CLINE, cs.x, cs.z)[2]) < RUNOFF_W)

        LASTGX[] = cs.x; LASTGZ[] = cs.z; OFFDIST[] = 0.0
        BND_FX[] = 0.0; BND_FY[] = 0.0; BND_MZ[] = 0.0; BND_PK[] = 0.0
    else
        OFFDIST[] += cs.v * (dt > 1e-4 ? dt : 1/60)

        nwx = LASTGX[]-cs.x; nwz = LASTGZ[]-cs.z; nl = hypot(nwx,nwz)
        nl < 1e-3 && (nwx = cos(cs.θ+π); nwz = sin(cs.θ+π); nl = 1.0)
        nwx /= nl; nwz /= nl
        if nl > FENCE_GRACE
            pvx = WVX[]; pvz = WVZ[]

            vn = pvx*nwx + pvz*nwz
            gdt = dt > 1e-4 ? dt : 1/60
            (bfx, bfy, bmz) = DriveRT3D.contact_force(nl - FENCE_GRACE, nwx, nwz, vn, cs.θ; kind = :wall, dt = gdt)
            BND_FX[] = bfx; BND_FY[] = bfy; BND_MZ[] = bmz
            BND_PK[] = hypot(bfx, bfy)
            BND_NL[] = nl - FENCE_GRACE; BND_VN[] = vn; BND_OFF[] = OFFDIST[]

            BND_NX[] = nwx; BND_NZ[] = nwz
            ffb_jolt = clamp(-vn*0.05, -1.0, 1.0)

            if nl > FENCE_GRACE + FENCE_FAR
                (sealx, sealz) = WreckSeal.seal_target(WRECKED[], cs.x, cs.z,
                                     LASTGX[], LASTGZ[];
                                     seal_back = haskey(ENV, "JM_WRECK_SEAL_BACK"))
                CLASSICW_ON && classicw_teleported!()
                containX!(cs, sealx, sealz; vdamp=(WRECKED[] ? 0.0 : 0.3), settle=true, groundz=groundz_phys)
                BND_FX[] = 0.0; BND_FY[] = 0.0; BND_MZ[] = 0.0; BND_PK[] = 0.0; OFFDIST[] = 0.0
            end
        else
            BND_FX[] = 0.0; BND_FY[] = 0.0; BND_MZ[] = 0.0; BND_PK[] = 0.0
        end
    end
    return (player_prog, player_s_prev, ffb_jolt)
end

function netlink_step!(cs, inp, frames, ai_poses, AICARS, CLINE, aibankK, ai_ground, net_last)

    yield()
    NetPlay.poll!(NETLINK)

    if NETMODE == "join" && !NET_SWITCHED[] && haskey(NETLINK.remote, 0x01)
        NET_SWITCHED[] = true
        net_free_car()
    end
    if cs.t - net_last[] >= 1/NET_HZ
        net_last[] = cs.t
        NetPlay.send_pose!(NETLINK, NET_ID, round(Int, cs.t*1000),
                           cs.x, cs.y, cs.z, cs.θ, cs.v, inp.steer; car = NETLINK.car)

        if NETMODE == "host" && !isempty(ai_poses)
            for (k, p) in enumerate(ai_poses)
                NetPlay.send_pose!(NETLINK, NetPlay.AI_ID0 + (k - 1), round(Int, cs.t*1000),
                                   p[1], p[2], p[3], p[4],
                                   k <= length(AICARS) ? AICARS[k].v : 0.0, 0.0; car = k <= length(AISPECS) ? car_type(AISPECS[k][1]) : 0xff)
            end
        end
    end

    netp = Tuple{Int,NTuple{6,Float64}}[]; empty!(NETVEL[])
    for (id, q) in NetPlay.remote_poses_at(NETLINK, time())
        b = aibankK((q.x, 0.0, q.z, q.yaw))
        push!(netp, (Int(id), ai_ground((q.x, q.y, q.z, q.yaw, b[1], b[2]))))
        NETCARS[][Int(id)] = q.car
        NETVEL[][Int(id)] = (q.v*cos(q.yaw), q.v*sin(q.yaw))
    end

    if NET_ERR
        for (id, q) in NETLINK.remote
            pv = get(NET_PREV[], id, nothing)
            if pv !== nothing && q.tick != pv.tick
                dt = (q.tick - pv.tick) / 1000
                if 0 < dt < 1.0
                    pr = NetPlay.predict(pv, dt)
                    push!(NET_ERRS, hypot(q.x - pr.x, q.z - pr.z))
                    push!(NET_DTS, dt)
                end
            end
            NET_PREV[][id] = q
        end
        if !isempty(NET_ERRS) && (frames % 600) == 0
            m = sum(NET_ERRS)/length(NET_ERRS)
            println("  [neterr] n=", length(NET_ERRS),
                    "  interval mean=", round(sum(NET_DTS)/length(NET_DTS), digits=3), "s",
                    "  ERR mean=", round(m, digits=4), " m  max=",
                    round(maximum(NET_ERRS), digits=4), " m")
            flush(stdout)
        end
    end
    NETPOSES[] = netp
    for (_, p) in netp
        _d = hypot(p[1] - cs.x, p[3] - cs.z)

        _g = _d < CAR_BOUND_D ? car_contact_geom(cs.x, cs.z, cs.θ, p[1], p[3], p[4]) : nothing
        _g !== nothing && (NET_PENMAX[] = max(NET_PENMAX[], _g[5]))
        NET_PENNOW[] = _g === nothing ? 0.0 : _g[5]
        if _d < NET_DMIN[]
            NET_DMIN[] = _d

            CLINE !== nothing && (NET_DMIN_AT[] = (RaceAI.project(CLINE, cs.x, cs.z), RaceAI.project(CLINE, p[1], p[3])))
        end
    end

    if NET_DIAG > 0 && (frames % NET_DIAG) == 0
        println("  [net] t=", round(cs.t,digits=1), " rx=", NETLINK.rx,
                " dropped=", NETLINK.dropped, " peers=", length(NETLINK.peers),
                " known=", length(NETLINK.remote), " drawn=", length(netp),
                " hits=", NET_HITS[], " dmin=", round(NET_DMIN[], digits = 2), " pen=", round(NET_PENMAX[], digits = 2), " pennow=", round(NET_PENNOW[], digits = 2), " v=", round(cs.v, digits = 1),
                NET_DMIN_AT[] === nothing ? "" : string(" at self(s,lat)=", round.(NET_DMIN_AT[][1], digits = 2),
                                                        " remote=", round.(NET_DMIN_AT[][2], digits = 2)))
        flush(stdout)
    end
    return nothing
end

"""CRASHBODY-1: the wreck as a free rigid body (wreck_body.jl), one frame. Hands the car over on the first call -- its pose,
speed and yaw rate, the wheels still on, each corner's grip -- then steps it on the ground under it against the Classic walls
and the trackside objects near it (a hedge gives and holds), with the driver's brake on the wheels, and writes the result
back to the game's car (position, heading, speed, attitude, the session clock) so lap position, camera, telemetry and the
replay carry on. Torn-off wheels leave the body as they leave the car (is_loose)."""
function wreck_body_step!(cs, inp, dt, groundz_phys::G, roll_ter; dry::Bool = false) where {G}
    WB = WreckBody; V3 = WB.V3
    gdt = dt > 1e-4 ? dt : 1/60
    b = WBODY[]
    if dry

        b = WB.Body(WB.Geometry(m = DriveRT3D.MASS[]))
        WB.start!(b, cs.x, cs.z, cs.y, cs.θ, cs.pitch, roll_ter + cs.roll, V3(WVX[], 0.0, -WVZ[]), V3(0.0, Float64(DriveRT3D.yawrate3d(cs)), 0.0))
    elseif b === nothing
        b = WB.Body(WB.Geometry(m = DriveRT3D.MASS[]))
        on = ntuple(i -> !is_loose(WHEEL_NAMES[i]), 4)

        (pp, q, w) = attrates3d(cs)
        WB.start!(b, cs.x, cs.z, cs.y, cs.θ, cs.pitch, roll_ter + cs.roll, V3(WVX[], w, -WVZ[]),
                  V3(pp, Float64(DriveRT3D.yawrate3d(cs)), q); on = on, dmu = ntuple(i -> DriveRT3D.damage_mu(i), 4))
        (SMOKE && WRECK_ROLL != 0) && (WRECK_ROLL_DUE[] = true)
        WBODY[] = b; WBODY_PLANE[] = WB.Plane(cs.x, -cs.z, cs.y, 0.0, 0.0)
        println(body_msg(WRECKED[], Float64(cs.v), count(on), pp, q, w)); flush(stdout)
    end
    dry || for i in 1:4; b.on[i] = !is_loose(WHEEL_NAMES[i]); b.dmu[i] = DriveRT3D.damage_mu(i); end
    b.brake = clamp(inp.brake, 0.0, 1.0)

    hq(X, Z) = (y = groundz_phys(X, -Z); y > -900 ? Float64(y) : NaN)
    pl = WB.fit_plane(hq, b.p[1], b.p[3])
    (isfinite(pl.h0) && isfinite(pl.gx) && isfinite(pl.gz)) ? (dry || (WBODY_PLANE[] = pl)) : (pl = WBODY_PLANE[])

    walls = WB.WallPlane[]
    gx, gz = b.p[1], -b.p[3]
    if CLASSICW_PHYS && !dry
        (lat, lo, hi, glo, ghi, ngx, ngz, nlo, nhi, gend, nend) = classicwall_gaps(gx, gz, cs.θ)
        for (g, n) in ((glo, nlo), (ghi, nhi))
            d = g + CARLF*abs(cos(cs.θ)*n[1] + sin(cs.θ)*n[2]) + CARW
            (isfinite(d) && d < 4.0) || continue
            push!(walls, WB.WallPlane(V3(gx - d*n[1], b.p[2], -(gz - d*n[2])), V3(n[1], 0.0, -n[2])))
        end
        if isfinite(gend) && gend < 3.0
            d = gend + CARLF
            push!(walls, WB.WallPlane(V3(gx - d*nend[1], b.p[2], -(gz - d*nend[2])), V3(nend[1], 0.0, -nend[2])))
        end
    end
    for k in eachindex(SOLIDS)
        (ox, oz, r, kind) = SOLIDS[k]
        hypot(ox - gx, oz - gz) < r + 6.0 || continue
        (g, nx, nz) = solid_gap(gx, gz, k)
        g < 4.0 || continue
        push!(walls, WB.WallPlane(V3(gx - g*nx, b.p[2], -(gz - g*nz)), V3(nx, 0.0, -nz), kind === :soft))
    end

    th(X, Z, yref) = SKIDPAD ? 0.0 : (r = JRTrack.hat3d(TERRAIN, X, -Z; ref = yref); r[3] ? Float64(r[1]) : objground_at(X, -Z, yref))
    ter = WB.Terrain(th, pl)

    if WRECK_ROLL_DUE[]
        WRECK_ROLL_DUE[] = false
        s = dot(b.v, b.R[:, 3]) >= 0 ? 1.0 : -1.0
        b.ω = b.ω + V3(s*abs(WRECK_ROLL), 0.0, 0.0); b.v = b.v + V3(0.0, 2.0, 0.0)
        println("[crashbody] test roll kick ", s > 0 ? "+" : "-", abs(WRECK_ROLL), " rad/s, sliding ", round(dot(b.v, b.R[:, 3]), digits = 1), " m/s to body +z")
    end
    v0 = b.v; θ0 = WB.attitude(b)[1]
    WB.step!(b, gdt, ter; walls = walls)
    dry && (WB.touching(b, ter); return nothing)
    sf = (b.v - v0)/max(gdt, 1e-6) + V3(0.0, 9.80665, 0.0)
    WB_TELE[] = (r = (mod(WB.attitude(b)[1] - θ0 + π, 2π) - π)/max(gdt, 1e-6), ax = dot(sf, b.R[:, 1]), ay = -dot(sf, b.R[:, 3]), az = dot(sf, b.R[:, 2]))

    (b.wall_peak > 3.0e4 && norm(b.v) > 3.0) &&
        detach_hit_wheels!(cs.x, cs.z, cs.θ, norm(b.v); fence = true, fence_nx = b.wall_n[1], fence_nz = -b.wall_n[3])

    TEAR_DIAG && (maximum(b.gnd_v) > 2.0 || maximum(b.inv_f) > 0.0) &&
        println("  [teardiag] t ", round(b.t, digits = 2), " roll ", round(rad2deg(WB.attitude(b)[3]), digits = 0), " strike ",
                round.(b.gnd_v, digits = 1), " inv ", round.(Int, b.inv_f), " N (weight ", round(Int, b.g.m*9.80665), ")")
    for i in WB.ground_tears(b, WHEEL_TEAR[1], WHEEL_TEAR[2]*b.g.m*9.80665)
        nm = WHEEL_NAMES[i]; is_loose(nm) && (b.on[i] = false; continue)
        b.on[i] = false
        hub = b.p + b.R*V3(b.g.wheel_x[i], b.g.wheel_r[i] - b.g.hcg, b.g.wheel_z[i])
        push!(LOOSE_WHEELS, (hub[1], hub[2], -hub[3], b.v[1], max(b.v[2], 0.0) + 2.0, -b.v[3], 0.0, b.spin[i], nm))
        println("  [WRECK] ", nm, " torn off on the ground (struck at ", round(b.gnd_v[i], digits = 1), " m/s, ",
                round(Int, b.inv_f[i]), " N inverted) at ", round(norm(b.v)*3.6, digits = 0), " km/h"); flush(stdout)
    end
    θ, pitch, roll = WB.attitude(b)
    _px = b.p[1]; _pz = b.p[3]
    if CRASHBODY_DIAG && (floor(Int, 4*b.t) != floor(Int, 4*(b.t - gdt)) || b.rest)
        println("  [crashbody] t ", round(b.t, digits = 2), " CG ", round(b.p[2] - WB.plane_y(pl, b.p[1], b.p[3]), digits = 2),
                " m up  roll ", round(rad2deg(roll), digits = 0), " pitch ", round(rad2deg(pitch), digits = 0), " heading ",
                round(rad2deg(θ), digits = 0), "  v ", round(norm(b.v), digits = 1), " m/s  w ", round(norm(b.ω), digits = 1),
                " rad/s  contacts ", b.contacts, "  walls ", length(walls), b.rest ? "  AT REST" : "",
                "  ground ", round(pl.h0, digits = 2), " [", join([round(hq(_px + dx, _pz + dz), digits = 2) for (dx, dz) in ((1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5))], " "),
                "] at game (", round(b.p[1], digits = 1), ", ", round(-b.p[3], digits = 1), ")"); flush(stdout)
    end
    o = b.p - b.R * V3(0.0, b.g.hcg, 0.0)
    cs.x = o[1]; cs.z = -o[3]; cs.y = o[2]; cs.θ = θ
    cs.v = hypot(b.v[1], b.v[3]); WVX[] = b.v[1]; WVZ[] = -b.v[3]
    cs.pitch = pitch; cs.roll = roll
    cs.toff += gdt; cs.t = cs.integ.t + cs.toff

    if CLASSICW_PHYS
        x0, z0 = cs.x, cs.z
        for _ in 1:2
            classicwall_clamp!(cs)
            OBS_ON && obs_clamp!(cs)
        end
        dx = cs.x - x0; dz = cs.z - z0; dl = hypot(dx, dz)
        if dl > 1e-4
            nr = V3(dx/dl, 0.0, -dz/dl)
            b.p += V3(dx, 0.0, -dz)
            vn = dot(b.v, nr); vn < 0 && (b.v -= vn*nr)
            WVX[] = b.v[1]; WVZ[] = -b.v[3]; cs.v = hypot(b.v[1], b.v[3])
        end
    end
    if AIRBORNE[] && !WRECKED[]
        (nt, nh) = WB.touching(b, ter)
        airbody_land!(cs, b, groundz_phys, nt, nh, WB.plane_n(pl), gdt)
    end
    nothing
end

"""CARLAUNCH-1: a launched car's flight, each frame after its step. A wheel lost in the air wrecks it; once it stands on its
four tyres, upright and settled, for 0.15 s, the driving model takes it back where it is, at its speed, slip and yaw rate;
if it comes to rest any other way (on its side, its roof, a hull point down) it is a wreck."""
function airbody_land!(cs, b, groundz_phys::G, nt::Int, nh::Int, n, gdt) where {G}
    WB = WreckBody; V3 = WB.V3
    if !all(b.on)
        AIRBORNE[] = false
        wreck!(cs.v; x = cs.x, z = cs.z, cause = "a wheel came off while the car was in the air (CARLAUNCH-1)")
        return nothing
    end
    (_, _p, _r) = WB.attitude(b)
    AIR_PEAK[] = (abs(_r) > abs(AIR_PEAK[][1]) ? _r : AIR_PEAK[][1], abs(_p) > abs(AIR_PEAK[][2]) ? _p : AIR_PEAK[][2])
    up = dot(b.R[:, 2], n)
    settled = nt == 4 && nh == 0 && up > cos(deg2rad(6.0)) && abs(b.ω[1]) < 0.6 && abs(b.ω[3]) < 0.6 && abs(dot(b.v, n)) < 0.6
    AIR_LAND[] = settled ? AIR_LAND[] + gdt : 0.0
    if b.rest && !settled
        AIRBORNE[] = false
        wreck!(cs.v; x = cs.x, z = cs.z, cause = "the launched car came to rest off its wheels (CARLAUNCH-1)")
        return nothing
    end
    (AIR_LAND[] >= 0.15 || (b.rest && settled)) || return nothing

    θ = WB.attitude(b)[1]
    o = b.p - b.R * V3(0.0, b.g.hcg, 0.0)
    x = o[1]; z = -o[3]; wvx = b.v[1]; wvz = -b.v[3]
    u = wvx*cos(θ) + wvz*sin(θ); vl = -wvx*sin(θ) + wvz*cos(θ)
    r = (b.R * b.ω)[2]
    h = groundz_phys(x, z)
    DriveRT3D.pin3d!(cs, x, z, θ, h > -900 ? Float64(h) : o[2])
    cs.s_vel(cs.integ, [u, vl]); cs.v = hypot(u, vl)
    DriveRT3D._RSET3D[cs.sys][2](cs.integ, r)
    DriveRT3D.OrdinaryDiffEq.SciMLBase.u_modified!(cs.integ, true)
    WVX[] = wvx; WVZ[] = wvz
    AIRBORNE[] = false; AIR_LAND[] = 0.0; WBODY[] = nothing
    println(landed_msg(b.t, b.peak_h, Float64(cs.v), AIR_PEAK[])); flush(stdout)
    nothing
end

launch_msg(v::Float64, rh) = string("  [airbody] launched at ", round(v*3.6, digits = 0), " km/h: ride heights ", round(rh[1], digits = 2), " ",
    round(rh[2], digits = 2), " ", round(rh[3], digits = 2), " ", round(rh[4], digits = 2), " m -- a free rigid body until it lands")
body_msg(wrecked::Bool, v::Float64, n::Int, pp::Float64, q::Float64, w::Float64) = string("  [crashbody] ",
    wrecked ? "the wreck" : "the launched car", " is a free rigid body from here: ", round(v*3.6, digits = 0), " km/h, ", n,
    " wheels on, roll rate ", round(pp, digits = 2), " pitch rate ", round(q, digits = 2), " heave ", round(w, digits = 2))
landed_msg(t::Float64, h::Float64, v::Float64, pk::Tuple{Float64,Float64}) = string("  [airbody] landed after ", round(t, digits = 2),
    " s (highest CG ", round(h, digits = 2), " m, roll to ", round(rad2deg(pk[1]), digits = 0), " deg, pitch to ",
    round(rad2deg(pk[2]), digits = 0), " deg): back on the driving model at ", round(v*3.6, digits = 0), " km/h")
