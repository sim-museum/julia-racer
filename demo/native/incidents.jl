

const INC_ON = get(ENV, "JM_INC", "1") != "0"
const INC_SEQ_S = parse(Float64, get(ENV, "JM_INC_SEQ_S", "2.5"))
const INC_LIVE = Ref(false)
const INC_PTS = Dict{Int,Int}()
const INC_SEQ = Dict{Int,Tuple{Float64,Int}}()
const INC_OFF = Dict{Int,Bool}(); const INC_SPUN = Dict{Int,Bool}()
const INC_PREV = Ref((NaN, NaN))
inc_reset!() = (empty!(INC_PTS); empty!(INC_SEQ); empty!(INC_OFF); empty!(INC_SPUN); INC_PREV[] = (NaN, NaN); bf_reset!(); nothing)
inc_name(k) = k == 0 ? "you" : string("AI ", k)
inc_points(k) = get(INC_PTS, k, 0)

"Record an incident of `pts` (0, 1, 2 or 4) for car `k` at clock `t`; returns the points added (reference's sequence rule)."
function inc_add!(k::Int, pts::Int, why::AbstractString, t::Float64)
    (INC_ON && INC_LIVE[]) || return 0
    sq = get(INC_SEQ, k, (-Inf, 0)); inseq = t - sq[1] <= INC_SEQ_S
    add = inseq ? max(pts - sq[2], 0) : pts
    INC_SEQ[k] = (t, inseq ? max(pts, sq[2]) : pts)
    if add > 0 || (pts == 0 && !inseq)
        INC_PTS[k] = inc_points(k) + add
        println("  [inc] t ", round(t, digits = 1), " ", inc_name(k), " ", pts, "x ", why, add != pts ? string(" (+", add, "x: in a sequence)") : "",
                " -> ", inc_points(k), "x"); flush(stdout)
        add > 0 && bf_check!(k, t)
    end
    add
end
"True on the frame `on` becomes true for car `k` in edge table `d`."
inc_edge!(d::Dict{Int,Bool}, k::Int, on::Bool) = (was = get(d, k, false); d[k] = on; on && !was)

"Your car's frame: off track (all four wheels on grass) and a spin (heading more than 100 deg from the direction of travel)."
function inc_player_step!(cs, grass_n::Int)
    (INC_ON && INC_LIVE[]) || (INC_PREV[] = (Float64(cs.x), Float64(cs.z)); return nothing)
    inc_edge!(INC_OFF, 0, grass_n >= 4 && cs.v > 3.0) && inc_add!(0, 1, "off track", Float64(cs.t))
    (px, pz) = INC_PREV[]; INC_PREV[] = (Float64(cs.x), Float64(cs.z))
    if isfinite(px)
        dx = cs.x - px; dz = cs.z - pz
        if hypot(dx, dz) > 0.05
            β = abs(atan(sin(cs.θ - atan(dz, dx)), cos(cs.θ - atan(dz, dx))))
            inc_edge!(INC_SPUN, 0, β > deg2rad(100.0)) && inc_add!(0, 2, "loss of control", Float64(cs.t))
            β < deg2rad(45.0) && (INC_SPUN[0] = false)
        end
    end
    nothing
end

const INC_LIMIT = parse(Int, get(ENV, "JM_INC_LIMIT", "15")); const INC_EVERY = parse(Int, get(ENV, "JM_INC_EVERY", "7"))
const INC_MAX = parse(Int, get(ENV, "JM_INC_MAX", "35"))
const BF_LAPS = parse(Int, get(ENV, "JM_BF_LAPS", "3")); const DT_KMH = parse(Float64, get(ENV, "JM_DT_KMH", "80"))
const BF_DUE = Dict{Int,Int}()
const BF_DEADLINE = Dict{Int,Int}()
const BF_NEXT = Dict{Int,Int}()
const DSQ = Dict{Int,String}()
const BF_PIT = Ref((false, 0.0))
const CLS_NC = Dict{Int,Bool}()
bf_reset!() = (empty!(BF_DUE); empty!(BF_DEADLINE); empty!(BF_NEXT); empty!(DSQ); empty!(CLS_NC); BF_PIT[] = (false, 0.0); empty!(INC_HIST); empty!(INC_LASTS); empty!(RULE_LAST); RULE_N[] = 0; nothing)
"S3: car k's result status -- \"DSQ\", \"NC\" or \"\" (classified)."
cls_status(k) = haskey(DSQ, k) ? "DSQ" : get(CLS_NC, k, false) ? "NC" : ""
cls_tag(k) = (s_ = cls_status(k); isempty(s_) ? "" : string(" -- ", s_ == "DSQ" ? "DISQUALIFIED" : "NOT CLASSIFIED"))
bf_pending(k) = get(BF_DUE, k, 0) > 0 && !haskey(DSQ, k)
function dsq!(k, why)
    haskey(DSQ, k) && return nothing
    DSQ[k] = why
    println("  [dsq] ", inc_name(k), " disqualified: ", why); flush(stdout)
    k == 0 && gearbox_notice!(string("DISQUALIFIED -- ", why), 10.0)
    nothing
end
"After car k's points grew: the black flags it has earned, and the maximum."
function bf_check!(k, t)
    (INC_LIMIT > 0 && !haskey(DSQ, k)) || return nothing
    p = inc_points(k)
    (INC_MAX > 0 && p >= INC_MAX) && return dsq!(k, string(p, "x incidents (the maximum is ", INC_MAX, "x)"))
    nx = get(BF_NEXT, k, INC_LIMIT)
    while p >= nx
        BF_DUE[k] = get(BF_DUE, k, 0) + 1; BF_DEADLINE[k] = -1; nx += max(INC_EVERY, 1)
        println("  [blackflag] t ", round(t, digits = 1), " ", inc_name(k), ": ", p, "x -- the black flag, a drive-through within ", BF_LAPS, " laps"); flush(stdout)
        k == 0 && gearbox_notice!(string("BLACK FLAG (", p, "x) -- drive through the pit lane under ", round(Int, DT_KMH), " km/h within ", BF_LAPS, " laps"), 8.0)
    end
    BF_NEXT[k] = nx
    nothing
end
"Car k on lap `lap`: the owed penalty's deadline; past it unserved = DSQ."
function bf_lap!(k, lap)
    (INC_LIVE[] && bf_pending(k)) || return nothing
    d = get(BF_DEADLINE, k, -1)
    d < 0 && (BF_DEADLINE[k] = lap + BF_LAPS; return nothing)
    lap > d && dsq!(k, string("the black flag not served within ", BF_LAPS, " laps"))
    nothing
end
function bf_serve!(k, t, how)
    bf_pending(k) || return false
    BF_DUE[k] -= 1; BF_DEADLINE[k] = -1
    println("  [blackflag] t ", round(t, digits = 1), " ", inc_name(k), " served the black flag (", how, ")", BF_DUE[k] > 0 ? string(", ", BF_DUE[k], " still owed") : ""); flush(stdout)
    k == 0 && gearbox_notice!(BF_DUE[k] > 0 ? "Black flag served -- another one owed" : "Black flag served", 3.0)
    true
end
"Your frame: the deadline, and a drive-through (the box stretch entered and left under DT_KMH)."
function bf_player_step!(cs, T, CLINE)
    INC_LIVE[] || return nothing
    bf_lap!(0, cs.laps)
    bf_pending(0) || (BF_PIT[] = (false, 0.0); return nothing)
    inb = in_pit_box(cs.x, cs.z, T, CLINE); (was, vmax) = BF_PIT[]
    if inb
        BF_PIT[] = (true, max(vmax, Float64(cs.v)))
    elseif was
        BF_PIT[] = (false, 0.0)
        if 3.6vmax <= DT_KMH
            bf_serve!(0, Float64(cs.t), string("a drive-through, top speed ", round(Int, 3.6vmax), " km/h"))
        else
            println("  [blackflag] too fast in the pit lane (", round(Int, 3.6vmax), " km/h, the limit ", round(Int, DT_KMH), "): not served"); flush(stdout)
            gearbox_notice!(string("TOO FAST IN THE PIT LANE (", round(Int, 3.6vmax), " km/h) -- not served"), 4.0)
        end
    end
    nothing
end

const RULE_ON = get(ENV, "JM_RULE", "1") != "0"
const RULE_ZONE = parse(Float64, get(ENV, "JM_RULE_ZONE", "0.85")); const RULE_GAP = parse(Float64, get(ENV, "JM_RULE_GAP", "1.0"))
const RULE_CLOSE = parse(Float64, get(ENV, "JM_RULE_CLOSE", "3.0")); const RULE_CARL = 4.0
const RULE_SIDE = parse(Float64, get(ENV, "JM_RULE_SIDE", "1.0")); const RULE_DIVE = parse(Float64, get(ENV, "JM_RULE_DIVE", "0.8"))
const RULE_SEQ_S = 3.0; const RULE_LAST = Dict{Tuple{Int,Int},Float64}()
const INC_HIST = Dict{Int,Vector{NTuple{4,Float64}}}()
const INC_LASTS = Dict{Int,Float64}()
const RULE_N = Ref(0)
"Car k's position sample at clock t: `s` its lap distance on the AI line (wrapped, `L` the lap), `v` its speed, `lane` its lateral."
function inc_hist_push!(k::Int, t::Float64, s::Float64, L::Float64, v::Float64, lane::Float64)
    last = get(INC_LASTS, k, s); su = s + L*round((last - s)/L); INC_LASTS[k] = su
    H = get!(() -> NTuple{4,Float64}[], INC_HIST, k)
    (isempty(H) || t - H[end][1] >= 0.1) && push!(H, (t, su, v, lane))
    while length(H) > 31; popfirst!(H); end
    nothing
end
"Car k's sample nearest clock t (nothing if none)."
function inc_hist_at(k::Int, t::Float64)
    H = get(INC_HIST, k, nothing); (H === nothing || isempty(H)) && return nothing
    j = argmin([abs(h[1] - t) for h in H]); abs(H[j][1] - t) < 0.3 ? H[j] : nothing
end
"Is lap distance s in a corner's braking / turn-in zone (the Racecraft profile under `frac` of its highest over the 150 m before)?"
rule_zone(s::Float64) = rule_zone_at(s, RULE_ZONE)
const RULE_AI_OBEY = get(ENV, "JM_RULE_AI", "1") != "0"
const RULE_AI_ZONE = parse(Float64, get(ENV, "JM_RULE_AI_ZONE", "0.75"))
function rule_zone_at(s::Float64, frac::Float64)
    P = RAI_PROF[]; P === nothing && return false
    (st, v) = P; n = length(v); n < 10 && return false
    ds = st[2] - st[1]; L = RAI_TOTAL[]; k = clamp(floor(Int, mod(s, L)/ds) + 1, 1, n)
    vmax = maximum(v[mod1(k - j, n)] for j in 0:round(Int, 150/ds))
    v[k] < frac*vmax
end
"After a 4x contact between cars a and b (0 = you) at clock t: the corner rule (see RULE_ON)."
function corner_rule!(a::Int, b::Int, t::Float64)
    (RULE_ON && INC_LIVE[] && INC_ON) || return nothing
    ha = inc_hist_at(a, t); hb = inc_hist_at(b, t); (ha === nothing || hb === nothing) && return nothing
    (behind, ahead, hbh, hah) = ha[2] <= hb[2] ? (a, b, ha, hb) : (b, a, hb, ha)
    rule_zone(hah[2]) || return nothing
    p1 = inc_hist_at(behind, t - 1.0); q1 = inc_hist_at(ahead, t - 1.0); (p1 === nothing || q1 === nothing) && return nothing
    gap1 = q1[2] - p1[2]
    gap1 > RULE_CARL + RULE_GAP || return nothing
    closing = hbh[3] - hah[3]
    closing > RULE_CLOSE || return nothing
    side = hbh[4] - hah[4]
    abs(side) >= RULE_SIDE || return nothing
    -sign(side)*(hbh[4] - p1[4]) >= RULE_DIVE || return nothing
    t - get(RULE_LAST, (min(a, b), max(a, b)), -Inf) < RULE_SEQ_S && return nothing
    RULE_LAST[(min(a, b), max(a, b))] = t
    haskey(DSQ, behind) && return nothing
    RULE_N[] += 1
    BF_DUE[behind] = get(BF_DUE, behind, 0) + 1; BF_DEADLINE[behind] = -1
    println("  [rule] t ", round(t, digits = 1), " ", inc_name(behind), " wedged into ", ahead == 0 ? "your" : string(inc_name(ahead), "'s"), " corner at s ", round(Int, mod(hah[2], RAI_TOTAL[])),
            " (", inc_name(ahead), " ", round(gap1, digits = 1), " m ahead 1 s before, closing ", round(closing, digits = 1),
            " m/s, ", round(abs(side), digits = 1), " m alongside after a ", round(abs(hbh[4] - p1[4]), digits = 1), " m move) -- the black flag: a drive-through within ", BF_LAPS, " laps"); flush(stdout)
    behind == 0 && gearbox_notice!(string("BLACK FLAG -- you wedged into ", inc_name(ahead), "'s corner: drive through the pit lane within ", BF_LAPS, " laps"), 8.0)
    nothing
end
