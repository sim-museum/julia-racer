

module DriveRT

using ModelingToolkit, OrdinaryDiffEq
using ModelingToolkit: t_nounits as t, D_nounits as D
const setp = ModelingToolkit.setp

const HERE = @__DIR__
for f in ("tyre.jl","corner.jl","corner_assembly.jl","powertrain.jl","vehicle_rt.jl")
    include(joinpath(HERE, "components", f))
end

export Car, build_car, step_car!, respawn!, telemetry, contain!

const GEARS    = [2.23, 1.72, 1.32, 1.09, 0.916]
gearratio(g::Int) = g <= 0 ? 0.0 : GEARS[g]
const FINAL    = 4.11
const MAXSTEER = 0.30
const RW_R     = 0.334

const TC_ON    = get(ENV, "JM_TC", "0") == "1" && !haskey(ENV, "JM_NOTC")
const TC_SLIP  = parse(Float64, get(ENV, "JM_TC_SLIP", "0.06"))

const TC_VLO   = parse(Float64, get(ENV, "JM_TC_VLO", "25.0"))
const TC_VHI   = parse(Float64, get(ENV, "JM_TC_VHI", "38.0"))

mutable struct Car
    sys; integ
    s_thr; s_brk; s_st; s_gr; s_clu
    s_we
    s_pos; s_vel
    getall
    gear::Int

    x::Float64; y::Float64; z::Float64; θ::Float64
    v::Float64; t::Float64; rpm::Float64; gear_n::Int
    tc::NTuple{4,NTuple{3,Float64}}
    lapdist::Float64; laps::Int; lateral::Float64; along::Float64; ontrack::Bool
end

"Build the real-time car, spawned at world (x0,z0) heading θ0.  v0=0 ⇒ standing
start (engine idling, clutch open; throttle revs + engages it to launch)."
function build_car(; x0 = 0.0, z0 = 0.0, θ0 = 0.0, v0 = 0.0, y0 = 0.0, brush = !haskey(ENV, "JM_MAGIC"))
    sys = mtkcompile(DrivenVehicleRT(name = :car, brush = brush))
    println(brush ? "  TYRE: physics-based brush model (default; μ≈1.4 peak, no fudge — JM_GRIP scales) — set JM_MAGIC=1 for the old tyre" :
                    "  TYRE: Magic-Formula tyre (JM_MAGIC)")
    prob = ODEProblem(sys, [sys.u => v0, sys.ωf => v0/0.30, sys.ωr => v0/RW_R,
                            sys.ωe => 209.4, sys.X => x0, sys.Y => z0, sys.ψ => θ0], (0.0, 1e7))

    integ = init(prob, Rosenbrock23(); save_everystep = false, dense = false,
                 adaptive = false, dt = 1/300)

    getall = ModelingToolkit.getsym(sys, [sys.X, sys.Y, sys.ψ, sys.u, sys.v, sys.rpm,
        sys.FL.tyre.Fx, sys.FR.tyre.Fx, sys.RL.tyre.Fx, sys.RR.tyre.Fx,
        sys.FL.tyre.Fy, sys.FR.tyre.Fy, sys.RL.tyre.Fy, sys.RR.tyre.Fy, sys.ωr])
    c = Car(sys, integ, setp(sys, sys.throttle), setp(sys, sys.brake), setp(sys, sys.δ),
            setp(sys, sys.gear), setp(sys, sys.clutch), ModelingToolkit.setu(sys, sys.ωe),
            ModelingToolkit.setu(sys, [sys.X, sys.Y]), ModelingToolkit.setu(sys, [sys.u, sys.v]), getall, 1,
            x0, 0.0, z0, θ0, v0, 0.0, 0.0, 1,
            ntuple(_ -> (0.0,0.0,0.0), 4), 0.0, 0, 0.0, 0.0, true)
    c.s_gr(c.integ, GEARS[c.gear]);  getall(integ)
    for _ in 1:3; step_car!(c, 0.3, 0.0, 0.0, 1/60); end
    for _ in 1:3; step_car!(c, 0.3, 0.0, 0.0, 1/60; clutch = 0.5, manual = true); end
    reinit!(c.integ); c.gear = 0; c.s_gr(c.integ, 0.0)
    a = getall(c.integ)
    c.x = a[1]; c.z = a[2]; c.θ = a[3]; c.v = sqrt(a[4]^2 + a[5]^2); c.rpm = a[6]
    c
end

"""Compile the planar car ONCE and build `length(poses)` cars sharing the system, so a
field of AI doesn't pay N× `mtkcompile`.  `poses` = Vector of (x0,z0,θ0,v0).  Each car gets
its own integrator (independent state) but shares the compiled setters/getter."""
function build_cars(poses; brush = !haskey(ENV, "JM_MAGIC"))
    sys = mtkcompile(DrivenVehicleRT(name = :car, brush = brush))
    s_thr = setp(sys, sys.throttle); s_brk = setp(sys, sys.brake); s_st = setp(sys, sys.δ)
    s_gr  = setp(sys, sys.gear);     s_clu = setp(sys, sys.clutch); s_we = ModelingToolkit.setu(sys, sys.ωe)
    s_pos = ModelingToolkit.setu(sys, [sys.X, sys.Y]); s_vel = ModelingToolkit.setu(sys, [sys.u, sys.v])
    getall = ModelingToolkit.getsym(sys, [sys.X, sys.Y, sys.ψ, sys.u, sys.v, sys.rpm,
        sys.FL.tyre.Fx, sys.FR.tyre.Fx, sys.RL.tyre.Fx, sys.RR.tyre.Fx,
        sys.FL.tyre.Fy, sys.FR.tyre.Fy, sys.RL.tyre.Fy, sys.RR.tyre.Fy, sys.ωr])
    cars = Car[]
    for (x0, z0, θ0, v0) in poses
        prob = ODEProblem(sys, [sys.u => v0, sys.ωf => v0/0.30, sys.ωr => v0/RW_R,
                                sys.ωe => 209.4, sys.X => x0, sys.Y => z0, sys.ψ => θ0], (0.0, 1e7))
        integ = init(prob, Rosenbrock23(); save_everystep = false, dense = false, adaptive = false, dt = 1/300)
        c = Car(sys, integ, s_thr, s_brk, s_st, s_gr, s_clu, s_we, s_pos, s_vel, getall, 1,
                x0, 0.0, z0, θ0, v0, 0.0, 0.0, 1, ntuple(_ -> (0.0,0.0,0.0), 4), 0.0, 0, 0.0, 0.0, true)
        c.s_gr(c.integ, GEARS[c.gear]); getall(integ)
        for _ in 1:3; step_car!(c, 0.3, 0.0, 0.0, 1/60); end
        for _ in 1:3; step_car!(c, 0.3, 0.0, 0.0, 1/60; clutch = 0.5, manual = true); end
        reinit!(c.integ); c.gear = 0; c.s_gr(c.integ, 0.0)
        a = getall(c.integ); c.x = a[1]; c.z = a[2]; c.θ = a[3]; c.v = sqrt(a[4]^2 + a[5]^2); c.rpm = a[6]
        push!(cars, c)
    end
    cars
end

"Advance the car by dt.  throttle/brake/steer ∈ [0,1]/[0,1]/[-1,1]; `clutch` ∈ [0,1]
(0 engaged, 1 pressed); `up`/`dn` are one-frame shift events; `manual=true` ⇒ the
driver works the clutch + gears, else the adapter auto-clutches and auto-shifts."
function step_car!(c::Car, throttle, brake, steer, dt;
                   clutch = 0.0, up = false, dn = false, manual = false,
                   groundz = (x,z)->0.0, lock = 1.0)
    held = abs(c.v) < 1.0 && throttle < 0.05
    if manual
        c.s_clu(c.integ, clamp(clutch, 0, 1))
        up && c.gear < 5 && (c.gear += 1; c.s_gr(c.integ, gearratio(c.gear)))
        dn && c.gear > 0 && (c.gear -= 1; c.s_gr(c.integ, gearratio(c.gear)))
    else

        if c.gear == 0

            g = 1
            while g < 5 && abs(c.v) > 0.9 * (9500.0*2π/60) / (GEARS[g]*FINAL) * RW_R; g += 1; end
            c.gear = g; c.s_gr(c.integ, GEARS[g])
        end
        ae = held ? 0.0 : clamp((c.rpm - 1400.0)/1000.0, 0, 1) * clamp(max(2*throttle, c.v/2), 0, 1)
        c.s_clu(c.integ, clamp(1.0 - ae, 0, 1))
    end

    thr = clamp(throttle, 0, 1)
    if TC_ON && thr > 0.0
        a0 = c.getall(c.integ)
        gate = clamp((abs(a0[4]) - TC_VLO) / (TC_VHI - TC_VLO), 0.0, 1.0)
        if gate > 0.0
            κr = (a0[15]*RW_R - a0[4]) / max(abs(a0[4]), 3.0)
            κr > TC_SLIP && (thr *= clamp(1.0 - 4.0*gate*(κr - TC_SLIP)/TC_SLIP, 0.05, 1.0))
        end
    end
    c.s_thr(c.integ, thr)
    c.s_brk(c.integ, clamp(brake, 0, 1))
    c.s_st(c.integ, clamp(steer, -1, 1) * MAXSTEER)
    step!(c.integ, max(dt, 1e-3), true)
    a = c.getall(c.integ)
    c.x = a[1];  c.z = a[2];  c.θ = a[3]
    c.v = sqrt(a[4]^2 + a[5]^2);  c.t = c.integ.t
    c.rpm = clamp(a[6], 0.0, 9700.0);  c.gear_n = c.gear

    if c.rpm < 350.0 && (clamp(clutch, 0, 1) > 0.5 || c.gear == 0)
        c.s_we(c.integ, 209.44); c.rpm = 2000.0
    end
    c.y = groundz(c.x, c.z)
    mg4 = 617.0*9.80665/4

    grip = (1.36*1376.0/mg4, 1.36*1376.0/mg4, 1.40*1650.0/mg4, 1.40*1650.0/mg4)
    c.tc = ntuple(i -> (a[6+i]/mg4, a[10+i]/mg4, grip[i]), 4)
    (!isfinite(c.v) || abs(c.v) > 110) && return respawn!(c)
    if !manual && c.gear >= 1
        grpm = (a[4]/RW_R)*GEARS[c.gear]*FINAL*60/(2π)
        if grpm > 8500 && c.gear < 5;                       c.gear += 1; c.s_gr(c.integ, GEARS[c.gear])
        elseif grpm < 3400 && c.gear > 1 && throttle < 0.9; c.gear -= 1; c.s_gr(c.integ, GEARS[c.gear]); end
    end
    c
end

"Extra physics observables for telemetry export (.ibt), beyond the per-frame
render fields: body velocities u (fwd), v (lat), yaw rate r, long/lat accel, and
front/rear wheel speeds.  The getter is compiled once per system and cached, so
this stays cheap when polled every tick.  Returns a NamedTuple."
const _TELCACHE = IdDict{Any,Any}()
function telemetry(c::Car)
    g = get!(_TELCACHE, c.sys) do
        ModelingToolkit.getsym(c.sys, [c.sys.u, c.sys.v, c.sys.r, c.sys.ax, c.sys.ay, c.sys.ωf, c.sys.ωr])
    end
    a = g(c.integ)
    (u = a[1], v = a[2], r = a[3], ax = a[4], ay = a[5], ωf = a[6], ωr = a[7])
end

"Snap the car onto the track boundary (xnew, znew) and bleed its speed — a
fence/hedge collision that keeps the car inside the game world (E7)."
function contain!(c::Car, xnew, znew; vdamp = 0.45, settle = false, groundz = nothing)
    a = c.getall(c.integ)
    c.s_pos(c.integ, [xnew, znew])
    c.s_vel(c.integ, [a[4]*vdamp, a[5]*vdamp])
    c.x = xnew; c.z = znew; c.v = sqrt((a[4]*vdamp)^2 + (a[5]*vdamp)^2)
    c
end

"Reset the car to its spawn position/state (R key)."
function respawn!(c::Car)

    g0 = get(ENV,"JM_SPAWN_IN_GEAR","0") != "0" ? 1 : 0
    reinit!(c.integ); c.gear = g0; c.s_gr(c.integ, gearratio(g0))
    a = c.getall(c.integ); c.x = a[1]; c.z = a[2]; c.θ = a[3]; c.v = a[4]; c.rpm = a[6]
    c
end

const _RSET = IdDict()
"Current yaw rate r [rad/s] (cached getter) — for the AI controller's yaw damping."
function yawrate(c::Car)
    gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET, c.sys)
    gs[1](c.integ)
end

const _PSI = IdDict()
"Place the car at world (x,z) facing θ with forward speed v (e.g. onto a grid slot)."
function place!(c::Car, x, z, θ; v = 0.0)
    c.s_pos(c.integ, [x, z])
    try
        pset = get!(() -> ModelingToolkit.setu(c.sys, c.sys.ψ), _PSI, c.sys)
        pset(c.integ, θ)
    catch; end
    c.s_vel(c.integ, [v, 0.0])
    c.x = x; c.z = z; c.θ = θ; c.v = v
    c
end
"Apply a rigid-body collision impulse to the car: a world-frame velocity change
`(dvx, dvz)` plus a yaw-rate change `dr` (rad/s) — so a hit knocks the car off line,
scrubs speed, and spins it, all through the real vehicle state."
function bump!(c::Car, dvx, dvz, dr)
    a = c.getall(c.integ); θ = a[3]; u = a[4]; v = a[5]
    wvx = u*cos(θ) - v*sin(θ) + dvx
    wvz = u*sin(θ) + v*cos(θ) + dvz
    nu =  wvx*cos(θ) + wvz*sin(θ)
    nv = -wvx*sin(θ) + wvz*cos(θ)
    c.s_vel(c.integ, [nu, nv])
    if dr != 0.0
        try
            gs = get!(() -> (ModelingToolkit.getsym(c.sys, c.sys.r), ModelingToolkit.setu(c.sys, c.sys.r)), _RSET, c.sys)
            gs[2](c.integ, gs[1](c.integ) + dr)
        catch; end
    end
    c.v = sqrt(nu^2 + nv^2)
    c
end

end
