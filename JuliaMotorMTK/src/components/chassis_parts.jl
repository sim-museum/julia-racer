

using ModelingToolkit
const Logging = Base.CoreLogging
using ModelingToolkit: t_nounits as t, D_nounits as D

@connector function Flange(; name)
    vars = @variables s(t) v(t) f(t) [connect = Flow]
    System(Equation[], t, vars, []; name)
end

struct _FlangeCheckFilter <: Logging.AbstractLogger
    inner::Logging.AbstractLogger
end
Logging.min_enabled_level(l::_FlangeCheckFilter) = Logging.min_enabled_level(l.inner)
Logging.shouldlog(l::_FlangeCheckFilter, args...) = Logging.shouldlog(l.inner, args...)
Logging.catch_exceptions(l::_FlangeCheckFilter) = Logging.catch_exceptions(l.inner)
function Logging.handle_message(l::_FlangeCheckFilter, level, msg, args...; kw...)
    occursin("flow variables, yet", string(msg)) && return nothing
    Logging.handle_message(l.inner, level, msg, args...; kw...)
end
"""Run `f()` -- building a System that connects flanges, or compiling one -- without MTK's false connector-balance
warning (see above)."""
quiet_flanges(f) = Logging.with_logger(f, _FlangeCheckFilter(Logging.current_logger()))

"""Prescribed motion (as Modelica's Translational.Sources.Position): a flange that moves as told -- the sprung body's
point above a wheel, or the road under it. `s`, `v` are set by the owner; `f` is the force the parts exert on it (on the
body: the suspension force at that mount)."""
function PrescribedMotion(; name)
    @named fl = Flange()
    vars = @variables s(t) v(t) f(t)
    System([fl.s ~ s, fl.v ~ v, f ~ fl.f], t, vars, []; systems = [fl], name)
end

"""Unsprung mass (wheel, hub, upright, brake): heave only, under gravity and the forces at its flange."""
function WheelMass(; name, m_u, g = 9.80665)
    @named fl = Flange()
    ps = @parameters m_u=m_u g=g
    vars = @variables zu(t)=0.0 [state_priority = 10] vu(t)=0.0 [state_priority = 10]
    System([fl.s ~ zu, fl.v ~ vu, D(zu) ~ vu, m_u*D(vu) ~ fl.f - m_u*g], t, vars, ps; systems = [fl], name)
end

"""Coil-over: the coil SPRING (rate `ks` at the wheel, static preload `P` = the corner's sprung weight) and the DAMPER
(bump `cb` / rebound `cr`, blended through zero velocity; one symmetric `cs` when cb/cr are NaN) on one strut.
The damper may be PROGRESSIVE (IRGOLD-1 S9): the rate × `clo` at low stroke speed, rising smoothly to the full rate above
~`vk` m/s (clo = 1 = linear). Transient roll strokes the shocks a few cm/s, a crest tens of cm/s -- a linear damper
cannot give both the gold's roll timing and its crest.
The strut can only push: the SEAT clamps the sum smoothly at zero (rounding `ε` N). That clamp stands in for the
strut's travel limits (droop) until they are modelled from the ibt's ShockDeflection; `Fbar` is the anti-roll bar's
force at this wheel, which today acts through the same seat (as it did in the single block -- see AntiRollBar)."""
function CoilOver(; name, ks, P, cs, cb = NaN, cr = NaN, clo = 1.0, vk = 0.10, ε = 80.0)
    @named a = Flange(); @named b = Flange()
    ps = @parameters ks=ks P=P ε=ε
    asym = isfinite(cb) && isfinite(cr)
    append!(ps, asym ? @parameters(cb=cb, cr=cr) : @parameters(cs=cs))
    append!(ps, @parameters(clo=clo, vk=vk))
    vars = @variables c(t) vrel(t) Fk(t) Fd(t) Fbar(t) F(t)
    cdmp = (asym ? cr + (cb - cr)*0.5*(1 + tanh(vrel/0.01)) : cs)*(clo + (1 - clo)*vrel^2/(vrel^2 + vk^2))
    eqs = [c ~ b.s - a.s, vrel ~ b.v - a.v,
           Fk ~ P + ks*c,
           Fd ~ cdmp*vrel,
           F ~ smoothpos(Fk + Fd + Fbar, ε),
           a.f ~ -F, b.f ~ F]
    System(eqs, t, vars, ps; systems = [a, b], name)
end

"""Anti-roll bar of one axle: a torsion bar linking the left and right wheels, stiff in roll and free in heave and
pitch. It reads each side's suspension compression from the mount (`mL`, `mR`) and wheel (`wL`, `wR`) flanges and
pushes each wheel down (and its mount up) by k·(c_this − c_other): `k` is the bar's roll stiffness at the wheel [N/m].
Its forces reach the car through each corner's coil-over (`FL`, `FR` → CoilOver.Fbar), so its sensing flanges carry
no force of their own."""
function AntiRollBar(; name, k)
    @named mL = Flange(); @named wL = Flange(); @named mR = Flange(); @named wR = Flange()
    ps = @parameters k=k
    vars = @variables cL(t) cR(t) FL(t) FR(t)
    eqs = [cL ~ wL.s - mL.s, cR ~ wR.s - mR.s,
           FL ~ k*(cL - cR), FR ~ k*(cR - cL),
           mL.f ~ 0, wL.f ~ 0, mR.f ~ 0, wR.f ~ 0]
    System(eqs, t, vars, ps; systems = [mL, wL, mR, wR], name)
end

"""Clutch (CARPHYS-1 S9): a dry plate clutch between the engine and the gearbox input. `engage` is the pedal's
engagement (0 out, 1 in), `slip` the speed difference engine − gearbox input [rad/s]. Below its capacity it couples
the two through the stiffness `c_c` (N·m per rad/s); slipping, it transmits its capacity, which the gold measures
(powertrain.jl CLUTCH_KNOTS: a bite point near 0.15, 407 N·m at 0.65)."""
function Clutch(; name, c_c = 60.0)
    ps = @parameters c_c=c_c
    vars = @variables engage(t) slip(t) cap(t) T(t)
    System([cap ~ clutch_capacity(engage), T ~ clamp(c_c*slip, -cap, cap)], t, vars, ps; name)
end

"""Tyre relaxation (CARPHYS-1 S11): a tyre does not make its lateral force from the kinematic slip angle at once -- the
carcass and tread deflect over a rolling distance σ before the contact patch reaches its new slip. First order in
distance: σ·dα/ds = α_in − α, with ds = V·dt (V the wheel's rolling speed, kept ≥ 1 m/s by the caller). `α_in` is
the kinematic slip; `α` the transient slip the tyre's force law sees."""
function TyreRelaxation(; name, σ = 0.4)
    ps = @parameters σ=σ
    vars = @variables α_in(t) α(t)=0.0 V(t)
    System([D(α) ~ V/σ*(α_in - α)], t, vars, ps; name)
end

"""Shock travel limits (CARPHYS-1 S7): the bump stop at full compression and the shock's own extension limit at full
droop, between the body mount (`a`) and the wheel (`b`), beside the coil-over. `c_bump` > 0 and `c_reb` < 0 are the
wheel travel from static to each end (the setup's ShockDeflection static/max and Packer, through the motion ratio).
Unlike the coil-over's seat this element PULLS at full droop -- the extended shock carries the hanging wheel, which is
what stops a wheel dropping without limit in the air (the gold: the front shocks bottom out at ~0 mm deflection, the
wheel −64 mm; before S7 the sim's wheels fell 150–210 mm over Flugplatz). Force: `k` per metre past the stop, with a
quadratic onset over `w` (continuous force and slope)."""
function TravelStops(; name, c_bump, c_reb, k = 5e5, kr = k, w = 0.002)
    @named a = Flange(); @named b = Flange()
    ps = @parameters c_bump=c_bump c_reb=c_reb k=k kr=kr w=w
    vars = @variables c(t) F(t)
    pen(x) = ifelse(x <= 0, 0.0, ifelse(x < w, x^2/(2w), x - w/2))
    System([c ~ b.s - a.s, F ~ k*pen(c - c_bump) - kr*pen(c_reb - c), a.f ~ -F, b.f ~ F], t, vars, ps;
           systems = [a, b], name)
end

"""Roll-centre link (IRGOLD-1 S18): the suspension LINKS' share of an axle's lateral load transfer. A wishbone axle reacts
its tyres' lateral force F_y partly through the links, at the roll centre's height h_rc: the links then move load from
the inside to the outside wheel directly, ΔF = F_y·h_rc/track, without rolling the body, and the springs and bar see
only (h − h_rc)·F_y of roll moment. Between a corner's body mount (`a`) and its wheel (`b`), like the coil-over, but
unlike its seat it can pull (it is a rigid link): `F` > 0 pushes the wheel down and the mount up (the outside wheel).
`F` is an input -- the vehicle sets ±F_y,axle·h_rc/track from its own tyre forces."""
function RollCentreLink(; name)
    @named a = Flange(); @named b = Flange()
    vars = @variables F(t)
    System([a.f ~ -F, b.f ~ F], t, vars, []; systems = [a, b], name)
end

"""Brake system (CARPHYS-1 S6): master cylinder, bias valve and calipers, as reference's Lotus 49 records them.
The pedal makes `P_full` bar of line pressure at full travel, split front/rear by the garage's BrakeBias `bias_p`
(the gold: LF 65.4 bar at full pedal with 53.5 %, 66.0 with 54 % -- 122.2 bar × bias; the pressure share is the
setting to the third decimal). Each axle's calipers and discs turn pressure into torque, `kF`/`kR` [N·m per bar,
axle]: inputs here, set by the vehicle from its fitted brake (BRAKE-2: 2956 N·m and a 0.585 torque split at full
pedal with 53.5 %), so a different bias moves the split AND the total the way the hardware does. `pF`/`pR` are the
.ibt's brakeLinePress channels; `TF`/`TR` the axle torques."""
function BrakeSystem(; name, bias_p = 0.535, P_full = 122.2)
    ps = @parameters bias_p=bias_p P_full=P_full
    vars = @variables pedal(t) kF(t) kR(t) pF(t) pR(t) TF(t) TR(t)
    System([pF ~ P_full*bias_p*pedal, pR ~ P_full*(1 - bias_p)*pedal, TF ~ kF*pF, TR ~ kR*pR], t, vars, ps; name)
end

"""Steering column (CARPHYS-1 S5): the torque the front tyres send up the column to the rim -- the force feedback.
`Fy` is the front axle's lateral force (the two tyres, N), `τ` the rim torque (N·m, positive turns the rim left).
Fitted to reference's SteeringWheelTorque (tools/steerfit_261009.jl, 19 gold skidpad events through the session car):
the rim torque is the front force on one effective trail -- 4.78 cm at the 10:1 rack (R² 0.982, 1.8 N·m RMS, the
same across both setups). A separate aligning-moment term (the brush Mz) and a centring term on the wheel angle add
nothing on the gold, so the column carries none: the rim goes light near the limit because the front FORCE does."""

function SteeringColumn(; name, ratio = 10.0, trail = 0.0478, tp = 0.0)
    ps = @parameters ratio=ratio trail=trail tp=tp
    vars = @variables Fy(t) τ(t) ξ(t)
    System([τ ~ -(trail + tp*(1.0 - min(ξ, 1.0)))*Fy/ratio], t, vars, ps; name)
end
const COLUMN_TRAIL = (0.0450, 0.0155)

"""Tyre, vertical: the carcass as a spring `kt` and damper `ct` between the wheel (`a`) and the road (`b`), loaded to
`Fz_static` at rest. It can only push -- a wheel off the ground carries nothing (rounding `ε` N). `Fz` is the load
the tyre's force law (BrushTyre) works with."""
function TyreVertical(; name, kt, ct, Fz_static, ε = 80.0)
    @named a = Flange(); @named b = Flange()
    ps = @parameters kt=kt ct=ct Fz_static=Fz_static ε=ε
    vars = @variables Fz(t)
    eqs = [Fz ~ smoothpos(Fz_static + kt*(b.s - a.s) + ct*(b.v - a.v), ε), a.f ~ -Fz, b.f ~ Fz]
    System(eqs, t, vars, ps; systems = [a, b], name)
end

"""IRGOLD-1 / FUEL-1: the FUEL TANK. The car is built with the session's fuel already in its corner weights (mass, CG,
corner masses); as the engine burns fuel, `m_burn` [kg] -- a runtime parameter the game advances from the measured burn
law -- leaves the car at the tank's position `x` [m ahead of the CG]: the in-plane mass, the sprung mass the springs carry
(the body rises as it burns, as a real car does) and a pitch moment (the front, where the tank's weight mostly sits,
rises more). Where the tank sits is measured: in the PO's 261011 fuel test the extra fuel's weight went 66 % to the front
axle at 20.5 → 43.2 L and at 145.4 → 179.5 L alike, i.e. 0.82 m behind the front axle (FUEL_TANK_X_FRONT)."""
function FuelTank(; name, x = 0.5)
    ps = @parameters m_burn = 0.0 x = x
    vars = @variables dm(t)
    System([dm ~ m_burn], t, vars, ps; name)
end
