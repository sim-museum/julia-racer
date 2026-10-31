

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

function DrivenVehicle(; name,
        m = 617.0, Izz = 890.0, a = 1.314, b = 1.096, tf = 1.50, tr = 1.50,
        h = 0.30, front_frac = 0.455,
        Rw_f = 0.30, Rw_r = 0.33, Iw = 1.0, Ieng = 0.10, η = 0.9, final = 4.11,
        bias = 0.535, Tbrake_max = 3000.0, CdA = 0.9, ρair = 1.10,
        front_corner = (Fz_static = 1376.0, ks = 18_250.0, cs = 2500.0,
                        m_s = 120.0, m_u = 20.0, kt = 180_000.0, ct = 300.0),
        rear_corner  = (Fz_static = 1650.0, ks = 29_200.0, cs = 3000.0,
                        m_s = 148.0, m_u = 20.0, kt = 200_000.0, ct = 300.0))
    L = a + b; mf = m*front_frac; mr = m*(1 - front_frac)
    @named FL = CornerAssembly(corner = front_corner, tyre = TYRE_SKIDPAD_FRONT)
    @named FR = CornerAssembly(corner = front_corner, tyre = TYRE_SKIDPAD_FRONT)
    @named RL = CornerAssembly(corner = rear_corner,  tyre = TYRE_SKIDPAD_REAR)
    @named RR = CornerAssembly(corner = rear_corner,  tyre = TYRE_SKIDPAD_REAR)

    ps = @parameters m=m Izz=Izz a=a b=b tf=tf tr=tr h=h mf=mf mr=mr L=L Rw_f=Rw_f Rw_r=Rw_r Iw=Iw Ieng=Ieng η=η final=final bias=bias Tbrake_max=Tbrake_max CdA=CdA ρair=ρair
    vars = @variables u(t)=25.0 v(t)=0.0 r(t)=0.0 ωf(t)=83.3 ωr(t)=75.8 δ(t) throttle(t) brake(t) gear(t) ay(t) ax(t) β(t) rpm(t)

    gr = gear*final
    drag = 0.5*ρair*CdA*u^2

    spec = ((FL, a,  tf/2, δ, -1, -1, mf, tf, Rw_f, :f),
            (FR, a, -tf/2, δ, +1, -1, mf, tf, Rw_f, :f),
            (RL,-b,  tr/2, 0, -1, +1, mr, tr, Rw_r, :r),
            (RR,-b, -tr/2, 0, +1, +1, mr, tr, Rw_r, :r))
    eqs = Equation[]
    Fyb = Any[]; Fxb = Any[]; Mz = Any[]; Fx_f = Any[]; Fx_r = Any[]
    for (ca, xi, yi, st, slat, slong, maxle, trk, Rw, axle) in spec
        vx = u - r*yi;  vy = v + r*xi
        α  = st - atan(vy, vx)
        ωax = axle == :f ? ωf : ωr
        κ  = (ωax*Rw - vx)/(vx + 0.5)
        fxb = ca.tyre.Fx*cos(st) - ca.tyre.Fy*sin(st)
        fyb = ca.tyre.Fx*sin(st) + ca.tyre.Fy*cos(st)
        push!(Fyb, fyb); push!(Fxb, fxb); push!(Mz, ca.tyre.Mz)
        axle == :f ? push!(Fx_f, ca.tyre.Fx) : push!(Fx_r, ca.tyre.Fx)
        ΔFz = slat*maxle*ay*h/trk + slong*m*ax*h/(2L)
        append!(eqs, [ca.tyre.α ~ α, ca.tyre.κ ~ κ, ca.corner.zr ~ 0.0, ca.corner.Fext ~ -ΔFz])
    end
    ΣFx = Fxb[1]+Fxb[2]+Fxb[3]+Fxb[4];  ΣFy = Fyb[1]+Fyb[2]+Fyb[3]+Fyb[4]
    push!(eqs,
        rpm ~ ωr*gr*60/(2π),
        ax ~ (ΣFx - drag)/m,
        ay ~ ΣFy/m,
        β  ~ atan(v, u),

        m*(D(u) - v*r) ~ ΣFx - drag,
        m*(D(v) + u*r) ~ ΣFy,
        Izz*D(r) ~ a*(Fyb[1]+Fyb[2]) - b*(Fyb[3]+Fyb[4])
                   - tf/2*(Fxb[1]-Fxb[2]) - tr/2*(Fxb[3]-Fxb[4]) + Mz[1]+Mz[2]+Mz[3]+Mz[4],

        2*Iw*D(ωf) ~ -brake*Tbrake_max*bias*tanh(ωf) - (Fx_f[1]+Fx_f[2])*Rw_f,
        (2*Iw + Ieng*gr^2)*D(ωr) ~ engine_torque(rpm, throttle)*gr*η
                                   - brake*Tbrake_max*(1-bias)*tanh(ωr) - (Fx_r[1]+Fx_r[2])*Rw_r,
    )
    System(eqs, t, vars, ps; systems = [FL, FR, RL, RR], name)
end
