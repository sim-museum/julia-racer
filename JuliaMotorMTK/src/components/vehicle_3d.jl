

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

smoothpos(x, ε) = 0.5*(x + sqrt(x^2 + ε^2))

const ETA_DRIVE = parse(Float64, get(ENV, "JM_ETA_DRIVE", "0.9"))
slip_bodyframe() = get(ENV, "JM_SLIP_BODYFRAME", "0") != "0"
ηdir(T, ηc, ηd) = ηc + (ηd - ηc)*(0.5 + 0.5*tanh(T/5.0))

function DrivenVehicle3D(; name,
        m = 617.0, Izz = nothing, Ixx = 120.0, Iyy = 850.0, iscale = (1.0, 1.0, 1.0),
        a = 1.314, b = 1.096, tf = 1.50, tr = 1.50, h = 0.30, front_frac = 0.455,
        Rw_f = 0.30, Rw_r = 0.334, Iw = 1.0, η = 0.9, final = 4.11,

        bias = 0.585, Tbrake_max = 2956.0,

        brake_bias_p = 0.535,

        travel = nothing, k_stop = 5e5, k_stop_reb = 2e4,

        relax = nothing,

        front_hubs = false,

        tank_x = 0.5,
        CdA = CDA_IBT, ρair = 1.10, g = 9.80665,
        throttle0 = 0.0, brake0 = 0.0, steer0 = 0.0, gear0 = 1.72, brush = false,

        front_corner = (ks = 18_250.0, cs = 2500.0, m_s = 120.0, m_u = 20.0, kt = 180_000.0, ct = 1000.0),
        rear_corner  = (ks = 29_200.0, cs = 3000.0, m_s = 148.0, m_u = 20.0, kt = 200_000.0, ct = 1100.0),

        fl_corner = front_corner, fr_corner = front_corner,
        rl_corner = rear_corner,  rr_corner = rear_corner,

        diff = nothing, toe_f = 0.0, toe_r = 0.0, karb_f = 0.0, karb_r = 0.0, camber = nothing, rc = CAMBER_RC)
    L = a + b; mf = m*front_frac; mr = m*(1 - front_frac)
    M_s = fl_corner.m_s + fr_corner.m_s + rl_corner.m_s + rr_corner.m_s

    cmb = brush && camber !== nothing

    bf = !cmb ? BRUSH_FRONT : USE_TYRE3 ? merge(BRUSH_FRONT, (μ = TYRE3.μf*_GRIP, Cα = TYRE3.Cαf, kμ = TYRE3.kμ)) :
                                          merge(BRUSH_FRONT, (μ = TYRE2.μf*_GRIP, Cα = TYRE2.Cαf))
    br = !cmb ? BRUSH_REAR  : USE_TYRE3 ? merge(BRUSH_REAR,  (μ = TYRE3.μr*_GRIP, Cα = TYRE3.Cαr, kμ = TYRE3.kμ)) :
                                          merge(BRUSH_REAR,  (μ = TYRE2.μr*_GRIP, Cα = TYRE2.Cαr))
    nshf = cmb && USE_TYRE3 ? TYRE3.nshf : 3.0; nshr = cmb && USE_TYRE3 ? TYRE3.nshr : 3.0
    nsrv = cmb && USE_TYRE3 ? TYRE3.nsr : BRUSH_NS
    claf = cmb && USE_TYRE3 && IRG_S20 ? AERO_CLAF : 0.0
    clar = cmb && USE_TYRE3 && IRG_S20 ? AERO_CLAR : 0.0

    front_hubs = front_hubs && cmb && USE_TYRE3

    Izz = Izz !== nothing ? Izz : cmb && USE_TYRE3 ? 1040.0 : 890.0

    iscale == (1.0, 1.0, 1.0) || (Izz *= iscale[1]; Ixx *= iscale[2]; Iyy *= iscale[3])
    kinF = brush && relax !== nothing && COLUMN_KIN
    FL = brush ? BrushTyre(; name=:FL, bf..., camber=cmb, nsh=nshf, kin=kinF) : Tyre(; name=:FL, TYRE_SKIDPAD_FRONT...)
    FR = brush ? BrushTyre(; name=:FR, bf..., camber=cmb, nsh=nshf, kin=kinF) : Tyre(; name=:FR, TYRE_SKIDPAD_FRONT...)
    RL = brush ? BrushTyre(; name=:RL, br...,  camber=cmb, nsh=nshr, ns=nsrv) : Tyre(; name=:RL, TYRE_SKIDPAD_REAR...)
    RR = brush ? BrushTyre(; name=:RR, br...,  camber=cmb, nsh=nshr, ns=nsrv) : Tyre(; name=:RR, TYRE_SKIDPAD_REAR...)

    gv, kfv, krv = g, karb_f, karb_r
    ps = @parameters m=m Izz=Izz Ixx=Ixx Iyy=Iyy a=a b=b tf=tf tr=tr h=h mf=mf mr=mr L=L M_s=M_s g=g Rw_f=Rw_f Rw_r=Rw_r Iw=Iw η=η final=final bias=bias Tbrake_max=Tbrake_max CdA=CdA ρair=ρair throttle=throttle0 brake=brake0 δ=steer0 gear=gear0 clutch=0.0 hrc_f=0.0 hrc_r=0.0 ClAf=claf ClAf_rh=AERO_CLAF_RH ClAr=clar antistall=0.0 as_r0=1957.0 as_k=495.0 as_w=490.0 Ie=(ENGINE_LUMPED ? 0.18 : IE_ENGINE) Igb=(ENGINE_LUMPED ? 0.0 : DRIVELINE_INPUT ? 0.18 - IE_ENGINE : IE_DRIVELINE) c_c=60.0 T_cap=500.0 k_idle=0.5 idle_rpm=2000.0 zrFL=0.0 zrFR=0.0 zrRL=0.0 zrRR=0.0 vrFL=0.0 vrFR=0.0 vrRL=0.0 vrRR=0.0 Fx_ext=0.0 Fy_ext=0.0 Mz_ext=0.0 CdA_scale=1.0 c_abl=(cmb && USE_TYRE3 ? TYRE3.cabl : C_ABL) toe_f=toe_f toe_r=toe_r karb_f=karb_f karb_r=karb_r η_d=ETA_DRIVE
    brk = BrakeSystem(; name = :brk, bias_p = brake_bias_p)
    tank = FuelTank(; name = :tank, x = tank_x)
    if cmb
        append!(ps, @parameters camFL=camber[1] camFR=camber[2] camRL=camber[3] camRR=camber[4] rc_f=rc[1] rc_r=rc[2])
    end
    lsd = diff !== nothing
    if lsd
        lsd_ps = @parameters lsd_pre=diff[1] lsd_cotd=cotd(diff[2]) lsd_cotc=cotd(diff[3]) lsd_plates=diff[4] lsd_k=LSD_K lsd_weps=LSD_WEPS
        append!(ps, lsd_ps)
    end

    if lsd; @variables ωr(t); else; @variables ωr(t)=0.0; end
    vplane = [@variables(u(t)=0.0, v(t)=0.0, r(t)=0.0, ωf(t)=0.0)...; ωr; (front_hubs ? @variables(ωfd(t)=0.0) : [])...; @variables ωe(t)=209.4 ωRL(t)=0.0 ωRR(t)=0.0 Tlsd(t) ay(t) ax(t) az(t) rpm(t) X(t)=0.0 Y(t)=0.0 ψ(t)=0.0]

    vatt = @variables z(t)=0.0 w(t)=0.0 th(t)=0.0 q(t)=0.0 ph(t)=0.0 pp(t)=0.0
    vfz  = @variables FzFL(t) FzFR(t) FzRL(t) FzRR(t)
    lsd || (vplane = filter(x -> !any(isequal(x), (ωRL, ωRR, Tlsd)), vplane))
    vars = vcat(vplane, vatt, vfz)

    gr = gear*final; drag = 0.5*ρair*CdA*CdA_scale*u*abs(u)

    Laf = 0.5*ρair*(ClAf + ClAf_rh*(z + a*th))*u^2; Lar = 0.5*ρair*ClAr*u^2

    mc = m - tank.m_burn
    rr = CRR_IBT*mc*g*tanh(u/0.12)
    εF = 80.0

    spec = ((FL,  a,  tf/2,  δ - toe_f, :f, fl_corner, zrFL, vrFL, FzFL, :FL),
            (FR,  a, -tf/2,  δ + toe_f, :f, fr_corner, zrFR, vrFR, FzFR, :FR),
            (RL, -b,  tr/2,     -toe_r, :r, rl_corner, zrRL, vrRL, FzRL, :RL),
            (RR, -b, -tr/2,      toe_r, :r, rr_corner, zrRR, vrRR, FzRR, :RR))

    karb(c) = hasproperty(c, :karb) ? c.karb : 0.0
    mount, road, wheel, strut, tyrev, arb, stops, rcl = quiet_flanges() do
    mount = [PrescribedMotion(; name = Symbol(:m, s[10])) for s in spec]
    road  = [PrescribedMotion(; name = Symbol(:r, s[10])) for s in spec]
    wheel = [WheelMass(; name = Symbol(:w, s[10]), m_u = s[6].m_u, g = gv) for s in spec]
    strut = [CoilOver(; name = Symbol(:s, s[10]), ks = s[6].ks, P = s[6].m_s*gv, cs = s[6].cs,
                      cb = hasproperty(s[6], :cb) ? s[6].cb : NaN, cr = hasproperty(s[6], :cr) ? s[6].cr : NaN,
                      clo = hasproperty(s[6], :clo) ? s[6].clo : 1.0, vk = hasproperty(s[6], :vk) ? s[6].vk : 0.10, ε = εF)
             for s in spec]
    tyrev = [TyreVertical(; name = Symbol(:tv, s[10]), kt = s[6].kt, ct = s[6].ct, Fz_static = (s[6].m_s + s[6].m_u)*gv, ε = εF)
             for s in spec]

    arb = [AntiRollBar(; name = :arbF, k = 0.5*(karb(fl_corner) + karb(fr_corner)) + kfv),
           AntiRollBar(; name = :arbR, k = 0.5*(karb(rl_corner) + karb(rr_corner)) + krv)]
    rcl = [RollCentreLink(; name = Symbol(:rc, s[10])) for s in spec]
    stops = travel === nothing ? nothing :
        [TravelStops(; name = Symbol(:ts, s[10]), c_bump = travel[i][1], c_reb = travel[i][2], k = k_stop, kr = k_stop_reb) for (i, s) in enumerate(spec)]
    (mount, road, wheel, strut, tyrev, arb, stops, rcl)
    end

    rlx = relax === nothing ? nothing :
        [TyreRelaxation(; name = Symbol(:rx, k), σ = (k in ("FL", "FR") ? relax[1] : relax[2])) for k in ("FL", "FR", "RL", "RR")]
    eqs = Equation[]; Fyb=Any[]; Fxb=Any[]; Mz=Any[]; Fx_f=Any[]; Fx_r=Any[]; Pslip=Any[]
    Fsusp=Any[]; xs=Any[]; ys=Any[]
    for (idx, (ty, xi, yi, st, axle, cor, zr, vr, Fz, key)) in enumerate(spec)
        pidx = isodd(idx) ? idx + 1 : idx - 1
        bar = arb[axle == :f ? 1 : 2]; left = isodd(idx)
        Rw  = axle == :f ? Rw_f : Rw_r
        ωax = axle == :f ? (!front_hubs ? ωf : idx == 1 ? ωf + ωfd : ωf - ωfd) : !lsd ? ωr : idx == 3 ? ωRL : ωRR
        mt, rd, wh, so, tv = mount[idx], road[idx], wheel[idx], strut[idx], tyrev[idx]
        zu = wh.zu
        append!(eqs, [
            mt.s ~ z + xi*th + yi*ph,
            mt.v ~ w + xi*q  + yi*pp,
            rd.s ~ zr, rd.v ~ vr,
            (stops === nothing ? connect(mt.fl, so.a, left ? bar.mL : bar.mR, rcl[idx].a) :
                                 connect(mt.fl, so.a, left ? bar.mL : bar.mR, stops[idx].a, rcl[idx].a)),
            (stops === nothing ? connect(so.b, wh.fl, tv.a, left ? bar.wL : bar.wR, rcl[idx].b) :
                                 connect(so.b, wh.fl, tv.a, left ? bar.wL : bar.wR, stops[idx].b, rcl[idx].b)),
            connect(tv.b, rd.fl),
            so.Fbar ~ (left ? bar.FL : bar.FR),
            Fz ~ tv.Fz])
        push!(Fsusp, mt.f); push!(xs, xi); push!(ys, yi)

        vx = u - r*yi;  vy = v + r*xi
        Vref = sqrt(vx^2 + 1.0)

        if slip_bodyframe()
            α = st - atan(vy, Vref)
        else
            vxw = vx*cos(st) + vy*sin(st);  vyw = vy*cos(st) - vx*sin(st)
            α = -atan(vyw, sqrt(vxw^2 + 1.0))
        end
        κ = (ωax*Rw - vx)/Vref
        if rlx === nothing
            append!(eqs, [ty.Fz ~ Fz, ty.α ~ α, ty.κ ~ κ])
        else
            append!(eqs, [ty.Fz ~ Fz, rlx[idx].α_in ~ α, rlx[idx].V ~ Vref, ty.α ~ rlx[idx].α, ty.κ ~ κ])
            kinF && idx <= 2 && push!(eqs, ty.αk ~ α)
        end
        if cmb

            camst = (camFL, -camFR, camRL, -camRR)[idx]
            zuo = wheel[pidx].zu; trk = axle == :f ? tf : tr
            φs = ph - (isodd(idx) ? zu - zuo : zuo - zu)/trk
            push!(eqs, ty.γ ~ camst - (axle == :f ? rc_f : rc_r)*φs)
        end
        fxb = ty.Fx*cos(st) - ty.Fy*sin(st)
        fyb = ty.Fx*sin(st) + ty.Fy*cos(st)
        push!(Fxb, fxb); push!(Fyb, fyb); push!(Mz, ty.Mz)
        push!(Pslip, sqrt((ty.Fy*sin(α))^2 + 1.0))
        axle == :f ? push!(Fx_f, ty.Fx) : push!(Fx_r, ty.Fx)
    end

    append!(eqs, [rcl[1].F ~ -(Fyb[1] + Fyb[2])*hrc_f/tf, rcl[2].F ~ (Fyb[1] + Fyb[2])*hrc_f/tf,
                  rcl[3].F ~ -(Fyb[3] + Fyb[4])*hrc_r/tr, rcl[4].F ~ (Fyb[3] + Fyb[4])*hrc_r/tr])

    abl = c_abl*(Pslip[1] + Pslip[2] + Pslip[3] + Pslip[4] - 4.0)*tanh(u/0.12)
    ΣFx = Fxb[1]+Fxb[2]+Fxb[3]+Fxb[4];  ΣFy = Fyb[1]+Fyb[2]+Fyb[3]+Fyb[4]
    ΣFs = Fsusp[1]+Fsusp[2]+Fsusp[3]+Fsusp[4]

    ωgb = ωr*gr

    epedal = (1.0 - clutch) * clamp(abs(gear)/0.5, 0.0, 1.0)
    emax   = clamp((ωe*60/(2π) - (as_r0 - as_k*throttle))/as_w, 0.0, 1.0)
    engage = epedal - antistall*max(0.0, epedal - emax)
    clu = Clutch(; name = :clu)
    append!(eqs, [clu.engage ~ engage, clu.slip ~ ωe - ωgb])
    Tcl   = clu.T

    dens  = ENGINE_RHO ? ρair/RHO_WOT : 1.0
    Teng  = ENGINE_LUMPED ? engine_torque(rpm, throttle; dens) : engine_torque_free(rpm, throttle; dens)
    Tgd   = ENGINE_LUMPED ? 0.0 : engage*gearbox_drag(ωgb*60/(2π), throttle)
    Tidle = clamp(k_idle*max(0.0, idle_rpm - rpm), 0.0, 120.0) * (1.0 - engage)
    run   = clamp((rpm - 300.0)/150.0, 0.0, 1.0)

    push!(eqs,
        rpm ~ ωe*60/(2π),

        ax ~ (ΣFx - drag - rr - abl + Fx_ext)/mc,
        ay ~ (ΣFy + Fy_ext)/mc,
        mc*(D(u) - v*r) ~ ΣFx - drag - rr - abl + Fx_ext,
        mc*(D(v) + u*r) ~ ΣFy + Fy_ext,
        Izz*D(r) ~ a*(Fyb[1]+Fyb[2]) - b*(Fyb[3]+Fyb[4])
                   - tf/2*(Fxb[1]-Fxb[2]) - tr/2*(Fxb[3]-Fxb[4]) + Mz[1]+Mz[2]+Mz[3]+Mz[4] + Mz_ext,

        D(z) ~ w,
        (M_s - tank.m_burn)*D(w) ~ ΣFs - (M_s - tank.m_burn)*g - Laf - Lar,
        az ~ D(w) + g,
        D(th) ~ q,
        Iyy*D(q) ~ (xs[1]*Fsusp[1]+xs[2]*Fsusp[2]+xs[3]*Fsusp[3]+xs[4]*Fsusp[4]) + h*ΣFx + tank.m_burn*g*tank.x - a*Laf + b*Lar,
        D(ph) ~ pp,
        Ixx*D(pp) ~ (ys[1]*Fsusp[1]+ys[2]*Fsusp[2]+ys[3]*Fsusp[3]+ys[4]*Fsusp[4]) + h*ΣFy,

        Ie*D(ωe) ~ (Teng + Tidle)*run - (1.0 - run)*45.0*ωe - Tcl,
        (front_hubs ? 2*Iw*D(ωf) ~ -brk.TF/2*(tanh(ωf + ωfd) + tanh(ωf - ωfd)) - (Fx_f[1]+Fx_f[2])*Rw_f :
                      2*Iw*D(ωf) ~ -brk.TF*tanh(ωf) - (Fx_f[1]+Fx_f[2])*Rw_f),

        D(X) ~ u*cos(ψ) - v*sin(ψ),
        D(Y) ~ u*sin(ψ) + v*cos(ψ),
        D(ψ) ~ r,
    )
    front_hubs && push!(eqs, 2*Iw*D(ωfd) ~ -brk.TF/2*(tanh(ωf + ωfd) - tanh(ωf - ωfd)) - (Fx_f[1]-Fx_f[2])*Rw_f)
    if !lsd
        push!(eqs, (2*Iw + Igb*(DRIVELINE_INPUT ? gr : final)^2*η)*D(ωr) ~ (Tcl - Tgd)*gr*ηdir(Tcl - Tgd, η, η_d) - brk.TR*tanh(ωr) - (Fx_r[1]+Fx_r[2])*Rw_r)
    else

        Tin  = (Tcl - Tgd)*gr*ηdir(Tcl - Tgd, η, η_d)
        Igr  = Igb*(DRIVELINE_INPUT ? gr : final)^2*η/4
        RLt  = Tin/2 + Tlsd/2 - brk.TR/2*tanh(ωRL) - Fx_r[1]*Rw_r
        RRt  = Tin/2 - Tlsd/2 - brk.TR/2*tanh(ωRR) - Fx_r[2]*Rw_r
        wdr  = 0.5*(1 + tanh(Tin/10.0))
        Tcap = lsd_pre + lsd_k*lsd_plates*(wdr*lsd_cotd + (1 - wdr)*lsd_cotc)*sqrt(Tin^2 + 1.0)
        append!(eqs, [
            Tlsd ~ Tcap*tanh((ωRR - ωRL)/lsd_weps),
            D(ωRL) ~ ((Iw + Igr)*RLt - Igr*RRt)/(Iw*(Iw + 2*Igr)),
            D(ωRR) ~ ((Iw + Igr)*RRt - Igr*RLt)/(Iw*(Iw + 2*Igr)),
            ωr ~ (ωRL + ωRR)/2,
        ])
    end

    col = kinF ? SteeringColumn(; name = :col, trail = COLUMN_TRAIL[1], tp = COLUMN_TRAIL[2]) : SteeringColumn(; name = :col)
    push!(eqs, col.Fy ~ (kinF ? FL.Fyk + FR.Fyk : FL.Fy + FR.Fy))
    push!(eqs, col.ξ ~ (kinF ? 0.5*(FL.ξk + FR.ξk) : brush ? 0.5*(FL.ξ + FR.ξ) : 1.0))
    append!(eqs, [brk.pedal ~ brake,
                  brk.kF ~ Tbrake_max*bias/(brk.P_full*0.535), brk.kR ~ Tbrake_max*(1 - bias)/(brk.P_full*(1 - 0.535))])
    System(eqs, t, vars, ps; systems = [FL, FR, RL, RR, mount..., road..., wheel..., strut..., tyrev..., arb..., col, brk, clu, tank, (stops === nothing ? [] : stops)...,
                                        (rlx === nothing ? [] : rlx)..., rcl...], name)
end
