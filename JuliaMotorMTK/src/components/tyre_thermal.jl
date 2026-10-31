

"Grip multiplier vs surface temperature (peaks at Topt)."
mu_temp(T; Topt = 90.0, kT = 0.45) = max(0.30, 1 - kT*((T - Topt)/100)^2)

"Grip multiplier vs hot pressure [kPa] (peaks at pref)."
mu_press(pk; pref = 152.0, kp = 0.6) = max(0.70, 1 - kp*((pk - pref)/pref)^2)

"Surface-temperature rate [°C/s]: friction-power heating vs convective cooling."
tyre_temp_rate(T, Q, v; Csurf = 600.0, h = 40.0, kv = 0.06, Tamb = 30.0) =
    (Q - h*(1 + kv*v)*(T - Tamb)) / Csurf

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

function ThermalTyre(; name, T0 = 60.0, γ = 0.0, pk = 152.0,
        Fz0 = TYRE_SKIDPAD_FRONT.Fz0, μy = TYRE_SKIDPAD_FRONT.μy, μx = TYRE_SKIDPAD_FRONT.μx,
        Cy = TYRE_SKIDPAD_FRONT.Cy, Cx = TYRE_SKIDPAD_FRONT.Cx, Ey = TYRE_SKIDPAD_FRONT.Ey,
        Ex = TYRE_SKIDPAD_FRONT.Ex, pKy1 = TYRE_SKIDPAD_FRONT.pKy1, pKy2 = TYRE_SKIDPAD_FRONT.pKy2,
        pKx1 = TYRE_SKIDPAD_FRONT.pKx1, t0 = TYRE_SKIDPAD_FRONT.t0, Bt = TYRE_SKIDPAD_FRONT.Bt,
        Cγ = 3.0, Topt = 90.0, kT = 0.45, pref = 152.0, kp = 0.6,
        Csurf = 600.0, hcool = 40.0, kv = 0.06, Tamb = 30.0)
    ps = @parameters Fz0=Fz0 μy=μy μx=μx Cy=Cy Cx=Cx Ey=Ey Ex=Ex pKy1=pKy1 pKy2=pKy2 pKx1=pKx1 t0=t0 Bt=Bt γ=γ pk=pk Cγ=Cγ Topt=Topt kT=kT pref=pref kp=kp Csurf=Csurf hcool=hcool kv=kv Tamb=Tamb
    vars = @variables Fz(t) α(t) κ(t) v(t) Fy(t) Fx(t) Mz(t) T(t)=T0 gT(t) gP(t) μye(t) μxe(t) Fy0(t) Fx0(t) gc(t) Q(t) Ky(t)
    eqs = [
        gT  ~ max(0.30, 1 - kT*((T - Topt)/100)^2),
        gP  ~ max(0.70, 1 - kp*((pk - pref)/pref)^2),
        μye ~ μy*gT*gP,  μxe ~ μx*gT*gP,
        Ky  ~ mf_stiffness(Fz, Fz0, pKy1, pKy2),
        Fy0 ~ μye*Fz*mf_branch(Ky/(Cy*μye*Fz + 1e-6)*α, Cy, Ey) + Fz*Cγ*γ,
        Fx0 ~ μxe*Fz*mf_branch((pKx1*Fz)/(Cx*μxe*Fz + 1e-6)*κ, Cx, Ex),
        gc  ~ min(1.0, 1.0/sqrt((Fx0/(μxe*Fz + 1e-6))^2 + (Fy0/(μye*Fz + 1e-6))^2 + 1e-9)),
        Fx  ~ gc*Fx0,  Fy ~ gc*Fy0,
        Mz  ~ -(t0/(1 + (Bt*α)^2))*Fy,
        Q   ~ abs(Fx*κ + Fy*sin(α))*v,
        D(T) ~ (Q - hcool*(1 + kv*v)*(T - Tamb))/Csurf,
    ]
    System(eqs, t, vars, ps; name)
end
