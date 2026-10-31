

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

include("tyre_law.jl")
include("brush_tyre.jl")

const TYRE_PC = haskey(ENV, "JM_TYRE_PC")

const TYRE_KMU2 = haskey(ENV, "JM_TYRE_KMU2")
function BrushTyre(; name, μ = BRUSH_FRONT.μ, μx = BRUSH_FRONT.μx, Cα = BRUSH_FRONT.Cα,
                   Cκ = BRUSH_FRONT.Cκ, kμ = BRUSH_FRONT.kμ, Fz0 = BRUSH_FRONT.Fz0, t0 = 0.035, μscale = 1.0,
                   rs = BRUSH_FRONT.rs, ws = BRUSH_FRONT.ws, camber = false, Cγ = CAMBER_CG, kγ = CAMBER_KG,
                   ns = BRUSH_NS, rsy = rs, prel = 0.0, kpμ = 0.0, kpc = 0.0, nsh = 3.0, kin = false, pc = 2.0, kμ2 = 0.0)

    ps = @parameters μ=μ μx=μx Cα=Cα Cκ=Cκ kμ=kμ Fz0=Fz0 t0=t0 μscale=μscale rs=rs ws=ws ns=ns rsy=rsy prel=prel kpμ=kpμ kpc=kpc nsh=nsh
    TYRE_PC && append!(ps, @parameters pc=pc)
    vars = @variables Fz(t) α(t) κ(t) Fy(t) Fx(t) Mz(t) μye(t) μxe(t) ξx(t) ξy(t) ξ(t) sat(t) saty(t)
    TYRE_KMU2 && append!(ps, @parameters kμ2=kμ2)
    lμ = TYRE_KMU2 ? 1.0 - kμ*(Fz/Fz0 - 1.0) + kμ2*(Fz/Fz0 - 1.0)^2 : 1.0 - kμ*(Fz/Fz0 - 1.0)

    lsens = clamp(Fz/Fz0, 0.2, 3.0)^(ns - 1.0) * (1.0 + kpc*prel)
    pmu = 1.0 + kpμ*prel

    if camber
        append!(ps, @parameters Cγ=Cγ kγ=kγ)
        @variables γ(t)
        push!(vars, γ)
        cthr = Cγ*sin(γ)
        cmu  = clamp(1.0 + kγ*sin(γ)*tanh(sin(α)/0.005), 0.7, 1.3)
    else
        cthr = 0.0; cmu = 1.0
    end
    eqs = [
        μye  ~ μscale * pmu * μ  * clamp(lμ, 0.4, 1.6) * cmu,
        μxe  ~ μscale * pmu * μx * clamp(lμ, 0.4, 1.6),

        ξx   ~ lsens*Cκ*κ      / (nsh*μxe),
        ξy   ~ lsens*(Cα*sin(α) + cthr) / (nsh*μye),

        ξ    ~ (TYRE_PC ? ((ξx^2 + 1e-12)^(pc/2) + (ξy^2 + 1e-12)^(pc/2) + 1e-9)^(1/pc) : sqrt(ξx^2 + ξy^2 + 1e-9)),
        sat  ~ (1.0 - (1.0 - min(ξ, 1.0))^nsh) * brush_slide(ξ, rs, ws),

        saty ~ (1.0 - (1.0 - min(ξ, 1.0))^nsh) * brush_slide(ξ, rsy, ws),
        Fx   ~ μxe*Fz * sat * ξx/ξ,
        Fy   ~ μye*Fz * saty * ξy/ξ,

        Mz   ~ -t0 * (1.0 - min(ξ, 1.0)) * (camber ? μye*Fz * saty * (lsens*Cα*sin(α)/(nsh*μye))/ξ : Fy),
    ]
    if kin

        kv = @variables αk(t) Fyk(t) ξyk(t) ξk(t) satyk(t)
        append!(vars, kv)
        append!(eqs, [ξyk ~ lsens*(Cα*sin(αk) + cthr) / (nsh*μye),
                      ξk ~ (TYRE_PC ? ((ξx^2 + 1e-12)^(pc/2) + (ξyk^2 + 1e-12)^(pc/2) + 1e-9)^(1/pc) : sqrt(ξx^2 + ξyk^2 + 1e-9)),
                      satyk ~ (1.0 - (1.0 - min(ξk, 1.0))^nsh) * brush_slide(ξk, rsy, ws),
                      Fyk ~ μye*Fz * satyk * ξyk/ξk])
    end
    System(eqs, t, vars, ps; name)
end

function Tyre(; name,
        Fz0 = TYRE_DEFAULTS.Fz0, μy = TYRE_DEFAULTS.μy, μx = TYRE_DEFAULTS.μx,
        Cy = TYRE_DEFAULTS.Cy, Cx = TYRE_DEFAULTS.Cx, Ey = TYRE_DEFAULTS.Ey,
        Ex = TYRE_DEFAULTS.Ex, pKy1 = TYRE_DEFAULTS.pKy1, pKy2 = TYRE_DEFAULTS.pKy2,
        pKx1 = TYRE_DEFAULTS.pKx1, t0 = TYRE_DEFAULTS.t0, Bt = TYRE_DEFAULTS.Bt)
    ps = @parameters Fz0=Fz0 μy=μy μx=μx Cy=Cy Cx=Cx Ey=Ey Ex=Ex pKy1=pKy1 pKy2=pKy2 pKx1=pKx1 t0=t0 Bt=Bt
    vars = @variables Fz(t) α(t) κ(t) Fy(t) Fx(t) Mz(t) Ky(t) σ(t) Fmag(t)

    eqs = [

        Ky   ~ mf_stiffness(Fz, Fz0, pKy1, pKy2),
        σ    ~ sqrt(κ^2 + sin(α)^2 + 0.02^2),
        Fmag ~ μy*Fz * mf_branch(Ky/(Cy*μy*Fz + 1e-6) * sqrt(κ^2 + sin(α)^2 + 1e-9), Cy, Ey),
        Fx   ~ Fmag * κ / σ,
        Fy   ~ Fmag * sin(α) / σ,
        Mz   ~ -(t0 / (1 + (Bt*α)^2)) * Fy,
    ]
    System(eqs, t, vars, ps; name)
end
