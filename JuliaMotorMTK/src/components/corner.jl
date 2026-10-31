

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

function Corner(; name,
        m_s = 157.0,
        m_u = 20.0,
        ks  = 26_000.0,
        cs  = 2_500.0,
        kt  = 180_000.0,
        ct  = 300.0,
        Fz_static = 1547.0 + 20.0*9.81)
    ps = @parameters m_s=m_s m_u=m_u ks=ks cs=cs kt=kt ct=ct Fz_static=Fz_static
    vars = @variables zs(t)=0.0 vs(t)=0.0 zu(t)=0.0 vu(t)=0.0 Fz(t) zr(t) Fext(t)

    eqs = [
        D(zs) ~ vs,
        D(zu) ~ vu,
        m_s*D(vs) ~ -ks*(zs - zu) - cs*(vs - vu) + Fext,
        m_u*D(vu) ~  ks*(zs - zu) + cs*(vs - vu) - kt*(zu - zr) - ct*(vu - D(zr)),
        Fz ~ Fz_static + kt*(zr - zu) + ct*(D(zr) - vu),
    ]
    System(eqs, t, vars, ps; name)
end
