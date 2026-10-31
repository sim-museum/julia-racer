

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

function ClosedLoopVehicle(; name, Kpu = 0.25, Kiu = 0.15, Kpr = 0.7, Kir = 0.5,
                           leak = 0.1, L = 2.41, vehicle = (;))
    @named car = DrivenVehicle(; vehicle...)
    ps = @parameters Kpu=Kpu Kiu=Kiu Kpr=Kpr Kir=Kir leak=leak L=L
    vars = @variables uref(t) rref(t) gear_in(t) eu(t) er(t) eui(t)=0.0 eri(t)=0.0 pedal(t) steer(t)
    eqs = [
        eu ~ uref - car.u,
        er ~ rref - car.r,
        D(eui) ~ eu - leak*eui,
        D(eri) ~ er - leak*eri,
        pedal ~ Kpu*eu + Kiu*eui,
        steer ~ rref*L/(car.u + 1.0) + Kpr*er + Kir*eri,
        car.throttle ~ min(1.0, max(0.0,  pedal)),
        car.brake    ~ min(1.0, max(0.0, -pedal)),
        car.δ        ~ steer,
        car.gear     ~ gear_in,
    ]
    System(eqs, t, vars, ps; systems = [car], name)
end
