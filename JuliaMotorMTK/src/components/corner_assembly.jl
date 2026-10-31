

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D

function CornerAssembly(; name, corner = (;), tyre = (;), brush = false, brushp = (;))
    @named corner = Corner(; corner...)

    ty = brush ? BrushTyre(; name = :tyre, brushp...) : Tyre(; name = :tyre, tyre...)
    eqs = [ty.Fz ~ corner.Fz]
    System(eqs, t, Num[], []; systems = [corner, ty], name)
end
