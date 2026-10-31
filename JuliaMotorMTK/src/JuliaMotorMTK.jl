module JuliaMotorMTK

include("ibt.jl");          using .IBT
include("setup.jl");        using .Setup
include("corner_loads.jl"); using .CornerLoads

export IBT, Setup, CornerLoads

end
