

module JRRender

import JRPhysics
const NAT = normpath(joinpath(@__DIR__, "..", "..", "demo", "native"))
include(joinpath(NAT, "render.jl"));    using .Render
include(joinpath(NAT, "classictrack.jl"));  using .CLASSICTrack
include(joinpath(NAT, "classicwall.jl"));   using .CLASSICWall
include(joinpath(NAT, "roadcurve.jl")); using .RoadCurve
export Render, CLASSICTrack, CLASSICWall, RoadCurve

using PrecompileTools
@compile_workload begin

    for st in eachline(joinpath(@__DIR__, "workload.jl"))
        startswith(st, "precompile(") || continue
        try; Core.eval(@__MODULE__, Meta.parse(st)); catch; end
    end
end
end
