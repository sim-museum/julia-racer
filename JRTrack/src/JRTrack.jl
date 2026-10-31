"""
    JRTrack

julia racer's track surface queries: the ribbon (`TrackSurface`, `hat`), the mesh height field (`TriangleHAT`,
`hat3d`), and the player's car state and inputs (`CarState`, `DriveInput`, `DriveCar`, `spawn`).
"""
module JRTrack

export TrackSurface, HATResult, hat, Tri, TriangleHAT, hat3d,
       CarState, DriveInput, DriveCar, spawn, survey!

include("surface.jl")
include("trihat.jl")
include("drive.jl")

end
