

"""
Live state of a human-driven car in world coordinates.

`x, z` are the world-plane position (m); `θ` the heading of the forward axis
(rad, forward = (cos θ, sin θ) in X–Z); `v` forward speed (m/s); `β` sideslip
(rad); `r` yaw rate (rad/s).  `y`, `ontrack`, `lapdist` are filled from the
track surface each step (diagnostics / lap timing).
"""
mutable struct CarState
    x::Float64
    z::Float64
    θ::Float64
    v::Float64
    β::Float64
    r::Float64
    gear::Int
    fuel::Float64
    rpm::Float64
    along::Float64
    y::Float64
    ontrack::Bool
    lateral::Float64
    lapdist::Float64
    t::Float64
    laps::Int

    tc::NTuple{4,NTuple{3,Float64}}
end

"""Driver inputs for one step: pedals in [0,1], `steer` in [-1,1] (fraction of
the steering lock, + = left), and discrete shift requests."""
Base.@kwdef struct DriveInput
    throttle::Float64 = 0.0
    brake::Float64 = 0.0
    steer::Float64 = 0.0
    clutch::Float64 = 0.0
    shift_up::Bool = false
    shift_down::Bool = false
    autoshift::Bool = true
end

"""A car the human drives: the physics model plus its fixed setup constants
(steering lock, the track surfaces for height/grade and lap timing)."""
struct DriveCar
    model::Any
    ts::TrackSurface
    terrain::Union{Nothing,TriangleHAT}
    max_steer::Float64
    v_couple::Float64
end

function DriveCar(model, ts::TrackSurface;
                  terrain::Union{Nothing,TriangleHAT}=nothing,
                  max_steer::Real=0.35, v_couple::Real=8.0)
    DriveCar(model, ts, terrain, Float64(max_steer), Float64(v_couple))
end

"""
    spawn(car; fuel=40.0, waypoint=1) -> CarState

Place the car at rest on the racing line at `waypoint`, heading along the
track tangent there.  Used at startup and on respawn after a crash/off.
"""
function spawn(car::DriveCar; fuel::Real=40.0, waypoint::Integer=1, v0::Real=0.0)
    ts = car.ts
    n = length(ts.pos)
    i = mod1(waypoint, n)
    j = mod1(i + 1, n)
    p = ts.pos[i]
    θ = atan(ts.pos[j][3] - p[3], ts.pos[j][1] - p[1])
    cs = CarState(p[1], p[3], θ, Float64(v0), 0.0, 0.0, 1, Float64(fuel),
                  0.0, 0.0, p[2], true, 0.0, ts.lapdist[i], 0.0, 0,
                  ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0),
                   (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)))
    survey!(cs, car)
    cs
end

function survey!(cs::CarState, car::DriveCar)
    h = hat(car.ts, cs.x, cs.z)
    if h.found
        cs.ontrack = h.on_track
        cs.lateral = h.lateral
        cs.lapdist = h.lapdist
    end
    if car.terrain !== nothing
        y, _, found = hat3d(car.terrain, cs.x, cs.z; ref=cs.y + 2.0)
        found && (cs.y = y)
    elseif h.found
        cs.y = h.height
    end
    cs
end

function travel_grade(car::DriveCar, x, z, θ, yref)
    car.terrain === nothing && return 0.0
    tx, tz = cos(θ), sin(θ)
    h1, _, f1 = hat3d(car.terrain, x - 3tx, z - 3tz; ref=yref + 2.0)
    h2, _, f2 = hat3d(car.terrain, x + 3tx, z + 3tz; ref=yref + 2.0)
    (f1 && f2) ? (h2 - h1) / 6.0 : 0.0
end
