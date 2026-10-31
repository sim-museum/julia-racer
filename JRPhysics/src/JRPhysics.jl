

module JRPhysics

using SampledSignals
const MTKSRC = normpath(joinpath(@__DIR__, "..", "..", "JuliaMotorMTK", "src"))
include(joinpath(MTKSRC, "drive_rt.jl"))
include(joinpath(MTKSRC, "drive_rt3d.jl"))
export DriveRT, DriveRT3D

using PrecompileTools
@setup_workload begin
    flat(x, z) = 0.0
    @compile_workload begin
        c = DriveRT3D.build_car3d(x0 = 0.0, z0 = 0.0, θ0 = 0.0, v0 = 0.0, y0 = 0.0)
        for i in 1:30; DriveRT3D.step_car3d!(c, 0.6, 0.0, 0.05, 1/60; manual = false, groundz = flat); end
        DriveRT3D.telemetry3d(c)
        field = DriveRT3D.build_cars3d([(0.0, 10.0, 0.0, 0.0), (0.0, 20.0, 0.0, 0.0)])
        for f in field, i in 1:10; DriveRT3D.step_car3d!(f, 0.5, 0.0, 0.0, 1/60; manual = false, groundz = flat); end
        DriveRT3D.damage_reset!()

        DriveRT3D.set_chassis!(DriveRT3D.Chassis(diff = (41.0, 75.0, 60.0, 4.0), toe = (-0.0017, 0.0015), karb = (1121.0, 12714.0),
                                                 camber = (-0.0087, -0.0070, -0.0070, -0.0087)))
        cs = DriveRT3D.build_car3d(x0 = 0.0, z0 = 0.0, θ0 = 0.0, v0 = 0.0, y0 = 0.0)
        for i in 1:30; DriveRT3D.step_car3d!(cs, 0.6, 0.0, 0.05, 1/60; manual = false, groundz = flat); end
        DriveRT3D.telemetry3d(cs)
        DriveRT3D.set_chassis!(DriveRT3D.Chassis())
    end
end
end
