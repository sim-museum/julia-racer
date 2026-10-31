

module CLASSICLP
export read_lp, lp_speed_mps, CLASSIC_TICK_HZ

const CLASSIC_TICK_HZ = 36.0

struct LPLine
    v_tick::Vector{Float32}
    dlat_v::Vector{Float32}
    dlat::Vector{Float32}
    f4::Vector{Float32}
    f5::Vector{Float32}
end

function read_lp(path::AbstractString)
    b = read(path)
    length(b) >= 8 || error("read_lp: $path too short")
    n = Int(reinterpret(UInt32, b[5:8])[1])
    length(b) == 8 + 20n || error("read_lp: $path is $(length(b)) bytes, expected 8 + 20×$n")
    f = reinterpret(Float32, b[9:end])
    LPLine(f[1:5:end], f[2:5:end], f[3:5:end], f[4:5:end], f[5:5:end])
end

"Speed in m/s at every record."
lp_speed_mps(l::LPLine) = Float64.(l.v_tick) .* CLASSIC_TICK_HZ

end
