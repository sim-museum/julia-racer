

module IBT

export IBTFile, channel, channels, ibt_open, session_yaml, session_space, write_ibt

const VARTYPE = Dict(
    0 => (Int8,    1),
    1 => (Bool,    1),
    2 => (Int32,   4),
    3 => (UInt32,  4),
    4 => (Float32, 4),
    5 => (Float64, 8),
)

struct VarInfo
    type::Int
    offset::Int
    count::Int
    name::String
    desc::String
    unit::String
end

struct IBTFile
    path::String
    tickRate::Int
    bufLen::Int
    dataOffset::Int
    nrows::Int
    vars::Dict{String,VarInfo}
    yaml::String
    raw::Vector{UInt8}
end

cstr(b) = String(b[1:something(findfirst(==(0x00), b), length(b)+1)-1])

"""    ibt_open(path) -> IBTFile

Parse the header + var table + YAML of an .ibt file (loads the whole file into
memory; these are tens of MB).  Channel samples are read lazily via `channel`.
"""
function ibt_open(path::AbstractString)
    raw = read(path)
    gi(o) = reinterpret(Int32, raw[o+1:o+4])[1] |> Int
    tickRate         = gi(8)
    sessionInfoLen   = gi(16)
    sessionInfoOffset= gi(20)
    numVars          = gi(24)
    varHeaderOffset  = gi(28)
    bufLen           = gi(36)
    dataOffset       = gi(48 + 4)

    vars = Dict{String,VarInfo}()
    for i in 0:numVars-1
        b = varHeaderOffset + i*144
        vtype  = gi(b)
        voff   = gi(b+4)
        vcount = gi(b+8)
        name = cstr(raw[b+16+1 : b+16+32])
        desc = cstr(raw[b+48+1 : b+48+64])
        unit = cstr(raw[b+112+1: b+112+32])
        vars[name] = VarInfo(vtype, voff, vcount, name, desc, unit)
    end
    yaml = String(raw[sessionInfoOffset+1 : sessionInfoOffset+sessionInfoLen])
    nrows = bufLen > 0 ? (length(raw) - dataOffset) ÷ bufLen : 0
    IBTFile(String(path), tickRate, bufLen, dataOffset, nrows, vars, yaml, raw)
end

channels(f::IBTFile) = sort!(collect(keys(f.vars)))
session_yaml(f::IBTFile) = f.yaml
"""Bytes the file reserves for its session YAML (a replacement written by write_ibt must fit them)."""
session_space(f::IBTFile) = Int(reinterpret(Int32, f.raw[17:20])[1])

"""    channel(f, name; idx=1) -> Vector

Full time series of a scalar (or `idx`-th element of an array) channel, native
type converted to Float64.  Sampled at `f.tickRate` Hz.
"""
function channel(f::IBTFile, name::AbstractString; idx::Int=1)
    v = f.vars[name]
    T, sz = VARTYPE[v.type]
    out = Vector{Float64}(undef, f.nrows)
    base = f.dataOffset + v.offset + (idx-1)*sz
    @inbounds for r in 0:f.nrows-1
        o = base + r*f.bufLen
        out[r+1] = Float64(reinterpret(T, f.raw[o+1:o+sz])[1])
    end
    out
end

function _putval!(buf::Vector{UInt8}, off::Int, vtype::Int, val::Real)
    T, sz = VARTYPE[vtype]
    x = T === Bool ? (val > 0.5) : T <: Integer ? round(T, clamp(val, Float64(typemin(T)), Float64(typemax(T)))) : T(val)
    b = reinterpret(UInt8, [x])
    @inbounds for k in 1:sz; buf[off+k] = b[k]; end
end

"""    write_ibt(out, template::IBTFile, samples) -> out

Write an reference-compatible .ibt at `out`, reusing `template`'s header + variable
table + session YAML verbatim (so any reference tool / our own reader parses it
identically), and filling the data buffer from `samples` — a vector of
`Dict{String,<:Real}` mapping reference channel names (e.g. "Speed", "RPM",
"LongAccel") to values, one dict per 1/`tickRate`-second tick.  Channels absent
from a sample are written as zero.  This lets juliaMotor emit telemetry in the
exact format reference produces, for side-by-side lap comparison / model tuning.
"""
function write_ibt(out::AbstractString, tmpl::IBTFile, samples::AbstractVector; yaml::Union{Nothing,AbstractString} = nothing)
    hdr = copy(tmpl.raw[1:tmpl.dataOffset])
    if yaml !== nothing

        len = Int(reinterpret(Int32, tmpl.raw[17:20])[1]); off = Int(reinterpret(Int32, tmpl.raw[21:24])[1])
        b = Vector{UInt8}(codeunits(yaml))
        if length(b) <= len && off + len <= length(hdr)
            hdr[off+1:off+len] .= 0x00; hdr[off+1:off+length(b)] .= b
        else
            @warn "write_ibt: the replacement YAML ($(length(b)) bytes) does not fit the template's $(len); kept the template's"
        end
    end
    n = length(samples)
    hdr[48+1:48+4] = reinterpret(UInt8, Int32[n])
    open(out, "w") do io
        write(io, hdr)
        row = Vector{UInt8}(undef, tmpl.bufLen)
        for s in samples
            fill!(row, 0x00)
            for (name, val) in s
                v = get(tmpl.vars, name, nothing); v === nothing && continue
                _putval!(row, v.offset, v.type, Float64(val))
            end
            write(io, row)
        end
    end
    out
end

end
