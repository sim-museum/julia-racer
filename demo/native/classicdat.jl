

module CLASSICDat

export parse_dat

dat_legacy() = get(ENV, "JM_DAT_LEGACY", "0") != "0"
dat_layout() = dat_legacy() ? "legacy" : "fixed27"

"""Parse a Classic .dat archive → Dict(lowercase filename => file bytes)."""
function parse_dat(path::AbstractString)
    b = read(path)
    u32(o) = UInt32(b[o+1]) | (UInt32(b[o+2])<<8) | (UInt32(b[o+3])<<16) | (UInt32(b[o+4])<<24)
    count = Int(UInt32(b[1]) | (UInt32(b[2])<<8))
    names = String[]; offs = Int[]
    pos = 12
    if dat_legacy()
        for _ in 1:count
            s = pos
            while pos < length(b) && b[pos+1] != 0x00; pos += 1; end
            push!(names, String(b[s+1:pos])); pos += 1
            while pos < length(b) && b[pos+1] == 0x00; pos += 1; end
            push!(offs, Int(u32(pos))); pos += 14
        end
    else
    for _ in 1:count
        pos + 27 <= length(b) || break
        nb = b[pos+1:pos+13]; z = findfirst(==(0x00), nb)
        push!(names, String(nb[1:(z === nothing ? 13 : z - 1)]))
        push!(offs, Int(u32(pos + 13)))
        pos += 27
    end
    end

    order = sortperm(offs)
    sizes = Dict{Int,Int}()
    for (i, idx) in enumerate(order)
        nxt = i < length(order) ? offs[order[i+1]] : length(b)
        sizes[idx] = nxt - offs[idx]
    end
    out = Dict{String,Vector{UInt8}}()
    for i in 1:length(names)
        o = offs[i]; sz = sizes[i]
        (o >= 0 && sz > 0 && o + sz <= length(b)) || continue
        out[lowercase(names[i])] = b[o+1 : o+sz]
    end
    out
end

end
