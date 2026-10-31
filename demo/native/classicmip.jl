

module CLASSICMip

export decode_mip, decode_mip_bytes, decode_srb_bytes

@inline u32(b,o) = (o+4 > length(b)) ? UInt32(0) :
    UInt32(b[o+1]) | (UInt32(b[o+2])<<8) | (UInt32(b[o+3])<<16) | (UInt32(b[o+4])<<24)
tag(b,o) = String(b[o+1:o+4])

function sections(b, start)
    secs = Tuple{String,Int,Int}[]
    o = start
    while o + 12 <= length(b)
        t = tag(b,o); sz = Int(u32(b,o+8)); data = o + 12
        push!(secs, (t, data, sz))
        o = data + sz; o += (4 - o % 4) % 4
    end
    secs
end

"""Decode a Classic .srb (sprite) -> the embedded MIP as (w,h,rgba).  An SRB is a small
size header + an embedded standard MIP file ("PIM "…)."""
function decode_srb_bytes(b::AbstractVector{UInt8})
    p = findfirst([0x50,0x49,0x4d,0x20], b)
    p === nothing && error("no embedded MIP in SRB")
    decode_mip_bytes(b[first(p):end])
end

"""Decode a .mip file -> (width, height, rgba::Vector{UInt8}) of the largest level."""
decode_mip(path::AbstractString) = decode_mip_bytes(read(path))
"Palette byte order this decoder assumes; part of the decoded-texture cache key so a change here
cannot be served stale pixels from disk."
const CMAP_ORDER = "bgra-v2"

function decode_mip_bytes(b::AbstractVector{UInt8})
    tag(b,0) == "PIM " || error("not a MIP")

    tag(b,12) == "DHPM" || error("no MPHD")
    mp = 24
    mtype = Int(b[mp+1])
    powW  = Int(u32(b, mp+1)); powH = Int(u32(b, mp+5))
    W = 1 << powW; H = 1 << powH

    function read_cmap(off, sz)
        n = sz ÷ 4
        [(b[off+i*4+3], b[off+i*4+2], b[off+i*4+1], b[off+i*4+4]) for i in 0:n-1]
    end

    secs = sections(b, 12)
    cmap = NTuple{4,UInt8}[]
    bmaps = Int[]
    bmsizes = Int[]
    for (t, off, sz) in secs
        if     t == "PAMC"; cmap = read_cmap(off, sz)
        elseif t == "PAMB"; push!(bmaps, off); push!(bmsizes, sz)
        end
    end
    isempty(bmaps) && error("no BMAP in $path")

    b0 = bmaps[1]; bsz = bmsizes[1]
    subs = sections(b, b0 - 12)

    px_off = 0; px_sz = 0; local_cmap = cmap
    o = b0
    stop = b0 + bsz
    while o + 12 <= stop
        t = tag(b,o); sz = Int(u32(b,o+8)); data = o + 12
        if     t == "ATAD"; px_off = data; px_sz = sz
        elseif t == "PAMC"; local_cmap = read_cmap(data, sz)
        end
        o = data + sz; o += (4 - o % 4) % 4
    end
    px_off == 0 && error("no DATA in first BMAP of $path")

    rgba = Vector{UInt8}(undef, W*H*4)
    pal = local_cmap
    npal = length(pal)
    bytesper = px_sz / (W*H)
    @inline function put(i, idx)
        c = (idx >= 0 && idx < npal) ? pal[idx+1] : (0xff,0x00,0xff,0xff)
        rgba[i*4+1]=c[1]; rgba[i*4+2]=c[2]; rgba[i*4+3]=c[3]; rgba[i*4+4]=c[4]
    end
    if npal > 0 && bytesper <= 0.6
        for i in 0:W*H-1
            byte = b[px_off + (i>>1) + 1]
            idx = (i & 1) == 0 ? (byte & 0x0f) : (byte >> 4)
            put(i, Int(idx))
        end
    elseif npal > 0 && bytesper <= 1.2
        for i in 0:W*H-1
            put(i, Int(b[px_off+i+1]))
        end
    elseif mtype == 3
        for i in 0:W*H-1
            v = UInt16(b[px_off+i*2+1]) | (UInt16(b[px_off+i*2+2])<<8)
            r=((v>>11)&0x1f); g=((v>>5)&0x3f); bl=(v&0x1f)
            rgba[i*4+1]=UInt8((r*255)÷31); rgba[i*4+2]=UInt8((g*255)÷63); rgba[i*4+3]=UInt8((bl*255)÷31); rgba[i*4+4]=0xff
        end
    elseif mtype == 4
        for i in 0:W*H-1
            v = UInt16(b[px_off+i*2+1]) | (UInt16(b[px_off+i*2+2])<<8)
            a=(v>>15); r=((v>>10)&0x1f); g=((v>>5)&0x1f); bl=(v&0x1f)
            rgba[i*4+1]=UInt8((r*255)÷31); rgba[i*4+2]=UInt8((g*255)÷31); rgba[i*4+3]=UInt8((bl*255)÷31)
            rgba[i*4+4]= a==1 ? 0xff : 0x00
        end
    else
        for i in 0:W*H-1
            v = UInt16(b[px_off+i*2+1]) | (UInt16(b[px_off+i*2+2])<<8)
            a=((v>>12)&0xf); r=((v>>8)&0xf); g=((v>>4)&0xf); bl=(v&0xf)
            rgba[i*4+1]=UInt8(r*17); rgba[i*4+2]=UInt8(g*17); rgba[i*4+3]=UInt8(bl*17); rgba[i*4+4]=UInt8(a*17)
        end
    end
    (W, H, rgba)
end

end
