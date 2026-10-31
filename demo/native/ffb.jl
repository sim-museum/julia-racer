

module FFB

export FFBDevice, open_ffb, set_force!, close_ffb

const FF_CONSTANT = 0x0052
const EV_FF       = 0x0015
const EFFSZ       = 48

const _E = Int(0x45)
const EVIOCSFF  = Culong((1<<30) | (EFFSZ<<16) | (_E<<8) | 0x80)
const EVIOCRMFF = Culong((1<<30) | (4<<16)     | (_E<<8) | 0x81)
const O_RDWR    = Cint(2)

mutable struct FFBDevice
    fd::Cint
    id::Int16
    buf::Vector{UInt8}
    ok::Bool
    path::String
end

@inline function _put16!(buf, off, v::UInt16)
    @inbounds buf[off+1] = UInt8(v & 0xff)
    @inbounds buf[off+2] = UInt8((v >> 8) & 0xff)
end
_puti16!(buf, off, v::Integer) = _put16!(buf, off, reinterpret(UInt16, Int16(v)))
_getu16(buf, off) = UInt16(buf[off+1]) | (UInt16(buf[off+2]) << 8)

"Find the wheel's event node by name substring (default: a Thrustmaster wheel)."
function find_event(match = "Thrustmaster")
    base = "/sys/class/input"
    isdir(base) || return nothing
    for d in readdir(base)
        startswith(d, "event") || continue
        nf = joinpath(base, d, "device", "name")
        isfile(nf) || continue
        try
            occursin(match, read(nf, String)) && return joinpath("/dev/input", d)
        catch
        end
    end
    nothing
end

"Open the wheel for FFB and upload+start a zero-level constant force. Returns an
FFBDevice; check `.ok`. Safe to call when no wheel/FFB is present (ok=false)."
function open_ffb(path = nothing; match = "Thrustmaster")
    path === nothing && (path = find_event(match))
    path === nothing && return FFBDevice(-1, 0, UInt8[], false, "")
    fd = ccall(:open, Cint, (Cstring, Cint), path, O_RDWR)
    fd < 0 && return FFBDevice(-1, 0, UInt8[], false, path)
    buf = zeros(UInt8, EFFSZ)
    _put16!(buf, 0,  FF_CONSTANT)
    _puti16!(buf, 2, -1)
    _put16!(buf, 4,  UInt16(0x4000))
    _put16!(buf, 10, UInt16(0))
    _put16!(buf, 12, UInt16(0))
    _puti16!(buf, 16, 0)
    rc = ccall(:ioctl, Cint, (Cint, Culong, Ptr{UInt8}), fd, EVIOCSFF, buf)
    if rc < 0
        ccall(:close, Cint, (Cint,), fd)
        return FFBDevice(-1, 0, UInt8[], false, path)
    end
    id = reinterpret(Int16, _getu16(buf, 2))
    dev = FFBDevice(fd, id, buf, true, path)
    _play(dev, true)
    dev
end

function _play(dev::FFBDevice, on::Bool)
    ev = zeros(UInt8, 24)
    _put16!(ev, 16, UInt16(EV_FF))
    _put16!(ev, 18, reinterpret(UInt16, dev.id))
    val = Int32(on ? 1 : 0)
    @inbounds for k in 0:3; ev[20+1+k] = UInt8((reinterpret(UInt32, val) >> (8k)) & 0xff); end
    ccall(:write, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), dev.fd, ev, 24)
end

"""Set the steering force, `f ∈ [-1,1]` (sign = direction). Updates the running effect
live (one ioctl). No-op if the device isn't OK."""
function set_force!(dev::FFBDevice, f::Real)
    dev.ok || return dev
    lvl = round(Int, clamp(f, -1.0, 1.0) * 32767)
    _puti16!(dev.buf, 16, lvl)
    ccall(:ioctl, Cint, (Cint, Culong, Ptr{UInt8}), dev.fd, EVIOCSFF, dev.buf)
    dev
end

"Stop the effect, remove it, and close the device."
function close_ffb(dev::FFBDevice)
    dev.ok || return
    _play(dev, false)
    ccall(:ioctl, Cint, (Cint, Culong, Clong), dev.fd, EVIOCRMFF, Clong(dev.id))
    ccall(:close, Cint, (Cint,), dev.fd)
    dev.ok = false
    nothing
end

end
