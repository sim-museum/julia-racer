

module NetPlay

using Sockets
export NetLink, netopen, netclose, send_pose!, poll!, remote_poses, predict, remote_poses_at,
       EXTRAP_MAX, STALE_S, is_stale, AI_ID0, chassis_slot, KNOCK_TICK

const MAGIC   = UInt32(0x4A4D5231)

const AI_ID0 = UInt8(10)

const KNOCK_TICK = typemax(UInt32)
"""Which loaded AI chassis draws remote car `id`: AI ids map to slot 1.. in order, clamped to the
models actually loaded; a human id draws with slot 1 (the Lotus-substitute chassis)."""
chassis_slot(id::Integer, nmodels::Integer) =
    nmodels <= 0 ? 0 : (id >= AI_ID0 ? clamp(Int(id) - Int(AI_ID0) + 1, 1, Int(nmodels)) : 1)

fnv1a32(s::AbstractString) = (h = 0x811c9dc5; for b in codeunits(s); h = (h ⊻ UInt32(b)) * 0x01000193; end; h)
const BUILDTAG = isempty(strip(get(ENV, "SGW_BUILD", ""))) ? UInt32(0) : fnv1a32(strip(ENV["SGW_BUILD"]))

passwordtag(s::AbstractString) = isempty(strip(s)) ? UInt32(0) : fnv1a32(strip(s))

const PKTSIZE = 4 + 4 + 4 + 1 + 4 + 2 + 6*4 + 1
const CARLESS_PKTSIZE = PKTSIZE - 1
const DEFAULT_PORT = 47700

const REFMAGIC = UInt32(0x4A4D5246)
const REF_VERSION, REF_NEEDPW, REF_BADPW = UInt8(1), UInt8(2), UInt8(3)
const REF_CAR = UInt8(4)
const NOREPLY_S = 10.0
const OLDPKTSIZE = CARLESS_PKTSIZE - 4
buildname() = (b = strip(get(ENV, "SGW_BUILD", "")); isempty(b) ? "unknown" : String(b))

"""One peer's view of the session. `remote` maps car id -> the last pose received for it."""

mutable struct NetLink
    rsock::UDPSocket
    ssock::UDPSocket
    port::Int
    peers::Vector{Tuple{IPAddr,Int}}
    remote::Dict{UInt8,NamedTuple}
    rxtime::Dict{UInt8,Float64}
    inbox::Channel{Tuple{IPAddr,Int,Vector{UInt8}}}
    task::Union{Task,Nothing}
    rx::Int
    dropped::Int
    skew::Int
    pwtag::UInt32
    ishost::Bool
    pwbad::Int
    open::Bool
    readererr::String
    refused::String
    reftime::Dict{Tuple{IPAddr,Int},Float64}
    t0::Float64
    carbad::Int
    car::UInt8
    carname::String
end

"""    netopen(; port, peer) -> NetLink

`peer = nothing` hosts: bind and learn each peer's address from its first packet (so a client
needs no configuration, which is how Classic behaved). `peer = (host, port)` joins.

A background task owns the socket and pushes datagrams into a channel. That is not decoration:
`recvfrom` BLOCKS, so calling it from the render loop would stall the frame whenever no packet had
arrived — and the obvious workaround (spawn a task per poll and abandon it on timeout) leaves a
dangling reader that eats the NEXT packet. One long-lived reader, one channel, and `poll!` never
blocks.
"""
function netopen(; port::Int = DEFAULT_PORT, peer = nothing, password::AbstractString = get(ENV, "JM_NET_PASSWORD", ""))
    r = UDPSocket()
    bind(r, ip"0.0.0.0", port) || error("netopen: cannot bind UDP port $port")
    sk = UDPSocket()
    peers = Tuple{IPAddr,Int}[]
    peer === nothing || push!(peers, (getaddrinfo(String(peer[1])), Int(peer[2])))
    n = NetLink(r, sk, port, peers, Dict{UInt8,NamedTuple}(), Dict{UInt8,Float64}(),
                Channel{Tuple{IPAddr,Int,Vector{UInt8}}}(256), nothing, 0, 0, 0, passwordtag(password), peer === nothing, 0, true, "",
                "", Dict{Tuple{IPAddr,Int},Float64}(), time(), 0, 0xff, "")
    n.task = @async begin
        while n.open
            try

                (from, buf) = recvfrom(n.rsock)
                isopen(n.inbox) && put!(n.inbox, (from.host, Int(from.port), buf))
            catch e
                n.open || break
                if get(ENV, "JM_NET_TRACE", "0") != "0"
                    @warn "netplay reader stopped" exception=(e, catch_backtrace())
                end
                n.readererr = string(typeof(e))
                break
            end
        end
    end
    n
end

function netclose(n::NetLink)
    n.open = false
    try; close(n.rsock); catch; end
    try; close(n.ssock); catch; end
    try; close(n.inbox); catch; end
    nothing
end

"""Broadcast one car's pose to every known peer. Six Float32: x, y, z, yaw, speed, steer; then its car type."""
function send_pose!(n::NetLink, id::Integer, tick::Integer, x, y, z, yaw, v = 0.0, steer = 0.0; car::Integer = 0xff)
    io = IOBuffer()
    write(io, MAGIC); write(io, BUILDTAG); write(io, n.pwtag); write(io, UInt8(id)); write(io, UInt32(tick))
    write(io, UInt16(n.port))
    for f in (x, y, z, yaw, v, steer); write(io, Float32(f)); end
    write(io, UInt8(car))
    buf = take!(io)
    isempty(n.refused) || return 0
    for (h, p) in n.peers
        send(n.ssock, h, p, buf)
    end
    length(n.peers)
end

"""Host: tell the racer at (host, its listen port) why it is ignored -- at most once a second per racer."""
function refuse!(n::NetLink, host::IPAddr, lport::Int, reason::UInt8; text::AbstractString = buildname())
    (n.ishost && lport > 0) || return false
    k = (host, lport)
    time() - get(n.reftime, k, -Inf) < 1.0 && return false
    n.reftime[k] = time()
    v = codeunits(text)[1:min(end, 255)]
    io = IOBuffer(); write(io, REFMAGIC); write(io, reason); write(io, UInt8(length(v))); write(io, v)
    try; send(n.ssock, host, lport, take!(io)); catch; return false; end
    true
end

"""The text a refused racer reads: what is wrong and what to do about it."""
function refusal_text(reason::Integer, hostver::AbstractString)
    reason == REF_VERSION ? (hostver == buildname() ?
                             "Different version: the host runs another version of Julia Racer. Both players need the same version." :
                             string("Different version: the host runs Julia Racer ", hostver, ", you run ", buildname(),
                                    ". Both players need the same version.")) :
    reason == REF_NEEDPW  ? "This race needs a password. Leave, then join again with the host's password." :
    reason == REF_BADPW   ? "Wrong password for this race. Leave, then join again with the host's password." :
    reason == REF_CAR     ? string("The host drives the ", hostver, ". Leave, pick another car on the launcher's Drive list, then join again.") :
                            string("The host refused this racer (reason ", reason, ").")
end

"""What a racer's screen should say about the link, or "" when all is well (host: always "")."""
function status_text(n::NetLink, now::Float64 = time())
    n.ishost && return ""
    isempty(n.refused) || return n.refused
    if n.rx == 0 && now - n.t0 > NOREPLY_S && !isempty(n.peers)
        h, p = n.peers[1]
        return string("No reply from the host at ", h, ":", p, ". Check the address and port, and that the host's race has started.")
    end
    ""
end

"""Drain whatever has arrived. NEVER blocks — it takes only what the reader task has already queued.

A host learns a peer here, on that peer's first packet. Anything without the magic is counted as
dropped rather than discarded silently: "no remote cars" and "the packets are being thrown away"
look identical from outside and need different fixes.
"""
function poll!(n::NetLink; maxpkts::Int = 64)
    got = 0
    while got < maxpkts && isready(n.inbox)
        (host, port, buf) = take!(n.inbox)
        if length(buf) >= 6 && reinterpret(UInt32, buf[1:4])[1] == REFMAGIC
            if !n.ishost && isempty(n.refused)
                nv = Int(buf[6]); hv = String(buf[7:min(end, 6 + nv)])
                n.refused = refusal_text(buf[5], hv)
                println("  netplay: refused by ", host, ": ", n.refused); flush(stdout)
            end
            continue
        end
        old = length(buf) == OLDPKTSIZE
        carless = length(buf) == CARLESS_PKTSIZE
        if length(buf) != PKTSIZE && !old && !carless
            n.dropped += 1; continue
        end
        io = IOBuffer(buf)
        if read(io, UInt32) != MAGIC
            n.dropped += 1; continue
        end
        tag = read(io, UInt32)
        if old || carless || tag != BUILDTAG
            n.skew += 1
            n.skew == 1 && println("  netplay: ignoring ", host, ": that racer runs a different version of Julia Racer ",
                                   "(yours: ", buildname(), "). Both players need the same version.")
            refuse!(n, host, Int(reinterpret(UInt16, buf[(old ? 14 : 18):(old ? 15 : 19)])[1]), REF_VERSION)
            continue
        end
        pw = read(io, UInt32)
        if !(pw == n.pwtag || (n.ishost ? n.pwtag == 0 : pw == 0))
            n.pwbad += 1
            n.pwbad == 1 && println("  netplay: ignoring ", host, ": ", n.ishost ?
                "that racer has the wrong race password." :
                (n.pwtag == 0 ? "that race needs a password." : "wrong race password for that race."))
            refuse!(n, host, Int(reinterpret(UInt16, buf[18:19])[1]), pw == 0 ? REF_NEEDPW : REF_BADPW)
            continue
        end
        id = read(io, UInt8); tick = read(io, UInt32)
        lport = Int(read(io, UInt16))
        x = read(io, Float32); y = read(io, Float32); z = read(io, Float32)
        yaw = read(io, Float32); v = read(io, Float32); st = read(io, Float32); car = read(io, UInt8)
        n.remote[id] = (tick = Int(tick), x = Float64(x), y = Float64(y), z = Float64(z),
                        yaw = Float64(yaw), v = Float64(v), steer = Float64(st), car = car)
        n.rxtime[id] = time()
        n.rx += 1; got += 1
        any(p -> p[1] == host && p[2] == lport, n.peers) || push!(n.peers, (host, lport))
    end
    got
end

"""The remote cars, as an id-ordered vector — the shape the AI field already draws."""
remote_poses(n::NetLink) = [(id, n.remote[id]) for id in sort(collect(keys(n.remote)))]

"""    predict(p, dt) -> NamedTuple

`p` as delivered by `poll!`, advanced `dt` seconds along its own heading at its own speed.
`dt <= 0` returns `p` unchanged. `y` is NOT extrapolated: height comes from the terrain under the
car, and guessing it is how a remote car ends up flying (see E104(a) — a car's drawn height must be
the ground beneath it, never a value carried from somewhere else).
"""
function predict(p, dt::Real)
    dt <= 0 && return p
    (; x = p.x + p.v*cos(p.yaw)*dt, y = p.y, z = p.z + p.v*sin(p.yaw)*dt,
       yaw = p.yaw, v = p.v, steer = p.steer, tick = p.tick, car = get(p, :car, 0xff))
end

"""How far ahead dead reckoning may extrapolate (s). Past this the pose FREEZES: beyond a couple of
packet intervals a straight-line guess is not evidence about where a car is, and a frozen car is a
smaller lie than a confidently wrong one."""
const EXTRAP_MAX = 0.25

"""After this long with no packet (s) a peer is GONE and is removed from the field, rather than
left driving as a ghost."""
const STALE_S = 2.0

"""True when `age` seconds have passed with no packet from a peer."""
is_stale(age::Real) = age >= STALE_S

"""    remote_poses_at(n, now) -> [(id, pose)]

`remote_poses`, dead-reckoned to `now` using each packet's own arrival time, with the staleness
policy applied: extrapolation capped at `EXTRAP_MAX`, and peers silent for `STALE_S` OMITTED.
`now` is passed in rather than read here so a caller can use the frame's timestamp and a test can
use a fixed clock.
"""
function remote_poses_at(n::NetLink, now::Real)
    out = Tuple{UInt8,NamedTuple}[]
    for id in sort(collect(keys(n.remote)))
        age = now - get(n.rxtime, id, now)
        is_stale(age) && continue
        n.remote[id].tick == Int(KNOCK_TICK) && continue
        push!(out, (id, predict(n.remote[id], min(age, EXTRAP_MAX))))
    end
    out
end

end
