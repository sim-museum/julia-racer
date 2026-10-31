

module EngineAudio
using PortAudio, SampledSignals

"""Load a RIFF/WAVE PCM file → (mono Float32 samples, samplerate)."""
function load_wav(path)
    b = read(path)
    (length(b) >= 44 && String(b[1:4])=="RIFF" && String(b[9:12])=="WAVE") || error("not a WAVE: $path")
    u32(o)=Int(reinterpret(UInt32, b[o:o+3])[1]); u16(o)=Int(reinterpret(UInt16, b[o:o+1])[1])
    pos=13; channels=2; samplerate=44100; bits=16; data=Int16[]
    while pos+8 <= length(b)
        cid=String(b[pos:pos+3]); csz=u32(pos+4); body=pos+8
        if cid=="fmt "
            channels=u16(body+2); samplerate=u32(body+4); bits=u16(body+14)
        elseif cid=="data"
            stop=min(body+csz-1, length(b))
            data=collect(reinterpret(Int16, b[body:stop - (stop-body+1)%2]))
        end
        pos = body + csz + (csz & 1)
    end
    mono = if channels==2
        n=length(data)÷2
        Float32[(Float32(data[2i-1])+Float32(data[2i]))*(1f0/65536f0) for i in 1:n]
    else
        Float32.(data) .* (1f0/32768f0)
    end
    (mono, samplerate)
end

mutable struct Voice; data::Vector{Float32}; natural::Float64; phase::Float64; end
mutable struct Engine
    voices::Vector{Voice}
    rpm::Threads.Atomic{Float64}
    master::Threads.Atomic{Float64}
    running::Threads.Atomic{Bool}
    proc::Bool
    phase::Float64
    single::Bool
end

include(joinpath(@__DIR__, "classicroot.jl"))
const CLASSIC_ENGINE_WAV = joinpath(classic_root(), "sound", "66fordV8.wav")
const CLASSIC_ENGINE_RPM = 5200.0

"Linear-resample mono Float32 audio from `sr` to 44.1 kHz (so playback pitch math is clean)."
function resample44k(data::Vector{Float32}, sr)
    sr == 44100 && return data
    n = max(1, round(Int, length(data)*44100/sr)); step = sr/44100
    out = Vector{Float32}(undef, n); L = length(data)
    @inbounds for i in 1:n
        p = (i-1)*step + 1; k = unsafe_trunc(Int, p); fr = Float32(p - k)
        a = data[clamp(k,1,L)]; b = data[clamp(k+1,1,L)]
        out[i] = a*(1f0-fr) + b*fr
    end
    out
end

"""Build the Lotus 49 engine from Classic's own audio: the Ford Cosworth DFV V8 loop
(`Classic/sound/66fordV8.wav`), pitch-shifted by RPM the way Classic plays it.  Falls back
to a procedural DFV synth if the Classic audio isn't found (never to WAV voice clips)."""
function build_lotus(; classic_engine = CLASSIC_ENGINE_WAV, kwargs...)
    if isfile(classic_engine)
        d, sr = load_wav(classic_engine)
        v = Voice(resample44k(d, sr), CLASSIC_ENGINE_RPM, 0.0)
        return Engine([v], Threads.Atomic{Float64}(1800.0), Threads.Atomic{Float64}(0.7),
                      Threads.Atomic{Bool}(false), false, 0.0, true)
    end
    Engine(Voice[], Threads.Atomic{Float64}(1800.0), Threads.Atomic{Float64}(0.7),
           Threads.Atomic{Bool}(false), true, 0.0, false)
end

function mix_single!(out::Matrix{Float32}, eng::Engine)
    v = eng.voices[1]; len = length(v.data)
    r = eng.rpm[]; rpm = isfinite(r) ? max(r, 500.0) : 500.0
    master = Float32(eng.master[])
    rate = clamp(rpm / v.natural, 0.22, 2.4)
    ph = isfinite(v.phase) ? v.phase : 0.0
    @inbounds for i in 1:size(out,1)
        idx = unsafe_trunc(Int, ph); fr = Float32(ph - idx)
        a = v.data[idx % len + 1]; b = v.data[(idx+1) % len + 1]
        s = (a*(1f0-fr) + b*fr) * master
        out[i,1] = s; out[i,2] = s
        ph += rate; ph >= len && (ph -= len)
    end
    v.phase = ph
end

const DFV_ORDERS = ((1,0.10),(2,0.55),(3,0.16),(4,1.00),(5,0.18),(6,0.45),
                    (8,0.60),(10,0.24),(12,0.32),(16,0.14))

function mix_proc!(out::Matrix{Float32}, eng::Engine)
    N = size(out,1)
    r = eng.rpm[]; rpm = isfinite(r) ? clamp(r, 700.0, 10500.0) : 700.0
    master = eng.master[]
    crank = rpm/60.0
    inc = crank/44100.0
    amp = master * clamp(0.30 + 0.70*(rpm-1200.0)/8000.0, 0.20, 1.0)
    ph = isfinite(eng.phase) ? eng.phase : 0.0
    @inbounds for i in 1:N
        s = 0.0
        for (k,a) in DFV_ORDERS; s += a*sin(2π*k*ph); end
        s += 0.05*(2rand() - 1)
        v = Float32(tanh(s*0.55) * amp * 0.5)
        out[i,1] = v; out[i,2] = v
        ph += inc; ph >= 1.0 && (ph -= 1.0)
    end
    eng.phase = ph
end

function mix!(out::Matrix{Float32}, eng::Engine)
    if eng.single; mix_single!(out, eng); return; end
    if eng.proc;   mix_proc!(out, eng);   return; end
    fill!(out, 0f0); N=size(out,1)
    r=eng.rpm[]; rpm = isfinite(r) ? max(r,700.0) : 700.0
    master=eng.master[]; span=2200.0
    tot=0.0; for v in eng.voices; tot += max(0.0,1-abs(rpm-v.natural)/span); end
    tot<1e-6 && (tot=1.0)
    @inbounds for v in eng.voices
        g=Float32(max(0.0,1-abs(rpm-v.natural)/span)/tot*master); g<1f-4 && continue
        rate=clamp(rpm/v.natural,0.5,1.5); len=length(v.data); ph=isfinite(v.phase) ? v.phase : 0.0
        for i in 1:N
            idx=unsafe_trunc(Int,ph); fr=Float32(ph-idx)
            a=v.data[idx % len + 1]; b=v.data[(idx+1) % len + 1]
            s=(a*(1f0-fr)+b*fr)*g; out[i,1]+=s; out[i,2]+=s
            ph+=rate; ph>=len && (ph-=len)
        end
        v.phase=ph
    end
end

const AUDIO_BLOCKWRITE = get(ENV, "JM_AUDIO_BLOCKWRITE", "0") != "0"

const AUDIO_DIAG = get(ENV, "JM_AUDIO_DIAG", "0") != "0"
const DIAG_T0 = Ref(0.0); const DIAG_FRAMES = Ref(0); const DIAG_WRITES = Ref(0)
const DIAG_WORST = Ref(0.0); const DIAG_WAITW = Ref(0.0)
"""Start the audio thread (needs Julia ≥2 threads).  Returns the Engine; update
`eng.rpm[]` from the game loop, call `stop!(eng)` to end."""
function start(eng::Engine)
    haskey(ENV, "JM_NOSOUND") && (println("  engine audio off (JM_NOSOUND)"); return eng)
    (!eng.proc && isempty(eng.voices)) && (@warn "no engine samples found — no sound"; return eng)
    if Threads.nthreads() < 2
        @warn "sound needs ≥2 threads — relaunch with: julia -t 2 …"; return eng
    end
    eng.running[]=true
    LAT = parse(Float64, get(ENV, "JM_AUDIO_LATENCY", "0.30"))
    Threads.@spawn begin
        buf = zeros(Float32, 1024, 2)
        ilv = zeros(Float32, 2, 1024)

        devpref = let e = get(ENV, "JM_AUDIO_DEV", "")
            isempty(e) ? ["pulse", "default", "sysdefault"] : [e]
        end
        havedev = Set{String}()
        try; for d in PortAudio.devices(); d.output_bounds.max_channels > 0 && push!(havedev, d.name); end
        catch; end
        audiodev = nothing
        for d in devpref; if d in havedev; audiodev = d; break; end; end
        if audiodev === nothing
            @warn "no PulseAudio/PipeWire audio device found — falling back to PortAudio's default; \
                   the system volume control may not affect the sim" candidates=devpref
        else
            println("  [audio] output -> '", audiodev, "' (follows the Ubuntu default sink; JM_AUDIO_DEV overrides)")
            flush(stdout)
        end
        openfails = 0; wfails = 0; AUDIO_DIAG && (DIAG_T0[] = time())

        while eng.running[]
            local stream
            try
                stream = audiodev === nothing ? PortAudioStream(0,2; samplerate=44100, latency=LAT) :
                                                PortAudioStream(audiodev, 0, 2; samplerate=44100, latency=LAT)
                openfails = 0
            catch e

                openfails += 1
                if openfails >= 60
                    @warn "engine audio unavailable (couldn't open an audio device after retries) — running silently; set JM_NOSOUND=1 to skip audio" e
                    eng.running[]=false; break
                end
                sleep(0.1); continue
            end
            try
                while eng.running[]
                    mix!(buf, eng)

                    @inbounds for i in eachindex(buf)
                        x = buf[i]
                        buf[i] = isfinite(x) ? (x > 1f0 ? 1f0 : x < -1f0 ? -1f0 : x) : 0f0
                    end

                    _t0 = time()
                    if AUDIO_BLOCKWRITE
                        write(stream, buf)
                    else
                        n_ = size(buf, 1)
                        @inbounds for i in 1:n_
                            ilv[1, i] = buf[i, 1]; ilv[2, i] = buf[i, 2]
                        end
                        rc = @ccall gc_safe=true PortAudio.LibPortAudio.libportaudio.Pa_WriteStream(
                                   stream.pointer_to::Ptr{Cvoid}, pointer(ilv)::Ptr{Cfloat}, Culong(n_)::Culong)::Cint

                        (rc == 0 || rc == -9980) || error("Pa_WriteStream returned $rc")
                    end
                    if AUDIO_DIAG
                        _dt = time() - _t0
                        DIAG_FRAMES[] += size(buf, 1); DIAG_WRITES[] += 1
                        _dt > DIAG_WORST[] && (DIAG_WORST[] = _dt)
                        if time() - DIAG_T0[] >= 2.0
                            el = time() - DIAG_T0[]
                            println("  [audio] ", round(Int, DIAG_FRAMES[]/el), " frames/s of 44100 (",
                                    round(100*DIAG_FRAMES[]/el/44100, digits=1), "% of real time), ",
                                    DIAG_WRITES[], " writes, worst write ", round(1000*DIAG_WORST[], digits=1),
                                    " ms, worst wait ", round(1000*DIAG_WAITW[], digits=1), " ms")
                            flush(stdout)
                            DIAG_T0[] = time(); DIAG_FRAMES[] = 0; DIAG_WRITES[] = 0; DIAG_WORST[] = 0.0; DIAG_WAITW[] = 0.0
                        end
                    end
                    wfails = 0
                end
            catch e
                wfails += 1
                wfails <= 2 && @warn "audio xrun — reopening stream" e
            finally
                try; close(stream); catch; end
            end
            wfails > 0 && sleep(min(0.4, 0.05*wfails))
        end
    end
    eng
end
stop!(eng::Engine) = (eng.running[]=false)

end
