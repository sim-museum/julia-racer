

using GLFW

const SLOT = let a = length(ARGS) >= 1 ? tryparse(Int, ARGS[1]) : nothing
    GLFW.Joystick(clamp((a === nothing ? 1 : a) - 1, 0, 15))
end

const AC_FD = Ref{Cint}(-1)
function autocenter_off!(match = "Thrustmaster")
    get(ENV, "JM_JOY_AUTOCENTER_OFF", "1") == "0" && return
    base = "/sys/class/input"; isdir(base) || return
    for d in readdir(base)
        startswith(d, "event") || continue
        nf = joinpath(base, d, "device", "name")
        (isfile(nf) && occursin(match, try read(nf, String) catch; "" end)) || continue
        fd = ccall(:open, Cint, (Cstring, Cint), joinpath("/dev/input", d), Cint(1))
        fd < 0 && (println(stderr, "joyserver: cannot open /dev/input/", d, " to turn the centring spring off"); return)
        ev = zeros(UInt8, 24)
        ev[17] = 0x15; ev[19] = 0x61
        n = ccall(:write, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), fd, ev, 24)
        println(stderr, "joyserver: wheel centring spring off on /dev/input/", d, n == 24 ? "" : " (write failed)")
        AC_FD[] = fd
        return
    end
end

jstr(s) = '"' * replace(string(s), '\\' => "\\\\", '"' => "\\\"") * '"'
farr(v) = '[' * join((string(round(Float64(x); digits=5)) for x in v), ',') * ']'
iarr(v) = '[' * join((string(Int(x)) for x in v), ',') * ']'

function main()
    GLFW.Init()
    GLFW.WindowHint(GLFW.VISIBLE, false)
    win = GLFW.CreateWindow(64, 64, "joyserver")
    GLFW.MakeContextCurrent(win)
    autocenter_off!()
    while !GLFW.WindowShouldClose(win)
        GLFW.PollEvents()
        if GLFW.JoystickPresent(SLOT)
            nm = GLFW.GetJoystickName(SLOT)
            ax = GLFW.GetJoystickAxes(SLOT)
            bs = GLFW.GetJoystickButtons(SLOT)
            print("{\"present\":true,\"slot\":", Int(SLOT) + 1,
                  ",\"name\":", jstr(nm === nothing ? "" : nm),
                  ",\"axes\":", farr(ax === nothing ? Float32[] : ax),
                  ",\"buttons\":", iarr(bs === nothing ? UInt8[] : bs), "}\n")
        else
            print("{\"present\":false,\"slot\":", Int(SLOT) + 1, "}\n")
        end
        flush(stdout)
        sleep(1 / 60)
    end
    GLFW.Terminate()
end

main()
