

using GLFW

const SLOT = let a = length(ARGS) >= 1 ? tryparse(Int, ARGS[1]) : nothing
    GLFW.Joystick(clamp((a === nothing ? 1 : a) - 1, 0, 15))
end

jstr(s) = '"' * replace(string(s), '\\' => "\\\\", '"' => "\\\"") * '"'
farr(v) = '[' * join((string(round(Float64(x); digits=5)) for x in v), ',') * ']'
iarr(v) = '[' * join((string(Int(x)) for x in v), ',') * ']'

function main()
    GLFW.Init()
    GLFW.WindowHint(GLFW.VISIBLE, false)
    win = GLFW.CreateWindow(64, 64, "joyserver")
    GLFW.MakeContextCurrent(win)
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
