

module Setup

export parse_yaml_block, setup_params, firstnum

"""    firstnum(s) -> Float64

First numeric token in a string, sign/decimals honoured ("26 N/mm"→26.0,
"53.50%"→53.5, "10:1"→10.0, "+0.5 deg"→0.5, "-1 mm"→-1.0). NaN if none.
"""
function firstnum(s::AbstractString)
    m = match(r"[-+]?\d+(?:\.\d+)?", s)
    m === nothing ? NaN : parse(Float64, m.match)
end

"""    parse_yaml_block(yaml, header) -> Dict

Parse the indentation-nested block under a top-level `header` (e.g. "CarSetup")
into nested Dict{String,Any}; leaves are the raw value strings.
"""
function parse_yaml_block(yaml::AbstractString, header::AbstractString)
    lines = split(yaml, '\n')

    istart = findfirst(l -> startswith(l, header * ":"), lines)
    istart === nothing && error("block $header not found")
    root = Dict{String,Any}()
    stack = Tuple{Int,Dict{String,Any}}[(-1, root)]
    for i in istart+1:length(lines)
        line = lines[i]
        isempty(strip(line)) && continue
        indent = length(line) - length(lstrip(line))
        indent == 0 && break
        content = strip(line)
        startswith(content, "- ") && continue
        while length(stack) > 1 && stack[end][1] >= indent
            pop!(stack)
        end
        parent = stack[end][2]
        if endswith(content, ":")
            key = rstrip(content, ':')
            d = Dict{String,Any}()
            parent[key] = d
            push!(stack, (indent, d))
        else
            kv = split(content, ": ", limit=2)
            length(kv) == 2 && (parent[kv[1]] = kv[2])
        end
    end
    root
end

"""    setup_params(yaml) -> NamedTuple

Pull the model parameters we care about out of CarSetup + DriverInfo, units
stripped to SI-ish Float64.  Corner keys are LF/RF/LR/RR.
"""
function setup_params(yaml::AbstractString)
    cs = parse_yaml_block(yaml, "CarSetup")
    di = parse_yaml_block(yaml, "DriverInfo")
    ch = cs["Chassis"]; dt = cs["Drivetrain"]; ti = cs["Tires"]
    corners = ("LeftFront"=>:LF, "RightFront"=>:RF, "LeftRear"=>:LR, "RightRear"=>:RR)
    g(d, k) = haskey(d, k) ? firstnum(d[k]) : NaN
    cw  = Dict(sym => g(ch[name], "CornerWeight") for (name,sym) in corners)
    spr = Dict(sym => g(ch[name], "SpringRate")  for (name,sym) in corners)
    rh  = Dict(sym => g(ch[name], "RideHeight")  for (name,sym) in corners)
    cmb = Dict(sym => g(ch[name], "Camber")      for (name,sym) in corners)
    cp  = Dict(sym => g(ti[name], "ColdPressure") for (name,sym) in corners)

    nums(s) = [parse(Float64, m.match) for m in eachmatch(r"-?\d+(?:\.\d+)?", string(s))]
    sdef(name) = haskey(ch[name], "ShockDeflection") && length(nums(ch[name]["ShockDeflection"])) >= 2 ? Tuple(nums(ch[name]["ShockDeflection"])[1:2]) : (NaN, NaN)
    shk = Dict(sym => sdef(name) for (name,sym) in corners)
    pk  = Dict(sym => g(ch[name], "Packer") for (name,sym) in corners)
    po  = Dict(sym => g(ch[name], "SpringPerchOffset") for (name,sym) in corners)
    trans = dt["Transmission"]
    gears = [firstnum(trans[k]) for k in ("FirstGear","SecondGear","ThirdGear","FourthGear","FifthGear")]
    (; corner_weight_N = cw,
       spring_rate_Npmm = spr,
       ride_height_mm = rh,
       camber_deg = cmb,
       cold_pressure_kPa = cp,
       shock_defl_mm    = shk,
       packer_mm        = pk,
       perch_mm         = po,
       brake_bias_pct   = g(ch["Front"], "BrakeBias"),
       steering_ratio   = g(ch["Front"], "SteeringRatio"),
       cross_weight_pct = g(ch["Front"], "CrossWeight"),
       final_drive      = firstnum(trans["FinalDrive"]),
       gear_ratios      = gears,
       diff_preload_Nm  = g(dt["Differential"], "Preload"),
       diff_drive_ramp  = g(dt["Differential"], "DriveRampAngle"),
       diff_coast_ramp  = g(dt["Differential"], "CoastRampAngle"),

       diff_plates      = g(dt["Differential"], "ClutchPlates"),
       toe_front_mm     = g(ch["Front"], "ToeIn"),
       toe_rear_mm      = g(ch["Rear"], "ToeIn"),
       arb_front        = strip(get(ch["Front"], "ArbDiameter", "?") * " " * get(ch["Front"], "ArbArms", "?")),
       arb_rear         = strip(get(ch["Rear"], "ArbDiameter", "?") * " " * get(ch["Rear"], "ArbArms", "?")),
       bump_clicks      = Dict(sym => g(ch[name], "BumpStiffness")    for (name,sym) in corners),
       rebound_clicks   = Dict(sym => g(ch[name], "ReboundStiffness") for (name,sym) in corners),
       fuel_L           = g(ch["Rear"], "FuelLevel"),
       idle_rpm   = firstnum(di["DriverCarIdleRPM"]),
       redline_rpm= firstnum(di["DriverCarRedLine"]),
       shift_rpm  = haskey(di, "DriverCarSLShiftRPM") ? firstnum(di["DriverCarSLShiftRPM"]) : NaN,
       n_gears    = firstnum(di["DriverCarGearNumForward"]),
       fuel_kg_per_L = firstnum(di["DriverCarFuelKgPerLtr"]))
end

end
