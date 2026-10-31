

module CornerLoads
export CORNERS, fit_corner_loads, corner_Fz, CLModel

const CORNERS = (:LF, :RF, :LR, :RR)
const G = 9.80665

med(v) = (s = sort(v); isempty(s) ? 0.0 : s[cld(length(s), 2)])

struct CLModel
    MR::Float64
    cd::Float64
    ref::Dict{Symbol,Float64}
    ks::Dict{Symbol,Float64}
    cw::Dict{Symbol,Float64}
    r2::Float64
end

"""    fit_corner_loads(shock, shockvel, vertaccel, m, ks, cw, good, static)

Calibrate the motion ratio + damper coeff.  `shock`/`shockvel` are Dicts corner→
Vector; `good`/`static` are sample-index vectors (general / quasi-static).
Returns a `CLModel`.
"""
function fit_corner_loads(shock, shockvel, vertaccel, m, ks, cw, good, static)
    ref = Dict(c => med([shock[c][i] for i in static]) for c in CORNERS)

    S = [sum(ks[c]*(shock[c][i] - ref[c]) for c in CORNERS) for i in good]
    V = [sum(shockvel[c][i] for c in CORNERS) for i in good]
    Y = [m*(vertaccel[i] - G) for i in good]
    Sss = sum(abs2, S); Vvv = sum(abs2, V); Svv = sum(S[k]*V[k] for k in eachindex(S))
    SY = sum(S[k]*Y[k] for k in eachindex(S)); VY = sum(V[k]*Y[k] for k in eachindex(V))
    det = Sss*Vvv - Svv^2
    MR  = (SY*Vvv - VY*Svv) / det
    cd  = (Sss*VY - Svv*SY) / det
    pred = [MR*S[k] + cd*V[k] for k in eachindex(S)]
    ȳ = sum(Y)/length(Y)
    r2 = 1 - sum(abs2, pred .- Y)/sum(x->abs2(x-ȳ), Y)
    CLModel(MR, cd, ref, ks, cw, r2)
end

"Per-corner vertical load [N] at one sample (clamped ≥ 0 — tyre can't pull)."
corner_Fz(M::CLModel, shock, shockvel, c::Symbol) =
    max(0.0, M.cw[c] + M.MR*M.ks[c]*(shock - M.ref[c]) + M.cd*shockvel)

end
