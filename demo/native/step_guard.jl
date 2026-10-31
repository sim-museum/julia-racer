

module StepGuard
export step_guard
function step_guard(g::Float64, last::Float64, thresh::Float64)
    (isnan(last) || g <= last + thresh) && return (true, g)
    (false, last)
end
end
