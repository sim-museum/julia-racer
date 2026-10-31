

function classic_root()
    haskey(ENV, "JR_CLASSIC") && return normpath(ENV["JR_CLASSIC"])
    own = normpath(joinpath(@__DIR__, "..", "..", "classic", "Classic"))
    isdir(own) && return own
    rel = normpath(joinpath(@__DIR__, "..", "..", "..", "..", "WP", "drive_c", "Vendor", "Classic"))
    isdir(rel) ? rel : own
end
