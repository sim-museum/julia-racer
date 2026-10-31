

module WreckSeal
export seal_target

"""    seal_target(wrecked, curx, curz, lastx, lastz; seal_back=false) -> (x, z)

A WRECK is sealed where it lies: the race is over, and moving it is the defect.
A DRIVEABLE car is sealed back to its last on-track point: that is a rescue, and the race continues.
`seal_back=true` (JM_WRECK_SEAL_BACK) restores the old behaviour for both.
"""
seal_target(wrecked::Bool, curx, curz, lastx, lastz; seal_back::Bool = false) =
    (wrecked && !seal_back) ? (curx, curz) : (lastx, lastz)

end
