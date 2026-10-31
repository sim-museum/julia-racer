

module PeopleFilter
export is_loose_person

const NAMED = ("chrisa", "sergioa", "thomasa", "hatzia", "stefana", "starter")

const PREFIXES = ("ppl", "peop", "person", "pelf",
                  "p_s",

                  "grndp", "crowd", "spect",
                  "flagger", "rescu",
                  "photo", "fotograf")

const NOT_PEOPLE = ("pelfspa",)

"""True if `nm` (lowercase object name) is a LOOSE person that must not be placed."""
is_loose_person(nm::AbstractString) = !(nm in NOT_PEOPLE) && ((nm in NAMED) || any(p -> startswith(nm, p), PREFIXES))

end
