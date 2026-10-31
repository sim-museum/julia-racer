

const TRACK_SECTIONS = Dict(
    "nurburgring" => (lap = 22815.2, secs = [
        (0.0,     "Start und Ziel"),
        (574.0,   "Südkehre"),
        (1760.0,  "Nordkehre"),
        (2359.1,  "Hatzenbach"),
        (3337.1,  "Hocheichen"),
        (3726.7,  "Quiddelbacher Höhe"),
        (4175.5,  "Flugplatz"),
        (4405.0,  "Kottenborn"),
        (5290.6,  "Schwedenkreuz"),
        (5613.0,  "Aremberg"),
        (5850.0,  "Postbrücke"),
        (6365.4,  "Fuchsröhre"),
        (6650.0,  "Kompression"),
        (6924.3,  "Adenauer Forst"),
        (7738.4,  "Metzgesfeld"),
        (8257.0,  "Kallenhard"),
        (8465.0,  "Spiegelkurve"),
        (8740.0,  "Dreifach-Rechts"),
        (9157.1,  "Wehrseifen"),
        (9700.0,  "Breidscheid"),
        (9925.5,  "Ex-Mühle"),
        (10420.0, "Lauda Links"),
        (10742.4, "Bergwerk"),
        (11798.4, "Kesselchen"),
        (12647.0, "Angstkurve / Mutkurve"),
        (13133.8, "Klostertal"),
        (13560.0, "Steilstrecke"),
        (13759.9, "Karussell"),
        (14300.0, "Posten 147"),
        (14845.4, "Hohe Acht"),
        (15030.0, "Hedwigshöhe"),
        (15409.6, "Wippermann"),
        (15912.1, "Eschbach"),
        (16296.3, "Brünnchen"),
        (16616.0, "Eiskurve"),
        (16822.9, "Pflanzgarten I"),
        (17925.0, "Pflanzgarten II"),
        (18000.0, "Stefan-Bellof-S"),
        (18150.0, "Großer Sprunghügel"),
        (18308.8, "Schwalbenschwanz"),
        (18500.0, "Kleines Karussell"),
        (19249.3, "Galgenkopf"),
        (19829.7, "Döttinger Höhe"),
        (21557.4, "Antoniusbuche"),
        (21990.0, "Tiergarten"),
        (22309.8, "Hohenrain")]),
    "watglen" => (lap = 3755.5, secs = [
        (329.0,  "Esses"),
        (806.0,  "Front Straight"),
        (1596.0, "Carousel"),
        (1942.0, "Back Straight"),
        (2461.0, "The Speed Trap"),
        (3065.0, "Big Bend"),
        (3524.0, "The \"90\"")]),
    "monza" => (lap = 5749.7, secs = [
        (950.0,  "Curva Grande"),
        (1672.0, "Della Roggia"),
        (2135.0, "Lesmos"),
        (2918.0, "Serraglio"),
        (3456.0, "Ascari"),
        (3952.0, "Rettifilo Centrale"),
        (4803.0, "Parabolica")]),
    "spa" => (lap = 14112.0, secs = [
        (220.0,   "L'Eau Rouge"),
        (1843.0,  "Les Combes"),
        (3340.0,  "Burnenville"),
        (4231.0,  "Malmedy"),
        (6491.0,  "Masta"),
        (7924.0,  "Stavelot"),
        (9925.0,  "La Carrière"),
        (12104.0, "Blanchimont"),
        (13643.0, "La Source")]),
    "zandvoort" => (lap = 4190.5, secs = [
        (331.0,  "Tarzanbocht"),
        (592.0,  "Gerlachbocht"),
        (846.0,  "Hugenholtzbocht"),
        (1082.0, "Hunzerug"),
        (1286.0, "Zijn Veld"),
        (1465.0, "Jan de Wyker"),
        (1683.0, "Scheivlak"),
        (2157.0, "Hondenvlak"),
        (2872.0, "Tunnel Oost"),
        (3115.0, "Panoramabocht"),
        (3275.0, "Pulleveld"),
        (3506.0, "Huzaren Vlak")]),
)

const SECTION_EN = Dict(

    "Start und Ziel" => "Start and Finish", "Südkehre" => "South Hairpin", "Nordkehre" => "North Hairpin",
    "Flugplatz" => "Airfield", "Schwedenkreuz" => "Swedish Cross", "Fuchsröhre" => "Foxhole",
    "Adenauer Forst" => "Adenau Forest", "Ex-Mühle" => "Ex-Mill", "Bergwerk" => "Mine", "Kesselchen" => "Little Cauldron",
    "Klostertal" => "Monastery Valley", "Karussell" => "Carousel", "Hohe Acht" => "High Eight", "Brünnchen" => "Little Well",
    "Pflanzgarten I" => "Plant Garden I", "Pflanzgarten II" => "Plant Garden II", "Schwalbenschwanz" => "Swallow's Tail", "Döttinger Höhe" => "Dötting Heights",
    "Antoniusbuche" => "St Anthony's Beech", "Tiergarten" => "Animal Park",

    "Curva Grande" => "Big Curve", "Rettifilo Centrale" => "Central Straight", "Parabolica" => "Parabolic",

    "L'Eau Rouge" => "Red Water", "La Carrière" => "The Quarry", "La Source" => "The Spring",

    "Tarzanbocht" => "Tarzan Bend", "Gerlachbocht" => "Gerlach Bend", "Hugenholtzbocht" => "Hugenholtz Bend",
    "Hondenvlak" => "Dogs' Flat", "Tunnel Oost" => "Tunnel East", "Panoramabocht" => "Panorama Bend",
    "Huzaren Vlak" => "Hussars' Flat",

    "Hatzenbach" => "Hatz Brook", "Aremberg" => "Eagle Mountain", "Metzgesfeld" => "Butcher's Field",
    "Kallenhard" => "Cold Ridge", "Wehrseifen" => "Weir Trickle", "Wippermann" => "Seesaw Man",
    "Eschbach" => "Ash Brook",

    "Hocheichen" => "High Oaks", "Quiddelbacher Höhe" => "Quiddelbach Heights", "Breidscheid" => "Broad Divide",
    "Angstkurve / Mutkurve" => "Fear / Courage Curve", "Eiskurve" => "Ice Curve", "Galgenkopf" => "Gallows Head", "Hohenrain" => "High Ridge",

    "Kottenborn" => "Kotten Spring", "Postbrücke" => "Post Bridge", "Kompression" => "Compression",
    "Spiegelkurve" => "Mirror Curve", "Dreifach-Rechts" => "Triple Right", "Lauda Links" => "Lauda's Left",
    "Steilstrecke" => "Steep Stretch", "Posten 147" => "Marshal Post 147", "Hedwigshöhe" => "Hedwig's Heights",
    "Großer Sprunghügel" => "Big Jump Hill", "Stefan-Bellof-S" => "Stefan Bellof's S", "Kleines Karussell" => "Little Carousel",
    "Della Roggia" => "Of the Irrigation Ditch", "Lesmos" => "Lazy Bends", "Serraglio" => "Seraglio", "Ascari" => "Ascari's Corner",
    "Les Combes" => "The Hollows", "Burnenville" => "Burning Town", "Malmedy" => "Bad Medicine", "Masta" => "Mast Village",
    "Stavelot" => "Stable Lot", "Blanchimont" => "White Mount",
    "Hunzerug" => "Huns' Back", "Zijn Veld" => "His Field", "Jan de Wyker" => "John the Yielder", "Scheivlak" => "Parting Flat",
    "Pulleveld" => "Puddle Field")
section_label(n) = (get(ENV, "JM_SEGNAME_EN", "1") != "0" && haskey(SECTION_EN, n)) ? string(n, " (", SECTION_EN[n], ")") : n

"""Section boundaries for `track`, scaled onto a ribbon of length `laplen`: a sorted Vector{Tuple{Float64,String}}
(empty if the track has none)."""
function track_sections(track::AbstractString, laplen::Real)
    t = get(TRACK_SECTIONS, track, nothing)
    t === nothing && return Tuple{Float64,String}[]
    k = laplen / t.lap
    sort!([(s*k, section_label(n)) for (s, n) in t.secs]; by = first)
end

"Index of the section containing lap distance `s` (the last boundary at or before it; wraps to the last section)."
function section_at(secs, s::Real, laplen::Real)
    isempty(secs) && return 0
    s = mod(s, laplen)
    i = searchsortedlast(secs, (s, ""); by = first)
    i == 0 ? length(secs) : i
end

"""Per-frame update of the section banner. Returns the new (current index, banner text, banner start time).
A banner starts only once the car is ≥ `hyst` metres into a new section, so lap-distance jitter on a
boundary cannot make it flicker, and the first call only records where the car is (no banner at spawn)."""
function section_update(secs, s::Real, laplen::Real, t::Real, cur::Int, text::String, t0::Float64; hyst = 3.0)
    isempty(secs) && return (0, text, t0)
    i = section_at(secs, s, laplen)
    cur == 0 && return (i, text, t0)
    (i == cur || section_at(secs, s - hyst, laplen) != i) && return (cur, text, t0)
    (i, secs[i][2], Float64(t))
end
