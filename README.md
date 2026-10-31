# Julia Racer

1967 Grand Prix cars on 1967 circuits, with physics fitted to measured telemetry.

Needs Julia 1.12, Python 3.11+ with PyQt6, and the julia racer content archive: the tracks, cars, sounds and setups,
built by community volunteers.

    ./install.sh /path/to/juliaRacerClassic.tar
    cd demo/native && python3 juliaRacer.py

The first launch compiles for several minutes. `JR_CLASSIC` can point at another copy of the content folder (the one
holding `tracks/`, `cars/` and `sound/`).
