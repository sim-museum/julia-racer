#!/bin/bash
# Julia Racer setup: the content archive into classic/, then the Julia packages.
#   ./install.sh /path/to/juliaRacerClassic.tar
#   JR_CLASSIC_URL=<address of the archive> ./install.sh
set -e
cd "$(dirname "$0")"
if [ ! -d classic/Classic ]; then
    tarf="$1"
    if [ -z "$tarf" ] && [ -n "$JR_CLASSIC_URL" ]; then
        tarf=$(mktemp --suffix=.tar); echo "downloading $JR_CLASSIC_URL"; curl -fL -o "$tarf" "$JR_CLASSIC_URL"
    fi
    [ -f "$tarf" ] || { echo "usage: ./install.sh <juliaRacerClassic.tar>   (or set JR_CLASSIC_URL)"; exit 1; }
    tar xf "$tarf" -C .
    [ -d classic/Classic ] || { echo "$tarf holds no classic/Classic"; exit 1; }
fi
command -v julia >/dev/null || { echo "Julia is missing: install Julia 1.12 (https://julialang.org/install/), then run this again"; exit 1; }
(cd demo/native && julia --project=. -e 'using Pkg; Pkg.instantiate()')
python3 -c "import PyQt6" 2>/dev/null || echo "PyQt6 is missing: pip install PyQt6 (or your distribution's python3-pyqt6)"
echo "ready:  cd demo/native && python3 juliaRacer.py"
