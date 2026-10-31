"""TRACKGUIDE-1 (PO 2026-10-09: "for the julia racer post-race analysis, use Classic track guides under ~/sgl/THU").

The Lights Out Racing guides (`DOC/trackGuidesAndSetups_LOR/CLASSIC_trackguide_<Track>_by_LOR.pdf`, one per 1967 circuit)
describe every corner the same way -- a heading (the map's section name, or "FIRST CORNER (Left hander):" under a
section at the Ring), then

    Entrance speed: approximately 170 mph [274 kph]
    Speed through the corner: approximately 60 mph [97 kph]
    Arrival time at the corner: 5.63 seconds
    Time to negotiate the corner: 6.09 seconds
    Exit time of the corner: 11.72 seconds

and a few paragraphs on braking, gears and line, all from the guide author's own replay lap. This module reads them
(at run time, from the PO's Classic documents -- the guides' text is not copied into the repo or the AppImage), places each
corner on the driver's lap, and compares: the guide's entrance and slowest speed and time through the corner against
the driver's. `pdftotext` (poppler) is needed; without it, or without the guide, `load` returns None and the analyser
says so.

Placement: each corner belongs to a section of the sim's own table (`track_sections.jl`, carried in the replay header);
the guide's clock between the end of the previous section's last corner and the arrival at the next section's first
corner is mapped linearly onto the driver's time through that section. A corner can therefore not leave its section,
whatever the difference in pace (the guide laps a Ferrari in 8:04 at the Ring, 1:05 at Watkins Glen).

AppImage rule (PO 2026-10-04: stand-alone, nothing read from ~/sgl): the image carries the guides as text, extracted at
build time (`python3 trackguide.py --export DIR`, tools/appimage/build_julia.sh); AppRun points JM_TRACKGUIDE_DIR at
it, so the image needs neither the PDFs nor pdftotext. Outside the image the PDFs are read directly.
"""
import os
import re
import shutil
import subprocess
import unicodedata

LOR = {"nurburgring": "Nuerburgring", "watglen": "WatkinsGlen", "monza": "Monza", "spa": "Spa", "zandvoort": "Zandvoort"}
FS = {"nurburgring": "nurburgring", "watglen": "watkinsGlen", "monza": "monza", "spa": "spa67", "zandvoort": "zandvort"}

ALIASES = {"theloop": "Carousel", "loop": "Carousel"}

GROUPS = {"nurburgring": [("Südkehre", True), ("Nordkehre", True), ("Hatzenbach", True), ("Flugplatz", True),
                          ("Schwedenkreuz", True), ("Aremberg", True), ("Fuchsröhre", True), ("Adenauer Forst", True),
                          ("Metzgesfeld", True), ("Kallenhard", True), ("Wehrseifen", True), ("Ex-Mühle", True),
                          ("Bergwerk", True), ("Kesselchen", True), ("Karussell", True), ("Hohe Acht", True),
                          ("Wippermann", True), ("Brünnchen", True), ("Pflanzgarten I", True), ("Schwalbenschwanz", True),
                          ("Döttinger Höhe", True), ("Antoniusbuche", False), ("Tiergarten", True)]}
FIELDS = (("entry", r"Entrance speed"), ("through", r"Speed through the corner"), ("arrive", r"Arrival time at the corner"),
          ("negotiate", r"Time to negotiate the corner"), ("exit", r"Exit time of the corner"))

def doc_dir():
    return os.environ.get("JM_CLASSIC_DOC") or os.path.expanduser("~/sgl/THU/DOC")

def guide_paths(track):
    d = doc_dir()
    lor = os.path.join(d, "trackGuidesAndSetups_LOR", f"CLASSIC_trackguide_{LOR[track]}_by_LOR.pdf") if track in LOR else None
    fs = os.path.join(d, "trackGuides_fs", f"fs_{FS[track]}TrackGuide.pdf") if track in FS else None
    return (lor if lor and os.path.isfile(lor) else None), (fs if fs and os.path.isfile(fs) else None)

def pdf_text(path):
    exe = shutil.which("pdftotext")
    if exe is None or path is None:
        return None
    try:
        return subprocess.run([exe, "-layout", path, "-"], capture_output=True, timeout=60, check=True).stdout.decode("utf-8", "replace")
    except (subprocess.SubprocessError, OSError):
        return None

def norm(s):
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode().lower()
    return re.sub(r"[^a-z0-9]", "", s)

def _kph(txt):
    """'approximately 170 mph [274 kph]' -> 274; '165-170 mph' -> the middle, converted."""
    m = re.search(r"\[(\d+)(?:\s*-\s*(\d+))?\s*kph\]", txt)
    if m:
        return (float(m.group(1)) + float(m.group(2) or m.group(1))) / 2
    m = re.search(r"(\d+)(?:\s*-\s*(\d+))?\s*mph", txt)
    if m:
        return 1.609344 * (float(m.group(1)) + float(m.group(2) or m.group(1))) / 2
    return None

def _secs(txt):
    """'5.63 seconds', '2:19.70 seconds', '1 minute 1.68 seconds', '7 minutes 39.26 seconds' -> seconds."""
    m = re.search(r"(?:(\d+)\s*(?::|minutes?)\s*)?(\d+(?:\.\d+)?)\s*sec", txt)
    if not m:
        return None
    return 60.0 * float(m.group(1) or 0) + float(m.group(2))

def _section_of(title, names):
    """The sim's section a guide heading names, or None. Matches without accents/case/punctuation, a leading 'the',
    and a plural (Lesmos <- 'THE FIRST LESMO')."""
    t = norm(title)
    t = t[3:] if t.startswith("the") and len(t) > 4 else t
    if t in ALIASES:
        return ALIASES[t]
    for n in names:
        k = norm(n); k = k[3:] if k.startswith("the") and len(k) > 4 else k
        stem = k[:-1] if k.endswith("s") and len(k) > 4 else k
        if k and (t == k or t.startswith(k) or k in t or (len(stem) >= 4 and stem in t)):
            return n
    return None

def parse_lor(text, names, track=None):
    """Corner entries, in lap order: dict(section, anchor, title, entry, through [km/h], arrive, negotiate, exit [s],
    advice). `anchor`: this entry opens the sim's section `section` (its start can pin the guide's clock)."""
    lines = text.replace("\f", "\n").split("\n")
    out = []; section = None; idx = [i for i, ln in enumerate(lines) if re.match(r"\s*Entrance speed\s*:", ln)]
    groups = GROUPS.get(track); g = -1

    heads = {}
    for i, ln in enumerate(lines):
        s = ln.strip().rstrip(":")
        if 2 < len(s) <= 40 and not s.endswith(".") and _section_of(s, names) and norm(s) in {norm(n) for n in names} | {norm("the " + n) for n in names}:
            heads[i] = _section_of(s, names)
    for q, i in enumerate(idx):
        e = {"title": "", "advice": "", "anchor": False}
        j = i - 1
        while j >= 0 and (not lines[j].strip() or lines[j].strip().isdigit()):
            j -= 1
        title = lines[j].strip().rstrip(":") if j >= 0 else ""
        e["title"] = title
        if groups is not None:
            if re.match(r"FIRST (CORNER|TURN)|THE MAIN STRAIGHT", title) and g + 1 < len(groups):
                g += 1; section = groups[g][0]; e["anchor"] = groups[g][1]
        else:
            prev = section
            for k in sorted(h for h in heads if h <= j):
                section = heads[k]
            sec = _section_of(title, names)
            if sec is not None and not re.search(r"CORNER", title):
                section = sec
            e["anchor"] = section is not None and section != prev
        e["section"] = section
        k = i
        for key, pat in FIELDS:
            while k < min(len(lines), i + 12) and not re.match(r"\s*" + pat + r"\s*:", lines[k]):
                k += 1
            if k < len(lines) and re.match(r"\s*" + pat + r"\s*:", lines[k]):
                v = lines[k].split(":", 1)[1]
                e[key] = _kph(v) if key in ("entry", "through") else _secs(v)
                k += 1
            else:
                e[key] = None
        nxt = idx[q + 1] if q + 1 < len(idx) else len(lines)
        stop = nxt - 1
        while stop > k and (not lines[stop].strip() or lines[stop].strip().isdigit()):
            stop -= 1
        body = [ln.strip() for ln in lines[k:stop]]
        body = [b for b in body if b and not b.startswith("Click here") and not b.startswith("LOR - ")
                and not re.match(r"^\*?This is when", b) and not (b.isdigit())]
        e["advice"] = re.sub(r"\s+", " ", " ".join(body)).strip()
        out.append(e)
    return out

def current_sections(track, laplen, fallback=()):
    """The sim's section table for `track` as it is NOW (track_sections.jl beside this file), scaled onto `laplen`: a
    replay's header carries the table of the day it was recorded (the Ring's before TRACKSEG-5 had Brünnchen before
    Eschbach). Falls back to `fallback` (the header's)."""
    try:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_sections.jl"), encoding="utf-8") as f:
            src = f.read()
        m = re.search(r'"' + re.escape(track) + r'"\s*=>\s*\(lap\s*=\s*([\d.]+),\s*secs\s*=\s*\[(.*?)\]\)', src, re.S)
        if not m:
            return list(fallback)
        k = laplen / float(m.group(1))
        return [(float(a) * k, b.replace('\\"', '"')) for a, b in re.findall(r'\(([\d.]+),\s*"((?:[^"\\]|\\.)*)"\)', m.group(2))]
    except OSError:
        return list(fallback)

def place(entries, lap, sections, laplen, guide_lap=None):
    """Give each entry `s_arrive`/`s_exit` on `lap` (an analyser Lap). The guide's clock is mapped piecewise linearly onto
    the driver's: at every entry that opens one of the sim's sections (`anchor`), the guide's arrival there (its braking
    point) is pinned to the driver's time at that section's start; the lap's ends pin 0 and the lap times."""
    start = {n: s for s, n in sections}
    t = lap.ch["time"]; d = lap.dist; dm = d[1] - d[0] if len(d) > 1 else 1.0

    def t_at(s):
        return t[max(0, min(len(t) - 1, int(round(s / dm))))]

    def s_at(tt):
        for k in range(1, len(t)):
            if t[k] >= tt:
                f = (tt - t[k - 1]) / (t[k] - t[k - 1]) if t[k] > t[k - 1] else 0.0
                return d[k - 1] + f * (d[k] - d[k - 1])
        return d[-1]
    last = max((e.get("exit") or 0.0) for e in entries) if entries else 0.0
    G = guide_lap if guide_lap else last + 2.0
    pins = [(0.0, 0.0)]
    for e in entries:
        if e.get("anchor") and e.get("arrive") is not None and e["section"] in start and start[e["section"]] > 1.0:
            pins.append((e["arrive"], t_at(start[e["section"]])))
    pins.append((G, t[-1]))
    mono = [pins[0]]
    for p in pins[1:]:
        if p[0] > mono[-1][0] and p[1] > mono[-1][1]:
            mono.append(p)

    def drv(g):
        for (g0, T0), (g1, T1) in zip(mono, mono[1:]):
            if g <= g1:
                return T0 + (g - g0) / (g1 - g0) * (T1 - T0)
        return mono[-1][1]
    for e in entries:
        for key, out in (("arrive", "s_arrive"), ("exit", "s_exit")):
            g = e.get(key)
            e[out] = None if g is None else s_at(drv(g))
    return entries

def compare(entries, lap):
    """The driver's numbers at each placed corner: entry = fastest in the 250 m before arrival, slowest between arrival
    and exit, time from arrival to exit."""
    v = lap.ch["kmh"]; t = lap.ch["time"]; dm = lap.dist[1] - lap.dist[0]
    rows = []
    for e in entries:
        a, b = e.get("s_arrive"), e.get("s_exit")
        if a is None or b is None:
            rows.append(dict(e, you_entry=None, you_min=None, you_time=None)); continue
        ka = int(a / dm); kb = max(ka + 1, int(b / dm)); kb = min(kb, len(v) - 1); ka = min(ka, kb - 1)
        pre = v[max(0, ka - int(250 / dm)):ka + 1]
        rows.append(dict(e, you_entry=max(pre) if pre else None, you_min=min(v[ka:kb + 1]), you_time=t[kb] - t[ka]))
    return rows

def _text(track, kind):
    """The guide's text: from JM_TRACKGUIDE_DIR (the AppImage's extracted copy) if set, else the PDF. (text, file)"""
    d = os.environ.get("JM_TRACKGUIDE_DIR")
    if d:
        p = os.path.join(d, f"{track}_{kind}.txt")
        if os.path.isfile(p):
            with open(p, encoding="utf-8") as f:
                return f.read(), p
        return None, None
    lor, fs = guide_paths(track)
    p = lor if kind == "lor" else fs
    return pdf_text(p), p

def load(track, names):
    """(entries, source) for `track`, or (None, reason)."""
    text, lor = _text(track, "lor")
    if lor is None:
        return None, f"no Lights Out Racing guide for this track under {os.environ.get('JM_TRACKGUIDE_DIR') or doc_dir() + '/trackGuidesAndSetups_LOR'}"
    if text is None:
        return None, "pdftotext (poppler-utils) is needed to read the track guides"
    m = re.search(r"replay is (\d+:\d+\.\d+)", text)
    src = f"Lights Out Racing guide ({os.path.basename(lor)})" + (f", replay lap {m.group(1)}" if m else "")
    return parse_lor(text, names, track), src

def guide_lap(text):
    """The guide replay's lap time (s), if the guide states it."""
    m = re.search(r"replay is (\d+):(\d+\.\d+)", text or "")
    return 60 * int(m.group(1)) + float(m.group(2)) if m else None

def fs_text(track):
    """The short fs guide's prose (machine-translated), or None."""
    t, _p = _text(track, "fs")
    if t is None:
        return None
    body = [ln.strip() for ln in t.split("\n") if ln.strip() and "translate.google" not in ln and not re.match(r"^\d+ of \d+", ln)
            and not re.search(r"\d+/\d+/\d+, \d+:\d+", ln) and ln.strip() not in ("back", "Home")]
    return re.sub(r"\s+", " ", " ".join(body)).strip()

if __name__ == "__main__":
    import sys
    if len(sys.argv) == 3 and sys.argv[1] == "--export":
        os.makedirs(sys.argv[2], exist_ok=True); n = 0
        for track in LOR:
            for kind, p in zip(("lor", "fs"), guide_paths(track)):
                t = pdf_text(p)
                if t:
                    with open(os.path.join(sys.argv[2], f"{track}_{kind}.txt"), "w", encoding="utf-8") as f:
                        f.write(t)
                    n += 1
        print(f"track guides: {n} exported to {sys.argv[2]}")
