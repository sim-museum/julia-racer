"""DOCGUIDES-2: julia's guides -- the track guide for each circuit, the racing glossary and the setup guide.

PO 2026-10-17 (after DOCGUIDES-1, which showed the 14 source PDFs page by page): only the CURRENT track's guide; every
guide condensed to what is needed (no provenance, no references to other games); the Racecraft A-Z a short glossary;
the setup guides one plain document, reached from the Garage tab. The documents are julia's own Markdown in
docs/guides_text/ (shipped with the launcher, so the AppImage needs nothing outside itself); the maps are drawn from the
.trk centrelines and the corner pictures are julia's own frames.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
TEXT_DIR = os.path.join(HERE, "docs", "guides_text")
TRACK_NAMES = {"spa": "Spa-Francorchamps", "monza": "Monza", "watglen": "Watkins Glen", "zandvoort": "Zandvoort",
               "nurburgring": "Nürburgring"}


def doc_path(name):
    return os.path.join(TEXT_DIR, name + ".md")


def track_doc(track):
    """The guide for a track key ("spa", ...), or None (the skidpad has none)."""
    p = doc_path("track_" + track)
    return p if os.path.isfile(p) else None


def glossary_doc():
    p = doc_path("glossary")
    return p if os.path.isfile(p) else None


def setup_doc():
    p = doc_path("setup")
    return p if os.path.isfile(p) else None


def show_markdown(browser, path):
    """Load a guide into a QTextBrowser: its images resolve against the guide's folder."""
    browser.setSearchPaths([os.path.dirname(path)])
    with open(path, encoding="utf-8") as fh:
        browser.setMarkdown(fh.read())
