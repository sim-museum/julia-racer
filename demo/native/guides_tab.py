"""DOCGUIDES-2: the launcher's Guides tab -- the CURRENT track's guide and the racing glossary.

The list on the left holds two entries: the guide for the track chosen on the Race tab (it follows the choice) and the
glossary. The document on the right is julia's own Markdown (guides.py), with its map and corner pictures; Ctrl+wheel
or +/- zooms the text. The setup guide lives on the Garage tab.
"""
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QHBoxLayout, QListWidget, QListWidgetItem, QSplitter, QTextBrowser, QWidget

import guides


class GuidesTab(QWidget):
    def __init__(self, track_key=lambda: "spa", parent=None):
        super().__init__(parent)
        self.track_key = track_key              # the Race tab's current track key ("spa", "skidpad", ...)
        lay = QHBoxLayout(self)
        split = QSplitter()
        lay.addWidget(split)
        self.list = QListWidget()
        self.list.setObjectName("guidelist")
        self.list.setMinimumWidth(200)
        self.list.currentItemChanged.connect(self._pick)
        split.addWidget(self.list)
        self.view = QTextBrowser()
        self.view.setObjectName("guide")
        self.view.setOpenExternalLinks(False)
        split.addWidget(self.view)
        split.setStretchFactor(1, 4)
        self.fill()

    def fill(self):
        """Rebuild the list for the current track (called when the Race tab's track changes)."""
        key = self.track_key()
        self.list.blockSignals(True)
        self.list.clear()
        items = []
        p = guides.track_doc(key)
        if p:
            items.append((f"Track guide: {guides.TRACK_NAMES.get(key, key)}", p))
        g = guides.glossary_doc()
        if g:
            items.append(("Racing glossary", g))
        for title, path in items:
            it = QListWidgetItem(title)
            it.setData(Qt.ItemDataRole.UserRole, path)
            self.list.addItem(it)
        self.list.blockSignals(False)
        if self.list.count():
            self.list.setCurrentRow(0)
            self._pick(self.list.item(0))
        else:
            self.view.setPlainText("No guide for this track.")

    def _pick(self, cur, _prev=None):
        if cur is None or cur.data(Qt.ItemDataRole.UserRole) is None:
            return
        guides.show_markdown(self.view, cur.data(Qt.ItemDataRole.UserRole))

    def wheelEvent(self, e):
        if e.modifiers() & Qt.KeyboardModifier.ControlModifier:
            self.view.zoomIn(1) if e.angleDelta().y() > 0 else self.view.zoomOut(1)
        else:
            super().wheelEvent(e)

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            self.view.zoomIn(1)
        elif e.key() == Qt.Key.Key_Minus:
            self.view.zoomOut(1)
        else:
            super().keyPressEvent(e)
