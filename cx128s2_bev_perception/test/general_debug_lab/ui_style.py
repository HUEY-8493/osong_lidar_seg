"""Native presentation primitives; no dataset or perception logic."""
from PyQt5 import QtCore as C, QtGui as G, QtWidgets as W, QtSvg

PATHS={
 'menu':'M4 6h16M4 12h16M4 18h16',
 'dataset':'M4 5h6l2 3h8v12H4z M4 8h8',
 'experiments':'M9 3h6M10 3v7l-6 9q-1 2 2 2h12q3 0 2-2l-6-9V3M8 14h8',
 'compare':'M9 4H4v16h5M15 4h5v16h-5M12 3v18',
 'settings':'M4 7h16M4 17h16M8 4v6M16 14v6',
 'layers':'M12 3L2 8l10 5 10-5zM3 12l9 5 9-5M3 16l9 5 9-5',
 'focus':'M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5M9 12h6M12 9v6',
 'expand':'M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5',
 'fit':'M4 4h16v16H4zM8 12h8M12 8v8',
 'export':'M12 15V3M8 7l4-4 4 4M4 14v6h16v-6',
 'play':'M8 4l12 8-12 8z',
 'pause':'M8 5v14M16 5v14',
 'prev':'M7 5v14M18 5l-9 7 9 7z',
 'next':'M17 5v14M6 5l9 7-9 7z',
 'chevron':'M9 5l7 7-7 7',
 'down':'M6 9l6 6 6-6',
 'more':'M5 12h.1M12 12h.1M19 12h.1',
 'check':'M5 12l4 4L19 6',
 'close':'M6 6l12 12M18 6L6 18',
 'search':'M10 3a7 7 0 1 0 0 14 7 7 0 0 0 0-14M15 15l6 6',
 'refresh':'M20 8a8 8 0 1 0 0 8M20 3v5h-5',
 'info':'M12 11v6M12 7h.01M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20',
 'clock':'M12 7v5l3 2M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20',
 'arrow':'M4 12h16M14 6l6 6-6 6',
}
def icon(name,color='#65645e'):
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><path d="{PATHS[name]}" fill="none" stroke="{color}" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/></svg>'
    renderer=QtSvg.QSvgRenderer(C.QByteArray(svg.encode()));pix=G.QPixmap(48,48);pix.fill(C.Qt.transparent);p=G.QPainter(pix);renderer.render(p);p.end();pix.setDevicePixelRatio(2);return G.QIcon(pix)
def button(text='',glyph=None,callback=None,role='',checkable=False):
    b=W.QPushButton(text);b.setCursor(C.Qt.PointingHandCursor);b.setCheckable(checkable)
    if glyph:b.setIcon(icon(glyph));b.setIconSize(C.QSize(18,18))
    if role:b.setProperty('role',role)
    if callback:b.clicked.connect(callback)
    return b
def label(text='',role='body',wrap=False):
    l=W.QLabel(text);l.setProperty('role',role);l.setWordWrap(wrap)
    if wrap:l.setMinimumWidth(0)
    l.setTextInteractionFlags(C.Qt.TextSelectableByMouse);return l
def line():
    frame=W.QFrame();frame.setObjectName('separator');frame.setFixedHeight(1);return frame
def vertical(widget=None,margins=(0,0,0,0),spacing=12):
    widget=widget or W.QWidget();layout=W.QVBoxLayout(widget);layout.setContentsMargins(*margins);layout.setSpacing(spacing);return widget,layout

def apply(window):
    W.QApplication.instance().setStyle('Fusion')
    window.setFont(G.QFont('Lato',11))
    window.setStyleSheet(STYLE)

STYLE='''
QMainWindow, QDialog {background:#faf9f6; color:#292a27;}
QWidget {font-family:"Lato"; font-size:15px; color:#292a27;}
QWidget#sidebar {background:#efeee9;}
QWidget#header, QWidget#timeline {background:#faf9f6;}
QWidget#inspector, QWidget#tunePanel {background:#faf9f6;}
QLabel {background:transparent; border:0;}
QLabel[role="brand"] {font-size:17px; font-weight:700; letter-spacing:.2px;}
QLabel[role="title"] {font-size:26px; font-weight:700;}
QLabel[role="subtitle"] {font-size:17px; font-weight:600;}
QLabel[role="eyebrow"] {font-size:11px; color:#85867e; font-weight:700; letter-spacing:1px;}
QLabel[role="muted"] {color:#73766b; font-size:13px;}
QLabel[role="metric"] {font-size:16px; font-weight:500;}
QLabel[role="success"] {color:#34775b;}
QLabel[role="danger"] {color:#a34e3e;}
QWidget#failure {background:#f5eae3; border-radius:9px;}
QWidget#failure QLabel {color:#914c38;}
QFrame#separator {background:#e7e6e0; border:0;}
QPushButton {background:transparent; border:1px solid transparent; padding:8px 12px; border-radius:8px; min-height:20px; font-weight:500;}
QPushButton:hover {background:#eae9e3;}
QPushButton:pressed {background:#dedfd6;}
QPushButton:disabled {color:#a5a69d;}
QPushButton::menu-indicator {image:none; width:0; height:0;}
QPushButton[role="nav"] {text-align:left; padding:11px 13px; color:#5e6058; border-radius:8px;}
QPushButton[role="nav"]:checked {background:#e0e1d8; color:#292d26;}
QPushButton[role="segment"] {padding:7px 16px; color:#71746b; border-radius:7px;}
QPushButton[role="segment"]:checked {background:#ffffff; color:#295f49; border:1px solid #e4e4dc;}
QWidget#segments {background:#efeee9; border-radius:9px;}
QPushButton[role="primary"] {background:#3f6f58; color:#ffffff; padding:9px 15px;}
QPushButton[role="primary"]:hover {background:#345f4b;}
QPushButton[role="secondary"] {border:1px solid #dedfd5; background:#faf9f6;}
QPushButton[role="small"] {padding:5px 8px; font-size:12px;}
QPushButton[role="icon"] {padding:8px; min-width:18px;}
QPushButton[role="gate"] {text-align:left; padding:10px 0; border-radius:0; font-size:13px;}
QComboBox {background:#f2f1eb; border:1px solid transparent; border-radius:8px; padding:8px 27px 8px 11px; min-height:19px; selection-background-color:#dce5da;}
QComboBox:hover {background:#eaeae2;}
QComboBox::drop-down {border:0; width:0px;}
QComboBox::down-arrow {image:none; width:0px; height:0px; border:0;}
QComboBox QAbstractItemView {background:#faf9f6; border:1px solid #dedfd5; selection-background-color:#e2e9de; color:#292a27; padding:5px; outline:0;}
QLineEdit, QSpinBox, QDoubleSpinBox {background:#f0efe9; border:1px solid transparent; border-radius:7px; padding:7px 9px; selection-background-color:#cbdcc9; min-height:18px;}
QLineEdit:focus,QSpinBox:focus,QDoubleSpinBox:focus {border:1px solid #9dae97;}
QAbstractSpinBox::up-button,QAbstractSpinBox::down-button {width:0; border:0;}
QCheckBox {spacing:12px; padding:7px;}
QCheckBox::indicator {width:16px; height:16px; border-radius:4px; background:#e6e7df; border:1px solid #c6cbbf;}
QCheckBox::indicator:checked {background:#46765d; border:3px solid #c9d8c5;}
QScrollArea {background:transparent; border:0;}
QScrollArea > QWidget > QWidget {background:transparent;}
QScrollBar:vertical {background:transparent; width:7px; margin:3px 0;}
QScrollBar::handle:vertical {background:#d5d6ce; border-radius:3px; min-height:35px;}
QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical {height:0;}
QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical {background:transparent;}
QScrollBar:horizontal {background:transparent; height:7px;}
QScrollBar::handle:horizontal {background:#d5d6ce; border-radius:3px; min-width:35px;}
QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal {width:0;}
QSlider::groove:horizontal {height:3px; background:#dfe1d6; border-radius:1px;}
QSlider::sub-page:horizontal {background:#71957b; border-radius:1px;}
QSlider::handle:horizontal {width:12px; height:12px; margin:-5px 0; border-radius:7px; background:#3f7056; border:2px solid #faf9f6;}
QTableWidget,QTreeWidget {background:transparent; border:0; gridline-color:#e9e9e1; outline:0; selection-background-color:#e6ebdf; selection-color:#292a27;}
QTableWidget::item {padding:10px 8px; border:0;}
QTableWidget::item:selected {background:#e9ede3; color:#292a27;}
QHeaderView::section {background:#f1f1eb; color:#74776d; border:0; padding:10px 8px; font-size:12px; font-weight:600;}
QTableCornerButton::section {background:#f1f1eb; border:0;}
QPlainTextEdit {border:1px solid #e3e5dc; border-radius:8px; background:#f3f3ed; padding:12px; font-family:"DejaVu Sans Mono";font-size:12px;}
QListWidget {background:transparent; border:0; outline:0;}
QListWidget::item {padding:14px 8px; border-bottom:1px solid #e9e9e1;}
QListWidget::item:selected {background:#e4e8dc; color:#293d2c; border-radius:6px;}
QTabWidget::pane {border:0; background:transparent;}
QTabBar::tab {background:transparent; border:0; padding:12px 16px; color:#787d70;}
QTabBar::tab:selected {color:#355f46; border-bottom:2px solid #4e7959;}
QSplitter::handle {background:transparent; width:8px;}
QMenu {background:#faf9f6; border:1px solid #e0e2d7; padding:5px;}
QMenu::item {padding:10px 18px; border-radius:5px;}
QMenu::item:selected {background:#e4e9dd;}
QToolTip {color:#e5e8df; background:#30352d; border:0; padding:7px; font-size:12px;}
QStatusBar {background:#faf9f6; color:#7e8178; border:0; font-size:11px;}
QStatusBar::item {border:0;}
QWidget#canvas,QWidget#canvasHeader,QWidget#canvasFooter {background:#191d1b;}
QWidget#canvas QLabel {color:#bdc6be;}
QWidget#canvas QLabel[role="eyebrow"] {color:#85958a;}
QWidget#canvas QPushButton {color:#c7d1c8;}
QWidget#canvas QPushButton:hover {background:#2a312b;}
QWidget#canvas QComboBox {background:#282f29; color:#d6dfd3;}
QWidget#canvas QGraphicsView {border:0; background:#191d1b;}
'''

class ComboBox(W.QComboBox):
    """A vector chevron replaces platform-dependent native arrow glyphs."""
    def __init__(self,parent=None):
        super().__init__(parent);self.currentTextChanged.connect(self.setToolTip)
    def paintEvent(self,event):
        option=W.QStyleOptionComboBox();self.initStyleOption(option);option.currentText=self.fontMetrics().elidedText(option.currentText,C.Qt.ElideMiddle,max(30,self.width()-45))
        painter=W.QStylePainter(self);painter.drawComplexControl(W.QStyle.CC_ComboBox,option);painter.drawControl(W.QStyle.CE_ComboBoxLabel,option);painter.end()
        p=G.QPainter(self);p.setRenderHint(G.QPainter.Antialiasing);p.setPen(G.QPen(G.QColor('#85897c'),1.4,C.Qt.SolidLine,C.Qt.RoundCap,C.Qt.RoundJoin));x=self.width()-17;y=self.height()/2;p.drawPolyline(G.QPolygonF([C.QPointF(x-3,y-1),C.QPointF(x,y+2),C.QPointF(x+3,y-1)]))
    def showPopup(self):
        width=min(self.screen().availableGeometry().width()-60,max(260,max((self.fontMetrics().horizontalAdvance(self.itemText(i))+40 for i in range(self.count())),default=260)))
        self.view().setMinimumWidth(width);super().showPopup()
