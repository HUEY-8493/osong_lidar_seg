#!/usr/bin/env python3
"""Native Qt offline debugger. No HTTP listener or embedded browser."""
import argparse, bisect, fcntl, html, json, math, os, signal, sys, time, uuid
from pathlib import Path
from PyQt5 import QtCore as C, QtGui as G, QtWidgets as W
from engine import Lab, dataset, dump, safe
import ui_style as ui
import bev_view as bev
View=bev.View

class Worker(C.QThread):
    done = C.pyqtSignal(object, object)
    def __init__(self, function, parent):
        super().__init__(parent); self.function = function
    def run(self):
        try: self.done.emit(self.function(), None)
        except Exception as e: self.done.emit(None, str(e))

class Timeline(W.QSlider):
    """Marks only events present in saved traces; no inferred physical identity."""
    def __init__(self):
        super().__init__(C.Qt.Horizontal);self.events=[]
    def paintEvent(self,event):
        painter=G.QPainter(self);painter.setRenderHint(G.QPainter.Antialiasing);left=8;right=self.width()-8;y=10;span=max(1,self.maximum()-self.minimum());x=left+(right-left)*(self.value()-self.minimum())/span
        painter.setPen(G.QPen(G.QColor('#dde1d5'),3,C.Qt.SolidLine,C.Qt.RoundCap));painter.drawLine(C.QPointF(left,y),C.QPointF(right,y));painter.setPen(G.QPen(G.QColor('#74927a'),3,C.Qt.SolidLine,C.Qt.RoundCap));painter.drawLine(C.QPointF(left,y),C.QPointF(x,y))
        painter.setBrush(G.QColor('#476c52'));painter.setPen(G.QPen(G.QColor('#faf9f6'),2));painter.drawEllipse(C.QPointF(x,y),6,6)
        for entry in self.events:
            color=G.QColor('#b87863' if any(k in entry['kind'] for k in ['lost','reject','DORMANT']) else '#88a188');color.setAlpha(130);painter.setPen(color);ex=left+int((right-left)*(entry['frame']-1)/max(1,self.maximum()-1));painter.drawLine(ex,22,ex,25)

class Window(W.QMainWindow):
    def __init__(self,lab):
        super().__init__();self.lab=lab;self.workers=set();self.busy=False;self.pending=False;self.data={};self.states=[{},{}];self.generation=0;self.params=[];self.catalog=[];self.selected_track=None;self.event_rows=[];self.event_revision=0;self.run_to_open=None;self.closing=False;self.pending_whatif=None
        self.mode='Explore';self.focused=False;self.syncing=False;self.current_value={};self.selected_cluster=None;self.selected_side=0;self._displayed_runs=[None,None];self.selection_source=None;self.fit_mode='Selected';self.lifecycle_key=None;self.lifecycle_revision=0;self.restored_cameras=None;self._view_ready=False
        self.setWindowTitle('General · Perception Lab');self.resize(1600,1000);self.setMinimumSize(960,680);ui.apply(self)
        shell=W.QWidget();self.setCentralWidget(shell);outer=W.QHBoxLayout(shell);outer.setContentsMargins(0,0,0,0);outer.setSpacing(0)
        self.sidebar=W.QWidget();self.sidebar.setObjectName('sidebar');self.sidebar.setFixedWidth(202);sl=W.QVBoxLayout(self.sidebar);sl.setContentsMargins(16,24,16,18);sl.setSpacing(8);outer.addWidget(self.sidebar)
        brandrow=W.QHBoxLayout();self.brand=ui.label('General','brand');brandrow.addWidget(self.brand,1);self.collapse=ui.button(glyph='menu',callback=self.toggle_sidebar,role='icon');self.collapse.setToolTip('Collapse sidebar');brandrow.addWidget(self.collapse);sl.addLayout(brandrow)
        self.brand_sub=ui.label('PERCEPTION LAB','eyebrow');sl.addWidget(self.brand_sub);sl.addSpacing(30);self.nav={}
        for name,glyph,callback in [('Dataset','dataset',self.show_datasets),('Experiments','experiments',self.show_experiments),('Comparisons','compare',lambda:self.set_mode('Compare')),('Settings','settings',self.show_settings)]:
            b=ui.button(name,glyph,callback,'nav',True);b.setToolTip(name);sl.addWidget(b);self.nav[name]=b
        self.nav['Dataset'].setChecked(True);sl.addStretch();sl.addWidget(ui.line());self.offline=ui.label('●  Offline workspace','muted');sl.addWidget(self.offline);self.sidebar_note=ui.label('Your datasets stay local.','muted');sl.addWidget(self.sidebar_note)
        self.content=W.QWidget();outer.addWidget(self.content,1);cl=W.QVBoxLayout(self.content);cl.setContentsMargins(24,18,24,8);cl.setSpacing(16)
        self.header=W.QWidget();self.header.setObjectName('header');hl=W.QHBoxLayout(self.header);hl.setContentsMargins(0,0,0,0);hl.setSpacing(12);cl.addWidget(self.header)
        self.analysis_mode=ui.ComboBox();self.analysis_mode.addItems(['General','Crane']);self.analysis_mode.currentTextChanged.connect(self.set_analysis_mode);hl.addWidget(self.analysis_mode)
        self.datasets=ui.ComboBox();self.datasets.setToolTip('Prepared dataset');self.datasets.setMinimumWidth(200);self.datasets.setMaximumWidth(240);hl.addWidget(self.datasets)
        self.runs=[ui.ComboBox(),ui.ComboBox()]
        for box in self.runs:
            box.setMinimumWidth(100);box.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Fixed);box.setSizeAdjustPolicy(W.QComboBox.AdjustToMinimumContentsLengthWithIcon);box.currentIndexChanged.connect(self.run_changed)
        self.runs[0].setMaximumWidth(280);hl.addWidget(self.runs[0],1);hl.addStretch()
        segments=W.QWidget();segments.setObjectName('segments');seg=W.QHBoxLayout(segments);seg.setContentsMargins(3,3,3,3);seg.setSpacing(2);self.modes={};self.mode_group=W.QButtonGroup(self)
        for name in ['Explore','Compare','Tune']:
            b=ui.button(name,callback=lambda _,m=name:self.set_mode(m),role='segment',checkable=True);self.modes[name]=b;self.mode_group.addButton(b);seg.addWidget(b)
        hl.addWidget(segments);self.replay_state=ui.label('●  Ready','muted');self.replay_state.setMinimumWidth(70);hl.addWidget(self.replay_state)
        self.workspace=W.QSplitter();self.workspace.setChildrenCollapsible(False);self.workspace.setHandleWidth(16);cl.addWidget(self.workspace,1)
        self.canvas=W.QWidget();self.canvas.setObjectName('canvas');cv=W.QVBoxLayout(self.canvas);cv.setContentsMargins(0,0,0,0);cv.setSpacing(0);self.workspace.addWidget(self.canvas)
        bar=W.QWidget();bar.setObjectName('canvasHeader');vl=W.QHBoxLayout(bar);vl.setContentsMargins(18,12,12,12);vl.setSpacing(4)
        self.canvas_title=ui.label('WORLD XY','eyebrow');vl.addWidget(self.canvas_title);vl.addStretch()
        self.angular_button=ui.button('Depth',callback=self.toggle_angular,role='small',checkable=True);vl.addWidget(self.angular_button)
        self.layer_button=ui.button(glyph='layers',callback=self.show_layers,role='icon');self.layer_button.setToolTip('Layers');vl.addWidget(self.layer_button)
        self.fit_button=ui.button('Fit',glyph='fit',callback=self.fit_views,role='small');self.fit_button.setIcon(ui.icon('fit','#b8c7b9'));self.fit_button.setToolTip('Fit Selected · falls back to visible only when nothing is selected');vl.addWidget(self.fit_button)
        self.fit_more=ui.button(glyph='down',callback=self.show_fit,role='icon');self.fit_more.setIcon(ui.icon('down','#b8c7b9'));self.fit_more.setToolTip('Fit target / Fit Visible / Fit ROI');vl.addWidget(self.fit_more)
        self.follow_button=ui.button('Follow',glyph='arrow',callback=self.follow_changed,role='small',checkable=True);self.follow_button.setIcon(ui.icon('arrow','#b8c7b9'));self.follow_button.setToolTip('Follow selected track · keep zoom; dragging pauses');vl.addWidget(self.follow_button)
        for tip,glyph,callback in [('Focus selected track · keep zoom','focus',self.focus_track),('Export image','export',self.export)]:
            b=ui.button(glyph=glyph,callback=callback,role='icon');b.setIcon(ui.icon(glyph,'#b8c7b9'));b.setToolTip(tip);vl.addWidget(b)
        self.layer_button.setIcon(ui.icon('layers','#b8c7b9'))
        self.full_button=ui.button(glyph='expand',callback=self.toggle_focus,role='icon');self.full_button.setIcon(ui.icon('expand','#b8c7b9'));self.full_button.setToolTip('Focus view · F11 / Esc');vl.addWidget(self.full_button);cv.addWidget(bar)
        self.views=[View(),View(),View()];self.view_split=W.QSplitter();self.view_split.setHandleWidth(1);cv.addWidget(self.view_split,1);self.view_hosts=[];self.view_labels=[]
        for index in range(2):
            host=W.QWidget();lh=W.QVBoxLayout(host);lh.setContentsMargins(0,0,0,0);lh.setSpacing(0)
            top=W.QWidget();th=W.QHBoxLayout(top);th.setContentsMargins(18,0,14,10);name=ui.label('BASELINE' if index==0 else 'CANDIDATE','eyebrow');th.addWidget(name);self.view_labels.append(top);top.setFixedHeight(50)
            if index==1:th.addWidget(self.runs[1],1)
            lh.addWidget(top)
            if index==0:
                self.view_stack=W.QStackedWidget();self.view_stack.addWidget(self.views[0]);self.view_stack.addWidget(self.views[2]);lh.addWidget(self.view_stack,1)
            else:lh.addWidget(self.views[1],1)
            self.view_hosts.append(host);self.view_split.addWidget(host)
        for v in self.views[:2]:
            v.selected.connect(lambda kind,ident,index=self.views.index(v):self.select_in_view(index,kind,ident));v.manual_pan.connect(self.pause_follow);v.coordinates.connect(lambda x,y:self.cursor_label.setText(f'{x:.2f}, {y:.2f} m'));v.camera_changed.connect(lambda source=v:self.sync_camera(source))
        footer=W.QWidget();footer.setObjectName('canvasFooter');fl=W.QHBoxLayout(footer);fl.setContentsMargins(18,10,18,12);self.point_label=ui.label('Loading frame…','muted');fl.addWidget(self.point_label,1);self.cursor_label=ui.label('World XY · m','muted');fl.addWidget(self.cursor_label);cv.addWidget(footer);self.camera_notice=ui.label('Fit selected · observed bounds first','muted',True);self.camera_notice.setContentsMargins(18,0,18,8);cv.addWidget(self.camera_notice);self.canvas.installEventFilter(self)
        self.right=W.QStackedWidget();self.right.setMinimumWidth(310);self.workspace.addWidget(self.right)
        # Primary inspector uses readable, ordered summaries. Full tables live in Advanced.
        inspector=W.QWidget();inspector.setObjectName('inspector');il=W.QVBoxLayout(inspector);il.setContentsMargins(0,0,0,0);il.setSpacing(12);self.right.addWidget(inspector)
        ir=W.QHBoxLayout();ir.addWidget(ui.label('Track inspector','subtitle'));ir.addStretch();advanced=ui.button(glyph='more',callback=self.show_advanced,role='icon');advanced.setToolTip('Advanced values / raw trace');ir.addWidget(advanced);il.addLayout(ir)
        self.track_choice=ui.ComboBox();self.track_choice.currentIndexChanged.connect(self.choose_track);il.addWidget(self.track_choice)
        scroll=W.QScrollArea();scroll.setWidgetResizable(True);scroll.setHorizontalScrollBarPolicy(C.Qt.ScrollBarAlwaysOff);self.inspector_body=W.QWidget();self.inspector_layout=W.QVBoxLayout(self.inspector_body);self.inspector_layout.setContentsMargins(0,0,8,0);self.inspector_layout.setSpacing(12);scroll.setWidget(self.inspector_body);il.addWidget(scroll,1)
        self.track_heading=ui.label('Select a track','title');self.track_state=ui.label('Choose a track in the scene or list.','muted',True);self.inspector_layout.addWidget(self.track_heading);self.inspector_layout.addWidget(self.track_state)
        self.decision=ui.label('No recorded decision','subtitle',True);self.association=ui.label('','muted',True);self.inspector_layout.addWidget(self.decision);self.inspector_layout.addWidget(self.association)
        self.failure=W.QWidget();self.failure.setObjectName('failure');fel=W.QVBoxLayout(self.failure);fel.setContentsMargins(14,14,14,14);fel.setSpacing(6);self.failure_heading=ui.label('','subtitle',True);self.failure_detail=ui.label('','body',True);fel.addWidget(self.failure_heading);fel.addWidget(self.failure_detail);self.inspector_layout.addWidget(self.failure);self.failure.hide()
        self.failure_scope=ui.label('','muted',True);self.inspector_layout.addWidget(self.failure_scope);self.inspector_layout.addWidget(ui.line())
        self.summary=ui.label('','body',True);self.summary.setTextFormat(C.Qt.RichText);self.inspector_layout.addWidget(self.summary)
        self.diff_label=ui.label('','muted',True);self.inspector_layout.addWidget(self.diff_label)
        self.inspector_layout.addWidget(ui.line());gr=W.QHBoxLayout();gr.addWidget(ui.label('Decision gates','subtitle'));gr.addStretch();details=ui.button('Details',callback=self.show_gate_details,role='small');gr.addWidget(details);self.inspector_layout.addLayout(gr)
        self.gate_calls=ui.ComboBox();self.gate_calls.currentIndexChanged.connect(self.fill_gates);self.inspector_layout.addWidget(self.gate_calls)
        self.gate_rows=W.QWidget();self.gate_rows_layout=W.QVBoxLayout(self.gate_rows);self.gate_rows_layout.setContentsMargins(0,0,0,0);self.gate_rows_layout.setSpacing(0);self.inspector_layout.addWidget(self.gate_rows)
        self.skipped=ui.button('',callback=self.toggle_skipped,role='small',checkable=True);self.inspector_layout.addWidget(self.skipped);self.gate_notice=ui.label('','muted',True);self.inspector_layout.addWidget(self.gate_notice);self.inspector_layout.addStretch()
        raw_link=ui.button('Advanced · raw values',glyph='chevron',callback=self.show_advanced,role='small');self.inspector_layout.addWidget(raw_link)
        # Existing parameter model and edit cells are retained, metadata is contextual.
        tune=W.QWidget();tune.setObjectName('tunePanel');tl=W.QVBoxLayout(tune);tl.setContentsMargins(0,0,0,0);tl.setSpacing(14);tune_scroll=W.QScrollArea();tune_scroll.setWidgetResizable(True);tune_scroll.setHorizontalScrollBarPolicy(C.Qt.ScrollBarAlwaysOff);tune_scroll.setWidget(tune);self.right.addWidget(tune_scroll)
        tl.addWidget(ui.label('Parameter lab','title'));tl.addWidget(ui.label('Experiment overrides. Operating config stays unchanged.','muted',True))
        self.param_search=W.QLineEdit();self.param_search.setPlaceholderText('Find a parameter…');self.param_search.textChanged.connect(self.filter_params);tl.addWidget(self.param_search)
        self.table=W.QTableWidget(0,10);self.table.setHorizontalHeaderLabels(['Parameter','Current','Override','Unit','C++ default','Functions','Range','Regenerate','What-if','Replay']);self.table.verticalHeader().hide();self.table.setShowGrid(False);self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);self.table.setSelectionMode(W.QAbstractItemView.SingleSelection);self.table.verticalHeader().setDefaultSectionSize(58);self.table.setWordWrap(True);self.table.setTextElideMode(C.Qt.ElideNone)
        for col in range(3,10):self.table.hideColumn(col)
        self.table.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.Stretch);self.table.horizontalHeader().setSectionResizeMode(1,W.QHeaderView.Fixed);self.table.horizontalHeader().setSectionResizeMode(2,W.QHeaderView.Fixed);self.table.setColumnWidth(1,90);self.table.setColumnWidth(2,90);self.table.itemSelectionChanged.connect(self.parameter_info);tl.addWidget(self.table,1)
        self.param_detail=ui.label('Select a parameter to see source, unit and experiment scope.','muted',True);tl.addWidget(self.param_detail)
        detailsrow=W.QHBoxLayout();self.range_start=W.QSpinBox();self.range_end=W.QSpinBox();detailsrow.addWidget(ui.label('Detail frames','muted'));detailsrow.addWidget(self.range_start);detailsrow.addWidget(ui.label('to','muted'));detailsrow.addWidget(self.range_end);tl.addLayout(detailsrow)
        self.label=W.QLineEdit('qt_experiment');self.label.setPlaceholderText('Experiment name');tl.addWidget(self.label)
        actionrow=W.QHBoxLayout();actionrow.addWidget(ui.button('Frame what-if',callback=lambda:self.start('what-if'),role='primary'));actionrow.addWidget(ui.button('Dataset replay',callback=lambda:self.start('dataset'),role='secondary'));more=ui.button(glyph='more',role='icon');menu=W.QMenu(more);menu.addAction('Full replay',lambda:self.start('full'));menu.addAction('All parameter metadata',self.show_parameter_details);more.setMenu(menu);actionrow.addWidget(more);tl.addLayout(actionrow)
        self.tune_result=ui.label('What-if evaluates frozen detection only. Dataset replay rebuilds tracking history.','muted',True);tl.addWidget(self.tune_result);tl.addWidget(ui.line());self.tune_context=ui.label('No recorded frame selected.','body',True);tl.addWidget(self.tune_context);tl.addStretch()
        # One quiet, bottom timeline; frame navigation is never stacked in the header.
        self.timeline=W.QWidget();self.timeline.setObjectName('timeline');bt=W.QVBoxLayout(self.timeline);bt.setContentsMargins(0,0,0,0);bt.setSpacing(8);cl.addWidget(self.timeline)
        lifecycle_top=W.QHBoxLayout();self.lifecycle_toggle=ui.button('Track lifecycle',callback=self.toggle_lifecycle,role='small',checkable=True);lifecycle_top.addWidget(self.lifecycle_toggle);self.lifecycle_label=ui.label('All tracks · major events','muted',True);lifecycle_top.addWidget(self.lifecycle_label,1);self.detail_replay=ui.button('Detail replay',callback=lambda:self.start('dataset'),role='small');self.detail_replay.hide();lifecycle_top.addWidget(self.detail_replay);lifecycle_top.addWidget(ui.button('All tracks',callback=self.show_all_events,role='small'));bt.addLayout(lifecycle_top);self.lifecycle_plot=bev.LifecycleTimeline();self.lifecycle_plot.seek.connect(self.frame_seek);self.lifecycle_plot.range_selected.connect(self.select_replay_range);bt.addWidget(self.lifecycle_plot);self.lifecycle_plot.hide()
        self.slider=Timeline();self.slider.setMinimumHeight(28);self.slider.setRange(1,1);bt.addWidget(self.slider)
        transport=W.QHBoxLayout();transport.setSpacing(8);self.frame_caption=ui.label('Frame','muted');transport.addWidget(self.frame_caption);self.frame=W.QSpinBox();self.frame.setRange(1,1);self.frame.setMaximumWidth(82);transport.addWidget(self.frame);self.time_label=ui.label('','muted');transport.addWidget(self.time_label);transport.addStretch()
        transport.addWidget(ui.button(glyph='prev',callback=lambda:self.frame.setValue(self.frame.value()-1),role='icon'));self.play=ui.button(glyph='play',callback=self.play_icon,role='secondary',checkable=True);self.play.setToolTip('Play / pause');transport.addWidget(self.play);transport.addWidget(ui.button(glyph='next',callback=lambda:self.frame.setValue(self.frame.value()+1),role='icon'))
        self.speed=ui.ComboBox();self.speed.addItems(['0.25×','0.5×','1×','2×','4×']);self.speed.setCurrentIndex(2);self.speed.setFixedWidth(78);transport.addWidget(self.speed);transport.addStretch()
        self.seek_time=W.QDoubleSpinBox();self.seek_time.setDecimals(3);self.seek_time.setSuffix(' s');self.seek_time.setMaximumWidth(105);transport.addWidget(self.seek_time);transport.addWidget(ui.button('Go',callback=self.seek_seconds,role='small'));transport.addWidget(ui.button('Events',glyph='clock',callback=self.show_events,role='small'));bt.addLayout(transport)
        self.slider.valueChanged.connect(self.frame.setValue);self.frame.valueChanged.connect(self.slider.setValue);self.frame.valueChanged.connect(self.request_frame)
        # Popovers and secondary views preserve controls without occupying the canvas.
        self.layer_dialog=self.dialog('Scene layers',400,640);ll=self.layer_dialog.layout();ll.addWidget(ui.label('Layers · display only','subtitle'));self.layer_tabs=W.QTabWidget();self.layers={}
        groups=[('PointCloud',['Points','Foreground','ROI non-foreground','ROI inside','Outside ROI','Background','Ignore / Excluded','Unknown points']),('Objects',['Clusters','Vehicle-like','Matched','Rejected components','Unknown-like','Unclassified metadata','Accepted classification','Unassociated']),('Tracking',['Tracks','Actual observations','Prediction-only','LOST lifecycle','DORMANT','Public VEHICLE','Non-public','Rejected tracks','Geometry','Memory Geometry','KF Prediction']),('Context',['ROI','FOV','Grid'])]
        for title,names in groups:
            layer_scroll=W.QScrollArea();layer_scroll.setWidgetResizable(True);layer_scroll.setHorizontalScrollBarPolicy(C.Qt.ScrollBarAlwaysOff);lb=W.QWidget();lbs=W.QVBoxLayout(lb);lbs.setContentsMargins(0,0,0,0);lbs.setSpacing(0);lbs.setAlignment(C.Qt.AlignTop)
            for name in names:
                box=W.QCheckBox(name);box.setChecked(bev.DEFAULTS.get(name,False));box.toggled.connect(self.filter_changed);lbs.addWidget(box);self.layers[name]=box
                if name in bev.UNSUPPORTED:box.setEnabled(False);box.setToolTip(bev.UNSUPPORTED[name])
            layer_scroll.setWidget(lb);self.layer_tabs.addTab(layer_scroll,{'PointCloud':'Points','Objects':'Objects','Tracking':'Tracks','Context':'Context'}[title])
        ll.addWidget(self.layer_tabs,1);actions=W.QHBoxLayout();actions.addWidget(ui.button('Show all',callback=lambda:self.set_all_layers(True),role='small'));self.focus_only_button=ui.button('Focus only',callback=self.filter_changed,role='secondary',checkable=True);actions.addWidget(self.focus_only_button);actions.addWidget(ui.button('Reset',callback=self.reset_layers,role='small'));ll.addLayout(actions);ll.addWidget(ui.label('Green: observed/selected · violet: prediction\nAmber: memory · red: recorded rejection\nROI non-foreground is not a background class.','muted',True));ll.addWidget(ui.button('Angular / Depth view',callback=lambda:self.angular_button.click(),role='small'))
        self.fit_dialog=self.dialog('Object camera',400,360);fitl=self.fit_dialog.layout();fitl.addWidget(ui.label('Fit target','subtitle'));self.fit_target=ui.ComboBox();self.fit_target.addItems(['Observed Cluster','Current Geometry','Reliable Geometry','Track + Prediction']);self.fit_target.currentIndexChanged.connect(self.save_view_settings);fitl.addWidget(self.fit_target);self.fit_include_roi=W.QCheckBox('Include ROI in Fit Visible');self.fit_include_roi.toggled.connect(self.save_view_settings);fitl.addWidget(self.fit_include_roi)
        for title,callback in [('Fit selected',self.fit_views),('Fit visible',self.fit_visible),('Fit ROI',self.fit_roi),('Focus track · keep zoom',self.focus_track)]:fitl.addWidget(ui.button(title,callback=callback,role='secondary'))
        self.advanced=self.dialog('Advanced trace',1050,780);self.side=W.QTabWidget();self.advanced.layout().addWidget(self.side);self.selection_label=ui.label();self.selection_label.hide()
        self.fields=W.QTableWidget(0,3);self.fields.setHorizontalHeaderLabels(['Recorded field','Before','After']);self.fields.setEditTriggers(W.QAbstractItemView.NoEditTriggers);self.fields.verticalHeader().hide();self.fields.horizontalHeader().setMinimumSectionSize(180);self.fields.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch);self.side.addTab(self.fields,'All values')
        self.inspector=W.QPlainTextEdit();self.inspector.setReadOnly(True);self.side.addTab(self.inspector,'Raw trace')
        self.compare_text=W.QPlainTextEdit();self.compare_text.setReadOnly(True);self.side.addTab(self.compare_text,'Comparison evidence')
        self.gates=W.QTableWidget(0,7);self.gates.setHorizontalHeaderLabels(['Side / pair','Gate','Value','Threshold','Result','Final reject','C++ location']);self.gates.setEditTriggers(W.QAbstractItemView.NoEditTriggers);self.gates.verticalHeader().hide();self.gates.horizontalHeader().setSectionResizeMode(W.QHeaderView.ResizeToContents);self.side.addTab(self.gates,'Raw gates')
        self.event_dialog=self.dialog('Recorded events',650,720);self.event_filter=W.QLineEdit();self.event_filter.setPlaceholderText('Filter recorded event / Track ID');self.event_filter.textChanged.connect(self.filter_events);self.event_dialog.layout().addWidget(self.event_filter);self.events=W.QListWidget();self.events.itemActivated.connect(self.activate_event);self.event_dialog.layout().addWidget(self.events)
        self.experiment_dialog=self.dialog('Experiments',700,720);self.experiment_dialog.layout().addWidget(ui.label('Saved experiments','title'));self.joblist=W.QListWidget();self.joblist.itemActivated.connect(self.show_job);self.experiment_dialog.layout().addWidget(self.joblist);self.experiment_dialog.layout().addWidget(ui.label('Double-click to open. Right-click to cancel a running experiment.','muted',True));cancel=W.QAction('Cancel selected experiment',self);cancel.triggered.connect(self.cancel);self.joblist.addAction(cancel);self.joblist.setContextMenuPolicy(C.Qt.ActionsContextMenu)
        self.dataset_dialog=self.dialog('Dataset',640,430);self.dataset_dialog.layout().addWidget(ui.label('Dataset library','title'));self.info=ui.label('Loading dataset catalog…','body',True);self.dataset_dialog.layout().addWidget(self.info);self.dataset_dialog.layout().addStretch();self.dataset_dialog.layout().addWidget(ui.button('Refresh library',glyph='refresh',callback=self.refresh,role='secondary'))
        self.statusBar().setSizeGripEnabled(False);self.statusBar().hide();self.timer=C.QTimer(self);self.timer.setInterval(150);self.timer.timeout.connect(self.tick);self.timer.start();self.jobtimer=C.QTimer(self);self.jobtimer.timeout.connect(self.jobs);self.jobtimer.start(1200)
        self.load_view_settings();self._view_ready=True;self.datasets.currentIndexChanged.connect(self.open_dataset);self.set_mode('Explore');C.QTimer.singleShot(0,lambda:self.set_mode(self.mode));self.refresh();W.QShortcut(G.QKeySequence('F11'),self,activated=self.toggle_focus);W.QShortcut(G.QKeySequence('Escape'),self,activated=lambda:self.toggle_focus() if self.focused else None)
    def set_analysis_mode(self,name):
        if not hasattr(self,'analysis_stack'):
            from crane_view import CranePane
            general=self.takeCentralWidget();self.analysis_stack=W.QStackedWidget();self.analysis_stack.addWidget(general)
            self.crane_pane=CranePane(Worker,self);self.analysis_stack.addWidget(self.crane_pane);self.crane_pane.change_mode.connect(self.set_analysis_mode);self.setCentralWidget(self.analysis_stack)
        crane=name=='Crane';self.analysis_stack.setCurrentIndex(1 if crane else 0)
        self.crane_pane.mode.blockSignals(True);self.crane_pane.mode.setCurrentText(name);self.crane_pane.mode.blockSignals(False)
        self.analysis_mode.blockSignals(True);self.analysis_mode.setCurrentText(name);self.analysis_mode.blockSignals(False)
        self.setWindowTitle(('Crane' if crane else 'General')+' · Perception Lab')
        if crane:
            self.play.setChecked(False)
            if self.crane_pane.data is None and not self.crane_pane.busy:self.crane_pane.load(getattr(self,'crane_folder',None) or __import__('crane_view').DEFAULT)
        else:self.crane_pane.play.setChecked(False)

    def eventFilter(self,obj,event):
        if obj is getattr(self,'canvas',None) and event.type()==C.QEvent.Resize and hasattr(self,'fit_button'):
            self.canvas_title.setVisible(obj.width()>700);self.angular_button.setVisible(self.mode=='Explore' and obj.width()>580)
        return super().eventFilter(obj,event)
    def select_in_view(self,index,kind,ident):
        self.selected_side=index;self.lifecycle_key=None;self.select(kind,ident)
    def frame_seek(self,frame):self.frame.setValue(frame)
    def view_settings(self):return {'schema':1,'layers':{k:v.isChecked() for k,v in self.layers.items()},'focus_only':self.focus_only_button.isChecked(),'fit_target':self.fit_target.currentText(),'fit_include_roi':self.fit_include_roi.isChecked()}
    def load_view_settings(self):
        try:value=json.loads((self.lab.state/'view_settings.json').read_text())
        except (OSError,ValueError):return
        for key,checked in value.get('layers',{}).items():
            if key in self.layers and isinstance(checked,bool):self.layers[key].setChecked(checked)
        self.focus_only_button.setChecked(bool(value.get('focus_only',False)));self.fit_include_roi.setChecked(bool(value.get('fit_include_roi',False)));index=self.fit_target.findText(value.get('fit_target','Observed Cluster'));self.fit_target.setCurrentIndex(max(0,index))
    def save_view_settings(self,*args):
        if getattr(self,'_view_ready',False):dump(self.lab.state/'view_settings.json',self.view_settings())
    def filter_changed(self,*args):
        if not getattr(self,'_view_ready',False):return
        self.redraw();self.save_view_settings()
    def set_all_layers(self,checked):
        self._view_ready=False
        for name,box in self.layers.items():
            if box.isEnabled():box.setChecked(checked)
        self.focus_only_button.setChecked(False);self._view_ready=True;self.filter_changed()
    def reset_layers(self):
        self._view_ready=False
        for name,box in self.layers.items():box.setChecked(bev.DEFAULTS.get(name,False))
        self.focus_only_button.setChecked(False);self._view_ready=True;self.filter_changed()
    def update_filter_capabilities(self):
        if not self.data:return
        state=self.states[self.selected_side];meta,links=bev.evidence(self.data,state);tags=set().union(*meta.values()) if meta else set()
        for name,reason in [('Matched','A match requires recorded current component ID, point count and identical stored feature bounds.'),('Rejected components','No initial component rejection reason is recorded in this frame. Rejected tracks have a separate filter.'),('Unknown-like','No unknown_like component classification is recorded in this frame.'),('Vehicle-like','No vehicle_like component classification is recorded in this frame.')]:
            self.layers[name].setEnabled(name in tags);self.layers[name].setToolTip('Recorded component evidence' if name in tags else reason)
        canonical=any(bev.flags(t)['lost'] for t in bev.tracks(state));self.layers['LOST lifecycle'].setEnabled(canonical);self.layers['LOST lifecycle'].setToolTip('Explicit lifecycle=LOST' if canonical else bev.UNSUPPORTED['LOST lifecycle'])
        if self.selected_track is not None and self.selected_track not in links:self.focus_only_button.setToolTip('No verified current initial-cluster point mapping. Focus only shows the track / recorded models; it does not claim nearby points.')
        else:self.focus_only_button.setToolTip('Uses actual initial-cluster ownership / recorded feature identity only')
    def show_fit(self):
        self.fit_dialog.setWindowFlags(C.Qt.Popup);self.fit_dialog.move(self.fit_more.mapToGlobal(C.QPoint(0,self.fit_more.height()+8)));self.fit_dialog.show()
    def apply_fit(self,bounds,reason,mode):
        if bounds is None:self.camera_notice.setText(reason);return False
        self.fit_mode=mode;view=self.views[self.selected_side];view.fitInView(bev.rect(bounds),C.Qt.KeepAspectRatio);view.update_point_size();self.sync_camera(view);self.camera_notice.setText(f'{mode} · {reason}');self.fit_button.setToolTip(f'Fit Selected · last: {mode} / {reason}');return True
    def fit_selected(self):
        if self.selected_track is None and self.selected_cluster is None:self.fit_visible();return
        b,reason=bev.selected_bounds(self.data,self.states[self.selected_side],self.selected_track,self.selected_cluster,self.fit_target.currentText());self.apply_fit(b,reason,'Fit selected')
    def fit_visible(self):
        view=self.views[2] if self.view_stack.currentIndex()==1 and self.mode=='Explore' else self.views[self.selected_side];bounds=list(view.visible_bounds)
        if self.fit_include_roi.isChecked() and self.layers['ROI'].isChecked():bounds+=view.roi_bounds
        if view is self.views[2]:
            b=bev.union(bounds)
            if b:view.fitInView(bev.rect(b),C.Qt.KeepAspectRatio);self.camera_notice.setText('Fit visible · recorded angular samples')
            return
        self.apply_fit(bev.union(bounds),'Rendered points / objects only'+(' + visible ROI' if self.fit_include_roi.isChecked() else '; ROI excluded'),'Fit visible')
    def fit_roi(self):
        bounds=[]
        for roi in self.data.get('roi',[]):
            if not roi.get('enabled'):continue
            if roi.get('polygon_x'):bounds.append((min(roi['polygon_x']),max(roi['polygon_x']),min(roi['polygon_y']),max(roi['polygon_y'])))
            else:bounds.append(tuple(roi['bounds']))
        self.apply_fit(bev.union(bounds),'Explicit enabled ROI bounds only; layer visibility unchanged','Fit ROI')
    def follow_changed(self):
        if self.follow_button.isChecked():
            if self.selected_track is None:self.follow_button.setChecked(False);self.camera_notice.setText('Select a Track before enabling Follow');return
            self.center_selected(follow=True)
        else:self.camera_notice.setText('Follow off · camera remains where you placed it')
    def pause_follow(self,*args):
        if hasattr(self,'follow_button') and self.follow_button.isChecked():self.follow_button.setChecked(False);self.camera_notice.setText('Follow paused · manual pan or selection change')
    def center_selected(self,follow=False):
        if self.selected_track is None:self.camera_notice.setText('Select a Track to focus');return
        state=self.states[self.selected_side];tr=next((t for t in bev.tracks(state) if int(t['id'])==self.selected_track),None)
        if tr is None:self.camera_notice.setText(f'Follow paused · T{self.selected_track} unavailable on selected run / frame' if follow else 'Selected track unavailable');return
        pos,reason=bev.position(tr,self.data,state)
        if pos is None:self.camera_notice.setText('Follow paused · no recorded position');return
        view=self.views[self.selected_side];view.centerOn(pos[0]*12,-pos[1]*12);self.sync_camera(view);side='Before' if self.selected_side==0 else 'After';self.camera_notice.setText(f'{"Following" if follow else "Focused"} {side} T{self.selected_track} · {reason} · zoom retained')
    def toggle_lifecycle(self):
        visible=self.lifecycle_toggle.isChecked();self.lifecycle_plot.setVisible(visible);self.detail_replay.setVisible(visible);self.update_lifecycle();self.refresh_timeline_events()
    def show_all_events(self):
        self.lifecycle_toggle.setChecked(False);self.toggle_lifecycle()
    def refresh_timeline_events(self):
        if hasattr(self,'lifecycle_toggle') and self.lifecycle_toggle.isChecked() and self.selected_track is not None:
            result=self.lifecycle_plot.result;self.slider.events=[{'frame':s['start'],'kind':s['lane']} for s in result.get('intervals',[])]+[{'frame':e['frame'],'kind':e['kind']} for e in result.get('events',[])]
        else:self.slider.events=self.event_rows
        self.slider.update()
    def update_lifecycle(self):
        if not hasattr(self,'lifecycle_plot'):return
        if self.selected_track is None or not self.identity():
            self.lifecycle_key=None;self.lifecycle_plot.set_data({},getattr(self,'rows',[]));self.lifecycle_toggle.setChecked(False);self.lifecycle_plot.hide();self.detail_replay.hide();self.lifecycle_label.setText('All tracks · Before run major events');self.refresh_timeline_events();return
        rid,key=self.identity();run=self.runs[self.selected_side].currentData();identity=(rid,key,run,self.selected_track)
        if identity==self.lifecycle_key:return
        self.lifecycle_key=identity;self.lifecycle_revision+=1;revision=self.lifecycle_revision;self.lifecycle_label.setText(f'T{self.selected_track} · loading recorded lifecycle…')
        if not run:self.lifecycle_plot.set_data({},self.rows);self.lifecycle_label.setText('Lifecycle unavailable · choose a completed run');return
        def finish(result,error):
            if error or revision!=self.lifecycle_revision:return
            self.lifecycle_plot.set_data(result,self.rows);side='Before' if self.selected_side==0 else 'After';self.lifecycle_label.setText(f'{side} T{result["track"]} · observed / missed counter; LOST enum not serialized');self.lifecycle_label.setToolTip(result['notice']);self.refresh_timeline_events()
        tid=self.selected_track;self.async_call(lambda:bev.lifecycle(self.lab,rid,key,run,tid),finish)
    def select_replay_range(self,start,end):
        self.range_start.setValue(start);self.range_end.setValue(end);self.lifecycle_label.setText(f'Detail frames {start}–{end} selected · replay warms up from frame 1')
    def camera_state(self):
        cameras=[]
        for v in self.views:
            center=v.mapToScene(v.viewport().rect().center());t=v.transform();cameras.append({'center_world_xy':[center.x()/12,-center.y()/12],'transform':[t.m11(),t.m12(),t.m21(),t.m22(),t.dx(),t.dy()],'viewport_size':[v.viewport().width(),v.viewport().height()]})
        return cameras
    def restore_cameras(self,cameras):
        for v,camera in zip(self.views,cameras):
            values=camera.get('transform',[]);xy=camera.get('center_world_xy',[])
            if len(values)==6 and len(xy)==2 and all(bev.finite(v) for v in values+xy):v.setTransform(G.QTransform(*values));v.centerOn(float(xy[0])*12,-float(xy[1])*12);v.update_point_size()
    def dialog(self,title,width,height):
        d=W.QDialog(self);d.setWindowTitle(title);d.resize(width,height);layout=W.QVBoxLayout(d);layout.setContentsMargins(24,24,24,24);layout.setSpacing(16);return d
    def present(self,d):
        area=self.screen().availableGeometry();d.resize(min(d.width(),area.width()-40),min(d.height(),area.height()-60));d.move(self.frameGeometry().center()-d.rect().center());d.show();d.raise_();d.activateWindow()
    def show_layers(self):
        self.layer_dialog.setWindowFlags(C.Qt.Popup);self.layer_dialog.resize(400,min(680,self.screen().availableGeometry().height()-90));pos=self.layer_button.mapToGlobal(C.QPoint(0,self.layer_button.height()+8));area=self.screen().availableGeometry();self.layer_dialog.move(min(pos.x(),area.right()-self.layer_dialog.width()-12),min(pos.y(),area.bottom()-self.layer_dialog.height()-12));self.layer_dialog.show()
    def toggle_sidebar(self):
        collapsed=self.sidebar.width()>100;self.sidebar.setFixedWidth(64 if collapsed else 202)
        for name,b in self.nav.items():b.setText('' if collapsed else name)
        self.brand.setVisible(not collapsed);self.brand_sub.setVisible(not collapsed);self.offline.setVisible(not collapsed);self.sidebar_note.setVisible(not collapsed);self.collapse.setToolTip('Expand sidebar' if collapsed else 'Collapse sidebar')
    def set_nav(self,name):
        for key,b in self.nav.items():b.setChecked(key==name)
    def show_datasets(self):self.set_nav('Dataset');self.present(self.dataset_dialog)
    def show_experiments(self):self.set_nav('Experiments');self.jobs();self.present(self.experiment_dialog)
    def show_events(self):self.present(self.event_dialog)
    def show_settings(self):
        self.set_nav('Settings');d=self.dialog('Workspace settings',520,360);d.layout().addWidget(ui.label('Workspace','title'));d.layout().addWidget(ui.label('Native Qt · offline','subtitle'));d.layout().addWidget(ui.label('Display layers affect rendering only. Dataset and operating parameters are not changed by appearance settings.','body',True));d.layout().addWidget(ui.label(str(self.lab.state),'muted',True));d.layout().addStretch();d.layout().addWidget(ui.button('Scene layers',glyph='layers',callback=self.show_layers,role='secondary'));self.settings_dialog=d;self.present(d)
    def show_advanced(self):self.present(self.advanced)
    def play_icon(self):self.play.setIcon(ui.icon('pause' if self.play.isChecked() else 'play'))
    def fit_views(self):
        if self.mode=='Explore' and self.view_stack.currentIndex()==1:self.views[2].reset_view();self.camera_notice.setText('Fit visible · recorded angular / depth samples');return
        self.fit_selected()
    def toggle_angular(self):
        self.view_stack.setCurrentIndex(1 if self.angular_button.isChecked() else 0);self.canvas_title.setText('ANGULAR / DEPTH' if self.angular_button.isChecked() else 'WORLD XY')
    def sync_camera(self,source):
        if self.mode!='Compare' or self.syncing or source not in self.views[:2]:return
        self.syncing=True;target=self.views[1] if source is self.views[0] else self.views[0];target.setTransform(source.transform());target.centerOn(source.mapToScene(source.viewport().rect().center()));target.update_point_size();self.syncing=False
    def set_mode(self,name):
        self.mode=name;self.modes[name].setChecked(True);compare=name=='Compare';tune=name=='Tune';self.view_hosts[1].setVisible(compare);self.view_labels[0].setVisible(compare);self.right.setCurrentIndex(1 if tune else 0)
        self.angular_button.setVisible(name=='Explore')
        if name!='Explore':self.view_stack.setCurrentIndex(0)
        else:self.toggle_angular()
        self.canvas_title.setText('BEFORE / AFTER' if compare else 'EXPERIMENT PREVIEW' if tune else ('ANGULAR / DEPTH' if self.angular_button.isChecked() else 'WORLD XY'))
        self.workspace.setSizes([max(400,self.content.width()-(530 if tune else 352)),530 if tune else 352]);self.view_split.setSizes([500,500]);self.set_nav('Comparisons' if compare else 'Dataset');self.sync_camera(self.views[self.selected_side]);self.present_summary();self.refresh_gate_rows()
    def toggle_focus(self):
        self.focused=not self.focused
        for widget in [self.sidebar,self.header,self.right,self.timeline]:widget.setVisible(not self.focused)
        self.content.layout().setContentsMargins(*(0,0,0,0) if self.focused else (24,18,24,8));self.full_button.setToolTip('Exit focus view · Esc' if self.focused else 'Focus view · F11')
        if self.focused:self.previous_maximized=self.isMaximized();self.showFullScreen()
        else:
            self.showNormal()
            if getattr(self,'previous_maximized',False):self.showMaximized()
            self.set_mode(self.mode)
    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'sidebar') and self.width()<1280 and self.sidebar.width()>100:self.toggle_sidebar()
    def filter_params(self,text):
        for row,p in enumerate(self.params):self.table.setRowHidden(row,text.lower() not in p['name'].lower())
        visible=sum(not self.table.isRowHidden(row) for row in range(self.table.rowCount()));height=min(360,max(90,44+visible*58));self.table.setMinimumHeight(height);self.table.setMaximumHeight(height)
    def parameter_info(self):
        row=self.table.currentRow()
        if row<0 or row>=len(self.params):return
        p=self.params[row];self.param_detail.setText(f"{p['name']} · {p['unit']}\n{'Editable override' if p['editable'] else 'Read only'} · {'Frame what-if supported' if p['whatif'] else 'Tracking replay required'}\nC++ default: {p['default_source']}");self.param_detail.setToolTip(json.dumps(p,ensure_ascii=False,indent=2))
    def show_parameter_details(self):
        row=self.table.currentRow()
        if row<0:return
        d=self.dialog('Parameter source',720,540);d.layout().addWidget(ui.label(self.params[row]['name'],'subtitle',True));text=W.QPlainTextEdit(json.dumps(self.params[row],ensure_ascii=False,indent=2));text.setReadOnly(True);d.layout().addWidget(text);self.parameter_dialog=d;self.present(d)
    def clear_layout(self,layout):
        while layout.count():
            item=layout.takeAt(0)
            if item.widget():item.widget().deleteLater()
    def present_summary(self):
        value=self.current_value;tr=value.get('after' if self.selected_side else 'before',{});after=value.get('before' if self.selected_side else 'after',{})
        if self.selected_track is not None and 'id' not in tr:
            self.track_heading.setText(f'Track {self.selected_track}');self.track_state.setText('Unavailable on selected run / frame');self.decision.setText('No current track record');self.association.setText('Previous geometry is not displayed.');self.failure.hide();self.summary.clear();self.diff_label.clear();self.tune_context.setText('No current recorded track selected.');return
        if not tr:
            self.track_heading.setText('Select a track');self.track_state.setText('Choose a track in the scene or list.');self.decision.setText('No recorded decision');self.association.clear();self.failure.hide();self.summary.clear();self.diff_label.clear();self.tune_context.setText('No current recorded track selected.');return
        if 'id' not in tr or ('count' in tr and 'geometry' not in tr):
            self.track_heading.setText(f"Cluster {tr.get('id','—')}");self.track_state.setText(f"{tr.get('count','—')} current points · stored initial cluster");self.decision.setText('Cluster selected');self.association.setText('A cluster is not a confirmed vehicle.');self.summary.setText('See Advanced for World XY bounds and actual sample rings.');self.failure.hide();self.failure_scope.clear();self.diff_label.clear();return
        ident=tr['id'];self.track_heading.setText(f'Track {ident}');ob=tr.get('observation',{});state=tr.get('lifecycle','Not recorded');lost=tr.get('lost_frames',tr.get('lost','—'));self.track_state.setText(f"{str(state).replace('_',' ').title()} · Lost frames {lost}")
        actual=bev.flags(tr)['actual'];observed='Current observation' if actual is True else 'Prediction only' if actual is False else 'Observation not recorded'
        vehicle=tr.get('vehicle',tr.get('valid')) in (True,1,'1','true');rejected=state=='REJECTED' or tr.get('valid') is False
        self.decision.setText('Detection rejected' if rejected else 'Public vehicle' if tr.get('public') in (True,1,'1','true') and vehicle else 'Vehicle candidate' if vehicle else 'Unconfirmed candidate')
        association=tr.get('association_source','Not recorded');self.association.setText(f"{observed}\nAssociation · {association}")
        kf=tr.get('kf',{});pos=kf.get('state',[]);geometry=tr.get('geometry',{})
        def num(v):
            try:return f'{float(v):.3f}' if v is not None else '—'
            except (ValueError,TypeError):return '—'
        x=tr.get('x',pos[0] if len(pos)>1 else None);y=tr.get('y',pos[1] if len(pos)>1 else None)
        vx=tr.get('vx',pos[2] if len(pos)>3 else None);vy=tr.get('vy',pos[3] if len(pos)>3 else None)
        rows=[('KF position',f'{num(x)}, {num(y)} m'),('Velocity x / y',f'{num(vx)}, {num(vy)} m/s'),('Geometry',' × '.join(num(geometry.get(k)) for k in ('length','width','height'))+' m'),('Observation points',str(ob.get('points','Not recorded')))]
        self.summary.setText('<table width="100%" cellspacing="0" cellpadding="0">'+''.join(f'<tr><td style="padding:5px 10px 5px 0;color:#787d70;font-size:12px">{html.escape(name)}</td><td style="padding:5px 0;color:#30362b;font-size:14px">{html.escape(v)}</td></tr>' for name,v in rows)+'</table>')
        self.tune_context.setText(f"Recorded frame {self.data.get('frame','—')} · Track {ident}\n{self.decision.text()} · {observed}\nOverrides are previews until an experiment is run.")
        differences=[]
        def flatten(obj,prefix=''):
            result={}
            for key,v in obj.items():
                if key in ('gate_calls','gate_inspector','selected_call'):continue
                if isinstance(v,dict):result.update(flatten(v,prefix+key+'.'))
                else:result[prefix+key]=v
            return result
        a=flatten(tr);b=flatten(after)
        differences=[key for key in a.keys()|b.keys() if a.get(key)!=b.get(key)];self.diff_label.setVisible(self.mode=='Compare')
        if self.mode=='Compare':
            lines=[f'{len(differences)} changed fields']
            for key,title in [('x','KF x'),('y','KF y'),('vx','Velocity x'),('vy','Velocity y'),('score','Vehicle score')]:
                if key in differences:lines.append(f'{title}: {num(tr.get(key))} → {num(after.get(key))}')
            if len(lines)==1:lines.append('KF / geometry details differ.' if differences else 'No recorded field changes.')
            lines.append('Common fields collapsed · full precision in Advanced.');lines.append('Same IDs do not establish physical correspondence.');self.diff_label.setText('\n'.join(lines))
    def refresh_gate_rows(self):
        if not hasattr(self,'gate_rows_layout'):return
        self.clear_layout(self.gate_rows_layout);index=self.gate_calls.currentIndex();groups=getattr(self,'gate_groups',[])
        if index<0 or index>=len(groups):self.failure.hide();self.failure_scope.setText('No detailed gate call recorded for this selection.');self.skipped.hide();return
        side,group=groups[index];gates=group['gates'];failed=[g for g in gates if g.get('evaluated',True) and g.get('pass') is False];skipped=[g for g in gates if not g.get('evaluated',True)]
        def fmt(v):return f'{v:.4g}' if isinstance(v,(int,float)) else str(v)
        self.failure.setVisible(bool(failed))
        if failed:
            first=failed[0];title={'height_min':'Height below minimum','width_min':'Width below minimum','length_min':'Length below minimum'}.get(first['gate'],first['gate'].replace('_',' ').capitalize()+' failed');self.failure_heading.setText(title);unit=' m' if first['gate'].startswith(('height_','width_','length_')) and not first['gate'].endswith('support') else '';self.failure_detail.setText(f"Gate value {fmt(first.get('value','—'))}{unit} · threshold {fmt(first.get('threshold','—'))}{unit}")
        self.failure_scope.setText(f"{group['function']} · Cluster {group.get('cluster','—')} · {side}\nFinal association cause is shown only when recorded.")
        for gate in gates:
            evaluated=gate.get('evaluated',True)
            if not evaluated and not self.skipped.isChecked():continue
            row=W.QWidget();rl=W.QVBoxLayout(row);rl.setContentsMargins(0,9,0,9);rl.setSpacing(3);head=W.QHBoxLayout();title=ui.label(gate['gate'].replace('_',' ').capitalize(),'body',True);head.addWidget(title,1)
            status='Pass' if evaluated and gate.get('pass') else 'Fail' if evaluated else 'Not evaluated';status_label=ui.label(status,'success' if status=='Pass' else 'danger' if status=='Fail' else 'muted');head.addWidget(status_label);rl.addLayout(head)
            if evaluated:rl.addWidget(ui.label(f"{fmt(gate.get('value','—'))}  /  {fmt(gate.get('threshold','—'))}",'muted'))
            row.setToolTip(f"{gate.get('function',group['function'])}:{gate.get('line','?')}\n"+json.dumps(gate,ensure_ascii=False));self.gate_rows_layout.addWidget(row)
        self.skipped.setText(f"{'Hide' if self.skipped.isChecked() else 'Show'} {len(skipped)} gates not evaluated");self.skipped.setVisible(bool(skipped));self.gate_notice.setText('Unexecuted gates are not treated as passing or failing. A passed gate does not prove association.')
    def toggle_skipped(self):self.refresh_gate_rows()
    def show_gate_details(self):
        index=self.gate_calls.currentIndex();groups=getattr(self,'gate_groups',[])
        if index<0 or index>=len(groups):return
        side,group=groups[index];d=self.dialog('Recorded gate detail',720,800);layout=d.layout();layout.addWidget(ui.label('Recorded gate detail','title'));layout.addWidget(ui.label(f"{group['function']} · Cluster {group.get('cluster')} · {side}",'muted',True))
        sc=W.QScrollArea();sc.setWidgetResizable(True);body=W.QWidget();bodylayout=W.QVBoxLayout(body);bodylayout.setContentsMargins(0,0,12,0);bodylayout.setSpacing(14)
        for gate in group['gates']:
            evaluated=gate.get('evaluated',True);result=('PASS' if gate.get('pass') else 'FAIL') if evaluated else 'NOT_EVALUATED';heading=W.QHBoxLayout();heading.addWidget(ui.label(gate['gate'].replace('_',' ').capitalize(),'subtitle'),1);heading.addWidget(ui.label(result,'success' if result=='PASS' else 'danger' if result=='FAIL' else 'muted'));bodylayout.addLayout(heading)
            actual=gate.get('value','Not recorded') if evaluated else 'Not evaluated';threshold=gate.get('threshold','Not recorded');bodylayout.addWidget(ui.label(f"Value: {actual}    Threshold: {threshold}",'body',True));bodylayout.addWidget(ui.label(f"{gate.get('function',group['function'])}:{gate.get('line','?')} · Final reject: {gate.get('final_reject','Not recorded')}",'muted',True));bodylayout.addWidget(ui.line())
        bodylayout.addWidget(ui.label('Only this recorded invocation is shown. Unknown final association or ownership competition is not inferred.','muted',True));sc.setWidget(body);layout.addWidget(sc,1);self.gate_detail_dialog=d;self.present(d)
    def async_call(self,fn,callback):
        worker=Worker(fn,self);self.workers.add(worker)
        def done(value,error):
            if error:
                print(error,file=sys.stderr,flush=True);self.statusBar().showMessage(error,15000);self.last_ui_error=error;self.replay_state.setText('● Attention');self.replay_state.setToolTip(error)
            callback(value,error)
        worker.done.connect(done);worker.finished.connect(lambda:self.workers.discard(worker));worker.finished.connect(worker.deleteLater);worker.start()
    def dataset_catalog(self):
        # One unavailable/empty dataset must not prevent opening other roots.
        values=[]
        for rid,root in self.lab.roots.items():
            for key in self.lab.keys(rid):
                try:values.append(self.lab.metadata(rid,key))
                except (ValueError,KeyError,OSError,RuntimeError,IndexError) as error:
                    message='Dataset contains no readable frame entries' if isinstance(error,IndexError) else str(error)
                    values.append({'root':rid,'key':key,'status':'Not Prepared','path':str(root/key),'error':message})
        return values
    def refresh(self):
        def finish(values,error):
            if error:return
            previous=self.datasets.currentData();self.catalog=values;self.datasets.blockSignals(True);self.datasets.clear()
            for d in values:self.datasets.addItem(f"{d['key']} · {d['root'].title()}"+(' · Not prepared' if d['status']!='Prepared' else ''),(d['root'],d['key']))
            target=tuple(previous) if previous else ('local','20261023');index=next((i for i,d in enumerate(values) if (d['root'],d['key'])==target),0);self.datasets.setCurrentIndex(index);self.datasets.blockSignals(False);self.open_dataset()
        self.async_call(self.dataset_catalog,finish)
    def identity(self):return self.datasets.currentData()
    def open_dataset(self):
        if not self.identity():return
        self.generation+=1;generation=self.generation;rid,key=self.identity();self.play.setChecked(False);self.pause_follow();self.selected_cluster=None;self.lifecycle_key=None;self.lifecycle_revision+=1;self.lifecycle_plot.set_data({},[]);self.track_choice.blockSignals(True);self.track_choice.clear();self.track_choice.addItem('All',None);self.track_choice.blockSignals(False);self.selection_source=None
        for box in self.runs:box.blockSignals(True);box.clear();box.blockSignals(False)
        self.rows=[];self.data={};self.states=[{},{}];self.event_rows=[];self.events.clear();self.slider.events=[];self.selected_track=None;self.redraw();self.fields.setRowCount(0);self.gates.setRowCount(0);self.inspector.clear();self.compare_text.clear();self.current_value={};self.present_summary();self.gate_groups=[];self.refresh_gate_rows();self.point_label.setText('Select a prepared dataset')
        for v in self.views:v.first=True
        def fetch():
            entry=next((d for d in self.catalog if d['root']==rid and d['key']==key),None)
            if entry and entry['status']!='Prepared':return entry,[],[],[],[]
            meta=self.lab.metadata(rid,key)
            if meta['status']!='Prepared':return meta,[],[],[],[]
            return meta,self.lab.rows(rid,key),self.lab.runs(rid,key),self.lab.parameters(rid,key),[]
        def finish(result,error):
            if error or generation!=self.generation:return
            meta,rows,runs,params,_=result;self.info.setText(f"{meta['status']} • {meta.get('frames',0):,} frames • {meta.get('duration_sec',0):.2f}s • Schema {meta.get('schema','—')} • prefix compatible: {meta.get('prefix_compatible','—')} • {meta.get('bytes',0)/2**30:.2f} GiB");self.info.setToolTip(json.dumps(meta,ensure_ascii=False,indent=2));self.rows=rows;self.params=params
            for box in self.runs:
                box.blockSignals(True);box.addItem('(no result)',None)
                for run in runs:box.addItem(run['label'],run['id'])
                preferred=self.run_to_open or box.currentData();index=box.findData(preferred) if preferred else -1
                box.setCurrentIndex(index if index>=0 else (len(runs) if runs else 0));box.blockSignals(False)
            self.table.setRowCount(len(params))
            for i,p in enumerate(params):
                for col,text in enumerate([p['name'],json.dumps(p['current']),'',p['unit'],p['default_source'],', '.join(p['functions']),p['range'],str(p['requires_regeneration']),str(p['whatif']),str(p['requires_replay'])]):
                    item=W.QTableWidgetItem(text)
                    if col!=2 or not p['editable']:item.setFlags(item.flags() & ~C.Qt.ItemIsEditable)
                    item.setToolTip(json.dumps(p,ensure_ascii=False,indent=2));self.table.setItem(i,col,item)
            self.run_to_open=None
            if params:self.table.selectRow(next((i for i,p in enumerate(params) if p['name']=='vehicle_min_score'),0))
            self.filter_params(self.param_search.text())
            for widget in [self.frame,self.slider,self.range_start,self.range_end]:widget.setRange(1,max(1,len(rows)))
            self.range_start.setValue(min(80,len(rows)) if rows else 1);self.range_end.setValue(min(80,len(rows)) if rows else 1);self.seek_time.setMaximum(meta.get('duration_sec',0))
            if self.pending_whatif and rows:self.frame.setValue(self.pending_whatif['whatif']['frame'])
            self.run_changed()
        self.async_call(fetch,finish)
    def run_changed(self,*args):
        current_runs=[b.currentData() for b in self.runs]
        changed=[i for i,r in enumerate(current_runs) if r!=self._displayed_runs[i]]
        if changed and self.data:
            for i in changed:self.states[i]={};self.current_value.pop('after' if i else 'before',None)
            self.redraw();self.present_summary()
        if self.selected_track is not None and self.selection_source and self.identity():
            current=(*self.identity(),self.runs[self.selected_side].currentData())
            if current!=self.selection_source and not self.pending_whatif:
                self.selected_track=None;self.selected_cluster=None;self.pause_follow();self.lifecycle_key=None;self.lifecycle_revision+=1;self.current_value={};self.present_summary();self.gate_groups=[];self.gate_calls.clear();self.gates.setRowCount(0);self.refresh_gate_rows();self.lifecycle_plot.set_data({},getattr(self,'rows',[]))
        self.request_frame()
        if not self.identity() or not self.runs[0].currentData():return
        rid,key=self.identity();run=self.runs[0].currentData();self.event_revision+=1;revision=self.event_revision;generation=self.generation
        self.async_call(lambda:self.lab.events(rid,key,run),lambda v,e:self.fill_events(v,e,generation,revision))
    def fill_events(self,value,error,generation,revision):
        if error or generation!=self.generation or revision!=self.event_revision:return
        self.event_rows=value['events'];self.refresh_timeline_events();self.filter_events()
    def filter_events(self,*args):
        self.events.clear();query=self.event_filter.text().lower()
        for event in self.event_rows:
            text=f"F{event['frame']} T{event.get('track')} {event['kind']}"
            if query and query not in text.lower():continue
            item=W.QListWidgetItem(text);item.setData(C.Qt.UserRole,event);item.setToolTip(str(event.get('detail','')));self.events.addItem(item)
    def activate_event(self,item):
        event=item.data(C.Qt.UserRole);self.selected_track=event.get('track');self.frame.setValue(event['frame']);self.request_frame()
    def seek_seconds(self):
        if not getattr(self,'rows',[]):return
        times=[row['relative_sec'] for row in self.rows];i=bisect.bisect_left(times,self.seek_time.value())
        if i and (i==len(times) or abs(times[i-1]-self.seek_time.value())<abs(times[i]-self.seek_time.value())):i-=1
        self.frame.setValue(min(i+1,len(times)))
    def choose_track(self,*args):
        self.selected_track=self.track_choice.currentData()
        if self.selected_track is not None:self.select('track',self.selected_track)
        self.redraw()
    def request_frame(self,*args):
        if not self.identity() or not getattr(self,'rows',[]):return
        if self.busy:self.pending=True;return
        self.busy=True;self.pending=False;generation=self.generation;rid,key=self.identity();frame=self.frame.value();runs=[box.currentData() for box in self.runs]
        self.statusBar().showMessage(f'Loading frame {frame}…')
        def fetch():
            data=self.lab.frame(rid,key,frame);states=[]
            for run in runs:
                try:states.append(self.lab.state_at(rid,key,run,frame) if run else {})
                except (ValueError,KeyError,IndexError,OSError) as error:states.append({'notice':str(error)})
            try:comparison=self.lab.compare(rid,key,*runs,frame) if all(runs) else {'notice':'Select two completed results'}
            except (ValueError,KeyError,IndexError,OSError) as error:comparison={'notice':'Comparison unavailable: '+str(error)}
            return data,states,comparison
        def finish(result,error):
            self.busy=False
            if error and generation==self.generation:self.point_label.setText('Frame unavailable · '+str(error)[:200])
            if not error and generation==self.generation and runs==[box.currentData() for box in self.runs] and frame==self.frame.value():
                if self.selected_cluster is not None and self.data.get('frame')!=frame:self.selected_cluster=None
                self.last_ui_error=None;self._displayed_runs=list(runs);self.data,self.states,comparison=result;self.point_label.setText(f"{self.data['actual_points']:,} points · {len(self.data['clusters'])} clusters · display LOD {self.data['rendered_points']:,}");self.frame_caption.setText(f"Frame / {len(self.rows)}");self.compare_text.setPlainText(json.dumps(safe(comparison),ensure_ascii=False,indent=2));self.inspector.setPlainText(json.dumps(safe(self.states[0]),ensure_ascii=False,indent=2));self.redraw()
                tracks=(self.states[0].get('trace') or {}).get('tracks',[])
                alltracks=[]
                for state in self.states:alltracks.extend((state.get('trace') or {}).get('tracks',[]) or state.get('tracks',[]))
                ids=sorted({int(t['id']) for t in alltracks});self.track_choice.blockSignals(True);self.track_choice.clear();self.track_choice.addItem('All',None)
                for ident in ids:self.track_choice.addItem(f'T{ident}',ident)
                idx=self.track_choice.findData(self.selected_track)
                if idx<0 and self.selected_track is not None:self.track_choice.addItem(f'T{self.selected_track} · unavailable',self.selected_track);idx=self.track_choice.count()-1
                self.track_choice.setCurrentIndex(max(0,idx));self.track_choice.blockSignals(False)
                if self.selected_track is not None:self.select('track',self.selected_track)
                elif tracks and self.selected_cluster is None:self.select('track',tracks[0]['id'],focus=False)
                self.update_filter_capabilities();self.lifecycle_plot.frame=frame;self.lifecycle_plot.update()
                if self.follow_button.isChecked():self.center_selected(follow=True)
                self.seek_time.setValue(self.data['index']['relative_sec'])
                self.time_label.setText(f"+{self.data['index']['relative_sec']:.3f}s");self.statusBar().showMessage(f"Frame {frame} • actual {self.data['actual_points']:,} points • displayed LOD {self.data['rendered_points']:,} • {len(self.data['clusters'])} clusters • recorded Jetson: Not Available")
            if self.pending_whatif and not self.pending and self.data.get('frame')==self.pending_whatif.get('whatif',{}).get('frame') and generation==self.generation:
                job=self.pending_whatif;self.pending_whatif=None;self.render_whatif(job)
            if self.pending and not self.closing:self.request_frame()
        self.async_call(fetch,finish)
    def redraw(self,*args):
        layers={k:v.isChecked() for k,v in self.layers.items()}
        for i,view in enumerate(self.views):
            view.selected_id=self.selected_track;view.selected_cluster=self.selected_cluster;view.focus_only=self.focus_only_button.isChecked() and (self.selected_track is not None or self.selected_cluster is not None);view.render_data(self.data,self.states[min(i,1)],layers,i==2)
        self.sync_camera(self.views[self.selected_side])
        if self.data:self.point_label.setText(f"{self.data['actual_points']:,} original points · {self.views[self.selected_side].last_render.get('point_count',0):,} shown / {self.data['rendered_points']:,} LOD")
    def select(self,kind,identifier,focus=True):
        if kind=='cluster':
            self.selected_cluster=identifier;self.selected_track=None;self.pause_follow();self.track_choice.blockSignals(True);self.track_choice.setCurrentIndex(0);self.track_choice.blockSignals(False)
            cluster=next((x for x in self.data.get('clusters',[]) if x['id']==identifier),{});points=[p for p in self.data.get('points',[]) if int(p[4])==identifier]
            value={'before':cluster,'after':cluster,'display_sample':{'notice':'LOD samples only; not full-point statistics','rings':sorted({int(p[3]) for p in points}),'sensor_origin':self.data.get('sensor_origin'),'sensor_rotation':self.data.get('sensor_rotation'),'angular_bin_deg':self.data.get('index',{}).get('azimuth_bin_deg'),'ray_spacing_deg':self.data.get('index',{}).get('ray_elevation_spacing_deg')}}
        else:
            value={}
            for label,state in zip(['before','after'],self.states):
                tracks=(state.get('trace') or {}).get('tracks',[]) or state.get('tracks',[])
                value[label]=next((t for t in tracks if int(t['id'])==identifier),{'detail':'Not recorded'})
        self.current_value=value;self.inspector.setPlainText(json.dumps(safe(value),ensure_ascii=False,indent=2));self.populate_fields(value);self.present_summary()
        self.selection_label.setText(f'{kind.title()} {identifier} • recorded values; missing = Not Available. Same numeric ID across runs is not proof of physical identity.')
        if kind=='track':
            if focus:
                self.selected_cluster=None;self.selected_track=identifier;self.selection_source=(*self.identity(),self.runs[self.selected_side].currentData());self.track_choice.blockSignals(True);self.track_choice.setCurrentIndex(max(0,self.track_choice.findData(identifier)));self.track_choice.blockSignals(False)
            self.gate_calls.blockSignals(True);self.gate_calls.clear();self.gate_groups=[]
            for label in ['before','after']:
                for group in value[label].get('gate_calls',[]):
                    self.gate_calls.addItem(f"{label.title()} · {group['function']} · C{group.get('cluster')}");self.gate_groups.append((label,group))
            self.gate_calls.blockSignals(False);self.fill_gates()
            if focus:self.redraw()
        else:self.gate_calls.clear();self.gates.setRowCount(0);self.refresh_gate_rows();self.redraw()
        self.update_lifecycle()
    def populate_fields(self,value):
        def flatten(obj,prefix=''):
            output={}
            for key,item in obj.items():
                if key in ('gate_calls','gate_inspector','selected_call'):continue
                name=prefix+key
                if isinstance(item,dict):output.update(flatten(item,name+'.'))
                else:output[name]=item
            return output
        a=flatten(value.get('before',{}));b=flatten(value.get('after',{}));keys=list(dict.fromkeys([*a,*b]));self.fields.setRowCount(len(keys))
        for row,key in enumerate(keys):
            for col,text in enumerate([key,json.dumps(safe(a[key]),ensure_ascii=False) if key in a else 'Not Available',json.dumps(safe(b[key]),ensure_ascii=False) if key in b else 'Not Available']):
                item=W.QTableWidgetItem(text)
                if col and a.get(key)!=b.get(key):item.setBackground(G.QColor('#fff2bd'))
                self.fields.setItem(row,col,item)
    def fill_gates(self,*args):
        self.gates.setRowCount(0);index=self.gate_calls.currentIndex()
        if index<0 or index>=len(getattr(self,'gate_groups',[])):self.refresh_gate_rows();return
        label,group=self.gate_groups[index];gates=group['gates'];self.gates.setRowCount(len(gates))
        for row,gate in enumerate(gates):
            evaluated=gate.get('evaluated',True);result=('PASS' if gate.get('pass') else 'FAIL') if evaluated else 'NOT_EVALUATED'
            final=gate.get('final_reject','Not recorded')
            values=[f"{label} C{group.get('cluster')}",gate['gate'],gate.get('value','Not Available') if evaluated else '—',gate.get('threshold','Not Available'),result,str(final) if evaluated else '—',f"{gate.get('function',group['function'])}:{gate.get('line','?')}"]
            for col,value in enumerate(values):
                item=W.QTableWidgetItem(f'{value:.6g}' if isinstance(value,float) else str(value));item.setToolTip(json.dumps(gate,ensure_ascii=False,indent=2))
                if col==4:item.setBackground(G.QColor('#d5f4dc' if result=='PASS' else '#ffdbd8' if result=='FAIL' else '#eeeeee'))
                self.gates.setItem(row,col,item)
        self.refresh_gate_rows()
    def focus_track(self):self.center_selected(follow=False)
    def tick(self):
        if self.play.isChecked() and not self.busy:
            if self.frame.value()>=self.range_end.value():self.play.setChecked(False)
            else:
                multipliers=[.25,.5,1,2,4];index=self.frame.value()-1;interval=.1
                if index+1<len(self.rows):interval=max(.001,self.rows[index+1]['relative_sec']-self.rows[index]['relative_sec'])
                self.timer.setInterval(max(10,int(interval*1000/multipliers[self.speed.currentIndex()])));self.frame.setValue(self.frame.value()+1)
    def start(self,mode):
        try:
            rid,key=self.identity();overrides={}
            for i,p in enumerate(self.params):
                value=self.table.item(i,2).text().strip()
                if value:overrides[p['name']]=json.loads(value)
            run=self.runs[0].currentData();frame=self.frame.value()
            if not self.rows:raise ValueError('Not Prepared')
            start=frame if mode=='what-if' else self.range_start.value();end=frame if mode=='what-if' else self.range_end.value()
            if start>end:raise ValueError('Detail start must be <= end')
            request={'root':rid,'dataset':key,'mode':mode,'label':self.label.text(),'overrides':overrides,'end':end if mode!='full' else len(self.rows),'detail_start':start,'detail_end':end,'run':run,'baseline':run,'track':self.selected_track}
            self.statusBar().showMessage('Preparing experiment…')
            self.async_call(lambda:self.lab.start(request),lambda result,error:self.statusBar().showMessage(error if error else f"Experiment {result['id']} queued; source settings unchanged"))
        except Exception as e:W.QMessageBox.warning(self,'Experiment',str(e))
    def jobs(self):
        selected=self.joblist.currentItem();identifier=selected.data(C.Qt.UserRole) if selected else None;self.joblist.clear();jobs=self.lab.jobs_view();active=[j for j in jobs if j['status'] in ('running','queued')];self.replay_state.setText(f"●  {active[0]['progress_frame']}/{active[0]['progress_total']}" if active else ('● Attention' if getattr(self,'last_ui_error',None) else '●  Ready'))
        for job in reversed(jobs):
            item=W.QListWidgetItem(f"{job['label']} • {job['status']} • {job['progress_frame']}/{job['progress_total']}");item.setData(C.Qt.UserRole,job['id']);self.joblist.addItem(item)
            if job['id']==identifier:self.joblist.setCurrentItem(item)
    def show_job(self,item):
        job=self.lab.jobs[item.data(C.Qt.UserRole)];self.compare_text.setPlainText(json.dumps(safe(job),ensure_ascii=False,indent=2))
        if job['status']=='completed' and job.get('mode')!='what-if':
            self.run_to_open=job['id'];target=(job['root'],job['dataset']);idx=next((i for i in range(self.datasets.count()) if tuple(self.datasets.itemData(i))==target),-1)
            if idx>=0 and idx!=self.datasets.currentIndex():self.datasets.setCurrentIndex(idx)
            else:self.open_dataset()
        elif job.get('mode')=='what-if' and job.get('status')=='completed':
            self.pending_whatif=job;target=(job['root'],job['dataset']);idx=next((i for i in range(self.datasets.count()) if tuple(self.datasets.itemData(i))==target),-1)
            if idx<0:self.pending_whatif=None;self.statusBar().showMessage('Saved dataset root is not configured');return
            if idx!=self.datasets.currentIndex():self.run_to_open=job.get('baseline');self.datasets.setCurrentIndex(idx)
            else:
                for box in self.runs:
                    source=box.findData(job.get('baseline'))
                    if source>=0:box.setCurrentIndex(source)
                self.frame.setValue(job['whatif']['frame']);self.request_frame()
    def render_whatif(self,job):
        self.inspector.setPlainText(json.dumps(safe(job),ensure_ascii=False,indent=2))
        result=job.get('whatif',{});rows=result.get('tracks',[]);tr=next((r for r in rows if r['id']==self.selected_track),rows[0] if rows else None)
        if tr:
            self.selected_track=tr['id'];self.redraw();self.focus_track()
            self.current_value={'before':dict(tr['baseline'],id=tr['id']),'after':dict(tr['candidate'],id=tr['id'])};self.populate_fields(self.current_value);self.present_summary();self.track_state.setText(f"Frozen detection · frame {result['frame']}");self.association.setText('What-if result · association and KF are not advanced.');self.tune_result.setText(f"Frame {result['frame']} · Frozen detection: {tr['baseline']['valid']} → {tr['candidate']['valid']}\nActual tracking success is not evaluated.");self.selection_label.setText(f"Frame What-if F{result['frame']} T{tr['id']} • {result['scope']} • actual tracking success: Not Evaluated")
            self.gate_calls.blockSignals(True);self.gate_calls.clear();self.gate_groups=[]
            catalog=json.loads((self.lab.tool/'gate_catalog.json').read_text())
            for label,name in [('before','baseline_gates'),('after','candidate_gates')]:
                recorded=tr[name];gates=recorded+[dict(g,evaluated=False,status='NOT_EVALUATED') for g in catalog if g['function']=='evaluateDetection' and not any(x['gate']==g['gate'] for x in recorded)]
                group={'function':'evaluateDetection','cluster':tr.get('component'),'call':1,'gates':gates};self.gate_groups.append((label,group));self.gate_calls.addItem(label+' frozen evaluateDetection')
            self.gate_calls.blockSignals(False);self.fill_gates();self.side.setCurrentIndex(0)
    def cancel(self):
        item=self.joblist.currentItem()
        if item:self.lab.cancel(item.data(C.Qt.UserRole));self.jobs()
    def export(self):
        if not self.data:return
        if self.busy or self.data.get('frame')!=self.frame.value():self.camera_notice.setText('Wait for the selected frame to finish loading before export');return
        folder=self.lab.state/'exports'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);folder.mkdir(parents=True)
        self.grab().save(str(folder/'desktop.png'));dump(folder/'selection.json',{'schema':2,'dataset':self.identity(),'frame':self.data['frame'],'runs':[r.currentData() for r in self.runs],'selected_track':self.selected_track,'selected_cluster':self.selected_cluster,'dataset_root':str(self.lab.roots[self.identity()[0]]),'stamp_ns':self.data['index']['stamp_ns'],'point_index_space':'Prepared-frame compact indices, not inferred membership','anchor_side':self.selected_side,'mode':self.mode,'fit_mode':self.fit_mode,'follow':self.follow_button.isChecked(),'view_settings':self.view_settings(),'cameras':self.camera_state(),'rendered_evidence':[v.last_render for v in self.views[:2]],'notice':self.data['notice']});self.statusBar().showMessage(f'Exported {folder}');W.QToolTip.showText(self.canvas.mapToGlobal(C.QPoint(18,self.canvas.height()-40)),f'Saved PNG and selection\n{folder}',self.canvas)
    def closeEvent(self,event):
        if hasattr(self,'crane_pane'):
            self.crane_pane.play.setChecked(False);self.crane_pane.pending=None;self.crane_pane.closing=True
            if self.crane_pane.workers:C.QTimer.singleShot(100,self.close);event.ignore();return
        if self.workers:
            self.play.setChecked(False);self.pending=False;self.closing=True;self.timer.stop();self.jobtimer.stop();self.statusBar().showMessage('Finishing current read before closing…')
            for job in self.lab.jobs_view():
                if job['status'] in ('running','queued'):self.lab.cancel(job['id'])
            C.QTimer.singleShot(100,self.close);event.ignore();return
        for job in self.lab.jobs_view():
            if job['status'] in ('running','queued'):self.lab.cancel(job['id'])
        event.accept()

def main():
    p=argparse.ArgumentParser();p.add_argument('--state',type=Path,default=Path('/home/sjjin/construct_ws/general_debug_lab'));p.add_argument('--dataset-root',action='append',default=[]);p.add_argument('--analysis-mode',choices=['General','Crane'],default='General');p.add_argument('--crane-data',type=Path);p.add_argument('--smoke',type=Path,help='Offscreen actual-frame smoke screenshot, then exit');a=p.parse_args()
    a.state.mkdir(parents=True,exist_ok=True);lock=(a.state/'desktop.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Debug Lab desktop is already running')
    roots={'local':dataset.ROOT,'z':Path('/mnt/z/osong/oosong_record/enterance_dataset')}
    if a.smoke:roots={'local':dataset.ROOT}
    for i,root in enumerate(a.dataset_root):roots[f'custom{i}']=Path(root).resolve()
    W.QApplication.setAttribute(C.Qt.AA_EnableHighDpiScaling,True);W.QApplication.setAttribute(C.Qt.AA_UseHighDpiPixmaps,True)
    app=W.QApplication(sys.argv[:1]);app.setApplicationName('General Perception Debug Lab');window=Window(Lab(a.state.resolve(),roots));window.show()
    window.crane_folder=a.crane_data
    if a.analysis_mode=='Crane':window.set_analysis_mode('Crane')
    if a.smoke:
        deadline=time.monotonic()+45;phase=[0]
        def smoke():
            if time.monotonic()>deadline:
                print('SMOKE TIMEOUT',window.identity(),window.statusBar().currentMessage(),len(window.workers),file=sys.stderr,flush=True);app.exit(2);return
            if not window.busy and getattr(window,'rows',[]) and not window.workers:
                if phase[0]==0:
                    for box,label in zip(window.runs,['debug_baseline','kf_noise_candidate']):
                        idx=box.findText(label)
                        if idx>=0:box.setCurrentIndex(idx)
                    window.frame.setValue(80);phase[0]=1
                elif window.data.get('frame')==80:
                    a.smoke.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(a.smoke));dump(a.smoke.with_suffix('.json'),{'frame':80,'points':window.data['actual_points'],'clusters':len(window.data['clusters']),'states':len(window.states),'native_qt':True,'server':False});app.quit()
        timer=C.QTimer();timer.timeout.connect(smoke);timer.start(250)
    signal.signal(signal.SIGTERM,lambda *_:window.close());signal.signal(signal.SIGINT,lambda *_:window.close())
    return app.exec_()
if __name__=='__main__':sys.exit(main())
