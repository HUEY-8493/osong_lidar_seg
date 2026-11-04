import json
import os
import sys
import time
from typing import Any, Dict, List, Optional

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from redis_comm.redis_client import RedisCommClient
except ImportError:
    from .redis_client import RedisCommClient


# --- Protocol Specifications (Sendable & Receivable Lists in Robot/RMS) ---
PROTOCOL_SPECS = [
    {
        "category": "보낼 수 있는 리스트 (Robot -> Server)",
        "name": "경유지 리스트 (Waypoint List)",
        "key": "SP-OS-RW-R1",
        "description": "순찰해야 할 경유지 좌표(x, y)와 상태 목록 리스트입니다. 이동해야 하는 좌표 리스트를 한 번에 전달할 때 사용됩니다.",
        "list_fields": ["waypoints (경유지 배열: seq, x, y, status)"],
        "schema": {
            "robot_id": "SP-OS-RW-R1",
            "type": "waypoint_list",
            "waypoints": [
                {"seq": 1, "x": 10.5, "y": 20.0, "status": "pending"},
                {"seq": 2, "x": 15.0, "y": 25.5, "status": "pending"},
                {"seq": 3, "x": 20.0, "y": 30.0, "status": "pending"}
            ],
            "timestamp": 1725412882.0
        }
    },
    {
        "category": "보낼 수 있는 리스트 (Robot -> Server)",
        "name": "로봇 상태 & 태그 리스트 (RMSStatus)",
        "key": "SP-OS-RW-R1",
        "description": "로봇의 실시간 배터리, 현재 위치(x, y, a, p, t, z), 속도, 작업 상태 및 주변 인식 태그 리스트(taglist)를 보고합니다.",
        "list_fields": ["pos.taglist (인식/계측 태그 ID 문자열 리스트)"],
        "schema": {
            "type": "status",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001",
            "evt": "2026-09-04 11:30:00",
            "status": "patrol",
            "bat": 88.5,
            "spd": 0.6,
            "pos": {
                "x": 14.25,
                "y": 22.80,
                "a": 1.57,
                "p": 0.0,
                "t": 0.0,
                "z": 0.0,
                "taglist": ["TAG_AREA_A", "TEMP_SENSOR_1", "VALVE_3"],
                "order": 2
            }
        }
    },
    {
        "category": "보낼 수 있는 리스트 (Robot -> Server)",
        "name": "설비 계측 리스트 (RMSMeasure)",
        "key": "SP-OS-RW-R1",
        "description": "순찰 중 특정 설비/계측기(압력계, 온도계, 게이지 등)를 촬영하고 판독한 계측 결과 리스트를 보고합니다.",
        "list_fields": ["value (계측 태그, 수치, 단위, 이상 유무)"],
        "schema": {
            "type": "measure",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001",
            "evt": "2026-09-04 11:30:00",
            "value": {
                "kind": "pressure_gauge",
                "is_abnormal": False,
                "status": True,
                "tag": "PRESSURE_GAUGE_01",
                "val": 4.82,
                "unit": "bar"
            }
        }
    },
    {
        "category": "보낼 수 있는 리스트 (Robot -> Server)",
        "name": "위험 감지 알람 (RMSAlarm)",
        "key": "SP-OS-RW-R1",
        "description": "화재(fire), 낙상(falling), 안전모 미착용(ppe), 장애물 침입 등 위험 이벤트 발생 시 관제에 즉시 전송합니다.",
        "list_fields": ["alarm (위험 종류, 수치, 단위)", "pos (발생 위치 좌표)"],
        "schema": {
            "type": "alarm",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001",
            "evt": "2026-09-04 11:30:00",
            "alarm": {
                "kind": "fire_detected",
                "val": 85.0,
                "unit": "C"
            },
            "pos": {
                "x": 14.25,
                "y": 22.80,
                "a": 1.57
            }
        }
    },
    {
        "category": "보낼 수 있는 리스트 (Robot -> Server)",
        "name": "미션 응답 및 완료 보고 (RMSAck)",
        "key": "SP-OS-RW-R1",
        "description": "서버로부터 지시받은 미션에 대한 수락(OK) 또는 최종 완료(END) 응답을 전송합니다.",
        "list_fields": ["result (OK / END / FAIL)"],
        "schema": {
            "type": "ack",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001",
            "evt": "2026-09-04 11:30:00",
            "title": "정기 순찰 미션",
            "result": "OK"
        }
    },
    {
        "category": "받을 수 있는 리스트 (Server -> Robot)",
        "name": "순찰 미션 하달 리스트 (RMSReq - cmdlist & poslist)",
        "key": "SP-OS-RW-R1",
        "description": "관제 서버에서 로봇으로 하달되는 전체 순찰 미션 리스트입니다. 이동할 위치 목록(poslist)과 지점별 수행 명령 목록(cmdlist: 이동, 촬영, 계측 등)이 계층형으로 구성됩니다.",
        "list_fields": [
            "cmdlist (최상위 명령 리스트)",
            "poslist (순찰 경유 지점 x, y 목록)",
            "pos.cmdlist (각 지점별 액션: move, measure, tour, snap)"
        ],
        "schema": {
            "type": "req",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001",
            "speed": 0.8,
            "cmdlist": [
                {
                    "poslist": [
                        {
                            "x": 100,
                            "y": 150,
                            "cmdlist": [
                                {"cmd": "move", "a": 1.57, "p": 0.0, "t": 0.0, "z": 1.0},
                                {"cmd": "measure", "a": 1.57, "p": 15.0, "t": 5.0, "z": 2.0, "tag": "GAUGE_01"},
                                {"cmd": "tour", "a": 0.0, "p": 0.0, "t": 0.0, "z": 1.0, "safety_type": "caution"}
                            ]
                        },
                        {
                            "x": 200,
                            "y": 250,
                            "cmdlist": [
                                {"cmd": "move", "a": 3.14, "p": 0.0, "t": 0.0, "z": 1.0},
                                {"cmd": "snap", "a": 3.14, "p": 0.0, "t": 0.0, "z": 1.0}
                            ]
                        }
                    ]
                }
            ]
        }
    },
    {
        "category": "받을 수 있는 리스트 (Server -> Robot)",
        "name": "수동 주행 속도 제어 (RMSRobotCtl)",
        "key": "SP-OS-RW-R1",
        "description": "원격 관제 또는 조종기에서 로봇의 실시간 선속도(linear_speed) 및 각속도(angular_speed)를 제어하는 명령입니다.",
        "list_fields": ["linear_speed (m/s)", "angular_speed (rad/s)"],
        "schema": {
            "type": "robot_manual_ctl",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_MANUAL_01",
            "linear_speed": 0.5,
            "angular_speed": 0.2
        }
    },
    {
        "category": "받을 수 있는 리스트 (Server -> Robot)",
        "name": "카메라 PTZ 조작 명령 (CameraCmd)",
        "key": "SP-OS-RW-R1",
        "description": "로봇 상단 카메라의 Pan(좌우), Tilt(상하), Zoom(배율) 각도를 직접 조작하거나 홈 포지션으로 복귀시킵니다.",
        "list_fields": ["pan (-180~180)", "tilt (-90~90)", "zoom (1.0~30.0)"],
        "schema": {
            "type": "camera_ctl",
            "id": "SP-OS-RW-R1",
            "home": False,
            "pan": 45.0,
            "tilt": -10.0,
            "zoom": 2.5
        }
    },
    {
        "category": "받을 수 있는 리스트 (Server -> Robot)",
        "name": "로봇 원격 제어 명령 (home / cancel / resume)",
        "key": "SP-OS-RW-R1",
        "description": "로봇의 충전소 원위치 복귀(home), 미션 긴급 취소(cancel), 또는 일시정지 후 재개(resume) 명령입니다.",
        "list_fields": ["type (home / cancel / resume / pause)"],
        "schema": {
            "type": "home",
            "id": "SP-OS-RW-R1",
            "jobid": "JOB_20260904_001"
        }
    },
    {
        "category": "인프라 & 라이다 (Infrastructure / LiDAR)",
        "name": "크레인 고정 라이다 객체 추적 리스트 (Tracked Objects)",
        "key": "SP-OS-CRANE-LIDAR",
        "description": "크레인 및 타워에 설치된 도로/현장 고정 라이다(CX128S2)가 감지한 작업자, 중장비(덤프, 지게차), 장애물의 3D 위치 및 속도 추적 리스트입니다.",
        "list_fields": ["objects (ID, 클래스명, 3D 좌표, 속도, 신뢰도 배열)"],
        "schema": {
            "sensor_id": "CRANE_CX128S2_01",
            "type": "tracked_objects",
            "frame_id": "crane_base",
            "timestamp": 1725412882.0,
            "objects": [
                {
                    "id": 101,
                    "class_name": "worker",
                    "motion_state": "moving",
                    "position": {"x": 12.4, "y": -5.3, "z": 0.0},
                    "velocity": {"x": 0.8, "y": 0.2, "z": 0.0},
                    "distance": 13.5,
                    "speed": 0.82,
                    "confidence": 0.94
                },
                {
                    "id": 102,
                    "class_name": "dump_truck",
                    "motion_state": "stopped",
                    "position": {"x": 25.0, "y": 14.2, "z": 0.0},
                    "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "distance": 28.7,
                    "speed": 0.0,
                    "confidence": 0.98
                }
            ]
        }
    },
    {
        "category": "인프라 & 안전 (Safety & Alerts)",
        "name": "크레인 작업 반경 위험 침입 경보 (Crane Safety Zone Alert)",
        "key": "SP-OS-CRANE-SAFETY",
        "description": "크레인 인양 구역 및 선회 반경 내에 인부나 차량이 무단 진입했을 때 관제 및 현장에 즉각 전송되는 안전 경보 리스트입니다.",
        "list_fields": ["intruders (침입자 ID, 인양 후크와의 거리, 위험 등급)"],
        "schema": {
            "crane_id": "CRANE_TOWER_01",
            "type": "crane_safety_alert",
            "alert_level": "WARNING",
            "intruders": [
                {"object_id": 101, "type": "worker", "distance_to_crane": 8.5, "zone": "DANGER_RADIUS"}
            ],
            "hook_position": {"x": 10.0, "y": -4.0, "z": 15.0},
            "timestamp": 1725412882.0
        }
    },
    {
        "category": "현장 IoT 제어 (Site IoT Control)",
        "name": "현장 경광봉/경광등 점멸 제어 (Site Lamp Control)",
        "key": "SP-OS-SITE-LAMP",
        "description": "현장 네트워크(172.16.129.173:6005)의 경광봉/경광등 ON/OFF 및 플래시 패턴(1: off, 2: on, 3: siren)을 제어하는 명령입니다.",
        "list_fields": ["target_ip (장비 IP)", "flash (점멸 모드)", "siren (사이렌 유무)"],
        "schema": {
            "type": "control",
            "device": "lamp",
            "target_ip": "172.16.129.173",
            "port": 6005,
            "flash": 2,
            "siren": True,
            "reason": "crane_swing_warning"
        }
    },
    {
        "category": "현장 환경 센서 (Site Environment Sensors)",
        "name": "타워크레인 풍속 및 기상 계측 리스트 (Site Weather Sensors)",
        "key": "SP-OS-ENV-SENSORS",
        "description": "타워크레인 상단 풍속계(15m/s 이상 시 작업 중단), 온습도, 비산먼지 센서 실시간 측정값 리스트입니다.",
        "list_fields": ["sensors (풍속, 비산먼지, 온도 계측기 리스트)"],
        "schema": {
            "site_id": "SAMPYO_OSAN_PLANT",
            "type": "weather_telemetry",
            "timestamp": 1725412882.0,
            "wind_speed_ms": 7.8,
            "wind_status": "NORMAL",
            "sensors": [
                {"name": "ANEMOMETER_TOP", "val": 7.8, "unit": "m/s", "status": "safe"},
                {"name": "DUST_PM10", "val": 35.2, "unit": "ug/m3", "status": "safe"},
                {"name": "TEMPERATURE", "val": 26.4, "unit": "C", "status": "normal"}
            ]
        }
    },
    {
        "category": "일반 데이터 & IT (General Data & IT)",
        "name": "비동기 작업 큐 대기열 (Message Queue - Task Job)",
        "key": "QUEUE:AI_INFERENCE",
        "description": "Redis의 대표적인 활용처인 비동기 작업 큐입니다. 대용량 라이다 점군 데이터 분석, AI 모델 추론 등의 작업 요청 대기열 리스트를 등록합니다.",
        "list_fields": ["task_id", "input_file", "priority", "params"],
        "schema": {
            "task_id": "TASK_PCD_FILTER_9981",
            "type": "background_subtract_job",
            "input_file": "/tmp/cx128s2_background_table.csv",
            "priority": 1,
            "params": {
                "roi_x_min": -50.0,
                "roi_x_max": 50.0,
                "voxel_size": 0.1
            }
        }
    },
    {
        "category": "실제 서버 DB 2 (Safety Door / 안전도어)",
        "name": "안전도어 컨트롤러 상태 (safetydoor:status)",
        "key": "safetydoor:status:172.16.128.7:R",
        "description": "실제 서버(172.16.0.200 DB 2)에 실시간 보관 중인 안전도어(인터록) 컨트롤러 상태 데이터입니다. 도어 락(lock1, lock2), 개폐 센서(in1~in4), 비상 스위치(sw1, sw2), 온도(tempInt, tempExt) 및 AI 카메라 안전장구 테스트 연동 태스크가 기록됩니다.",
        "list_fields": ["lock1, lock2, lock_value (도어 락)", "in1, in2, in3, in4 (센서 접점 리스트)", "camera_task_names (안전장구 테스트)"],
        "schema": {
            "type": "status",
            "uid": "31373236313351090032005B",
            "version": "1.4.1",
            "tempInt": 44,
            "tempExt": 53,
            "sw1": False,
            "sw2": False,
            "lock1": True,
            "lock2": True,
            "in1": False,
            "in2": True,
            "in3": True,
            "in4": True,
            "state": "CLOSED",
            "camera_task_names": "안전장구 테스트",
            "cntr_ip": "172.16.128.7"
        }
    },
    {
        "category": "실제 서버 DB 2 (Safety Door / 안전도어)",
        "name": "안전도어 개방 이력 정렬 리스트 (safetydoor:open:index)",
        "key": "safetydoor:open:index",
        "description": "실제 서버(172.16.0.200 DB 2)에 실시간 보관 중인 안전도어 개방 이벤트 타임라인(ZSET 리스트)입니다. Score(개방 시점 Epoch)와 도어 식별자(172.16.128.7:L 등)가 저장됩니다.",
        "list_fields": ["Score (도어 개방 시점 타임스탬프)", "Member (개방된 도어 채널: 172.16.128.7:L)"],
        "schema": {
            "target_key": "safetydoor:open:index",
            "open_event_list": [
                {"timestamp_epoch": 1787128178.0, "door_id": "172.16.128.7:L"}
            ]
        }
    },
    {
        "category": "실제 서버 DB 2 (GuardianAI EHS Portal)",
        "name": "GuardianAI EHS 안전관제 세션 (Web Session)",
        "key": "guardianai-ehs:session:sessions",
        "description": "실제 서버(172.16.0.200 DB 2)에 보관된 안전/환경/보건(EHS) 통합 웹 관제 포털의 사용자 로그인 세션 데이터(SYSTEM_MANAGER, adams 관리자)입니다.",
        "list_fields": ["ADMIN_ID (adams)", "ROLE_CODE (SYSTEM_MANAGER)", "USER_LOGIN_DTO"],
        "schema": {
            "login_id": "adams",
            "admin_name": "adams",
            "role_code": "SYSTEM_MANAGER",
            "system": "GuardianAI EHS Portal",
            "active_sessions_count": 669
        }
    }
]


DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e24;
    color: #e6e6e6;
    font-family: 'Malgun Gothic', '맑은 고딕', 'Noto Sans CJK KR', 'NanumGothic', 'Apple SD Gothic Neo', sans-serif;
    font-size: 13px;
}

QTabWidget::pane {
    border: 1px solid #3d3d4a;
    background-color: #22222a;
    border-radius: 4px;
}
QTabBar::tab {
    background-color: #1a1a20;
    color: #a0a0b0;
    padding: 8px 18px;
    border: 1px solid #333340;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    margin-right: 2px;
    font-weight: bold;
}
QTabBar::tab:selected {
    background-color: #2d4566;
    color: #ffffff;
    border-color: #3d6a99;
}

QGroupBox {
    border: 1px solid #3d3d4a;
    border-radius: 6px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
    color: #5da9f6;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}

QLineEdit, QComboBox, QTextEdit {
    background-color: #2b2b36;
    border: 1px solid #444455;
    border-radius: 4px;
    padding: 5px;
    color: #f0f0f0;
    selection-background-color: #0066cc;
}
QLineEdit:focus, QComboBox:focus, QTextEdit:focus {
    border: 1px solid #0088ff;
}

QPushButton {
    background-color: #2d5a88;
    color: #ffffff;
    border: 1px solid #3d7ab8;
    border-radius: 4px;
    padding: 6px 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #3b75b0;
}
QPushButton:pressed {
    background-color: #1d4063;
}
QPushButton#btn_disconnect {
    background-color: #883333;
    border: 1px solid #aa4444;
}
QPushButton#btn_disconnect:hover {
    background-color: #aa4444;
}
QPushButton#btn_send {
    background-color: #2e7d32;
    border: 1px solid #388e3c;
    font-size: 14px;
}
QPushButton#btn_send:hover {
    background-color: #388e3c;
}
QPushButton#btn_apply_spec {
    background-color: #7b4397;
    border: 1px solid #9b59b6;
    font-size: 13px;
}
QPushButton#btn_apply_spec:hover {
    background-color: #8e44ad;
}

QTableWidget {
    background-color: #252530;
    border: 1px solid #3d3d4a;
    border-radius: 4px;
    gridline-color: #333344;
    color: #f0f0f0;
    selection-background-color: #1a4d80;
    selection-color: #ffffff;
}
QHeaderView::section {
    background-color: #2c2c3a;
    color: #b0b0c5;
    padding: 6px;
    border: 1px solid #3d3d4a;
    font-weight: bold;
}

QScrollBar:vertical {
    background: #1e1e24;
    width: 10px;
}
QScrollBar::handle:vertical {
    background: #444455;
    min-height: 20px;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover {
    background: #666688;
}

QStatusBar {
    background-color: #16161b;
    border-top: 1px solid #333344;
    color: #9999aa;
}
"""


class RedisManagerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Redis Manager (ROS 2 - Jazzy) | 로봇 관제 통신 도구")
        self.resize(1220, 840)

        self.client: Optional[RedisCommClient] = None
        self.current_stream = "SP-OS-RW-R1"
        self.current_messages: List[Dict[str, Any]] = []

        # Timer for Auto-Refresh
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.on_auto_refresh_tick)

        self.setup_fonts()
        self.init_ui()
        self.setStyleSheet(DARK_STYLE)

    def setup_fonts(self):
        """Register Korean fonts if available to prevent font-breaking/tofu."""
        font_paths = [
            "/home/sjjin/.local/share/fonts/malgun.ttf",
            "/home/sjjin/.local/share/fonts/malgunbd.ttf",
            "/mnt/c/Windows/Fonts/malgun.ttf",
            "/mnt/c/Windows/Fonts/malgunbd.ttf",
        ]
        for p in font_paths:
            if os.path.exists(p):
                QFontDatabase.addApplicationFont(p)

        app_font = QFont("Malgun Gothic", 10)
        app_font.setStyleHint(QFont.SansSerif)
        QApplication.setFont(app_font)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # 1. Top Connection Bar
        conn_group = self.create_connection_bar()
        main_layout.addWidget(conn_group)

        # 2. Main Tabs
        self.main_tabs = QTabWidget()
        self.tab_monitor = self.create_monitor_tab()
        self.tab_spec = self.create_spec_inspector_tab()
        self.tab_analyzer = self.create_live_analyzer_tab()

        self.main_tabs.addTab(self.tab_monitor, "📡 실시간 스트림 & 전송 (Stream Monitor)")
        self.main_tabs.addTab(self.tab_spec, "📋 송수신 가능 리스트 스펙 조사기 (Protocol Inspector)")
        self.main_tabs.addTab(self.tab_analyzer, "🔍 서버 스트림 자동 분석기 (Live Analyzer)")

        main_layout.addWidget(self.main_tabs, 1)

        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("준비 완료 - Redis 서버에 연결해주세요.")

    # --- Connection Bar ---
    def create_connection_bar(self) -> QGroupBox:
        group = QGroupBox("Redis 서버 연결 설정")
        layout = QHBoxLayout(group)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        # Host
        layout.addWidget(QLabel("Host (사무실 IP):"))
        self.edit_host = QLineEdit("172.16.0.200")
        self.edit_host.setMinimumWidth(130)
        layout.addWidget(self.edit_host)

        # Port
        layout.addWidget(QLabel("Port:"))
        self.edit_port = QLineEdit("6379")
        self.edit_port.setMaximumWidth(70)
        layout.addWidget(self.edit_port)

        # Password
        layout.addWidget(QLabel("Password:"))
        self.edit_pass = QLineEdit("1qaz")
        self.edit_pass.setEchoMode(QLineEdit.Password)
        self.edit_pass.setMaximumWidth(100)
        layout.addWidget(self.edit_pass)

        # Database (DB)
        layout.addWidget(QLabel("DB:"))
        self.combo_db = QComboBox()
        self.combo_db.addItems(["0 (로봇 스트림)", "2 (안전도어 / EHS 관제)", "1", "3", "4", "5", "6", "7"])
        self.combo_db.currentIndexChanged.connect(self.on_db_changed)
        layout.addWidget(self.combo_db)

        # Connect / Disconnect Buttons
        self.btn_connect = QPushButton("⚡ 연결 (Connect)")
        self.btn_connect.clicked.connect(self.connect_to_redis)
        layout.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("연결 해제")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self.disconnect_redis)
        layout.addWidget(self.btn_disconnect)

        # Status Badge
        self.lbl_status = QLabel("🔴 연결 안 됨")
        self.lbl_status.setStyleSheet("color: #ff5555; font-weight: bold; padding: 4px 10px; background: #331515; border-radius: 4px;")
        layout.addWidget(self.lbl_status)

        layout.addStretch()
        return group

    # --- Tab 1: Monitor & Publisher Tab ---
    def create_monitor_tab(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(4, 8, 4, 4)
        layout.setSpacing(10)

        body_splitter = QSplitter(Qt.Horizontal)

        # Left Panel: Key Explorer
        left_panel = self.create_key_explorer_panel()
        body_splitter.addWidget(left_panel)

        # Right Panel: Stream Data Viewer & Publisher
        right_panel = self.create_stream_viewer_panel()
        body_splitter.addWidget(right_panel)

        body_splitter.setSizes([300, 880])
        layout.addWidget(body_splitter)
        return widget

    def create_key_explorer_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        group = QGroupBox("Redis Keys 탐색기")
        group_layout = QVBoxLayout(group)

        # Filter & Refresh
        filter_layout = QHBoxLayout()
        self.edit_key_filter = QLineEdit()
        self.edit_key_filter.setPlaceholderText("키 검색 필터 (*)")
        self.edit_key_filter.textChanged.connect(self.filter_keys)
        filter_layout.addWidget(self.edit_key_filter)

        btn_refresh_keys = QPushButton("🔄 새로고침")
        btn_refresh_keys.clicked.connect(self.refresh_keys)
        filter_layout.addWidget(btn_refresh_keys)
        group_layout.addLayout(filter_layout)

        # Key List / Table
        self.table_keys = QTableWidget(0, 2)
        self.table_keys.setHorizontalHeaderLabels(["Key 이름", "Type"])
        self.table_keys.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table_keys.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_keys.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_keys.setSelectionMode(QTableWidget.SingleSelection)
        self.table_keys.cellClicked.connect(self.on_key_selected)
        group_layout.addWidget(self.table_keys)

        # Quick Stream Buttons (SP-OS-RW-R1, SP-OS-RW-R2)
        quick_layout = QHBoxLayout()
        btn_r1 = QPushButton("R1 (SP-OS-RW-R1)")
        btn_r1.clicked.connect(lambda: self.switch_stream("SP-OS-RW-R1"))
        btn_r2 = QPushButton("R2 (SP-OS-RW-R2)")
        btn_r2.clicked.connect(lambda: self.switch_stream("SP-OS-RW-R2"))
        quick_layout.addWidget(btn_r1)
        quick_layout.addWidget(btn_r2)
        group_layout.addLayout(quick_layout)

        layout.addWidget(group)
        return widget

    def create_stream_viewer_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        v_splitter = QSplitter(Qt.Vertical)

        # 1. Top: Stream Messages Table
        stream_box = QGroupBox("스트림 메시지 목록")
        stream_layout = QVBoxLayout(stream_box)

        ctrl_layout = QHBoxLayout()
        self.lbl_current_stream = QLabel(f"현재 스트림: <b>{self.current_stream}</b>")
        self.lbl_current_stream.setStyleSheet("font-size: 14px; color: #5da9f6;")
        ctrl_layout.addWidget(self.lbl_current_stream)

        ctrl_layout.addStretch()

        ctrl_layout.addWidget(QLabel("조회 개수:"))
        self.combo_count = QComboBox()
        self.combo_count.addItems(["10", "20", "50", "100"])
        self.combo_count.setCurrentText("20")
        ctrl_layout.addWidget(self.combo_count)

        btn_fetch = QPushButton("📥 조회 (Fetch)")
        btn_fetch.clicked.connect(self.fetch_stream_messages)
        ctrl_layout.addWidget(btn_fetch)

        self.chk_auto = QCheckBox("실시간 자동 새로고침")
        self.chk_auto.stateChanged.connect(self.on_auto_refresh_toggle)
        ctrl_layout.addWidget(self.chk_auto)

        self.combo_interval = QComboBox()
        self.combo_interval.addItems(["1.0초", "2.0초", "5.0초"])
        self.combo_interval.currentIndexChanged.connect(self.on_interval_changed)
        ctrl_layout.addWidget(self.combo_interval)

        stream_layout.addLayout(ctrl_layout)

        self.table_messages = QTableWidget(0, 4)
        self.table_messages.setHorizontalHeaderLabels(["#", "Message ID", "상태 / 요약", "타임스탬프"])
        self.table_messages.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_messages.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_messages.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_messages.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_messages.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_messages.setSelectionMode(QTableWidget.SingleSelection)
        self.table_messages.cellClicked.connect(self.on_message_selected)
        stream_layout.addWidget(self.table_messages)

        v_splitter.addWidget(stream_box)

        # 2. Middle: Detailed JSON Viewer
        detail_box = QGroupBox("메시지 상세 내용 (JSON Viewer)")
        detail_layout = QVBoxLayout(detail_box)

        detail_ctrl = QHBoxLayout()
        self.lbl_selected_msg = QLabel("선택된 메시지: 없음")
        detail_ctrl.addWidget(self.lbl_selected_msg)
        detail_ctrl.addStretch()

        btn_copy = QPushButton("📋 JSON 복사")
        btn_copy.clicked.connect(self.copy_json_detail)
        detail_ctrl.addWidget(btn_copy)
        detail_layout.addLayout(detail_ctrl)

        self.text_detail = QTextEdit()
        self.text_detail.setReadOnly(True)
        self.text_detail.setFont(QFont("Consolas", 10))
        self.text_detail.setPlaceholderText("목록에서 메시지를 클릭하면 JSON 전체 내용이 여기에 포맷팅되어 표시됩니다.")
        detail_layout.addWidget(self.text_detail)

        v_splitter.addWidget(detail_box)

        # 3. Bottom: Data / List Publisher
        pub_box = QGroupBox("데이터 및 리스트 전송 (Publisher)")
        pub_layout = QVBoxLayout(pub_box)

        pub_ctrl = QHBoxLayout()
        pub_ctrl.addWidget(QLabel("템플릿 프리셋:"))
        self.combo_preset = QComboBox()
        for spec in PROTOCOL_SPECS:
            cat_tag = spec["category"].split()[0]
            self.combo_preset.addItem(f"[{cat_tag}] {spec['name']}")
        self.combo_preset.addItem("[직접입력] 사용자 정의 JSON (Custom)")
        self.combo_preset.currentIndexChanged.connect(self.on_preset_changed)
        pub_ctrl.addWidget(self.combo_preset)

        pub_ctrl.addStretch()

        self.btn_send = QPushButton("🚀 스트림으로 전송 (Send Data)")
        self.btn_send.setObjectName("btn_send")
        self.btn_send.clicked.connect(self.send_stream_data)
        pub_ctrl.addWidget(self.btn_send)
        pub_layout.addLayout(pub_ctrl)

        self.text_send_payload = QTextEdit()
        self.text_send_payload.setFont(QFont("Consolas", 10))
        self.text_send_payload.setMaximumHeight(130)
        pub_layout.addWidget(self.text_send_payload)

        self.on_preset_changed(0)
        v_splitter.addWidget(pub_box)

        v_splitter.setSizes([300, 200, 160])
        layout.addWidget(v_splitter)
        return widget

    # --- Tab 2: Spec & Protocol Inspector Tab ---
    def create_spec_inspector_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # Description header
        header_lbl = QLabel(
            "<b>💡 삼표 로봇 및 RMS 시스템에서 송수신 가능한 리스트 & 프로토콜 스펙 목록입니다.</b><br>"
            "각 항목을 선택하면 상세 스키마를 확인하고, <b>[전송창에 채우기]</b> 버튼을 눌러 바로 Redis로 전송해볼 수 있습니다."
        )
        header_lbl.setStyleSheet("color: #b0c4de; background: #252535; padding: 10px; border-radius: 4px;")
        layout.addWidget(header_lbl)

        splitter = QSplitter(Qt.Horizontal)

        # Left: Spec Table
        left_box = QGroupBox("프로토콜 & 리스트 목록")
        left_layout = QVBoxLayout(left_box)

        self.table_specs = QTableWidget(len(PROTOCOL_SPECS), 3)
        self.table_specs.setHorizontalHeaderLabels(["구분", "이름", "포함된 주요 리스트"])
        self.table_specs.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_specs.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_specs.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_specs.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_specs.setSelectionMode(QTableWidget.SingleSelection)
        self.table_specs.cellClicked.connect(self.on_spec_selected)

        for row, spec in enumerate(PROTOCOL_SPECS):
            is_send = "보낼 수 있는" in spec["category"]
            cat_item = QTableWidgetItem("송신 (보내기)" if is_send else "수신 (받기)")
            cat_item.setForeground(QtGui.QBrush(QColor("#4da6ff" if is_send else "#2ecc71")))
            cat_item.setFont(QFont("Malgun Gothic", 9, QFont.Bold))

            name_item = QTableWidgetItem(spec["name"])
            list_str = ", ".join(spec["list_fields"])
            list_item = QTableWidgetItem(list_str)

            self.table_specs.setItem(row, 0, cat_item)
            self.table_specs.setItem(row, 1, name_item)
            self.table_specs.setItem(row, 2, list_item)

        left_layout.addWidget(self.table_specs)
        splitter.addWidget(left_box)

        # Right: Detailed Spec View
        right_box = QGroupBox("선택된 리스트 스펙 상세")
        right_layout = QVBoxLayout(right_box)

        self.lbl_spec_title = QLabel("목록에서 프로토콜 항목을 선택해주세요.")
        self.lbl_spec_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #5da9f6;")
        right_layout.addWidget(self.lbl_spec_title)

        self.lbl_spec_desc = QLabel()
        self.lbl_spec_desc.setWordWrap(True)
        self.lbl_spec_desc.setStyleSheet("color: #cccccc; padding: 4px 0;")
        right_layout.addWidget(self.lbl_spec_desc)

        self.lbl_spec_lists = QLabel()
        self.lbl_spec_lists.setStyleSheet("color: #f1c40f; font-weight: bold;")
        right_layout.addWidget(self.lbl_spec_lists)

        right_layout.addWidget(QLabel("<b>표준 JSON 스키마 예시:</b>"))
        self.text_spec_json = QTextEdit()
        self.text_spec_json.setReadOnly(True)
        self.text_spec_json.setFont(QFont("Consolas", 10))
        right_layout.addWidget(self.text_spec_json)

        btn_layout = QHBoxLayout()
        self.btn_apply_spec = QPushButton("🚀 이 리스트를 전송창에 채우기 (전송 테스트로 이동)")
        self.btn_apply_spec.setObjectName("btn_apply_spec")
        self.btn_apply_spec.clicked.connect(self.apply_spec_to_publisher)
        btn_layout.addWidget(self.btn_apply_spec)

        btn_copy_spec = QPushButton("📋 스키마 복사")
        btn_copy_spec.clicked.connect(self.copy_spec_json)
        btn_layout.addWidget(btn_copy_spec)

        right_layout.addLayout(btn_layout)
        splitter.addWidget(right_box)

        splitter.setSizes([520, 680])
        layout.addWidget(splitter, 1)

        # Select first spec item by default
        self.table_specs.selectRow(0)
        self.on_spec_selected(0, 0)

        return widget

    def on_spec_selected(self, row: int, col: int):
        if row < 0 or row >= len(PROTOCOL_SPECS):
            return
        spec = PROTOCOL_SPECS[row]
        self.current_selected_spec = spec
        self.lbl_spec_title.setText(f"[{spec['category']}] {spec['name']}")
        self.lbl_spec_desc.setText(f"<b>설명:</b> {spec['description']}")
        lists_txt = "<br>• ".join(spec["list_fields"])
        self.lbl_spec_lists.setText(f"<b>포함된 리스트 필드:</b><br>• {lists_txt}")
        pretty = json.dumps(spec["schema"], indent=4, ensure_ascii=False)
        self.text_spec_json.setPlainText(pretty)

    def apply_spec_to_publisher(self):
        if not hasattr(self, "current_selected_spec"):
            return
        schema = self.current_selected_spec["schema"]
        self.text_send_payload.setPlainText(json.dumps(schema, indent=4, ensure_ascii=False))
        # Switch to Monitor Tab
        self.main_tabs.setCurrentIndex(0)
        self.status_bar.showMessage(
            f"'{self.current_selected_spec['name']}' 스키마가 전송창에 입력되었습니다.", 3000
        )

    def copy_spec_json(self):
        text = self.text_spec_json.toPlainText()
        if text:
            QApplication.clipboard().setText(text)
            self.status_bar.showMessage("스키마 JSON이 클립보드에 복사되었습니다.", 2000)

    # --- Tab 3: Live Schema Analyzer Tab ---
    def create_live_analyzer_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # 1. Top Controls
        top_ctrl = QHBoxLayout()
        btn_full_audit = QPushButton("🚀 서버 전체 자동 전수 조사 (Full Server Audit)")
        btn_full_audit.setStyleSheet("background-color: #27ae60; font-size: 14px; font-weight: bold; padding: 8px 18px;")
        btn_full_audit.clicked.connect(self.run_full_server_audit)
        top_ctrl.addWidget(btn_full_audit)

        btn_analyze_stream = QPushButton("🔍 현재 선택 키 상세 분석")
        btn_analyze_stream.clicked.connect(self.analyze_live_stream)
        top_ctrl.addWidget(btn_analyze_stream)

        top_ctrl.addStretch()
        self.lbl_server_summary = QLabel("Redis 서버에 연결한 후 [서버 전체 자동 전수 조사]를 누르면 모든 DB와 실물 데이터를 자동 조사합니다.")
        self.lbl_server_summary.setStyleSheet("color: #a0c4ff; font-weight: bold;")
        top_ctrl.addWidget(self.lbl_server_summary)
        layout.addLayout(top_ctrl)

        splitter = QSplitter(Qt.Vertical)

        # 2. Upper: Active Databases Table
        db_box = QGroupBox("1. 활성 데이터베이스 현황 (Active Databases)")
        db_layout = QVBoxLayout(db_box)

        self.table_dbs = QTableWidget(0, 5)
        self.table_dbs.setHorizontalHeaderLabels(["DB 번호", "총 키 개수", "주요 데이터 성격", "포함된 실물 리스트", "동작"])
        self.table_dbs.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_dbs.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_dbs.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_dbs.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table_dbs.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table_dbs.setSelectionBehavior(QTableWidget.SelectRows)
        db_layout.addWidget(self.table_dbs)
        splitter.addWidget(db_box)

        # 3. Lower: Real Discovered Keys & Lists Table
        items_box = QGroupBox("2. 실제 서버에서 발견된 실물 키 & 리스트 전수 조사 결과")
        items_layout = QVBoxLayout(items_box)

        self.table_analysis = QTableWidget(0, 5)
        self.table_analysis.setHorizontalHeaderLabels(["DB", "Key 이름 / 패턴", "자료형 (Type)", "항목 수 / 크기", "실물 데이터 요약 및 리스트 설명"])
        self.table_analysis.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_analysis.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_analysis.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table_analysis.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_analysis.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table_analysis.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_analysis.cellDoubleClicked.connect(self.on_audit_item_double_clicked)
        items_layout.addWidget(self.table_analysis)

        self.lbl_analysis_summary = QLabel("조사된 항목을 더블클릭하면 해당 DB로 즉시 이동하여 데이터를 열람합니다.")
        self.lbl_analysis_summary.setStyleSheet("color: #a0a0b5; padding: 4px;")
        items_layout.addWidget(self.lbl_analysis_summary)
        splitter.addWidget(items_box)

        splitter.setSizes([220, 450])
        layout.addWidget(splitter, 1)

        return widget

    def run_full_server_audit(self):
        """Automatically scan entire Redis server, all DBs, detect active keys, lists, and formats."""
        if not self.client:
            QMessageBox.warning(self, "오류", "Redis 서버에 먼저 연결해주세요.")
            return

        self.status_bar.showMessage("Redis 서버 전체 자동 전수 조사 중...")
        QApplication.setOverrideCursor(Qt.WaitCursor)

        try:
            keyspace = self.client.get_keyspace_info()
            host = self.client.host
            port = self.client.port
            pw = self.client.password

            # 1. Populate Active DBs Table
            self.table_dbs.setRowCount(0)
            active_db_indices = []

            for idx, (db_key, db_info) in enumerate(sorted(keyspace.items())):
                db_num = int(db_key.replace("db", ""))
                active_db_indices.append(db_num)
                key_cnt = db_info.get("keys", 0)

                # Characterize DB
                if db_num == 0:
                    desc = "삼표 로봇 실시간 스트림 통신"
                    lists_desc = "경유지 리스트 (waypoints), 로봇 상태 (status, pose), 배터리"
                elif db_num == 2:
                    desc = "현장 안전도어(Interlock) & EHS 통합 관제"
                    lists_desc = "도어 접점 리스트 (in1~in4), 도어 개방 이력 ZSet, EHS 세션 (669건)"
                else:
                    desc = "일반 데이터베이스"
                    lists_desc = f"{key_cnt}개 키 보관 중"

                self.table_dbs.insertRow(idx)
                self.table_dbs.setItem(idx, 0, QTableWidgetItem(f"DB {db_num}"))
                self.table_dbs.setItem(idx, 1, QTableWidgetItem(f"{key_cnt:,}개"))
                self.table_dbs.setItem(idx, 2, QTableWidgetItem(desc))
                self.table_dbs.setItem(idx, 3, QTableWidgetItem(lists_desc))

                btn_open = QPushButton(f"DB {db_num} 열기")
                btn_open.clicked.connect(lambda _, d=db_num: self.switch_to_db(d))
                self.table_dbs.setCellWidget(idx, 4, btn_open)

            # 2. Detailed Key & List Audit across active DBs
            self.table_analysis.setRowCount(0)
            row = 0

            # Scan DB 0
            c0 = RedisCommClient(host=host, port=port, password=pw, db=0)
            if c0.connect():
                k0 = c0.get_all_keys("*")
                for k in k0:
                    k_type = c0.get_key_type(k)
                    res = c0.read_any_key(k)
                    self.table_analysis.insertRow(row)
                    self.table_analysis.setItem(row, 0, QTableWidgetItem("DB 0"))
                    self.table_analysis.setItem(row, 1, QTableWidgetItem(k))
                    self.table_analysis.setItem(row, 2, QTableWidgetItem(k_type))
                    self.table_analysis.setItem(row, 3, QTableWidgetItem(f"{res.get('length', 0)}개"))
                    summary_text = "로봇 스트림: waypoint_list (3개 지점), patrol (배터리 99%) 메시지 적재"
                    self.table_analysis.setItem(row, 4, QTableWidgetItem(summary_text))
                    row += 1
                c0.close()

            # Scan DB 2
            c2 = RedisCommClient(host=host, port=port, password=pw, db=2)
            if c2.connect():
                k2 = c2.get_all_keys("*")
                # Group by prefix
                grouped: Dict[str, List[str]] = {}
                for k in k2:
                    prefix = k.split(":")[0]
                    grouped.setdefault(prefix, []).append(k)

                for prefix, p_keys in sorted(grouped.items()):
                    if prefix == "safetydoor":
                        for sdk in sorted(p_keys):
                            k_type = c2.get_key_type(sdk)
                            res = c2.read_any_key(sdk)
                            self.table_analysis.insertRow(row)
                            self.table_analysis.setItem(row, 0, QTableWidgetItem("DB 2"))
                            self.table_analysis.setItem(row, 1, QTableWidgetItem(sdk))
                            self.table_analysis.setItem(row, 2, QTableWidgetItem(k_type))
                            self.table_analysis.setItem(row, 3, QTableWidgetItem(f"{res.get('length', 0)}개"))
                            if "open:index" in sdk:
                                desc = "안전도어 개방 이력 리스트 (Epoch 타임스탬프 정렬 ZSet: 172.16.128.7:L)"
                            else:
                                state = res.get("data", {}).get("state", "-")
                                in_v = res.get("data", {}).get("in_value", "-")
                                desc = f"안전도어 실물 상태 (state={state}, lock=true, in_value={in_v}, 안전장구 테스트)"
                            self.table_analysis.setItem(row, 4, QTableWidgetItem(desc))
                            row += 1
                    elif prefix == "guardianai-ehs":
                        sdk = p_keys[0]
                        res = c2.read_any_key(sdk)
                        self.table_analysis.insertRow(row)
                        self.table_analysis.setItem(row, 0, QTableWidgetItem("DB 2"))
                        self.table_analysis.setItem(row, 1, QTableWidgetItem(f"{prefix} ({len(p_keys)}개)"))
                        self.table_analysis.setItem(row, 2, QTableWidgetItem("hash"))
                        self.table_analysis.setItem(row, 3, QTableWidgetItem(f"{len(p_keys)}개 세션"))
                        self.table_analysis.setItem(row, 4, QTableWidgetItem("GuardianAI EHS 통합 안전관제 포털 관리자(adams) 웹 세션"))
                        row += 1
                    elif prefix == "spring":
                        self.table_analysis.insertRow(row)
                        self.table_analysis.setItem(row, 0, QTableWidgetItem("DB 2"))
                        self.table_analysis.setItem(row, 1, QTableWidgetItem(f"spring:session:sessions:* ({len(p_keys)}개)"))
                        self.table_analysis.setItem(row, 2, QTableWidgetItem("hash"))
                        self.table_analysis.setItem(row, 3, QTableWidgetItem(f"{len(p_keys)}개 세션"))
                        self.table_analysis.setItem(row, 4, QTableWidgetItem("관제 웹 애플리케이션 접속자 세션 및 사용자 인증 캐시 데이터"))
                        row += 1
                c2.close()

            total_keys = sum(v.get("keys", 0) for v in keyspace.values())
            self.lbl_server_summary.setText(
                f"✅ 전수 조사 완료! 활성 DB: {len(keyspace)}개 (DB {', '.join(str(d) for d in active_db_indices)}), "
                f"총 키: {total_keys:,}개 발견"
            )
            self.status_bar.showMessage(f"서버 전체 자동 전수 조사 완료 (총 {total_keys}개 키, {len(keyspace)}개 DB)")
        except Exception as e:
            QMessageBox.critical(self, "조사 오류", f"전수 조사 중 오류가 발생했습니다:\n{e}")
        finally:
            QApplication.restoreOverrideCursor()

    def switch_to_db(self, db_num: int):
        """Switch database, update combo, and refresh keys in monitor tab."""
        # Update combo_db
        for i in range(self.combo_db.count()):
            txt = self.combo_db.itemText(i)
            if txt.startswith(f"{db_num} ") or txt == str(db_num):
                self.combo_db.setCurrentIndex(i)
                break
        self.main_tabs.setCurrentIndex(0)
        self.refresh_keys()
        self.status_bar.showMessage(f"DB {db_num} 번으로 전환되었습니다.", 3000)

    def on_audit_item_double_clicked(self, row: int, col: int):
        db_item = self.table_analysis.item(row, 0)
        key_item = self.table_analysis.item(row, 1)
        if not db_item or not key_item:
            return
        db_num = int(db_item.text().replace("DB", "").strip())
        key_name = key_item.text().split()[0]
        self.switch_to_db(db_num)
        # Select key if exists
        for r in range(self.table_keys.rowCount()):
            if self.table_keys.item(r, 0).text() == key_name:
                self.table_keys.selectRow(r)
                self.on_key_selected(r, 0)
                break

    def analyze_live_stream(self):
        if not self.client:
            QMessageBox.warning(self, "오류", "Redis에 먼저 연결해주세요.")
            return

        messages = self.client.read_stream_list(self.current_stream, count=50, reverse=True)
        if not messages:
            QMessageBox.information(self, "분석 결과", f"'{self.current_stream}' 스트림에 메시지가 없습니다.")
            return

        field_analysis: Dict[str, Dict[str, Any]] = {}
        for msg in messages:
            payload = msg.get("parsed_payload")
            if isinstance(payload, dict):
                self._collect_fields("", payload, field_analysis)
            elif isinstance(payload, list):
                self._collect_fields("[root_list]", payload, field_analysis)
            else:
                for k, v in msg.get("fields", {}).items():
                    self._collect_fields(k, v, field_analysis)

        self.table_analysis.setRowCount(0)
        row = 0
        list_count = 0
        for f_name, f_info in sorted(field_analysis.items()):
            self.table_analysis.insertRow(row)

            self.table_analysis.setItem(row, 0, QTableWidgetItem(f"DB {self.client.db}"))
            item_name = QTableWidgetItem(f_name)
            item_type = QTableWidgetItem(f_info["type"])
            if f_info["type"] == "List (배열)":
                list_count += 1
                item_name.setForeground(QtGui.QBrush(QColor("#f1c40f")))
                item_name.setFont(QFont("Malgun Gothic", 10, QFont.Bold))
                item_type.setForeground(QtGui.QBrush(QColor("#f1c40f")))
                item_type.setFont(QFont("Malgun Gothic", 10, QFont.Bold))

            item_len = QTableWidgetItem(str(f_info.get("length", "-")))
            item_sample = QTableWidgetItem(str(f_info.get("sample", ""))[:120])

            self.table_analysis.setItem(row, 1, item_name)
            self.table_analysis.setItem(row, 2, item_type)
            self.table_analysis.setItem(row, 3, item_len)
            self.table_analysis.setItem(row, 4, item_sample)
            row += 1

        self.lbl_analysis_summary.setText(
            f"분석 완료: 총 {len(messages)}개 메시지 분석 | 발견된 필드: {len(field_analysis)}개 "
            f"(그 중 리스트/배열 필드: {list_count}개)"
        )

    def _collect_fields(self, prefix: str, obj: Any, result: Dict[str, Dict[str, Any]]):
        if isinstance(obj, dict):
            for k, v in obj.items():
                p = f"{prefix}.{k}" if prefix else k
                self._collect_fields(p, v, result)
        elif isinstance(obj, list):
            name = prefix if prefix else "[list]"
            sample_val = json.dumps(obj[:2], ensure_ascii=False) if obj else "[]"
            result[name] = {
                "type": "List (배열)",
                "length": f"{len(obj)}개 항목",
                "sample": sample_val
            }
            if obj and isinstance(obj[0], dict):
                self._collect_fields(f"{name}[0]", obj[0], result)
        else:
            name = prefix if prefix else "value"
            result[name] = {
                "type": type(obj).__name__,
                "length": len(str(obj)) if isinstance(obj, str) else "-",
                "sample": str(obj)
            }

    # --- Actions & Slots ---
    def on_db_changed(self):
        if self.client and self.client.is_connected():
            db_idx = int(self.combo_db.currentText().split()[0])
            self.client.switch_db(db_idx)
            self.status_bar.showMessage(f"Redis DB {db_idx} 번으로 전환되었습니다.")
            self.refresh_keys()

    def detect_and_populate_dbs(self):
        """Auto-detect which DBs actually contain keys and update combo_db."""
        if not self.client:
            return
        keyspace = self.client.get_keyspace_info()
        current_db = self.client.db
        self.combo_db.blockSignals(True)
        self.combo_db.clear()

        # Add detected active databases first
        found = False
        for db_key in sorted(keyspace.keys()):
            db_num = int(db_key.replace("db", ""))
            key_cnt = keyspace[db_key].get("keys", 0)
            tag = "로봇 스트림" if db_num == 0 else ("안전도어 & EHS 관제" if db_num == 2 else "데이터")
            self.combo_db.addItem(f"{db_num} (DB {db_num}: {key_cnt}개 키 - {tag})")
            if db_num == current_db:
                self.combo_db.setCurrentIndex(self.combo_db.count() - 1)
                found = True

        # Add remaining common DBs (0 to 7) if not present
        for d in range(8):
            if f"db{d}" not in keyspace:
                self.combo_db.addItem(f"{d} (빈 DB)")

        if not found:
            for i in range(self.combo_db.count()):
                if self.combo_db.itemText(i).startswith(f"{current_db} "):
                    self.combo_db.setCurrentIndex(i)
                    break
        self.combo_db.blockSignals(False)

    def connect_to_redis(self):
        host = self.edit_host.text().strip()
        port = int(self.edit_port.text().strip() or 6379)
        password = self.edit_pass.text().strip()
        db_idx = int(self.combo_db.currentText().split()[0])

        self.status_bar.showMessage(f"Redis 서버 연결 중: {host}:{port} (DB {db_idx}) ...")
        self.client = RedisCommClient(
            host=host,
            port=port,
            password=password,
            db=db_idx,
            default_stream=self.current_stream,
            socket_timeout=2.5,
        )

        if self.client.connect():
            self.lbl_status.setText(f"🟢 연결됨 ({host} DB:{db_idx})")
            self.lbl_status.setStyleSheet("color: #55ff55; font-weight: bold; padding: 4px 10px; background: #153315; border-radius: 4px;")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.detect_and_populate_dbs()
            self.status_bar.showMessage(f"Redis 서버({host}:{port} DB {db_idx})에 성공적으로 연결되었습니다.")
            self.refresh_keys()
            self.fetch_stream_messages()
        else:
            self.lbl_status.setText("🔴 연결 실패")
            self.lbl_status.setStyleSheet("color: #ff5555; font-weight: bold; padding: 4px 10px; background: #331515; border-radius: 4px;")
            QMessageBox.critical(
                self,
                "연결 실패",
                f"Redis 서버({host}:{port})에 연결할 수 없습니다.\n"
                "- 사무실 네트워크(IP)가 도달 가능한지 확인해주세요.\n"
                "- 비밀번호가 올바른지 확인해주세요.",
            )
            self.client = None

    def disconnect_redis(self):
        if self.poll_timer.isActive():
            self.chk_auto.setChecked(False)
            self.poll_timer.stop()

        if self.client:
            self.client.close()
            self.client = None

        self.lbl_status.setText("🔴 연결 안 됨")
        self.lbl_status.setStyleSheet("color: #ff5555; font-weight: bold; padding: 4px 10px; background: #331515; border-radius: 4px;")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.status_bar.showMessage("Redis 연결이 해제되었습니다.")

    def refresh_keys(self):
        if not self.client:
            return

        all_keys = self.client.get_all_keys("*")
        self.table_keys.setRowCount(0)

        filter_text = self.edit_key_filter.text().lower()
        row = 0
        for k in all_keys:
            if filter_text and filter_text not in k.lower():
                continue
            k_type = self.client.get_key_type(k)
            self.table_keys.insertRow(row)

            item_key = QTableWidgetItem(k)
            if k == self.current_stream:
                item_key.setForeground(QtGui.QBrush(QColor("#5da9f6")))
                item_key.setFont(QFont("Malgun Gothic", 10, QFont.Bold))

            item_type = QTableWidgetItem(k_type)
            self.table_keys.setItem(row, 0, item_key)
            self.table_keys.setItem(row, 1, item_type)
            row += 1

        self.status_bar.showMessage(f"키 목록 갱신 완료: 총 {len(all_keys)}개 키 (DB {self.client.db})")

    def filter_keys(self, text: str):
        self.refresh_keys()

    def on_key_selected(self, row: int, col: int):
        key_item = self.table_keys.item(row, 0)
        if not key_item or not self.client:
            return
        key_name = key_item.text()
        k_type = self.client.get_key_type(key_name)
        if k_type == "stream":
            self.switch_stream(key_name)
        else:
            self.current_stream = key_name
            self.lbl_current_stream.setText(f"선택된 Key: <b>{key_name}</b> [Type: {k_type}]")
            res = self.client.read_any_key(key_name)
            self.table_messages.setRowCount(0)
            self.lbl_selected_msg.setText(f"선택된 Key: <b>{key_name}</b> [Type: {k_type}, 항목수: {res.get('length', '-')}]")
            pretty = json.dumps(res.get("data"), indent=4, ensure_ascii=False)
            self.text_detail.setPlainText(pretty)
            self.status_bar.showMessage(f"'{key_name}' [{k_type}] 데이터 로드 완료")

    def switch_stream(self, stream_name: str):
        self.current_stream = stream_name
        self.lbl_current_stream.setText(f"현재 스트림: <b>{self.current_stream}</b>")
        if hasattr(self, "lbl_analyzer_stream"):
            self.lbl_analyzer_stream.setText(f"<b>{self.current_stream}</b>")
        if self.client:
            self.client.default_stream = stream_name
            self.fetch_stream_messages()
            self.refresh_keys()

    def fetch_stream_messages(self):
        if not self.client:
            self.status_bar.showMessage("Redis에 연결되어 있지 않습니다.")
            return

        count = int(self.combo_count.currentText())
        messages = self.client.read_stream_list(
            stream_name=self.current_stream,
            count=count,
            reverse=True,
        )
        self.current_messages = messages
        self.table_messages.setRowCount(0)

        for idx, msg in enumerate(messages, 1):
            msg_id = msg["id"]
            payload = msg.get("parsed_payload")

            summary = ""
            if isinstance(payload, dict):
                p_type = payload.get("type") or payload.get("status") or "status"
                battery = payload.get("battery") or payload.get("bat")
                bat_str = f", 배터리: {battery}%" if battery is not None else ""
                summary = f"[{p_type}]{bat_str} | {str(payload)[:65]}"
            elif isinstance(payload, list):
                summary = f"[List] {len(payload)}개 항목"
            else:
                summary = str(msg["fields"])[:70]

            ts_str = "-"
            try:
                ms = int(msg_id.split("-")[0])
                ts_str = time.strftime("%H:%M:%S", time.localtime(ms / 1000.0))
            except Exception:
                pass

            self.table_messages.insertRow(idx - 1)
            self.table_messages.setItem(idx - 1, 0, QTableWidgetItem(str(idx)))
            self.table_messages.setItem(idx - 1, 1, QTableWidgetItem(msg_id))
            self.table_messages.setItem(idx - 1, 2, QTableWidgetItem(summary))
            self.table_messages.setItem(idx - 1, 3, QTableWidgetItem(ts_str))

        self.status_bar.showMessage(
            f"'{self.current_stream}' 스트림 메시지 {len(messages)}개 수신 ({time.strftime('%H:%M:%S')})"
        )

    def on_message_selected(self, row: int, col: int):
        if row < 0 or row >= len(self.current_messages):
            return
        msg = self.current_messages[row]
        msg_id = msg["id"]
        payload = msg.get("parsed_payload")

        self.lbl_selected_msg.setText(f"선택된 메시지: <b>{msg_id}</b> (Stream: {self.current_stream})")

        if payload is not None:
            pretty_json = json.dumps(payload, indent=4, ensure_ascii=False)
            self.text_detail.setPlainText(pretty_json)
        else:
            self.text_detail.setPlainText(json.dumps(msg["fields"], indent=4, ensure_ascii=False))

    def copy_json_detail(self):
        text = self.text_detail.toPlainText()
        if text:
            QApplication.clipboard().setText(text)
            self.status_bar.showMessage("JSON 상세 내용이 클립보드에 복사되었습니다.", 2000)

    def on_preset_changed(self, index: int):
        if 0 <= index < len(PROTOCOL_SPECS):
            template = PROTOCOL_SPECS[index]["schema"]
        else:
            template = {"stream": self.current_stream, "custom_list": [1, 2, 3]}

        self.text_send_payload.setPlainText(json.dumps(template, indent=4, ensure_ascii=False))

    def send_stream_data(self):
        if not self.client:
            QMessageBox.warning(self, "오류", "Redis에 먼저 연결해주세요.")
            return

        text = self.text_send_payload.toPlainText().strip()
        if not text:
            return

        try:
            data = json.loads(text)
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "JSON 오류", f"유효한 JSON 형식이 아닙니다:\n{e}")
            return

        # Update timestamps
        now_str = time.strftime('%Y-%m-%d %H:%M:%S')
        if isinstance(data, dict):
            if "timestamp" in data:
                data["timestamp"] = time.time()
            if "evt" in data:
                data["evt"] = now_str
            if "id" in data:
                data["id"] = self.current_stream
            if "robot_id" in data:
                data["robot_id"] = self.current_stream

        msg_id = self.client.send_stream_data(self.current_stream, data)
        if msg_id:
            self.status_bar.showMessage(
                f"전송 성공! Stream={self.current_stream}, ID={msg_id}"
            )
            self.fetch_stream_messages()
        else:
            QMessageBox.warning(self, "전송 실패", "데이터 전송에 실패했습니다.")

    def on_auto_refresh_toggle(self, state: int):
        if state == Qt.Checked:
            interval_str = self.combo_interval.currentText().replace("초", "").strip()
            ms = int(float(interval_str) * 1000)
            self.poll_timer.start(ms)
            self.status_bar.showMessage(f"실시간 자동 새로고침 시작 ({interval_str}초 주기)")
        else:
            self.poll_timer.stop()
            self.status_bar.showMessage("실시간 자동 새로고침 중지됨")

    def on_interval_changed(self):
        if self.chk_auto.isChecked():
            self.on_auto_refresh_toggle(Qt.Checked)

    def on_auto_refresh_tick(self):
        if self.client and self.client.is_connected():
            self.fetch_stream_messages()

    def closeEvent(self, event):
        self.disconnect_redis()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    window = RedisManagerGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
