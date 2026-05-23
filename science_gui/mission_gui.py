"""기지국 미션 GUI: 카메라(좌) + 탭형식 제어 패널(우)."""
from __future__ import annotations

import csv
import json
import os
import sys
from collections import deque
from datetime import datetime

import cv2
for _k in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_PLUGIN_PATH'):
    os.environ.pop(_k, None)

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from science_gui.capture_report import register_capture

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSizePolicy,
    QFileDialog, QSlider, QSpinBox, QCheckBox,
    QGroupBox, QLineEdit, QScrollArea, QTabWidget,
)

try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

CLR_IDLE = '#A6E3A1'
CLR_BUSY = '#F38BA8'
CLR_JAM  = '#FAB387'


def parse_scilab_feedback(line: str) -> dict[str, str] | None:
    """Arduino 피드백: ``FB,...`` 또는 ``SENSOR,...`` (KEY:VAL 콤마 구분)."""
    line = line.strip()
    if line.startswith('FB,'):
        body = line[3:]
    elif line.startswith('SENSOR,'):
        body = line[7:]
    else:
        return None
    out: dict[str, str] = {}
    for part in body.split(','):
        part = part.strip()
        if ':' not in part:
            continue
        k, v = part.split(':', 1)
        out[k.strip()] = v.strip()
    return out


def _fb_get(s: dict[str, str], *keys: str, default: str = '-') -> str:
    for k in keys:
        if k in s and str(s[k]).strip() != '':
            return str(s[k]).strip()
    return default

SLOT_INDEX_VALUES = [0, 1, 2, 3, 4, 5]

GRAPH_WINDOW_SEC = 60
GRAPH_STEP_SEC   = 4
GRAPH_MAX_POINTS = 15

# 실제 UDP 송신 레이아웃 (gst_sender 1x3 코드와 다름):
#   [ 토양 | 캐시 ]           ← 상단
#   [ 파노라마 스트림 (전폭) ] ← 하단
STREAM_REGIONS = (
    ('soil',      '토양', 'top',    0),
    ('cashe',     '캐시', 'top',    1),
    ('pano_live', '파노', 'bottom', 0),
)


def _crop_stream_region(merged: np.ndarray, row: str, col: int) -> np.ndarray:
    h, w = merged.shape[:2]
    mid = h // 2
    if row == 'top':
        tw = w // 2
        return merged[:mid, col * tw:(col + 1) * tw]
    return merged[mid:, :]


# 토양 축척 표시 (1280×720 기준 좌표 → 캡처 해상도에 비례 스케일)
_SOIL_SCALE_REF = (1280, 720)
_SOIL_SCALE_BAR = (570, 797, 5, 35)   # x1, x2, y_top, y_bottom (x2: +1/3 bar length, right only)
_SOIL_SCALE_TEXT_OFFSET_Y = 20
_SOIL_SCALE_LABEL = '20mm'


def _draw_soil_scale_bar(frame_bgr: np.ndarray) -> None:
    h, w = frame_bgr.shape[:2]
    ref_w, ref_h = _SOIL_SCALE_REF
    sx, sy = w / ref_w, h / ref_h
    scale = (sx + sy) / 2

    x1, x2, y_top, y_bottom = _SOIL_SCALE_BAR
    x1 = int(round(x1 * sx))
    x2 = int(round(x2 * sx))
    y_top = int(round(y_top * sy))
    y_bottom = int(round(y_bottom * sy))
    center_y = (y_top + y_bottom) // 2

    color = (0, 0, 255)
    thickness = max(1, int(round(2 * scale)))

    cv2.line(frame_bgr, (x1, y_top), (x1, y_bottom), color, thickness)
    cv2.line(frame_bgr, (x2, y_top), (x2, y_bottom), color, thickness)
    cv2.line(frame_bgr, (x1, center_y), (x2, center_y), color, thickness)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6 * scale
    text_thickness = max(1, int(round(2 * scale)))
    text_size, _ = cv2.getTextSize(_SOIL_SCALE_LABEL, font, font_scale, text_thickness)
    text_x = (x1 + x2) // 2 - text_size[0] // 2
    text_y = center_y + int(round(_SOIL_SCALE_TEXT_OFFSET_Y * sy)) + text_size[1]
    cv2.putText(
        frame_bgr, _SOIL_SCALE_LABEL, (text_x, text_y),
        font, font_scale, color, text_thickness, cv2.LINE_AA,
    )


STYLE = """
* { font-family: "Segoe UI", "Noto Sans", "Ubuntu", sans-serif; }
QMainWindow, QWidget { background-color: #1e1e2e; }
QLabel { color: #cdd6f4; }
QGroupBox {
    font-weight: bold; color: #cdd6f4; border: 1px solid #313244;
    border-radius: 8px; margin-top: 14px; padding-top: 18px;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 12px; padding: 2px 8px;
    background-color: #11111b; color: #f5f5f5; font-size: 13px;
    border-radius: 4px;
}
QLineEdit, QSpinBox {
    background: #313244; color: #cdd6f4; border: 1px solid #45475a;
    border-radius: 4px; padding: 4px 8px; font-size: 12px;
}
QCheckBox { color: #cdd6f4; }
QPushButton {
    background-color: #45475a; color: #cdd6f4; border: none;
    border-radius: 6px; padding: 8px 12px; font-size: 13px; font-weight: bold;
    min-height: 40px;
}
QPushButton:hover   { background-color: #585b70; }
QPushButton:pressed { background-color: #6c7086; }
QPushButton:disabled { background-color: #313244; color: #45475a; }
QScrollArea { border: none; background: transparent; }
QTabWidget::pane { border: 1px solid #313244; border-radius: 0 8px 8px 8px; }
QTabBar::tab {
    background: #181825; color: #6c7086; padding: 10px 20px;
    border-radius: 6px 6px 0 0; margin-right: 3px;
    font-weight: bold; font-size: 13px;
}
QTabBar::tab:selected        { background: #313244; color: #cdd6f4; }
QTabBar::tab:hover:!selected { background: #1e1e2e; color: #a6adc8; }
QLabel#videoLabel {
    background-color: #11111b; border: 2px solid #313244; border-radius: 10px;
}
QLabel#statusLabel { color: #6c7086; font-size: 12px; padding: 4px 8px; }
QLabel#sectionLabel { color: #a6adc8; font-size: 11px; font-weight: bold; padding: 2px 0px; }
QPushButton#stopBtn   { background-color: #f38ba8; color: #11111b; font-size: 14px; min-height: 48px; }
QPushButton#stopBtn:hover  { background-color: #eba0ac; }
QPushButton#fwdBtn    { background-color: #a6e3a1; color: #11111b; font-size: 14px; min-height: 48px; }
QPushButton#fwdBtn:hover   { background-color: #b4f0a8; }
QPushButton#revBtn    { background-color: #fab387; color: #11111b; font-size: 14px; min-height: 48px; }
QPushButton#revBtn:hover   { background-color: #fbc4a0; }
QPushButton#accentBtn { background-color: #89b4fa; color: #11111b; font-size: 14px; min-height: 44px; }
QPushButton#accentBtn:hover { background-color: #74c7ec; }
QPushButton#panoBtn   { background-color: #cba6f7; color: #11111b; font-size: 14px; border-radius: 8px; min-height: 44px; }
QPushButton#panoBtn:hover   { background-color: #b4befe; }
QPushButton#panoBtn:disabled { background-color: #45475a; color: #6c7086; }
QPushButton#camCapBtn {
    background-color: #89b4fa; color: #11111b; font-size: 12px;
    border-radius: 8px; min-height: 44px;
}
QPushButton#camCapBtn:hover { background-color: #74c7ec; }
QPushButton#utilBtn   { background: transparent; color: #6c7086; border: 1px solid #313244; border-radius: 6px; font-size: 12px; min-height: 40px; }
QPushButton#utilBtn:hover  { color: #cdd6f4; border-color: #45475a; }
QPushButton#saveBtn   { background-color: #94e2d5; color: #11111b; min-height: 40px; }
QPushButton#saveBtn:hover  { background-color: #a6f0e4; }
QPushButton#rpmBtn    { background-color: #585b70; color: #cdd6f4; font-size: 14px; min-height: 52px; }
QPushButton#rpmBtn:hover   { background-color: #6c7086; }
QPushButton#drillBigOn {
    background-color: #a6e3a1; color: #11111b; font-size: 20px; font-weight: bold;
    min-height: 72px; min-width: 180px; border-radius: 10px;
}
QPushButton#drillBigOn:hover { background-color: #b4f0a8; }
QPushButton#drillBigOff {
    background-color: #f38ba8; color: #11111b; font-size: 20px; font-weight: bold;
    min-height: 72px; min-width: 180px; border-radius: 10px;
}
QPushButton#drillBigOff:hover { background-color: #eba0ac; }
QPushButton#slotBtn   { background-color: #585b70; color: #cdd6f4; font-size: 14px; min-height: 52px; }
QPushButton#slotBtn:hover  { background-color: #6c7086; }
"""


def _register_capture_html(save_dir: str, image_path: str) -> None:
    try:
        register_capture(save_dir, image_path)
    except OSError:
        pass


class FrameBridge(QObject):
    merged_frame     = pyqtSignal(np.ndarray)
    pano_status      = pyqtSignal(str)
    pano_result      = pyqtSignal(np.ndarray)
    scilab_feedback  = pyqtSignal(str)
    npk_data         = pyqtSignal(str)
    spec_spectrum    = pyqtSignal(object)
    spec_wavelengths = pyqtSignal(object)
    spec_status      = pyqtSignal(object)


class MissionGuiNode(Node):
    def __init__(self, signals: FrameBridge) -> None:
        super().__init__('mission_gui')
        self.declare_parameter('save_dir',     os.path.expanduser('~/camera_captures'))
        self.declare_parameter('topic_merged', '/camera/merged/image_raw')
        self._cv      = CvBridge()
        self._signals = signals
        self._merged: np.ndarray | None = None
        merged_topic = self.get_parameter('topic_merged').get_parameter_value().string_value
        self.create_subscription(Image, merged_topic, self._on_merged, SENSOR_QOS)
        self._pano_cli   = self.create_client(Trigger, '/mission/panorama/trigger')
        self._scilab_pub = self.create_publisher(String, '/scilab/cmd', 10)
        self.create_subscription(String, '/scilab/feedback', self._on_scilab_fb, 10)
        self.create_subscription(String, '/npk/data', self._on_npk, 10)
        self._spec_cmd_pub = self.create_publisher(String, '/spectrometer/cmd', 10)
        self.create_subscription(Float32MultiArray, '/spectrometer/spectrum',    self._on_spec,    SENSOR_QOS)
        self.create_subscription(Float32MultiArray, '/spectrometer/wavelengths', self._on_spec_wl, 10)
        self.create_subscription(String,            '/spectrometer/status',      self._on_spec_st, 10)
        self.create_subscription(Image, '/panorama/result', self._on_pano_result, 1)

    def send_spec_cmd(self, payload: dict) -> None:
        msg = String(); msg.data = json.dumps(payload); self._spec_cmd_pub.publish(msg)

    def send_scilab_cmd(self, cmd: str) -> None:
        msg = String(); msg.data = cmd; self._scilab_pub.publish(msg)

    def call_panorama(self) -> None:
        if not self._pano_cli.wait_for_service(timeout_sec=3.0):
            self._signals.pano_status.emit(
                'mission_panorama 미연결 — 로버에서 rover.launch 실행·ROS_DOMAIN_ID 확인'
            )
            return
        self._signals.pano_status.emit('파노라마 촬영 시작...')
        self._pano_cli.call_async(Trigger.Request()).add_done_callback(self._pano_done)

    def _pano_done(self, future) -> None:
        try:
            res = future.result()
            self._signals.pano_status.emit(res.message if res.success else f'실패: {res.message}')
        except Exception as e:
            self._signals.pano_status.emit(f'오류: {e}')

    def _on_merged(self, msg: Image) -> None:
        frame = self._cv.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self._merged = frame; self._signals.merged_frame.emit(frame)

    def _on_pano_result(self, msg: Image) -> None:
        frame = self._cv.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._signals.pano_result.emit(frame)

    def _on_scilab_fb(self, msg: String) -> None: self._signals.scilab_feedback.emit(msg.data)
    def _on_npk(self,      msg: String) -> None: self._signals.npk_data.emit(msg.data)
    def _on_spec(self, msg: Float32MultiArray) -> None:
        self._signals.spec_spectrum.emit(np.asarray(msg.data, dtype=np.float32))
    def _on_spec_wl(self, msg: Float32MultiArray) -> None:
        self._signals.spec_wavelengths.emit(np.asarray(msg.data, dtype=np.float32))
    def _on_spec_st(self, msg: String) -> None:
        try: self._signals.spec_status.emit(json.loads(msg.data))
        except json.JSONDecodeError: pass


class StatusLabel(QLabel):
    def __init__(self, title: str, default: str = '-'):
        super().__init__(); self._title = title
        self.setText(f'{title}: {default}'); self._apply('#313244')

    def set_value(self, val: str, bg: str = '#313244') -> None:
        self.setText(f'{self._title}: {val}'); self._apply(bg)

    def _apply(self, bg: str) -> None:
        fg = '#11111b' if bg not in ('#313244', '#45475a') else '#cdd6f4'
        self.setStyleSheet(
            f'padding:4px 8px;border:1px solid #45475a;border-radius:4px;'
            f'background:{bg};color:{fg};font-size:12px;font-weight:bold;')


class NPKValueCard(QWidget):
    def __init__(self, title: str):
        super().__init__()
        self.setStyleSheet('NPKValueCard{background:#313244;border:1px solid #45475a;border-radius:8px;}')
        lay = QVBoxLayout(self); lay.setContentsMargins(10, 6, 10, 6)
        t = QLabel(title); t.setAlignment(Qt.AlignCenter)
        t.setStyleSheet('font-size:11px;color:#6c7086;font-weight:bold;')
        self._val = QLabel('---'); self._val.setAlignment(Qt.AlignCenter)
        self._val.setStyleSheet('font-size:20px;color:#cdd6f4;font-weight:bold;')
        lay.addWidget(t); lay.addWidget(self._val)

    def set_value(self, text: str) -> None: self._val.setText(text)



class SciLabTab(QWidget):
    def __init__(self, node: MissionGuiNode, signals: FrameBridge, parent=None):
        super().__init__(parent)
        self._node   = node
        self._latest: dict[str, str] = {}
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        content = QWidget(); scroll.setWidget(content)
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0); outer.addWidget(scroll)
        root = QVBoxLayout(content); root.setSpacing(10); root.setContentsMargins(10,10,10,10)
        root.addWidget(self._build_stepper_reset())
        cols = QHBoxLayout(); cols.setSpacing(10)
        lw = QWidget(); ll = QVBoxLayout(lw); ll.setSpacing(10)
        ll.addWidget(self._build_drill()); ll.addWidget(self._build_linear())
        ll.addStretch()
        rw = QWidget(); rl = QVBoxLayout(rw); rl.setSpacing(10)
        rl.addWidget(self._build_servo()); rl.addWidget(self._build_io()); rl.addStretch()
        cols.addWidget(lw, 1); cols.addWidget(rw, 1)
        root.addLayout(cols)
        signals.scilab_feedback.connect(self._on_feedback)

    def _cmd(self, c: str) -> None: self._node.send_scilab_cmd(c)

    def _build_stepper_reset(self) -> QGroupBox:
        box = QGroupBox('스텝모터 초기화'); lay = QHBoxLayout(box); lay.setSpacing(8)
        b_no = QPushButton('설정만 재적용\n(위치·영점 유지)')
        b_no.setObjectName('accentBtn')
        b_no.setToolTip(
            'MOTOR_INIT_ALL_NO_ZERO\n'
            '· 모든 스텝모터 운용 설정 재적용\n'
            '· 현재 위치 0점은 변경하지 않음\n'
            '· 필드 복구용 권장'
        )
        b_no.clicked.connect(lambda: self._cmd('MOTOR_INIT_ALL_NO_ZERO'))
        lay.addWidget(b_no)
        b_all = QPushButton('전체 초기화\n(현재 위치 → 영점)')  # MOTOR_INIT_ALL
        b_all.setObjectName('stopBtn')
        b_all.setToolTip(
            'MOTOR_INIT_ALL\n'
            '· 모든 스텝모터 운용 설정 재적용\n'
            '· 리니어/슬롯 현재 위치를 0점으로 설정\n'
            '· 기준 위치에 있을 때만 사용 권장'
        )
        b_all.clicked.connect(lambda: self._cmd('MOTOR_INIT_ALL'))
        lay.addWidget(b_all)
        return box

    def _linear_mm_go(self, mm: float) -> None:
        m = max(0.0, min(300.0, float(mm)))
        s = f'{m:.4f}'.rstrip('0').rstrip('.')
        self._cmd(f'LINEAR_MM:{s}')

    def _build_drill(self) -> QGroupBox:
        box = QGroupBox('드릴')
        lay = QVBoxLayout(box)
        lay.setSpacing(10)
        self._drill_st = StatusLabel('상태')
        self._drill_rpm_fb = StatusLabel('현재 RPM')
        for w in (self._drill_st, self._drill_rpm_fb):
            lay.addWidget(w)
        row = QHBoxLayout()
        row.setSpacing(16)
        bon = QPushButton('ON\n(DRILL_RUN)')
        bon.setObjectName('drillBigOn')
        bon.setToolTip('DRILL_RUN — 펌웨어 기본 800 rpm (필요 시 아래 RPM 먼저 설정)')
        bon.clicked.connect(lambda: self._cmd('DRILL_RUN'))
        boff = QPushButton('OFF\n(DRILL_STOP)')
        boff.setObjectName('drillBigOff')
        boff.setToolTip('DRILL_STOP')
        boff.clicked.connect(lambda: self._cmd('DRILL_STOP'))
        row.addWidget(bon)
        row.addWidget(boff)
        row.addStretch()
        lay.addLayout(row)
        rpm_row = QHBoxLayout()
        rpm_row.setSpacing(8)
        rpm_row.addWidget(QLabel('RPM 설정'))
        self._drill_rpm_spin = QSpinBox()
        self._drill_rpm_spin.setRange(0, 1000)
        self._drill_rpm_spin.setSingleStep(50)
        self._drill_rpm_spin.setValue(800)
        self._drill_rpm_spin.setToolTip('DRILL_RPM:0~1000 — 전송 후 ON이면 펌웨어에 따라 반영')
        rpm_row.addWidget(self._drill_rpm_spin)
        br = QPushButton('RPM 적용')
        br.setObjectName('accentBtn')
        br.setToolTip('DRILL_RPM:<값> 시리얼 전송')
        br.clicked.connect(self._drill_apply_rpm)
        rpm_row.addWidget(br)
        rpm_row.addStretch()
        lay.addLayout(rpm_row)
        return box

    def _drill_apply_rpm(self) -> None:
        self._cmd(f'DRILL_RPM:{int(self._drill_rpm_spin.value())}')

    def _build_linear(self) -> QGroupBox:
        box = QGroupBox('리니어 이동 (mm)')
        box.setToolTip('LINEAR_MM 0–300 mm. 원하면 언제든 덮어쓰기, LINEAR_STOP으로 정지')
        lay = QVBoxLayout(box); lay.setSpacing(8)
        self._m2_state = StatusLabel('상태'); self._m2_fb = StatusLabel('현재 위치')
        self._m2_cmd   = StatusLabel('목표');  self._m2_err = StatusLabel('오차')
        for w in (self._m2_state, self._m2_fb, self._m2_cmd, self._m2_err): lay.addWidget(w)
        inp = QHBoxLayout(); inp.setSpacing(6)
        inp.addWidget(QLabel('목표'))
        self._m2_input = QLineEdit(); self._m2_input.setPlaceholderText('0 ~ 300')
        self._m2_input.returnPressed.connect(self._move_direct)
        inp.addWidget(self._m2_input, 1)
        go = QPushButton('이동'); go.setObjectName('accentBtn'); go.setFixedWidth(70)
        go.clicked.connect(self._move_direct); inp.addWidget(go)
        lay.addLayout(inp)
        util = QHBoxLayout(); util.setSpacing(8)
        zu = QPushButton('영점'); zu.setToolTip('LINEAR_ZERO — 수동 영점 절차')
        zu.clicked.connect(lambda: self._cmd('LINEAR_ZERO'))
        st = QPushButton('정지'); st.setObjectName('stopBtn'); st.setToolTip('LINEAR_STOP')
        st.clicked.connect(lambda: self._cmd('LINEAR_STOP'))
        util.addWidget(zu); util.addWidget(st); util.addStretch()
        lay.addLayout(util)
        return box

    def _move_direct(self) -> None:
        try:
            val = float(self._m2_input.text().strip())
        except ValueError:
            return
        self._linear_mm_go(val)

    def _build_servo(self) -> QGroupBox:
        box = QGroupBox('서보 제어'); lay = QVBoxLayout(box); lay.setSpacing(8)
        self._canister_st = StatusLabel('캐니스터')
        self._lid_st   = StatusLabel('뚜껑')
        self._mixer_st = StatusLabel('믹서')
        for w in (self._canister_st, self._lid_st, self._mixer_st): lay.addWidget(w)
        lc = QLabel('캐니스터'); lc.setObjectName('sectionLabel'); lay.addWidget(lc)
        cg = QGridLayout(); cg.setSpacing(6)
        for i, (txt, ang) in enumerate([('표층 시료', 0), ('보관', 60), ('쓰레기통', 120), ('빈공간', 180)]):
            b = QPushButton(txt); b.setObjectName('accentBtn')
            b.clicked.connect(lambda _, v=ang: self._cmd(f'CANISTER_ANGLE:{v}'))
            cg.addWidget(b, i // 2, i % 2)
        lay.addLayout(cg)
        lbl2 = QLabel('뚜껑'); lbl2.setObjectName('sectionLabel'); lay.addWidget(lbl2)
        r = QHBoxLayout(); r.setSpacing(6)
        bo = QPushButton('열기'); bo.setObjectName('fwdBtn'); bo.clicked.connect(lambda: self._cmd('LID_OPEN'))
        bc = QPushButton('닫기'); bc.setObjectName('stopBtn'); bc.clicked.connect(lambda: self._cmd('LID_CLOSE'))
        r.addWidget(bo); r.addWidget(bc); lay.addLayout(r)
        lbl3 = QLabel('믹서'); lbl3.setObjectName('sectionLabel'); lay.addWidget(lbl3)
        r2 = QHBoxLayout(); r2.setSpacing(6)
        mon = QPushButton('ON');  mon.setObjectName('fwdBtn');  mon.clicked.connect(lambda: self._cmd('MIXER_ON'))
        mof = QPushButton('OFF'); mof.setObjectName('stopBtn'); mof.clicked.connect(lambda: self._cmd('MIXER_OFF'))
        r2.addWidget(mon); r2.addWidget(mof); lay.addLayout(r2)
        return box

    def _build_io(self) -> QGroupBox:
        box = QGroupBox('펌프'); lay = QVBoxLayout(box); lay.setSpacing(8)
        self._p1_st = StatusLabel('펌프1')
        self._p23_st   = StatusLabel('펌프2+3')
        for w in (self._p1_st, self._p23_st): lay.addWidget(w)
        for lbl_txt, attr, run_fn, stop in [('펌프 1','_p1_ms',self._pump1_run,'PUMP1_STOP'),
                                             ('펌프 2+3','_p23_ms',self._pump23_run,'PUMP23_STOP')]:
            ll = QLabel(lbl_txt); ll.setObjectName('sectionLabel'); lay.addWidget(ll)
            row = QHBoxLayout(); row.setSpacing(6)
            edit = QLineEdit('2000' if '1' in lbl_txt else '3000')
            edit.setFixedWidth(70); edit.setPlaceholderText('ms')
            setattr(self, attr, edit); row.addWidget(edit)
            bf = QPushButton('정회전'); bf.setObjectName('fwdBtn'); bf.clicked.connect(lambda _=None,f=run_fn: f('FWD'))
            br = QPushButton('역회전'); br.setObjectName('revBtn'); br.clicked.connect(lambda _=None,f=run_fn: f('REV'))
            bs = QPushButton('정지');   bs.setObjectName('stopBtn'); bs.clicked.connect(lambda _=None,c=stop: self._cmd(c))
            row.addWidget(bf); row.addWidget(br); row.addWidget(bs); lay.addLayout(row)
        return box

    def _pump1_run(self, d: str) -> None:
        ms = self._p1_ms.text().strip()
        if ms.isdigit() and int(ms) > 0: self._cmd(f'PUMP1_RUN:{d},{ms}')

    def _pump23_run(self, d: str) -> None:
        ms = self._p23_ms.text().strip()
        if ms.isdigit() and int(ms) > 0: self._cmd(f'PUMP23_RUN:{d},{ms}')

    def _on_feedback(self, line: str) -> None:
        s = parse_scilab_feedback(line)
        if s is None:
            return
        self._latest = s
        self._update_ui(s)

    def _state_bg(self, st: str) -> str:
        if st == 'IDLE': return CLR_IDLE
        if st in ('RUN','SPEED_UP','SPEED_DOWN','HOMING','CAL','RECOVER'): return CLR_BUSY
        if st == 'JAM': return CLR_JAM
        return '#313244'

    def _update_ui(self, s: dict) -> None:
        ds = _fb_get(s, 'DRILL_STATE', 'M1_STATE')
        self._drill_st.set_value(
            '작동' if ds == 'RUN' else '정지' if ds in ('STOP', 'IDLE') else ds,
            CLR_BUSY if ds == 'RUN' else CLR_IDLE if ds in ('STOP', 'IDLE') else '#313244',
        )
        drpm = _fb_get(s, 'DRILL_RPM', 'M1_RPM')
        if drpm == '-':
            self._drill_rpm_fb.set_value('-')
        else:
            try:
                self._drill_rpm_fb.set_value(f'{int(float(drpm))} rpm')
            except ValueError:
                self._drill_rpm_fb.set_value(f'{drpm} rpm')
        lin = _fb_get(s, 'LINEAR_STATE', 'M2_STATE')
        self._m2_state.set_value(lin, self._state_bg(lin))
        self._m2_fb.set_value(f"{_fb_get(s, 'LINEAR_FB_MM', 'M2_MM_FB')} mm")
        self._m2_cmd.set_value(f"{_fb_get(s, 'LINEAR_CMD_MM', 'LINEAR_MM_CMD', 'M2_MM_CMD')} mm")
        self._m2_err.set_value(f"{_fb_get(s, 'LINEAR_ERR_MM', 'M2_ERR_MM')} mm")
        self._canister_st.set_value(f"{_fb_get(s, 'CANISTER_ANGLE')} deg")
        lid = _fb_get(s, 'LID_ANGLE')
        self._lid_st.set_value('열림' if lid == '90' else '닫힘' if lid == '0' else lid)
        mx = _fb_get(s, 'MIXER_STATE')
        self._mixer_st.set_value(
            '작동' if mx == 'RUN' else '정지' if mx in ('STOP', 'IDLE') else mx,
            CLR_BUSY if mx == 'RUN' else CLR_IDLE if mx in ('STOP', 'IDLE') else '#313244',
        )
        p1s = _fb_get(s, 'PUMP1_STATE', 'P1_STATE')
        self._p1_st.set_value(
            f"{p1s}/{_fb_get(s, 'PUMP1_DIR', 'P1_DIR')}/{_fb_get(s, 'PUMP1_REMAIN_MS', 'P1_REMAIN_MS')}ms",
            CLR_BUSY if p1s == 'RUN' else CLR_IDLE if p1s in ('STOP', 'IDLE') else '#313244',
        )
        p23s = _fb_get(s, 'PUMP23_STATE', 'P23_STATE')
        self._p23_st.set_value(
            f"{p23s}/{_fb_get(s, 'PUMP23_DIR', 'P23_DIR')}/{_fb_get(s, 'PUMP23_REMAIN_MS', 'P23_REMAIN_MS')}ms",
            CLR_BUSY if p23s == 'RUN' else CLR_IDLE if p23s in ('STOP', 'IDLE') else '#313244',
        )



class NPKTab(QWidget):
    def __init__(self, signals: FrameBridge, save_dir: str, parent=None):
        super().__init__(parent)
        self._save_dir = save_dir
        self._npk_keys = ['Moist','Temp','EC','pH','N','P','K']
        self._npk_history: dict[str, deque] = {k: deque(maxlen=GRAPH_MAX_POINTS) for k in self._npk_keys}
        self._npk_time_labels: deque[str] = deque(maxlen=GRAPH_MAX_POINTS)
        self._npk_sample_time: deque      = deque(maxlen=GRAPH_MAX_POINTS)
        self._auto_csv_path: str | None   = None
        self._auto_csv_count: int         = 0
        root = QVBoxLayout(self); root.setSpacing(10); root.setContentsMargins(10,10,10,10)
        top = QHBoxLayout()
        self._npk_status = QLabel('NPK 데이터 대기중...'); self._npk_status.setObjectName('statusLabel')
        top.addWidget(self._npk_status, 1)
        sg = QPushButton('그래프 캡처'); sg.setObjectName('saveBtn'); sg.clicked.connect(self._save_graph)
        top.addWidget(sg); root.addLayout(top)
        cards = QGridLayout(); cards.setHorizontalSpacing(8); cards.setVerticalSpacing(8)
        self._npk_cards: dict[str, NPKValueCard] = {}
        labels = {'Moist':'습도 %','Temp':'온도 C','EC':'EC us/cm','pH':'pH','N':'질소 mg/kg','P':'인 mg/kg','K':'칼륨 mg/kg'}
        for (r,c), key in zip([(0,0),(0,1),(0,2),(0,3),(1,0),(1,1),(1,2)], self._npk_keys):
            card = NPKValueCard(labels[key]); self._npk_cards[key] = card; cards.addWidget(card,r,c)
        root.addLayout(cards)
        if HAS_PYQTGRAPH:
            pg.setConfigOptions(antialias=True)
            self._graph_w = QWidget(); gl = QGridLayout(self._graph_w); gl.setContentsMargins(0,0,0,0)
            self._build_graphs(gl); root.addWidget(self._graph_w)
        else:
            self._graph_w = None; root.addWidget(QLabel('pyqtgraph 미설치: 그래프 비활성'))
        signals.npk_data.connect(self._on_npk_data)

    def _mk_plot(self, title, ylabel, legend=False):
        pw = pg.PlotWidget(); pw.setBackground('#1e1e2e')
        pw.showGrid(x=True, y=True, alpha=0.15); pw.setLabel('bottom','시간','s')
        if ylabel: pw.setLabel('left', ylabel)
        if title:  pw.setTitle(title, color='#cdd6f4', size='10pt')
        if legend: pw.addLegend()
        pw.setXRange(-GRAPH_WINDOW_SEC, 0, padding=0); pw.getPlotItem().setMenuEnabled(False)
        return pw

    def _build_graphs(self, grid):
        self._plot_moist = self._mk_plot('습도','%'); self._plot_moist.setYRange(0,100,padding=0)
        self._c_moist = self._plot_moist.plot(pen=pg.mkPen((0,120,255),width=2)); grid.addWidget(self._plot_moist,0,0)
        self._plot_temp = self._mk_plot('온도','C'); self._plot_temp.setYRange(0,50,padding=0)
        self._c_temp = self._plot_temp.plot(pen=pg.mkPen((255,80,80),width=2)); grid.addWidget(self._plot_temp,0,1)
        self._plot_ph = self._mk_plot('pH','pH'); self._plot_ph.setYRange(3,9,padding=0)
        self._c_ph = self._plot_ph.plot(pen=pg.mkPen((0,170,0),width=2)); grid.addWidget(self._plot_ph,1,0)
        self._plot_ec = self._mk_plot('EC','us/cm')
        self._c_ec = self._plot_ec.plot(pen=pg.mkPen((180,140,0),width=2)); grid.addWidget(self._plot_ec,1,1)
        self._plot_npk = self._mk_plot('N / P / K','mg/kg',legend=True)
        self._c_n = self._plot_npk.plot(name='N',pen=pg.mkPen((220,60,60),width=2),symbol='o',symbolSize=5)
        self._c_p = self._plot_npk.plot(name='P',pen=pg.mkPen((60,180,60),width=2),symbol='o',symbolSize=5)
        self._c_k = self._plot_npk.plot(name='K',pen=pg.mkPen((60,100,255),width=2),symbol='o',symbolSize=5)
        grid.addWidget(self._plot_npk,2,0,1,2)

    def _init_auto_csv(self) -> None:
        d = os.path.join(self._save_dir, 'npk')
        os.makedirs(d, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._auto_csv_path = os.path.join(d, f'npk_{stamp}.csv')
        with open(self._auto_csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(['Time'] + self._npk_keys)

    def _on_npk_data(self, raw: str) -> None:
        try: d = json.loads(raw)
        except json.JSONDecodeError: return
        self._npk_cards['Moist'].set_value(f"{d['Moist']:.1f} %")
        self._npk_cards['Temp'].set_value(f"{d['Temp']:.1f} C")
        self._npk_cards['EC'].set_value(f"{d['EC']} us/cm")
        self._npk_cards['pH'].set_value(f"{d['pH']:.1f}")
        self._npk_cards['N'].set_value(f"{d['N']} mg/kg")
        self._npk_cards['P'].set_value(f"{d['P']} mg/kg")
        self._npk_cards['K'].set_value(f"{d['K']} mg/kg")
        t = 0 if not self._npk_sample_time else self._npk_sample_time[-1] + GRAPH_STEP_SEC
        self._npk_sample_time.append(t)
        ts = datetime.now().strftime('%H:%M:%S')
        self._npk_time_labels.append(ts)
        for k in self._npk_keys: self._npk_history[k].append(d[k])

        # 자동 저장
        if self._auto_csv_path is None:
            self._init_auto_csv()
        with open(self._auto_csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([ts] + [d[k] for k in self._npk_keys])
        self._auto_csv_count += 1
        self._npk_status.setText(
            f"습도={d['Moist']:.1f}%  온도={d['Temp']:.1f}C  pH={d['pH']:.1f}"
            f"  |  자동저장 {self._auto_csv_count}회: {os.path.basename(self._auto_csv_path)}")

        if HAS_PYQTGRAPH: self._refresh_graphs()

    def _refresh_graphs(self) -> None:
        if not self._npk_sample_time: return
        x = [t - self._npk_sample_time[-1] for t in self._npk_sample_time]
        self._c_moist.setData(x, list(self._npk_history['Moist']))
        self._c_temp.setData(x,  list(self._npk_history['Temp']))
        self._c_ph.setData(x,    list(self._npk_history['pH']))
        self._c_ec.setData(x,    list(self._npk_history['EC']))
        self._c_n.setData(x, list(self._npk_history['N']))
        self._c_p.setData(x, list(self._npk_history['P']))
        self._c_k.setData(x, list(self._npk_history['K']))
        for pw in (self._plot_moist,self._plot_temp,self._plot_ph,self._plot_ec,self._plot_npk):
            pw.setXRange(-GRAPH_WINDOW_SEC, 0, padding=0)
        ec = list(self._npk_history['EC'])
        if ec:
            lo,hi = min(ec),max(ec)
            if lo==hi: lo-=1; hi+=1
            m = max((hi-lo)*0.15,1.0); self._plot_ec.setYRange(lo-m,hi+m,padding=0)
        comb = list(self._npk_history['N'])+list(self._npk_history['P'])+list(self._npk_history['K'])
        if comb:
            lo,hi = min(comb),max(comb)
            if lo==hi: lo-=1; hi+=1
            m = max((hi-lo)*0.15,1.0); self._plot_npk.setYRange(lo-m,hi+m,padding=0)

    def _apply_npk_plot_theme(self, *, export: bool) -> None:
        plots = (
            (self._plot_moist, '습도', '%'),
            (self._plot_temp, '온도', 'C'),
            (self._plot_ph, 'pH', 'pH'),
            (self._plot_ec, 'EC', 'us/cm'),
            (self._plot_npk, 'N / P / K', 'mg/kg'),
        )
        if export:
            bg, fg, title_c, grid_a = '#ffffff', '#11111b', '#11111b', 0.25
            self._graph_w.setStyleSheet('background-color:#ffffff;')
        else:
            bg, fg, title_c, grid_a = '#1e1e2e', '#cdd6f4', '#cdd6f4', 0.15
            self._graph_w.setStyleSheet('background-color:transparent;')

        for pw, plot_title, ylabel in plots:
            pw.setBackground(bg)
            pw.setLabel('left', ylabel, color=fg)
            pw.setLabel('bottom', '시간', 's', color=fg)
            pw.setTitle(plot_title, color=title_c, size='10pt')
            pw.showGrid(x=True, y=True, alpha=grid_a)
            pi = pw.getPlotItem()
            for axis_name in ('left', 'bottom'):
                ax = pi.getAxis(axis_name)
                ax.setPen(pg.mkPen(fg))
                ax.setTextPen(pg.mkPen(fg))
            leg = pi.legend
            if leg is not None:
                for _sample, label in leg.items:
                    label.setText(label.text, color=fg)

    def _save_graph(self) -> None:
        if self._graph_w is None: return
        d = os.path.join(self._save_dir,'npk'); os.makedirs(d,exist_ok=True)
        path = os.path.join(d,f'npk_graph_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
        self._apply_npk_plot_theme(export=True)
        QApplication.processEvents()
        try:
            self._graph_w.grab().save(path)
        finally:
            self._apply_npk_plot_theme(export=False)
            QApplication.processEvents()
        _register_capture_html(self._save_dir, path)
        self._npk_status.setText(f'그래프 저장: {os.path.basename(path)}')



def _science_gui_share_calibration_dir() -> str | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('science_gui'), 'calibration')
        return d if os.path.isdir(d) else None
    except Exception:
        pass
    here = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calibration'))
    return here if os.path.isdir(here) else None


def _default_bca_gold_csv_path() -> str | None:
    base = _science_gui_share_calibration_dir()
    if base:
        p = os.path.join(base, 'bca_gold_laboratory_2026.csv')
        if os.path.isfile(p):
            return p
    return None


class SpectrometerTab(QWidget):
    def __init__(self, node: MissionGuiNode, signals: FrameBridge, save_dir: str, parent=None):
        super().__init__(parent)
        self._node=node; self._save_dir=save_dir; self._target_nm=480.0
        self._spec_wl=None; self._spec_data=None; self._spec_ref=None
        self._blank_abs: float | None = None
        self._cal_points: list[tuple[float,float]]=[]
        self._cal_slope=0.0; self._cal_intercept=0.0; self._cal_ready=False
        scroll=QScrollArea(); scroll.setWidgetResizable(True)
        content=QWidget(); scroll.setWidget(content)
        outer=QVBoxLayout(self); outer.setContentsMargins(0,0,0,0); outer.addWidget(scroll)
        root=QVBoxLayout(content); root.setSpacing(8); root.setContentsMargins(10,10,10,10)
        self._spec_status_lbl=QLabel('분광기 연결 대기...')
        self._spec_status_lbl.setObjectName('statusLabel'); self._spec_status_lbl.setWordWrap(True)
        root.addWidget(self._spec_status_lbl)
        wl_row=QHBoxLayout(); wl_row.addWidget(QLabel('측정 파장 (nm)'))
        self._target_nm_input=QSpinBox(); self._target_nm_input.setRange(200,1100); self._target_nm_input.setValue(480)
        self._target_nm_input.valueChanged.connect(self._on_target_nm_changed); wl_row.addWidget(self._target_nm_input)
        bf=QPushButton('피크 자동 검출'); bf.setObjectName('accentBtn'); bf.clicked.connect(self._spec_find_peak)
        wl_row.addWidget(bf); root.addLayout(wl_row)
        rr=QHBoxLayout()
        self._abs_lbl=QLabel('흡광도: ---')
        self._abs_lbl.setStyleSheet('font-size:16px;font-weight:bold;color:#89b4fa;background:#313244;padding:6px 12px;border-radius:6px;')
        self._protein_lbl=QLabel('단백질: ---')
        self._protein_lbl.setStyleSheet('font-size:16px;font-weight:bold;color:#a6e3a1;background:#313244;padding:6px 12px;border-radius:6px;')
        rr.addWidget(self._abs_lbl); rr.addWidget(self._protein_lbl); root.addLayout(rr)
        if HAS_PYQTGRAPH:
            self._spec_plot=pg.PlotWidget(background='#1e1e2e')
            self._spec_plot.showGrid(x=True,y=True,alpha=0.2)
            self._spec_plot.setLabel('bottom','파장','nm'); self._spec_plot.setLabel('left','강도','counts')
            self._spec_plot.setMinimumHeight(200); self._spec_plot.getPlotItem().setMenuEnabled(False)
            self._spec_curve=self._spec_plot.plot(pen=pg.mkPen('#89b4fa',width=2))
            self._spec_curve_ref=self._spec_plot.plot(pen=pg.mkPen('#f9e2af',width=1,style=Qt.DashLine))
            self._spec_vline=pg.InfiniteLine(pos=self._target_nm,angle=90,pen=pg.mkPen('#f38ba8',width=1,style=Qt.DashLine))
            self._spec_plot.addItem(self._spec_vline); root.addWidget(self._spec_plot)
        else:
            self._spec_plot=None
        mr=QHBoxLayout()
        self._abs_mode=QCheckBox('흡광도 모드'); self._abs_mode.toggled.connect(self._spec_toggle_abs)
        self._spec_auto_y=QCheckBox('Y축 자동 스케일'); self._spec_auto_y.setChecked(True)
        mr.addWidget(self._abs_mode); mr.addWidget(self._spec_auto_y); root.addLayout(mr)
        acq=QGridLayout(); acq.setHorizontalSpacing(8); acq.setVerticalSpacing(6)
        acq.addWidget(QLabel('적분시간 (us)'),0,0)
        self._spec_int=QSpinBox(); self._spec_int.setRange(1000,60_000_000); self._spec_int.setSingleStep(1000); self._spec_int.setValue(360000)
        acq.addWidget(self._spec_int,0,1)
        bi=QPushButton('적용'); bi.setObjectName('accentBtn'); bi.clicked.connect(lambda: self._spec_cmd({'cmd':'set_integration','us':self._spec_int.value()}))
        acq.addWidget(bi,0,2); acq.addWidget(QLabel('평균 횟수'),1,0)
        self._spec_avg=QSpinBox(); self._spec_avg.setRange(1,200); self._spec_avg.setValue(1)
        acq.addWidget(self._spec_avg,1,1)
        ba=QPushButton('적용'); ba.setObjectName('accentBtn'); ba.clicked.connect(lambda: self._spec_cmd({'cmd':'set_average','n':self._spec_avg.value()}))
        acq.addWidget(ba,1,2); root.addLayout(acq)
        ctrl=QHBoxLayout()
        bs=QPushButton('연속 시작'); bs.setObjectName('fwdBtn'); bs.clicked.connect(lambda: self._spec_cmd({'cmd':'set_streaming','enabled':True}))
        bx=QPushButton('정지');     bx.setObjectName('stopBtn'); bx.clicked.connect(lambda: self._spec_cmd({'cmd':'set_streaming','enabled':False}))
        bo=QPushButton('단발');     bo.setObjectName('accentBtn'); bo.clicked.connect(lambda: self._spec_cmd({'cmd':'single_shot'}))
        ctrl.addWidget(bs); ctrl.addWidget(bx); ctrl.addWidget(bo); root.addLayout(ctrl)
        corr=QHBoxLayout()
        bd=QPushButton('다크 저장'); bd.setObjectName('accentBtn'); bd.clicked.connect(lambda: self._spec_cmd({'cmd':'store_dark'}))
        bc=QPushButton('다크 해제'); bc.clicked.connect(lambda: self._spec_cmd({'cmd':'clear_dark'}))
        br=QPushButton('레퍼런스 저장'); br.setObjectName('saveBtn'); br.clicked.connect(self._spec_capture_ref)
        brc=QPushButton('레퍼런스 해제'); brc.clicked.connect(self._spec_clear_ref)
        corr.addWidget(bd); corr.addWidget(bc); corr.addWidget(br); corr.addWidget(brc); root.addLayout(corr)
        root.addWidget(self._build_lab_cuvette_slot())
        root.addWidget(self._build_spec_relay())
        bca_seq=QGroupBox('BCA 현장 시퀀스')
        bca_seq_l=QVBoxLayout(bca_seq); bca_seq_l.setSpacing(6)
        _sq=QLabel(
            '[표준곡선 사용 — 농도만 알 때] 연속 시작 → 실험실 표준곡선 불러오기(또는 자동 로드) → '
            '블랭크(0 µg/mL, 시약+용매)로 레퍼런스 저장 → CSV가 ΔAbs면 같은 블랭크로 Blank Abs 저장 → '
            '시료로 교체 후 단백질(µg/mL) 확인. (Abs 추가·표준액 여러 개 돌리기 불필요)\n'
            '[현장 곡선/보정] 표준 농도 시료마다 농도 입력 후「현재 Abs 추가」·필요 시「표준곡선 저장」\n'
            '공통: 480 nm(실험실 gold 기준) 또는「피크 자동 검출」·수치 어긋나면 Abs 추가로 재보정\n'
            '저장:「CSV 저장」= 파장·강도·레퍼·Abs 테이블 /「그래프 캡처」= 스펙 그림 /「표준곡선 저장」= 보정 곡선 CSV'
        )
        _sq.setWordWrap(True); _sq.setStyleSheet('font-size:12px;color:#bac2de;')
        bca_seq_l.addWidget(_sq)
        _ddef=QHBoxLayout()
        _btn_def=QPushButton('실험실 표준곡선 불러오기'); _btn_def.setObjectName('accentBtn')
        _btn_def.setToolTip('bca_gold_laboratory_2026.csv (패키지 calibration/)')
        _btn_def.clicked.connect(self._load_default_bca_gold)
        _ddef.addWidget(_btn_def); bca_seq_l.addLayout(_ddef)
        root.addWidget(bca_seq)
        drow=QHBoxLayout()
        self._delta_cb=QCheckBox('ΔAbs (0 µg/mL blank 차, 실험실 CSV와 동일)')
        self._delta_cb.toggled.connect(self._on_delta_toggled)
        _bb=QPushButton('Blank Abs 저장'); _bb.setObjectName('saveBtn'); _bb.clicked.connect(self._blank_save)
        _bxb=QPushButton('Blank 해제'); _bxb.clicked.connect(self._blank_clear)
        drow.addWidget(self._delta_cb); drow.addWidget(_bb); drow.addWidget(_bxb)
        root.addLayout(drow)
        cl=QLabel('BCA 표준곡선'); cl.setObjectName('sectionLabel'); root.addWidget(cl)
        self._cal_info=QLabel('표준곡선 미설정')
        self._cal_info.setStyleSheet('font-size:11px;color:#6c7086;'); self._cal_info.setWordWrap(True); root.addWidget(self._cal_info)
        ar=QHBoxLayout(); ar.addWidget(QLabel('농도 (ug/mL)'))
        self._cal_conc_input=QLineEdit(); self._cal_conc_input.setPlaceholderText('예: 25'); self._cal_conc_input.setFixedWidth(80)
        ar.addWidget(self._cal_conc_input)
        badd=QPushButton('현재 Abs 추가'); badd.setObjectName('accentBtn'); badd.clicked.connect(self._cal_add_point)
        bclr=QPushButton('초기화'); bclr.setObjectName('stopBtn'); bclr.clicked.connect(self._cal_clear)
        ar.addWidget(badd); ar.addWidget(bclr); root.addLayout(ar)
        cio=QHBoxLayout()
        bsc=QPushButton('표준곡선 저장'); bsc.setObjectName('saveBtn'); bsc.clicked.connect(self._cal_save)
        blc=QPushButton('표준곡선 불러오기'); blc.setObjectName('accentBtn'); blc.clicked.connect(self._cal_load)
        cio.addWidget(bsc); cio.addWidget(blc); root.addLayout(cio)
        sr=QHBoxLayout()
        bcsv=QPushButton('CSV 저장'); bcsv.setObjectName('saveBtn'); bcsv.clicked.connect(self._spec_save_csv)
        bpng=QPushButton('그래프 캡처'); bpng.setObjectName('saveBtn'); bpng.clicked.connect(self._spec_save_png)
        sr.addWidget(bcsv); sr.addWidget(bpng); root.addLayout(sr)
        signals.spec_spectrum.connect(self._on_spec_spectrum)
        signals.spec_wavelengths.connect(self._on_spec_wavelengths)
        signals.spec_status.connect(self._on_spec_status)
        signals.scilab_feedback.connect(self._on_scilab_feedback_spec_lab)
        self._try_autoload_default_bca()

    def _scilab_cmd(self, c: str) -> None:
        self._node.send_scilab_cmd(c)

    def _build_lab_cuvette_slot(self) -> QGroupBox:
        box = QGroupBox('실험실 큐벳 — 슬롯 (모터3)'); lay = QVBoxLayout(box); lay.setSpacing(8)
        self._m3_state = StatusLabel('상태')
        self._m3_slot = StatusLabel('슬롯')
        self._m3_err = StatusLabel('오차')
        for w in (self._m3_state, self._m3_slot, self._m3_err):
            lay.addWidget(w)
        g = QGridLayout()
        g.setSpacing(6)
        for i, idx in enumerate(SLOT_INDEX_VALUES):
            b = QPushButton(f'슬롯  {idx}')
            b.setObjectName('slotBtn')
            b.clicked.connect(lambda _, v=idx: self._scilab_cmd(f'SLOT_INDEX:{v}'))
            g.addWidget(b, i // 3, i % 3)
        lay.addLayout(g)
        zero = QPushButton('슬롯 영점')
        zero.clicked.connect(lambda: self._scilab_cmd('SLOT_ZERO'))
        lay.addWidget(zero)
        return box

    def _build_spec_relay(self) -> QGroupBox:
        box = QGroupBox('릴레이 (실험 보조)')
        lay = QVBoxLayout(box)
        lay.setSpacing(8)
        self._relay_st = StatusLabel('릴레이 상태')
        lay.addWidget(self._relay_st)
        row = QHBoxLayout()
        row.setSpacing(8)
        ron = QPushButton('릴레이 ON'); ron.setObjectName('fwdBtn')
        ron.setToolTip('RELAY_ON')
        ron.clicked.connect(lambda: self._scilab_cmd('RELAY_ON'))
        rof = QPushButton('릴레이 OFF'); rof.setObjectName('stopBtn')
        rof.setToolTip('RELAY_OFF')
        rof.clicked.connect(lambda: self._scilab_cmd('RELAY_OFF'))
        row.addWidget(ron)
        row.addWidget(rof)
        lay.addLayout(row)
        return box

    def _on_scilab_feedback_spec_lab(self, line: str) -> None:
        s = parse_scilab_feedback(line)
        if s is None:
            return
        m3 = _fb_get(s, 'SLOT_STATE', 'M3_STATE')

        def _bg(st: str) -> str:
            if st == 'IDLE':
                return CLR_IDLE
            if st in ('RUN', 'SPEED_UP', 'SPEED_DOWN', 'HOMING', 'CAL', 'RECOVER'):
                return CLR_BUSY
            if st == 'JAM':
                return CLR_JAM
            return '#313244'

        self._m3_state.set_value(m3, _bg(m3))
        self._m3_slot.set_value(_fb_get(s, 'SLOT_CMD', 'SLOT_INDEX', 'M3_SLOT_CMD'))
        self._m3_err.set_value(_fb_get(s, 'SLOT_ERR', 'SLOT_ERR_ENC', 'M3_ERR_ENC'))
        rl = _fb_get(s, 'RELAY')
        self._relay_st.set_value(rl, CLR_BUSY if rl == 'ON' else CLR_IDLE if rl == 'OFF' else '#313244')

    def _parse_cal_csv(self, path: str) -> tuple[list[tuple[float, float]], float | None, bool]:
        pts: list[tuple[float, float]] = []
        nm: float | None = None
        mode_delta = False
        with open(path, newline='') as f:
            for row in csv.reader(f):
                if not row or not str(row[0]).strip():
                    continue
                s0 = str(row[0]).strip()
                if s0.startswith('#'):
                    low = s0.lower()
                    if 'target_nm' in low:
                        try:
                            tail = s0.lower().split('target_nm', 1)[1].lstrip('= \t')
                            nm = float(tail.split()[0])
                        except (IndexError, ValueError):
                            pass
                    if 'bca_abs_mode' in low and 'delta' in low:
                        mode_delta = True
                    continue
                if s0.lower().startswith('absorbance'):
                    continue
                if len(row) >= 2:
                    try:
                        pts.append((float(row[0]), float(row[1])))
                    except ValueError:
                        continue
        return pts, nm, mode_delta

    def _apply_loaded_cal(self, pts: list[tuple[float, float]], nm: float | None, mode_delta: bool, info_tail: str) -> None:
        if not pts:
            self._cal_info.setText('유효한 데이터 없음' + info_tail)
            return
        self._cal_points = pts
        if nm is not None:
            self._target_nm = float(nm)
            self._target_nm_input.setValue(int(round(nm)))
        self._delta_cb.blockSignals(True)
        self._delta_cb.setChecked(mode_delta)
        self._delta_cb.blockSignals(False)
        self._cal_fit()
        self._cal_info.setText(self._cal_info.text() + info_tail)

    def _load_cal_path(self, path: str, info_tail: str = '') -> None:
        pts, nm, mode_delta = self._parse_cal_csv(path)
        self._apply_loaded_cal(pts, nm, mode_delta, info_tail)

    def _load_default_bca_gold(self) -> None:
        p = _default_bca_gold_csv_path()
        if not p:
            self._cal_info.setText('bca_gold_laboratory_2026.csv 없음 (colcon install 후 share/science_gui/calibration 확인)')
            return
        self._load_cal_path(p, f'  |  {os.path.basename(p)}')

    def _try_autoload_default_bca(self) -> None:
        p = _default_bca_gold_csv_path()
        if not p:
            return
        pts, nm, mode_delta = self._parse_cal_csv(p)
        if not pts:
            return
        self._apply_loaded_cal(pts, nm, mode_delta, f'  |  자동: {os.path.basename(p)}')

    def _bca_raw_and_effective(self) -> tuple[float | None, float | None]:
        raw = self._get_abs_at_nm(self._target_nm)
        if raw is None:
            return None, None
        if not self._delta_cb.isChecked():
            return raw, raw
        if self._blank_abs is None:
            return raw, None
        return raw, raw - self._blank_abs

    def _refresh_bca_display(self) -> None:
        if self._spec_data is None or self._spec_wl is None or len(self._spec_wl) != len(self._spec_data):
            return
        raw, eff = self._bca_raw_and_effective()
        if raw is None:
            self._abs_lbl.setText(f'Abs@{int(self._target_nm)}nm: (레퍼런스 필요)')
            self._protein_lbl.setText('단백질: ---')
            return
        if self._delta_cb.isChecked():
            self._abs_lbl.setText(
                f'Abs raw={raw:.4f}  |  ΔAbs={"저장 필요" if eff is None else f"{eff:.4f}"}'
            )
        else:
            self._abs_lbl.setText(f'Abs@{int(self._target_nm)}nm: {raw:.4f}')
        conc_use = eff if self._delta_cb.isChecked() else raw
        if self._cal_ready:
            if conc_use is None:
                self._protein_lbl.setText('단백질: Blank(Δ) 저장')
            else:
                self._protein_lbl.setText(
                    f'단백질: {max(0.0, self._cal_slope * conc_use + self._cal_intercept):.1f} ug/mL'
                )
        else:
            self._protein_lbl.setText('단백질: 표준곡선 필요')

    def _on_delta_toggled(self, _checked: bool) -> None:
        self._refresh_bca_display()

    def _blank_save(self) -> None:
        raw = self._get_abs_at_nm(self._target_nm)
        if raw is None:
            self._cal_info.setText('Blank 저장: 레퍼런스·스펙트럼 필요')
            return
        self._blank_abs = float(raw)
        self._delta_cb.setChecked(True)
        self._refresh_bca_display()

    def _blank_clear(self) -> None:
        self._blank_abs = None
        self._refresh_bca_display()

    def _spec_cmd(self, p): self._node.send_spec_cmd(p)
    def _on_target_nm_changed(self, v):
        self._target_nm=float(v)
        if HAS_PYQTGRAPH and self._spec_plot: self._spec_vline.setValue(self._target_nm)
        self._refresh_bca_display()
    def _spec_find_peak(self):
        af=self._calc_absorbance_full()
        if af is None or self._spec_wl is None: self._spec_status_lbl.setText('피크 검출: 레퍼런스 저장 후 시도'); return
        valid=(self._spec_wl>=300)&(self._spec_wl<=900)
        if not np.any(valid): return
        idx=int(np.argmax(np.where(valid,af,-np.inf)))
        self._target_nm=float(self._spec_wl[idx]); self._target_nm_input.setValue(int(round(self._target_nm)))
        self._spec_status_lbl.setText(f'피크: {self._target_nm:.1f} nm (Abs={af[idx]:.4f})')
    def _spec_toggle_abs(self, c):
        if HAS_PYQTGRAPH and self._spec_plot:
            self._spec_plot.setLabel('left','흡광도' if c else '강도','AU' if c else 'counts')
    def _get_abs_at_nm(self, nm):
        if self._spec_wl is None or self._spec_data is None or self._spec_ref is None or len(self._spec_ref)!=len(self._spec_wl): return None
        idx=int(np.argmin(np.abs(self._spec_wl-nm))); s=float(self._spec_data[idx]); r=float(self._spec_ref[idx])
        if r<=0 or s<=0: return None
        return -np.log10(s/r)
    def _calc_absorbance_full(self):
        if self._spec_data is None or self._spec_ref is None or len(self._spec_ref)!=len(self._spec_data): return None
        with np.errstate(divide='ignore',invalid='ignore'):
            return -np.log10(np.clip(self._spec_data/self._spec_ref,1e-10,None))
    def _on_spec_spectrum(self, arr):
        self._spec_data=arr
        if self._spec_wl is None or len(self._spec_wl)!=len(arr): return
        self._refresh_bca_display()
        if HAS_PYQTGRAPH and self._spec_plot:
            if self._abs_mode.isChecked():
                af=self._calc_absorbance_full()
                if af is not None: self._spec_curve.setData(self._spec_wl,af); self._spec_curve_ref.setData([],[])
                else: self._spec_curve.setData(self._spec_wl,arr)
            else:
                self._spec_curve.setData(self._spec_wl,arr)
                if self._spec_ref is not None and len(self._spec_ref)==len(self._spec_wl):
                    self._spec_curve_ref.setData(self._spec_wl,self._spec_ref)
            if self._spec_auto_y.isChecked(): self._spec_plot.enableAutoRange(axis='y',enable=True)
    def _on_spec_wavelengths(self, arr): self._spec_wl=arr
    def _on_spec_status(self, st):
        self._spec_status_lbl.setText(
            f"모델:{st.get('model','-')} SN:{st.get('serial','-')}\n"
            f"적분:{st.get('integration_us','-')}us 평균:{st.get('average','-')}\n"
            f"스트리밍:{st.get('streaming','-')} 다크:{st.get('has_dark','-')} 픽셀:{st.get('pixels','-')}")
    def _spec_capture_ref(self):
        if self._spec_data is not None: self._spec_ref=self._spec_data.copy()
    def _spec_clear_ref(self):
        self._spec_ref=None
        if HAS_PYQTGRAPH and self._spec_plot: self._spec_curve_ref.setData([],[])
    def _cal_add_point(self):
        try: conc=float(self._cal_conc_input.text().strip())
        except ValueError: return
        raw, eff = self._bca_raw_and_effective()
        use = eff if self._delta_cb.isChecked() else raw
        if raw is None:
            self._cal_info.setText('레퍼런스 저장 후 시도')
            return
        if self._delta_cb.isChecked() and eff is None:
            self._cal_info.setText('ΔAbs: Blank Abs 먼저 저장')
            return
        if use is None:
            return
        self._cal_points.append((use, conc)); self._cal_conc_input.clear(); self._cal_fit()
    def _cal_clear(self):
        self._cal_points.clear(); self._cal_ready=False; self._cal_slope=0.0; self._cal_intercept=0.0
        self._cal_info.setText('초기화됨')
    def _cal_fit(self):
        n=len(self._cal_points)
        if n<2: self._cal_ready=False; self._cal_info.setText(f'포인트 {n}개 (최소 2개 필요)'); return
        x=np.array([p[0] for p in self._cal_points]); y=np.array([p[1] for p in self._cal_points])
        c=np.polyfit(x,y,1); self._cal_slope=float(c[0]); self._cal_intercept=float(c[1]); self._cal_ready=True
        r2=1.0
        if n>2:
            yp=np.polyval(c,x); ss_res=np.sum((y-yp)**2); ss_tot=np.sum((y-np.mean(y))**2)
            r2=1.0-ss_res/ss_tot if ss_tot>0 else 0.0
        self._cal_info.setText(f'y={self._cal_slope:.1f}x+{self._cal_intercept:.2f}  R²={r2:.4f}  ({n}개)')
    def _cal_save(self):
        if not self._cal_points: return
        d=os.path.join(self._save_dir,'spectrometer'); os.makedirs(d,exist_ok=True)
        path=os.path.join(d,f'bca_cal_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        with open(path,'w',newline='') as f:
            w=csv.writer(f)
            w.writerow([f'# target_nm={self._target_nm}'])
            if self._delta_cb.isChecked():
                w.writerow(['# bca_abs_mode=delta'])
            w.writerow(['absorbance','concentration_ug_ml'])
            for a,c in self._cal_points: w.writerow([a,c])
        self._cal_info.setText(f'저장: {os.path.basename(path)}')
    def _cal_load(self):
        path,_=QFileDialog.getOpenFileName(self,'BCA CSV',self._save_dir,'CSV (*.csv)')
        if not path: return
        self._load_cal_path(path, f'  |  {os.path.basename(path)}')
    def _spec_save_csv(self):
        if self._spec_wl is None or self._spec_data is None: return
        d=os.path.join(self._save_dir,'spectrometer'); os.makedirs(d,exist_ok=True)
        path=os.path.join(d,f'spectrum_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        af=self._calc_absorbance_full()
        with open(path,'w',newline='') as f:
            w=csv.writer(f); w.writerow(['wavelength_nm','intensity','reference','absorbance'])
            for i,(wl,sp) in enumerate(zip(self._spec_wl,self._spec_data)):
                rv=float(self._spec_ref[i]) if self._spec_ref is not None and len(self._spec_ref)==len(self._spec_wl) else ''
                av=float(af[i]) if af is not None else ''
                w.writerow([float(wl),float(sp),rv,av])
        self._spec_status_lbl.setText(f'CSV: {os.path.basename(path)}')
    def _spec_save_png(self):
        if not HAS_PYQTGRAPH or self._spec_plot is None: return
        d=os.path.join(self._save_dir,'spectrometer'); os.makedirs(d,exist_ok=True)
        path=os.path.join(d,f'spectrum_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
        self._spec_plot.grab().save(path)
        _register_capture_html(self._save_dir, path)
        self._spec_status_lbl.setText(f'PNG: {os.path.basename(path)}')


class MainWindow(QMainWindow):
    def __init__(self, node: MissionGuiNode, signals: FrameBridge) -> None:
        super().__init__()
        save_dir = node.get_parameter('save_dir').get_parameter_value().string_value
        self._node = node
        self._save_dir = os.path.expanduser(save_dir)
        self.setWindowTitle('미션 제어 패널'); self.setMinimumSize(1440,800)
        central=QWidget(); self.setCentralWidget(central)
        root=QHBoxLayout(central); root.setContentsMargins(8,8,8,8); root.setSpacing(8)
        left=QVBoxLayout(); left.setSpacing(6)
        self._video_label=QLabel(); self._video_label.setObjectName('videoLabel')
        self._video_label.setAlignment(Qt.AlignCenter)
        self._video_label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self._video_label.setMinimumSize(600,340); left.addWidget(self._video_label)
        bar=QHBoxLayout(); bar.setSpacing(8)
        self._status=QLabel('영상 대기중...'); self._status.setObjectName('statusLabel')
        bar.addWidget(self._status,1)
        for region_id, label, _row, _col in STREAM_REGIONS:
            btn = QPushButton(label)
            btn.setObjectName('camCapBtn')
            btn.setFixedSize(72, 44)
            btn.setToolTip(f'{label} 영역 캡처 ({region_id})')
            btn.clicked.connect(lambda _checked=False, rid=region_id: self._capture_region(rid))
            bar.addWidget(btn)
        btn_all = QPushButton('전체')
        btn_all.setObjectName('accentBtn')
        btn_all.setFixedSize(72, 44)
        btn_all.setToolTip('합성 영상 전체 캡처 (상단 토양·캐시 + 하단 파노)')
        btn_all.clicked.connect(self._capture_all)
        bar.addWidget(btn_all)
        self._pano_btn=QPushButton('파노라마'); self._pano_btn.setObjectName('panoBtn'); self._pano_btn.setFixedSize(100,44)
        self._pano_btn.clicked.connect(self._trigger_panorama); bar.addWidget(self._pano_btn)
        dir_btn=QPushButton('...'); dir_btn.setObjectName('utilBtn'); dir_btn.setFixedSize(44,44)
        dir_btn.setToolTip('저장 폴더 변경'); dir_btn.clicked.connect(self._pick_dir); bar.addWidget(dir_btn)
        left.addLayout(bar); root.addLayout(left,3)
        tabs=QTabWidget()
        tabs.addTab(SciLabTab(node,signals),                    '사이언스랩')
        tabs.addTab(NPKTab(signals,self._save_dir),             'NPK 센서')
        tabs.addTab(SpectrometerTab(node,signals,self._save_dir),'분광기')
        root.addWidget(tabs,2)
        self._toast=QLabel(self); self._toast.setAlignment(Qt.AlignCenter)
        self._toast.setStyleSheet('background-color:#a6e3a1;color:#11111b;border-radius:8px;padding:8px 20px;font-size:13px;font-weight:600;')
        self._toast.hide()
        self._toast_timer=QTimer(self); self._toast_timer.setSingleShot(True); self._toast_timer.timeout.connect(self._toast.hide)
        signals.merged_frame.connect(self._update_video)
        signals.pano_status.connect(self._on_pano_status)
        signals.pano_result.connect(self._save_pano_result)

    def _update_video(self, frame: np.ndarray) -> None:
        h,w,ch=frame.shape; qimg=QImage(frame.data,w,h,w*ch,QImage.Format_RGB888)
        lbl=self._video_label
        lbl.setPixmap(QPixmap.fromImage(qimg).scaled(lbl.width(),lbl.height(),Qt.KeepAspectRatio,Qt.FastTransformation))
        self._status.setText(f'영상: {w}x{h}')

    def _capture_region(self, region_id: str) -> None:
        merged = self._node._merged
        if merged is None:
            self._show_toast('영상 없음')
            return
        region = next(
            ((rid, lbl, row, col) for rid, lbl, row, col in STREAM_REGIONS if rid == region_id),
            None,
        )
        if region is None:
            return
        rid, label, row, col = region
        frame = _crop_stream_region(merged, row, col)
        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(self._save_dir, f'{rid}_{ts}.png')
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if rid == 'soil':
            _draw_soil_scale_bar(frame_bgr)
        cv2.imwrite(path, frame_bgr)
        _register_capture_html(self._save_dir, path)
        self._show_toast(f'{label} 저장: {os.path.basename(path)}')

    def _capture_all(self) -> None:
        merged = self._node._merged
        if merged is None:
            self._show_toast('영상 없음')
            return
        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(self._save_dir, f'merged_{ts}.png')
        cv2.imwrite(path, cv2.cvtColor(merged, cv2.COLOR_RGB2BGR))
        _register_capture_html(self._save_dir, path)
        self._show_toast(f'전체 저장: {os.path.basename(path)}')

    def _trigger_panorama(self) -> None:
        self._pano_btn.setEnabled(False); self._pano_btn.setText('촬영중...'); self._node.call_panorama()

    def _save_pano_result(self, frame: np.ndarray) -> None:
        d = os.path.join(self._save_dir, 'panorama')
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f'panorama_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
        cv2.imwrite(path, frame)
        _register_capture_html(self._save_dir, path)
        self._show_toast(f'파노라마 저장: {os.path.basename(path)}')

    def _on_pano_status(self, text: str) -> None:
        self._pano_btn.setEnabled(True); self._pano_btn.setText('파노라마'); self._show_toast(text)

    def _pick_dir(self) -> None:
        d=QFileDialog.getExistingDirectory(self,'저장 폴더 선택',self._save_dir)
        if d: self._save_dir=d

    def _show_toast(self, text: str) -> None:
        self._toast.setText(text); self._toast.adjustSize()
        self._toast.move((self.width()-self._toast.width())//2, self.height()-self._toast.height()-24)
        self._toast.show(); self._toast_timer.start(2500)


def main(args=None) -> None:
    rclpy.init(args=args)
    app=QApplication(sys.argv); app.setStyleSheet(STYLE)
    signals=FrameBridge(); node=MissionGuiNode(signals)
    threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    win=MainWindow(node,signals); win.show()
    ret=app.exec_()
    node.destroy_node(); rclpy.shutdown(); sys.exit(ret)


if __name__ == '__main__':
    main()
