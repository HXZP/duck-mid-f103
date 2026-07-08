#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@brief Duck Mid PCAN 云台调试上位机。
@note 该工具只依赖 Python 标准库和 PEAK PCAN-Basic 驱动。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox
from tkinter import ttk


CHART_HISTORY_SECONDS = 10.0
CHART_MIN_WINDOW_SECONDS = 0.2
CHART_MAX_WINDOW_SECONDS = 120.0
CHART_REFRESH_MS = 100
CHART_MAX_POINTS = 70000
CHART_MAX_DRAW_POINTS = 1600
EVENT_QUEUE_MAX = 5000
EVENT_PROCESS_LIMIT = 300
LOG_MAX_LINES = 300

PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020
PCAN_MESSAGE_STANDARD = 0x00

PCAN_GIMBAL_REQUEST_ID = 0x300
PCAN_GIMBAL_RESPONSE_ID = 0x301
PCAN_MOTOR_REPORT_BASE_ID = 0x200

GIMBAL_CMD_DISABLE = 0x02
GIMBAL_CMD_SET_AXIS_TARGET = 0x01
GIMBAL_CMD_SET_PID_FIELD = 0x10
GIMBAL_CMD_GET_PID_FIELD = 0x11

GIMBAL_STATUS_TEXT = {
    0: "OK",
    1: "INVALID_CMD",
    2: "INVALID_PARAM",
    3: "EXEC_FAILED",
}

PCAN_CHANNELS = {
    "PCAN_USBBUS1": 0x51,
    "PCAN_USBBUS2": 0x52,
    "PCAN_USBBUS3": 0x53,
    "PCAN_USBBUS4": 0x54,
    "PCAN_USBBUS5": 0x55,
    "PCAN_USBBUS6": 0x56,
    "PCAN_USBBUS7": 0x57,
    "PCAN_USBBUS8": 0x58,
    "PCAN_USBBUS9": 0x509,
    "PCAN_USBBUS10": 0x50A,
    "PCAN_USBBUS11": 0x50B,
    "PCAN_USBBUS12": 0x50C,
    "PCAN_USBBUS13": 0x50D,
    "PCAN_USBBUS14": 0x50E,
    "PCAN_USBBUS15": 0x50F,
    "PCAN_USBBUS16": 0x510,
}

PCAN_DEFAULT_CHANNEL_NAME = "PCAN_USBBUS1"

PCAN_CHANNEL_LABELS = [
    f"{channel_name} (0x{channel_value:X})"
    for channel_name, channel_value in PCAN_CHANNELS.items()
]

PCAN_BITRATES = {
    "1M": 0x0014,
    "800K": 0x0016,
    "500K": 0x001C,
    "250K": 0x011C,
    "125K": 0x031C,
}

MOTOR_NODE_NAMES = {
    0x01: "pitch",
    0x02: "yaw",
    0x03: "roll",
}

PID_AXIS_IDS = {
    "roll": 0,
    "pitch": 1,
    "yaw": 2,
}

PID_LOOP_IDS = {
    "position": 0,
    "speed": 1,
}

PID_FIELD_IDS = {
    "kp": 0,
    "ki": 1,
    "kd": 2,
    "integral_limit_percent": 3,
    "output_limit_percent": 4,
}

PID_FIELD_ORDER = [
    "kp",
    "ki",
    "kd",
    "integral_limit_percent",
    "output_limit_percent",
]


class TPCANMsg(ctypes.Structure):
    """@brief PCAN-Basic CAN 报文结构。"""

    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("MSGTYPE", ctypes.c_ubyte),
        ("LEN", ctypes.c_ubyte),
        ("DATA", ctypes.c_ubyte * 8),
    ]


class TPCANTimestamp(ctypes.Structure):
    """@brief PCAN-Basic 时间戳结构。"""

    _fields_ = [
        ("millis", ctypes.c_uint32),
        ("millis_overflow", ctypes.c_uint16),
        ("micros", ctypes.c_uint16),
    ]


@dataclass
class MotorState:
    """@brief 单个电机的接收状态。"""

    name: str
    node_id: int
    position_mrad: int | None = None
    speed_mrad_s: int | None = None
    last_update_ms: int | None = None
    rx_count: int = 0


@dataclass
class MotorReportEvent:
    """@brief 电机主动上报事件。"""

    node_id: int
    position_mrad: int
    speed_mrad_s: int
    timestamp_ms: int


@dataclass
class PidResponseEvent:
    """@brief 云台 PID 调试响应事件。"""

    command: int
    status: int
    axis_id: int
    selector: int
    value: int


@dataclass
class ErrorEvent:
    """@brief 上位机错误事件。"""

    message: str


def now_monotonic_ms() -> int:
    """
    @brief 获取当前单调时间。
    @return int 当前单调时间，单位毫秒。
    """
    return int(time.monotonic() * 1000)


def int32_from_le(data: bytes) -> int:
    """
    @brief 从小端字节解析 int32。
    @param data 输入字节。
    @return int 解析出的有符号 32 位整数。
    """
    return int.from_bytes(data, byteorder="little", signed=True)


def int32_to_le(value: int) -> bytes:
    """
    @brief 将整数编码为小端 int32。
    @param value 输入整数。
    @return bytes 小端 int32 字节。
    """
    return int(value).to_bytes(4, byteorder="little", signed=True)


def mrad_to_degree(value_mrad: int | None) -> str:
    """
    @brief 将 mrad 格式化为角度字符串。
    @param value_mrad 输入角度，单位 mrad。
    @return str 角度字符串。
    """
    if value_mrad is None:
        return "--"

    return f"{(value_mrad / 1000.0) * 180.0 / math.pi:.2f}"


def format_optional_int(value: int | None) -> str:
    """
    @brief 格式化可选整数。
    @param value 输入整数。
    @return str 格式化结果。
    """
    if value is None:
        return "--"

    return str(value)


def parse_int_auto(text: str) -> int:
    """
    @brief 解析十进制或 0x 前缀十六进制整数。
    @param text 输入文本。
    @return int 解析结果。
    """
    return int(text.strip(), 0)


def channel_label_from_value(channel_value: int) -> str:
    """
    @brief 根据 PCAN 通道值生成下拉框显示文本。
    @param channel_value PCAN 通道值。
    @return str 下拉框显示文本。
    """
    for channel_name, item_value in PCAN_CHANNELS.items():
        if item_value == channel_value:
            return f"{channel_name} (0x{item_value:X})"

    return f"0x{channel_value:X}"


def channel_value_from_label(channel_label: str) -> int:
    """
    @brief 从下拉框显示文本解析 PCAN 通道值。
    @param channel_label 下拉框显示文本。
    @return int PCAN 通道值。
    """
    stripped_text = channel_label.strip()

    for channel_name, channel_value in PCAN_CHANNELS.items():
        if stripped_text.startswith(channel_name):
            return channel_value

    return parse_int_auto(stripped_text)


def make_pid_selector(loop_name: str, field_name: str) -> int:
    """
    @brief 生成 mid 云台 PID 调试选择字节。
    @param loop_name PID 环名称。
    @param field_name PID 字段名称。
    @return int 选择字节。
    """
    loop = PID_LOOP_IDS[loop_name]
    field = PID_FIELD_IDS[field_name]

    return (loop << 4) | field


def find_key_by_value(items: dict[str, int], value: int) -> str:
    """
    @brief 根据字典值查找字典键。
    @param items 字典对象。
    @param value 目标值。
    @return str 找到的键，未找到时返回数值字符串。
    """
    for key, item_value in items.items():
        if item_value == value:
            return key

    return str(value)


class PcanBasic:
    """@brief PCAN-Basic DLL 轻量封装。"""

    def __init__(self) -> None:
        """
        @brief 加载 PCAN-Basic DLL 并配置函数原型。
        @return None
        """
        self.dll = ctypes.WinDLL("PCANBasic.dll")
        self._configure_prototypes()

    def _configure_prototypes(self) -> None:
        """
        @brief 配置 PCAN-Basic DLL 函数原型。
        @return None
        """
        self.dll.CAN_Initialize.argtypes = [
            ctypes.c_ushort,
            ctypes.c_ushort,
            ctypes.c_ubyte,
            ctypes.c_uint32,
            ctypes.c_ushort,
        ]
        self.dll.CAN_Initialize.restype = ctypes.c_uint32

        self.dll.CAN_Uninitialize.argtypes = [ctypes.c_ushort]
        self.dll.CAN_Uninitialize.restype = ctypes.c_uint32

        self.dll.CAN_Read.argtypes = [
            ctypes.c_ushort,
            ctypes.POINTER(TPCANMsg),
            ctypes.POINTER(TPCANTimestamp),
        ]
        self.dll.CAN_Read.restype = ctypes.c_uint32

        self.dll.CAN_Write.argtypes = [
            ctypes.c_ushort,
            ctypes.POINTER(TPCANMsg),
        ]
        self.dll.CAN_Write.restype = ctypes.c_uint32

        self.dll.CAN_GetErrorText.argtypes = [
            ctypes.c_uint32,
            ctypes.c_ushort,
            ctypes.c_char_p,
        ]
        self.dll.CAN_GetErrorText.restype = ctypes.c_uint32

    def initialize(self, channel: int, bitrate: int) -> int:
        """
        @brief 初始化 PCAN 通道。
        @param channel PCAN 通道。
        @param bitrate PCAN 波特率枚举值。
        @return int PCAN-Basic 状态码。
        """
        return int(self.dll.CAN_Initialize(channel, bitrate, 0, 0, 0))

    def probe_channel(self, channel: int, bitrate: int) -> bool:
        """
        @brief 探测 PCAN 通道是否可以初始化。
        @param channel PCAN 通道。
        @param bitrate PCAN 波特率枚举值。
        @return bool true 表示可用，false 表示不可用。
        """
        result = self.initialize(channel, bitrate)

        if result != PCAN_ERROR_OK:
            return False

        self.uninitialize(channel)
        return True

    def uninitialize(self, channel: int) -> int:
        """
        @brief 反初始化 PCAN 通道。
        @param channel PCAN 通道。
        @return int PCAN-Basic 状态码。
        """
        return int(self.dll.CAN_Uninitialize(channel))

    def read(self, channel: int) -> tuple[int, TPCANMsg, TPCANTimestamp]:
        """
        @brief 从 PCAN 通道读取一帧。
        @param channel PCAN 通道。
        @return tuple[int, TPCANMsg, TPCANTimestamp] 状态码、报文和时间戳。
        """
        message = TPCANMsg()
        timestamp = TPCANTimestamp()
        result = self.dll.CAN_Read(channel,
                                   ctypes.byref(message),
                                   ctypes.byref(timestamp))

        return int(result), message, timestamp

    def write(self, channel: int, frame_id: int, payload: bytes) -> int:
        """
        @brief 向 PCAN 通道发送标准帧。
        @param channel PCAN 通道。
        @param frame_id 标准帧 ID。
        @param payload 负载字节。
        @return int PCAN-Basic 状态码。
        """
        message = TPCANMsg()
        message.ID = frame_id
        message.MSGTYPE = PCAN_MESSAGE_STANDARD
        message.LEN = len(payload)

        for index, byte_value in enumerate(payload):
            message.DATA[index] = byte_value

        return int(self.dll.CAN_Write(channel, ctypes.byref(message)))

    def get_error_text(self, error_code: int) -> str:
        """
        @brief 获取 PCAN-Basic 错误文本。
        @param error_code PCAN-Basic 状态码。
        @return str 错误文本。
        """
        buffer = ctypes.create_string_buffer(256)
        result = self.dll.CAN_GetErrorText(error_code, 0x09, buffer)

        if int(result) != PCAN_ERROR_OK:
            return f"PCAN error 0x{error_code:08X}"

        return buffer.value.decode(errors="replace")


class PcanClient:
    """@brief PCAN 收发线程封装。"""

    def __init__(self, event_queue: queue.Queue[object]) -> None:
        """
        @brief 初始化 PCAN 客户端。
        @param event_queue UI 事件队列。
        @return None
        """
        self.event_queue = event_queue
        self.basic: PcanBasic | None = None
        self.channel = 0
        self.running = threading.Event()
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()

    def connect(self, channel: int, bitrate: int) -> None:
        """
        @brief 连接 PCAN 通道并启动接收线程。
        @param channel PCAN 通道。
        @param bitrate PCAN 波特率枚举值。
        @return None
        """
        if self.running.is_set():
            return

        self.basic = PcanBasic()
        result = self.basic.initialize(channel, bitrate)

        if result != PCAN_ERROR_OK:
            raise RuntimeError(self.basic.get_error_text(result))

        self.channel = channel
        self.running.set()
        self.thread = threading.Thread(target=self._read_worker,
                                       name="pcan-read",
                                       daemon=True)
        self.thread.start()

    def scan_channels(self, bitrate: int) -> list[tuple[str, int]]:
        """
        @brief 扫描当前可初始化的 PCAN 通道。
        @param bitrate PCAN 波特率枚举值。
        @return list[tuple[str, int]] 可用通道名称和值列表。
        """
        available_channels: list[tuple[str, int]] = []
        basic = PcanBasic()

        for channel_name, channel_value in PCAN_CHANNELS.items():
            if basic.probe_channel(channel_value, bitrate):
                available_channels.append((channel_name, channel_value))

        return available_channels

    def disconnect(self) -> None:
        """
        @brief 断开 PCAN 通道并停止接收线程。
        @return None
        """
        self.running.clear()

        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

        if self.basic is not None and self.channel != 0:
            self.basic.uninitialize(self.channel)

        self.channel = 0
        self.basic = None

    def send_frame(self, frame_id: int, payload: bytes) -> None:
        """
        @brief 发送标准 CAN 帧。
        @param frame_id 标准帧 ID。
        @param payload 负载字节。
        @return None
        """
        if self.basic is None or self.channel == 0:
            raise RuntimeError("PCAN 未连接")

        with self.lock:
            result = self.basic.write(self.channel, frame_id, payload)

        if result != PCAN_ERROR_OK:
            raise RuntimeError(self.basic.get_error_text(result))

    def send_pid_get(self, axis_name: str, loop_name: str, field_name: str) -> None:
        """
        @brief 发送 PID 字段读取请求。
        @param axis_name 云台轴名称。
        @param loop_name PID 环名称。
        @param field_name PID 字段名称。
        @return None
        """
        axis_id = PID_AXIS_IDS[axis_name]
        selector = make_pid_selector(loop_name, field_name)
        payload = bytes([GIMBAL_CMD_GET_PID_FIELD, axis_id, selector, 0, 0, 0, 0, 0])
        self.send_frame(PCAN_GIMBAL_REQUEST_ID, payload)

    def send_pid_get_all(self, axis_name: str, loop_name: str) -> None:
        """
        @brief 发送整组 PID 字段读取请求。
        @param axis_name 云台轴名称。
        @param loop_name PID 环名称。
        @return None
        """
        for field_name in PID_FIELD_ORDER:
            self.send_pid_get(axis_name, loop_name, field_name)
            time.sleep(0.002)

    def send_pid_set(self,
                     axis_name: str,
                     loop_name: str,
                     field_name: str,
                     value: int) -> None:
        """
        @brief 发送 PID 字段设置请求。
        @param axis_name 云台轴名称。
        @param loop_name PID 环名称。
        @param field_name PID 字段名称。
        @param value 字段值。
        @return None
        """
        axis_id = PID_AXIS_IDS[axis_name]
        selector = make_pid_selector(loop_name, field_name)
        payload = bytes([GIMBAL_CMD_SET_PID_FIELD, axis_id, selector])
        payload += int32_to_le(value)
        payload += bytes([0])
        self.send_frame(PCAN_GIMBAL_REQUEST_ID, payload)

    def send_pid_set_all(self,
                         axis_name: str,
                         loop_name: str,
                         values: dict[str, int]) -> None:
        """
        @brief 发送整组 PID 字段设置请求。
        @param axis_name 云台轴名称。
        @param loop_name PID 环名称。
        @param values PID 字段值表。
        @return None
        """
        for field_name in PID_FIELD_ORDER:
            self.send_pid_set(axis_name,
                              loop_name,
                              field_name,
                              values[field_name])
            time.sleep(0.002)

    def send_axis_target(self, axis_name: str, target_mrad: int) -> None:
        """
        @brief 发送单轴目标角度设置请求。
        @param axis_name 云台轴名称。
        @param target_mrad 目标角度，单位 mrad。
        @return None
        """
        axis_id = PID_AXIS_IDS[axis_name]
        payload = bytes([GIMBAL_CMD_SET_AXIS_TARGET, axis_id])
        payload += int32_to_le(target_mrad)
        payload += bytes([0, 0])
        self.send_frame(PCAN_GIMBAL_REQUEST_ID, payload)

    def send_disable(self) -> None:
        """
        @brief 发送关闭云台输出命令。
        @return None
        """
        payload = bytes([GIMBAL_CMD_DISABLE, 0, 0, 0, 0, 0, 0, 0])
        self.send_frame(PCAN_GIMBAL_REQUEST_ID, payload)

    def _read_worker(self) -> None:
        """
        @brief PCAN 接收线程入口。
        @return None
        """
        assert self.basic is not None

        while self.running.is_set():
            read_any = False

            while self.running.is_set():
                result, message, _timestamp = self.basic.read(self.channel)

                if result == PCAN_ERROR_QRCVEMPTY:
                    break

                if result != PCAN_ERROR_OK:
                    error_text = self.basic.get_error_text(result)
                    self.event_queue.put(ErrorEvent(error_text))
                    break

                read_any = True
                self._handle_message(message)

            if read_any:
                time.sleep(0.001)
            else:
                time.sleep(0.005)

    def _handle_message(self, message: TPCANMsg) -> None:
        """
        @brief 处理一帧 PCAN 报文。
        @param message PCAN 报文。
        @return None
        """
        data = bytes(message.DATA[:message.LEN])

        if message.ID in range(PCAN_MOTOR_REPORT_BASE_ID + 1,
                               PCAN_MOTOR_REPORT_BASE_ID + 4):
            if len(data) >= 8:
                node_id = message.ID - PCAN_MOTOR_REPORT_BASE_ID
                position_mrad = int32_from_le(data[0:4])
                speed_mrad_s = int32_from_le(data[4:8])
                event = MotorReportEvent(node_id=node_id,
                                         position_mrad=position_mrad,
                                         speed_mrad_s=speed_mrad_s,
                                         timestamp_ms=now_monotonic_ms())
                self._put_motor_event(event)

            return

        if message.ID == PCAN_GIMBAL_RESPONSE_ID:
            if len(data) >= 8:
                event = PidResponseEvent(command=data[0],
                                         status=data[1],
                                         axis_id=data[2],
                                         selector=data[3],
                                         value=int32_from_le(data[4:8]))
                self.event_queue.put(event)

    def _put_motor_event(self, event: MotorReportEvent) -> None:
        """
        @brief 投递电机上报事件。
        @param event 电机主动上报事件。
        @return None
        @note 电机上报频率高，队列满时丢弃新采样，避免 GUI 长时间运行后卡死。
        """
        if self.event_queue.qsize() >= EVENT_QUEUE_MAX:
            return

        self.event_queue.put(event)


class GimbalToolApp:
    """@brief Duck Mid PCAN 云台调试 GUI。"""

    def __init__(self, root: tk.Tk, default_channel: int, default_bitrate: str) -> None:
        """
        @brief 初始化 GUI。
        @param root Tk 根窗口。
        @param default_channel 默认 PCAN 通道。
        @param default_bitrate 默认波特率名称。
        @return None
        """
        self.root = root
        self.root.title("Duck Mid PCAN Gimbal Tool")
        self.events: queue.Queue[object] = queue.Queue()
        self.client = PcanClient(self.events)
        self.motor_states = {
            node_id: MotorState(name=name, node_id=node_id)
            for node_id, name in MOTOR_NODE_NAMES.items()
        }
        self.channel_var = tk.StringVar(value=channel_label_from_value(default_channel))
        self.bitrate_var = tk.StringVar(value=default_bitrate)
        self.axis_var = tk.StringVar(value="pitch")
        self.loop_var = tk.StringVar(value="speed")
        self.target_var = tk.StringVar(value="0")
        self.chart_window_var = tk.StringVar(value=str(int(CHART_HISTORY_SECONDS)))
        self.pid_value_vars = {
            field_name: tk.StringVar(value="0")
            for field_name in PID_FIELD_ORDER
        }
        self.status_var = tk.StringVar(value="未连接")
        self.connected = False
        self.chart_history = {
            node_id: deque(maxlen=CHART_MAX_POINTS)
            for node_id in MOTOR_NODE_NAMES
        }

        self._build_layout()
        self._refresh_motor_table()
        self._refresh_chart()
        self._process_events()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_layout(self) -> None:
        """
        @brief 创建 GUI 布局。
        @return None
        """
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)
        self.root.rowconfigure(2, weight=0)
        self.root.rowconfigure(3, weight=1)

        top_frame = ttk.Frame(self.root, padding=8)
        top_frame.grid(row=0, column=0, sticky="ew")
        top_frame.columnconfigure(6, weight=1)

        ttk.Label(top_frame, text="PCAN 通道").grid(row=0, column=0, padx=(0, 4))
        self.channel_combo = ttk.Combobox(top_frame,
                                          textvariable=self.channel_var,
                                          values=PCAN_CHANNEL_LABELS,
                                          width=22)
        self.channel_combo.grid(row=0, column=1, padx=(0, 8))
        ttk.Label(top_frame, text="波特率").grid(row=0, column=2, padx=(0, 4))
        ttk.Combobox(top_frame,
                     textvariable=self.bitrate_var,
                     values=list(PCAN_BITRATES.keys()),
                     width=8,
                     state="readonly").grid(row=0, column=3, padx=(0, 12))
        ttk.Button(top_frame,
                   text="扫描",
                   command=self._scan_channels).grid(row=0, column=4, padx=(0, 8))
        self.connect_button = ttk.Button(top_frame,
                                         text="连接",
                                         command=self._toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(0, 12))
        ttk.Label(top_frame, textvariable=self.status_var).grid(row=0,
                                                                column=6,
                                                                sticky="w")

        self._build_motor_table()
        self._build_pid_panel()
        self._build_chart_panel()

    def _build_motor_table(self) -> None:
        """
        @brief 创建电机状态表格。
        @return None
        """
        table_frame = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        table_frame.grid(row=1, column=0, sticky="nsew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        columns = ("node", "position", "degree", "speed", "age", "count")
        self.motor_table = ttk.Treeview(table_frame,
                                        columns=columns,
                                        show="tree headings",
                                        height=4)
        self.motor_table.heading("#0", text="电机")
        self.motor_table.heading("node", text="节点")
        self.motor_table.heading("position", text="位置 mrad")
        self.motor_table.heading("degree", text="位置 deg")
        self.motor_table.heading("speed", text="速度 mrad/s")
        self.motor_table.heading("age", text="更新 ms")
        self.motor_table.heading("count", text="收包")

        self.motor_table.column("#0", width=90, anchor="center")
        self.motor_table.column("node", width=70, anchor="center")
        self.motor_table.column("position", width=120, anchor="e")
        self.motor_table.column("degree", width=100, anchor="e")
        self.motor_table.column("speed", width=120, anchor="e")
        self.motor_table.column("age", width=90, anchor="e")
        self.motor_table.column("count", width=80, anchor="e")
        self.motor_table.grid(row=0, column=0, sticky="nsew")

        for node_id in (0x01, 0x02, 0x03):
            state = self.motor_states[node_id]
            self.motor_table.insert("",
                                    "end",
                                    iid=str(node_id),
                                    text=state.name,
                                    values=(f"0x{node_id:02X}",
                                            "--",
                                            "--",
                                            "--",
                                            "--",
                                            "0"))

    def _build_chart_panel(self) -> None:
        """
        @brief 创建电机位置和速度曲线区域。
        @return None
        """
        chart_frame = ttk.LabelFrame(self.root, text="当前轴曲线", padding=8)
        chart_frame.grid(row=3, column=0, sticky="nsew", padx=8, pady=(0, 8))
        chart_frame.rowconfigure(1, weight=1)
        chart_frame.columnconfigure(0, weight=1)

        chart_toolbar = ttk.Frame(chart_frame)
        chart_toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(chart_toolbar, text="窗口 s").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(chart_toolbar,
                  textvariable=self.chart_window_var,
                  width=8).grid(row=0, column=1, padx=(0, 8))
        ttk.Label(chart_toolbar,
                  text="横轴按最近窗口数据铺满").grid(row=0, column=2, sticky="w")

        self.chart_canvas = tk.Canvas(chart_frame,
                                      height=260,
                                      bg="#111827",
                                      highlightthickness=0)
        self.chart_canvas.grid(row=1, column=0, sticky="nsew")

    def _build_pid_panel(self) -> None:
        """
        @brief 创建 PID 调试面板。
        @return None
        """
        pid_frame = ttk.LabelFrame(self.root, text="PID 调试", padding=8)
        pid_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 8))
        pid_frame.columnconfigure(12, weight=1)

        ttk.Label(pid_frame, text="轴").grid(row=0, column=0, padx=(0, 4))
        ttk.Combobox(pid_frame,
                     textvariable=self.axis_var,
                     values=list(PID_AXIS_IDS.keys()),
                     width=8,
                     state="readonly").grid(row=0, column=1, padx=(0, 8))

        ttk.Label(pid_frame, text="环").grid(row=0, column=2, padx=(0, 4))
        ttk.Combobox(pid_frame,
                     textvariable=self.loop_var,
                     values=list(PID_LOOP_IDS.keys()),
                     width=10,
                     state="readonly").grid(row=0, column=3, padx=(0, 8))

        ttk.Button(pid_frame, text="读取整组", command=self._send_pid_get_all).grid(row=0,
                                                                                  column=4,
                                                                                  padx=(0, 8))
        ttk.Button(pid_frame, text="设置整组", command=self._send_pid_set_all).grid(row=0,
                                                                                  column=5,
                                                                                  padx=(0, 8))
        ttk.Button(pid_frame,
                   text="关闭输出",
                   command=self._send_disable).grid(row=0, column=6, padx=(0, 8))

        ttk.Label(pid_frame, text="目标 mrad").grid(row=0, column=7, padx=(8, 4))
        ttk.Entry(pid_frame,
                  textvariable=self.target_var,
                  width=12).grid(row=0, column=8, padx=(0, 8))
        ttk.Button(pid_frame,
                   text="设置目标",
                   command=self._send_axis_target).grid(row=0, column=9, padx=(0, 8))

        for column_index, field_name in enumerate(PID_FIELD_ORDER):
            ttk.Label(pid_frame, text=field_name).grid(row=1,
                                                       column=column_index,
                                                       padx=(0, 4),
                                                       pady=(8, 0),
                                                       sticky="w")
            ttk.Entry(pid_frame,
                      textvariable=self.pid_value_vars[field_name],
                      width=14).grid(row=2,
                                     column=column_index,
                                     padx=(0, 8),
                                     sticky="ew")

        self.log_text = tk.Text(pid_frame, height=7, width=100)
        self.log_text.grid(row=3, column=0, columnspan=13, sticky="ew", pady=(8, 0))
        self.log_text.configure(state="disabled")

    def _toggle_connection(self) -> None:
        """
        @brief 切换 PCAN 连接状态。
        @return None
        """
        if self.connected:
            self.client.disconnect()
            self.connected = False
            self.connect_button.configure(text="连接")
            self.status_var.set("未连接")
            return

        try:
            channel = channel_value_from_label(self.channel_var.get())
            bitrate = PCAN_BITRATES[self.bitrate_var.get()]
            self.client.connect(channel, bitrate)
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return

        self.connected = True
        self.connect_button.configure(text="断开")
        self.status_var.set(f"已连接 channel=0x{channel:X} bitrate={self.bitrate_var.get()}")
        self._append_log("PCAN connected")

    def _scan_channels(self) -> None:
        """
        @brief 扫描当前可用 PCAN 通道。
        @return None
        """
        try:
            bitrate = PCAN_BITRATES[self.bitrate_var.get()]
            available_channels = self.client.scan_channels(bitrate)
        except Exception as exc:
            messagebox.showerror("扫描失败", str(exc))
            return

        if len(available_channels) == 0:
            self.status_var.set("未找到可用 PCAN 通道")
            messagebox.showwarning("扫描结果", "未找到可用 PCAN 通道")
            return

        labels = [
            f"{channel_name} (0x{channel_value:X})"
            for channel_name, channel_value in available_channels
        ]
        self.channel_combo.configure(values=labels)
        self.channel_var.set(labels[0])
        self.status_var.set(f"找到 {len(labels)} 个 PCAN 通道")
        self._append_log("Available channels: " + ", ".join(labels))

    def _send_pid_get_all(self) -> None:
        """
        @brief 发送整组 PID 读取请求。
        @return None
        """
        try:
            self.client.send_pid_get_all(self.axis_var.get(), self.loop_var.get())
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _send_pid_set_all(self) -> None:
        """
        @brief 发送整组 PID 设置请求。
        @return None
        """
        try:
            values = {
                field_name: parse_int_auto(self.pid_value_vars[field_name].get())
                for field_name in PID_FIELD_ORDER
            }
            self.client.send_pid_set_all(self.axis_var.get(),
                                         self.loop_var.get(),
                                         values)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _send_axis_target(self) -> None:
        """
        @brief 发送单轴目标角度设置请求。
        @return None
        """
        try:
            target_mrad = parse_int_auto(self.target_var.get())
            self.client.send_axis_target(self.axis_var.get(), target_mrad)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _send_disable(self) -> None:
        """
        @brief 发送关闭云台输出请求。
        @return None
        """
        try:
            self.client.send_disable()
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _process_events(self) -> None:
        """
        @brief 处理 PCAN 接收线程投递的事件。
        @return None
        """
        processed_count = 0

        while processed_count < EVENT_PROCESS_LIMIT:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break

            processed_count += 1

            if isinstance(event, MotorReportEvent):
                self._handle_motor_report(event)

            if isinstance(event, PidResponseEvent):
                self._handle_pid_response(event)

            if isinstance(event, ErrorEvent):
                self._append_log(f"ERROR: {event.message}")

        self.root.after(20, self._process_events)

    def _handle_motor_report(self, event: MotorReportEvent) -> None:
        """
        @brief 处理电机主动上报事件。
        @param event 电机主动上报事件。
        @return None
        """
        state = self.motor_states.get(event.node_id)

        if state is None:
            return

        state.position_mrad = event.position_mrad
        state.speed_mrad_s = event.speed_mrad_s
        state.last_update_ms = event.timestamp_ms
        state.rx_count += 1
        self._append_chart_sample(event)

    def _handle_pid_response(self, event: PidResponseEvent) -> None:
        """
        @brief 处理 PID 调试响应事件。
        @param event PID 调试响应事件。
        @return None
        """
        axis_name = find_key_by_value(PID_AXIS_IDS, event.axis_id)
        status = GIMBAL_STATUS_TEXT.get(event.status, str(event.status))

        if event.command == GIMBAL_CMD_SET_AXIS_TARGET:
            self._append_log(
                f"rsp cmd=0x{event.command:02X} status={status} "
                f"axis={axis_name} target={event.value}"
            )
            return

        loop_id = event.selector >> 4
        field_id = event.selector & 0x0F
        loop_name = find_key_by_value(PID_LOOP_IDS, loop_id)
        field_name = find_key_by_value(PID_FIELD_IDS, field_id)

        self._append_log(
            f"rsp cmd=0x{event.command:02X} status={status} "
            f"axis={axis_name} loop={loop_name} field={field_name} value={event.value}"
        )

        if event.status == 0 and event.command == GIMBAL_CMD_GET_PID_FIELD:
            value_var = self.pid_value_vars.get(field_name)

            if value_var is not None:
                value_var.set(str(event.value))

    def _refresh_motor_table(self) -> None:
        """
        @brief 刷新电机状态表格。
        @return None
        """
        current_ms = now_monotonic_ms()

        for node_id, state in self.motor_states.items():
            if state.last_update_ms is None:
                age_text = "--"
            else:
                age_text = str(current_ms - state.last_update_ms)

            self.motor_table.item(str(node_id),
                                  values=(f"0x{node_id:02X}",
                                          format_optional_int(state.position_mrad),
                                          mrad_to_degree(state.position_mrad),
                                          format_optional_int(state.speed_mrad_s),
                                          age_text,
                                          str(state.rx_count)))

        self.root.after(100, self._refresh_motor_table)

    def _append_chart_sample(self, event: MotorReportEvent) -> None:
        """
        @brief 追加一条电机曲线采样。
        @param event 电机主动上报事件。
        @return None
        """
        history = self.chart_history.get(event.node_id)

        if history is None:
            return

        history.append((event.timestamp_ms, event.position_mrad, event.speed_mrad_s))

        cutoff_ms = event.timestamp_ms - int(CHART_MAX_WINDOW_SECONDS * 1000.0)

        while len(history) > 0 and history[0][0] < cutoff_ms:
            history.popleft()

    def _get_chart_window_ms(self) -> int:
        """
        @brief 获取曲线显示窗口长度。
        @return 曲线显示窗口长度，单位 ms。
        """
        try:
            window_seconds = float(self.chart_window_var.get().strip())
        except ValueError:
            window_seconds = CHART_HISTORY_SECONDS

        if not math.isfinite(window_seconds):
            window_seconds = CHART_HISTORY_SECONDS

        if window_seconds < CHART_MIN_WINDOW_SECONDS:
            window_seconds = CHART_MIN_WINDOW_SECONDS

        if window_seconds > CHART_MAX_WINDOW_SECONDS:
            window_seconds = CHART_MAX_WINDOW_SECONDS

        return int(window_seconds * 1000.0)

    def _decimate_chart_history(self,
                                history: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
        """
        @brief 抽样曲线历史数据。
        @param history 曲线历史数据。
        @return 抽样后的曲线历史数据。
        """
        if len(history) <= CHART_MAX_DRAW_POINTS:
            return history

        bucket_count = max(CHART_MAX_DRAW_POINTS // 5, 1)
        step = math.ceil(len(history) / bucket_count)
        decimated_history: list[tuple[int, int, int]] = []

        for start_index in range(0, len(history), step):
            bucket = history[start_index:start_index + step]
            selected_indices = {
                0,
                len(bucket) - 1,
            }
            selected_indices.add(min(range(len(bucket)), key=lambda index: bucket[index][1]))
            selected_indices.add(max(range(len(bucket)), key=lambda index: bucket[index][1]))
            selected_indices.add(min(range(len(bucket)), key=lambda index: bucket[index][2]))
            selected_indices.add(max(range(len(bucket)), key=lambda index: bucket[index][2]))

            for bucket_index in sorted(selected_indices):
                decimated_history.append(bucket[bucket_index])

        return decimated_history

    def _refresh_chart(self) -> None:
        """
        @brief 刷新当前轴位置和速度曲线。
        @return None
        """
        self._draw_chart()
        self.root.after(CHART_REFRESH_MS, self._refresh_chart)

    def _draw_chart(self) -> None:
        """
        @brief 绘制当前轴位置和速度曲线。
        @return None
        """
        axis_name = self.axis_var.get()
        axis_id = PID_AXIS_IDS.get(axis_name, 0)
        node_id = self._node_id_from_axis_id(axis_id)
        history = self.chart_history.get(node_id, [])
        canvas = self.chart_canvas
        width = max(canvas.winfo_width(), 400)
        height = max(canvas.winfo_height(), 160)
        left = 58
        right = width - 18
        top = 18
        bottom = height - 32
        mid_y = (top + bottom) // 2
        current_ms = now_monotonic_ms()
        window_ms = self._get_chart_window_ms()

        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#111827", outline="")
        canvas.create_text(left,
                           8,
                           text=f"{axis_name} position/speed {window_ms / 1000.0:g}s",
                           fill="#e5e7eb",
                           anchor="nw")

        self._draw_chart_grid(canvas, left, right, top, bottom, mid_y)

        recent_history = [
            sample
            for sample in history
            if sample[0] >= current_ms - window_ms
        ]

        if len(recent_history) < 2:
            canvas.create_text(width // 2,
                               height // 2,
                               text="waiting motor report...",
                               fill="#9ca3af")
            return

        draw_history = self._decimate_chart_history(recent_history)

        self._draw_chart_series(canvas,
                                draw_history,
                                value_index=1,
                                color="#38bdf8",
                                left=left,
                                right=right,
                                top=top,
                                bottom=mid_y - 8)
        self._draw_chart_series(canvas,
                                draw_history,
                                value_index=2,
                                color="#f97316",
                                left=left,
                                right=right,
                                top=mid_y + 8,
                                bottom=bottom)

        last_sample = recent_history[-1]
        canvas.create_text(8,
                           top,
                           text=f"pos\n{last_sample[1]}",
                           fill="#38bdf8",
                           anchor="nw")
        canvas.create_text(8,
                           mid_y + 8,
                           text=f"spd\n{last_sample[2]}",
                           fill="#f97316",
                           anchor="nw")

    def _draw_chart_grid(self,
                         canvas: tk.Canvas,
                         left: int,
                         right: int,
                         top: int,
                         bottom: int,
                         mid_y: int) -> None:
        """
        @brief 绘制曲线网格。
        @param canvas 目标画布。
        @param left 绘图区左边界。
        @param right 绘图区右边界。
        @param top 绘图区上边界。
        @param bottom 绘图区下边界。
        @param mid_y 中线坐标。
        @return None
        """
        for index in range(6):
            x = left + ((right - left) * index) // 5
            canvas.create_line(x, top, x, bottom, fill="#1f2937")

        canvas.create_line(left, top, right, top, fill="#374151")
        canvas.create_line(left, mid_y, right, mid_y, fill="#374151")
        canvas.create_line(left, bottom, right, bottom, fill="#374151")

    def _draw_chart_series(self,
                           canvas: tk.Canvas,
                           history: list[tuple[int, int, int]],
                           value_index: int,
                           color: str,
                           left: int,
                           right: int,
                           top: int,
                           bottom: int) -> None:
        """
        @brief 绘制一条曲线。
        @param canvas 目标画布。
        @param history 曲线历史数据。
        @param value_index 数值字段下标。
        @param color 曲线颜色。
        @param left 绘图区左边界。
        @param right 绘图区右边界。
        @param top 绘图区上边界。
        @param bottom 绘图区下边界。
        @return None
        """
        values = [sample[value_index] for sample in history]
        min_value = min(values)
        max_value = max(values)
        span = max(max_value - min_value, 1)
        points: list[float] = []
        first_ms = history[0][0]
        last_ms = history[-1][0]
        window_ms = max(last_ms - first_ms, 1)

        for sample in history:
            x = left + ((right - left) * (sample[0] - first_ms) / window_ms)
            y = bottom - ((sample[value_index] - min_value) * (bottom - top) / span)
            points.extend([x, y])

        if len(points) >= 4:
            canvas.create_line(*points, fill=color, width=2)

        canvas.create_text(right,
                           top,
                           text=str(max_value),
                           fill=color,
                           anchor="ne")
        canvas.create_text(right,
                           bottom,
                           text=str(min_value),
                           fill=color,
                           anchor="se")

    def _node_id_from_axis_id(self, axis_id: int) -> int:
        """
        @brief 将云台轴编号转换为电机节点 ID。
        @param axis_id 云台轴编号。
        @return int 电机节点 ID。
        """
        if axis_id == PID_AXIS_IDS["pitch"]:
            return 0x01

        if axis_id == PID_AXIS_IDS["yaw"]:
            return 0x02

        return 0x03

    def _append_log(self, text: str) -> None:
        """
        @brief 向日志窗口追加一行文本。
        @param text 日志文本。
        @return None
        """
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {text}\n")
        self._trim_log_text()
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _trim_log_text(self) -> None:
        """
        @brief 裁剪日志窗口行数。
        @return None
        """
        line_count = int(float(self.log_text.index("end-1c").split(".")[0]))

        if line_count <= LOG_MAX_LINES:
            return

        delete_end_line = line_count - LOG_MAX_LINES + 1
        self.log_text.delete("1.0", f"{delete_end_line}.0")

    def _on_close(self) -> None:
        """
        @brief 处理窗口关闭事件。
        @return None
        """
        self.client.disconnect()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    """
    @brief 解析命令行参数。
    @return argparse.Namespace 命令行参数。
    """
    parser = argparse.ArgumentParser(description="Duck Mid PCAN gimbal tool")
    parser.add_argument("--channel",
                        default=PCAN_DEFAULT_CHANNEL_NAME,
                        help="PCAN channel name or hex value, default: PCAN_USBBUS1")
    parser.add_argument("--bitrate",
                        default="500K",
                        choices=list(PCAN_BITRATES.keys()),
                        help="CAN bitrate, default: 500K")

    return parser.parse_args()


def main() -> None:
    """
    @brief 程序入口。
    @return None
    """
    args = parse_args()
    root = tk.Tk()
    GimbalToolApp(root,
                  default_channel=channel_value_from_label(args.channel),
                  default_bitrate=args.bitrate)
    root.mainloop()


if __name__ == "__main__":
    main()
