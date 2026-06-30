#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi vehicle server: manual keyboard control + live ArUco camera.

웹 구조:
- templates/web.html: 기존 Web.html의 상단 디자인 유지 + 하단 카메라/방향키 패널 추가
- /video_feed: 실시간 카메라 MJPEG 스트림
- /api/route_markers: 웹에서 계산한 최적 경로의 ArUco ID 등록
- /api/drive_state: WASD 키 상태를 받아 ESC/서보 PWM 갱신
- /api/status: 상태 확인

자율주행 기능:
- 없음.
- /start, /stop 자율주행 루프를 사용하지 않음.
- 안전용 내부 endpoint로 /api/neutral과 /emergency_stop은 남겨 두지만, 웹 UI에는 중립 전송 버튼을 두지 않음.
"""

import json
import os
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Set

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template, request

# ============================================================
# HARDWARE / PWM SETTINGS
# 기존 코드의 하드웨어 값, ESC/서보 보정값을 여기에 그대로 옮겨 넣으세요.
# ============================================================

ESC_CHANNEL = 0          # 사용자가 확인한 ESC 채널
SERVO_CHANNEL = 1        # 기존 서보 채널이 다르면 기존 값으로 수정
PWM_FREQ = 50

# 예시 안전값. 기존 코드에 보정값이 있으면 반드시 기존 값을 유지하세요.
ESC_NEUTRAL_US = 1500
ESC_FORWARD_US = 1570
ESC_REVERSE_US = 1430

# 조향은 이전 방식처럼 서보 각도 기준으로 관리합니다.
# 단, 중심 PWM 1500us는 고정 유지합니다.
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_CENTER_ANGLE = 22
SERVO_LEFT_ANGLE = 35
SERVO_RIGHT_ANGLE = 9
SERVO_CENTER_US = 1500

# 키 입력/heartbeat가 끊기면 자동 중립
COMMAND_TIMEOUT_SEC = 0.45

# Camera
# CAMERA_INDEX를 None으로 두면 /dev/video0~3과 0~3 index를 자동 탐색합니다.
# 특정 카메라만 강제로 쓰려면 0, 1, 2처럼 숫자로 지정하세요.
CAMERA_INDEX = None
CAMERA_CANDIDATES = [0, 1, 2, 3]
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15
JPEG_QUALITY = 70

# auto: Picamera2 먼저 시도 후 OpenCV/V4L2 시도
# opencv: USB 카메라/OpenCV만 사용
# picamera2: CSI 카메라/Picamera2만 사용
CAMERA_BACKEND = os.environ.get("CAMERA_BACKEND", "auto").strip().lower()
BLACK_FRAME_MEAN_THRESHOLD = 8.0
BLACK_FRAME_STD_THRESHOLD = 4.0
BLACK_FRAME_RETRY_LIMIT = 15

# 환경변수로도 강제 지정 가능:
# CAMERA_INDEX=1 python3 raspberry_pi_car_server_manual_web.py
_env_camera_index = os.environ.get("CAMERA_INDEX")
if _env_camera_index is not None and _env_camera_index.strip() != "":
    try:
        CAMERA_INDEX = int(_env_camera_index)
    except ValueError:
        CAMERA_INDEX = None

# ArUco
# 기존 마커 생성 때 사용한 dictionary와 일치해야 합니다.
ARUCO_DICT_NAME = "DICT_4X4_50"

# 웹 격자 좌표와 실제 ArUco marker ID의 매핑 파일입니다.
# 기존처럼 이 파일 하나만 수정하면 전체 코드가 같은 매핑을 사용합니다.
GRID_MARKER_MAP_FILE = Path(__file__).with_name("grid_marker_map.json")

# ============================================================
# PCA9685
# ============================================================

HARDWARE_AVAILABLE = True
PCA9685_IMPORT_ERROR = None

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except Exception as exc:  # noqa: BLE001
    HARDWARE_AVAILABLE = False
    PCA9685_IMPORT_ERROR = repr(exc)


def us_to_duty_cycle(pulse_us: int, freq: int = PWM_FREQ) -> int:
    period_us = 1_000_000.0 / freq
    duty = int((pulse_us / period_us) * 0xFFFF)
    return max(0, min(0xFFFF, duty))


def servo_angle_to_us(angle: float) -> int:
    """Convert servo angle to PWM pulse width. Center angle is forced to 1500us."""
    if angle == SERVO_CENTER_ANGLE:
        return SERVO_CENTER_US

    angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle)))
    ratio = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)
    pulse = SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US)
    return int(round(pulse))


SERVO_LEFT_US = servo_angle_to_us(SERVO_LEFT_ANGLE)
SERVO_RIGHT_US = servo_angle_to_us(SERVO_RIGHT_ANGLE)


class CarController:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.last_command_ts = 0.0
        self.last_pwm = {
            "throttle_us": ESC_NEUTRAL_US,
            "servo_us": SERVO_CENTER_US,
        }
        self.last_state: Dict[str, bool] = {
            "up": False,
            "down": False,
            "left": False,
            "right": False,
        }
        self.pca = None

        if HARDWARE_AVAILABLE:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = PWM_FREQ

        self.neutral()
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def _write_us(self, channel: int, pulse_us: int) -> None:
        if self.pca is None:
            # PC 테스트 모드: 실제 PWM 출력 없음
            return
        self.pca.channels[channel].duty_cycle = us_to_duty_cycle(pulse_us)

    def neutral(self) -> Dict[str, int]:
        with self.lock:
            self._write_us(ESC_CHANNEL, ESC_NEUTRAL_US)
            self._write_us(SERVO_CHANNEL, SERVO_CENTER_US)
            self.last_state = {"up": False, "down": False, "left": False, "right": False}
            self.last_pwm = {
                "throttle_us": ESC_NEUTRAL_US,
                "servo_us": SERVO_CENTER_US,
                "servo_angle": SERVO_CENTER_ANGLE,
            }
            self.last_command_ts = time.time()
            return dict(self.last_pwm)

    def apply_keyboard_state(self, state: Dict[str, bool]) -> Dict[str, int]:
        up = bool(state.get("up", False))       # W
        down = bool(state.get("down", False))   # S
        left = bool(state.get("left", False))   # A
        right = bool(state.get("right", False)) # D

        # ESC: W/S가 없으면 1500us 중립.
        if up and not down:
            throttle_us = ESC_FORWARD_US
        elif down and not up:
            throttle_us = ESC_REVERSE_US
        else:
            throttle_us = ESC_NEUTRAL_US

        # SERVO: A 또는 D로 조작한 경우에만 좌/우 각도 출력.
        # A/D 입력이 없거나 둘 다 눌리면 무조건 1500us 중심각으로 복귀.
        if left and not right:
            servo_us = SERVO_LEFT_US
            servo_angle = SERVO_LEFT_ANGLE
        elif right and not left:
            servo_us = SERVO_RIGHT_US
            servo_angle = SERVO_RIGHT_ANGLE
        else:
            servo_us = SERVO_CENTER_US
            servo_angle = SERVO_CENTER_ANGLE

        with self.lock:
            self._write_us(ESC_CHANNEL, throttle_us)
            self._write_us(SERVO_CHANNEL, servo_us)
            self.last_state = {"up": up, "down": down, "left": left, "right": right}
            self.last_pwm = {
                "throttle_us": throttle_us,
                "servo_us": servo_us,
                "servo_angle": servo_angle,
            }
            self.last_command_ts = time.time()
            return dict(self.last_pwm)

    def _watchdog_loop(self) -> None:
        while True:
            time.sleep(0.05)
            with self.lock:
                elapsed = time.time() - self.last_command_ts
                state_active = any(self.last_state.values())
            if state_active and elapsed > COMMAND_TIMEOUT_SEC:
                self.neutral()

    def status(self) -> Dict[str, object]:
        with self.lock:
            return {
                "hardware_available": HARDWARE_AVAILABLE,
                "pca9685_import_error": PCA9685_IMPORT_ERROR,
                "last_state": dict(self.last_state),
                "last_pwm": dict(self.last_pwm),
                "last_command_age_sec": round(time.time() - self.last_command_ts, 3),
                "channels": {
                    "esc": ESC_CHANNEL,
                    "servo": SERVO_CHANNEL,
                },
                "pwm_us": {
                    "esc_neutral": ESC_NEUTRAL_US,
                    "esc_forward": ESC_FORWARD_US,
                    "esc_reverse": ESC_REVERSE_US,
                    "servo_center": SERVO_CENTER_US,
                    "servo_left": SERVO_LEFT_US,
                    "servo_right": SERVO_RIGHT_US,
                },
                "servo_angle": {
                    "center": SERVO_CENTER_ANGLE,
                    "left": SERVO_LEFT_ANGLE,
                    "right": SERVO_RIGHT_ANGLE,
                },
            }


# ============================================================
# ArUco / camera
# ============================================================

route_lock = threading.Lock()
optimal_route_marker_ids: Set[int] = set()
latest_route_cells: List[Dict[str, int]] = []


def get_aruco_dictionary():
    if not hasattr(cv2, "aruco"):
        return None

    if not hasattr(cv2.aruco, ARUCO_DICT_NAME):
        raise ValueError(f"Unknown ArUco dictionary: {ARUCO_DICT_NAME}")

    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT_NAME))


def make_aruco_detector():
    aruco_dict = get_aruco_dictionary()
    if aruco_dict is None:
        return None, None, None

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    else:
        detector = None

    return aruco_dict, params, detector


ARUCO_DICT, ARUCO_PARAMS, ARUCO_DETECTOR = make_aruco_detector()


def detect_markers(gray):
    if ARUCO_DICT is None:
        return [], None

    if ARUCO_DETECTOR is not None:
        corners, ids, _ = ARUCO_DETECTOR.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    return corners, ids


def annotate_frame(frame):
    if ARUCO_DICT is None:
        cv2.putText(
            frame,
            "cv2.aruco not available. Install opencv-contrib-python.",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids = detect_markers(gray)

    with route_lock:
        route_ids = set(optimal_route_marker_ids)

    if ids is None or len(ids) == 0:
        cv2.putText(
            frame,
            "No ArUco marker detected",
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    for marker_corners, marker_id_arr in zip(corners, ids.flatten()):
        marker_id = int(marker_id_arr)
        pts = marker_corners.reshape((4, 2)).astype(int)

        is_route = marker_id in route_ids

        # OpenCV color is BGR.
        box_color = (0, 255, 255) if is_route else (255, 255, 255)
        text_color = (0, 255, 255) if is_route else (255, 255, 255)
        thickness = 4 if is_route else 2

        cv2.polylines(frame, [pts], True, box_color, thickness)

        center_x = int(pts[:, 0].mean())
        center_y = int(pts[:, 1].mean())
        cv2.circle(frame, (center_x, center_y), 6 if is_route else 4, box_color, -1)

        label = f"ROUTE ID {marker_id}" if is_route else f"ID {marker_id}"
        cv2.putText(
            frame,
            label,
            (pts[0][0], max(22, pts[0][1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68 if is_route else 0.56,
            text_color,
            2,
            cv2.LINE_AA,
        )

    cv2.putText(
        frame,
        f"Route marker IDs: {sorted(route_ids)}",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return frame


camera_status_lock = threading.Lock()
latest_camera_status: Dict[str, object] = {
    "opened": False,
    "backend": None,
    "selected_index": None,
    "tried": [],
    "last_error": None,
    "black_frame_count": 0,
}


def update_camera_status(**kwargs) -> None:
    with camera_status_lock:
        latest_camera_status.update(kwargs)


def get_camera_status_snapshot() -> Dict[str, object]:
    with camera_status_lock:
        return dict(latest_camera_status)


class CameraSource:
    def read(self):
        raise NotImplementedError

    def release(self) -> None:
        pass


class OpenCVCameraSource(CameraSource):
    def __init__(self, cap, index: int, backend_name: str) -> None:
        self.cap = cap
        self.index = index
        self.backend_name = backend_name

    def read(self):
        return self.cap.read()

    def release(self) -> None:
        try:
            self.cap.release()
        except Exception:
            pass


class Picamera2Source(CameraSource):
    def __init__(self, picam2) -> None:
        self.picam2 = picam2

    def read(self):
        try:
            frame = self.picam2.capture_array()
            if frame is None:
                return False, None

            # Picamera2 RGB888 -> OpenCV BGR
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            return True, frame
        except Exception:
            return False, None

    def release(self) -> None:
        try:
            self.picam2.stop()
        except Exception:
            pass


def is_black_frame(frame) -> bool:
    if frame is None:
        return True
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return float(gray.mean()) < BLACK_FRAME_MEAN_THRESHOLD and float(gray.std()) < BLACK_FRAME_STD_THRESHOLD
    except Exception:
        return False


def open_picamera2_source() -> Optional[CameraSource]:
    tried = [{"backend": "Picamera2", "opened": False}]

    try:
        from picamera2 import Picamera2  # type: ignore

        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
            controls={"FrameRate": CAMERA_FPS},
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.6)

        source = Picamera2Source(picam2)
        ok, frame = source.read()
        if ok and frame is not None and not is_black_frame(frame):
            tried[-1]["opened"] = True
            update_camera_status(
                opened=True,
                backend="Picamera2",
                selected_index="picamera2",
                tried=tried,
                last_error=None,
                black_frame_count=0,
            )
            return source

        # Some cameras need more exposure warm-up time.
        black_count = 0
        for _ in range(10):
            time.sleep(0.12)
            ok, frame = source.read()
            if ok and frame is not None and not is_black_frame(frame):
                tried[-1]["opened"] = True
                update_camera_status(
                    opened=True,
                    backend="Picamera2",
                    selected_index="picamera2",
                    tried=tried,
                    last_error=None,
                    black_frame_count=black_count,
                )
                return source
            black_count += 1

        source.release()
        tried[-1]["black_frame_failed"] = True
        update_camera_status(
            opened=False,
            backend="Picamera2",
            selected_index=None,
            tried=tried,
            last_error="Picamera2 opened but returned black frames.",
            black_frame_count=black_count,
        )
        return None

    except Exception as exc:  # noqa: BLE001
        tried[-1]["error"] = repr(exc)
        update_camera_status(
            opened=False,
            backend="Picamera2",
            selected_index=None,
            tried=tried,
            last_error=repr(exc),
        )
        return None


def open_camera_by_index(index: int):
    """
    Open camera through V4L2 first. This is the normal path for USB webcams
    and for cameras exposed as /dev/video*.
    """
    backends = []
    if hasattr(cv2, "CAP_V4L2"):
        backends.append(("CAP_V4L2", cv2.CAP_V4L2))
    backends.append(("DEFAULT", 0))

    tried = []

    for backend_name, backend_id in backends:
        try:
            if backend_name == "DEFAULT":
                cap = cv2.VideoCapture(index)
            else:
                cap = cv2.VideoCapture(index, backend_id)

            if hasattr(cv2, "VideoWriter_fourcc"):
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            opened = bool(cap.isOpened())
            tried.append({"index": index, "backend": backend_name, "opened": opened})

            if opened:
                # Warm-up frames. Some cameras output black frames immediately after open.
                ok = False
                frame = None
                black_count = 0

                for _ in range(12):
                    ok, frame = cap.read()
                    if ok and frame is not None and not is_black_frame(frame):
                        update_camera_status(
                            opened=True,
                            backend=backend_name,
                            selected_index=index,
                            tried=tried,
                            last_error=None,
                            black_frame_count=black_count,
                        )
                        return OpenCVCameraSource(cap, index, backend_name), tried

                    if ok and frame is not None and is_black_frame(frame):
                        black_count += 1

                    time.sleep(0.08)

                cap.release()
                tried[-1]["opened"] = False
                tried[-1]["black_frame_failed"] = True
                tried[-1]["black_frame_count"] = black_count

        except Exception as exc:  # noqa: BLE001
            tried.append({
                "index": index,
                "backend": backend_name,
                "opened": False,
                "error": repr(exc),
            })

    return None, tried


def open_first_available_camera() -> Optional[CameraSource]:
    """
    Try Picamera2 first for CSI cameras, then OpenCV/V4L2 for USB or /dev/video* cameras.
    """
    all_tried = []

    if CAMERA_BACKEND in ("auto", "picamera2"):
        source = open_picamera2_source()
        picam_status = get_camera_status_snapshot()
        all_tried.extend(picam_status.get("tried", []))
        if source is not None:
            return source
        if CAMERA_BACKEND == "picamera2":
            return None

    if CAMERA_BACKEND in ("auto", "opencv", "v4l2"):
        candidate_indices = [CAMERA_INDEX] if CAMERA_INDEX is not None else list(CAMERA_CANDIDATES)

        for index in candidate_indices:
            source, tried = open_camera_by_index(int(index))
            all_tried.extend(tried)
            if source is not None:
                return source

    update_camera_status(
        opened=False,
        backend=None,
        selected_index=None,
        tried=all_tried,
        last_error=(
            "No usable camera frame. The device may be absent, busy, unsupported by OpenCV, "
            "or returning black frames. Try CAMERA_BACKEND=picamera2 for CSI camera or "
            "CAMERA_INDEX=1 for USB camera."
        ),
    )
    return None



def placeholder_frame(message: str):
    frame = np.full((480, 640, 3), 245, dtype=np.uint8)
    cv2.putText(
        frame,
        message,
        (32, 240),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )
    return frame


def video_stream_generator():
    cap = open_first_available_camera()
    black_frame_count = 0

    if cap is None:
        while True:
            status = get_camera_status_snapshot()
            frame = placeholder_frame("Camera frame unavailable")
            cv2.putText(
                frame,
                str(status.get("last_error", ""))[:68],
                (32, 285),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                "Check /api/camera_status",
                (32, 325),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
            ok, buffer = cv2.imencode(".jpg", frame)
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
            time.sleep(0.5)

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            update_camera_status(
                opened=False,
                last_error="Camera frame read failed after successful open. Reopening camera...",
            )
            cap.release()
            time.sleep(0.3)
            cap = open_first_available_camera()
            black_frame_count = 0
            if cap is None:
                frame = placeholder_frame("Camera reconnect failed")
            else:
                continue
        elif is_black_frame(frame):
            black_frame_count += 1
            update_camera_status(
                opened=True,
                last_error="Camera is returning black frames.",
                black_frame_count=black_frame_count,
            )

            if black_frame_count >= BLACK_FRAME_RETRY_LIMIT:
                cap.release()
                time.sleep(0.3)
                cap = open_first_available_camera()
                black_frame_count = 0
                if cap is None:
                    frame = placeholder_frame("Black camera frames")
                else:
                    continue
            else:
                # Show diagnostic overlay instead of a plain black screen.
                cv2.putText(
                    frame,
                    "Black camera frame detected",
                    (24, 46),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    "Check lens cap, lighting, backend, or /api/camera_status",
                    (24, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
        else:
            black_frame_count = 0
            update_camera_status(opened=True, last_error=None, black_frame_count=0)
            frame = annotate_frame(frame)

        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            continue

        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"


def clean_marker_ids(marker_ids_raw) -> Set[int]:
    clean_ids: Set[int] = set()
    if not isinstance(marker_ids_raw, list):
        return clean_ids

    for value in marker_ids_raw:
        try:
            clean_ids.add(int(value))
        except Exception:  # noqa: BLE001
            pass

    return clean_ids


def clean_path_cells(path_raw) -> List[Dict[str, int]]:
    clean_path: List[Dict[str, int]] = []
    if not isinstance(path_raw, list):
        return clean_path

    for cell in path_raw:
        try:
            if isinstance(cell, dict):
                clean_path.append({"row": int(cell["row"]), "col": int(cell["col"])})
            elif isinstance(cell, list) and len(cell) == 2:
                clean_path.append({"row": int(cell[0]), "col": int(cell[1])})
        except Exception:  # noqa: BLE001
            continue

    return clean_path




def load_grid_marker_map() -> Dict[str, object]:
    """
    Load grid_marker_map.json.

    Expected format:
    {
      "rows": 5,
      "cols": 5,
      "grid": [
        [0, 1, 2, 3, 4],
        ...
      ]
    }
    """
    if not GRID_MARKER_MAP_FILE.exists():
        return {
            "loaded": False,
            "error": f"Missing file: {GRID_MARKER_MAP_FILE}",
            "rows": 0,
            "cols": 0,
            "grid": [],
        }

    try:
        data = json.loads(GRID_MARKER_MAP_FILE.read_text(encoding="utf-8"))
        grid = data.get("grid", [])
        rows = int(data.get("rows", len(grid)))
        cols = int(data.get("cols", len(grid[0]) if grid else 0))

        if not isinstance(grid, list) or not grid:
            raise ValueError("grid must be a non-empty 2D list")

        normalized_grid: List[List[int]] = []
        for row in grid:
            if not isinstance(row, list):
                raise ValueError("each grid row must be a list")
            normalized_grid.append([int(value) for value in row])

        return {
            "loaded": True,
            "error": None,
            "rows": rows,
            "cols": cols,
            "grid": normalized_grid,
            "source_file": str(GRID_MARKER_MAP_FILE.name),
            "coordinate_rule": data.get(
                "coordinate_rule",
                "grid[row][col], where row=0 is top and col=0 is left",
            ),
            "description": data.get("description", ""),
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "loaded": False,
            "error": repr(exc),
            "rows": 0,
            "cols": 0,
            "grid": [],
            "source_file": str(GRID_MARKER_MAP_FILE.name),
        }


def marker_id_for_cell(row: int, col: int):
    mapping = load_grid_marker_map()
    if not mapping.get("loaded"):
        return None

    grid = mapping.get("grid", [])
    try:
        return int(grid[row][col])
    except Exception:  # noqa: BLE001
        return None


def marker_ids_from_path_cells(path_cells: List[Dict[str, int]]) -> Set[int]:
    marker_ids: Set[int] = set()

    for cell in path_cells:
        try:
            marker_id = marker_id_for_cell(int(cell["row"]), int(cell["col"]))
        except Exception:  # noqa: BLE001
            marker_id = None

        if marker_id is not None:
            marker_ids.add(marker_id)

    return marker_ids


# ============================================================
# Flask app
# ============================================================

app = Flask(__name__)
car = CarController()


@app.after_request
def add_cors_headers(response):
    # 파일로 직접 연 Web.html에서 테스트할 때도 fetch가 막히지 않게 함.
    # 실제 운용은 Flask가 제공하는 http://라즈베리파이IP:5000 페이지에서 하는 것이 가장 안정적입니다.
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return response


@app.route("/")
def web():
    return render_template("web.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        video_stream_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/drive_state", methods=["POST", "OPTIONS"])
def drive_state():
    if request.method == "OPTIONS":
        return jsonify({"ok": True})

    data = request.get_json(silent=True) or {}
    result = car.apply_keyboard_state(data)
    return jsonify({"ok": True, **result, "state": car.status()})


@app.route("/api/neutral", methods=["POST", "GET", "OPTIONS"])
@app.route("/emergency_stop", methods=["POST", "GET", "OPTIONS"])
def neutral():
    if request.method == "OPTIONS":
        return jsonify({"ok": True})

    result = car.neutral()
    return jsonify({"ok": True, **result, "state": car.status()})


@app.route("/api/route_markers", methods=["POST", "OPTIONS"])
@app.route("/route", methods=["POST", "OPTIONS"])  # backward-compatible alias
def route_markers():
    if request.method == "OPTIONS":
        return jsonify({"ok": True})

    data = request.get_json(silent=True) or {}
    clean_path = clean_path_cells(data.get("path", []))

    # Prefer marker_ids sent by the browser. If absent or empty, compute from grid_marker_map.json.
    clean_ids = clean_marker_ids(data.get("marker_ids", []))
    if not clean_ids:
        clean_ids = marker_ids_from_path_cells(clean_path)

    grid_map_status = load_grid_marker_map()

    with route_lock:
        optimal_route_marker_ids.clear()
        optimal_route_marker_ids.update(clean_ids)
        latest_route_cells.clear()
        latest_route_cells.extend(clean_path)

    return jsonify(
        {
            "ok": True,
            "mode": "marker_highlight_only",
            "message": "Route marker IDs registered from grid_marker_map.json. Autonomous driving is disabled.",
            "marker_ids": sorted(clean_ids),
            "path": clean_path,
            "grid_marker_map": {
                "loaded": grid_map_status.get("loaded", False),
                "source_file": grid_map_status.get("source_file"),
                "error": grid_map_status.get("error"),
            },
        }
    )




@app.route("/api/grid_marker_map")
@app.route("/grid_marker_map.json")
def grid_marker_map_api():
    mapping = load_grid_marker_map()
    http_status = 200 if mapping.get("loaded") else 500
    return jsonify(mapping), http_status


@app.route("/api/status")
@app.route("/status")  # backward-compatible alias
def status():
    with route_lock:
        route_ids = sorted(optimal_route_marker_ids)
        route_cells = list(latest_route_cells)

    return jsonify(
        {
            "ok": True,
            "mode": "manual_keyboard_control",
            "autonomous_driving": False,
            "car": car.status(),
            "route_marker_ids": route_ids,
            "route_cells": route_cells,
            "aruco_dictionary": ARUCO_DICT_NAME,
            "aruco_available": ARUCO_DICT is not None,
            "grid_marker_map": {
                "loaded": load_grid_marker_map().get("loaded", False),
                "source_file": load_grid_marker_map().get("source_file"),
                "error": load_grid_marker_map().get("error"),
            },
            "camera": {
                "index": CAMERA_INDEX,
                "width": CAMERA_WIDTH,
                "height": CAMERA_HEIGHT,
            },
        }
    )


if __name__ == "__main__":
    # 같은 네트워크/핫스팟에서 접속하려면 0.0.0.0 유지.
    app.run(host="0.0.0.0", port=5000, threaded=True)
