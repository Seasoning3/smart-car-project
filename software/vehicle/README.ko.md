# README.ko.md

이 README는 `raspberry_pi_car_server.py` 서버를 라즈베리파이에서 처음부터 실행하기 위한 전체 셋업 절차입니다.  
현재 서버는 자율주행을 수행하지 않고, 웹 기반 지도 표시·경로 마커 하이라이트·실시간 카메라·WASD 수동 조작만 담당합니다.

## 1. 최종 파일 구조

라즈베리파이에 아래 구조로 파일을 배치합니다.

```text
smart_vehicle/
├─ raspberry_pi_car_server.py
├─ grid_marker_map.json
├─ web.html
└─ README.md
```

각 파일 역할은 다음과 같습니다.

| File | Role |
|------------------------------|---------------------------------------------------------|
| `raspberry_pi_car_server.py` | Flask 서버, PCA9685 PWM 제어, 카메라 스트리밍, ArUco 인식 |
| `grid_marker_map.json` | 5×5 격자 좌표와 실제 ArUco marker ID 매핑 |
| `web.html` | 웹 UI |
| `README.md` | 서버 설치 및 실행 설명 |

## 2. 하드웨어 연결 기준

현재 서버 코드는 다음 연결을 기준으로 합니다.

| Device | Connection |
|------------|-------------------|
| ESC signal | PCA9685 channel 0 |
| servo signal | PCA9685 channel 14 |
| PCA9685 SDA | Raspberry Pi SDA |
| PCA9685 SCL | Raspberry Pi SCL |
| PCA9685 GND | Raspberry Pi GND |
| Servo/ESC signal GND | Raspberry Pi/PCA9685 GND와 공통 접지 |
| Camera | CSI camera |

서버 코드 상단 기본값은 다음과 같습니다.

```python
ESC_CHANNEL = 0
SERVO_CHANNEL = 14
PWM_FREQ = 50
```

## 3. Raspberry Pi OS 기본 준비

라즈베리파이를 켠 뒤 터미널에서 먼저 업데이트합니다.

```bash
sudo apt update
sudo apt upgrade -y
```

필수 도구를 설치합니다.

```bash
sudo apt install -y python3-pip python3-venv git i2c-tools v4l-utils
```

## 4. I2C 활성화

PCA9685를 사용하려면 I2C가 켜져 있어야 합니다.

```bash
sudo raspi-config
```

아래 순서로 들어갑니다.

```text
Interface Options
→ I2C
→ Enable
```

재부팅합니다.

```bash
sudo reboot
```

재부팅 후 PCA9685가 잡히는지 확인합니다.

```bash
i2cdetect -y 1
```

정상이라면 보통 `0x40` 주소가 보입니다.

```text
     0 1 2 3 4 5 6 7 8 9 a b c d e f
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

`40`이 보이지 않으면 다음을 확인합니다.

- PCA9685 VCC 연결
- PCA9685 GND 연결
- SDA/SCL 배선
- I2C 활성화 여부
- 라즈베리파이와 PCA9685의 공통 GND

## 5. 카메라 활성화 및 확인

라즈베리파이 카메라 모듈을 사용하는 경우 카메라가 인식되는지 확인합니다.

```bash
rpicam-hello --list-cameras
```

위 명령이 없으면 다음을 시도합니다.

```bash
libcamera-hello --list-cameras
```

카메라 사진 테스트:

```bash
rpicam-still -o test.jpg
```

또는:

```bash
libcamera-still -o test.jpg
```

CSI 카메라가 여기서는 잡히는데 웹에서 검은 화면이 나오면 서버 실행 시 Picamera2 backend를 강제합니다.

```bash
CAMERA_BACKEND=picamera2 python3 raspberry_pi_car_server.py
```

## 6. 프로젝트 폴더 만들기

홈 디렉터리에 서버 폴더를 만듭니다.

```bash
cd ~
mkdir -p smart_vehicle
cd smart_vehicle
```

아래 파일을 같은 구조로 복사합니다.

```text
smart_vehicle/
├─ raspberry_pi_car_server.py
├─ grid_marker_map.json
└─ web.html
```

## 7. Python 가상환경 만들기

라즈베리파이에서 프로젝트 폴더로 이동합니다.

```bash
cd ~/smart_vehicle
```

가상환경을 만듭니다.

```bash
python3 -m venv --system-site-packages smartcar
```

가상환경에 들어갑니다.

```bash
source smartcar/bin/activate
```

프롬프트 앞에 `(smartcar)`가 보이면 가상환경에 들어간 상태입니다.

```text
(smartcar) pi@raspberrypi:~/smart_vehicle $
```

가상환경에서 나가려면 다음을 입력합니다.

```bash
deactivate
```

## 8. Python 패키지 설치

가상환경 안에서 pip를 업데이트합니다.

```bash
python -m pip install --upgrade pip setuptools wheel
```

서버 기본 패키지를 설치합니다.

```bash
pip install flask numpy adafruit-circuitpython-pca9685
```

OpenCV는 ArUco 기능 때문에 `opencv-contrib-python`이 필요합니다.

```bash
pip install opencv-contrib-python
```

라즈베리파이 3에서 위 설치가 너무 오래 걸리거나 실패하면, 먼저 현재 OpenCV에 ArUco가 포함되어 있는지 확인합니다.

```bash
python3 - <<'PY'
import cv2
print("OpenCV:", cv2.__version__)
print("Has aruco:", hasattr(cv2, "aruco"))
PY
```

`Has aruco: True`가 나오면 추가 설치 없이 사용할 수 있습니다.

CSI 카메라를 Picamera2로 쓸 경우 보통 apt 패키지가 필요합니다.

```bash
sudo apt install -y python3-picamera2
```

가상환경을 `--system-site-packages`로 만들었기 때문에 apt로 설치된 `picamera2`를 가상환경 안에서도 사용할 수 있습니다.

## 9. 서버 설정값 확인

`raspberry_pi_car_server.py` 상단에서 다음 값을 실제 차량에 맞게 확인합니다.

```python
ESC_CHANNEL = 0
SERVO_CHANNEL = 14
PWM_FREQ = 50

ESC_NEUTRAL_US = 1500
ESC_FORWARD_US = 1641
ESC_REVERSE_US = 1453

SERVO_CENTER_ANGLE = 20
SERVO_LEFT_ANGLE = 32
SERVO_RIGHT_ANGLE = 8
SERVO_CENTER_US = 1500
```

중요한 기준은 다음입니다.

| Setting | Meaning |
|-------------------------|----------|
| `ESC_NEUTRAL_US = 1500` | ESC 중립 |
| `ESC_FORWARD_US` | W 입력 시 전진 PWM |
| `ESC_REVERSE_US` | S 입력 시 후진 PWM |
| `SERVO_CENTER_US = 1500` | 조향 중립 PWM, 유지 |
| `SERVO_LEFT_ANGLE` | A 입력 시 좌측 조향 각도 |
| `SERVO_RIGHT_ANGLE` | D 입력 시 우측 조향 각도 |

현재 조향 로직은 다음과 같습니다.

```text
A 입력 → SERVO_LEFT_ANGLE
D 입력 → SERVO_RIGHT_ANGLE
A/D 입력 없음 → SERVO_CENTER_US = 1500
A와 D 동시 입력 → SERVO_CENTER_US = 1500
```

즉, `A` 또는 `D`로 조작하지 않으면 서보는 항상 중립 각도입니다.

## 10. grid_marker_map.json 설정

마커 매핑은 HTML 내부가 아니라 `grid_marker_map.json`에서 관리합니다.

기본 형식:

```json
{
  "description": "5x5 grid-cell to ArUco marker ID mapping. Edit this file only when physical marker placement changes.",
  "coordinate_rule": "grid[row][col], where row=0 is top and col=0 is left",
  "rows": 5,
  "cols": 5,
  "grid": [
    [0, 1, 2, 3, 4],
    [5, 6, 7, 8, 9],
    [10, 11, 12, 13, 14],
    [15, 16, 17, 18, 19],
    [20, 21, 22, 23, 24]
  ]
}
```

좌표 규칙:

```text
grid[row][col]
row = 위에서 아래, 0부터 시작
col = 왼쪽에서 오른쪽, 0부터 시작
```

예를 들어 왼쪽 위 칸 중앙의 실제 ArUco ID가 17이면 첫 번째 값을 17로 바꿉니다.

```json
"grid": [
  [17, 1, 2, 3, 4],
  [5, 6, 7, 8, 9],
  [10, 11, 12, 13, 14],
  [15, 16, 17, 18, 19],
  [20, 21, 22, 23, 24]
]
```

단, 실제 운용에서는 같은 ArUco ID를 여러 칸에 중복 배치하지 않는 것이 좋습니다.

## 11. 서버 실행

프로젝트 폴더로 이동합니다.

```bash
cd ~/smart_vehicle
```

가상환경에 들어갑니다.

```bash
source smartcar/bin/activate
```

기본 실행:

```bash
python3 raspberry_pi_car_server.py
```

CSI 카메라를 강제할 때:

```bash
CAMERA_BACKEND=picamera2 python3 raspberry_pi_car_server.py
```

서버가 정상 실행되면 보통 다음과 비슷하게 표시됩니다.

```text
* Running on http://0.0.0.0:5000
```

## 12. 웹 접속

PC와 라즈베리파이가 같은 Wi-Fi 또는 같은 핫스팟에 연결되어 있어야 합니다.

라즈베리파이 IP 확인:

```bash
hostname -I
```

예를 들어 IP가 `192.168.0.50`이면 PC 브라우저에서 다음 주소로 접속합니다.

```text
http://192.168.0.50:5000
```

웹페이지가 라즈베리파이에서 직접 열렸다면 `현재 주소 적용` 버튼으로 서버 주소를 자동 적용할 수 있습니다.

## 13. 웹 조작 방법

### 13.1 지도 조작

| Tool | Role |
|-------|-------------------------|
| 구조물 | 차량이 지나갈 수 없는 칸 |
| 출발점 | 경로 시작 위치 |
| 도착점 / 경유점 | 방문해야 할 지점 |

절차:

1. 구조물을 배치합니다.
2. 출발점을 배치합니다.
3. 도착점 또는 경유점을 배치합니다.
4. `경로 계산`을 누릅니다.
5. `경로 전송`을 누릅니다.
6. 카메라 화면에서 최적 경로에 해당하는 ArUco ID가 하이라이트되는지 확인합니다.

경로 전송은 카메라 화면에서 최적 경로 마커를 강조하기 위한 용도입니다.

### 13.2 차량 수동 조작

| Key | Action |
|---|------|
| W | 전진 |
| S | 후진 |
| A | 좌측 조향 |
| D | 우측 조향 |

키를 떼면 해당 방향 상태가 `false`로 서버에 전송됩니다.  
W/S가 모두 `false`이면 ESC는 1500us입니다.  
A/D가 모두 `false`이면 서보는 1500us입니다.

## 14. 서버 API

| API | Method | Role |
|-----|-----|-------|
| `/` | GET | 웹 UI |
| `/video_feed` | GET | MJPEG 카메라 스트림 |
| `/api/drive_state` | POST | WASD 상태 수신 후 ESC/서보 PWM 갱신 |
| `/api/route_markers` | POST | 최적 경로 marker ID 등록 |
| `/api/grid_marker_map` | GET | `grid_marker_map.json` 로드 |
| `/api/status` | GET | 서버, PWM, 카메라, 마커 상태 확인 |
| `/api/camera_status` | GET | 카메라 backend, index, black frame 상태 확인 |
| `/api/neutral` | POST | 내부 안전용 중립 PWM |
| `/emergency_stop` | GET/POST | 내부 안전용 중립 PWM |

웹 UI에는 중립 전송 버튼을 두지 않습니다. `/api/neutral`과 `/emergency_stop`은 안전용으로 서버에만 남아 있습니다.

## 15. 상태 확인

전체 상태:

```text
http://라즈베리파이IP:5000/api/status
```

카메라 상태:

```text
http://라즈베리파이IP:5000/api/camera_status
```

마커 매핑 상태:

```text
http://라즈베리파이IP:5000/api/grid_marker_map
```

카메라 상태에서 확인할 주요 항목:

| Field | Meaning |
|-----------|---------------------|
| `backend` | 사용 중인 카메라 방식 |
| `selected_index` | OpenCV 카메라 번호 |
| `opened` | 카메라 열림 여부 |
| `black_frame_count` | 검은 프레임 감지 횟수 |
| `last_error` | 마지막 카메라 오류 |

## 16. 자주 발생하는 문제

### 16.1 웹페이지는 열리는데 차량이 움직이지 않음

확인 순서:

```bash
i2cdetect -y 1
```

`40`이 보이지 않으면 PCA9685 연결 또는 I2C 설정 문제입니다.

서버 상태 확인:

```text
http://라즈베리파이IP:5000/api/status
```

`hardware_available`이 `false`이면 Python에서 PCA9685 관련 라이브러리 또는 I2C 접근에 실패한 것입니다.

패키지 재설치:

```bash
pip install adafruit-circuitpython-pca9685
```

### 16.2 W/S를 눌러도 모터가 돌지 않음

확인할 것:

- ESC signal이 PCA9685 channel 0에 꽂혀 있는지
- ESC와 PCA9685/Raspberry Pi GND가 공통인지
- ESC가 arming 되었는지
- 배터리 전압이 충분한지
- `ESC_FORWARD_US` 값이 너무 낮지 않은지

전진 PWM을 조금 올려 테스트할 수 있습니다.

```python
ESC_FORWARD_US = 1650
```

너무 크게 올리면 갑자기 튈 수 있으므로 작은 폭으로만 조정합니다.

### 16.3 A/D를 누르지 않았는데 서보가 한쪽으로 돌아감

현재 코드상 A/D 입력이 없으면 서버는 반드시 `SERVO_CENTER_US = 1500`을 출력합니다. 그래도 한쪽으로 돌아가면 원인은 코드보다 하드웨어 보정 문제일 가능성이 큽니다.

확인할 것:

- 서보혼이 기계적으로 중앙에 맞게 꽂혔는지
- 실제 서보 중앙이 1500us인지
- PCA9685 GND와 서보 전원 GND가 공통인지
- 서보 전원이 충분한지

기계적 중앙이 다르면 서보혼을 다시 장착하는 것이 먼저입니다.

### 16.4 카메라 화면에 `Camera frame unavailable` 또는 검은 화면이 나옴

상태 확인:

```text
http://라즈베리파이IP:5000/api/camera_status
```

CSI 카메라라면:

```bash
CAMERA_BACKEND=picamera2 python3 raspberry_pi_car_server.py
```

OS 장치 확인:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

CSI 카메라 확인:

```bash
rpicam-hello --list-cameras
libcamera-hello --list-cameras
```

### 16.5 ArUco가 인식되지 않음

확인할 것:

- 서버의 `ARUCO_DICT_NAME`이 실제 생성한 ArUco dictionary와 같은지
- 마커가 너무 작지 않은지
- 조명이 너무 어둡거나 반사가 심하지 않은지
- 카메라 초점이 맞는지
- 마커가 화면에서 충분히 크게 보이는지
- `opencv-contrib-python` 또는 `cv2.aruco`가 설치되어 있는지

확인 명령:

```bash
python3 - <<'PY'
import cv2
print(cv2.__version__)
print(hasattr(cv2, "aruco"))
PY
```

`False`가 나오면 ArUco 모듈이 없는 OpenCV입니다.

### 16.6 경로 하이라이트가 실제 마커와 다름

`grid_marker_map.json`이 실제 보드의 마커 배치와 맞지 않는 상태입니다.

확인할 것:

- 왼쪽 위 칸이 `grid[0][0]`인지
- 오른쪽 아래 칸이 `grid[4][4]`인지
- 실제 부착한 ArUco ID와 JSON 값이 일치하는지
- 같은 ID를 여러 칸에 중복으로 넣지 않았는지

## 17. 서버를 종료하는 방법

터미널에서 서버가 실행 중이면 `Ctrl + C`를 누릅니다.

가상환경에서 나가려면:

```bash
deactivate
```

## 18. 자동 실행이 필요한 경우

부팅 후 서버를 자동 실행하려면 systemd 서비스를 만들 수 있습니다.

서비스 파일 생성:

```bash
sudo nano /etc/systemd/system/carserver.service
```

아래 내용을 붙여 넣습니다. 사용자 이름과 경로가 다르면 수정합니다.

```ini
[Unit]
Description=Manual ArUco Car Server
After=network-online.target
Wants=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/smart_vehicle
Environment=CAMERA_BACKEND=auto
ExecStart=/home/pi/smart_vehicle/smartcar/bin/python /home/pi/smart_vehicle/raspberry_pi_car_server.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

서비스 적용:

```bash
sudo systemctl daemon-reload
sudo systemctl enable carserver.service
sudo systemctl start carserver.service
```

상태 확인:

```bash
sudo systemctl status carserver.service
```

로그 확인:

```bash
journalctl -u carserver.service -f
```

중지:

```bash
sudo systemctl stop carserver.service
```

자동 실행 해제:

```bash
sudo systemctl disable carserver.service
```

## 19. 최종 실행 체크리스트

아래 순서대로 확인합니다.

```text
1. I2C enabled
2. i2cdetect -y 1에서 0x40 확인
3. 카메라가 rpicam/libcamera 또는 /dev/video*에서 확인됨
4. Python 가상환경 생성 완료
5. Flask, OpenCV, PCA9685 라이브러리 설치 완료
6. grid_marker_map.json이 실제 보드 마커 ID와 일치
7. 서버 실행
8. PC에서 http://라즈베리파이IP:5000 접속
9. /api/status 확인
10. /api/camera_status 확인
11. W/A/S/D 조작 테스트
12. 경로 계산 후 카메라 화면에서 route marker highlight 확인
```
