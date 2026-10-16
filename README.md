# lerobot_robot_piper

[![Release](https://img.shields.io/github/v/release/WeGo-Robotics/lerobot_robot_piper)](https://github.com/WeGo-Robotics/lerobot_robot_piper/releases)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](https://www.apache.org/licenses/LICENSE-2.0)
![Python](https://img.shields.io/badge/python-%E2%89%A53.10-blue)

LeRobot plugin for the **Agilex Piper** 7-DOF robotic arm. Provides follower (robot) and leader (teleoperator) interfaces that integrate directly with the [LeRobot](https://github.com/huggingface/lerobot) framework for teleoperation, dataset recording, and autonomous policy deployment.

> 한국어 안내는 [아래](#한국어-korean)에 있습니다.

```bash
# The whole setup, start to finish:
git clone https://github.com/WeGo-Robotics/lerobot_robot_piper.git
cd lerobot_robot_piper && pip install -e .
piper-doctor                                  # verify install & hardware
piper-setup                                   # name your arms (once)
sudo bash scripts/install_can_persist.sh      # survive reboots (once)
piper-teleop                                  # go
```

---

## Features

- **Leader-Follower Teleoperation**: Mirror a leader arm's movements onto a follower arm in real time
- **Dataset Recording**: Collect episodes (joint positions + camera frames) for imitation learning
- **CAN Bus Communication**: Direct hardware control via `piper_sdk`
- **Safety Limits**: Configurable `max_relative_target` to cap per-step joint movement
- **Camera Integration**: Attach multiple USB cameras as observations on the follower
- **GUI Tools**: Tkinter-based UIs for setup (`piper-setup`), direct control (`piper-ui`), teleoperation monitoring (`piper-teleop`), and zero-point calibration (`piper-calibrate`)
- **Boot-persistent CAN setup**: One-time install script keeps canonical CAN names across reboots and hotplugs

---

## Installation

**From PyPI**:

```bash
pip install lerobot_robot_wego_piper
```

> The PyPI name is `lerobot_robot_wego_piper` — the package named `lerobot_robot_piper` on PyPI is an **unrelated third-party project**. The import name stays `lerobot_robot_piper` either way.

**From source** — needed for the `scripts/` (CAN persistence) and development:

```bash
git clone https://github.com/WeGo-Robotics/lerobot_robot_piper.git
cd lerobot_robot_piper
pip install -e .
```

Wheels are also attached to each [GitHub Release](https://github.com/WeGo-Robotics/lerobot_robot_piper/releases). All dependencies, including [`piper_sdk`](https://pypi.org/project/piper_sdk/), resolve automatically from PyPI.

Verify the installation (and later, the hardware) with:

```bash
piper-doctor
```

**Requirements**: Python >= 3.10, LeRobot >= 0.3.0, Linux with SocketCAN (`iproute2`, `ethtool`, `can-utils`), CAN-USB interface (gs_usb) connected to the Piper arm.

```bash
sudo apt update && sudo apt install iproute2 ethtool can-utils
```

**Without SocketCAN (macOS, Windows) — experimental.** The follower and leader can open the
same gs_usb adapter through libusb instead of SocketCAN. Set `can_interface` and pass the
adapter index as `port`. The setup scripts and GUI tools below stay Linux-only. Not yet
tried with a real arm; a missing adapter or an unpowered arm fails `connect()` with
`No frame from the arm ...` instead of looking connected.

```bash
brew install libusb                      # macOS; Windows needs a libusb driver for the adapter
pip install "python-can[gs_usb]" "piper_sdk>=0.6.1"

python -m lerobot.teleoperate \
    --robot.type=piper_follower --robot.port=0 --robot.can_interface=gs_usb \
    --teleop.type=piper_leader --teleop.port=1 --teleop.can_interface=gs_usb
```

`can_interface` takes any python-can interface name, e.g. `slcan` with `port` set to the
serial device of a serial CAN module.

---

## Hardware Setup

### First-time setup (recommended path)

```bash
# 1. Assign canonical names to your arms (GUI wizard, see below)
piper-setup

# 2. Make the CAN setup persistent across reboots, hotplugs, and port swaps
#    (keep the arms plugged in — adapter serials are read live)
sudo bash scripts/install_can_persist.sh
```

After this, `can_leader1`, `can_follower1`, … come back up automatically on every boot and every re-plug.

**Why serial pinning matters**: plain `can0`/`can1` are kernel enumeration order — swap two adapters between USB ports and the arms silently swap names, so a leader can receive follower commands. The installer generates udev rules pinned to each adapter's USB **serial**, so the name follows the arm no matter which port it's plugged into. Re-run `piper-setup` + the installer only when an arm is rewired to a different adapter.

To see which adapter is which (e.g. before rewiring):

```bash
python3 scripts/list_can_adapters.py          # serial <-> interface table
python3 scripts/list_can_adapters.py --watch  # move an arm by hand to identify it
```

### `piper-doctor` — When something doesn't work

```bash
piper-doctor
```

Checks the whole chain in dependency order and prints a fix for every failure:

1. Python packages & LeRobot plugin registration (`piper_follower` / `piper_leader` types)
2. CAN adapter hardware on the USB bus
3. CAN interface state vs `~/piper_config.json` (names, UP, bitrate)
4. Arm response — passive RX counters, then an active firmware query
5. Boot persistence (udev rules + `piper-can-up.service`)

Read-only — never changes interface state or sends arm commands. Exit code 0 = healthy, 1 = problems found.

### Manual CAN activation (single arm, no wizard)

```bash
# Activate the only plugged-in CAN module as can0 at 1 Mbps
bash scripts/can_activate.sh can0 1000000

# Multiple modules plugged in: disambiguate by USB address
sudo ethtool -i can0 | grep bus-info
bash scripts/can_activate.sh can0 1000000 1-2:1.0
```

Or fully by hand:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

The `piper-ui` and `piper-teleop` GUIs can also detect and initialize CAN interfaces.

### Scripts

| Script | Purpose |
|--------|---------|
| `scripts/can_activate.sh` | Activate a single CAN interface (name, bitrate, optional USB address) |
| `scripts/can_restore.sh` | Bring up all arms from `~/piper_config.json` (rename if needed, bitrate, link up) |
| `scripts/install_can_persist.sh` | Generate serial-pinned udev `NAME=` rules + install boot/hotplug bring-up service; `uninstall` to remove |
| `scripts/list_can_adapters.py` | Show adapter serial ↔ interface table; `--watch` identifies an arm by moving it |

### Wiring

| Arm | Interface |
|-----|-----------|
| Follower (robot) | e.g., `can_follower1` (or `can0`) |
| Leader (human input) | e.g., `can_leader1` (or `can1`) |

---

## GUI Tools

![piper-ui](asset/piper-ui.png)
![piper-teleop](asset/piper-teleop.png)
![piper-setup](asset/piper-setup.png)

### `piper-setup` — Multi-Arm Setup Wizard

Run this first when setting up multiple arms.

```bash
piper-setup
```

4-step wizard for configuring up to 4 arms:

1. **Scan** — detect all CAN ports, initialize, connect, and read firmware version from each arm
2. **Config** — select arm configuration (1 Leader/1 Follower, 2 Followers, 2 Leaders, 2 Leaders/2 Followers)
3. **Identify** — click `Find` for each slot, then physically move the target arm more than 45° to assign it; use `Set Role` to apply leader/follower mode and `Torque Off` to release the arm
4. **Finalize** — rename CAN ports to canonical names (`can_leader1`, `can_follower1`, etc.) and save `~/piper_config.json`

### `piper-ui` — Direct Control

```bash
piper-ui
```

- **CAN Setup**: detect interfaces, view firmware version and USB port per interface, select role (leader/follower), initialize bitrate
- **Connect**: select a port via radio button, then click Connect — sliders automatically sync to the arm's current joint positions
- **Torque**: toggle torque on/off; sliders and Parking are disabled while torque is off
- **Set Leader / Set Follower**: apply `MasterSlaveConfig` to switch the arm between teaching input mode and motion output mode
- **Parking**: move the arm to the home position (joint values: 0, −100, 100, 0, 35, 0, 0)
- **Joints**: drag sliders to move joints in real time (only active when torque is on); live position readout next to each slider

### `piper-teleop` — Teleoperation Monitor

```bash
piper-teleop
```

- Detect and initialize CAN interfaces
- Launch pre-configured teleoperation/recording commands
- Real-time joint position bars for both arms (leader & follower)
- Follower arm status (enable state, motion status, error codes)

### `piper-calibrate` — Joint Zero-Point Calibration

```bash
piper-calibrate
```

1. Connect to a Piper arm via CAN (teaching/free mode recommended)
2. Manually move each joint to its zero (home) position
3. Click **Set Zero** — sends `JointConfig(set_zero=0xAE)` (gripper: `GripperCtrl(..., 0xAE)`), which writes the zero-point into the **motor's flash memory**; it persists across power cycles
4. The response flag (`is_set_zero_successfully`) confirms each write; a guard asks before zeroing a joint that sits more than 5° away from raw zero
5. **Clear Errors** sends `JointConfig(clear_err=0xAE)` to all joints

#### Why calibration lives in the motor, not in software

The `MotorCalibration` table in [piper_follower.py](lerobot_robot_piper/piper_follower.py) is intentionally **identical for every arm** — it is not a per-arm correction. Its `range_min`/`range_max` are the *model specification* of each joint's travel (in 0.001° units, e.g. joint 1 = ±150000 = ±150°), used only to map absolute angles to the normalized `[-100, 100]` action/observation space. `homing_offset` is 0 and LeRobot's file-based calibration (`_load_calibration`/`_save_calibration`) is a deliberate no-op.

Per-arm error is absorbed **one layer below**: the Piper SDK reports absolute angles already corrected by the zero-point stored in each motor's flash. So:

- **Arm zeroed correctly** → raw angles are true angles → identical normalization across arms → per-arm error does *not* leak into recorded datasets.
- **Arm zeroed poorly** → the offset rides along in every recorded value. The fix is `piper-calibrate`, once per arm.
- **Leader–follower pairs**: zero both arms in the *same physical reference pose* so their coordinate frames agree.

Limitations (honest ones): zeroing accuracy depends on how precisely you place the physical reference pose (use a jig if you need repeatability), and scale-type errors (link-length deviation, gear backlash) are not corrected by any zero-point — neither here nor by LeRobot's file calibration.

### `camera_check.py` — Camera Grid Viewer

```bash
python camera_check.py
```

Probes `/dev/video0` … `/dev/video17`, opens every working camera, and shows all streams in one grid — useful for finding camera indices before recording.

---

## Quick Start

### 1. Python API

```python
from lerobot_robot_piper import PiperFollowerConfig, PiperLeaderConfig, PiperFollower, PiperLeader

follower_cfg = PiperFollowerConfig(port="can_follower1")
leader_cfg   = PiperLeaderConfig(port="can_leader1")

follower = PiperFollower(follower_cfg)
leader   = PiperLeader(leader_cfg)

follower.connect()
leader.connect()

try:
    while True:
        action = leader.get_action()        # read leader joint positions
        follower.send_action(action)        # mirror to follower
        obs = follower.get_observation()    # read follower state + cameras
finally:
    follower.disconnect()
    leader.disconnect()
```

### 2. LeRobot CLI

```bash
# Teleoperate
python -m lerobot.teleoperate \
    --robot.type=piper_follower \
    --robot.port=can_follower1 \
    --teleop.type=piper_leader \
    --teleop.port=can_leader1

# Record a dataset
python -m lerobot.record \
    --robot.type=piper_follower \
    --robot.port=can_follower1 \
    --teleop.type=piper_leader \
    --teleop.port=can_leader1 \
    --dataset-id=my_piper_dataset
```

---

## Configuration

### PiperFollowerConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | `str` | `"can0"` | CAN port for the follower arm |
| `disable_torque_on_disconnect` | `bool` | `True` | Disable motors when disconnecting |
| `cameras` | `dict[str, CameraConfig]` | `{}` | Named cameras for observation |
| `max_relative_target` | `float \| dict \| None` | `None` | Max per-step joint movement (safety limit) |
| `can_interface` | `str` | `"socketcan"` | python-can interface. `gs_usb` or `slcan` without SocketCAN |
| `can_bitrate` | `int` | `1000000` | CAN bitrate for non-SocketCAN interfaces |

### PiperLeaderConfig

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | `str` | `"can1"` | CAN port for the leader arm |
| `gripper_open_pos` | `float` | `50.0` | Position value representing gripper fully open |
| `can_interface` | `str` | `"socketcan"` | Same as the follower |
| `can_bitrate` | `int` | `1000000` | Same as the follower |

---

## Motor Configuration

The Piper arm has 7 joints. Normalization maps raw encoder counts to the ranges shown below.

| Joint | Model | Normalized Range | Physical Range |
|-------|-------|-----------------|----------------|
| Joint 1 | AGILEX-M | −100 to +100 | ±150° |
| Joint 2 | AGILEX-M | −100 to +100 | 0–180° |
| Joint 3 | AGILEX-M | −100 to +100 | −170–0° |
| Joint 4 | AGILEX-S | −100 to +100 | ±100° |
| Joint 5 | AGILEX-S | −100 to +100 | ±65° |
| Joint 6 | AGILEX-S | −100 to +100 | ±100–130° |
| Gripper | AGILEX-S | 0 to 100 | 0–68° |

The parking (home) position in normalized values: `0, −100, 100, 0, 35, 0, 0`.

---

## Project Structure

```
lerobot_robot_piper/
├── config_piper.py          # PiperFollowerConfig
├── config_piper_leader.py   # PiperLeaderConfig
├── piper_follower.py        # PiperFollower (Robot)
├── piper_leader.py          # PiperLeader (Teleoperator)
├── ui.py                    # piper-ui entrypoint
├── teleop_ui.py             # piper-teleop entrypoint
├── arm_setup_ui.py          # piper-setup entrypoint (multi-arm wizard)
├── calibration_ui.py        # piper-calibrate entrypoint
├── doctor.py                # piper-doctor entrypoint (installation checkup)
└── motors/
    ├── piper_motors_bus.py  # CAN bus abstraction
    ├── port_handler.py      # CAN port lifecycle (absorbed from wego_piper)
    └── tables.py            # Motor model tables
lerobot_robot_wego_piper/    # PyPI-name discovery shim (re-exports the package above)
scripts/
├── can_activate.sh          # single-CAN activation
├── can_restore.sh           # bring up arms from ~/piper_config.json
├── install_can_persist.sh   # serial-pinned udev rules + bring-up service
└── list_can_adapters.py     # adapter serial <-> interface, --watch to identify
camera_check.py              # camera grid viewer
```

---

## License

Apache-2.0

---

# 한국어 (Korean)

**Agilex Piper** 7자유도 로봇팔용 LeRobot 플러그인. 팔로워(로봇)와 리더(텔레오퍼레이터) 인터페이스를 제공하며, [LeRobot](https://github.com/huggingface/lerobot) 프레임워크에 바로 통합되어 텔레오퍼레이션·데이터셋 수집·자율 정책 배포에 사용됩니다.

```bash
# 처음부터 끝까지 전체 셋업:
git clone https://github.com/WeGo-Robotics/lerobot_robot_piper.git
cd lerobot_robot_piper && pip install -e .
piper-doctor                                  # 설치·하드웨어 점검
piper-setup                                   # 팔 이름 지정 (1회)
sudo bash scripts/install_can_persist.sh      # 재부팅 영속화 (1회)
piper-teleop                                  # 시작
```

## 주요 기능

- **리더-팔로워 텔레오퍼레이션**: 리더 팔의 움직임을 팔로워 팔에 실시간 미러링
- **데이터셋 수집**: 모방학습용 에피소드(관절 위치 + 카메라 프레임) 기록
- **CAN 버스 통신**: `piper_sdk` 기반 직접 하드웨어 제어
- **안전 제한**: `max_relative_target`으로 스텝당 관절 이동량 상한 설정
- **카메라 연동**: 팔로워 관측에 USB 카메라 다중 연결
- **GUI 도구**: 셋업(`piper-setup`), 직접 제어(`piper-ui`), 텔레옵 모니터(`piper-teleop`), 영점 캘리브레이션(`piper-calibrate`)
- **부팅 영속 CAN 셋업**: 1회 설치로 재부팅·재연결에도 정식 CAN 이름 유지

## 설치

**PyPI에서**:

```bash
pip install lerobot_robot_wego_piper
```

> PyPI 패키지명은 `lerobot_robot_wego_piper`입니다 — PyPI의 `lerobot_robot_piper`는 **무관한 타인의 프로젝트**입니다. 임포트 이름은 어느 쪽으로 설치해도 `lerobot_robot_piper`로 동일합니다.

**소스에서** — `scripts/`(CAN 영속화)와 개발에 필요:

```bash
git clone https://github.com/WeGo-Robotics/lerobot_robot_piper.git
cd lerobot_robot_piper
pip install -e .
```

각 [GitHub Release](https://github.com/WeGo-Robotics/lerobot_robot_piper/releases)에도 wheel이 첨부됩니다. [`piper_sdk`](https://pypi.org/project/piper_sdk/)를 포함한 모든 의존성은 PyPI에서 자동 해결됩니다.

설치(그리고 이후 하드웨어) 확인:

```bash
piper-doctor
```

**요구사항**: Python >= 3.10, LeRobot >= 0.3.0, SocketCAN 지원 Linux(`iproute2`, `ethtool`, `can-utils`), Piper 팔에 연결된 CAN-USB 인터페이스(gs_usb).

```bash
sudo apt update && sudo apt install iproute2 ethtool can-utils
```

**SocketCAN 없이 (macOS, Windows) — 실험 단계.** 팔로워와 리더는 같은 gs_usb 어댑터를
SocketCAN 대신 libusb로 열 수 있습니다. `can_interface`를 정하고 `port`에 어댑터 순번을
넣습니다. 아래 셋업 스크립트와 GUI 도구는 그대로 Linux 전용입니다. 실제 팔로는 아직
시험하지 않았습니다. 어댑터가 없거나 팔 전원이 꺼져 있으면 연결된 것처럼 보이지 않고
`connect()`가 `No frame from the arm ...` 오류를 냅니다.

```bash
brew install libusb                      # macOS. Windows는 어댑터용 libusb 드라이버가 필요합니다
pip install "python-can[gs_usb]" "piper_sdk>=0.6.1"

python -m lerobot.teleoperate \
    --robot.type=piper_follower --robot.port=0 --robot.can_interface=gs_usb \
    --teleop.type=piper_leader --teleop.port=1 --teleop.can_interface=gs_usb
```

`can_interface`에는 python-can 인터페이스 이름을 넣습니다. 예를 들어 직렬 CAN 모듈은
`slcan`으로 두고 `port`에 직렬 장치를 넣습니다.

## 하드웨어 셋업

### 최초 셋업 (권장 경로)

```bash
# 1. 팔에 정식 이름 부여 (GUI 마법사, 아래 참조)
piper-setup

# 2. 재부팅·재연결·포트 교체에도 유지되도록 영속화
#    (팔을 꽂은 상태로 실행 — 어댑터 시리얼을 라이브로 읽음)
sudo bash scripts/install_can_persist.sh
```

이후 `can_leader1`, `can_follower1` 등이 매 부팅·재연결마다 자동으로 올라옵니다.

**시리얼 고정이 중요한 이유**: `can0`/`can1`은 커널 열거 순서일 뿐이라, 어댑터 두 개를 USB 포트만 바꿔 꽂아도 두 팔의 이름이 조용히 뒤바뀝니다 — 리더 팔에 팔로워 명령이 갈 수 있습니다. 설치 스크립트는 각 어댑터의 USB **시리얼**에 고정된 udev 규칙을 생성하므로, 어느 포트에 꽂아도 이름이 팔을 따라갑니다. 팔을 다른 어댑터에 재배선했을 때만 `piper-setup` + 설치 스크립트를 다시 실행하면 됩니다.

어느 어댑터가 어느 것인지 확인(재배선 전 등):

```bash
python3 scripts/list_can_adapters.py          # 시리얼 <-> 인터페이스 표
python3 scripts/list_can_adapters.py --watch  # 팔을 손으로 움직여 식별
```

### `piper-doctor` — 뭔가 안 될 때

```bash
piper-doctor
```

의존 순서대로 전체 체인을 검사하고 실패마다 해결 명령을 출력합니다:

1. 파이썬 패키지 & LeRobot 플러그인 등록 (`piper_follower`/`piper_leader` 타입)
2. USB 버스의 CAN 어댑터 하드웨어
3. CAN 인터페이스 상태 vs `~/piper_config.json` (이름·UP·bitrate)
4. 팔 응답 — 수동 RX 카운터, 이어서 능동 펌웨어 질의
5. 부팅 영속화 (udev 규칙 + `piper-can-up.service`)

읽기 전용 — 인터페이스 상태 변경이나 팔 명령 전송 없음. 종료 코드 0 = 정상, 1 = 문제 발견.

### 수동 CAN 활성화 (단일 팔, 마법사 없이)

```bash
# 유일하게 꽂힌 CAN 모듈을 can0, 1 Mbps로 활성화
bash scripts/can_activate.sh can0 1000000

# 모듈이 여러 개면 USB 주소로 구분
sudo ethtool -i can0 | grep bus-info
bash scripts/can_activate.sh can0 1000000 1-2:1.0
```

완전 수동:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

`piper-ui`, `piper-teleop` GUI에서도 CAN 인터페이스 감지·초기화가 가능합니다.

### 스크립트

| 스크립트 | 용도 |
|--------|---------|
| `scripts/can_activate.sh` | 단일 CAN 인터페이스 활성화 (이름, bitrate, USB 주소 선택) |
| `scripts/can_restore.sh` | `~/piper_config.json` 기준 전체 팔 올리기 (필요 시 rename, bitrate, up) |
| `scripts/install_can_persist.sh` | 시리얼 고정 udev `NAME=` 규칙 생성 + 부팅/핫플러그 서비스 설치; `uninstall`로 제거 |
| `scripts/list_can_adapters.py` | 어댑터 시리얼 ↔ 인터페이스 표; `--watch`로 팔을 움직여 식별 |

### 배선

| 팔 | 인터페이스 |
|-----|-----------|
| 팔로워 (로봇) | 예: `can_follower1` (또는 `can0`) |
| 리더 (사람 입력) | 예: `can_leader1` (또는 `can1`) |

## GUI 도구

### `piper-setup` — 다중 팔 셋업 마법사

여러 팔을 처음 설정할 때 가장 먼저 실행합니다.

```bash
piper-setup
```

최대 4개 팔을 설정하는 4단계 마법사:

1. **Scan** — 모든 CAN 포트 감지·초기화·연결, 각 팔의 펌웨어 버전 읽기
2. **Config** — 팔 구성 선택 (리더1/팔로워1, 팔로워2, 리더2, 리더2/팔로워2)
3. **Identify** — 슬롯마다 `Find` 클릭 후 해당 팔을 45° 이상 크게 움직여 할당; `Set Role`로 리더/팔로워 모드 적용, `Torque Off`로 팔 해제
4. **Finalize** — CAN 포트를 정식 이름(`can_leader1`, `can_follower1` 등)으로 변경하고 `~/piper_config.json` 저장

### `piper-ui` — 직접 제어

```bash
piper-ui
```

- **CAN Setup**: 인터페이스 감지, 인터페이스별 펌웨어·USB 포트 확인, 역할 선택, bitrate 초기화
- **Connect**: 라디오 버튼으로 포트 선택 후 연결 — 슬라이더가 현재 관절 위치에 자동 동기화
- **Torque**: 토크 on/off 토글; off 동안 슬라이더·파킹 비활성화
- **Set Leader / Set Follower**: `MasterSlaveConfig`로 티칭 입력 모드 ↔ 모션 출력 모드 전환
- **Parking**: 홈 포지션으로 이동 (관절값: 0, −100, 100, 0, 35, 0, 0)
- **Joints**: 슬라이더로 실시간 관절 제어 (토크 on일 때만), 슬라이더 옆 실시간 위치 표시

### `piper-teleop` — 텔레오퍼레이션 모니터

```bash
piper-teleop
```

- CAN 인터페이스 감지·초기화
- 사전 구성된 텔레옵/기록 명령 실행
- 두 팔(리더·팔로워)의 실시간 관절 위치 바
- 팔로워 상태 표시 (enable 상태, 모션 상태, 에러 코드)

### `piper-calibrate` — 관절 영점 캘리브레이션

```bash
piper-calibrate
```

1. CAN으로 팔에 연결 (티칭/프리 모드 권장)
2. 각 관절을 손으로 영점(홈) 자세에 위치
3. **Set Zero** 클릭 — `JointConfig(set_zero=0xAE)` 전송(그리퍼는 `GripperCtrl(..., 0xAE)`), 영점이 **모터 플래시 메모리에 기록**되어 전원을 꺼도 유지
4. 응답 플래그(`is_set_zero_successfully`)로 기록 성공 확인; raw 기준 5° 이상 벗어난 관절은 실수 방지 확인창 표시
5. **Clear Errors**는 전 관절에 `JointConfig(clear_err=0xAE)` 전송

#### 캘리브레이션이 소프트웨어가 아니라 모터에 있는 이유

[piper_follower.py](lerobot_robot_piper/piper_follower.py)의 `MotorCalibration` 테이블은 의도적으로 **모든 팔에 동일**합니다 — 팔 개별 보정값이 아닙니다. `range_min`/`range_max`는 각 관절 가동범위의 *기종 스펙*(0.001° 단위, 예: joint 1 = ±150000 = ±150°)이며, 절대각을 정규화 좌표 `[-100, 100]`으로 매핑하는 스케일로만 쓰입니다. `homing_offset`은 0이고, LeRobot의 파일 기반 캘리브레이션(`_load_calibration`/`_save_calibration`)은 의도된 no-op입니다.

팔 개별 오차는 **한 층 아래에서** 흡수됩니다: Piper SDK가 보고하는 절대각은 이미 각 모터 플래시에 저장된 영점으로 보정된 값입니다. 따라서:

- **영점이 정확한 팔** → raw 각도 = 실제 각도 → 팔 간 동일한 정규화 → 개별 오차가 수집 데이터에 *들어가지 않음*
- **영점이 어긋난 팔** → 그 오프셋이 모든 기록값에 그대로 실림. 해결책은 팔당 1회 `piper-calibrate`
- **리더-팔로워 짝**: 두 팔을 *같은 물리 기준자세*에서 각각 영점 설정해야 좌표계가 일치

한계(정직하게): 영점 정확도는 물리 기준자세를 얼마나 정확히 만드느냐에 의존하며(재현성이 필요하면 지그 사용), 링크 치수 편차·기어 백래시 같은 스케일성 오차는 어떤 영점으로도 — LeRobot 파일 캘리브레이션으로도 — 보정되지 않습니다.

### `camera_check.py` — 카메라 그리드 뷰어

```bash
python camera_check.py
```

`/dev/video0` … `/dev/video17`을 탐색해 동작하는 카메라를 모두 열고 한 그리드에 표시 — 기록 전 카메라 인덱스 확인에 유용합니다.

## 빠른 시작

### 1. Python API

```python
from lerobot_robot_piper import PiperFollowerConfig, PiperLeaderConfig, PiperFollower, PiperLeader

follower_cfg = PiperFollowerConfig(port="can_follower1")
leader_cfg   = PiperLeaderConfig(port="can_leader1")

follower = PiperFollower(follower_cfg)
leader   = PiperLeader(leader_cfg)

follower.connect()
leader.connect()

try:
    while True:
        action = leader.get_action()        # 리더 관절 위치 읽기
        follower.send_action(action)        # 팔로워에 미러링
        obs = follower.get_observation()    # 팔로워 상태 + 카메라 읽기
finally:
    follower.disconnect()
    leader.disconnect()
```

### 2. LeRobot CLI

```bash
# 텔레오퍼레이션
python -m lerobot.teleoperate \
    --robot.type=piper_follower \
    --robot.port=can_follower1 \
    --teleop.type=piper_leader \
    --teleop.port=can_leader1

# 데이터셋 기록
python -m lerobot.record \
    --robot.type=piper_follower \
    --robot.port=can_follower1 \
    --teleop.type=piper_leader \
    --teleop.port=can_leader1 \
    --dataset-id=my_piper_dataset
```

## 설정

### PiperFollowerConfig

| 파라미터 | 타입 | 기본값 | 설명 |
|-----------|------|---------|-------------|
| `port` | `str` | `"can0"` | 팔로워 팔의 CAN 포트 |
| `disable_torque_on_disconnect` | `bool` | `True` | 연결 해제 시 모터 비활성화 |
| `cameras` | `dict[str, CameraConfig]` | `{}` | 관측용 카메라 (이름 지정) |
| `max_relative_target` | `float \| dict \| None` | `None` | 스텝당 최대 관절 이동량 (안전 제한) |
| `can_interface` | `str` | `"socketcan"` | python-can 인터페이스. SocketCAN이 없으면 `gs_usb`나 `slcan` |
| `can_bitrate` | `int` | `1000000` | SocketCAN이 아닌 인터페이스의 CAN 속도 |

### PiperLeaderConfig

| 파라미터 | 타입 | 기본값 | 설명 |
|-----------|------|---------|-------------|
| `port` | `str` | `"can1"` | 리더 팔의 CAN 포트 |
| `gripper_open_pos` | `float` | `50.0` | 그리퍼 완전 개방에 해당하는 위치값 |
| `can_interface` | `str` | `"socketcan"` | 팔로워와 같음 |
| `can_bitrate` | `int` | `1000000` | 팔로워와 같음 |

## 모터 구성

Piper 팔은 관절 7개입니다. 정규화는 raw 인코더 값을 아래 범위로 매핑합니다.

| 관절 | 모델 | 정규화 범위 | 물리 범위 |
|-------|-------|-----------------|----------------|
| Joint 1 | AGILEX-M | −100 ~ +100 | ±150° |
| Joint 2 | AGILEX-M | −100 ~ +100 | 0–180° |
| Joint 3 | AGILEX-M | −100 ~ +100 | −170–0° |
| Joint 4 | AGILEX-S | −100 ~ +100 | ±100° |
| Joint 5 | AGILEX-S | −100 ~ +100 | ±65° |
| Joint 6 | AGILEX-S | −100 ~ +100 | ±100–130° |
| Gripper | AGILEX-S | 0 ~ 100 | 0–68° |

파킹(홈) 위치의 정규화 값: `0, −100, 100, 0, 35, 0, 0`

## 라이선스

Apache-2.0
