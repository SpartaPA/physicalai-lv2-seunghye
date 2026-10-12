# Lv2 Module 3 — ROS 2 HSV Mask / Contour 객체 검출

기존 EYE4 프로젝트의 `realsense_tracker` 인지 코드를 재사용하여 파란색 퍽의 HSV Mask 및 Contour 검출을 확인하고, **같은 rosbag·같은 프레임**에서 HSV 임계값과 Contour 면적 필터를 바꾸어 비교한 개인 과제다.

- 상세한 가설·실제 측정·분석: [report.md](report.md)
- 소스: `ros2_ws/src/realsense_tracker/`
- ROS 2 배포판: Lyrical
- 실험 입력: `results/input/raw_reference/` (로컬 원본 MCAP, 약 748.2 MiB)
- 기능 범위: 영상 인지 / ROS 2 토픽 검증; 모터·OpenCR·Dynamixel 제어 제외

## 재사용 코드 출처

- 팀 프로젝트: https://github.com/SpartaPA/Lv2_EYE4_Assignment
- 재사용 기준 커밋: **[제출 전 실제 커밋 해시 기입]**
- 개인 과제 커밋: **[제출 전 실제 커밋 해시 기입]**
- 수정 범위: 모듈 3용 비전 단독 실행 구성, HSV/Contour 설정 실험, `/perception/mask` (`mono8`) 출력 및 증빙 저장

## 데이터 흐름

`RealSense 컬러 영상 또는 rosbag` → `/camera/camera/color/image_raw` → `perception_node` → HSV + Morphology Mask → Contour 후보 필터 → `/perception/mask`, `/target`, `/perception/debug_image`

| 토픽 | 메시지 | 의미 |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Raw 컬러 입력 |
| `/perception/mask` | `sensor_msgs/msg/Image` | HSV+Morphology 이진 Mask (`mono8`) |
| `/target` | `geometry_msgs/msg/PointStamped` | x/y 정규화 중심 오차, z 윤곽선 면적 비율; 미검출 `(0,0,0)` |
| `/perception/debug_image` | `sensor_msgs/msg/Image` | 검출 중심/윤곽선 Overlay |

`/target`의 Publisher는 BEST_EFFORT이므로 echo에서 `--qos-reliability best_effort`를 사용한다. Raw 구독은 RELIABLE 설정이다.

## 실행

### 전제조건

Ubuntu에서 ROS 2 Lyrical 및 `realsense2_camera`, Python `cv2`/NumPy, 이 ROS 2 패키지가 준비되어 있어야 한다. **새 터미널마다 환경을 source한다.** rosbag 재생 실험에서는 실시간 카메라를 동시에 시작하지 않는다.

```bash
cd ~/git/physicalai-lv2-seunghye/lv2_module3/ros2_ws
source /opt/ros/lyrical/setup.bash
# 최초 빌드 또는 소스 수정 후
colcon build --symlink-install
source install/setup.bash
```

실시간 카메라 실행:

```bash
ros2 launch realsense_tracker tracker.launch.py \
  start_realsense:=true \
  tracker_params:=$PWD/src/realsense_tracker/config/tracker.yaml
```

동일 영상 재생 실험: 첫 터미널에서 `start_realsense:=false`로 실행한 뒤, **별도 터미널**에서 rosbag을 재생한다.

```bash
ros2 launch realsense_tracker tracker.launch.py \
  start_realsense:=false \
  tracker_params:=$PWD/src/realsense_tracker/config/tracker.yaml
```

```bash
cd ~/git/physicalai-lv2-seunghye/lv2_module3/ros2_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 bag play ../results/input/raw_reference --loop
```

토픽 수신과 적용값 확인 (세 번째 터미널):

```bash
ros2 topic list -t
ros2 topic echo /target --qos-reliability best_effort --once
ros2 param get /perception_node hsv_lower
ros2 param get /perception_node min_area_ratio
```

설정은 `ros2_ws/src/realsense_tracker/config/tracker.yaml`에서 변경한다. 이번 실험에서는 인지 노드를 종료한 후 설정을 고치고 launch를 다시 실행해 적용했다. **실험 종료 후 원래 기준값 `[93,120,35]`, `[130,255,255]`, `0.002`로 복원한다.**

## 실험 개요

| 실험 | 변경 변수 | 조건 | 주요 결과 |
|---|---|---|---|
| 1 | HSV S 하한 | 120 / 160 | 검출 면적 비율 8.2586% → 8.0734% |
| 2 | `min_area_ratio` | 0.002 / 0.005 / 0.10 | A/B 검출, C 미검출 |
| 미검출 | 물리적 목표 제거 | 실시간 입력 | `/target` = `(0,0,0)` |

## 결과 폴더

```text
results/
├── topic_info/             # 노드·토픽 목록/메시지/초기 화면 증빙
├── input/
│   └── raw_reference/      # 원본 MCAP (대용량, 로컬 보존)
├── experiment1/
│   ├── A/                   # HSV S=120
│   └── B/                   # HSV S=160
├── experiment2/
│   ├── A/                   # 최소 면적 0.002
│   ├── B/                   # 최소 면적 0.005
│   └── C/                   # 추가 검증 0.10
└── no_target/               # 실시간 카메라 미검출
```

각 실험 폴더에는 `settings.txt`, `raw.png`, `mask.png`, `overlay.png`, `detection.txt`, `frame_stamp.txt`를 보관한다. `results/no_target/detection.txt`와 스크린샷은 수집 시점이 다르므로 동일 프레임이라고 주장하지 않는다.

### 실험 원본 rosbag

- **파일명:** `module3_raw_reference.zip`
- **입력 토픽:** `/camera/camera/color/image_raw`
- **형식:** ROS 2 MCAP
- **녹화 시간:** 약 28.33초
- **메시지 수:** 851개
- **원본 크기:** 약 748MB
- **실험 비교 프레임:** `1791790952.834225098`

### 원본 다운로드

[Google Drive에서 rosbag 다운로드](https://drive.google.com/file/d/1PTjfT3OOkpNkC9UcmDkL779aGIQ2Bnne/view?usp=sharing)

원본 rosbag은 파일 크기로 인해 GitHub 저장소에 포함하지 않고
Google Drive에 별도로 보관하였다.

실험 1-A/B 및 실험 2-A/B/C는 동일한 rosbag의 동일 프레임을
사용하여 설정값 변경에 따른 검출 결과를 비교하였다.

### rosbag 재생 방법

압축 해제 후 ROS 2 환경에서 다음 명령어를 실행한다.

```bash
ros2 bag play raw_reference
```

> 제출 전 확인: 본 README에서 참조하는 결과 파일을 실제 Git에 추가한다. 원본 MCAP의 크기가 약 748MB이므로 무리하게 커밋하지 말고 로컬 보존 여부 및 별도 전달 방법을 명확히 표시한다.