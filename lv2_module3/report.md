# Lv2 Module 3 — HSV Mask 및 Contour 객체 검출 실험 보고서

## 1. 목적 및 범위

기존 EYE4 팀 프로젝트의 ROS 2 인지 노드(`realsense_tracker`)를 재사용하여 파란색 퍽을 HSV 색상 기준으로 분리하고, Contour 최소 면적 조건에 따른 검출 결과를 비교했다. 카메라 영상을 ROS 2 `Image`로 입력받고 Raw / Mask / Detection / Overlay 토픽을 확인했다. **본 과제의 범위는 인지와 데이터 분석이며, 모터·OpenCR·Dynamixel 제어는 수행하지 않았다.**

## 2. 환경 및 구현

- ROS 2: Lyrical / Ubuntu 환경
- 카메라: Intel RealSense 컬러 영상, 설정 해상도 640×480, 목표 FPS 30
- 입력 토픽: `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- 출력 토픽:
  - `/perception/mask` (`sensor_msgs/msg/Image`, `mono8`): HSV 임계값과 Morphology를 적용한 이진 영상
  - `/target` (`geometry_msgs/msg/PointStamped`): `point.x` 정규화 수평 중심 오차, `point.y` 정규화 수직 중심 오차, `point.z` 검출 Contour 면적 / 화면 면적
  - `/perception/debug_image` (`sensor_msgs/msg/Image`): 검출 시각화 Overlay
- 주요 코드: `ros2_ws/src/realsense_tracker/realsense_tracker/detector.py`, `perception_node.py`
- 주요 설정: `ros2_ws/src/realsense_tracker/config/tracker.yaml`, `camera.yaml`
- 처리 흐름: Raw → Gaussian blur → BGR→HSV → `inRange` → Morphology opening/closing → `findContours(RETR_EXTERNAL)` → 최소 면적 필터 → 최대 면적 후보 선택 → 중심·면적 계산 → ROS 2 발행
- 검출 실패 시 해당 구현은 `/target`의 `x=y=z=0`을 발행한다. 이 값은 **미검출 출력 규칙**이지, 퍽의 실제 면적이 0이라는 뜻이 아니다.
- 이번 실험은 `use_depth=false`이며 `point.z`는 깊이·거리 값이 아니다.

### ROS 2 통신 확인

`results/topic_info/`에 노드·토픽 목록, `ros2 topic info` 출력, Raw/Mask/Detection 메시지 수신 증빙 및 스크린샷을 저장했다. 앞선 측정에서 Raw/Mask/Overlay가 각각 약 30 Hz로 수신됐다. 영상 구독은 RELIABLE, `/target` 발행은 BEST_EFFORT이므로 메시지를 받을 때 구독 QoS를 일치시켰다.

## 3. 공통 실험 입력 및 통제

실시간 촬영 장면이 달라지는 문제를 피하고자 한 번 녹화한 Raw Image rosbag을 전체 비교 실험의 입력으로 사용했다.

| 항목 | 기록 |
|---|---|
| 입력 폴더 | `results/input/raw_reference/` |
| rosbag 저장 형식 | MCAP |
| 녹화 길이 | 28.332135384초 |
| 녹화 메시지 수 | 851개 |
| 원본 토픽 | `/camera/camera/color/image_raw` |
| 녹화 범위(초) | `1791790928.982186110` ~ `1791790957.314321494` |
| A/B/C 비교 프레임 | `1791790952.834225098` |
| 색상 범위 상한 | `[130, 255, 255]` |
| 기타 영상 처리 | blur kernel 5 / morphology kernel 5 (설정 기본값) |

검출 비교 시 실시간 카메라 실행을 끄고 (`start_realsense:=false`) 같은 rosbag을 재생했다. 증빙 저장 도구는 Raw, Mask, Overlay, `/target`의 `header.stamp`를 맞춰 파일을 저장했다. 프레임 일치는 `frame_stamp.txt` 비교로 확인했다.

## 4. 실험 1 — HSV 채도(S) 하한 변경

### 가설

다른 값을 유지하고 **S 하한을 120에서 160으로 높이면**, 상대적으로 채도가 낮은 파란 영역 일부가 Mask에서 제외되어 검출 면적이 줄어들 수 있다. 중심 좌표에도 소폭 변화가 생길 수 있다.

### 조건과 실제 결과

| 항목 | A | B |
|---|---:|---:|
| HSV 하한 | `[93, 120, 35]` | `[93, 160, 35]` |
| HSV 상한 | `[130, 255, 255]` | `[130, 255, 255]` |
| `min_area_ratio` | 0.002 | 0.002 |
| `/target.point.x` | 0.4229848027 | 0.4227765752 |
| `/target.point.y` | -0.1176486142 | -0.1128248571 |
| `/target.point.z` | 0.0825862630 | 0.0807340495 |
| 검출 여부 | 검출 | 검출 |

- 검출 면적 비율: **8.2586% → 8.0734%** (약 **0.1852%p 감소**, 상대적으로 약 2.24% 감소)
- 수평 중심 오차 변화: 약 -0.0002082
- 수직 중심 오차 변화: 약 +0.0048238

### 해석

S 하한을 높이자 검출 Contour의 면적 비율이 소폭 감소했고 중심값도 약간 달라졌다. 이는 채도 임계값이 색상 마스크와 후속 Contour 모양에 영향을 준다는 가설에 부합한다. **단, 이 한 프레임만으로 노이즈 감소나 전체 검출 정확도 향상을 입증할 수는 없다.** 시각적 판단은 `results/experiment1/A|B/`의 Mask 및 Overlay를 나란히 비교해 보완한다.

증빙: [A Raw](results/experiment1/A/raw.png) · [A Mask](results/experiment1/A/mask.png) · [A Overlay](results/experiment1/A/overlay.png) / [B Raw](results/experiment1/B/raw.png) · [B Mask](results/experiment1/B/mask.png) · [B Overlay](results/experiment1/B/overlay.png). 수치는 각 폴더 `detection.txt` 참고.

## 5. 실험 2 — Contour 최소 면적 기준 변경

### 가설

HSV를 고정하고 `min_area_ratio`를 0.002 → 0.005 → 0.10으로 높이면, 면적이 작은 Contour 후보가 제외된다. 현재 퍽의 면적 비율이 약 8.26%이므로 앞의 두 기준은 통과하고 10%에서는 정상 퍽마저 제외될 것으로 예상했다.

### 조건과 실제 결과

| 항목 | A | B | C (추가 검증) |
|---|---:|---:|---:|
| HSV 하한 | `[93,120,35]` | `[93,120,35]` | `[93,120,35]` |
| HSV 상한 | `[130,255,255]` | `[130,255,255]` | `[130,255,255]` |
| `min_area_ratio` | 0.002 | 0.005 | 0.10 |
| 최소 면적 기준 | 화면의 0.2% | 화면의 0.5% | 화면의 10% |
| `/target.point.x` | 0.4229848027 | 0.4229848027 | 0.0 |
| `/target.point.y` | -0.1176486142 | -0.1176486142 | 0.0 |
| `/target.point.z` | 0.0825862630 | 0.0825862630 | 0.0 |
| 결과 | 검출 | 검출 | 미검출 출력 |

A와 B의 수치가 동일한 것은 같은 퍽 Contour가 두 조건을 모두 통과했기 때문이다. C에서는 면적 필터를 지나치게 높여 **실제로 존재하는 정상 객체가 검출 대상에서 제외되었다.** 세 조건은 입력 및 프레임 시각이 동일하다.

**구현상 주의:** `min_area_ratio`는 `findContours` **이후** 후보를 걸러내므로, HSV와 Morphology 파라미터가 같다면 A/B/C의 **Mask 자체는 바뀌지 않는 것이 정상**이다. 달라지는 것은 필터 통과 여부와 검출 결과/Overlay이다. 이 실험은 ‘작은 노이즈 제거 성능’을 직접 측정한 것이 아니라 ‘면적 필터 임계값이 정상 검출을 차단할 수 있는 경계’를 확인한 것이다.

증빙: [A Overlay](results/experiment2/A/overlay.png) · [B Overlay](results/experiment2/B/overlay.png) · [C Overlay](results/experiment2/C/overlay.png). 각 폴더의 `raw.png`, `mask.png`, `detection.txt`, `settings.txt`, `frame_stamp.txt` 참고.

## 6. 미검출 조건 시험

파란색 퍽을 실제 카메라 시야에서 제거하고, **실시간 RealSense** 입력으로 시험했다. 실험 2-C와 달리 이 시험은 물리적으로 목표가 없는 장면이다.

- 사용 설정: `results/no_target/settings.txt` (당시 `min_area_ratio=0.005`)
- 관측 시각: `1791792131.862364258`
- 실제 수신: `/target.point.x = 0.0`, `point.y = 0.0`, `point.z = 0.0`
- 결과: 노드의 미검출 출력 규칙과 일치
- 주의: `detection.txt`와 이미지 3장은 **같은 환경에서 별도로 수집**되었으며, 정확히 동일 프레임이라고 주장하지 않는다. 저장된 이미지끼리는 `frame_stamp.txt` 기준으로 일치한다.

증빙: [Raw](results/no_target/raw.png) · [Mask](results/no_target/mask.png) · [Overlay](results/no_target/overlay.png) · `results/no_target/detection.txt`.

## 7. 결론 및 한계

1. ROS 2 영상 입력과 Raw/Mask/Detection/Overlay 출력의 실수신을 확인했다.
2. HSV S 하한 상승은 동일 프레임의 검출 영역 크기와 중심 오차를 일부 변화시켰다.
3. 최소 면적 기준 0.2%와 0.5%는 퍽 검출 결과에 차이가 없었으나, 10%에서는 정상 퍽까지 미검출로 처리되었다.
4. 실시간으로 퍽을 제거한 장면에서도 미검출 출력 `(0,0,0)`을 관측했다.
5. 제한: 대표 프레임 중심의 정성·수치 비교이며, 다수 장면의 오검출률·미검출률·정확도 또는 통계적 유의성은 평가하지 않았다. `point.z`는 깊이가 아니고 면적 비율이다. 모터 안전정지까지 확인한 실험도 아니다.

## 8. 재현 및 근거 자료

실행 명령은 [README.md](README.md) 참고. 원본 rosbag은 크기 약 748.2 MiB이므로 GitHub 업로드 가능 여부를 별도로 판단하고, 제출 버전에 포함하지 않는다면 보고서에 **로컬 보존 및 미포함 사실**을 명시한다. 본문에 링크된 증빙 이미지·설정·로그는 저장소에 포함해야 한다.

**최종 제출 전 기입할 항목:** EYE4 원본 소스 저장소 링크 및 재사용 기준 커밋 해시, 개인 저장소 커밋 해시. 저장소의 실제 증빙 파일 존재 여부 확인.