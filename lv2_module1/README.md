[장비,환경,실행방법과 결과 파일 위치]
# Module 1. 임베디드 제어 기초

## 1. 장비 및 환경

이번 모듈에서는 Raspberry Pi, OpenCR, Dynamixel을 이용하여 기본적인 모터 제어와 P 제어 실습을 진행하였다.

### 사용 환경

| 항목 | 내용 |
|---|---|
| Raspberry Pi OS | Ubuntu 22.04.5 LTS |
| Architecture | aarch64 |
| OpenCR 연결 포트 | `/dev/ttyACM0` |
| Serial 통신 | 115200 baud |
| 제어 보드 | OpenCR |
| 구동 장치 | Dynamixel |

전체 연결 구조는 다음과 같다.

```text
Laptop
  ↓ SSH
Raspberry Pi
  ↓ USB Serial
OpenCR
  ↓ 제어 명령
Dynamixel
```

---

## 2. 실행 방법

### OpenCR 연결 확인

OpenCR이 `/dev/ttyACM0` 포트에 연결되어 있는지 확인한다.

```bash
ls -l /dev/ttyACM0
```

Serial Monitor를 실행한다.

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 --eol LF -e
```

### 실행 A

```text
s 3 20 30
```

설정값:

- `Kp = 3.0`
- 속도 제한 = `20 °/s`
- 상대 목표각 = `+30 °`

### 실행 B

```text
s 0.5 20 30
```

설정값:

- `Kp = 0.5`
- 속도 제한 = `20 °/s`
- 상대 목표각 = `+30 °`

실행 A와 실행 B는 목표각과 속도 제한을 동일하게 유지하고 `Kp`만 변경하여 응답 차이를 비교하였다.

실행 중 정지가 필요한 경우 다음 명령을 사용하였다.

```text
x
```

---

## 3. 결과 파일 위치

실습 결과 및 증거 파일은 `results/` 폴더에 저장하였다.

| 파일 | 내용 |
|---|---|
| `results/환경확인.txt` | Raspberry Pi 호스트명, Ubuntu 버전, 아키텍처, OpenCR 포트 및 USB 장치 확인 |
| `results/upload.log` | OpenCR 펌웨어 업로드 결과 |
| `results/실행A.log` | `Kp=3.0`, 속도 제한 `20 °/s`, 목표각 `+30°` 실행 결과 |
| `results/실행B.log` | `Kp=0.5`, 속도 제한 `20 °/s`, 목표각 `+30°` 실행 결과 |
| `report.md` | 문제 1~4의 설정, 결과, 계산 및 해석 |

---

## 4. 주요 실습 내용

이번 모듈에서는 다음 내용을 확인하였다.

- Raspberry Pi에서 OpenCR 연결 및 통신 환경 확인
- OpenCR 펌웨어 업로드
- Dynamixel 목표 위치 제어
- 목표값과 현재값을 이용한 오차 계산
- P 제어에서 `Kp` 변화에 따른 응답 비교
- `p_deg_s`와 최종 제어 출력 `u_deg_s`의 차이 확인
- 제어 주기와 상태 발행 주기의 차이 이해
- PC ROS2 Node, micro-ROS Agent, OpenCR, Dynamixel 간의 역할 구분

자세한 실험 결과와 해석은 [`report.md`](./report.md)에서 확인할 수 있다.