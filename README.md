
# Physical AI Lv2 과제 제출 Repository

## 수강생
- 이름: 전승혜

## 모듈별 제출 폴더

### Module 1 — 임베디드 제어 기초
- [과제 안내 및 실행 방법](lv2_module1/README.md)
- [실험 보고서](lv2_module1/report.md)

### Module 3 — HSV Mask / Contour 객체 검출
- [과제 안내 및 실행 방법](lv2_module3/README.md)
- [실험 보고서](lv2_module3/report.md)

## 제출 구조

```text
physicalai-lv2-seunghye/
├── README.md
├── lv2_module1/
│   ├── README.md
│   ├── report.md
│   └── results/
│       ├── 실행A.log
│       ├── 실행B.log
│       ├── 환경확인.txt
│       └── upload.log
└── lv2_module3/
    ├── README.md
    ├── report.md
    ├── ros2_ws/
    │   └── src/
    │       └── realsense_tracker/
    └── results/
        ├── topic_info/
        ├── experiment1/
        │   ├── A/
        │   └── B/
        ├── experiment2/
        │   ├── A/
        │   ├── B/
        │   └── C/
        └── no_target/
```

## 과제별 수행 내용

### Module 1 — 임베디드 제어 기초

OpenCR 기반 임베디드 제어 환경을 구성하고, 펌웨어 업로드 및 모터 제어 실험을 수행했다.

### Module 3 — HSV Mask / Contour 객체 검출

기존 EYE4 프로젝트의 인지 노드를 재사용하여 ROS 2 기반 객체 검출을 수행했다.

- Raw·Mask·Detection·Overlay 토픽 수신 검증
- 동일한 rosbag 프레임을 활용한 HSV 임계값 A/B 비교 실험
- Contour 최소 면적 필터 A/B/C 비교 실험
- 목표물 미검출 조건 확인 및 결과 기록

자세한 실행 방법과 실험 결과는 각 모듈의 `README.md` 및 `report.md`를 참고한다.
