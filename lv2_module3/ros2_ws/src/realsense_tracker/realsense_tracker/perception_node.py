"""모듈 3 개인 과제 — HSV Mask 및 객체 검출 인지 노드.

[1. 목적]
기존 EYE4 팀 프로젝트의 비전 처리 코드를 재사용한다.

카메라에서 받은 원본 영상에 HSV 색상 조건을 적용하여
Mask를 생성하고, Contour와 최소 면적 조건으로
파란색 목표 물체를 검출한다.

이번 과제에서는 HSV 범위와 최소 면적 비율을 각각
변경하면서 Mask와 검출 결과의 차이를 분석한다.

모터 제어, OpenCR 통신 및 Dynamixel 구동은 수행하지 않는다.

[2. 입력 토픽]
Raw Image:
    config/camera.yaml의 color_topic 값을 사용
    타입: sensor_msgs/msg/Image

Depth Image:
    use_depth=true일 때만 구독
    이번 과제에서는 use_depth=false로 비활성화

[3. 출력 토픽]
/target
    타입: geometry_msgs/msg/PointStamped
    point.x: 좌우 정규화 중심 오차
    point.y: 상하 정규화 중심 오차
    point.z: 목표 Contour 면적 / 전체 이미지 면적
    미검출: point.x = point.y = point.z = 0

/perception/debug_image
    타입: sensor_msgs/msg/Image
    검출 중심점 및 표시 정보를 포함하는 Overlay
    publish_debug_image=true일 때 발행

/perception/mask
    타입: sensor_msgs/msg/Image
    HSV와 Morphology 처리를 거친 Mask
    mono8 형식으로 발행

[4. 처리 흐름]
Raw Image 구독
    → NumPy 배열 변환
    → HSV Mask 생성
    → Contour 추출
    → 최소 면적 조건 적용
    → 검출 결과 계산
    → Mask / Detection / Overlay 발행

[5. 설정 파일]
config/tracker.yaml
    HSV 하한·상한, 최소 면적 비율, QoS 설정

config/camera.yaml
    원본 영상 토픽 이름, 해상도, FPS

[6. ROS 2 QoS]
/target:
    target_reliable=false → BEST_EFFORT, depth 1

Raw Image 구독:
    image_reliable=true → RELIABLE, depth 1

[7. 실험 조건]
한 실험에서 변수 하나만 변경한다.
실험 전 가설과 실제 적용값을 기록한다.
A/B는 동일 입력을 사용하고 나머지 설정을 고정한다.

코드 출처와 사용한 커밋은 개인 과제 README.md에 기록한다.
"""

import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .detector import PARAM_DEFAULTS, cfg_from_params, detect, draw

# 인지 파라미터 외에 노드에서만 쓰는 파라미터
NODE_PARAM_DEFAULTS = {
    "camera_config": "",            # 패키지 config/camera.yaml 경로 — 카메라 토픽 이름을 여기서 읽음 (launch가 자동 지정)
    "use_depth": False,             # true면 깊이 영상도 받아 거리(dist_cm)를 확인 화면·로그에 표시 (gate는 enforce_depth_range)
    "depth_unit_m": 0.001,          # 16UC1 깊이 값 1이 몇 m인지 (RealSense 기본: 1mm)
    "publish_debug_image": False,   # true면 /perception/debug_image 발행 (rqt_image_view 등으로 확인)
    "stats_period_sec": 5.0,        # 이 간격(초)마다 처리 FPS·처리 시간을 로그로 남김 (0이면 끔)
    # /target 발행 QoS. false = best effort, depth 1 (발제문 규약: best-effort, depth 1부터 적용)
    # ※ best effort 발행은 reliable 구독과 연결되지 않음 → 구독 측(제어)도 best effort로 받아야 함
    "target_reliable": False,
    # 영상 구독 QoS (depth 1). true = reliable, false = best effort
    # 실측(2026-10-06, RealSense wrapper 640x480 30fps, Fast DDS 기본 설정, 다른 프로세스에서 구독):
    #   best effort → 컬러 1.2 fps, 정렬 깊이 0 fps / reliable → 둘 다 29.4 fps  → reliable 사용
    "image_reliable": True,
}
DEPTH_PAIR_SLOP_NS = 50_000_000   # 컬러·깊이 시각 차 50 ms 이내만 같은 장면으로 봄
CAMERA_TOPIC_KEYS = ("color_topic", "aligned_depth_topic", "camera_info_topic")


class ConfigError(Exception):
    """설정이 없거나 비어 있어 노드를 시작할 수 없음"""


def load_camera_topics(path):
    """camera.yaml에서 카메라 토픽 이름을 읽는다. 반환: {color_topic, aligned_depth_topic, camera_info_topic}"""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return {key: str(data.get(key) or "").strip() for key in CAMERA_TOPIC_KEYS}


def image_to_numpy(msg):
    """sensor_msgs/Image → numpy 배열 (cv_bridge 없이 변환)

    컬러: bgr8 / rgb8 → BGR (H, W, 3) uint8
    깊이: 16UC1 / mono16 → (H, W) uint16,  32FC1 → (H, W) float32 (m)
    """
    enc = msg.encoding.lower()
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)  # 줄 끝 여분(step)까지 포함
    if enc in ("bgr8", "rgb8"):
        img = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(img[..., ::-1] if enc == "rgb8" else img)
    if enc in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        return rows[:, : msg.width * 2].copy().view(dtype).reshape(msg.height, msg.width).astype(np.uint16)
    if enc == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        return rows[:, : msg.width * 4].copy().view(dtype).reshape(msg.height, msg.width).astype(np.float32)
    raise ValueError(f"지원하지 않는 영상 형식: {msg.encoding}")


def numpy_to_image(bgr, header):
    """BGR numpy 배열 → sensor_msgs/Image (bgr8)"""
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg

def mask_to_image(mask, header):
    """OpenCV uint8 1채널 Mask를 ROS 2 Image로 변환한다.

    mask:
        검출 알고리즘이 반환한 2차원 NumPy 배열.
        일반적인 이진 Mask에서 0=배경, 255=통과 영역.

    header:
        원본 컬러 영상의 시간 정보와 좌표계.
        동일 프레임과 비교할 수 있도록 그대로 사용한다.
    """
    msg = Image()

    # 원본 영상과 같은 시각·좌표계 사용
    msg.header = header

    # 2차원 영상의 높이와 너비
    msg.height, msg.width = mask.shape

    # 1픽셀당 8비트인 단일 채널 영상
    msg.encoding = "mono8"

    # uint8 단일 채널이므로 픽셀당 1바이트
    msg.is_bigendian = 0
    msg.step = msg.width

    # NumPy 배열을 ROS 이미지 메시지 데이터로 변환
    msg.data = np.ascontiguousarray(
        mask, dtype=np.uint8
    ).tobytes()

    return msg

class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        params = {name: self.declare_parameter(name, default).value
                  for name, default in {**PARAM_DEFAULTS, **NODE_PARAM_DEFAULTS}.items()}
        self.cfg = cfg_from_params(params)
        self.use_depth = bool(params["use_depth"])
        self.depth_unit_m = float(params["depth_unit_m"])
        self.last_found = None          # 검출 상태가 바뀔 때만 로그를 남기기 위함
        self.warned_size = False
        self.latest_depth = None        # 가장 최근 깊이 영상 (컬러와 시각이 맞을 때만 사용)
        # 처리 FPS 측정 (발제문: 처리 완료 프레임 수 / 실제 경과 초 — 카메라 설정 FPS와 구분)
        self.stats_period = float(params["stats_period_sec"])
        self.stats = {"t0": None, "n": 0, "found": 0, "depth_ok": 0, "proc_ms": []}

        # 카메라 토픽 이름: config/camera.yaml (가이드: 코드에 하드코딩하지 않음)
        cam_path = params["camera_config"]
        if not cam_path:
            raise ConfigError("camera_config 파라미터가 비어 있음 → "
                              "launch 파일로 실행하거나 "
                              "-p camera_config:=<실제 camera.yaml 절대경로> 를 지정하세요")
        try:
            topics = load_camera_topics(cam_path)
        except OSError as e:
            raise ConfigError(f"camera_config 파일을 읽을 수 없음: {e}")
        needed = ["color_topic"] + (["aligned_depth_topic"] if self.use_depth else [])
        missing = [key for key in needed if not topics[key]]
        if missing:
            raise ConfigError(f"{cam_path}의 {', '.join(missing)} 값이 비어 있음 → RealSense wrapper 실행 후 "
                              "`ros2 topic list`로 실제 이름을 확인해 기록하세요")

        def qos(reliable):  # depth 1 = 가장 최근 메시지만 유지 (밀린 메시지를 쌓아 두지 않음)
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE if reliable
                              else ReliabilityPolicy.BEST_EFFORT)

        # /target Publisher
        self.pub = self.create_publisher(PointStamped, "/target", qos(params["target_reliable"]))
        
        # HSV·Morphology Mask 발행
        # RELIABLE, KEEP_LAST depth 1:
        # rqt_image_view에서 결과 영상을 받을 수 있도록 사용
        self.mask_pub = self.create_publisher(Image,"/perception/mask",qos(True),)

        self.debug_pub = (self.create_publisher(Image, "/perception/debug_image", 1)
                          if params["publish_debug_image"] else None)

        # 카메라 Image Subscriber
        # 영상 구독
        image_qos = qos(params["image_reliable"])
        self.create_subscription(Image, topics["color_topic"], self.on_color, image_qos)
        
        if self.use_depth:
            self.create_subscription(Image, topics["aligned_depth_topic"], self.on_depth, image_qos)

        name = lambda reliable: "reliable" if reliable else "best effort"
        r = self.cfg["hsv"]["ranges"][0]
        d = self.cfg["depth"]
        self.get_logger().info(
            f"인지 노드 시작: 컬러 {topics['color_topic']}"
            + (f", 깊이 {topics['aligned_depth_topic']}" if self.use_depth else "")
            + f", HSV {r['lower']}~{r['upper']}, min_area_ratio={self.cfg['min_area_ratio']}, "
            + (f"거리 gate {d['min_distance_cm']:g}~{d['max_distance_cm']:g} cm 사용, "
               if self.use_depth and d["enforce_range"] else "거리 gate 없음(Color 검출 기준), ")
            + f"/target 발행={name(params['target_reliable'])}·depth 1, "
            f"영상 구독={name(params['image_reliable'])}·depth 1, "
            f"확인 화면 발행={self.debug_pub is not None},"
            f"Mask 발행=/perception/mask (mono8, reliable)")

    def on_depth(self, msg):
        self.latest_depth = msg

    def paired_depth(self, color_msg):
        """컬러 영상과 시각 차 50 ms 이내인 최근 깊이 영상, 없으면 None"""
        d = self.latest_depth
        if d is None:
            return None
        stamp = lambda m: m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        return d if abs(stamp(d) - stamp(color_msg)) <= DEPTH_PAIR_SLOP_NS else None

    def on_color(self, color_msg):
        t_start = time.perf_counter()
        depth_msg = self.paired_depth(color_msg) if self.use_depth else None
        try:
            frame = image_to_numpy(color_msg)
            depth, scale = None, self.depth_unit_m
            if depth_msg is not None:
                depth = image_to_numpy(depth_msg)
                if depth_msg.encoding.lower() == "32fc1":
                    scale = 1.0                        # 32FC1은 이미 m 단위
                if depth.shape != frame.shape[:2]:     # 컬러에 정렬되지 않은 깊이는 쓸 수 없음
                    if not self.warned_size:
                        self.get_logger().warning(
                            f"깊이 {depth.shape}와 컬러 {frame.shape[:2]} 크기가 달라 거리 계산을 건너뜀 "
                            "(깊이를 컬러에 정렬해서 발행해야 함)")
                        self.warned_size = True
                    depth = None
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        # 기존 알고리즘: Mask 생성 및 객체 검출
        res = detect(frame, self.cfg, depth, scale)
        
        # 추가 기능: Mask 이미지 토픽 발행
        # detector.py가 계산한 Mask를 ROS 2 이미지로 발행
        # 원본 컬러 영상의 header를 그대로 사용
        self.mask_pub.publish(mask_to_image(res["mask"], color_msg.header))
        
        # 기존 기능: 검출 수치 메시지 생성
        out = PointStamped()
        out.header = color_msg.header
        out.point.x = float(res["ex"])  # 미검출이면 detect()가 ex = ey = z = 0 을 돌려줌
        out.point.y = float(res["ey"])
        out.point.z = float(res["z"])
        self.pub.publish(out)

        # 기존 Overlay 발행
        if self.debug_pub is not None:
            self.debug_pub.publish(numpy_to_image(draw(frame, res), color_msg.header))

        self.update_stats(res["found"], res["dist_cm"] is not None, (time.perf_counter() - t_start) * 1000)

        if res["found"] != self.last_found:     # 검출 ↔ 미검출이 바뀔 때만 기록
            dist = "--" if res["dist_cm"] is None else f"{res['dist_cm']:.1f} cm"
            if res["found"]:
                self.get_logger().info(f"목표 검출: ex={res['ex']:+.3f} ey={res['ey']:+.3f} "
                                       f"z={res['z']:.4f} dist={dist} 후보={res['n_candidates']}")
            else:
                self.get_logger().info("목표 미검출 → x = y = z = 0 발행")
            self.last_found = res["found"]

    def update_stats(self, found, depth_ok, proc_ms):
        """처리 완료 프레임을 세고, stats_period_sec마다 처리 FPS와 처리 시간(영상 변환~발행)을 로그로 남김"""
        if self.stats_period <= 0:
            return
        s, now = self.stats, time.monotonic()
        if s["t0"] is None:
            s["t0"] = now
        s["n"] += 1
        s["found"] += int(found)
        s["depth_ok"] += int(depth_ok)
        s["proc_ms"].append(proc_ms)
        elapsed = now - s["t0"]
        if elapsed >= self.stats_period:
            self.get_logger().info(
                f"처리 FPS {s['n'] / elapsed:.1f} (최근 {elapsed:.1f}초 동안 {s['n']}장 처리), "
                f"처리 시간 평균 {sum(s['proc_ms']) / len(s['proc_ms']):.1f} ms·최대 {max(s['proc_ms']):.1f} ms, "
                f"노드가 검출로 표시한 프레임 {s['found']}/{s['n']} (사람 대조 검출률과 다름)"
                + (f", 유효 거리 측정 {s['depth_ok']}/{s['n']}" if self.use_depth else ""))
            self.stats = {"t0": now, "n": 0, "found": 0, "depth_ok": 0, "proc_ms": []}


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
    except ConfigError as e:  # 설정 문제는 긴 오류 대신 해야 할 일을 알려주고 종료
        rclpy.logging.get_logger("perception_node").fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C·종료 신호는 정상 종료로 처리
        pass
    except Exception:
        if rclpy.ok():  # 실행 중에 난 오류는 그대로 알리고, 종료 신호와 겹쳐 난 오류(context 무효)만 무시
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
