"""HSV·Contour 단일 색상 목표 검출기 (인지 알고리즘 — ROS·카메라와 무관하게 동작)

perception_node.py에서 호출하는 HSV·Contour 검출 알고리즘이다.
기존 EYE4 비전 코드를 재사용하며, 검출 알고리즘 자체는 변경하지 않는다.

파이프라인: 영상 → 블러 → HSV → 색상 마스크 → 잡음 제거 → 컨투어 → 대상 선택 → 중심 계산
대상 선택 규칙: min_area_ratio 이상인 컨투어 중 면적이 가장 큰 것 1개
깊이 영상(컬러에 정렬된 것)을 주면 목표까지의 실제 거리(dist_cm)도 계산해 기록·확인에 쓴다.
기본 추적은 Color 중심 오차만 사용한다. 깊이로 검출을 막는 거리 gate는 enforce_depth_range: true일 때만 켠다
(기본 false → 깊이가 없거나 불안정해도 Color 검출 결과를 미검출로 바꾸지 않음).
/target.point.z에는 깊이가 아니라 면적비(area_ratio)가 들어간다.
"""
import cv2
import numpy as np

# 인지 파라미터 기본값 (ROS 파라미터 이름 그대로) — 실제 값은 config/tracker.yaml에서 관리
PARAM_DEFAULTS = {
    "hsv_lower": [93, 120, 35],      # 목표 색 [H, S, V] 하한 (OpenCV: H 0~179, S/V 0~255)
    "hsv_upper": [130, 255, 255],    # 목표 색 [H, S, V] 상한
    "min_area_ratio": 0.002,         # 화면 넓이 대비 최소 크기 (640x480에서 약 614px)
    "blur_ksize": 5,                 # 가우시안 블러 크기 (홀수, 짝수면 1 크게 보정, 1이면 블러 안 함)
    "morph_ksize": 5,                # 잡음 제거(열기/닫기) 크기 (1이면 안 함)
    "depth_min_valid_ratio": 0.5,    # 목표 영역에서 깊이가 측정된 비율이 이보다 낮으면 거리를 비움
    "depth_max_spread_cm": 5.0,      # 목표 영역 깊이 값의 흩어짐(p25~p75)이 이보다 크면 거리를 비움
    "enforce_depth_range": False,    # true일 때만 거리 범위 밖·측정 불가를 미검출로 처리 (선택 기능)
    "depth_min_distance_cm": 13.0,   # enforce_depth_range: true일 때의 최소 거리 (cm)
    "depth_max_distance_cm": 100.0,  # enforce_depth_range: true일 때의 최대 거리 (cm)
}


def cfg_from_params(p):
    """ROS 파라미터 형식(평평한 이름)의 값 → detect()가 쓰는 설정 dict"""
    return {
        "hsv": {"ranges": [{"lower": [int(v) for v in p["hsv_lower"]],
                            "upper": [int(v) for v in p["hsv_upper"]]}]},
        "min_area_ratio": float(p["min_area_ratio"]),
        "blur_ksize": int(p["blur_ksize"]),
        "morph_ksize": int(p["morph_ksize"]),
        "depth": {"min_valid_ratio": float(p["depth_min_valid_ratio"]),
                  "max_spread_cm": float(p["depth_max_spread_cm"]),
                  "enforce_range": bool(p["enforce_depth_range"]),
                  "min_distance_cm": float(p["depth_min_distance_cm"]),
                  "max_distance_cm": float(p["depth_max_distance_cm"])},
    }


def make_mask(frame, cfg):
    k = cfg.get("blur_ksize", 5)
    if k > 1 and k % 2 == 0:  # GaussianBlur는 홀수 크기만 가능 → 짝수면 1 크게 보정
        k += 1
    blurred = cv2.GaussianBlur(frame, (k, k), 0) if k > 1 else frame
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for r in cfg["hsv"]["ranges"]:
        mask |= cv2.inRange(hsv, np.array(r["lower"]), np.array(r["upper"]))

    m = cfg.get("morph_ksize", 5)
    if m > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (m, m))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 작은 점 잡음 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 목표 내부 구멍 메우기
    return mask


def target_distance_cm(depth, mask, contour, depth_scale, min_valid_ratio=0.5, max_spread_cm=5.0):
    """목표까지의 거리(cm). 믿을 수 없으면 None

    목표 컨투어 안에서 색 마스크에 해당하는 픽셀들의 깊이 중앙값을 쓴다.
    단, 아래 경우에는 틀린 값일 가능성이 커서 None을 돌려준다. (실측 근거: 아래 두 기준으로
    정상 측정은 모두 통과하고, 카메라 바로 앞을 손으로 가렸을 때의 틀린 값은 모두 걸러짐)
      - 깊이가 측정된 픽셀 비율 < min_valid_ratio  → 대부분 측정 실패
      - 깊이 값의 흩어짐(p25~p75) > max_spread_cm  → 다른 물체·배경의 깊이가 섞임
    RealSense는 약 5cm 떨어진 적외선 카메라 두 대로 거리를 재므로, 카메라 가까이에 손 같은 물체가 있어
    한쪽 카메라의 시야만 가리면 컬러 화면에 목표가 보여도 깊이는 비거나 엉뚱한 값이 된다.
    """
    region = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)  # 목표 영역을 채운 마스크
    target = depth[(region > 0) & (mask > 0)]          # 목표 픽셀들의 깊이 (0 = 측정 실패)
    values = target[target > 0]
    if values.size == 0 or values.size / target.size < min_valid_ratio:
        return None
    p25, p50, p75 = np.percentile(values, [25, 50, 75]) * depth_scale * 100
    if p75 - p25 > max_spread_cm:
        return None
    return float(p50)


def detect(frame, cfg, depth=None, depth_scale=0.001):
    """반환: dict(found, ex, ey, z, dist_cm, cx, cy, contour, n_candidates, mask)

    미검출이면 found=False, ex=ey=z=0 (이전 좌표를 재사용하지 않음)
    ex=(cx-W/2)/(W/2), ey=(cy-H/2)/(H/2)  → 오른쪽·아래가 +
    z = contour_area / (W*H)  → 검출되면 항상 0보다 큼 (그래서 z == 0 이 미검출 신호)
    dist_cm = 깊이 영상(depth, 컬러에 정렬됨)으로 잰 목표까지의 거리.
              depth가 없거나, 측정에 실패했거나, 믿을 수 없으면 None
    depth_scale = 깊이 값 1이 몇 m인지 (RealSense 16UC1은 0.001, 32FC1(m 단위)은 1.0)
    거리 gate (선택): cfg["depth"]["enforce_range"]가 true이고 depth를 주면
                    min_distance_cm <= dist_cm <= max_distance_cm 일 때만 유효 목표.
                    범위 밖이거나 거리를 잴 수 없으면 found=False, ex=ey=z=0 (dist_cm은 확인용으로 남김)
                    enforce_range가 false(기본)이면 dist_cm은 기록용일 뿐 검출 여부에 영향을 주지 않는다.
    """
    H, W = frame.shape[:2]
    mask = make_mask(frame, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = cfg["min_area_ratio"] * W * H
    candidates = [c for c in contours if cv2.contourArea(c) >= min_area]

    result = dict(found=False, ex=0.0, ey=0.0, z=0.0, dist_cm=None, cx=None, cy=None,
                  contour=None, n_candidates=len(candidates), mask=mask)
    if not candidates:
        return result

    target = max(candidates, key=cv2.contourArea)
    M = cv2.moments(target)
    if M["m00"] == 0:
        return result

    dcfg = cfg.get("depth") or {}  # 설정에 depth 항목이 없으면 기본값 사용
    dist = None if depth is None else target_distance_cm(
        depth, mask, target, depth_scale,
        min_valid_ratio=dcfg.get("min_valid_ratio", 0.5),
        max_spread_cm=dcfg.get("max_spread_cm", 5.0))
    result["dist_cm"] = dist  # 확인 화면·로그용 (/target에는 들어가지 않음)
    if depth is not None and dcfg.get("enforce_range", False):
        # 선택 기능: 거리 범위 밖이거나 거리를 잴 수 없으면 미검출과 같이 ex=ey=z=0
        if dist is None or not dcfg["min_distance_cm"] <= dist <= dcfg["max_distance_cm"]:
            return result

    cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
    result.update(
        found=True,
        cx=cx, cy=cy,
        ex=(cx - W / 2) / (W / 2),
        ey=(cy - H / 2) / (H / 2),
        z=cv2.contourArea(target) / (W * H),
        dist_cm=dist,
        contour=target,
    )
    return result


def draw(frame, res):
    """원본 위에 영상 중심(흰 십자), 컨투어(초록), 목표 중심(빨강), 수치 표시"""
    out = frame.copy()
    H, W = out.shape[:2]
    cv2.drawMarker(out, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)

    if res["found"]:
        cv2.drawContours(out, [res["contour"]], -1, (0, 255, 0), 2)
        c = (int(res["cx"]), int(res["cy"]))
        cv2.circle(out, c, 6, (0, 0, 255), -1)
        cv2.line(out, (W // 2, H // 2), c, (0, 0, 255), 1)
        text = f"ex={res['ex']:+.3f} ey={res['ey']:+.3f} z={res['z']:.4f}"
        color = (0, 255, 0)
    else:
        text = "NOT DETECTED (z=0)"
        color = (0, 0, 255)
    dist = "--" if res["dist_cm"] is None else f"{res['dist_cm']:.0f} cm"
    cv2.putText(out, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(out, f"candidates={res['n_candidates']}  dist={dist}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out
