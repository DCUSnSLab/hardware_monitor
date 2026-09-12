"""바이너리 프레임 인코딩 (순수 함수, ROS/WS 비의존).

프레임 레이아웃: [uint16 BE 헤더길이 H][H바이트 UTF-8 JSON 헤더][바이너리 페이로드]
포인트클라우드는 Float32(xyz[+intensity], LE), 압축 이미지는 원본 바이트를 payload로 싣는다.
"""
import json
import struct
import time

import numpy as np
from sensor_msgs_py import point_cloud2


def build_binary_frame(header, payload):
    header_bytes = json.dumps(header).encode("utf-8")
    return struct.pack(">H", len(header_bytes)) + header_bytes + payload


def pointcloud_to_xyz(msg):
    field_names = [f.name for f in msg.fields]
    names = ("x", "y", "z", "intensity") if "intensity" in field_names else ("x", "y", "z")

    try:
        arr = point_cloud2.read_points_numpy(msg, field_names=names, skip_nans=True)
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(-1, len(names))
    except Exception:
        # 구버전 sensor_msgs_py 폴백(느림)
        pts = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
        arr = np.array([tuple(p) for p in pts], dtype=np.float32)
        if arr.size == 0:
            arr = arr.reshape(-1, len(names))

    return arr, list(names)


def encode_pointcloud(vehicle_id, topic, msg):
    arr, names = pointcloud_to_xyz(msg)

    payload = np.ascontiguousarray(arr, dtype="<f4").tobytes()

    header = {
        "type": "sensor_data",
        "vehicle_id": vehicle_id,
        "topic": topic,
        "msg_type": "sensor_msgs/msg/PointCloud2",
        "fields": names,
        "count": int(arr.shape[0]),
        "sent_at": time.time() * 1000,
    }
    return build_binary_frame(header, payload)


def encode_compressed_image(vehicle_id, topic, msg):
    payload = bytes(msg.data)  # 이미 압축된 jpeg/png 바이트

    header = {
        "type": "sensor_data",
        "vehicle_id": vehicle_id,
        "topic": topic,
        "msg_type": "sensor_msgs/msg/CompressedImage",
        "format": msg.format,
        "sent_at": time.time() * 1000,
    }
    return build_binary_frame(header, payload)
