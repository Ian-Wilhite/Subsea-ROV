"""ROS 2 camera pipeline node for CSI/USB sources."""

from __future__ import annotations

import os
from typing import Tuple, Union

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

class CameraPipelineNode(Node):
    """Capture CSI/USB camera frames via OpenCV and publish sensor_msgs/Image messages."""

    def __init__(self) -> None:
        super().__init__('camera_pipeline')
        self._image_topic: str = self.declare_parameter(
            'image_topic',
            '/obc/camera/front/image_raw',
        ).value
        self._frame_id: str = self.declare_parameter(
            'frame_id',
            'camera_front_optical_frame',
        ).value
        self._camera_device: Union[int, str] = self.declare_parameter('camera_device', 0).value
        self._gstreamer_pipeline: str = self.declare_parameter('gstreamer_pipeline', '').value
        self._width: int = int(self.declare_parameter('width', 1280).value)
        self._height: int = int(self.declare_parameter('height', 720).value)
        self._fps: float = float(self.declare_parameter('fps', 30.0).value)
        self._exposure: float = float(self.declare_parameter('exposure', -1.0).value)
        self._encoding: str = self.declare_parameter('encoding', 'bgr8').value
        timer_period = 1.0 / self._fps if self._fps > 0.0 else 0.01

        self._publisher = self.create_publisher(
            Image,
            self._image_topic,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._capture = self._open_capture()
        self._capture_timer = self.create_timer(timer_period, self._publish_frame)

    def _open_capture(self) -> cv2.VideoCapture:
        pipeline = str(self._gstreamer_pipeline).strip()
        if pipeline:
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            device = self._camera_device
            if isinstance(device, str):
                try:
                    device = int(device)
                except ValueError:
                    pass
            backend = cv2.CAP_V4L2 if os.name != 'nt' else cv2.CAP_ANY
            capture = cv2.VideoCapture(device, backend)
        if not capture.isOpened():
            raise RuntimeError('Failed to open camera input')

        if self._width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._width))
        if self._height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._height))
        if self._fps > 0.0:
            capture.set(cv2.CAP_PROP_FPS, float(self._fps))
        if self._exposure >= 0.0:
            capture.set(cv2.CAP_PROP_EXPOSURE, float(self._exposure))
        return capture

    def _publish_frame(self) -> None:
        if self._capture is None:
            return
        ok, frame = self._capture.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to capture frame, reinitializing camera')
            self._reinitialize_capture()
            return

        prepared_frame, encoding = self._prepare_frame(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = prepared_frame.shape[0]
        msg.width = prepared_frame.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = prepared_frame.strides[0]
        msg.data = prepared_frame.tobytes()
        self._publisher.publish(msg)

    def _prepare_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, str]:
        encoding = self._encoding.lower()
        if encoding == 'mono8':
            processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        elif encoding == 'rgb8':
            processed = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            processed = frame
            encoding = 'bgr8'

        return np.ascontiguousarray(processed), encoding

    def _reinitialize_capture(self) -> None:
        if self._capture is not None:
            self._capture.release()
        try:
            self._capture = self._open_capture()
        except RuntimeError as exc:
            self.get_logger().error(f'Camera reopen failed: {exc}')
            self._capture = None

    def destroy_node(self) -> bool:
        if hasattr(self, '_capture_timer') and self._capture_timer is not None:
            self._capture_timer.cancel()
            self._capture_timer = None
        if hasattr(self, '_capture') and self._capture is not None:
            self._capture.release()
            self._capture = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPipelineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
