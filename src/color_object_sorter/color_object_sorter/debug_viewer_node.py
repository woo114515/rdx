"""OpenCV viewer for the detector's compressed debug stream."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .image_display import decode_jpeg, fit_image


class ColorSorterDebugViewer(Node):
    """Display annotated perception frames without publishing commands."""

    def __init__(self) -> None:
        super().__init__("color_sorter_debug_viewer")
        self.declare_parameter("image_topic", "/color_sorter/debug/compressed")
        self.declare_parameter("window_title", "Color sorter detections")
        self.declare_parameter("maximum_width", 1280)
        self.declare_parameter("maximum_height", 720)
        self.declare_parameter("image_timeout", 2.0)
        self._window_title = str(self.get_parameter("window_title").value)
        self._maximum_width = int(self.get_parameter("maximum_width").value)
        self._maximum_height = int(self.get_parameter("maximum_height").value)
        self._image_timeout = float(self.get_parameter("image_timeout").value)
        if self._image_timeout <= 0.0:
            raise ValueError("image_timeout must be positive")
        if self._maximum_width < 1 or self._maximum_height < 1:
            raise ValueError("display bounds must be positive")
        self._lock = threading.Lock()
        self._image: np.ndarray | None = None
        self._last_image_monotonic: float | None = None
        self._quit_requested = False
        topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(
            CompressedImage, topic, self._on_image, qos_profile_sensor_data
        )
        cv2.namedWindow(self._window_title, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            f"Waiting for {topic}; press q or Escape in the window to quit"
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def _on_image(self, message: CompressedImage) -> None:
        image = decode_jpeg(message.data)
        if image is None:
            self.get_logger().warning("JPEG decode failed")
            return
        with self._lock:
            self._image = image
            self._last_image_monotonic = time.monotonic()

    def _render(self) -> None:
        with self._lock:
            image = None if self._image is None else self._image.copy()
            last_image = self._last_image_monotonic
        if image is None:
            image = self._status_image("Waiting for annotated image...")
        elif (
            last_image is not None
            and time.monotonic() - last_image > self._image_timeout
        ):
            cv2.putText(
                image, "IMAGE STALE", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2, cv2.LINE_AA,
            )
        cv2.imshow(
            self._window_title,
            fit_image(image, self._maximum_width, self._maximum_height),
        )
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            self._quit_requested = True

    @staticmethod
    def _status_image(text: str) -> np.ndarray:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            image, text, (40, 240), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2, cv2.LINE_AA,
        )
        return image

    def close(self) -> None:
        cv2.destroyWindow(self._window_title)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ColorSorterDebugViewer | None = None
    try:
        node = ColorSorterDebugViewer()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
