"""Interactive HSV sampler for the Task 3 compressed camera stream."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .sampling import HsvPixel, extract_hsv_patch


LABEL_KEYS = {
    ord("b"): "blue",
    ord("g"): "green",
    ord("r"): "red",
    ord("y"): "yellow_distractor",
    ord("n"): "background",
}


@dataclass(frozen=True)
class SampleBatch:
    sample_id: int
    label: str
    stamp_sec: int
    stamp_nanosec: int
    center_x: int
    center_y: int
    image_width: int
    image_height: int
    pixels: tuple[HsvPixel, ...]


class HsvSamplerNode(Node):
    """Display compressed images and record labelled HSV patches on clicks."""

    def __init__(self) -> None:
        super().__init__("color_object_hsv_sampler")
        self.declare_parameter("image_topic", "/csi/image_raw/compressed")
        self.declare_parameter("output_file", "")
        self.declare_parameter("patch_radius", 5)
        self._patch_radius = int(self.get_parameter("patch_radius").value)
        if self._patch_radius < 0:
            raise ValueError("patch_radius must be non-negative")

        requested_output = str(self.get_parameter("output_file").value)
        self._output_path = self._resolve_output_path(requested_output)
        self._window_name = "HSV sampler - color_object_sorter"
        self._lock = threading.Lock()
        self._image: np.ndarray | None = None
        self._stamp_sec = 0
        self._stamp_nanosec = 0
        self._label = "blue"
        self._batches: list[SampleBatch] = []
        self._next_sample_id = 1
        self._saved = False
        self._quit_requested = False

        topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(
            CompressedImage,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self._window_name, self._on_mouse)
        self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            f"Sampling {topic}; output={self._output_path}; "
            "keys: b/g/r/y/n, u=undo, s=save, q=save and quit"
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def _on_image(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("JPEG decode failed")
            return
        with self._lock:
            self._image = image
            self._stamp_sec = message.header.stamp.sec
            self._stamp_nanosec = message.header.stamp.nanosec

    def _on_mouse(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        data: object,
    ) -> None:
        del flags, data
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self._lock:
            if self._image is None:
                return
            pixels = extract_hsv_patch(
                self._image,
                x,
                y,
                self._patch_radius,
            )
            height, width = self._image.shape[:2]
            batch = SampleBatch(
                sample_id=self._next_sample_id,
                label=self._label,
                stamp_sec=self._stamp_sec,
                stamp_nanosec=self._stamp_nanosec,
                center_x=x,
                center_y=y,
                image_width=width,
                image_height=height,
                pixels=pixels,
            )
            self._batches.append(batch)
            self._next_sample_id += 1
            self._saved = False
        median = np.median(
            [(pixel.hue, pixel.saturation, pixel.value) for pixel in pixels],
            axis=0,
        )
        self.get_logger().info(
            f"sample={batch.sample_id} label={batch.label} center=({x},{y}) "
            f"pixels={len(pixels)} median_hsv={median.round(1).tolist()}"
        )

    def _render(self) -> None:
        with self._lock:
            image = None if self._image is None else self._image.copy()
            label = self._label
            sample_count = len(self._batches)
        if image is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                image,
                "Waiting for compressed image...",
                (30, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
        status = f"label={label} samples={sample_count} radius={self._patch_radius}"
        help_text = (
            "b/g/r target  y yellow-noise  n background  "
            "u undo  s save  q quit"
        )
        cv2.rectangle(image, (0, 0), (image.shape[1], 58), (0, 0, 0), -1)
        cv2.putText(
            image,
            status,
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            help_text,
            (10, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow(self._window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in LABEL_KEYS:
            self._label = LABEL_KEYS[key]
            self.get_logger().info(f"active label: {self._label}")
        elif key == ord("u"):
            self._undo()
        elif key == ord("s"):
            self.save()
        elif key in (ord("q"), 27):
            self.save()
            self._quit_requested = True

    def _undo(self) -> None:
        with self._lock:
            if not self._batches:
                self.get_logger().info("nothing to undo")
                return
            removed = self._batches.pop()
            self._saved = False
        self.get_logger().info(
            f"removed sample={removed.sample_id} label={removed.label}"
        )

    def save(self) -> None:
        with self._lock:
            batches = tuple(self._batches)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._output_path.with_suffix(
            self._output_path.suffix + ".tmp"
        )
        with temporary.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "sample_id",
                    "label",
                    "stamp_sec",
                    "stamp_nanosec",
                    "center_x",
                    "center_y",
                    "image_width",
                    "image_height",
                    "offset_x",
                    "offset_y",
                    "h",
                    "s",
                    "v",
                )
            )
            for batch in batches:
                for pixel in batch.pixels:
                    writer.writerow(
                        (
                            batch.sample_id,
                            batch.label,
                            batch.stamp_sec,
                            batch.stamp_nanosec,
                            batch.center_x,
                            batch.center_y,
                            batch.image_width,
                            batch.image_height,
                            pixel.offset_x,
                            pixel.offset_y,
                            pixel.hue,
                            pixel.saturation,
                            pixel.value,
                        )
                    )
        temporary.replace(self._output_path)
        self._saved = True
        self.get_logger().info(
            f"saved {len(batches)} sample patches to {self._output_path}"
        )

    def close(self) -> None:
        if not self._saved and self._batches:
            self.save()
        cv2.destroyAllWindows()

    @staticmethod
    def _resolve_output_path(requested: str) -> Path:
        if requested:
            return Path(requested).expanduser().resolve()
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        return Path.cwd() / f"hsv_samples_{timestamp}.csv"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: HsvSamplerNode | None = None
    try:
        node = HsvSamplerNode()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
