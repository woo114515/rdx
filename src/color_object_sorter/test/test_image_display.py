import cv2
import numpy as np

from color_object_sorter.image_display import decode_jpeg, fit_image


def test_decode_jpeg_round_trip() -> None:
    source = np.full((20, 30, 3), (10, 80, 200), dtype=np.uint8)
    success, encoded = cv2.imencode(".jpg", source)
    assert success
    decoded = decode_jpeg(encoded.tobytes())
    assert decoded is not None
    assert decoded.shape == source.shape


def test_decode_jpeg_rejects_invalid_data() -> None:
    assert decode_jpeg(b"") is None
    assert decode_jpeg(b"not a jpeg") is None


def test_fit_image_preserves_aspect_ratio() -> None:
    image = np.zeros((600, 800, 3), dtype=np.uint8)
    fitted = fit_image(image, 400, 400)
    assert fitted.shape == (300, 400, 3)


def test_fit_image_does_not_enlarge() -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    assert fit_image(image, 400, 400) is image
