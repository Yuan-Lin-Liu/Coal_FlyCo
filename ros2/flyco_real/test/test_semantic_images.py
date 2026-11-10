"""Ground segmentation transport and image contracts, without neural inference."""
import importlib
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros2/flyco_real'))


def test_pending_images_are_replaced_and_old_prompt_results_are_rejected():
    mailbox = importlib.import_module('flyco_real.semantic_ipc').FrameMailbox()
    mailbox.set_text('a gray chair')
    first, latest = object(), object()
    mailbox.offer(first)
    mailbox.offer(latest)
    job = mailbox.take(timeout=.01)
    assert job.frame is latest
    assert job.text == 'a gray chair'
    assert mailbox.is_current(job)
    published = []
    assert mailbox.commit(job, lambda: published.append(job.frame))
    assert published == [latest]
    mailbox.set_text('a red bag')
    assert not mailbox.is_current(job)
    assert not mailbox.commit(job, lambda: published.append(first))
    assert published == [latest]
    assert mailbox.take(timeout=.01) is None
    mailbox.close()


def test_images_wait_for_text_and_clearing_text_invalidates_inflight_work():
    mailbox = importlib.import_module('flyco_real.semantic_ipc').FrameMailbox()
    mailbox.offer(object())
    assert mailbox.take(timeout=.01) is None
    mailbox.set_text('chair')
    frame = object()
    mailbox.offer(frame)
    job = mailbox.take(timeout=.01)
    assert mailbox.set_text(' chair ') is False
    assert mailbox.is_current(job)
    mailbox.set_text('')
    assert not mailbox.is_current(job)
    mailbox.close()


def test_result_images_preserve_the_original_header_and_resolution():
    import cv2
    from cv_bridge import CvBridge
    bridge = CvBridge()
    original = bridge.cv2_to_imgmsg(np.full((4, 6, 3), 30, np.uint8), 'bgr8')
    original.header.stamp.sec = 123
    original.header.stamp.nanosec = 456
    original.header.frame_id = 'camera_0'
    small_mask = np.array([[0, 255, 0], [0, 0, 0]], np.uint8)
    ok, encoded = cv2.imencode('.png', small_mask)
    assert ok
    module = importlib.import_module('flyco_real.semantic_image')
    mask, overlay, pixels = module.result_images(original, encoded.tobytes())
    assert (mask.height, mask.width, mask.encoding) == (4, 6, 'mono8')
    assert (overlay.height, overlay.width, overlay.encoding) == (4, 6, 'bgr8')
    assert mask.header == overlay.header == original.header
    assert pixels == 4
    result = bridge.imgmsg_to_cv2(mask)
    assert set(np.unique(result)) == {0, 255}
    colored = bridge.imgmsg_to_cv2(overlay)
    assert np.array_equal(colored[3, 0], [30, 30, 30])
    assert colored[0, 2, 1] > colored[0, 2, 0]


def test_empty_mask_publishes_an_unmodified_overlay():
    from cv_bridge import CvBridge
    bridge = CvBridge()
    image = np.full((3, 5, 3), 60, np.uint8)
    original = bridge.cv2_to_imgmsg(image, 'bgr8')
    module = importlib.import_module('flyco_real.semantic_image')
    mask, overlay, count = module.result_images(original, None)
    assert count == 0
    assert not np.any(bridge.imgmsg_to_cv2(mask))
    assert np.array_equal(bridge.imgmsg_to_cv2(overlay), image)


def test_invalid_mask_is_reported_instead_of_publishing_a_false_result():
    from cv_bridge import CvBridge
    original = CvBridge().cv2_to_imgmsg(np.zeros((2, 2, 3), np.uint8), 'bgr8')
    module = importlib.import_module('flyco_real.semantic_image')
    with pytest.raises(ValueError, match='mask'):
        module.result_images(original, b'not a PNG')
