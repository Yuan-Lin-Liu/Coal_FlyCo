"""Subscribe to ground ROS images and text, and publish local segmentation."""
import base64
import copy
import json
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .semantic_ipc import FrameMailbox, WorkerClient, WorkerStopped


def result_images(original, mask_png):
    """Keep image coordinates and timestamps; never interpolate binary labels."""
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(original, desired_encoding='bgr8')
    if mask_png is None:
        mask = np.zeros(bgr.shape[:2], np.uint8)
    else:
        mask = cv2.imdecode(np.frombuffer(mask_png, np.uint8), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError('Cannot decode result mask')
        if mask.shape != bgr.shape[:2]:
            mask = cv2.resize(mask, (original.width, original.height), interpolation=cv2.INTER_NEAREST)
        mask = (mask > 0).astype(np.uint8) * 255
    overlay = bgr.copy()
    selected = mask > 0
    overlay[selected] = (bgr[selected].astype(np.float32) * .6 +
                         np.array([0., 255., 0.]) * .4).astype(np.uint8)
    binary_message = bridge.cv2_to_imgmsg(mask, 'mono8')
    overlay_message = bridge.cv2_to_imgmsg(overlay, 'bgr8')
    binary_message.header = copy.deepcopy(original.header)
    overlay_message.header = copy.deepcopy(original.header)
    return binary_message, overlay_message, int(np.count_nonzero(mask))


class SemanticImageNode(Node):
    def __init__(self):
        super().__init__('ground_segmentation')
        def param(name, default):
            return self.declare_parameter(name, default).value

        self.image_topic = param('image_topic', '/odin1/image')
        self.text_topic = param('text_topic', '/query_text')
        source_dir = param('source_dir', '')
        model_python = param('model_python', '')
        self.startup_timeout = float(param('startup_timeout', 180.))
        self.inference_timeout = float(param('inference_timeout', 60.))
        self.max_fps = float(param('max_fps', 2.))
        status_period = float(param('status_period', 2.))
        if min(self.startup_timeout, self.inference_timeout, self.max_fps, status_period) <= 0:
            raise ValueError('Timeouts, max_fps, and status_period must be positive')
        self.mask_pub = self.create_publisher(Image, param('mask_topic', '/perception/segmentation/mask'), 1)
        self.overlay_pub = self.create_publisher(Image, param('overlay_topic', '/perception/segmentation/overlay'), 1)
        self.status_pub = self.create_publisher(String, param('status_topic', '/perception/segmentation/status'), 1)
        self.mailbox = FrameMailbox()
        self.mailbox.set_text(param('initial_text', ''))
        self.bridge = CvBridge()
        self._client = None
        self._client_lock = threading.Lock()
        self._ready = False
        self._fatal = ''
        self._last_error = ''
        self._received = self._calls = self._published = self._pixels = 0
        self._latency = 0.
        self._phase = 'loading_model'
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, self.text_topic, self.on_text, 10)
        self.create_timer(status_period, self.publish_status)
        self.get_logger().info('Local segmentation: image=%s text=%s; loading model in its uv environment' %
                               (self.image_topic, self.text_topic))
        self._thread = threading.Thread(target=self._work, args=(model_python, source_dir),
                                        name='local-segmentation', daemon=True)
        self._thread.start()

    def on_image(self, message):
        self._received += 1
        self.mailbox.offer(message)

    def on_text(self, message):
        if self.mailbox.set_text(message.data):
            self._last_error = ''
            self._phase = 'waiting_for_image' if message.data.strip() else 'waiting_for_text'
            self.get_logger().info('Segmentation target: ' + (message.data.strip() or '(cleared)'))

    def _work(self, python, source_dir):
        try:
            client = WorkerClient(python, source_dir)
            with self._client_lock:
                self._client = client
                if self.mailbox.closed:
                    client.close()
                    return
            ready = client.wait_ready(self.startup_timeout)
            self._ready = True
            self.get_logger().info('Local segmentation model ready: ' + json.dumps(ready['model'], ensure_ascii=False))
            next_frame = 0.
            while not self.mailbox.closed:
                # Rate limiting happens before taking the newest pending image.
                delay = next_frame - time.monotonic()
                if delay > 0:
                    time.sleep(min(delay, .1))
                    continue
                job = self.mailbox.take(timeout=.2)
                if job is None:
                    continue
                self._phase = 'running'
                try:
                    bgr = self.bridge.imgmsg_to_cv2(job.frame, desired_encoding='bgr8')
                    ok, encoded = cv2.imencode('.png', bgr)
                    if not ok:
                        raise ValueError('Cannot encode input image')
                    self._calls += 1
                    stamp = job.frame.header.stamp
                    result = client.segment({
                        'kind': 'frame', 'id': self._calls, 'prompt_id': job.prompt_id, 'text': job.text,
                        'header': {'sec': stamp.sec, 'nanosec': stamp.nanosec,
                                   'frame_id': job.frame.header.frame_id},
                        'image_png': base64.b64encode(encoded).decode('ascii')}, self.inference_timeout)
                    mask_data = base64.b64decode(result['mask_png'], validate=True) if result['mask_png'] else None
                    mask, overlay, pixels = result_images(job.frame, mask_data)
                    def publish():
                        self.mask_pub.publish(mask)
                        self.overlay_pub.publish(overlay)
                        self._published += 1
                        self._pixels = pixels
                        self._latency = result['inference_ms']
                        self._phase = 'tracking' if pixels else 'no_target'
                        self._last_error = ''
                    self.mailbox.commit(job, publish)
                except WorkerStopped:
                    raise
                except Exception as error:
                    self._last_error = str(error)
                    self.get_logger().error('Segmentation frame failed: ' + str(error))
                next_frame = time.monotonic() + 1. / self.max_fps
        except Exception as error:
            if not self.mailbox.closed:
                self._fatal = str(error)
                self.get_logger().error('Local segmentation stopped: ' + str(error))
        finally:
            with self._client_lock:
                if self._client is not None:
                    self._client.close()

    def publish_status(self):
        state = self._phase
        if self._fatal or self._last_error:
            state = 'error'
        elif not self._ready:
            state = 'loading_model'
        elif not self.mailbox.text:
            state = 'waiting_for_text'
        elif not self._received:
            state = 'waiting_for_image'
        status = dict(state=state, image_topic=self.image_topic, text=self.mailbox.text,
                      prompt_id=self.mailbox.prompt_id, images_received=self._received,
                      inference_calls=self._calls, published_frames=self._published,
                      mask_pixels=self._pixels, inference_ms=round(self._latency, 1),
                      error=self._fatal or self._last_error)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self.get_logger().info('Segmentation status: state=%s images=%d inference_calls=%d published=%d '
                               'mask_pixels=%d inference_ms=%.1f' %
                               (state, self._received, self._calls, self._published, self._pixels, self._latency))
        if self._fatal:
            self.context.try_shutdown()

    def close(self):
        self.mailbox.close()
        with self._client_lock:
            if self._client is not None:
                self._client.close()
        self._thread.join(timeout=6)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticImageNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        failed = bool(node and node._fatal)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return int(failed)
