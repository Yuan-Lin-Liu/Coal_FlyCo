#!/usr/bin/env python3
"""Display semantic segmentation on its timestamp-matched camera image."""
import time
from collections import OrderedDict

import cv2
import numpy as np


def render_overlay(rgb, mask=None, status='Waiting for segmentation'):
    image = rgb.copy()
    if mask is not None:
        selected = mask > 0
        image[selected] = (image[selected] * .55 + np.array([235, 70, 180]) * .45).astype(np.uint8)
        contours, _ = cv2.findContours(selected.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (255, 145, 215), 1)
        status = 'Target detected' if selected.any() else 'No reliable mask'
    cv2.rectangle(image, (0, 0), (min(image.shape[1], 325), 31), (22, 26, 32), -1)
    cv2.putText(image, status, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (245, 245, 245), 1, cv2.LINE_AA)
    return image


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class PerceptionImage(Node):
        def __init__(self):
            super().__init__('semantic_perception_image')
            self.bridge = CvBridge()
            self.images = OrderedDict()
            self.pending = OrderedDict()
            self.latest = None
            self.last_result = 0.
            self.publisher = self.create_publisher(Image, '/perception/segmentation/overlay', 1)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, '/isaac/camera/rgb', self.image_callback, qos)
            self.create_subscription(Image, '/perception/segmentation/mask', self.mask_callback, 5)
            self.create_timer(.5, self.status_callback)

        @staticmethod
        def stamp(message):
            return (message.header.stamp.sec, message.header.stamp.nanosec)

        def image_callback(self, message):
            stamp = self.stamp(message)
            self.images[stamp] = message
            self.latest = message
            while len(self.images) > 60:
                self.images.popitem(last=False)
            self.match(stamp)

        def mask_callback(self, message):
            stamp = self.stamp(message)
            self.pending[stamp] = message
            while len(self.pending) > 5:
                self.pending.popitem(last=False)
            self.match(stamp)

        def match(self, stamp):
            if stamp not in self.images or stamp not in self.pending:
                return
            raw = self.images[stamp]
            message = self.pending.pop(stamp)
            mask = self.bridge.imgmsg_to_cv2(message, 'mono8')
            if mask.ndim != 2:
                self.get_logger().warning('Invalid segmentation image; preview skipped')
                return
            rgb = self.bridge.imgmsg_to_cv2(raw, 'rgb8')
            if mask.shape != rgb.shape[:2]:
                mask = cv2.resize(mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
            self.publish(render_overlay(rgb, mask), raw.header)
            self.last_result = time.monotonic()

        def publish(self, image, header):
            message = self.bridge.cv2_to_imgmsg(image, 'rgb8')
            message.header = header
            self.publisher.publish(message)

        def status_callback(self):
            if self.latest is not None and time.monotonic() - self.last_result > 2.5:
                rgb = self.bridge.imgmsg_to_cv2(self.latest, 'rgb8')
                self.publish(render_overlay(rgb), self.latest.header)

    rclpy.init()
    node = PerceptionImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
