"""Explicit ROS 2 executor ownership, including GUI main-thread integration."""
from contextlib import contextmanager
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, ShutdownException


def spin_node(node, threads=8):
    executor = MultiThreadedExecutor(num_threads=threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, ShutdownException):
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)


@contextmanager
def background_executor(node, threads=8):
    """Keep OpenCV/Qt events on the main thread while ROS callbacks run."""
    executor = MultiThreadedExecutor(num_threads=threads)
    executor.add_node(node)
    failure = []

    def run():
        try:
            executor.spin()
        except (ExternalShutdownException, ShutdownException):
            pass
        except Exception as error:
            failure.append(error)
            if rclpy.ok():
                rclpy.try_shutdown()

    thread = threading.Thread(target=run, name="flyco_ros2_callbacks")
    thread.start()
    try:
        yield executor
    finally:
        executor.shutdown()
        thread.join()
        executor.remove_node(node)
    if failure:
        raise RuntimeError("A ROS 2 callback failed") from failure[0]


def main_for(node_factory, *args, gui=False, **kwargs):
    rclpy.init()
    node = None
    try:
        node = node_factory(*args, **kwargs)
        if gui:
            with background_executor(node):
                node.run()
        else:
            spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
