"""Explicit ROS 2 executor ownership, including GUI main-thread integration."""
from contextlib import contextmanager
import threading
import signal
from rclpy.signals import SignalHandlerOptions

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, ShutdownException


def spin_node(node, threads=8, stop=None):
    executor = MultiThreadedExecutor(num_threads=threads)
    executor.add_node(node)
    try:
        if stop is None:
            executor.spin()
        else:
            while rclpy.ok() and not stop.is_set():
                executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException, ShutdownException):
        pass
    finally:
        # Humble's MultiThreadedExecutor.shutdown() does not join its thread pool.
        # Drain callbacks before destroying publishers or their ROS context.
        executor._executor.shutdown(wait=True)
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
    stop = threading.Event()
    previous_handlers = {}
    if gui:
        rclpy.init()
    else:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        for number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[number] = signal.signal(number, lambda *_: stop.set())
    node = None
    try:
        node = node_factory(*args, **kwargs)
        if gui:
            with background_executor(node):
                node.run()
        else:
            spin_node(node, stop=stop)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
