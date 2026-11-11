"""Optional real DDS smoke test. Run after sourcing ROS2; not part of pytest.

python learning/tests/ros_smoke.py --checkpoint outputs/.../model.pt
Use an isolated ROS_DOMAIN_ID when running alongside other applications.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as ROSPath
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_limm.runtime import StreamingPredictor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True)
    args = parser.parse_args()
    runtime = StreamingPredictor.from_checkpoint(args.checkpoint)
    import torch
    torch.set_num_threads(1)
    config = runtime.model.config
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])+os.pathsep+environment.get("PYTHONPATH", "")
    child = subprocess.Popen([sys.executable, "-m", "uav_limm.ros_node", "--ros-args",
                              "-p", f"checkpoint:={Path(args.checkpoint).resolve()}",
                              "-p", "observation_topic:=/limm_test/observation",
                              "-p", "prediction_topic:=/limm_test/prediction"], env=environment)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("limm_smoke_observer")
        publisher = node.create_publisher(Odometry, "/limm_test/observation", 50)
        received = []
        subscription = node.create_subscription(ROSPath, "/limm_test/prediction", received.append, 10)
        def spin_until(condition, seconds=15):
            deadline = time.monotonic()+seconds
            while not condition() and time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError(f"ROS node exited: {child.returncode}")
                rclpy.spin_once(node, timeout_sec=.05)
            assert condition(), "Timed out waiting for ROS node"
        spin_until(lambda: publisher.get_subscription_count() > 0)
        for i in range(config.observation):
            stamp = 100+i*config.dt
            position = np.array([np.sin(i*.1), np.cos(i*.07), i*.05])+[100., -200., 30.]
            message = Odometry()
            message.header.stamp = Time(nanoseconds=round(stamp*1e9)).to_msg()
            message.header.frame_id = "map"
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, position)
            publisher.publish(message)
            runtime.update(stamp, position)
            rclpy.spin_once(node, timeout_sec=.01)
        spin_until(lambda: bool(received))
        path = received[-1]
        assert len(path.poses) == config.horizon
        assert path.header.frame_id == "map"
        actual = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses])
        np.testing.assert_allclose(actual, runtime.predict(), atol=1e-5, rtol=1e-6)
        for i, pose in enumerate(path.poses):
            assert isinstance(pose, PoseStamped)
            stamp = Time.from_msg(pose.header.stamp).nanoseconds/1e9
            assert abs(stamp - (100+(config.observation+i)*config.dt)) < 1e-6
        print(f"ROS DDS smoke passed: {config.observation} observations -> {config.horizon} finite poses, matches offline inference")
    finally:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
