"""Optional ROS2 position subscriber -> LIMM nav_msgs/Path publisher.

Launch after sourcing ROS2: python -m uav_limm.ros_node --ros-args ...
No rclpy dependency is imported by the standalone package.
"""
import math

from .runtime import StreamingPredictor


def main():
    import rclpy
    import torch
    from rclpy.node import Node
    from rclpy.time import Time
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped

    class LIMMNode(Node):
        def __init__(self):
            super().__init__("limm_predictor")
            checkpoint = self.declare_parameter("checkpoint", "").value
            device = self.declare_parameter("device", "cpu").value
            torch.set_num_threads(self.declare_parameter("threads", 1).value)
            self.predictor = StreamingPredictor.from_checkpoint(checkpoint, device)
            topic = self.declare_parameter("observation_topic", "/measurement_position").value
            output = self.declare_parameter("prediction_topic", "/limm/prediction").value
            self.next_sample = None
            self.publisher = self.create_publisher(Path, output, 10)
            self.subscription = self.create_subscription(Odometry, topic, self.receive, 50)
            cfg = self.predictor.model.config
            self.get_logger().info(f"LIMM: {cfg.observation} history samples, dt={cfg.dt}s, horizon={cfg.horizon}")

        def receive(self, message):
            timestamp = Time.from_msg(message.header.stamp).nanoseconds/1e9
            if not math.isfinite(timestamp):
                return
            dt = self.predictor.model.config.dt
            if self.next_sample is None:
                self.next_sample = timestamp
            # Choose first received measurement at each grid time, never interpolate
            # from a future message. Gaps flush history instead of inventing points.
            if timestamp < self.next_sample - 1e-6:
                return
            if timestamp-self.next_sample > dt*.05:
                self.predictor.reset()
                self.next_sample = timestamp
                self.get_logger().warning("Measurement gap: restarting LIMM warmup")
            p = message.pose.pose.position
            try:
                self.predictor.update(timestamp, [p.x, p.y, p.z])
            except ValueError as error:
                self.get_logger().warning(str(error))
                self.predictor.reset()
                self.next_sample = None
                return
            self.next_sample += dt
            if not self.predictor.ready:
                return
            positions = self.predictor.predict()
            path = Path()
            path.header = message.header
            for i, position in enumerate(positions):
                pose = PoseStamped()
                pose.header.frame_id = message.header.frame_id
                pose.header.stamp = Time(nanoseconds=round((timestamp+(i+1)*dt)*1e9)).to_msg()
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
                pose.pose.orientation.w = 1.
                path.poses.append(pose)
            self.publisher.publish(path)

    rclpy.init()
    node = None
    try:
        node = LIMMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
