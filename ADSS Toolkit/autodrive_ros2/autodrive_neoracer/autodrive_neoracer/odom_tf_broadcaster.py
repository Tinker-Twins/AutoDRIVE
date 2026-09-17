#!/usr/bin/env python3

# Ground-truth odom -> base_footprint TF for the twin.
#
# On the car this transform is owned by the EKF, which exists to estimate the
# true pose from imperfect wheel odometry and a noisy IMU. The simulator's
# /odom IS the true pose, and estimation layers tuned for real sensor
# pathologies misbehave on noiseless data (the complementary filter's bias
# estimator absorbs sustained gentle turns - see the twin devkit notes). The
# twin therefore broadcasts TF straight from /odom; the EKF still runs for
# topic parity but with publish_tf off (twin_autonomy.launch.py).

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
