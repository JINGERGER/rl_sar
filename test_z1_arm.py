#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class TrajectoryCaller(Node):

    def __init__(self):
        super().__init__("joint_trajectory_caller")

        self.get_logger().info("Creating client")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",  # Changed from joint_trajectory_controller
        )
        self.action_client.wait_for_server()
        self.get_logger().info("Connection established")

    def send_traj_request(self, positions: list[np.ndarray]):
        self.get_logger().info("Creating trajectory request")
        traj_msg = FollowJointTrajectory.Goal()
        traj: JointTrajectory = traj_msg.trajectory
        traj.joint_names = [
            "z1_joint1",  # Changed from joint1
            "z1_joint2",
            "z1_joint3",
            "z1_joint4",
            "z1_joint5",
            "z1_joint6",
        ]
        for i, pos in enumerate(positions):
            self.get_logger().info(f"{pos}")
            pt = JointTrajectoryPoint()
            pt.positions = pos.tolist()
            pt.time_from_start.sec = (i+1) * 2
            traj.points.append(pt)

        self.get_logger().info("Sending trajectory request")
        return self.action_client.send_goal_async(traj_msg)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryCaller()

    future = node.send_traj_request([
        np.array([0.0, 0.02, -0.02, 0.0, 0.0, 0.0]),
        np.array([0.1, 1.0, -1.0, 0.0, 0.0, 0.0]),
        np.array([np.pi / 4, 1.0, -1.0, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.02, -0.05, 0.0, 0.0, 0.0]),
    ])
    rclpy.spin_until_future_complete(node, future)


if __name__ == '__main__':
    main()
