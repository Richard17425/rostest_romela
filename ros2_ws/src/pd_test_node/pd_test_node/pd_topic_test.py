#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_msgs.msg import Bool,Float32MultiArray, Int64, Int8

class PdTopicTest(Node):
    def __init__(self):
        super().__init__('pd_topic_test')  # <--- This is the node name very important
        self.player_states2_pub = self.create_publisher(Pose2D, 'player_states2', 10)
        self.get_logger().info("Publisher initialized.")

        self.my_cur_pose_w = np.array([0.0, 0.0, 0.0])  # or actual starting pose
        self.dt = 0.01  # or whatever your time step is
        self.cmd_vel_msg = None  # etc.

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, 'C1_traj', self.c1_traj_callback, 10)
        # self.create_subscription(Float32MultiArray, 'LP_traj', self.lp_traj_callback, 10)
        # self.create_subscription(Float32MultiArray, 'mpc_traj', self.mpc_traj_callback, 10)
        # self.create_subscription(Twist, 'oppo_radium', self.radium_callback, 10)
        # self.create_subscription(Pose2D, 'adj_des_pose_rel', self.adj_pose_callback, 10)
        # self.create_subscription(Pose2D, 'offset_ball_pose', self.offset_ball_callback, 10)
        # self.create_subscription(Bool, 'LP_path_success', self.lp_success_callback, 10)
        # self.create_subscription(Bool, 'MPC_success', self.mpc_success_callback, 10)
        # self.create_subscription(Int64, 'midlevel_approach', self.approach_callback, 10)

        self.update_player_states()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg
        print("Cmd Vel: ", msg.linear.x, msg.linear.y, msg.angular.z)

    def c1_traj_callback(self, msg):
        self.c1_traj_msg = msg
        print("C1 Trajectory: ", msg.data)

    def update_player_states(self):
        if self.cmd_vel_msg is None or self.my_cur_pose_w is None:
            print("Waiting for cmd_vel_msg or my_cur_pose_w to be set.")
            return  # wait until data is ready

        vel = np.array([
            self.cmd_vel_msg.linear.x,
            self.cmd_vel_msg.linear.y,
            self.cmd_vel_msg.angular.z
        ])

        theta = self.my_cur_pose_w[2]
        jaco = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

        self.my_cur_pose_w = self.my_cur_pose_w + jaco @ vel * self.dt
        player_states2 = self.my_cur_pose_w

        # Publish player_states2
        msg = Pose2D()
        msg.x = player_states2[0]
        msg.y = player_states2[1]
        msg.theta = player_states2[2]
        self.player_states2_pub.publish(msg)


def main(args=None):
    print("Node is running")
    rclpy.init(args=args)
    node = PdTopicTest()
    rclpy.spin(node) # Keep the node running
    node.destroy_node() # Destroy the node when done
    rclpy.shutdown()


if __name__ == '__main__':
    main()