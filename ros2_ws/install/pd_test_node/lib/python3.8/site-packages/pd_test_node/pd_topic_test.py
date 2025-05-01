#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose2D
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_msgs.msg import Bool,Float32MultiArray, Int64, Int8
import numpy as np

class PdTopicTest(Node):
    def __init__(self):
        super().__init__('pd_topic_test')  # <--- This is the node name very important
        self.player_states2_pub = self.create_publisher(Float32MultiArray, 'player_states2', 10)
        self.get_logger().info("Publisher initialized.")

        self.my_cur_pose_w = np.array([0.0, 0.0, 0.0])  # or actual starting pose
        self.dt = 0.25  # or whatever your time step is
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
        print("Subscribers initialized.")

        # self.update_player_states()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg
        print("Cmd Vel: ", msg.linear.x, msg.linear.y, msg.angular.z)
        self.update_player_states()

    def c1_traj_callback(self, msg):
        self.c1_traj_msg = msg
        # print("C1 Trajectory: ", msg.data)
        
        traj_np = np.array(msg.data).reshape(-1, 3)
    
        print("[midlevel_conti_PDtest-1] C1_traj: [")
        for row in traj_np:
            print(f"[{row[0]: .8f}  {row[1]: .8f}  {row[2]: .8f}]")
        print("]")

    def update_player_states(self):
        if self.cmd_vel_msg is None or self.my_cur_pose_w is None:
            print("Waiting for cmd_vel_msg or my_cur_pose_w to be set.")
            return  # wait until data is ready
        
        print("update_player_states called, msg received.")

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
        self.my_cur_pose_w[2] = np.arctan2(np.sin(self.my_cur_pose_w[2]), np.cos(self.my_cur_pose_w[2]))  # normalize theta


        # Publish player_states2
        msg = Float32MultiArray()
        matrix = np.zeros((6, 5), dtype=np.float32)

        # Fill matrix[0] with [x, y, theta, confidence, radius]
        matrix[0, 0:3] = self.my_cur_pose_w
        matrix[0, 3] = 1.0  # confidence
        matrix[0, 4] = 0.09  # radius or whatever value fits your system


        for i in range(1, 4):
            matrix[i, :3] = [-1.0 - i, -2.0 - i, 0.0]  # placeholder
            matrix[i, 3] = 0.8  # confidence
            matrix[i, 4] = 0.18

        # Rows 4-5: goals — dummy data
        matrix[4, :2] = [7.5, 0.9]  # goal 1
        matrix[4, 3] = 1.0
        matrix[5, :2] = [7.5, -0.9]  # goal 2
        matrix[5, 3] = 1.0

        msg.data = matrix.flatten().tolist()


        # Define and append layout dimensions
        dim0 = MultiArrayDimension()
        dim0.label = "rows"
        dim0.size = 6
        dim0.stride = 30  # 6*5

        dim1 = MultiArrayDimension()
        dim1.label = "cols"
        dim1.size = 5
        dim1.stride = 5

        msg.layout.dim.append(dim0)
        msg.layout.dim.append(dim1)

        print('published message -player_states:', matrix)
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