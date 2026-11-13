#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
from scipy.io import savemat
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cartesian_interface_ros.action import ReachPose
from cartesian_interface_ros.srv import GetCartesianTaskInfo
from xbot2_interface import pyxbot2_interface as xb

TASK = 'ee_E'

rclpy.init()
node = Node('repeatability_test_node')

## create robot ifc
qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
_, urdf = wait_for_message(String, node, '/xbotcore/robot_description', qos_profile=qos, time_to_wait=5.0)
_, srdf = wait_for_message(String, node, '/xbotcore/robot_description_semantic', qos_profile=qos, time_to_wait=5.0)
if urdf is None or srdf is None:
    raise RuntimeError('failed to receive urdf / srdf')
robot = xb.RobotInterface2(urdf.data, srdf.data)

## TBD go to home
qhome = [.000, 1.069, 0.030, 2.261, 0.009, -0.50]

## get cartesio task
info_cli = node.create_client(GetCartesianTaskInfo, f'/cartesian/{TASK}/get_cartesian_task_properties')
if not info_cli.wait_for_service(timeout_sec=5.0):
    raise RuntimeError(f'cartesio task {TASK} not available')
future = info_cli.call_async(GetCartesianTaskInfo.Request())
rclpy.spin_until_future_complete(node, future)
base_link, distal_link = future.result().base_link, future.result().distal_link

reach = ActionClient(node, ReachPose, f'/{TASK}/reach')
if not reach.wait_for_server(timeout_sec=5.0):
    raise RuntimeError(f'action /{TASK}/reach not available')


def go_to(pos, quat, trj_time):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, pos)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
    goal = ReachPose.Goal(frames=[pose], time=[trj_time], incremental=False)
    goal_future = reach.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future)
    handle = goal_future.result()
    if not handle.accepted:
        raise RuntimeError('reach goal rejected')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)


## define trajectory (orientation as x, y, z, w)
initial_pos = np.array([0.6, 0.00, 1.10])
initial_quat = np.array([0, 0.7, 0, 0.7])
initial_quat = initial_quat / np.linalg.norm(initial_quat)

trj_dy = 0.4
trj_dz = 0.3

positions = [
    initial_pos + np.array([0, trj_dy, trj_dz]),
    initial_pos + np.array([0, -trj_dy, trj_dz]),
    initial_pos + np.array([0, -trj_dy, -trj_dz]),
    initial_pos + np.array([0, trj_dy, -trj_dz]),
]

trj_segment = 0

pose_ref_list = []
pose_meas_list = []

pos_ref = initial_pos
trj_time = 10.0

while rclpy.ok():

    try:

        go_to(pos_ref, initial_quat, trj_time)

        while True:
            robot.sense()
            qdot = robot.getMotorVelocity()
            if np.linalg.norm(qdot, ord=np.inf) < 1e-3:
                print('robot came to a complete stop')
                break
            time.sleep(0.1)

        robot.update()
        pose_meas = robot.getPose(distal_link, base_link)
        pose_ref_list.append(pos_ref)
        pose_meas_list.append(pose_meas.translation)

        trj_segment = (trj_segment + 1) % len(positions)
        pos_ref = positions[trj_segment]
        trj_time = 2.0

    except KeyboardInterrupt:
        break

print('saving log')
savemat('/tmp/repeatability_test_log.mat',
        {
            'pose_ref_list': pose_ref_list,
            'pose_meas_list': pose_meas_list,
        })
