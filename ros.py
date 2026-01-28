#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node

# 你文档里的 HelloNode（不同版本 import 路径可能不同）
# 常见是：from stretch_ros2.hello_node import HelloNode
# 或：from hello_helpers.hello_node import HelloNode
from stretch_ros2.hello_node import HelloNode  # 如果报错，按你机器上的实际包名改


def get_joint_pos(node: HelloNode, joint_name: str) -> float:
    """
    Read current joint position from node.joint_state.
    """
    js = node.joint_state
    if js is None or js.name is None or js.position is None:
        raise RuntimeError("joint_state not available yet (wait for joint_state topic).")
    if joint_name not in js.name:
        raise KeyError(f"{joint_name} not in joint_state.name. Available={list(js.name)}")
    idx = js.name.index(joint_name)
    return float(js.position[idx])


def wait_for_joint_state(node: HelloNode, timeout_s: float = 5.0) -> None:
    """
    Spin until joint_state arrives.
    """
    t0 = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if getattr(node, "joint_state", None) is not None and getattr(node.joint_state, "name", None):
            return
        if time.time() - t0 > timeout_s:
            raise TimeoutError("Timed out waiting for joint_state.")


def main():
    rclpy.init()

    node = HelloNode()  # 文档风格：node.stow_the_robot(), node.move_to_pose(...)
    try:
        # 先确保能读到 joint_state（否则后面相对运动没法算）
        wait_for_joint_state(node, timeout_s=10.0)

        # -------------------------
        # 0) Stow
        # -------------------------
        node.stow_the_robot()
        # stow 本身可能是 action；稳一点 sleep/再 spin
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(2.0)

        # -------------------------
        # 1) Extend arm + raise lift (blocking)
        #    你原本用 params range；ROS 这里直接给一个“实验室可接受”的大值
        #    （不同机型上限不同；你可按需要调）
        # -------------------------
        node.move_to_pose({"joint_arm": 0.50, "joint_lift": 1.00}, blocking=True)

        # -------------------------
        # 2) Wrist motors one at a time (relative move)
        # -------------------------
        wrist_seq = [
            ("joint_wrist_yaw",   np.deg2rad(45)),
            ("joint_wrist_pitch", np.deg2rad(30)),
            ("joint_wrist_roll",  np.deg2rad(45)),
        ]

        for jname, delta in wrist_seq:
            curr = get_joint_pos(node, jname)
            node.move_to_pose({jname: curr + float(delta)}, blocking=True)
            time.sleep(0.3)

        # -------------------------
        # 3) Gripper open then close
        #    你给的 joint: joint_gripper_finger_left / right 二选一
        #    这里用 left，如果你机器 publish 的是 right 就换掉
        # -------------------------
        gripper_joint = "joint_gripper_finger_left"
        curr_g = get_joint_pos(node, gripper_joint)

        # 轻微打开/关闭：同样用相对位移更安全（不同夹爪范围不同）
        node.move_to_pose({gripper_joint: curr_g + 0.02}, blocking=True)  # open a bit
        time.sleep(0.5)
        node.move_to_pose({gripper_joint: curr_g}, blocking=True)         # close back
        time.sleep(0.5)

        # -------------------------
        # 4) Head pan/tilt (relative)
        # -------------------------
        head_seq = [
            ("joint_head_pan",  np.deg2rad(30)),
            ("joint_head_tilt", np.deg2rad(20)),
        ]
        for jname, delta in head_seq:
            curr = get_joint_pos(node, jname)
            node.move_to_pose({jname: curr + float(delta)}, blocking=True)
            time.sleep(0.3)

        # -------------------------
        # 5) Stow again
        # -------------------------
        node.stow_the_robot()
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(2.0)

        # -------------------------
        # 6) Base: forward 0.5m, rotate 180deg, forward 0.5m
        #    你给的 names: translate_mobile_base / rotate_mobile_base
        #    这里按“增量”去加
        # -------------------------
        # forward 0.5
        curr_t = get_joint_pos(node, "translate_mobile_base")
        node.move_to_pose({"translate_mobile_base": curr_t + 0.5}, blocking=True)

        # rotate pi
        curr_r = get_joint_pos(node, "rotate_mobile_base")
        node.move_to_pose({"rotate_mobile_base": curr_r + float(np.pi)}, blocking=True)

        # forward 0.5
        curr_t2 = get_joint_pos(node, "translate_mobile_base")
        node.move_to_pose({"translate_mobile_base": curr_t2 + 0.5}, blocking=True)

        # Final stow
        node.stow_the_robot()
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info("Motion demo complete (ROS2 HelloNode).")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
