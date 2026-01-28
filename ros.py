#!/usr/bin/env python3
import time
import numpy as np

import hello_helpers.hello_misc as hm


# ---- Joint names (from your list) ----
JOINT_ARM = "joint_arm"
JOINT_LIFT = "joint_lift"
JOINT_WY = "joint_wrist_yaw"
JOINT_WP = "joint_wrist_pitch"
JOINT_WR = "joint_wrist_roll"
JOINT_GRIPPER = "joint_gripper_finger_left"  # or "joint_gripper_finger_right"
JOINT_HEAD_PAN = "joint_head_pan"
JOINT_HEAD_TILT = "joint_head_tilt"
JOINT_BASE_TRANS = "translate_mobile_base"
JOINT_BASE_ROT = "rotate_mobile_base"


def get_joint_pos(node: hm.HelloNode, joint_name: str) -> float:
    """
    Read current joint position from node.joint_state.
    HelloNode subscribes to /stretch/joint_states (sensor_msgs/JointState). :contentReference[oaicite:4]{index=4}
    """
    js = node.joint_state
    if js is None or js.name is None or js.position is None or len(js.name) == 0:
        raise RuntimeError("joint_state not ready yet (did you launch stretch_driver?)")
    if joint_name not in js.name:
        raise KeyError(f"{joint_name} not in joint_state.name. Available={list(js.name)}")
    i = js.name.index(joint_name)
    return float(js.position[i])


def wait_for_joint_state(node: hm.HelloNode, timeout_s: float = 10.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hm.HelloNode.spin_once(node, 0.1) if hasattr(hm.HelloNode, "spin_once") else time.sleep(0.1)
        if getattr(node, "joint_state", None) is not None and getattr(node.joint_state, "name", None):
            return
    raise TimeoutError("Timed out waiting for /stretch/joint_states")


class StretchAPIRos2Demo(hm.HelloNode):
    def __init__(self):
        super().__init__()

    def main(self):
        # Standard HelloNode startup pattern. :contentReference[oaicite:5]{index=5}
        super().main("stretch_api_ros2_demo", "stretch_api_ros2_demo", wait_for_first_pointcloud=False)

        wait_for_joint_state(self, timeout_s=10.0)

        # 0) Stow
        self.stow_the_robot()  # convenience wrapper around /stow_the_robot service :contentReference[oaicite:6]{index=6}
        time.sleep(2.0)

        # 1) Arm + lift extend (blocking=True waits for completion) :contentReference[oaicite:7]{index=7}
        # Note: values depend on your robot limits; these mirror your Stretch Body script defaults.
        self.move_to_pose({JOINT_ARM: 0.50, JOINT_LIFT: 1.00}, blocking=True)

        # 2) Wrist one-at-a-time (relative)
        wrist_seq = [
            (JOINT_WY, np.deg2rad(45)),
            (JOINT_WP, np.deg2rad(30)),
            (JOINT_WR, np.deg2rad(45)),
        ]
        for jname, delta in wrist_seq:
            curr = get_joint_pos(self, jname)
            self.move_to_pose({jname: curr + float(delta)}, blocking=True)
            time.sleep(0.3)

        # 3) Gripper open then close (relative nudge is safest)
        g0 = get_joint_pos(self, JOINT_GRIPPER)
        self.move_to_pose({JOINT_GRIPPER: g0 + 0.02}, blocking=True)  # open a bit
        time.sleep(0.3)
        self.move_to_pose({JOINT_GRIPPER: g0}, blocking=True)         # close back
        time.sleep(0.3)

        # 4) Head pan/tilt
        head_seq = [
            (JOINT_HEAD_PAN, np.deg2rad(30)),
            (JOINT_HEAD_TILT, np.deg2rad(20)),
        ]
        for jname, delta in head_seq:
            curr = get_joint_pos(self, jname)
            self.move_to_pose({jname: curr + float(delta)}, blocking=True)
            time.sleep(0.3)

        # 5) Stow again
        self.stow_the_robot()
        time.sleep(2.0)

        # 6) Base: forward 0.5m, rotate 180deg, forward 0.5m
        # HelloNode.move_to_pose accepts pose dict of joint_name -> position_goal. :contentReference[oaicite:8]{index=8}
        t0 = get_joint_pos(self, JOINT_BASE_TRANS)
        self.move_to_pose({JOINT_BASE_TRANS: t0 + 0.5}, blocking=True)

        r0 = get_joint_pos(self, JOINT_BASE_ROT)
        self.move_to_pose({JOINT_BASE_ROT: r0 + float(np.pi)}, blocking=True)

        t1 = get_joint_pos(self, JOINT_BASE_TRANS)
        self.move_to_pose({JOINT_BASE_TRANS: t1 + 0.5}, blocking=True)

        # Final stow
        self.stow_the_robot()
        time.sleep(2.0)

        self.get_logger().info("ROS2 HelloNode motion demo complete.")


if __name__ == "__main__":
    node = StretchAPIRos2Demo()
    node.main()
