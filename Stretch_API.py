import time
import numpy as np
import stretch_body.robot

def get_max_range_m(component, default):
    """
    Try to read max travel range from Stretch Body params.
    Falls back to a conservative default if not found.
    """
    try:
        # Many Stretch Body components expose params['range_m'] = [min, max]
        r = component.params.get('range_m', None)
        if r is not None and len(r) == 2:
            return float(r[1])
    except Exception:
        pass
    return float(default)

def safe_wait(robot, seconds=0.1):
    robot.push_command()
    time.sleep(seconds)

def safe_push(robot, dt=0.05):
    robot.push_command()
    time.sleep(dt)

def wait_component(comp, timeout=15.0):
    # Stretch components大多有 wait_until_at_setpoint()
    if hasattr(comp, "wait_until_at_setpoint"):
        comp.wait_until_at_setpoint(timeout=timeout)

def get_eoa_joint(robot, joint_name: str):
    eoa = robot.end_of_arm
    # 优先官方方法
    if hasattr(eoa, "get_joint"):
        j = eoa.get_joint(joint_name)
        if j is not None:
            return j
    # fallback：有些版本 joint 其实在 motors dict 里
    if hasattr(eoa, "motors") and isinstance(eoa.motors, dict) and joint_name in eoa.motors:
        return eoa.motors[joint_name]
    raise KeyError(f"Cannot resolve joint '{joint_name}'. Available motors={list(getattr(eoa,'motors',{}).keys())}")


def move_by_or_to(j, delta, fallback_sleep=2.0):
    """
    Try relative move; otherwise absolute from status['pos'].
    """
    if hasattr(j, "move_by"):
        j.move_by(delta)
        return
    curr = 0.0
    if hasattr(j, "status") and isinstance(j.status, dict):
        curr = float(j.status.get("pos", 0.0))
    if hasattr(j, "move_to"):
        j.move_to(curr + delta)
        return
    # last resort
    time.sleep(fallback_sleep)



def main():
    robot = stretch_body.robot.Robot()
    robot.startup()
    print("EOA motors:", list(robot.end_of_arm.motors.keys()))

    try:
        # -------------------------
        # 0) Stow (single line)
        # -------------------------
        # Make sure free space around robot before running.
        robot.stow()
        robot.push_command()
        time.sleep(5.0)  # stow can take a few seconds

        # -------------------------
        # 1) Extend arm fully + raise lift fully at same time
        # -------------------------
        arm_max = get_max_range_m(robot.arm, default=0.50)     # typical ~0.52m
        lift_max = get_max_range_m(robot.lift, default=1.00)   # depends on model; use params when possible

        robot.arm.move_to(arm_max)
        robot.lift.move_to(lift_max)
        robot.push_command()

        # Wait until both reach setpoints
        robot.arm.wait_until_at_setpoint()
        robot.lift.wait_until_at_setpoint()

        # -------------------------
        # 2) Move the three wrist motors, one at a time (visible rotation)
        #    Typical joints: wrist_yaw, wrist_pitch, wrist_roll
        # -------------------------
        # Some end-of-arm toolchains may not have all three; we guard with hasattr.
        wrist = [
            ("joint_wrist_yaw",   np.deg2rad(45)),
            ("joint_wrist_pitch", np.deg2rad(30)),
            ("joint_wrist_roll",  np.deg2rad(45)),
        ]
        for name, delta in wrist:
            j = get_eoa_joint(robot, name)
            move_by_or_to(j, delta)
            safe_push(robot, 0.1)
            wait_component(j, timeout=10.0)
            time.sleep(0.5)  # 让你肉眼更明显看到“一次一个”

        # 3) gripper open/close
        # 你的 gripper 是 finger joint；左右选一个即可（通常会联动）
        gripper_joint_name = None
        for cand in ["joint_gripper_finger_left", "joint_gripper_finger_right"]:
            try:
                _ = get_eoa_joint(robot, cand)
                gripper_joint_name = cand
                break
            except Exception:
                pass

        if gripper_joint_name is None:
            raise RuntimeError("No gripper finger joint found (left/right).")

        g = get_eoa_joint(robot, gripper_joint_name)

        # 注意：不同夹爪 pos 单位/范围不同（rad 或 m），下面用“相对运动”更稳
        move_by_or_to(g, +0.015)   # 尝试打开一点
        safe_push(robot, 0.1)
        wait_component(g, timeout=10.0)
        time.sleep(1.0)

        move_by_or_to(g, -0.015)   # 再关回去
        safe_push(robot, 0.1)
        wait_component(g, timeout=10.0)
        time.sleep(1.0)
        # -------------------------
        # 4) Rotate both motors connected to the RealSense (head pan/tilt), one at a time
        # -------------------------
        # Typical: robot.head.pan and robot.head.tilt
        head_moves = [
            ("pan",  np.radians(30)),
            ("tilt", np.radians(20)),
        ]
        if hasattr(robot, "head"):
            for axis_name, delta in head_moves:
                if hasattr(robot.head, axis_name):
                    ax = getattr(robot.head, axis_name)
                    try:
                        ax.move_by(delta)
                    except Exception:
                        curr = ax.status.get('pos', 0.0) if hasattr(ax, "status") else 0.0
                        ax.move_to(curr + delta)
                    robot.push_command()
                    if hasattr(ax, "wait_until_at_setpoint"):
                        ax.wait_until_at_setpoint()
                    else:
                        time.sleep(2.0)
                else:
                    print(f"[WARN] robot.head has no axis '{axis_name}', skipping.")
        else:
            print("[WARN] No robot.head found; skipping head pan/tilt motion.")

        # -------------------------
        # 5) Reset everything back to stow
        # -------------------------
        robot.stow()  # single line
        robot.push_command()
        time.sleep(5.0)

        # -------------------------
        # 6) Drive forward 0.5m, rotate 180deg, drive forward 0.5m
        # -------------------------
        # Note: base commands are usually non-blocking; we use sleep to let them finish.
        # If your Stretch Body supports wait methods for base, you can replace sleeps with waits.
        robot.base.translate_by(0.5)
        robot.push_command()
        time.sleep(4.0)

        robot.base.rotate_by(np.pi)
        robot.push_command()
        time.sleep(6.0)

        robot.base.translate_by(0.5)
        robot.push_command()
        time.sleep(4.0)

        # Final stow (optional, but nice)
        robot.stow()
        robot.push_command()
        time.sleep(5.0)

        print("[DONE] Motion demo complete.")

    finally:
        robot.stop()

if __name__ == "__main__":
    main()
