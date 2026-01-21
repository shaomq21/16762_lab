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

def main():
    robot = stretch_body.robot.Robot()
    robot.startup()

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
        wrist_joints = [
            ("wrist_yaw",  np.radians(45)),
            ("wrist_pitch", np.radians(30)),
            ("wrist_roll", np.radians(45)),
        ]

        for joint_name, delta in wrist_joints:
            if hasattr(robot.end_of_arm, joint_name):
                j = getattr(robot.end_of_arm, joint_name)

                # Move relative: read current position if available, else just "move_by"
                try:
                    j.move_by(delta)
                except Exception:
                    # fallback to absolute if move_by isn't available
                    # try using status if present
                    curr = j.status.get('pos', 0.0) if hasattr(j, "status") else 0.0
                    j.move_to(curr + delta)

                robot.push_command()
                # Give time for visible motion (and ensure one-at-a-time)
                if hasattr(j, "wait_until_at_setpoint"):
                    j.wait_until_at_setpoint()
                else:
                    time.sleep(2.0)
            else:
                print(f"[WARN] end_of_arm has no joint named '{joint_name}', skipping.")

        # -------------------------
        # 3) Open gripper then close it
        # -------------------------
        if hasattr(robot.end_of_arm, "stretch_gripper"):
            g = robot.end_of_arm.stretch_gripper
            # open
            try:
                g.open()
            except Exception:
                # some versions: move_to(1.0) for open
                g.move_to(1.0)
            robot.push_command()
            time.sleep(2.0)

            # close
            try:
                g.close()
            except Exception:
                # some versions: move_to(0.0) for close
                g.move_to(0.0)
            robot.push_command()
            time.sleep(2.0)
        else:
            print("[WARN] No stretch_gripper found under end_of_arm; skipping gripper open/close.")

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
