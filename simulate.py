# Written with assistance from Github Co-Pilot

import time
import dm_control.mujoco
import mujoco.viewer


def main():
    m = dm_control.mujoco.MjModel.from_xml_path("example.xml")
    d = dm_control.mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Set camera parameters
        # These parameters can be adjusted to change the camera angle and perspective
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -20  # Elevation angle (in degrees)
        viewer.cam.distance = 3.0  # Distance from the camera to the target
        viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
        viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
        viewer.cam.lookat[2] = 0.75  # Z-coordinate of the target position

        for i in range(1000):
            # Rotate the spheres using the actuators
            motor_names = ["motor_front_left", "motor_front_right", "motor_rear_left", "motor_rear_right"]
            for m_name in motor_names:
                i = dm_control.mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name)
                d.ctrl[i] = 1.0

            dm_control.mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(1/100)

        viewer.close()


if __name__ == "__main__":
    main()