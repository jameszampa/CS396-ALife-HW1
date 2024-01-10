import dm_control.mujoco
import mujoco.viewer


def main():
    m = dm_control.mujoco.MjModel.from_xml_path("example.xml")
    d = dm_control.mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.sync()
        viewer.close()


if __name__ == "__main__":
    main()