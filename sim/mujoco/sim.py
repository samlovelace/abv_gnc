import sys
import threading

import mujoco
import rclpy
import yaml

from rclpy.executors import SingleThreadedExecutor
from sim_modules import Simulation, build_modules

DEFAULT_CONFIG = "config/rebot_tabletop.yaml"


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG

    with open(config_path) as f:
        config = yaml.safe_load(f)

    model = mujoco.MjModel.from_xml_path(config["world"])
    data = mujoco.MjData(model)

    for mount in config.get("mounts", []):
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            mount["body"]
        )

        if "pos" in mount:
            model.body_pos[body_id] = mount["pos"]

        if "quat" in mount:
            model.body_quat[body_id] = mount["quat"]

    mujoco.mj_forward(model, data)

    rclpy.init()
    node = rclpy.create_node(
        config.get("node_name", "mujoco_sim")
    )

    modules = build_modules(config, node)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    ros_thread.start()

    try:
        Simulation(model, data, modules).run()

    finally:
        executor.shutdown()
        ros_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()