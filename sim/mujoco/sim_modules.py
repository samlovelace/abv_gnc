import math
import time

import mujoco
import mujoco.viewer
from cv_bridge import CvBridge
import cv2 
import numpy as np
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, JointState


# ---------- framework ----------

class SimModule:
    rate_hz = None  # None = post_step runs every step

    def __init__(self, node=None):
        self._next = 0.0
        self.node = node  # shared rclpy Node, injected by build_modules - may be None

    def on_start(self, model, data): pass
    def pre_step(self, model, data): pass     # runs every step, before mj_step
    def post_step(self, model, data): pass    # runs at rate_hz, after mj_step
    def on_stop(self): pass

    def due(self, t):
        if self.rate_hz is None:
            return True
        if t < self._next:
            return False
        self._next += 1.0 / self.rate_hz
        return True


class Simulation:
    def __init__(self, model, data, modules, view_hz=60, realtime=True):
        self.model, self.data, self.modules = model, data, modules
        self.view_period = 1.0 / view_hz
        self.realtime = realtime

    def run(self):
        model, data = self.model, self.data
        for m in self.modules:
            m.on_start(model, data)
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                wall0 = time.perf_counter()
                next_view = 0.0
                while viewer.is_running():
                    for m in self.modules:
                        m.pre_step(model, data)

                    mujoco.mj_step(model, data)

                    for m in self.modules:
                        if m.due(data.time):
                            m.post_step(model, data)

                    if data.time >= next_view:
                        viewer.sync()
                        next_view += self.view_period

                    if self.realtime:
                        ahead = data.time - (time.perf_counter() - wall0)
                        if ahead > 0:
                            time.sleep(ahead)
        finally:
            for m in reversed(self.modules):
                m.on_stop()


def joint_addrs(model, name):
    """(qpos address, qvel/dof address) for a named joint."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[jid], model.jnt_dofadr[jid]


# ---------- modules ----------

class ArmControl(SimModule):
    """Holds a manipulator's joints at whatever pose they start in (via
    real position actuators, so it doesn't droop under gravity) and
    publishes JointState for those same joints. Not a command-following
    controller yet - accepting external goals is a natural follow-up once
    this lands, not built now.
    """
    rate_hz = 100

    def __init__(self, joint_names, state_topic="joint_states", cmd_topic="joint_commands", node=None):
        super().__init__(node=node)
        self.joint_names = list(joint_names)
        self.state_topic = state_topic
        self.cmd_topic = cmd_topic
        self.aidx = []
        self.qpos_adr = []
        self.qvel_adr = []
        self.goal = []
        self.pub = None

    def on_start(self, model, data):
        self.aidx = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{j}_act")
            for j in self.joint_names
        ]
        addrs = [joint_addrs(model, j) for j in self.joint_names]
        self.qpos_adr = [a[0] for a in addrs]
        self.qvel_adr = [a[1] for a in addrs]
        self.goal = [data.qpos[a] for a in self.qpos_adr]  # hold start pose

        if self.node is not None:
            self.pub = self.node.create_publisher(JointState, self.state_topic, 10)
            self.node.create_subscription(JointState, self.cmd_topic, self.cmd_callback, 10)

    def pre_step(self, model, data):
        for aidx, goal in zip(self.aidx, self.goal):
            #print("Setting actuator", aidx, "to goal", goal)
            data.ctrl[aidx] = goal

    def post_step(self, model, data):
        if self.pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(data.qpos[a]) for a in self.qpos_adr]
        msg.velocity = [float(data.qvel[a]) for a in self.qvel_adr]
        self.pub.publish(msg)
        
    def cmd_callback(self, msg):
        """Accepts a JointState message with the same joint names as this module
        and updates the goal pose to match.
        """
        if set(msg.name) != set(self.joint_names):
            raise ValueError("JointState names do not match ArmControl joint_names")
        name_to_pos = dict(zip(msg.name, msg.position))
        self.goal = [name_to_pos[j] for j in self.joint_names]


class CameraPublisher(SimModule):
    """Renders a MuJoCo camera and publishes it as Image + CameraInfo."""
    rate_hz = 30

    def __init__(self, camera_name="overhead", width=640, height=480,
                 topic="camera/image_raw", depth_topic=None, frame_id="camera", node=None):
        super().__init__(node=node)
        self.camera_name = camera_name
        self.w, self.h = width, height
        self.topic = topic
        self.depth_topic = depth_topic
        self.frame_id = frame_id
        self.renderer = None
        self.bridge = CvBridge()
        self.image_pub = None
        self.comp_image_pub = None
        self.info_pub = None
        self.camera_info = None
        self.depth_pub = None
        self.depth_info_pub = None

    def on_start(self, model, data):
        self.renderer = mujoco.Renderer(model, height=self.h, width=self.w)
        self.camera_info = self._build_camera_info(model)

        if self.node is not None:
            self.image_pub = self.node.create_publisher(Image, self.topic, 10)
            self.info_pub = self.node.create_publisher(CameraInfo, self._sibling_topic("camera_info"), 10)
            self.comp_image_pub = self.node.create_publisher(CompressedImage, self._sibling_topic("compressed_image"), 10)

            if self.depth_topic is not None:
                self.depth_pub = self.node.create_publisher(Image, self.depth_topic, 10)
                self.depth_info_pub = self.node.create_publisher(
                    CameraInfo, self._depth_sibling_topic("camera_info"), 10)

    def _sibling_topic(self, key):
        if "/" in self.topic:
            base, _ = self.topic.rsplit("/", 1)
            return f"{base}/" + key
        return key

    def _depth_sibling_topic(self, key):
        # self.depth_topic ("aligned_depth_to_color/...") isn't a sibling of
        # self.topic ("color/...") - different second-to-last path segment -
        # so this needs its own base, not _sibling_topic's.
        if "/" in self.depth_topic:
            base, _ = self.depth_topic.rsplit("/", 1)
            return f"{base}/" + key
        return key

    def _build_camera_info(self, model):
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        fovy = math.radians(model.cam_fovy[cam_id])
        fy = self.h / (2 * math.tan(fovy / 2))
        fx = fy
        cx, cy = self.w / 2.0, self.h / 2.0

        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.w
        info.height = self.h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info
    
    def _build_compressed_image(self, rgb_image: np.ndarray):
        # MuJoCo typically produces RGB images.
        # OpenCV expects BGR.
        bgr_image = cv2.cvtColor(
            rgb_image,
            cv2.COLOR_RGB2BGR
        )

        # Encode image as JPEG.
        success, encoded = cv2.imencode(
            ".jpg",
            bgr_image,
            [cv2.IMWRITE_JPEG_QUALITY, 80]
        )

        if not success:
            return

        msg = CompressedImage()
        msg.header.frame_id = "camera_optical_frame"

        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg

    def post_step(self, model, data):
        self.renderer.update_scene(data, camera=self.camera_name)
        rgb = self.renderer.render()

        if self.image_pub is None:
            return

        stamp = self.node.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)
        
        comp_img_msg = self._build_compressed_image(rgb)
        comp_img_msg.header.stamp = stamp
        self.comp_image_pub.publish(comp_img_msg)

        if self.depth_pub is not None:
            # MuJoCo's Renderer toggles RGB/depth mode on one instance - a
            # single render() call can't return both, so this is a second
            # update_scene+render pass (doubles per-frame render cost at
            # rate_hz, acceptable for sim testing).
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(data, camera=self.camera_name)
            depth = self.renderer.render()  # float32, meters
            self.renderer.disable_depth_rendering()

            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.depth_pub.publish(depth_msg)

            self.camera_info.header.stamp = stamp
            self.depth_info_pub.publish(self.camera_info)  # same intrinsics as color - that's what "aligned" means

    def on_stop(self):
        if self.renderer is not None:
            self.renderer.close()


# ---------- config-driven module loading ----------

MODULE_REGISTRY = {
    "ArmControl": ArmControl,
    "CameraPublisher": CameraPublisher,
}


def build_modules(config, node):
    modules = []
    for entry in config.get("modules", []):
        if not entry.get("enabled", True):
            continue
        cls = MODULE_REGISTRY[entry["type"]]
        modules.append(cls(node=node, **entry.get("args", {})))
    return modules
