import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
from image_geometry import PinholeCameraModel
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose


class BBoxTo3DNode(Node):
    def __init__(self):
        super().__init__('bbox_to_3d_node')

        # Parameters
        self.declare_parameter('detection_topic', 'detections')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_class', '')  # label string, e.g., 'cup'
        self.declare_parameter('depth_window', 7)  # odd number
        self.declare_parameter('target_frame', '')  # e.g., 'base_link'
        self.declare_parameter('output_topic', 'target_pose')
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 5.0)

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.depth_window = int(self.get_parameter('depth_window').get_parameter_value().integer_value or 7)
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_marker = self.get_parameter('publish_marker').get_parameter_value().bool_value
        self.min_depth_m = float(self.get_parameter('min_depth_m').get_parameter_value().double_value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').get_parameter_value().double_value)

        # State
        self.bridge = CvBridge()
        self.latest_depth_msg: Image = None
        self.latest_depth_image: np.ndarray = None
        self.depth_encoding: str = None
        self.cam_model = PinholeCameraModel()
        self.have_cam_info = False

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subs
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.caminfo_cb, 10)
        self.det_sub = self.create_subscription(Detection2DArray, self.detection_topic, self.det_cb, 10)

        # Pubs
        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)

        self.get_logger().info(f'BBoxTo3D started. Subscribing to {self.detection_topic}, {self.depth_topic}, {self.camera_info_topic}')

    def depth_cb(self, msg: Image):
        self.latest_depth_msg = msg
        self.depth_encoding = msg.encoding
        if msg.encoding == '16UC1':
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.uint16)
            self.latest_depth_image = depth
        elif msg.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
            self.latest_depth_image = depth
        else:
            # try passthrough
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth

    def caminfo_cb(self, msg: CameraInfo):
        self.cam_model.fromCameraInfo(msg)
        self.cam_frame = msg.header.frame_id
        self.have_cam_info = True

    def det_cb(self, msg: Detection2DArray):
        if not self.have_cam_info or self.latest_depth_image is None:
            self.get_logger().warn_throttle(2.0, 'Waiting for camera info and depth...')
            return
        if len(msg.detections) == 0:
            return

        # Pick target detection
        det = self._select_detection(msg.detections)
        if det is None:
            return

        u = int(round(det.bbox.center.x))
        v = int(round(det.bbox.center.y))
        depth_m = self._depth_at(u, v, self.depth_window)
        if depth_m is None or not (self.min_depth_m <= depth_m <= self.max_depth_m):
            self.get_logger().warn_throttle(2.0, f'Invalid depth at ({u},{v}).')
            return

        # Project to 3D in camera frame
        ray = np.array(self.cam_model.projectPixelTo3dRay((u, v)), dtype=np.float64)
        ray = ray / ray[2]  # normalize so z == 1
        point_cam = ray * depth_m  # meters

        pose_cam = PoseStamped()
        pose_cam.header.stamp = msg.header.stamp
        pose_cam.header.frame_id = self.cam_frame
        pose_cam.pose.position.x = float(point_cam[0])
        pose_cam.pose.position.y = float(point_cam[1])
        pose_cam.pose.position.z = float(point_cam[2])
        pose_cam.pose.orientation.x = 0.0
        pose_cam.pose.orientation.y = 0.0
        pose_cam.pose.orientation.z = 0.0
        pose_cam.pose.orientation.w = 1.0

        # Publish in target frame if requested
        out_pose = pose_cam
        if self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, pose_cam.header.frame_id, rclpy.time.Time())
                out_pose = do_transform_pose(pose_cam, tf)
                out_pose.header.frame_id = self.target_frame
            except TransformException as ex:
                self.get_logger().warn_throttle(2.0, f'TF unavailable: {ex}')
                out_pose = pose_cam

        self.pose_pub.publish(out_pose)
        if self.publish_marker:
            self._publish_marker(out_pose)

    def _select_detection(self, detections):
        selected = None
        best_score = -1.0
        for det in detections:
            if len(det.results) == 0:
                continue
            label = det.results[0].hypothesis.class_id  # string label from yolo_rgs
            score = det.results[0].hypothesis.score
            if self.target_class:
                if label == self.target_class and score > best_score:
                    best_score = score
                    selected = det
            else:
                if score > best_score:
                    best_score = score
                    selected = det
        return selected

    def _depth_at(self, u: int, v: int, window: int):
        img = self.latest_depth_image
        h, w = img.shape[:2]
        half = max(1, window // 2)
        u0 = max(0, u - half)
        v0 = max(0, v - half)
        u1 = min(w - 1, u + half)
        v1 = min(h - 1, v + half)

        roi = img[v0:v1 + 1, u0:u1 + 1]
        if roi.size == 0:
            return None
        if self.depth_encoding == '16UC1':
            vals = roi.astype(np.float32)
            vals = vals[(vals > 0)]
            if vals.size == 0:
                return None
            depth_m = float(np.median(vals) / 1000.0)  # mm -> m
            return depth_m
        elif self.depth_encoding == '32FC1':
            vals = roi
            vals = vals[np.isfinite(vals)]
            vals = vals[(vals > 0.0)]
            if vals.size == 0:
                return None
            depth_m = float(np.median(vals))
            return depth_m
        else:
            # Try best-effort: assume meters
            vals = roi.astype(np.float32)
            vals = vals[(vals > 0)]
            if vals.size == 0:
                return None
            return float(np.median(vals))

    def _publish_marker(self, pose: PoseStamped):
        m = Marker()
        m.header = pose.header
        m.ns = 'bbox_to_3d'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 0.03
        m.scale.y = 0.03
        m.scale.z = 0.03
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.9
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BBoxTo3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()



