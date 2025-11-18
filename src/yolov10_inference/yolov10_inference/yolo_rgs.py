import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import supervision as sv
from ultralytics import YOLO
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from builtin_interfaces.msg import Time

class YOLOv10Node(Node):
    def __init__(self):
        super().__init__('yolov10_node')

        # 参数：模型路径与图像话题
        default_model_path = os.path.join(os.path.dirname(__file__), 'models', 'best1.pt')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # 初始化 YOLOv10 模型
        self.model = YOLO(model_path)
        # self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.bridge = CvBridge()
	
        # 发布检测结果
        self.result_publisher = self.create_publisher(String, 'yolo_detection_results', 10)
        self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            image_topic,  # 可通过参数覆盖
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 推理
        results = self.model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
	
        # 如果有检测结果
        if len(detections.class_id) > 0:
            detected_objects = []
            for class_id, confidence in zip(detections.class_id, detections.confidence):
                label = self.model.names[class_id]
                detected_objects.append(f'{label} ({confidence:.2f})')
        
            # 将检测结果拼接成字符串
            detection_message = ", ".join(detected_objects)
            self.result_publisher.publish(String(data=detection_message))
            self.get_logger().info(f'Detected: {detection_message}')
        else:
            detection_message = "none (1.00)"
            self.result_publisher.publish(String(data=detection_message))
            self.get_logger().info('No objects detected.')

        # 发布 Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header
        names = self.model.names

        if len(detections.class_id) > 0 and hasattr(detections, 'xyxy'):
            for (x1, y1, x2, y2), class_id, score in zip(detections.xyxy, detections.class_id, detections.confidence):
                det = Detection2D()
                det.header = msg.header
                # 置信度与类别
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(names[int(class_id)]) if int(class_id) in names else str(int(class_id))
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)
                # 边框
                bbox = BoundingBox2D()
                bbox.center.x = float((x1 + x2) / 2.0)
                bbox.center.y = float((y1 + y2) / 2.0)
                bbox.size_x = float(x2 - x1)
                bbox.size_y = float(y2 - y1)
                det.bbox = bbox
                det_array.detections.append(det)

        self.det_pub.publish(det_array)
        
        # 标注图像
        annotated_image = self.bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections)

        # 显示标注图像
        #cv2.imshow('YOLOv10 Detection', annotated_image)
        #cv2.waitKey(1)

    def destroy(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv10Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

