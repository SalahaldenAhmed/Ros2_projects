import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.bridge = CvBridge()
        self.model  = YOLO('yolov8n.pt')

        # run inference at 5 fps — not spammy
        self.inference_interval  = 0.2
        self.last_inference_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info('YOLOv8n detector started — watching /camera/image_raw')

    def image_callback(self, msg):

        # throttle to 5 fps
        now = time.time()
        if now - self.last_inference_time < self.inference_interval:
            return
        self.last_inference_time = now

        try:
            # convert ROS image → OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # log camera resolution once every 10s so we know it's working
            h, w = cv_image.shape[:2]
            self.get_logger().info(
                f'Camera feed: {w}x{h}', throttle_duration_sec=10.0)

            # run YOLO — conf=0.1 catches sim objects that look different from real ones
            results = self.model(cv_image, conf=0.1, verbose=False)

            # log what was detected
            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    label  = self.model.names[cls_id]
                    conf   = float(box.conf[0])
                    self.get_logger().info(
                        f'DETECTED ▶  {label}  ({conf:.0%} confidence)')
            else:
                self.get_logger().info(
                    'No detections', throttle_duration_sec=2.0)

            # draw bounding boxes and show window
            annotated = results[0].plot()
            cv2.imshow('YOLOv8 Detection', annotated)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()