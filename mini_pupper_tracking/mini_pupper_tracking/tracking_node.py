import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mini_pupper_interfaces.msg import Tracking, TrackingArray
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import cv2
import time
import os
from threading import Lock
from ament_index_python.packages import get_package_share_directory
from motpy import Detection, MultiObjectTracker

MODEL_NAME = "yolo11n.onnx" 
MODEL_PATH = os.path.join(
    get_package_share_directory('mini_pupper_tracking'), 'models', MODEL_NAME)

class TrackingNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_tracking_node')
        self.get_logger().info("Tracking Node Created")

        # Parameter Fetching
        self.declare_parameter('yolo.image_size', 320)
        self.declare_parameter('yolo.confidence_threshold', 0.7)
        self.declare_parameter('yolo.iou_threshold', 0.35)

        self.image_size = self.get_parameter('yolo.image_size').value
        self.confidence_threshold = self.get_parameter('yolo.confidence_threshold').value
        self.iou_threshold = self.get_parameter('yolo.iou_threshold').value

        # Flask Parameter Declaration
        self.declare_parameter('flask.image_display_size', 1280)
        self.declare_parameter('flask.frame_rate', 15)
        self.declare_parameter('flask.auto_open_browser', True)

        self.subscription = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.publisher = self.create_publisher(TrackingArray, "/tracking_array", 10)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = Lock()
        self.last_processed = 0
        self.frame_counter = 0
        self.frame_skip = 1
        self.min_interval = 0.015

        self.sess = ort.InferenceSession(
            MODEL_PATH,
            providers=['CPUExecutionProvider']
        )
        self.get_logger().info(f"Loaded model: {MODEL_NAME}")

        self.tracker = MultiObjectTracker(dt=self.min_interval)
        self.get_logger().info(f"Loaded tracker: motpy")


    def _apply_nms(self, boxes, scores, iou_threshold):
        """Non-Maximum Suppression to remove overlapping boxes"""
        # Convert from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        boxes = np.column_stack([x1, y1, x2, y2])
        
        # Sort by descending confidence
        order = scores.argsort()[::-1]
        keep = []
        
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            # Compute IoU between current box and remaining
            xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
            yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
            xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
            yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            intersection = w * h
            
            area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
            area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * (boxes[order[1:], 3] - boxes[order[1:], 1])
            union = area_i + area_j - intersection
            
            iou = intersection / (union + 1e-7)  # Avoid division by zero
            
            # Keep boxes with IoU < threshold
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]
        
        return keep

    def _preprocess_frame(self, frame):
        """Resize while preserving aspect ratio and pad to square"""
        h, w = frame.shape[:2]
        scale = self.image_size / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(frame, (new_w, new_h))
        
        # Pad to self.image_size x self.image_size
        top = (self.image_size - new_h) // 2
        bottom = self.image_size - new_h - top
        left = (self.image_size - new_w) // 2
        right = self.image_size - new_w - left
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, 
                                   cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return padded, (scale, left, top)

    def image_callback(self, msg):
        now = time.time()
        self.frame_counter += 1

        if self.frame_counter % self.frame_skip != 0:
            return
        if (now - self.last_processed) < self.min_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed, detections = self.process_frame(cv_image)  # Ensure detections is a list of dicts

            with self.frame_lock:
                self.latest_frame = processed

            track_array_msg = TrackingArray()
            track_array_msg.tracks = []

            for det in detections:
                track_msg = Tracking()
                track_msg.confidence = float(det['score'])
                track_msg.center_x = det['center_x']
                track_msg.top_y = det['top_y']
                track_msg.bounding_area = det['area']
                track_msg.track_id = str(det.get('track_id', ''))
                track_array_msg.tracks.append(track_msg)

            self.publisher.publish(track_array_msg)
            self.last_processed = now

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")


    def process_frame(self, frame):
        try:
            detections = [] 

            frame_h, frame_w = frame.shape[:2]
            img, (scale, pad_left, pad_top) = self._preprocess_frame(frame)
            img = img.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0

            # Add error checking for ONNX inference
            outputs = self.sess.run(None, {"images": img})
            if outputs is None or len(outputs) == 0:
                self.get_logger().error("ONNX inference returned None or empty")
                return frame, []
                
            predictions = np.squeeze(outputs[0]).T
            scores = np.max(predictions[:, 4:], axis=1)
            class_ids = np.argmax(predictions[:, 4:], axis=1)
            boxes = predictions[:, :4]

            # Valid if person and if confidence is above the threshold
            valid_indices = [i for i in range(len(scores)) 
                            if class_ids[i] == 0 and scores[i] > self.confidence_threshold]
            
            if not valid_indices:
                return frame, []  # No detections

            boxes_filtered = boxes[valid_indices]
            scores_filtered = scores[valid_indices]
            keep_indices = self._apply_nms(boxes_filtered, scores_filtered, self.iou_threshold)

            # Prepare detections for motpy
            motpy_detections = []
            detection_data = []  # Store original data for later use

            for i in keep_indices:
                center_x, center_y, w, h = boxes_filtered[i]
                
                center_x = (center_x - pad_left) / scale
                center_y = (center_y - pad_top) / scale
                w /= scale
                h /= scale
                
                # Convert to [x1, y1, x2, y2] for motpy
                x1 = center_x - w / 2
                y1 = center_y - h / 2
                x2 = center_x + w / 2
                y2 = center_y + h / 2
                
                # Create motpy Detection
                motpy_detections.append(Detection(box=np.array([x1, y1, x2, y2]), score=scores_filtered[i]))
                
                # Store data for later
                detection_data.append({
                    'score': scores_filtered[i],
                    'center_x': center_x / frame_w,
                    'top_y': y1 / frame_h,
                    'area': (w * h) / (frame_w * frame_h),
                    'pixel_coords': (int(x1), int(y1), int(x2), int(y2))
                })

            # Update tracker
            self.tracker.step(detections=motpy_detections)
            tracks = self.tracker.active_tracks()

            # Match tracks to detections and create output
            for track_idx, track in enumerate(tracks):
                if track_idx < len(detection_data):
                    data = detection_data[track_idx]
                    
                    detections.append({
                        'score': data['score'],
                        'center_x': data['center_x'],
                        'top_y': data['top_y'],
                        'area': data['area'],
                        'track_id': str(track.id)  # Convert UUID to string
                    })
                    
                    # Draw box with track ID
                    x1, y1, x2, y2 = data['pixel_coords']
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"ID:{track.id}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            return frame, detections
            
        except Exception as e:
            self.get_logger().error(f"Exception in process_frame: {e}")
            return frame, []