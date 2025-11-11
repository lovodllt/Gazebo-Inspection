#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import torch

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolov8_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 模型路径
        pkg_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(pkg_path, '..', 'models', 'v8.pt')
        
        if not os.path.exists(model_path):
            rospy.logerr(f"模型文件不存在: {model_path}")
            rospy.signal_shutdown("模型文件未找到")
            return
            
        rospy.loginfo(f"加载模型: {model_path}")
        self.model = YOLO(model_path)
        
        try:
            self.imgsz = self.model.model.yaml['imgsz']
            if isinstance(self.imgsz, list):
                self.imgsz = self.imgsz[0]
            rospy.loginfo(f"从模型配置读取 imgsz: {self.imgsz}")
        except:
            self.imgsz = 640
            rospy.logwarn(f"无法读取模型 imgsz，使用默认 640")
        
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.box_color = (255, 0, 0)  
        self.text_color = (255, 255, 255)
        
        self.image_sub = rospy.Subscriber("/cam", Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge错误: {e}")
            return
            
        results = self.model(
            source=cv_image,
            imgsz=self.imgsz,
            conf=self.confidence_threshold,
            iou=0.45,
            max_det=300,
            verbose=False,
            device=0 if torch.cuda.is_available() else 'cpu'
        )
        
        annotated_frame = self.draw_detections(cv_image, results)
        
        h, w = annotated_frame.shape[:2]
        resized = cv2.resize(annotated_frame, (w//2, h//2))
        cv2.imshow('YOLOv8 Detection (Blue Boxes)', resized)
        cv2.waitKey(1)
        
    def draw_detections(self, image, results):
        annotated = image.copy()
        h, w = image.shape[:2]
        img_area = h * w
        max_area = img_area / 5.0              
        detected_names = set()                      
        
        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue                      
                
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy())
                name = self.model.names[cls_id]
                
                box_w = x2 - x1
                box_h = y2 - y1
                box_area = box_w * box_h
                
                # === 过滤1：面积 > 1/5 图像 ===
                if box_area > max_area:
                    continue
                
                # === 过滤2：长宽比太离谱 ===
                aspect = box_w / box_h if box_h > 0 else 999
                if aspect > 4.0 or aspect < 0.25:
                    continue
                
                # === 过滤3：太贴边（距离边缘 < 框宽或框高的 1/10）===
                min_margin_w = box_w / 10.0
                min_margin_h = box_h / 10.0
                if (x1 < min_margin_w or y1 < min_margin_h or 
                    x2 > w - min_margin_w or y2 > h - min_margin_h):
                    continue
                
                detected_names.add(name)
                
                # 限制坐标
                x1 = max(0, min(x1, w-1))
                y1 = max(0, min(y1, h-1))
                x2 = max(0, min(x2, w-1))
                y2 = max(0, min(y2, h-1))
                
                cv2.rectangle(annotated, (x1, y1), (x2, y2), self.box_color, 3)
                label = f"{name} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                label_bg_top = max(y1 - th - 10, 0)
                cv2.rectangle(annotated, (x1, label_bg_top), (x1 + tw, y1), self.box_color, -1)
                cv2.putText(annotated, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.text_color, 2)
        
        for name in detected_names:
            rospy.loginfo(f"检测到 {name}")
        
        return annotated
    
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("正在关闭节点...")
        finally:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = YOLODetector()
    detector.run()
