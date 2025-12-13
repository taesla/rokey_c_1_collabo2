########## YoloOBBModel - OBB 모델 + 각도 추출 ##########
import os
import json
import time
import math
from collections import Counter

import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np
import cv2


PACKAGE_NAME = "pick_and_place_text"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

# OBB 모델 사용
YOLO_MODEL_FILENAME = "best_obb.pt"
YOLO_CLASS_NAME_JSON = "class_name_obb.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, "resource", YOLO_CLASS_NAME_JSON)


class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}
        print(f"[YoloOBB] Loaded model: {YOLO_MODEL_FILENAME}")
        print(f"[YoloOBB] Classes: {self.reversed_class_dict}")

    def calculate_angle(self, obb_points):
        """OBB 4개 꼭짓점에서 회전 각도 계산 (도 단위)"""
        # obb_points: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        p1, p2 = obb_points[0], obb_points[1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def get_frames(self, img_node, duration=1.0):
        """get frames while target_time"""
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("[YoloOBB] No frames captured in %.2f seconds", duration)

        print(f"[YoloOBB] {len(frames)} frames captured")
        return list(frames.values())

    def get_best_detection(self, img_node, target):
        """OBB 탐지 + 각도 반환"""
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None, None, None

        results = self.model(frames, verbose=False)
        detections = self._aggregate_obb_detections(results)

        label_id = self.reversed_class_dict.get(target.lower())
        if label_id is None:
            print(f"[YoloOBB] Unknown target: {target}")
            return None, None, None
        
        print(f"[YoloOBB] Looking for label_id: {label_id} ({target})")
        print(f"[YoloOBB] Detections: {detections}")

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("[YoloOBB] No matches found for the target label.")
            return None, None, None
        
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"], best_det["angle"]

    def get_best_detection_with_viz(self, img_node, target):
        """Detection 결과와 함께 시각화된 이미지를 반환합니다."""
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None, None, None, None

        results = self.model(frames, verbose=False)
        detections = self._aggregate_obb_detections(results)

        label_id = self.reversed_class_dict.get(target.lower())
        if label_id is None:
            print(f"[YoloOBB] Unknown target: {target}")
            return None, None, None, frames[-1]

        print(f"[YoloOBB] label_id: {label_id}")
        print(f"[YoloOBB] detections: {detections}")

        # 마지막 프레임에 OBB 그리기
        annotated_frame = frames[-1].copy()
        for det in detections:
            obb_points = det["obb_points"]
            score = det["score"]
            label = det["label"]
            angle = det["angle"]
            
            # 클래스명 찾기
            class_name = None
            for name, lid in self.reversed_class_dict.items():
                if lid == label:
                    class_name = name
                    break
            
            # OBB 그리기
            pts = np.array(obb_points, dtype=np.int32)
            color = (0, 255, 0) if label == label_id else (255, 0, 0)
            cv2.polylines(annotated_frame, [pts], isClosed=True, color=color, thickness=2)
            
            # 중심점 계산
            cx = int(np.mean([p[0] for p in obb_points]))
            cy = int(np.mean([p[1] for p in obb_points]))
            
            # 라벨 표시
            cv2.putText(annotated_frame, f"{class_name} {score:.2f} | {angle:.1f}deg", 
                       (cx - 60, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(annotated_frame, (cx, cy), 5, color, -1)

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("[YoloOBB] No matches found for the target label.")
            return None, None, None, annotated_frame
        
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"], best_det["angle"], annotated_frame

    def _aggregate_obb_detections(self, results, confidence_threshold=0.5):
        """
        OBB 결과에서 detection 정보 추출
        """
        raw = []
        for res in results:
            if res.obb is not None and len(res.obb) > 0:
                for i in range(len(res.obb)):
                    obb_xyxyxyxy = res.obb.xyxyxyxy[i].cpu().numpy()  # 4개 꼭짓점
                    score = float(res.obb.conf[i].cpu().numpy())
                    label = int(res.obb.cls[i].cpu().numpy())
                    
                    if score >= confidence_threshold:
                        # 중심점 기반 박스 계산 (depth 서비스용)
                        cx = np.mean([p[0] for p in obb_xyxyxyxy])
                        cy = np.mean([p[1] for p in obb_xyxyxyxy])
                        
                        # 각도 계산
                        angle = self.calculate_angle(obb_xyxyxyxy)
                        
                        raw.append({
                            "box": [cx, cy, cx, cy],  # 중심점 기반
                            "obb_points": obb_xyxyxyxy.tolist(),
                            "score": score, 
                            "label": label,
                            "angle": angle
                        })

        # 같은 클래스 중 가장 높은 confidence만 유지 (간단한 NMS)
        final = []
        seen_labels = set()
        sorted_raw = sorted(raw, key=lambda x: x["score"], reverse=True)
        
        for det in sorted_raw:
            if det["label"] not in seen_labels:
                final.append(det)
                seen_labels.add(det["label"])

        return final

    def _iou(self, box1, box2):
        """
        Compute Intersection over Union (IoU) between two boxes [x1, y1, x2, y2].
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0
