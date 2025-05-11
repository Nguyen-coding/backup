#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hot.msg import StepEvent
import numpy as np
import cv2
import time

def estimate_ground_plane_ransac(points, max_iterations=50, distance_threshold=0.01): ### 0.03 > 0.015
    best_plane = None
    max_inliers = 0
    for _ in range(max_iterations):
        sample_idx = np.random.choice(points.shape[0], 3, replace=False)
        sample = points[sample_idx]
        v1 = sample[1] - sample[0]
        v2 = sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm == 0:
            continue
        normal_unit = normal / norm
        a, b, c = normal_unit
        d = -np.dot(normal_unit, sample[0])
        distances = np.abs(points.dot(normal_unit) + d)
        inliers = distances < distance_threshold
        num_inliers = np.sum(inliers)
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_plane = (a, b, c, d)
    return best_plane

def get_step_mask_and_distances(points, depth_shape, plane_coeffs, threshold=0.01):
    a, b, c, d = plane_coeffs
    h, w = depth_shape
    pts = points.reshape(h, w, 3)   
    dist_map = np.abs(pts.dot(np.array([a, b, c])) + d) / np.linalg.norm([a, b, c])
    step_mask = dist_map > threshold
    plane_mask = dist_map <= threshold
    return (step_mask.astype(np.uint8) * 255,
            (plane_mask.astype(np.uint8) * 255),
            dist_map)
"""
def detect_step_line(step_mask, dist_map, points, rgb=None, scale=2):
    kernel = np.ones((3, 3), np.uint8)
    filt = cv2.morphologyEx(step_mask, cv2.MORPH_OPEN, kernel)

    edges = cv2.Canny(filt, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=30, maxLineGap=10)

    if lines is None:
        return None

    horizontal_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if abs(angle) < 10:  # 수평
            horizontal_lines.append((x1, y1, x2, y2))

    if not horizontal_lines:
        return None

    bottom_line = max(horizontal_lines, key=lambda l: max(l[1], l[3]))
    x1, y1, x2, y2 = bottom_line
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)

    h, w = dist_map.shape
    center_x = np.clip(center_x, 0, w - 1)
    center_y = np.clip(center_y, 0, h - 1)

    height = dist_map[center_y, center_x]
    forward = points.reshape(h, w, 3)[center_y, center_x, 2]
    text = f"H:{height:.2f}m D:{forward:.2f}m"

    if rgb is not None:
        cv2.line(rgb, (x1 * scale, y1 * scale), (x2 * scale, y2 * scale), (255, 0, 255), 2)
        cv2.drawMarker(rgb, (center_x * scale, center_y * scale), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
        cv2.putText(rgb, f"H:{height:.2f}m D:{forward:.2f}m",
                    (center_x * scale + 5, center_y * scale - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    return {'height': height, 'forward': forward, 'position': (center_x, center_y)}

"""
"""
def detect_step_line(step_mask, dist_map, points, rgb=None, scale=2):
    kernel = np.ones((3, 3), np.uint8)
    filt = cv2.morphologyEx(step_mask, cv2.MORPH_OPEN, kernel)

    edges = cv2.Canny(filt, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=30, maxLineGap=10)

    if lines is None:
        return None

    horizontal_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if abs(angle) < 10:  # 수평선
            horizontal_lines.append((x1, y1, x2, y2))

    if len(horizontal_lines) < 2:
        return None

    # ↓ 가장 아래에서부터 위로 정렬 (y 좌표 기준)
    lines_sorted = sorted(horizontal_lines, key=lambda l: max(l[1], l[3]))
    target_line = lines_sorted[1]  # 두 번째로 낮은 선

    x1, y1, x2, y2 = target_line
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)

    h, w = dist_map.shape
    center_x = np.clip(center_x, 0, w - 1)
    center_y = np.clip(center_y, 0, h - 1)

    #height = dist_map[center_y, center_x]
    #forward = points.reshape(h, w, 3)[center_y, center_x, 2]

    patch = dist_map[center_y-2:center_y+3, center_x-2:center_x+3]
    height = np.median(patch)

    zs_patch = points.reshape(h, w, 3)[center_y-2:center_y+3, center_x-2:center_x+3, 2]
    forward = np.median(zs_patch)


    if rgb is not None:
        cv2.line(rgb, (x1 * scale, y1 * scale), (x2 * scale, y2 * scale), (255, 0, 255), 2)
        cv2.drawMarker(rgb, (center_x * scale, center_y * scale), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
        cv2.putText(rgb, f"H:{height:.2f}m D:{forward:.2f}m",
                    (center_x * scale + 5, center_y * scale - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    return {'height': height, 'forward': forward, 'position': (center_x, center_y)}
"""

"""
def detect_step_line(step_mask, dist_map, points, rgb=None, fx=604.12, scale=2):
    kernel = np.ones((3, 3), np.uint8)
    filt = cv2.morphologyEx(step_mask, cv2.MORPH_OPEN, kernel)

    edges = cv2.Canny(filt, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=30, maxLineGap=10)

    if lines is None:
        return None

    h, w = dist_map.shape
    horizontal_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if abs(angle) < 10:
            mid_x = int((x1 + x2) / 2)
            mid_y = int((y1 + y2) / 2)

            zs_patch = points.reshape(h, w, 3)[mid_y-2:mid_y+3, mid_x-2:mid_x+3, 2]
            depth = np.median(zs_patch)
            if depth <= 0 or np.isnan(depth):
                continue

            pixel_len = np.linalg.norm([x2 - x1, y2 - y1])
            width_m = pixel_len * depth / fx

            if width_m > 0.5:
                continue

            horizontal_lines.append((x1, y1, x2, y2))

    if len(horizontal_lines) < 1:
        return None

    # 두 번째로 낮은 수평선 기준
    lines_sorted = sorted(horizontal_lines, key=lambda l: (l[1] + l[3]) / 2.0)
    target_line = lines_sorted[1] if len(lines_sorted) >= 2 else lines_sorted[0]

    x1, y1, x2, y2 = target_line
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)

    center_x = np.clip(center_x, 2, w - 3)
    center_y = np.clip(center_y, 2, h - 3)

    patch = dist_map[center_y-2:center_y+3, center_x-2:center_x+3]
    zs_patch = points.reshape(h, w, 3)[center_y-2:center_y+3, center_x-2:center_x+3, 2]

    height = np.median(patch)
    forward = np.median(zs_patch)

    if rgb is not None:
        cv2.line(rgb, (x1 * scale, y1 * scale), (x2 * scale, y2 * scale), (255, 0, 255), 2)
        cv2.drawMarker(rgb, (center_x * scale, center_y * scale), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
        cv2.putText(rgb, f"H:{height:.2f}m D:{forward:.2f}m",
                    (center_x * scale + 5, center_y * scale - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    return {
        'height': height,
        'forward': forward,
        'position': (center_x, center_y)
    }
"""

class StepDetectorOptimized(Node):
    def __init__(self):
        super().__init__('step_detector_optimized')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_image = None
        self.rgb_image = None
        self.frame_count = 0
        self.plane = None
        self.step_pub = self.create_publisher(StepEvent, '/step_detected', 10) ###
        self.last_step_time = 0.0
        self.step_cooldown = 15.0
        # 카메라 내부 파라미터
        self.fx, self.fy = 604.12383027, 604.1441706
        self.cx, self.cy = 324.92216095, 243.2522445

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def process(self):
        if self.depth_image is None or self.rgb_image is None:
            return

        self.frame_count += 1
        rgb = self.rgb_image.copy()

        # 포인트 클라우드 생성
        depth_ds = cv2.resize(self.depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        depth_ds = np.nan_to_num(depth_ds, nan=0.0)
        h, w = depth_ds.shape
        fx, fy = self.fx, self.fy
        cx, cy = self.cx / 2.0, self.cy / 2.0

        zs = depth_ds / 1000.0
        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        xs = (xs - cx) * zs / fx
        ys = (ys - cy) * zs / fy
        points = np.stack((xs, ys, zs), axis=-1).reshape(-1, 3)

        roi_mask = np.zeros_like(zs, dtype=bool)
        roi_mask[int(h * 0.7):, int(w *0.3):int(w*0.7)] = True # 화면 아래 30%주으 좌우 40%만 검사
        #roi_mask[int(h * 0.8):, int(w * 0.35):int(w * 0.65)] = True

        valid = (zs > 0.1) & (zs < 5.0)
        final_mask = valid & roi_mask
        valid_pts = points[final_mask.flatten()]

        if self.frame_count % 10 == 0 and valid_pts.shape[0] >= 100:
            plane_candidate = estimate_ground_plane_ransac(valid_pts)
            if plane_candidate:
                a, b, c, _ = plane_candidate
                normal = np.array([a, b, c])
                angle = np.degrees(np.arccos(np.dot(normal, [0, 1, 0]) / np.linalg.norm(normal)))
                if angle < 30:
                    self.plane = plane_candidate
                else:
                    self.get_logger().warn(f"기울기 {angle:.1f}° → 바닥 아님")

        if self.plane is None:
            cv2.imshow("Optimized Step Detection", rgb)
            cv2.waitKey(1)
            return

        step_mask, plane_mask, dist_map = get_step_mask_and_distances(
            points, depth_ds.shape, self.plane, threshold=0.015)
        

        kernel = np.ones((3, 3), np.uint8)
        filt = cv2.morphologyEx(step_mask, cv2.MORPH_OPEN, kernel)

        plane_mask_rs = cv2.resize(plane_mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        overlay = rgb.copy()
        overlay[plane_mask_rs > 0] = (0, 255, 255)
        cv2.addWeighted(overlay, 0.4, rgb, 0.6, 0, rgb)
        
        contours, _ = cv2.findContours(filt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            x, y, ww, hh = cv2.boundingRect(c)
            if ww * hh < 500:
                continue
            mask_cont = np.zeros_like(filt)
            cv2.drawContours(mask_cont, [c], -1, 255, -1)
            dists = dist_map[mask_cont.astype(bool)]
            zs_in = points.reshape(h, w, 3)[mask_cont.astype(bool), 2]
            if len(dists) == 0:
                continue
            height = np.median(dists)
            forward = np.median(zs_in) / 2 
            if height < 0.03 or height > 0.20:
                continue
            if forward > 1.0:
                continue
            
            candidates.append({'rect': (x, y, ww, hh), 'h': height, 'f': forward})
            

        if not candidates:
            self.get_logger().info("No step candidates found.")
            cv2.imshow("Optimized Step Detection", rgb)
            cv2.waitKey(1)
            return

        best = min(candidates, key=lambda c: c['f'])
        
        
        x, y, ww, hh = best['rect']
        height = best['h'] * 2
        forward = best['f']
        
        """
        result = detect_step_line(filt, dist_map, points, rgb) ###?

        if result is None:
            self.get_logger().info("No step line detected.")
            cv2.imshow("Optimized Step Detection", rgb)
            cv2.waitKey(1)
            return

        height = result['height']
        forward = result['forward']
        center_x, center_y = result['position']
        """
        now = time.time()

        if now - self.last_step_time >= self.step_cooldown:
            self.last_step_time = now
            msg = StepEvent() ###
            msg.height = height ###
            self.step_pub.publish(msg) ###
        # 박스 표시
        cv2.rectangle(rgb, (x * 2, y * 2), ((x + ww) * 2, (y + hh) * 2), (0, 0, 255), 2)

        # 중심점 및 텍스트 표시
        center_x = int((x + ww / 2) * 2)
        center_y = int((y + hh / 2) * 2)
        cv2.drawMarker(rgb, (center_x, center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        text = f"H:{height:.2f}m D:{forward:.2f}m"
        cv2.putText(rgb, text, (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        self.get_logger().info(text)
        cv2.imshow("Optimized Step Detection", rgb)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = StepDetectorOptimized()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
