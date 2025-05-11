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

def get_step_mask_and_distances(points, depth_shape, plane_coeffs, threshold=0.007):
    a, b, c, d = plane_coeffs
    h, w = depth_shape
    pts = points.reshape(h, w, 3)   
    dist_map = np.abs(pts.dot(np.array([a, b, c])) + d) / np.linalg.norm([a, b, c])
    step_mask = dist_map > threshold
    plane_mask = dist_map <= threshold
    return (step_mask.astype(np.uint8) * 255,
            (plane_mask.astype(np.uint8) * 255),
            dist_map)

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
        #depth_ds = cv2.resize(self.depth_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        #depth_ds = np.nan_to_num(depth_ds, nan=0.0)
        #h, w = depth_ds.shape
        #fx, fy = self.fx , self.fy 
        #cx, cy = self.cx / 2.0, self.cy / 2.0
        depth_ds = self.depth_image.copy()
        depth_ds = np.nan_to_num(depth_ds, nan=0.0)
        h, w = depth_ds.shape
        fx, fy = self.fx, self.fy
        cx, cy = self.cx, self.cy

        

        zs = depth_ds / 1000.0
        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        xs = (xs - cx) * zs / fx
        ys = (ys - cy) * zs / fy
        points = np.stack((xs, ys, zs), axis=-1).reshape(-1, 3)

        roi_mask = np.zeros_like(zs, dtype=bool)
    #    roi_mask[int(h * 0.55):, int(w * 0.2):int(w * 0.8)] = True
#        roi_mask[int(h * 0.8):, int(w * 0.35):int(w * 0.65)] = True
#        roi_mask[int(h * 0.5):, int(w * 0.2):int(w * 0.8)] = True
        #roi_mask[int(h * 0.7):, int(w * 0.3):int(w * 0.7)] = True
        roi_mask[int(h * 0.75):int(h * 0.95), int(w * 0.35):int(w * 0.65)] = True

        # 시각화용 ROI 이미지 생성
        roi_vis = (roi_mask.astype(np.uint8) * 255)
        roi_vis = cv2.resize(roi_vis, (rgb.shape[1], rgb.shape[0]))
        roi_color = cv2.cvtColor(roi_vis, cv2.COLOR_GRAY2BGR)
        roi_overlay = cv2.addWeighted(rgb, 0.6, roi_color, 0.4, 0)

        cv2.imshow("ROI Region (Yellow Highlight)", roi_overlay)


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
            points, depth_ds.shape, self.plane, threshold=0.01)
        
        ### 스텝 / 플레인 마스크 시각화 ###
        step_vis = cv2.applyColorMap(step_mask, cv2.COLORMAP_JET)
        plane_vis = cv2.applyColorMap(plane_mask, cv2.COLORMAP_SUMMER)

        cv2.imshow("Step Mask (JET)", step_vis)
        cv2.imshow("Plane Mask (SUMMER)", plane_vis)

        kernel = np.ones((3, 3), np.uint8)
        filt = cv2.morphologyEx(step_mask, cv2.MORPH_CLOSE, kernel)
        filt = cv2.dilate(filt, kernel, iterations=2)  # 윤곽선 생성 유도

        plane_mask_rs = cv2.resize(plane_mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        overlay = rgb.copy()
        overlay[plane_mask_rs > 0] = (0, 255, 255)
        cv2.addWeighted(overlay, 0.4, rgb, 0.6, 0, rgb)
        
        contours, _ = cv2.findContours(filt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            x, y, ww, hh = cv2.boundingRect(c)
            #if ww * hh < 50:
            if ww * hh < 50 or y + hh < int(h * 0.5):  # 화면 위쪽 제거
                continue
            mask_cont = np.zeros_like(filt)
            cv2.drawContours(mask_cont, [c], -1, 255, -1)
            dists = dist_map[mask_cont.astype(bool)]
            # zs_in = points.reshape(h, w, 3)[mask_cont.astype(bool), 2]
            mask = mask_cont.astype(bool)  # ✅ 여기
            dists = dist_map[mask]
            zs_patch = points.reshape(h, w, 3)[mask]  # ✅ 3D 포인트 선택
            zs_in = zs_patch[:, 2]                    # ✅ Z값만 추출
            if len(dists) == 0:
                continue
            #height = np.median(dists) * 2
            # 아래와 같이 90% 분위수 기준으로 계산
            height = np.percentile(dists, 90) * 2

            forward = np.median(zs_in)

            
            #pixel_width = ww ###
            #width_m = pixel_width * forward / self.fx  # fx 기준 실제 거리 ###
            if not (0.5 < forward < 1.2):  # 30cm ~ 1.2m 사이 단차만 선택
                continue

            if height < 0.04 or height > 0.40: # 조정 필요
                continue
            if forward > 2.0:
                continue
            
            #candidates.append({'rect': (x, y, ww, hh), 'h': height, 'f': forward})
            candidates.append({'rect': (x, y, ww, hh), 'h': height, 'f': forward, 'contour': c})

            


        if not candidates:
            self.get_logger().info("No step candidates found.")
            cv2.imshow("Optimized Step Detection", rgb)
            cv2.waitKey(1)
            return

        #best = min(candidates, key=lambda c: c['f'])
        # 후보들 중에서 (1) height가 충분하고, (2) contour 면적이 충분히 넓고, (3) 거리도 적당한 것
        filtered = [c for c in candidates if c['h'] > 0.05 and cv2.contourArea(c['contour']) > 1000]
        if filtered:
            best = min(filtered, key=lambda c: c['f'])  # 그중에서 가장 가까운 단차 선택
        else:
            best = max(candidates, key=lambda c: cv2.contourArea(c['contour']))

        #best = max(candidates, key=lambda c: cv2.contourArea(c['contour']) / (c['f'] + 0.001))

        
        x, y, ww, hh = best['rect']
        height = best['h']  
        forward = best['f']
        
        #cv2.drawContours(rgb, [best['contour']], -1, (0, 255, 0), 2) ####


        now = time.time()

        if now - self.last_step_time >= self.step_cooldown:
            self.last_step_time = now
            msg = StepEvent() ###
            msg.height = height ###
            self.step_pub.publish(msg) ###
        # 박스 표시
        #cv2.rectangle(rgb, (x * 2, y * 2), ((x + ww) * 2, (y + hh) * 2), (0, 0, 255), 2)
        cv2.rectangle(rgb, (x, y), (x + ww, y + hh), (0, 0, 255), 2)


        # 중심점 및 텍스트 표시
        #center_x = int((x + ww / 2) * 2)
        #center_y = int((y + hh / 2) * 2)
        center_x = int((x + ww / 2))
        center_y = int((y + hh / 2))
        #cv2.drawContours(rgb, [c], -1, (0, 255, 0), 2)  # 초록색 외곽선
        #cv2.drawMarker(rgb, (center_x, center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        cv2.drawMarker(rgb, (center_x, center_y), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
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
