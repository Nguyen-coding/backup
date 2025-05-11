import cv2
import numpy as np

# 카메라 장치 ID 설정
camera_id = 4
cap = cv2.VideoCapture(camera_id)

# 카메라가 열려 있는지 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    # 그레이스케일 변환 및 블러 적용
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 2)
    edge = cv2.Canny(gray, 50, 150, 3)

    # Hough 변환을 이용한 선 감지
    lines = cv2.HoughLinesP(edge, 1, np.pi / 180, 200, minLineLength=50, maxLineGap=10)

    if lines is not None:
        min_angle = -1.0 * (np.pi / 180.0)
        max_angle = 1.0 * (np.pi / 180.0)

        lowest_line_y = 0
        horizontal_lines = []

        # 감지된 선에서 수평선만 필터링
        for l in lines:
            x1, y1, x2, y2 = l[0]
            angle = np.arctan2(y2 - y1, x2 - x1)

            if min_angle < angle < max_angle:
                horizontal_lines.append((x1, y1, x2, y2))
                lowest_line_y = max(lowest_line_y, y1, y2)

        merged_lines = []
        height_threshold = 50

        # 수평선 그룹화
        for l1 in horizontal_lines:
            merged = False
            for i, l2 in enumerate(merged_lines):
                if abs(l1[1] - l2[1]) < height_threshold and abs(l1[3] - l2[3]) < height_threshold:
                    l2[0] = min(l2[0], l1[0])
                    l2[1] = (l2[1] + l1[1]) // 2
                    l2[2] = max(l2[2], l1[2])
                    l2[3] = (l2[3] + l1[3]) // 2
                    merged = True
                    break

            if not merged:
                merged_lines.append(list(l1))

        # 그룹화되지 않은 선을 빨간색으로 그리기
        for x1, y1, x2, y2 in horizontal_lines:
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)  # 빨간색 (BGR: 0, 0, 255)

        # 그룹화된 선을 파란색으로 그리기
        for x1, y1, x2, y2 in merged_lines:
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)  # 파란색 (BGR: 255, 0, 0)

        if lowest_line_y > 0:
            print(f"가장 낮은 가로선의 높이: {lowest_line_y}")

    # 프레임을 화면에 표시
    cv2.imshow("Frame", frame)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()