# -*- coding: utf-8 -*-
"""
ArUco跟踪 + 手动对焦辅助工具
功能：
1. 实时检测ArUco码，显示ID、距离、位姿（和aruco_tracker.py一致）
2. 实时计算画面清晰度，帮你拧对焦环找到最清晰位置
3. 提示当前距离下的最佳对焦刻度
操作：[q] 退出
"""
import cv2 as cv
import numpy as np
import os
import sys
import time

# === 参数（和aruco_tracker.py保持一致）===
DICT = cv.aruco.DICT_6X6_250
MARKER_SIZE_MM = 50.0
# =====================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMS_FILE = os.path.join(SCRIPT_DIR, "camera_params.npz")
sys.path.append(SCRIPT_DIR)
from hik_camera import HikCamera

# 加载相机内参
if not os.path.exists(PARAMS_FILE):
    print(f"[错误] 找不到 {PARAMS_FILE}，请先运行 calibrate.py")
    sys.exit(1)
data = np.load(PARAMS_FILE)
K = data["K"]
d = data["d"]

dictionary = cv.aruco.getPredefinedDictionary(DICT)
detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())

# 清晰度跟踪状态
best_sharpness = 0
best_focus_z = 0

def rotation_vector_to_euler(rvec):
    R, _ = cv.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

def draw_axis(img, corner, img_pts):
    origin = tuple(corner[0].astype(int))
    img = cv.line(img, origin, tuple(img_pts[0][0].astype(int)), (0, 0, 255), 2)
    img = cv.line(img, origin, tuple(img_pts[1][0].astype(int)), (0, 255, 0), 2)
    img = cv.line(img, origin, tuple(img_pts[2][0].astype(int)), (255, 0, 0), 2)
    return img

def main():
    global best_sharpness, best_focus_z
    cam = HikCamera(dev_index=0)
    cam.open()
    cam.start_grabbing()

    print("\n=== 对焦辅助模式 ===")
    print("拧镜头对焦环，直到画面上出现绿色【对焦最佳】提示")
    print("按 q 退出\n")

    t_prev = time.time()
    fps = 0
    last_z = 0

    while True:
        frame = cam.get_frame()
        if frame is None:
            continue

        # 计算画面清晰度（拉普拉斯方差，越大越清晰）
        sharpness = cv.Laplacian(frame, cv.CV_64F).var()

        # 检测ArUco
        corners, ids, rejected = detector.detectMarkers(frame)
        vis = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

        if ids is not None and len(ids) > 0:
            cv.aruco.drawDetectedMarkers(vis, corners, ids)
            half = MARKER_SIZE_MM / 2
            obj_pts = np.float32([[-half, half, 0], [half, half, 0],
                                  [half, -half, 0], [-half, -half, 0]])

            for i, mid in enumerate(ids.flatten()):
                ret, rvec, tvec = cv.solvePnP(obj_pts, corners[i], K, d, flags=cv.SOLVEPNP_IPPE_SQUARE)
                tvec = tvec.ravel()
                dist = np.linalg.norm(tvec)
                last_z = dist

                # 画坐标轴
                img_pts, _ = cv.projectPoints(
                    np.float32([[0,0,0], [MARKER_SIZE_MM,0,0], [0,MARKER_SIZE_MM,0], [0,0,MARKER_SIZE_MM]]),
                    rvec, tvec, K, d)
                vis = draw_axis(vis, corners[i][0], img_pts)

                # 画ID和距离
                text_pos = tuple(corners[i][0][0].astype(int))
                cv.putText(vis, f"ID:{mid}", (text_pos[0], text_pos[1]-10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                cv.putText(vis, f"{dist:.0f}mm", (text_pos[0], text_pos[1]-30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                euler = rotation_vector_to_euler(rvec)
                print(f"ID={mid:3d} | Z={dist:6.1f}mm | 清晰度={sharpness:.1f}")

        # 更新最佳对焦记录
        if sharpness > best_sharpness:
            best_sharpness = sharpness
            best_focus_z = last_z

        # 在画面顶部画状态信息
        info_y = 30
        cv.putText(vis, f"当前距离: {last_z/1000:.2f}m", (10, info_y),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv.putText(vis, f"建议对焦刻度: {last_z/1000:.1f}m", (10, info_y+30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv.putText(vis, f"清晰度: {sharpness:.0f} (最佳: {best_sharpness:.0f})", (10, info_y+60),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # 当当前清晰度达到历史峰值时，提示对焦最佳
        if abs(sharpness - best_sharpness) < 5 and sharpness > 10:
            cv.putText(vis, "✓ 对焦最佳", (10, info_y+100),
                       cv.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 3)

        # FPS
        t_now = time.time()
        dt = t_now - t_prev
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        t_prev = t_now
        cv.putText(vis, f"FPS: {fps:.1f}", (vis.shape[1]-150, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv.imshow("Focus Assist", vis)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cam.close()
    cv.destroyAllWindows()
    print(f"\n本次最佳对焦清晰度: {best_sharpness:.0f}，对应距离: {best_focus_z/1000:.2f}m")

if __name__ == "__main__":
    main()
