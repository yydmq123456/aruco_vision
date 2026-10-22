# -*- coding: utf-8 -*-
"""
生成 ArUco 标记图，用于打印
用法: python generate_marker.py
会在 markers/ 目录生成 ID=0..9 的 ArUco 码 PNG，物理边长 50mm
打印时选"实际大小"，不要缩放
"""
import cv2 as cv
import numpy as np
import os

# === 参数 ===
DICT = cv.aruco.DICT_6X6_250   # 6x6 比特，250 个 ID
MARKER_SIZE_MM = 50            # 物理边长（毫米）
DPI = 300                      # 打印分辨率
NUM_MARKERS = 10               # 生成 ID 0..9
# ============

OUT_DIR = os.path.join(os.path.dirname(__file__), "markers")
os.makedirs(OUT_DIR, exist_ok=True)

# 毫米 → 像素
px_per_mm = DPI / 25.4
marker_px = int(MARKER_SIZE_MM * px_per_mm)

dictionary = cv.aruco.getPredefinedDictionary(DICT)

for mid in range(NUM_MARKERS):
    img = np.zeros((marker_px, marker_px), dtype=np.uint8)
    cv.aruco.generateImageMarker(dictionary, mid, marker_px, img, 1)
    # 加白边，方便打印后裁剪
    border = int(5 * px_per_mm)
    img = cv.copyMakeBorder(img, border, border, border, border,
                            cv.BORDER_CONSTANT, value=255)
    path = os.path.join(OUT_DIR, f"aruco_id{mid}_50mm.png")
    # 用 imencode 绕过 OpenCV 中文路径无法写入的问题
    success, buffer = cv.imencode('.png', img)
    if success:
        buffer.tofile(path)
    else:
        raise IOError(f"写入失败: {path}")
    print(f"生成: {path}  ({MARKER_SIZE_MM}mm x {MARKER_SIZE_MM}mm, {DPI} DPI)")

print(f"\n完成！共 {NUM_MARKERS} 张，在 {OUT_DIR}")
print("打印注意:")
print("  1. 用图片查看器打开 → 打印 → 选【实际大小/100%】，不要选【适应边框】")
print(f"  2. 打印后用尺子量黑框实际边长，应该正好是 {MARKER_SIZE_MM}mm")
print("  3. 如果不准，按 实际尺寸/理论尺寸 比例缩放图片后重打")
