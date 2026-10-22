# -*- coding: utf-8 -*-
"""
把所有ArUco码排版到一张A4纸上，生成可直接打印的PDF
打印时选【实际大小/100%】，打出来每个码黑框正好50mm
"""
import cv2 as cv
import numpy as np
import os

# === 参数 ===
DPI = 300
MM_PER_INCH = 25.4
px_per_mm = DPI / MM_PER_INCH

# A4尺寸（mm）
A4_W_MM = 210
A4_H_MM = 297
# 页面边距（mm）
MARGIN_MM = 12
# 每个码加上白边后的总尺寸（mm），和generate_marker.py一致：50mm码 + 两边各5mm白边 = 60mm
CELL_MM = 60
# ============

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MARKER_DIR = os.path.join(SCRIPT_DIR, "markers")
OUT_FILE = os.path.join(SCRIPT_DIR, "aruco_all_print.png")

# A4画布像素
a4_w_px = int(A4_W_MM * px_per_mm)
a4_h_px = int(A4_H_MM * px_per_mm)
page = np.ones((a4_h_px, a4_w_px), dtype=np.uint8) * 255  # 白底

cell_px = int(CELL_MM * px_per_mm)
margin_px = int(MARGIN_MM * px_per_mm)

# 每行/列能放几个
cols = int((A4_W_MM - 2*MARGIN_MM) // CELL_MM)
rows = int((A4_H_MM - 2*MARGIN_MM) // CELL_MM)
print(f"页面可放: {cols}列 x {rows}行 = {cols*rows}个码")

# 读取所有marker图
marker_files = sorted([f for f in os.listdir(MARKER_DIR) if f.endswith('.png')])
print(f"找到 {len(marker_files)} 个marker")

for idx, fname in enumerate(marker_files):
    row = idx // cols
    col = idx % cols
    if row >= rows:
        print(f"警告: 第{idx+1}个码超出页面，放不下了")
        break
    img_path = os.path.join(MARKER_DIR, fname)
    # 用imdecode读，兼容任何路径
    img_data = np.fromfile(img_path, dtype=np.uint8)
    img = cv.imdecode(img_data, cv.IMREAD_GRAYSCALE)
    # 缩放到cell尺寸（和我们生成的尺寸一致，其实不用缩，但保险）
    img = cv.resize(img, (cell_px, cell_px), interpolation=cv.INTER_NEAREST)
    # 贴到画布上
    x = margin_px + col * cell_px
    y = margin_px + row * cell_px
    page[y:y+cell_px, x:x+cell_px] = img

# 保存PNG，用imencode写
success, buffer = cv.imencode('.png', page)
if success:
    buffer.tofile(OUT_FILE)
    print(f"\n完成！打印文件已生成: {OUT_FILE}")
    print("打印注意事项:")
    print("  1. 打开PDF → 打印 → 纸张大小选A4")
    print("  2. 缩放选项必须选【实际大小/100%/无缩放】，绝对不能选【适应页面】")
    print("  3. 打印后量其中一个码的黑框边长，必须正好50mm")
else:
    print("生成PDF失败")
