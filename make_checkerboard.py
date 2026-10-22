# -*- coding: utf-8 -*-
"""
生成和calibrate.py参数完全对应的棋盘格标定板，A4纸打印
内角点9x6，每个小方格20mm
"""
import cv2 as cv
import numpy as np
import os

# === 参数（和calibrate.py对应）===
DPI = 300
MM_PER_INCH = 25.4
px_per_mm = DPI / MM_PER_INCH

# A4尺寸
A4_W_MM = 210
A4_H_MM = 297

# 棋盘格参数：内角点9x6 → 10列x7行方格
BOARD_COLS_CORNERS = 9
BOARD_ROWS_CORNERS = 6
SQUARE_SIZE_MM = 20.0   # 每个小方格边长，打出来量一下，不对就改这个数
# ================================

# 转像素
a4_w = int(A4_W_MM * px_per_mm)
a4_h = int(A4_H_MM * px_per_mm)
square_px = int(SQUARE_SIZE_MM * px_per_mm)

# 棋盘总尺寸
board_cols = BOARD_COLS_CORNERS + 1   # 10个方格
board_rows = BOARD_ROWS_CORNERS + 1   # 7个方格
board_w = board_cols * square_px
board_h = board_rows * square_px

# 创建A4白底画布
page = np.ones((a4_h, a4_w), dtype=np.uint8) * 255

# 棋盘左上角位置（居中）
start_x = (a4_w - board_w) // 2
start_y = (a4_h - board_h) // 2

# 画黑白棋盘
for i in range(board_rows):
    for j in range(board_cols):
        color = 0 if (i + j) % 2 == 0 else 255  # 黑块
        y0 = start_y + i * square_px
        y1 = y0 + square_px
        x0 = start_x + j * square_px
        x1 = x0 + square_px
        page[y0:y1, x0:x1] = color

# 保存
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(SCRIPT_DIR, "checkerboard_9x6_20mm.png")
success, buffer = cv.imencode('.png', page)
if success:
    buffer.tofile(OUT_FILE)
    print(f"标定板已生成: {OUT_FILE}")
    print(f"参数: 内角点 {BOARD_COLS_CORNERS}x{BOARD_ROWS_CORNERS}, 方格边长 {SQUARE_SIZE_MM}mm")
    print("打印注意:")
    print("  1. 打开图片 → 打印 → A4竖版 → 缩放选【实际大小/100%】")
    print(f"  2. 打印后量一个小方格的实际边长，如果不是{SQUARE_SIZE_MM}mm，告诉我实际值，我改calibrate.py里的对应参数")
else:
    print("生成失败")
