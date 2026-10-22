# -*- coding: utf-8 -*-
"""
相机标定脚本 v2（升级版）
新增能力：
  1. 单图重投影误差排行（标出 >2×中位数的图，按 [x] 一键剔除并重标）
  2. 整体/单图重投影误差随 camera_params.npz 一起保存
  3. 九宫格覆盖度提示（画面画 3x3 网格，报告统计四角/边缘覆盖，约束畸变）
  4. 实时距离档位提示（0.35/0.5/0.7m 三档，目标张数 8:10:6）
  5. 启动时自动载入 calib_images 历史图，支持分多次采集
  6. [b] 回退删除最后一张
  7. --out 指定输出文件名（水下标定存 camera_params_underwater.npz，不覆盖空气参数）
用法:
  python calibrate.py
  python calibrate.py --out camera_params_underwater.npz
操作:
  [空格] 采集一张 | [b] 删除最后一张 | [c] 标定并保存 | [x] 剔除高误差图并重标 | [q] 退出
"""
import argparse
import glob
import os
import sys
import datetime

import cv2 as cv
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from hik_camera import HikCamera

# === 标定板参数 ===
BOARD_COLS = 9          # 内角点列数
BOARD_ROWS = 6          # 内角点行数
SQUARE_SIZE_MM = 20.0   # 方格物理边长（毫米）
# ==================

# === 距离档位（按 CLAUDE.md 三档方案）：名称, 下限m, 上限m, 目标张数 ===
BANDS = [
    ("near 0.35m", 0.00, 0.43, 8),
    ("mid  0.50m", 0.43, 0.60, 10),
    ("far  0.70m", 0.60, 0.95, 6),
]
# =====================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(SCRIPT_DIR, "calib_images")
DEFAULT_OUT = os.path.join(SCRIPT_DIR, "camera_params.npz")
os.makedirs(IMG_DIR, exist_ok=True)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

# 棋盘格三维点（毫米）
objp = np.zeros((BOARD_COLS * BOARD_ROWS, 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM


def imread_unicode(path):
    return cv.imdecode(np.fromfile(path, dtype=np.uint8), cv.IMREAD_GRAYSCALE)


def imwrite_unicode(path, img):
    ok, buf = cv.imencode(".png", img)
    if ok:
        buf.tofile(path)
    return ok


def rough_K(w, h):
    """采集阶段的粗略内参，仅用于实时估算距离/档位（不是标定结果）"""
    f = 1.363 * w
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], np.float64)


def find_board(gray):
    """检测棋盘格并做亚像素细化，返回 (found, corners)"""
    found, corners = cv.findChessboardCorners(
        gray, (BOARD_COLS, BOARD_ROWS),
        cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK)
    if not found:
        return False, None
    corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def rvec_to_tilt(rvec):
    """棋盘相对相机的倾斜角（度），取 pitch/roll 绝对值较大者"""
    R, _ = cv.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-6:
        return 90.0
    pitch = np.degrees(np.arctan2(-R[2, 0], sy))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    return max(abs(pitch), abs(roll))


def band_of(z_m):
    for name, lo, hi, _target in BANDS:
        if lo <= z_m < hi:
            return name
    return "out-of-range"


def grid_coverage(records, w, h):
    """3x3 覆盖矩阵：每个格子里出现过多少张图的角点"""
    grid = np.zeros((3, 3), dtype=int)
    for r in records:
        cells = set()
        for (x, y) in r["corners"].reshape(-1, 2):
            gx = min(2, int(x / w * 3))
            gy = min(2, int(y / h * 3))
            cells.add((gy, gx))
        for (gy, gx) in cells:
            grid[gy, gx] += 1
    return grid


def load_existing():
    """启动时载入 calib_images 中已有图片，支持分多次采集"""
    records = []
    for f in sorted(glob.glob(os.path.join(IMG_DIR, "img_*.png"))):
        img = imread_unicode(f)
        if img is None:
            continue
        found, corners = find_board(img)
        if found:
            records.append({"fname": f, "corners": corners, "band": "?", "excluded": False})
        else:
            print(f"[跳过] 未检测到棋盘格: {os.path.basename(f)}")
    return records


def run_calibration(records, shape):
    """用未剔除的图标定，返回结果 dict（含每图 RMS）"""
    active = [r for r in records if not r["excluded"]]
    if len(active) < 5:
        print(f"[警告] 有效图不足 5 张（当前 {len(active)}），无法标定")
        return None
    objpts = [objp] * len(active)
    imgpts = [r["corners"] for r in active]
    ret, K, d, rvecs, tvecs = cv.calibrateCamera(objpts, imgpts, shape, None, None)
    per = []
    for i, r in enumerate(active):
        proj, _ = cv.projectPoints(objp, rvecs[i], tvecs[i], K, d)
        diff = proj.reshape(-1, 2) - r["corners"].reshape(-1, 2)
        per.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    return {"ret": float(ret), "K": K, "d": d, "per": np.array(per), "active": active}


def print_report(res, w, h, out_file):
    ret, K, d, per, active = res["ret"], res["K"], res["d"], res["per"], res["active"]
    med = float(np.median(per))
    fxy = abs(K[0, 0] - K[1, 1]) / ((K[0, 0] + K[1, 1]) / 2) * 100.0

    print(f"\n===== 标定报告（{len(active)} 张）=====")
    print(f"整体重投影误差 RMS: {ret:.4f} px   验收线 <0.5（优秀 <0.3）")
    print(f"fx={K[0, 0]:.2f}  fy={K[1, 1]:.2f}  cx={K[0, 2]:.2f}  cy={K[1, 2]:.2f}")
    print(f"fx/fy 差: {fxy:.3f}%   {'OK' if fxy < 1 else '[注意] 应 <1%'}")
    print(f"畸变: k1={d[0,0]:+.4f} k2={d[0,1]:+.4f} p1={d[0,2]:+.4f} p2={d[0,3]:+.4f} k3={d[0,4]:+.4f}")
    if abs(d[0, 4]) > 0.05:
        print("  [注意] k3 偏大（边缘畸变外推风险），多补四角/边缘图")

    print("\n单图误差排行（高→低）:")
    order = np.argsort(per)[::-1]
    for idx in order:
        r = active[idx]
        mark = "   <== 建议剔除(>2×中位)" if per[idx] > 2 * med else ""
        print(f"  {os.path.basename(r['fname'])}: {per[idx]:.4f} px{mark}")
    print(f"  中位数: {med:.4f} px")

    grid = grid_coverage(active, w, h)
    print("\n九宫格覆盖（数字=该格内出现角点的图数）:")
    for gy in range(3):
        print("  " + " | ".join(f"{grid[gy, gx]:3d}" for gx in range(3)))
    corners_cnt = [grid[0, 0], grid[0, 2], grid[2, 0], grid[2, 2]]
    if min(corners_cnt) < 3:
        print("  [注意] 四角覆盖不足（建议每角 >=3 张），畸变主要靠边缘图约束")

    print("\n距离档位分布:")
    for name, lo, hi, target in BANDS:
        cnt = sum(1 for r in active if r["band"] == name)
        flag = "OK" if cnt >= target else "不足"
        print(f"  {name}: {cnt}/{target}  {flag}")

    # 保存（含 rms 与每图误差）
    np.savez(out_file,
             K=K, d=d,
             img_size=np.array([w, h]),
             square_size=SQUARE_SIZE_MM,
             board=np.array([BOARD_COLS, BOARD_ROWS]),
             rms=ret, per_rms=per,
             calib_time=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print(f"\n已保存: {out_file}（含 rms / per_rms）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help="内参输出文件；水下标定用 camera_params_underwater.npz")
    args = ap.parse_args()
    if not os.path.isabs(args.out):
        args.out = os.path.join(SCRIPT_DIR, args.out)

    records = load_existing()
    if records:
        print(f"[加载] 已从 calib_images 载入 {len(records)} 张历史图")

    cam = HikCamera(0)
    try:
        cam.open()
        cam.start_grabbing()
    except Exception as e:
        print(f"[错误] {e}")
        sys.exit(1)

    print("\n=== 标定模式 v2 ===")
    print("[空格] 采集 | [b] 删除最后一张 | [c] 标定并保存 | [x] 剔除高误差图并重标 | [q] 退出")

    K0 = None
    last_res = None
    while True:
        frame = cam.get_frame()
        if frame is None:
            continue
        h, w = frame.shape
        if K0 is None:
            K0 = rough_K(w, h)

        found, corners = find_board(frame)
        vis = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

        # 九宫格参考线
        for i in (1, 2):
            cv.line(vis, (w * i // 3, 0), (w * i // 3, h), (80, 80, 80), 1)
            cv.line(vis, (0, h * i // 3), (w, h * i // 3), (80, 80, 80), 1)

        n_active = len([r for r in records if not r["excluded"]])
        if found:
            cv.drawChessboardCorners(vis, (BOARD_COLS, BOARD_ROWS), corners, found)
            ret, rvec, tvec = cv.solvePnP(objp, corners, K0, None)
            if ret:
                z_m = float(tvec[2, 0]) / 1000.0
                tilt = rvec_to_tilt(rvec)
                band = band_of(z_m)
                color = (0, 200, 0) if band != "out-of-range" else (0, 0, 255)
                cv.putText(vis, f"{z_m:.2f}m {band} tilt:{tilt:.0f}d",
                           (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv.putText(vis, f"saved: {n_active}", (w - 190, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        y = 62
        for name, lo, hi, target in BANDS:
            cnt = sum(1 for r in records if not r["excluded"] and r["band"] == name)
            c = (0, 200, 0) if cnt >= target else (0, 180, 255)
            cv.putText(vis, f"{name}: {cnt}/{target}", (10, y),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
            y += 28

        cv.imshow("calibrate", vis)
        key = cv.waitKey(1) & 0xFF

        if key == ord(' '):
            if found:
                ret, rvec, tvec = cv.solvePnP(objp, corners, K0, None)
                z_m = float(tvec[2, 0]) / 1000.0 if ret else -1.0
                band = band_of(z_m) if ret else "?"
                n = len(records) + 1
                fname = os.path.join(IMG_DIR, f"img_{n:02d}.png")
                imwrite_unicode(fname, frame)
                records.append({"fname": fname, "corners": corners, "band": band, "excluded": False})
                print(f"  采集 {n}: {z_m:.2f}m  {band}")
            else:
                print("  [警告] 未检测到棋盘格，调整位置/角度/光照")
        elif key == ord('b'):
            if records:
                r = records.pop()
                try:
                    os.remove(r["fname"])
                except OSError:
                    pass
                print(f"  删除最后一张: {os.path.basename(r['fname'])}")
        elif key == ord('c'):
            res = run_calibration(records, (w, h))
            if res:
                last_res = res
                print_report(res, w, h, args.out)
        elif key == ord('x'):
            if last_res is None:
                print("  请先按 [c] 标定")
                continue
            med = float(np.median(last_res["per"]))
            bad = [r for r, e in zip(last_res["active"], last_res["per"]) if e > 2 * med]
            if not bad:
                print("  没有需要剔除的图（所有图均 <=2×中位数）")
                continue
            names = [os.path.basename(r["fname"]) for r in bad]
            print(f"  将剔除 {len(bad)} 张: {names}")
            ans = input("  确认删除文件并重标? (y/N): ").strip().lower()
            if ans == "y":
                for r in bad:
                    try:
                        os.remove(r["fname"])
                    except OSError:
                        pass
                    if r in records:
                        records.remove(r)
                res = run_calibration(records, (w, h))
                if res:
                    last_res = res
                    print_report(res, w, h, args.out)
        elif key == ord('q'):
            break

    cam.close()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
