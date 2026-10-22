# -*- coding: utf-8 -*-
"""
ArUco 实时位姿跟踪 v2（新增统计模式）
用法:
  python aruco_tracker.py
  python aruco_tracker.py --params camera_params_underwater.npz   # 水下标定后
  python aruco_tracker.py --stat-sec 30
操作:
  [d] 进入统计模式（默认采集 30s，输出检测率 + Z std 报告并保存到 stats/）
  [q] 退出；统计中按 q 提前结束并出报告
前置: 先运行 calibrate.py 生成 camera_params*.npz
"""
import argparse
import os
import sys
import time
import datetime

import cv2 as cv
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from hik_camera import HikCamera

# === 参数 ===
DICT = cv.aruco.DICT_6X6_250
MARKER_SIZE_MM = 50.0   # 打印的 ArUco 码实际边长（毫米）
# ============

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATS_DIR = os.path.join(SCRIPT_DIR, "stats")

dictionary = cv.aruco.getPredefinedDictionary(DICT)
detector = cv.aruco.ArucoDetector(dictionary, cv.aruco.DetectorParameters())


def rotation_vector_to_euler(rvec):
    """旋转向量 → 欧拉角 (度)"""
    R, _ = cv.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy >= 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])


def draw_axis(img, corner, img_pts):
    """画 XYZ 坐标轴: X红 Y绿 Z蓝"""
    origin = tuple(corner[0].astype(int))
    img = cv.line(img, origin, tuple(img_pts[0][0].astype(int)), (0, 0, 255), 2)
    img = cv.line(img, origin, tuple(img_pts[1][0].astype(int)), (0, 255, 0), 2)
    img = cv.line(img, origin, tuple(img_pts[2][0].astype(int)), (255, 0, 0), 2)
    return img


def solve_markers(gray, corners, ids, K, d, marker_size):
    """对检测到的每个 marker 求位姿，返回 [(id, tvec, euler, dist), ...]"""
    half = marker_size / 2.0
    obj_pts = np.float32([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0]
    ])
    results = []
    for i, mid in enumerate(ids.flatten()):
        ret, rvec, tvec = cv.solvePnP(obj_pts, corners[i], K, d,
                                      flags=cv.SOLVEPNP_IPPE_SQUARE)
        if not ret:
            continue
        tvec = tvec.ravel()
        euler = rotation_vector_to_euler(rvec)
        results.append((int(mid), tvec, euler, float(np.linalg.norm(tvec)), rvec))
    return results


def draw_overlay(vis, corners, ids, results, marker_size, K, d):
    """画面上画 marker 框、坐标轴、ID、距离"""
    cv.aruco.drawDetectedMarkers(vis, corners, ids)
    axis_pts = np.float32([[0, 0, 0],
                           [marker_size, 0, 0],
                           [0, marker_size, 0],
                           [0, 0, marker_size]])
    for (mid, tvec, euler, dist, rvec) in results:
        idx = list(ids.flatten()).index(mid)
        img_pts, _ = cv.projectPoints(axis_pts, rvec, tvec, K, d)
        vis = draw_axis(vis, corners[idx][0], img_pts)
        tp = tuple(corners[idx][0][0].astype(int))
        cv.putText(vis, f"ID:{mid}", (tp[0], tp[1] - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv.putText(vis, f"{dist:.0f}mm", (tp[0], tp[1] - 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def collect_stats(cam, K, d, marker_size, duration, params_file):
    """统计模式：采 duration 秒，记录每帧检测结果，结束后出报告"""
    samples = []
    t0 = time.time()
    last_print = 0.0
    print(f"\n[统计] 开始采集 {duration:.0f}s，保持 marker 静止……")

    while True:
        elapsed = time.time() - t0
        frame = cam.get_frame()
        if frame is None:
            continue
        corners, ids, _rejected = detector.detectMarkers(frame)
        rec = {"det": ids is not None and len(ids) > 0, "ids": {}}
        if rec["det"]:
            for (mid, tvec, euler, dist, rvec) in solve_markers(frame, corners, ids, K, d, marker_size):
                rec["ids"][mid] = (float(tvec[0]), float(tvec[1]), float(tvec[2]), dist)
        samples.append(rec)

        vis = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
        if rec["det"]:
            results = solve_markers(frame, corners, ids, K, d, marker_size)
            draw_overlay(vis, corners, ids, results, marker_size, K, d)
        # 进度条
        h, w = frame.shape
        p = min(1.0, elapsed / duration)
        cv.rectangle(vis, (60, h - 50), (w - 60, h - 20), (60, 60, 60), -1)
        cv.rectangle(vis, (60, h - 50), (60 + int((w - 120) * p), h - 20),
                     (0, 200, 0), -1)
        cv.putText(vis, f"STATS {elapsed:5.1f}/{duration:.0f}s  frames={len(samples)} "
                        f"det={sum(1 for s in samples if s['det'])}",
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.imshow("ArUco Tracker", vis)

        if elapsed - last_print > 0.5:
            print(f"  [{elapsed:5.1f}s] 已采 {len(samples)} 帧")
            last_print = elapsed

        key = cv.waitKey(1) & 0xFF
        if key == ord('q') and elapsed > 2.0:
            print("[统计] 提前结束")
            break
        if elapsed >= duration:
            break

    print_report(samples, duration, marker_size, params_file)


def print_report(samples, duration, marker_size, params_file):
    n = len(samples)
    if n == 0:
        print("[统计] 没有采集到帧")
        return
    det_n = sum(1 for s in samples if s["det"])
    rate = det_n / n * 100.0

    all_ids = set()
    for s in samples:
        all_ids.update(s["ids"].keys())

    lines = []
    lines.append("===== ArUco 统计验收报告 =====")
    lines.append(f"时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"内参文件: {os.path.basename(params_file)}")
    lines.append(f"marker 边长: {marker_size}mm | 设定时长: {duration:.0f}s | 实际帧数: {n}")
    lines.append(f"检测率(任意 marker): {det_n}/{n} = {rate:.1f}%")
    lines.append("")
    lines.append(" ID | 出现帧数 | 出现率% | Z均值mm | Z std | Z min~max | X std | Y std")
    lines.append("-" * 82)

    print("\n" + "\n".join(lines))
    rows = []
    for mid in sorted(all_ids):
        zs = [s["ids"][mid][2] for s in samples if mid in s["ids"]]
        xs = [s["ids"][mid][0] for s in samples if mid in s["ids"]]
        ys = [s["ids"][mid][1] for s in samples if mid in s["ids"]]
        nid = len(zs)
        row = (mid, nid, nid / n * 100.0,
               float(np.mean(zs)), float(np.std(zs)),
               float(np.min(zs)), float(np.max(zs)),
               float(np.std(xs)), float(np.std(ys)))
        rows.append(row)
        line = (f"{mid:3d} | {nid:6d} | {row[2]:6.1f} | {row[3]:7.1f} | {row[4]:5.2f} | "
                f"{row[5]:6.1f}~{row[6]:-6.1f} | {row[7]:5.2f} | {row[8]:5.2f}")
        print(line)

    # 达标判定：以出现帧最多的主 ID 为准，按 Z 均值自动套近/远档标准
    verdict = ""
    if rows:
        main = max(rows, key=lambda r: r[1])
        z_mean, z_std = main[3], main[4]
        if z_mean < 700:
            std_ok, std_thr, rate_thr = z_std < 2.0, 2.0, 99.0
            band = "近档(0.3/0.5m)"
        else:
            std_ok, std_thr, rate_thr = z_std < 10.0, 10.0, 95.0
            band = "远档(~1m)"
        rate_ok = rate >= rate_thr
        verdict = (f"\n主ID={main[0]}（{band}，Z均值={z_mean:.1f}mm）: "
                   f"Z std={z_std:.2f}mm {'PASS' if std_ok else 'FAIL'} (<{std_thr}mm) | "
                   f"检测率={rate:.1f}% {'PASS' if rate_ok else 'FAIL'} (>{rate_thr:.0f}%)")
        overall = "PASS" if std_ok and rate_ok else "FAIL"
        verdict = f"[{overall}] " + verdict
        print(verdict)

    os.makedirs(STATS_DIR, exist_ok=True)
    out = os.path.join(STATS_DIR,
                       f"stats_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
        for row in rows:
            f.write((f"{row[0]:3d} | {row[1]:6d} | {row[2]:6.1f} | {row[3]:7.1f} | {row[4]:5.2f} | "
                     f"{row[5]:6.1f}~{row[6]:-6.1f} | {row[7]:5.2f} | {row[8]:5.2f}\n"))
        if verdict:
            f.write("\n" + verdict + "\n")
    print(f"\n报告已保存: {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=None, help="内参 npz 文件路径/文件名")
    ap.add_argument("--name", default=None,
                    help="内参名称后缀，如 --name underwater → camera_params_underwater.npz")
    ap.add_argument("--stat-sec", type=float, default=30.0, help="统计模式采集时长（秒）")
    ap.add_argument("--marker-size", type=float, default=MARKER_SIZE_MM, help="marker 边长（毫米）")
    args = ap.parse_args()

    if args.params:
        params_file = args.params
        if not os.path.isabs(params_file):
            params_file = os.path.join(SCRIPT_DIR, params_file)
    elif args.name:
        params_file = os.path.join(SCRIPT_DIR, f"camera_params_{args.name}.npz")
    else:
        params_file = os.path.join(SCRIPT_DIR, "camera_params.npz")

    if not os.path.exists(params_file):
        print(f"[错误] 找不到 {params_file}")
        print("请先运行: python calibrate.py")
        sys.exit(1)

    data = np.load(params_file)
    K, d = data["K"], data["d"]
    data.close()
    print(f"[加载] {os.path.basename(params_file)}")
    print(f"[加载] K=\n{K}")
    print(f"[加载] d={d.ravel()}")

    cam = HikCamera(0)
    cam.open()
    cam.start_grabbing()

    print("\n=== ArUco 跟踪中 ===")
    print("画面: X=红 Y=绿 Z=蓝 | [d] 统计 30s | [q] 退出\n")

    t_prev = time.time()
    fps = 0.0

    while True:
        frame = cam.get_frame()
        if frame is None:
            continue

        corners, ids, _rejected = detector.detectMarkers(frame)
        vis = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

        if ids is not None and len(ids) > 0:
            results = solve_markers(frame, corners, ids, K, d, args.marker_size)
            draw_overlay(vis, corners, ids, results, args.marker_size, K, d)
            for (mid, tvec, euler, dist, rvec) in results:
                print(f"ID={mid:3d} | "
                      f"X={tvec[0]:+7.1f}mm Y={tvec[1]:+7.1f}mm Z={tvec[2]:+7.1f}mm "
                      f"| 距离={dist:6.1f}mm "
                      f"| 俯仰={euler[0]:+6.1f}° 横滚={euler[1]:+6.1f}° 偏航={euler[2]:+6.1f}°")

        t_now = time.time()
        dt = t_now - t_prev
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        t_prev = t_now
        cv.putText(vis, f"FPS: {fps:.1f}", (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.putText(vis, "[d] stats", (10, 70),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv.imshow("ArUco Tracker", vis)
        key = cv.waitKey(1) & 0xFF
        if key == ord('d'):
            collect_stats(cam, K, d, args.marker_size, args.stat_sec, params_file)
            t_prev = time.time()
        elif key == ord('q'):
            break

    cam.close()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
