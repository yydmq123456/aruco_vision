# 水下机器人 ArUco 视觉定位 — 空气阶段全流程上手指南

基于海康 MV-CA013-20GM 工业相机（Mono8, 1280×1024）+ OpenCV 的 ArUco 码实时位姿估计。
本 README 面向第一次接手的人：从装环境开始，按下面 8 步顺序做完，即可完整跑通
**打印 ArUco 码 → 相机标定 → 实时定位 → 精度验收** 的空气阶段全流程。

> 项目背景与决策记录（为什么这么定）见 `CLAUDE.md`；本文件只讲"怎么操作"。

## 全流程总览

| 步骤 | 做什么 | 用什么 | 产物 / 验收 |
|---|---|---|---|
| 0 | 装环境、连相机 | MVS 客户端 + Python | 相机能出图 |
| 1 | 打印 ArUco 码 | `aruco_all_print.png` | 黑框实测 50mm |
| 2 | 打印棋盘格标定板 | `checkerboard_9x6_20mm.png` | 方格实测 20mm |
| 3 | 对焦并锁死 | `focus_assist.py` | 锁焦 0.5m、光圈 f/8 |
| 4 | 相机标定 | `calibrate.py` | `camera_params.npz`，RMS<0.5px |
| 5 | 实时跟踪 | `aruco_tracker.py` | 画面出 ID/XYZ/姿态 |
| 6 | 统计验收 | `aruco_tracker.py` 按 [d] | `stats/` 报告 PASS |
| 7 | 卷尺实测验证 | 卷尺 | 0.3/0.5m 误差 <1% |

⚠️ **顺序不能乱：必须先锁焦（第 3 步）再标定（第 4 步）**。
锁焦之后标定、跟踪、验收全程不许再碰对焦环和光圈环，否则标定参数作废。

## 目录结构

```
vision/
├── hik_camera.py           # 海康相机封装（SDK 取流 → numpy 灰度图）
├── generate_marker.py      # 生成 ArUco 单码 PNG（id0~9，50mm，300DPI）
├── make_print_sheet.py     # 把所有码排版成一张 A4 合集
├── make_checkerboard.py    # 生成 9×6 内角点 / 20mm 方格棋盘格标定板
├── calibrate.py            # 相机标定 v2（距离档位提示/单图误差排行/九宫格覆盖）
├── aruco_tracker.py        # ArUco 实时位姿跟踪 v2（含统计验收模式）
├── focus_assist.py         # 手动对焦辅助（清晰度峰值提示）
├── MvImport/               # 海康官方 Python SDK 接口封装
├── markers/                # 生成的 ArUco 单码 PNG
├── calib_images/           # 标定采集的棋盘格照片（img_01.png ...）
├── stats/                  # 统计验收报告（自动生成）
├── backup/                 # 旧文件备份
├── aruco_all_print.png     # 排版好的 10 码 A4 合集（可直接打印）
├── checkerboard_9x6_20mm.png  # 棋盘格标定板（可直接打印）
└── camera_params.npz       # 标定结果（K/畸变/rms，自动生成）
```

---

## 第 0 步：环境准备

### 0.1 硬件清单
- 海康 MV-CA013-20GM（GigE 网口相机）+ 镜头 + 网线 + 电源
- 电脑（带网口，装海康 MVS 客户端）
- A4 打印机、尺子、硬纸板（标定板和码都要贴硬板上防卷曲）

### 0.2 软件安装
1. **海康 MVS 客户端**：从海康官网或随机资料安装（相机驱动 + Python SDK 运行库都在里面）。仓库里的 `MvImport/` 是官方 Python 接口封装，装好 MVS 后直接可用。
2. **Python 环境**：项目固定使用 conda 环境 `mengq_env`。所有命令都在该环境下、`vision/` 目录里执行：

```powershell
conda activate mengq_env
cd D:\THU\underwater_robot\vision

# 装包必须用完整路径的写法（保持环境纯净）：
& "C:\Users\37334\miniforge3\envs\mengq_env\python.exe" -m pip install numpy "opencv-python>=4.7"
```

> `opencv-python` 需要 ≥4.7：代码用了新版 `ArucoDetector` 接口，并已适配 OpenCV 5
> （`estimatePoseSingleMarkers` 已删除，本项目用 `solvePnP(IPPE_SQUARE)` 实现）。

### 0.3 连接相机
1. 网线连接相机与电脑，相机上电
2. 打开 MVS 客户端 → 应能枚举到相机并正常出图；GigE 相机枚举不到时，把电脑网卡 IP 改成与相机同网段（MVS 里可一键配置）
3. **关闭 MVS**——MVS 与脚本不能同时占用相机（互斥模式）
4. 跑任意脚本看到 `[相机] 打开成功 → 开始取流` 即 OK

---

## 第 1 步：打印 ArUco 码

仓库里已有生成好的文件，**直接打印即可**：
- 单码：`markers/aruco_id0_50mm.png` ~ `aruco_id9_50mm.png`
- 合集（推荐）：`aruco_all_print.png` —— 一张 A4 排 10 个码

如需重新生成：
```powershell
python generate_marker.py     # 生成 markers/ 下 id0~9，50mm，300DPI
python make_print_sheet.py    # 排版成 A4 合集 aruco_all_print.png
```

打印与制作要求：
1. 打印缩放必须选【实际大小 / 100%】，**绝不能选【适应页面】**
2. 打完用尺子量**黑框**边长，必须正好 50mm（黑框外留的白边是裁剪余量）
3. 不准 → 按比例（实际尺寸/50mm）缩放图片重打
4. 码贴在硬板上保持平整，用哑光纸避免反光

**验收：黑框 50mm ±0.5mm。** 码的尺寸误差是线性系统误差，1% 的尺寸误差直接变成 1% 的测距误差。

---

## 第 2 步：打印棋盘格标定板

仓库已有现成的 `checkerboard_9x6_20mm.png`，直接 A4 打印（选【实际大小 / 100%】）。

如需重新生成：
```powershell
python make_checkerboard.py
```

规格与 `calibrate.py` 严格对应：**内角点 9×6，方格边长 20mm**
（⚠️ 注意不是网上老教程的 25mm，本项目标定板是自生成的 20mm 版）。

- 打印后量一个方格的实际边长，必须正好 20mm，不对就重打
- 贴到硬纸板/亚克力上，必须平整，任何弯曲都会毁掉标定精度

---

## 第 3 步：对焦并锁死（标定前必做）

既定方案（理由见 CLAUDE.md）：**锁焦 0.5m（精度区中心），光圈 f/8**。
0~0.5m 是精度要求最高的区间，f/8 的景深能兼顾 0.3~0.7m 清晰。

操作：
1. 把 ArUco 码用卷尺固定在距镜头 **0.5m** 处
2. 光圈环调到 **f/8**（看镜头环刻度）
3. 运行对焦辅助：
   ```powershell
   python focus_assist.py
   ```
4. 缓慢拧对焦环，直到画面出现绿色【对焦最佳】、清晰度数值达到峰值
   （脚本用拉普拉斯方差判断清晰度，不依赖标定精度；`camera_params.npz` 用现有的即可）
5. 退出脚本，**用顶丝/胶带把对焦环、光圈环彻底锁死**——此后直到换场景前都不许再动

**验收：锁焦完成，环已物理固定。** 之后若碰撞/移动过相机或镜头，回到本步重做。

---

## 第 4 步：相机标定

### 采集前准备
- **清空 `calib_images/` 里的旧图**（挪到 `backup/`）：脚本启动时会自动加载历史图，新旧混采会毁掉标定
- 标定板不能只拍画面中央——畸变参数靠画面**边缘/四角**的图约束

### 采集与标定
```powershell
python calibrate.py
```
画面会实时显示角点、估算距离、档位和倾斜角。按三档距离采集，共约 24 张：

| 档位 | 距离区间 | 目标张数 | 侧重 |
|---|---|---|---|
| near 0.35m | 0 ~ 0.43m | 8 | 板占画面约 3/4（定 fx） |
| mid 0.50m | 0.43 ~ 0.60m | 10 | 覆盖四角/边缘（定畸变） |
| far 0.70m | 0.60 ~ 0.95m | 6 | 含倾斜 ±15~30°、随机旋转 90°/180° |

按键：`[空格]` 采集 · `[b]` 删最后一张 · `[c]` 标定并保存 · `[x]` 剔除高误差图并重标 · `[q]` 退出
（支持分多次采集：重启脚本会自动载入 `calib_images/` 已有图片）

### 验收（看 `[c]` 输出的标定报告）
- 整体重投影 RMS **< 0.5px**（优秀 < 0.3px）
- fx/fy 差 **< 1%**
- 单图误差 > 2× 中位数的图：按 `[x]` → 确认 `y` → 自动删图重标
- 九宫格覆盖：四个角格每格 ≥ 3 张
- 全部通过后自动保存 `camera_params.npz`（含 rms 与每图误差，可复查）

水下标定时用 `python calibrate.py --out camera_params_underwater.npz`，不会覆盖空气参数。

---

## 第 5 步：实时跟踪

```powershell
python aruco_tracker.py
```

- ArUco 码放相机前：画面画四角框 + XYZ 坐标轴（**X 红 Y 绿 Z 蓝**），码上方显示 ID 和距离
- 终端逐帧打印：`ID | X/Y/Z（mm）| 总距离 | 俯仰/横滚/偏航角`
- `[q]` 退出

常用变体：
```powershell
python aruco_tracker.py --name underwater          # 读 camera_params_underwater.npz（水下阶段）
python aruco_tracker.py --marker-size 100          # 换用 100mm 大码时
```

---

## 第 6 步：统计验收（检测率 + 稳定性）

把码固定不动，进入统计模式（默认采 30s）：
```powershell
python aruco_tracker.py                 # 运行中按 [d]
python aruco_tracker.py --stat-sec 60   # 或直接指定采 60s
```

采完后输出报告：检测率、每个 ID 的出现率 / Z 均值 / Z std / Z 范围 / X·Y std，
并自动给出 **PASS / FAIL** 判定；报告落盘 `stats/stats_年月日_时分秒.txt`（统计中按 `[q]` 可提前结束）。

达标线（按需求分级）：

| 距离 | 检测率 | Z std |
|---|---|---|
| 0.3 / 0.5m（精度区） | ≈100%（脚本判 >99%） | < 2mm |
| 1.0m（粗定位） | > 95% | < 10mm |

判定逻辑：按主 ID 的 Z 均值自动套档——<0.7m 用近档标准，否则远档。

**1m 不达标怎么办：优先换更大的码（80~100mm，已建议加订），不要动焦距。**

---

## 第 7 步：卷尺实测验证

- 码分别固定在 **0.3m / 0.5m**，对照 tracker 的 Z 读数与卷尺实测
- 误差 **< 1%**（0.5m 处即 < 5mm）为合格

关于读数的物理含义：程序输出的 Z **原点在镜头内部的光心**（透镜组中间，前端玻璃后约 15~25mm），不是壳面也不是 CMOS 表面：
- 壳面到目标的实际距离 = 程序 Z 值 − 光心到壳面的固定偏移
- 本步用卷尺多点对比，平均求出该偏移量，之后所有读数减去它
- 装进防水壳后此偏移会变，需重测

---

## 关键参数速查

| 参数 | 当前值 | 修改位置 |
|---|---|---|
| ArUco 字典 | DICT_6X6_250 | `generate_marker.py` / `aruco_tracker.py` 顶部 |
| ArUco 码边长 | 50mm（打印后必须实测） | `aruco_tracker.py` 的 `MARKER_SIZE_MM` |
| 标定板 | 内角点 9×6，方格 20mm | `calibrate.py` 顶部 / `make_checkerboard.py` |
| 对焦 / 光圈 | 锁焦 0.5m、f/8（锁定后禁动） | 镜头环（物理操作） |
| 重投影验收 | < 0.5px（优秀 < 0.3px） | `calibrate.py` 输出 |
| 跟踪验收 | 见第 6 步表格 | `aruco_tracker.py` 统计模式 |

## 常见问题

- **找不到相机 / OpenDevice 失败**：先在 MVS 里确认能看到相机；跑脚本前必须**关掉 MVS**（互斥占用）。GigE 枚举不到 → 网卡 IP 与相机设为同网段
- **取流失败**：相机被其他程序占用（包括另开的一个脚本实例）
- **画面太黑**：在 MVS 里调曝光/增益（这俩不影响标定几何，可放心调）；**对焦/光圈环不能动**
- **棋盘格检测不到**：光照均匀、板平整、倾斜别超过 ±30°、距离别近到出画
- **ArUco 检测不到**：光照均匀、码平贴；1m 处 50mm 码已接近极限 → 换 80~100mm 大码
- **位姿跳变 / 抖动大**：标定质量不够 → 重标、多采边缘图；或打印尺寸没量准
- **OpenCV 5 报 `estimatePoseSingleMarkers` 不存在**：新版本已删除该函数，本项目已改用 `solvePnP(IPPE_SQUARE)`，无需改动
- **中文路径读写问题**：代码已统一用 `imencode/imdecode` 处理，一般无需关心

---

## 水下阶段预告（空气全流程跑通后再做）

1. 相机装进防水壳，对准平面玻璃窗；码做过防水处理（过塑/贴防水膜）
2. 壳入水后，对着水中 0.5m 处的实际码**重新对焦锁死**（水下与空气的最佳对焦位置不同）
3. 水下重新标定：`python calibrate.py --out camera_params_underwater.npz`
4. 水下跟踪：`python aruco_tracker.py --name underwater`

原理性提醒：平面防水玻璃存在折射 → 水下 0.5m 的物体在光学上 ≈ 0.38m，
等效焦距 ×1.33、畸变特性改变，**空气标定参数下水全部作废**；水下只能定焦，一切以水下实际工况为准。
