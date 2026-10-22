# -*- coding: utf-8 -*-
"""
海康工业相机简化封装 - 直接输出 OpenCV 可用的 numpy 数组
适配 MV-CA013-20GM (Mono8, 1280x1024)
"""
import numpy as np
import sys, os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "MvImport"))
from MvCameraControl_class import *


class HikCamera:
    def __init__(self, dev_index=0):
        self.dev_index = dev_index
        self.cam = MvCamera()
        self.device_list = MV_CC_DEVICE_INFO_LIST()
        self.is_opened = False

    def enumerate(self):
        """枚举设备，返回数量"""
        MvCamera.MV_CC_Initialize()
        layer = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(layer, self.device_list)
        if ret != 0:
            raise RuntimeError(f"枚举设备失败 ret=0x{ret:x}")
        n = self.device_list.nDeviceNum
        print(f"[相机] 发现 {n} 台设备")
        return n

    def open(self):
        """打开第 dev_index 台相机"""
        if self.device_list.nDeviceNum == 0:
            self.enumerate()
        if self.dev_index >= self.device_list.nDeviceNum:
            raise RuntimeError("设备索引超出范围")

        # 取设备信息指针
        dev_info = cast(self.device_list.pDeviceInfo[self.dev_index],
                        POINTER(MV_CC_DEVICE_INFO)).contents
        ret = self.cam.MV_CC_CreateHandle(dev_info)
        if ret != 0:
            raise RuntimeError(f"CreateHandle 失败 ret=0x{ret:x}")

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise RuntimeError(f"OpenDevice 失败 ret=0x{ret:x}")

        # 设置为 Mono8（如果相机默认不是）
        self.cam.MV_CC_SetEnumValueByString("PixelFormat", "Mono8")

        self.is_opened = True
        print("[相机] 打开成功")

    def start_grabbing(self):
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError(f"StartGrabbing 失败 ret=0x{ret:x}")
        print("[相机] 开始取流")

    def get_frame(self, timeout_ms=1000):
        """取一帧，返回 grayscale numpy array (H, W)，失败返回 None"""
        out_frame = MV_FRAME_OUT()
        ctypes.memset(byref(out_frame), 0, sizeof(out_frame))
        ret = self.cam.MV_CC_GetImageBuffer(out_frame, timeout_ms)
        if ret != 0:
            return None

        w = out_frame.stFrameInfo.nWidth
        h = out_frame.stFrameInfo.nHeight
        # 把缓冲区拷贝成 numpy array (Mono8 灰度图)
        buf = (c_ubyte * (w * h))()
        ctypes.memmove(buf, out_frame.pBufAddr, w * h)
        img = np.frombuffer(buf, dtype=np.uint8).reshape(h, w).copy()

        self.cam.MV_CC_FreeImageBuffer(out_frame)
        return img

    def stop_grabbing(self):
        if self.is_opened:
            self.cam.MV_CC_StopGrabbing()

    def close(self):
        if self.is_opened:
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.is_opened = False
            print("[相机] 已关闭")

    def __enter__(self):
        self.open()
        self.start_grabbing()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_grabbing()
        self.close()
