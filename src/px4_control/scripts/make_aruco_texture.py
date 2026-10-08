#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""生成 ArUco 降落标识纹理（确定性尺寸，供 aruco_pad 模型使用）。

几何关系（必须与视觉节点里的 marker_length 保持一致）：
    纹理画布        1200 x 1200 px
    标识(含黑框)     800 x  800 px，居中
    白边(quiet zone) 200 px/侧 == 标识单元的 1.5 倍，满足 ArUco 静区要求
    面板实际尺寸     0.6 x 0.6 m
    => marker_length = 0.6 * 800 / 1200 = 0.4 m

用法：
    /usr/bin/python3 make_aruco_texture.py <输出目录>
    # 输出目录应为 aruco_pad/materials/textures
"""
import os
import sys

import cv2
import numpy as np

CANVAS = 1200          # 画布边长 px
MARKER = 800           # 标识（含黑色外框）边长 px
DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
PAD_SIZE_M = 0.6       # 面板实际边长 m


def main(out_dir):
    dictionary = cv2.aruco.Dictionary_get(DICT)
    marker = cv2.aruco.drawMarker(dictionary, MARKER_ID, MARKER)

    # drawMarker 的黑色外框必须铺满图像边缘，否则"图像边长 == 标识边长"不再成立
    edges = (marker[0, :], marker[-1, :], marker[:, 0], marker[:, -1])
    if not all((e == 0).all() for e in edges):
        sys.exit('ERROR: drawMarker 外框不是纯黑，尺寸关系失效，请检查 OpenCV 版本')

    canvas = np.full((CANVAS, CANVAS), 255, dtype=np.uint8)
    off = (CANVAS - MARKER) // 2
    canvas[off:off + MARKER, off:off + MARKER] = marker

    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(out_dir, 'aruco_marker_0.png'), canvas)
    # 备用纹理：若 Gazebo 立方体顶面把纹理渲染成镜像导致检不出，改用这一张
    cv2.imwrite(os.path.join(out_dir, 'aruco_marker_0_mirrored.png'),
                cv2.flip(canvas, 1))

    marker_len = PAD_SIZE_M * MARKER / CANVAS
    print('OK  canvas=%dx%d px  marker=%d px  pad=%.3f m  marker_length=%.3f m'
          % (CANVAS, CANVAS, MARKER, PAD_SIZE_M, marker_len))
    print('    dictionary=DICT_4X4_50  id=%d' % MARKER_ID)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
