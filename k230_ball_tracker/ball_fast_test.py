"""K230钢球快速测试，不使用YOLO。"""

from libs.PipeLine import PipeLine
from libs.Utils import *

import gc
import os
import sys
import time
import ulab.numpy as np


# 摄像头和屏幕路径与可用版本一致
RGB888P_SIZE = [640, 320]
DISPLAY_MODE = "lcd"
PIPE_ROI = (40, 90, 560, 150)

# 深色模式下管道比钢球亮
# 反光明显时改为假
USE_DARK_TARGET = True
DARK_MAX_RGB = 125
BRIGHT_MIN_RGB = 165

# 每列匹配像素数和水平平滑
MIN_COLUMN_PIXELS = 4
MIN_PEAK_OVER_AVERAGE = 3.0
SMOOTH_RADIUS = 4
GC_EVERY_FRAMES = 60


def get_rgb_planes(frame):
    """返回常见图像布局的RGB视图。"""
    shape = frame.shape
    if len(shape) == 4 and shape[0] == 1 and shape[1] >= 3:
        return frame[0, 0, :, :], frame[0, 1, :, :], frame[0, 2, :, :]
    if len(shape) == 3 and shape[0] >= 3 and shape[0] <= 4:
        return frame[0, :, :], frame[1, :, :], frame[2, :, :]
    if len(shape) == 3 and shape[2] >= 3 and shape[2] <= 4:
        return frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
    raise ValueError("unsupported frame shape %s" % (str(shape)))


def locate_ball_x(frame):
    """用列直方图定位深色或亮色目标。"""
    red, green, blue = get_rgb_planes(frame)
    roi_x, roi_y, roi_w, roi_h = PIPE_ROI
    red = red[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    green = green[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    blue = blue[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

    # 旧版ulab不支持astype
    # 只用uint8比较，避免类型转换和减法
    if USE_DARK_TARGET:
        mask = ((red <= DARK_MAX_RGB) &
                (green <= DARK_MAX_RGB) &
                (blue <= DARK_MAX_RGB))
    else:
        mask = ((red >= BRIGHT_MIN_RGB) &
                (green >= BRIGHT_MIN_RGB) &
                (blue >= BRIGHT_MIN_RGB))

    # 旧版CanMV要求axis使用位置参数
    columns = np.sum(mask, 0)
    width = len(columns)
    total = 0.0
    for index in range(width):
        total += float(columns[index])
    average = total / float(width) if width > 0 else 0.0

    best_x = -1
    best_score = -1.0
    for center in range(width):
        start = center - SMOOTH_RADIUS
        end = center + SMOOTH_RADIUS + 1
        if start < 0:
            start = 0
        if end > width:
            end = width
        score = 0.0
        for index in range(start, end):
            score += float(columns[index])
        if score > best_score:
            best_score = score
            best_x = center

    required = float(MIN_COLUMN_PIXELS * (SMOOTH_RADIUS * 2 + 1))
    prominence = best_score - average * float(SMOOTH_RADIUS * 2 + 1)
    valid = best_score >= required and prominence >= MIN_PEAK_OVER_AVERAGE
    return (roi_x + best_x if valid else -1), best_score, average


def main():
    pipeline = None
    try:
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create()
        display_size = pipeline.get_display_size()
        scale_x = float(display_size[0]) / float(RGB888P_SIZE[0])
        scale_y = float(display_size[1]) / float(RGB888P_SIZE[1])
        clock = time.clock()
        frame_count = 0
        last_log_ms = time.ticks_ms()
        shape_printed = False

        print("READY,PIPELINE_NDARRAY_BALL_TEST")
        print("MODE,%s" % ("DARK" if USE_DARK_TARGET else "BRIGHT"))

        while True:
            os.exitpoint()
            clock.tick()
            frame_count += 1
            frame = pipeline.get_frame()
            if not shape_printed:
                print("FRAME_SHAPE,%s" % str(frame.shape))
                shape_printed = True

            ball_x, peak, average = locate_ball_x(frame)
            pipeline.osd_img.clear()
            roi_x, roi_y, roi_w, roi_h = PIPE_ROI
            display_x = int(roi_x * scale_x)
            display_y = int(roi_y * scale_y)
            display_w = int(roi_w * scale_x)
            display_h = int(roi_h * scale_y)
            pipeline.osd_img.draw_rectangle(
                display_x, display_y, display_w, display_h,
                color=(255, 0, 255, 0), thickness=2)

            if ball_x >= 0:
                center_y = roi_y + roi_h // 2
                pipeline.osd_img.draw_rectangle(
                    int((ball_x - 12) * scale_x),
                    int((center_y - 18) * scale_y),
                    int(24 * scale_x), int(36 * scale_y),
                    color=(255, 255, 0, 0), thickness=2)
                pipeline.osd_img.draw_cross(
                    int(ball_x * scale_x), int(center_y * scale_y),
                    color=(255, 255, 255, 0), size=12, thickness=2)
                state = "BALL X:%d" % ball_x
            else:
                state = "BALL LOST"

            fps = clock.fps()
            pipeline.osd_img.draw_string(
                display_x + 4, display_y + 4,
                "FPS:%.1f %s" % (fps, state),
                color=(255, 255, 255, 0), scale=2)
            pipeline.show_image()

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_log_ms) >= 1000:
                last_log_ms = now_ms
                print("FPS,%.1f,X,%d,PEAK,%.1f,AVG,%.1f" % (
                    fps, ball_x, peak, average))
            del frame
            if frame_count % GC_EVERY_FRAMES == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("STOP,keyboard")
    except BaseException as error:
        print("ERROR,%s" % str(error))
        sys.print_exception(error)
    finally:
        if pipeline is not None:
            pipeline.destroy()
        gc.collect()


if __name__ == "__main__":
    main()
