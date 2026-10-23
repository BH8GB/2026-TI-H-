"""K230 YOLO11钢球跟踪，串口输出到MSPM0G。"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *

import os
import sys
import gc
import time
from machine import FPIOA, UART


# 模型和显示参数需与kmodel一致
KMODEL_PATH = "/sdcard/kmodel/steelball.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_MODE = "lcd"
RGB888P_SIZE = [640, 320]
CONFIDENCE_THRESHOLD = 0.15
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 50

# 只接受中心在管道区域内的检测
# 固定相机支架后调整该矩形
PIPE_ROI = (40, 90, 560, 150)
MAX_TRACK_JUMP_PX = 160.0
DISPLAY_ROI_ONLY = True
GC_EVERY_FRAMES = 30
FPS_LOG_PERIOD_MS = 1000

# 三点标定，读取负125、0、正125毫米处的图像横坐标
# 方向相反时修改X方向
CAL_LEFT_PX = 60.0
CAL_CENTER_PX = 320.0
CAL_RIGHT_PX = 580.0
CAL_LEFT_MM = -125.0
CAL_CENTER_MM = 0.0
CAL_RIGHT_MM = 125.0
X_DIRECTION = 1.0

POSITION_ALPHA = 0.50
VELOCITY_ALPHA = 0.30
MAX_SPEED_MM_S = 2200.0

# K230 UART1接MSPM0G PA9，注意共地
# GPIO10只做映射，旧版UART会检查接收
UART_TX_GPIO = 9
UART_RX_GPIO = 10
UART_BAUD = 9600
FRAME_SIZE = 10


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def pixel_to_mm(x_px):
    """Convert image x to groove position using piecewise linear calibration."""
    if x_px <= CAL_CENTER_PX:
        span = CAL_CENTER_PX - CAL_LEFT_PX
        if span < 1.0:
            return 0.0
        ratio = (x_px - CAL_LEFT_PX) / span
        value = CAL_LEFT_MM + ratio * (CAL_CENTER_MM - CAL_LEFT_MM)
    else:
        span = CAL_RIGHT_PX - CAL_CENTER_PX
        if span < 1.0:
            return 0.0
        ratio = (x_px - CAL_CENTER_PX) / span
        value = CAL_CENTER_MM + ratio * (CAL_RIGHT_MM - CAL_CENTER_MM)
    return value * X_DIRECTION


class PacketSender:
    """Send the fixed 10-byte frame expected by ball_control.c."""

    def __init__(self):
        self.fpioa = FPIOA()
        self.fpioa.set_function(UART_TX_GPIO, FPIOA.UART1_TXD)
        self.fpioa.set_function(UART_RX_GPIO, FPIOA.UART1_RXD)
        self.uart = UART(
            UART.UART1,
            baudrate=UART_BAUD,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        self.sequence = 0

    def send(self, valid, position_mm, velocity_mm_s, confidence):
        position = int(clamp(round(position_mm), -32768, 32767)) & 0xFFFF
        velocity = int(clamp(round(velocity_mm_s), -32768, 32767)) & 0xFFFF
        confidence_u8 = int(clamp(round(confidence * 255.0), 0, 255))
        packet = bytearray(FRAME_SIZE)
        packet[0] = 0xAA
        packet[1] = 0x55
        packet[2] = 0x01 if valid else 0x00
        packet[3] = self.sequence
        packet[4] = position & 0xFF
        packet[5] = (position >> 8) & 0xFF
        packet[6] = velocity & 0xFF
        packet[7] = (velocity >> 8) & 0xFF
        packet[8] = confidence_u8 if valid else 0
        checksum = 0
        for index in range(FRAME_SIZE - 1):
            checksum = (checksum + packet[index]) & 0xFF
        packet[9] = checksum
        self.uart.write(packet)
        self.sequence = (self.sequence + 1) & 0xFF

    def deinit(self):
        try:
            self.uart.deinit()
        except Exception:
            pass


class BallTracker:
    """Select one ROI detection and filter its position and velocity."""

    def __init__(self):
        self.visible = False
        self.x_px = 0.0
        self.y_px = 0.0
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.confidence = 0.0
        self.last_update_ms = None

    def select(self, result):
        if result is None or len(result) < 3:
            return None
        boxes = result[0]
        scores = result[2]
        roi_x, roi_y, roi_w, roi_h = PIPE_ROI
        best = None
        best_score = 1000000.0
        for index in range(len(boxes)):
            box = boxes[index].tolist()
            x_px = float(box[0]) + float(box[2]) * 0.5
            y_px = float(box[1]) + float(box[3]) * 0.5
            confidence = float(scores[index])
            if not (roi_x <= x_px <= roi_x + roi_w and
                    roi_y <= y_px <= roi_y + roi_h):
                continue
            if self.visible:
                jump = abs(x_px - self.x_px)
                if jump > MAX_TRACK_JUMP_PX:
                    continue
                score = jump + 0.35 * abs(y_px - self.y_px)
            else:
                score = abs(y_px - (roi_y + roi_h * 0.5))
            score -= confidence * 80.0
            if score < best_score:
                best_score = score
                best = (x_px, y_px, confidence)
        return best

    def update(self, selected, now_ms):
        if selected is None:
            self.visible = False
            self.velocity_mm_s = 0.0
            return False
        raw_x, raw_y, confidence = selected
        old_position = self.position_mm
        if not self.visible or self.last_update_ms is None:
            self.x_px = raw_x
            self.y_px = raw_y
            self.position_mm = pixel_to_mm(raw_x)
            self.velocity_mm_s = 0.0
        else:
            self.x_px += POSITION_ALPHA * (raw_x - self.x_px)
            self.y_px += POSITION_ALPHA * (raw_y - self.y_px)
            self.position_mm = pixel_to_mm(self.x_px)
            dt_ms = time.ticks_diff(now_ms, self.last_update_ms)
            if dt_ms > 0:
                measured = (self.position_mm - old_position) * 1000.0 / dt_ms
                measured = clamp(measured, -MAX_SPEED_MM_S, MAX_SPEED_MM_S)
                self.velocity_mm_s += VELOCITY_ALPHA * (
                    measured - self.velocity_mm_s)
        self.visible = True
        self.confidence = confidence
        self.last_update_ms = now_ms
        return True


def main():
    pipeline = None
    yolo = None
    sender = None
    try:
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create()
        display_size = pipeline.get_display_size()
        yolo = YOLO11(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        yolo.config_preprocess()
        sender = PacketSender()
        tracker = BallTracker()
        clock = time.clock()
        frame_count = 0
        last_fps_log_ms = time.ticks_ms()
        print("READY,K230_YOLO11_BALL_UART1_9600")

        while True:
            os.exitpoint()
            clock.tick()
            frame_count += 1
            image = pipeline.get_frame()
            result = yolo.run(image)
            now_ms = time.ticks_ms()
            valid = tracker.update(tracker.select(result), now_ms)
            sender.send(
                valid,
                tracker.position_mm if valid else 0.0,
                tracker.velocity_mm_s if valid else 0.0,
                tracker.confidence if valid else 0.0,
            )
            yolo.draw_result(result, pipeline.osd_img)
            roi_x, roi_y, roi_w, roi_h = PIPE_ROI

            # 显示层有缩放，先把相机区域转换到显示坐标
            # 再遮住上下未使用区域
            scale_x = float(display_size[0]) / float(RGB888P_SIZE[0])
            scale_y = float(display_size[1]) / float(RGB888P_SIZE[1])
            display_roi_x = int(roi_x * scale_x)
            display_roi_y = int(roi_y * scale_y)
            display_roi_w = int(roi_w * scale_x)
            display_roi_h = int(roi_h * scale_y)
            if DISPLAY_ROI_ONLY:
                pipeline.osd_img.draw_rectangle(
                    0, 0, display_size[0], display_roi_y,
                    color=(255, 0, 0, 0), fill=True)
                lower_y = display_roi_y + display_roi_h
                pipeline.osd_img.draw_rectangle(
                    0, lower_y, display_size[0], display_size[1] - lower_y,
                    color=(255, 0, 0, 0), fill=True)
            pipeline.osd_img.draw_rectangle(
                display_roi_x, display_roi_y, display_roi_w, display_roi_h,
                color=(255, 0, 255, 0), thickness=2)
            if valid:
                pipeline.osd_img.draw_cross(
                    int(tracker.x_px * scale_x), int(tracker.y_px * scale_y),
                    color=(255, 255, 255, 0), size=12, thickness=2)
            fps = clock.fps()
            status = "FPS:%.1f  X:%dmm" % (
                fps, int(tracker.position_mm) if valid else 0)
            pipeline.osd_img.draw_string(
                display_roi_x + 4, display_roi_y + 4, status,
                color=(255, 255, 255, 0), scale=2)
            pipeline.show_image()

            if time.ticks_diff(now_ms, last_fps_log_ms) >= FPS_LOG_PERIOD_MS:
                last_fps_log_ms = now_ms
                print("FPS,%.1f,VALID,%d,X_MM,%d" % (
                    fps, 1 if valid else 0,
                    int(tracker.position_mm) if valid else 0))
            if frame_count % GC_EVERY_FRAMES == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("Program stopped")
    except Exception as error:
        print("Program error:")
        sys.print_exception(error)
    finally:
        if sender is not None:
            sender.deinit()
        if yolo is not None:
            yolo.deinit()
        if pipeline is not None:
            pipeline.destroy()
        gc.collect()


if __name__ == "__main__":
    main()
