"""K230钢球跟踪，串口输出到MSPM0G。相机固定，用三点标定把像素换算为毫米。"""

import gc
import os
import sys
import time

import ulab.numpy as np
from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from machine import FPIOA, UART



# 模型、显示和管道区域


KMODEL_PATH = "/sdcard/model.kmodel"
LABELS = ["ball"]
MODEL_INPUT = [640, 640]
RGB888P_SIZE = [640, 480]
DISPLAY_SIZE = [640, 480]
DISPLAY_MODE = "st7701"
CONF_THRESH = 0.50
NMS_THRESH = 0.50
MAX_BOXES = 20

# 只接受中心在该矩形内的检测
# 固定相机支架后调整高度，只覆盖管道
PIPE_ROI = (40, 140, 560, 200)
PIPE_CENTER_Y = PIPE_ROI[1] + PIPE_ROI[3] // 2
MAX_TRACK_JUMP_PX = 160.0



# 像素到毫米标定


# 钢球放在负125、0、正125毫米处，读取屏幕横坐标
# 分段插值可容忍轻微倾斜
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



# K230到MSPM0串口


# K230排针3和4对应UART1的GPIO9和GPIO10
# 旧版CanMV即使只发送也检查收发映射
# 先配置两个信号再创建UART
# 排针3接MSPM0G接收脚并共地
UART_TX_GPIO = 9
UART_RX_GPIO = 10
UART_BAUD = 9600

# 固定10字节帧
# 帧头AA 55，后接标志、序号、位置、速度、置信度和校验
# 标志最低位表示本帧有效，校验为前9字节累加和低8位
FRAME_SIZE = 10


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def pixel_to_mm(x_px):
    """Map the observed x coordinate using the three physical scale marks."""
    if x_px <= CAL_CENTER_PX:
        span_px = CAL_CENTER_PX - CAL_LEFT_PX
        if span_px < 1.0:
            return 0.0
        ratio = (x_px - CAL_LEFT_PX) / span_px
        value = CAL_LEFT_MM + ratio * (CAL_CENTER_MM - CAL_LEFT_MM)
    else:
        span_px = CAL_RIGHT_PX - CAL_CENTER_PX
        if span_px < 1.0:
            return 0.0
        ratio = (x_px - CAL_CENTER_PX) / span_px
        value = CAL_CENTER_MM + ratio * (CAL_RIGHT_MM - CAL_CENTER_MM)
    return value * X_DIRECTION


class PacketSender:
    """Pack signed measurements without relying on struct module availability."""

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
    """Select one ROI target and filter its position and measured velocity."""

    def __init__(self):
        self.visible = False
        self.x_px = 0.0
        self.y_px = 0.0
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.confidence = 0.0
        self.last_update_ms = None

    def _inside_roi(self, x_px, y_px):
        roi_x, roi_y, roi_w, roi_h = PIPE_ROI
        return (x_px >= roi_x and x_px <= roi_x + roi_w and
                y_px >= roi_y and y_px <= roi_y + roi_h)

    def select(self, result):
        boxes = result[0]
        scores = result[2]
        best = None
        best_score = 1000000.0

        for index in range(len(boxes)):
            box = boxes[index].tolist()
            x_px = float(box[0]) + float(box[2]) * 0.5
            y_px = float(box[1]) + float(box[3]) * 0.5
            confidence = float(scores[index])
            if not self._inside_roi(x_px, y_px):
                continue

            if self.visible:
                jump = abs(x_px - self.x_px)
                if jump > MAX_TRACK_JUMP_PX:
                    continue
                score = jump + 0.35 * abs(y_px - self.y_px)
            else:
                score = 0.50 * abs(y_px - PIPE_CENTER_Y)
            score -= confidence * 80.0

            if score < best_score:
                best_score = score
                best = (box, x_px, y_px, confidence)
        return best

    def update(self, selected, now_ms):
        if selected is None:
            self.visible = False
            self.velocity_mm_s = 0.0
            return False

        box, raw_x_px, raw_y_px, confidence = selected
        old_position_mm = self.position_mm

        if not self.visible or self.last_update_ms is None:
            self.x_px = raw_x_px
            self.y_px = raw_y_px
            self.position_mm = pixel_to_mm(self.x_px)
            self.velocity_mm_s = 0.0
        else:
            self.x_px += POSITION_ALPHA * (raw_x_px - self.x_px)
            self.y_px += POSITION_ALPHA * (raw_y_px - self.y_px)
            self.position_mm = pixel_to_mm(self.x_px)
            dt_ms = time.ticks_diff(now_ms, self.last_update_ms)
            if dt_ms > 0:
                measured_velocity = ((self.position_mm - old_position_mm) *
                                     1000.0 / float(dt_ms))
                measured_velocity = clamp(
                    measured_velocity, -MAX_SPEED_MM_S, MAX_SPEED_MM_S)
                self.velocity_mm_s += VELOCITY_ALPHA * (
                    measured_velocity - self.velocity_mm_s)

        self.visible = True
        self.confidence = confidence
        self.last_update_ms = now_ms
        return True


def draw_overlay(osd_image, tracker, selected, fps):
    """Draw only the groove ROI and the selected ball, not every YOLO box."""
    roi_x, roi_y, roi_w, roi_h = PIPE_ROI
    osd_image.draw_rectangle(
        roi_x, roi_y, roi_w, roi_h, color=(0, 255, 0), thickness=2)

    if tracker.visible and selected is not None:
        box = selected[0]
        osd_image.draw_rectangle(
            int(box[0]), int(box[1]), int(box[2]), int(box[3]),
            color=(255, 0, 0), thickness=2)
        osd_image.draw_cross(
            int(tracker.x_px), int(tracker.y_px),
            color=(255, 255, 0), size=12, thickness=2)
        status = "BALL px=%d x=%dmm" % (
            int(tracker.x_px), int(tracker.position_mm))
    else:
        status = "BALL LOST"

    try:
        osd_image.draw_string(4, 4, status, color=(255, 0, 0), scale=2)
        osd_image.draw_string(
            4, 30, "FPS=%.1f c=%.2f" % (fps, tracker.confidence),
            color=(255, 255, 0), scale=2)
    except Exception:
        pass


def main():
    pl = None
    yolo = None
    sender = None
    try:
        pl = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pl.create()
        yolo = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT,
            display_size=DISPLAY_SIZE,
            conf_thresh=CONF_THRESH,
            nms_thresh=NMS_THRESH,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        yolo.config_preprocess()
        sender = PacketSender()
        tracker = BallTracker()
        clock = time.clock()
        frame_count = 0
        last_log_ms = time.ticks_ms()

        print("READY,K230_YOLO_BALL_UART1_9600")
        print("CAL,left=%.1f,center=%.1f,right=%.1f" % (
            CAL_LEFT_PX, CAL_CENTER_PX, CAL_RIGHT_PX))

        while True:
            os.exitpoint()
            clock.tick()
            frame_count += 1
            image = pl.get_frame()
            result = yolo.run(image)
            now_ms = time.ticks_ms()
            selected = tracker.select(result)
            valid = tracker.update(selected, now_ms)

            if valid:
                sender.send(
                    True,
                    tracker.position_mm,
                    tracker.velocity_mm_s,
                    tracker.confidence,
                )
            else:
                sender.send(False, 0.0, 0.0, 0.0)

            pl.osd_img.clear()
            draw_overlay(pl.osd_img, tracker, selected, clock.fps())
            pl.show_image()

            if time.ticks_diff(now_ms, last_log_ms) >= 500:
                last_log_ms = now_ms
                if valid:
                    print("BALL,1,%d,%d,%d,%d" % (
                        int(tracker.position_mm),
                        int(tracker.velocity_mm_s),
                        int(tracker.confidence * 255.0),
                        int(tracker.x_px),
                    ))
                else:
                    print("BALL,0,0,0,0")

            del image
            if frame_count % 30 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("STOP,keyboard")
    except BaseException as error:
        print("ERROR,%s" % str(error))
        sys.print_exception(error)
    finally:
        if sender is not None:
            sender.deinit()
        if yolo is not None:
            yolo.deinit()
        if pl is not None:
            pl.destroy()
        print("CLEANUP,done")


if __name__ == "__main__":
    main()
