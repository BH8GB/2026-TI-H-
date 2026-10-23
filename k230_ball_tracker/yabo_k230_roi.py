"""K230钢球检测，只接受管道区域内的钢球，并输出25厘米坐标。"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *

import os
import sys
import gc
import time
from machine import FPIOA, UART



# 模型和显示配置


KMODEL_PATH = "/sdcard/kmodel/steelball.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_MODE = "lcd"
RGB888P_SIZE = [640, 320]

CONFIDENCE_THRESHOLD = 0.15
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 50
GC_EVERY_FRAMES = 300

# 每30毫秒发送一次，兼顾延迟和9600波特率
UART_TX_GPIO = 9
UART_RX_GPIO = 10
UART_BAUD = 9600
UART_SEND_PERIOD_MS = 30
PACKET_SIZE = 10

# 位置低通后差分得到速度，供D环提前制动
POSITION_ALPHA = 0.50
VELOCITY_ALPHA = 0.30
MAX_SPEED_MM_S = 2200.0



# 水管区域和25厘米坐标标定


# 钢球放在负125、0、正125毫米处，读取屏幕横坐标
# 用分段插值补偿轻微倾斜

IDE_FRAMEBUFFER_SIZE = [480.0, 640.0]
MARKER_BOXES = (
    (253.0, 63.0, 6.0, 6.0),
    (247.0, 634.0, 6.0, 6.0),
    (208.0, 48.0, 6.0, 6.0),
    (202.0, 640.0, 6.0, 3.0),
)

PIPE_LENGTH_CM = 25.0
POSITION_DIRECTION = 1.0  # 右侧为正，方向相反时改为负1
CENTER_RADIUS_FRAME_PX = 10
OVERLAY_Y_SCALE = RGB888P_SIZE[1] / IDE_FRAMEBUFFER_SIZE[0]
OVERLAY_Y_OFFSET_FRAME_PX = 6.0  # 正数向下微调屏幕标记


def marker_center_to_frame(marker):
    """将 IDE 竖屏取点框中心转换为 YOLO 检测坐标。"""
    marker_x, marker_y, marker_w, marker_h = marker
    frame_x = marker_y + marker_h * 0.5
    frame_y = marker_x + marker_w * 0.5
    return frame_x, frame_y


def detection_to_overlay(point_x, point_y):
    """把 YOLO 检测点转换为与视频画面重合的 OSD 绘图点。"""
    return (point_x,
            point_y * OVERLAY_Y_SCALE + OVERLAY_Y_OFFSET_FRAME_PX)


# 四边形按左上、右上、右下、左下排列
_point_1 = marker_center_to_frame(MARKER_BOXES[0])
_point_2 = marker_center_to_frame(MARKER_BOXES[1])
_point_3 = marker_center_to_frame(MARKER_BOXES[2])
_point_4 = marker_center_to_frame(MARKER_BOXES[3])
PIPE_QUAD = (_point_3, _point_4, _point_2, _point_1)

# 用左右短边中点确定水管中心轴
AXIS_START = (
    (PIPE_QUAD[0][0] + PIPE_QUAD[3][0]) * 0.5,
    (PIPE_QUAD[0][1] + PIPE_QUAD[3][1]) * 0.5,
)
AXIS_END = (
    (PIPE_QUAD[1][0] + PIPE_QUAD[2][0]) * 0.5,
    (PIPE_QUAD[1][1] + PIPE_QUAD[2][1]) * 0.5,
)
AXIS_CENTER = (
    (AXIS_START[0] + AXIS_END[0]) * 0.5,
    (AXIS_START[1] + AXIS_END[1]) * 0.5,
)
AXIS_DX = AXIS_END[0] - AXIS_START[0]
AXIS_DY = AXIS_END[1] - AXIS_START[1]
AXIS_LENGTH_SQUARED = AXIS_DX * AXIS_DX + AXIS_DY * AXIS_DY


def point_in_quad(point_x, point_y):
    """用射线法判断钢球中心是否位于标定四边形内。"""
    inside = False
    previous = len(PIPE_QUAD) - 1
    for current in range(len(PIPE_QUAD)):
        current_x, current_y = PIPE_QUAD[current]
        previous_x, previous_y = PIPE_QUAD[previous]
        crosses_y = ((current_y > point_y) != (previous_y > point_y))
        if crosses_y:
            crossing_x = ((previous_x - current_x) *
                          (point_y - current_y) /
                          (previous_y - current_y) + current_x)
            if point_x < crossing_x:
                inside = not inside
        previous = current
    return inside


def pixel_to_pipe_cm(point_x, point_y):
    """将钢球像素中心投影到水管轴，并换算成相对中点的厘米坐标。"""
    if AXIS_LENGTH_SQUARED <= 0.0:
        return 0.0

    projection = (((point_x - AXIS_START[0]) * AXIS_DX +
                   (point_y - AXIS_START[1]) * AXIS_DY) /
                  AXIS_LENGTH_SQUARED)
    if projection < 0.0:
        projection = 0.0
    elif projection > 1.0:
        projection = 1.0

    return ((projection - 0.5) * PIPE_LENGTH_CM *
            POSITION_DIRECTION)


def clamp(value, lower, upper):
    """限制串口定点数和速度异常值。"""
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


class PacketSender:
    """发送 ball_control.c 约定的 AA 55 十字节定长帧。"""

    def __init__(self):
        self.fpioa = FPIOA()
        self.fpioa.set_function(UART_TX_GPIO, FPIOA.UART1_TXD)
        # 只用发送，但旧版UART仍检查接收映射
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
        packet = bytearray(PACKET_SIZE)
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
        for index in range(PACKET_SIZE - 1):
            checksum = (checksum + packet[index]) & 0xFF
        packet[9] = checksum
        self.uart.write(packet)
        self.sequence = (self.sequence + 1) & 0xFF

    def deinit(self):
        try:
            self.uart.deinit()
        except Exception:
            pass


class BallMotionFilter:
    """对毫米位置低通，并用固定时标差分估计钢球速度。"""

    def __init__(self):
        self.has_state = False
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.last_valid_ms = None

    def update(self, selected, now_ms):
        if selected is None:
            return False

        raw_position_mm = pixel_to_pipe_cm(selected[4], selected[5]) * 10.0
        if not self.has_state or self.last_valid_ms is None:
            self.position_mm = raw_position_mm
            self.velocity_mm_s = 0.0
            self.has_state = True
        else:
            old_position_mm = self.position_mm
            self.position_mm += POSITION_ALPHA * (
                raw_position_mm - self.position_mm)
            elapsed_ms = time.ticks_diff(now_ms, self.last_valid_ms)
            if elapsed_ms > 0:
                measured_velocity = ((self.position_mm - old_position_mm) *
                                     1000.0 / elapsed_ms)
                measured_velocity = clamp(
                    measured_velocity, -MAX_SPEED_MM_S, MAX_SPEED_MM_S)
                self.velocity_mm_s += VELOCITY_ALPHA * (
                    measured_velocity - self.velocity_mm_s)
        self.last_valid_ms = now_ms
        return True


def select_pipe_ball(result):
    """只从水管四边形内部选择置信度最高的钢球。"""
    if result is None or len(result) < 3:
        return None

    boxes = result[0]
    scores = result[2]
    best = None
    best_score = -1.0

    for index in range(len(boxes)):
        box = boxes[index]
        if len(box) < 4:
            continue

        # 逐项读取，兼容旧版ulab
        box_x = float(box[0])
        box_y = float(box[1])
        box_w = float(box[2])
        box_h = float(box[3])
        center_x = box_x + box_w * 0.5
        center_y = box_y + box_h * 0.5

        if not point_in_quad(center_x, center_y):
            continue

        score = float(scores[index])
        if score > best_score:
            best_score = score
            best = (box_x, box_y, box_w, box_h,
                    center_x, center_y, score)

    return best


def clamp_int(value, lower, upper):
    """限制绘图坐标，防止最右侧标定点略微超出帧边界。"""
    value = int(value)
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def frame_to_display(point_x, point_y, scale_x, scale_y,
                     display_width, display_height):
    """把模型帧坐标转换成 OSD 显示坐标。"""
    return (
        clamp_int(point_x * scale_x, 0, display_width - 1),
        clamp_int(point_y * scale_y, 0, display_height - 1),
    )


def draw_pipe_overlay(osd_img, scale_x, scale_y,
                      display_width, display_height):
    """绘制限定区域、中心轴和代表 0 cm 的圆圈。"""
    for index in range(len(PIPE_QUAD)):
        next_index = (index + 1) % len(PIPE_QUAD)
        overlay_start = detection_to_overlay(
            PIPE_QUAD[index][0], PIPE_QUAD[index][1])
        overlay_end = detection_to_overlay(
            PIPE_QUAD[next_index][0], PIPE_QUAD[next_index][1])
        start = frame_to_display(
            overlay_start[0], overlay_start[1],
            scale_x, scale_y, display_width, display_height)
        end = frame_to_display(
            overlay_end[0], overlay_end[1],
            scale_x, scale_y, display_width, display_height)
        osd_img.draw_line(
            start[0], start[1], end[0], end[1],
            color=(255, 0, 255, 0), thickness=2)

    overlay_axis_start = detection_to_overlay(
        AXIS_START[0], AXIS_START[1])
    overlay_axis_end = detection_to_overlay(
        AXIS_END[0], AXIS_END[1])
    axis_start = frame_to_display(
        overlay_axis_start[0], overlay_axis_start[1], scale_x, scale_y,
        display_width, display_height)
    axis_end = frame_to_display(
        overlay_axis_end[0], overlay_axis_end[1], scale_x, scale_y,
        display_width, display_height)
    osd_img.draw_line(
        axis_start[0], axis_start[1], axis_end[0], axis_end[1],
        color=(255, 255, 255, 0), thickness=1)

    overlay_center = detection_to_overlay(
        AXIS_CENTER[0], AXIS_CENTER[1])
    center = frame_to_display(
        overlay_center[0], overlay_center[1], scale_x, scale_y,
        display_width, display_height)
    center_radius = int(CENTER_RADIUS_FRAME_PX *
                        (scale_x + scale_y) * 0.5)
    if center_radius < 4:
        center_radius = 4
    osd_img.draw_circle(
        center[0], center[1], center_radius,
        color=(255, 255, 255, 0), thickness=2)
    osd_img.draw_cross(
        center[0], center[1],
        color=(255, 255, 255, 0), size=center_radius, thickness=1)


pl = None
yolo = None
sender = None


try:
    
    # 初始化摄像头、屏幕和YOLO11
    

    pl = PipeLine(
        rgb888p_size=RGB888P_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pl.create()
    display_size = pl.get_display_size()
    display_width = int(display_size[0])
    display_height = int(display_size[1])
    scale_x = float(display_width) / float(RGB888P_SIZE[0])
    scale_y = float(display_height) / float(RGB888P_SIZE[1])

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
    motion = BallMotionFilter()

    clock = time.clock()
    frame_count = 0
    last_send_ms = time.ticks_ms() - UART_SEND_PERIOD_MS
    print("READY,PIPE_ROI_25CM_UART1_9600")

    
    # 检测、区域过滤、坐标映射和显示
    

    while True:
        os.exitpoint()
        clock.tick()
        frame_count += 1

        image = pl.get_frame()
        result = yolo.run(image)
        selected = select_pipe_ball(result)
        now_ms = time.ticks_ms()
        valid = motion.update(selected, now_ms)

        if time.ticks_diff(now_ms, last_send_ms) >= UART_SEND_PERIOD_MS:
            last_send_ms = now_ms
            sender.send(
                valid,
                motion.position_mm if valid else 0.0,
                motion.velocity_mm_s if valid else 0.0,
                selected[6] if valid else 0.0,
            )

        # 不调用draw_result，避免显示区域外检测框
        pl.osd_img.clear()
        draw_pipe_overlay(
            pl.osd_img, scale_x, scale_y,
            display_width, display_height)

        if selected is not None:
            box_x, box_y, box_w, box_h = selected[0:4]
            center_x, center_y, confidence = selected[4:7]
            position_cm = pixel_to_pipe_cm(center_x, center_y)

            overlay_box_start = detection_to_overlay(box_x, box_y)
            overlay_box_end = detection_to_overlay(
                box_x + box_w, box_y + box_h)
            overlay_ball_center = detection_to_overlay(center_x, center_y)

            box_start = frame_to_display(
                overlay_box_start[0], overlay_box_start[1], scale_x, scale_y,
                display_width, display_height)
            box_end = frame_to_display(
                overlay_box_end[0], overlay_box_end[1], scale_x, scale_y,
                display_width, display_height)
            ball_center = frame_to_display(
                overlay_ball_center[0], overlay_ball_center[1],
                scale_x, scale_y,
                display_width, display_height)

            pl.osd_img.draw_rectangle(
                box_start[0], box_start[1],
                max(1, box_end[0] - box_start[0]),
                max(1, box_end[1] - box_start[1]),
                color=(255, 255, 0, 0), thickness=2)
            pl.osd_img.draw_circle(
                ball_center[0], ball_center[1], 8,
                color=(255, 255, 0, 0), thickness=2)
            pl.osd_img.draw_cross(
                ball_center[0], ball_center[1],
                color=(255, 255, 0, 0), size=8, thickness=1)

            status = "X:%+.1fcm V:%+dmm/s C:%.2f" % (
                motion.position_mm / 10.0,
                int(motion.velocity_mm_s), confidence)
        else:
            status = "BALL LOST IN PIPE ROI"

        pl.osd_img.draw_string(
            8, 8, "FPS:%.1f %s" % (clock.fps(), status),
            color=(255, 255, 255, 0), scale=2)
        pl.show_image()

        del image
        if frame_count % GC_EVERY_FRAMES == 0:
            gc.collect()


except KeyboardInterrupt:
    print("Program stopped")


except BaseException as error:
    print("Program error:")
    sys.print_exception(error)


finally:
    if yolo is not None:
        yolo.deinit()

    if sender is not None:
        sender.deinit()

    if pl is not None:
        pl.destroy()

    gc.collect()
