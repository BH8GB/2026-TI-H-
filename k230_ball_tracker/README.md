# K230 钢球检测与坐标输出

第三问使用 `yabo_k230_roi.py`。它运行 YOLO11 `steelball.kmodel`，只接受四个
标定点围成的水管区域内的钢球，并通过串口把位置、滚动速度和置信度发给
MSPM0G3507。舵机 PWM 和 P+D 闭环在 TI 板上完成，K230 不直接驱动舵机。

## 部署

1. 把 `yabo_k230_roi.py` 和训练好的 `steelball.kmodel` 放到 K230 的 `/sdcard`。
2. 保持模型路径为 `/sdcard/kmodel/steelball.kmodel`，显示模式为 `lcd`。
3. 固定相机后确认紫色四边形只覆盖凹槽，不包含车体和背景。
4. K230 排针 3 `GPIO9/UART1_TXD` 接 MSPM0G3507 的串口 RX。
5. K230 排针 4 `GPIO10/UART1_RXD` 已在 FPIOA 中配置；单向发送时可以不接线。
6. K230 GND、MSPM0G GND、舵机电源 GND 必须共地。

串口参数为 `9600, 8N1`。GPIO9/GPIO10 是 3.3 V 信号，不能接 5 V 串口电平。
当前 legacy 固件创建 UART 时会检查 TX 和 RX，因此程序必须同时执行
`FPIOA.UART1_TXD` 与 `FPIOA.UART1_RXD` 的引脚复用，即使只使用 TX 发送。

## 相机标定

不需要测相机到钢球的距离。当前四个标记框已经写入 `MARKER_BOXES`，程序取
左右短边中点作为 25 cm 水管轴，将中间位置映射为 `0 mm`：

```python
MARKER_BOXES = (
    (253.0, 63.0, 6.0, 6.0),
    (247.0, 634.0, 6.0, 6.0),
    (208.0, 48.0, 6.0, 6.0),
    (202.0, 640.0, 6.0, 3.0),
)
```

程序把球心投影到倾斜轴，可补偿相机轻微旋转。若正负方向相反，把
`POSITION_DIRECTION` 改为 `-1.0`。移动摄像头、改变支架高度或焦距后必须重标。

## 数据帧

程序最多每 30 ms 发送一个 10 字节帧：

```text
AA 55 flags seq position_mm_L position_mm_H velocity_mm_s_L velocity_mm_s_H confidence checksum
```

- `flags.bit0=1`：当前帧识别到 ROI 内钢球。
- `position_mm`、`velocity_mm_s`：小端 `int16`。
- `confidence`：0~255。
- `checksum`：字节 0~8 累加和的低 8 位。

未识别到钢球时仍发送无效帧。TI 板保留上一有效测量 120 ms，用来过滤单帧
漏检；连续失联或第三问 250 ms 没有新有效帧会触发安全保护。

## 调试顺序

先只运行 K230，确认屏幕框始终落在唯一钢球上，并完成三点像素标定。然后接
串口，TI 板 OLED 第 7 行应从 `B:NOLINK` 变为 `B:OK`。最后拆下舵机连杆调
机械中位，再以很小的摆杆角度开始闭环整定。屏幕中的 `V:mm/s` 是钢球沿
水管的滚动速度，不是小车轮速。
