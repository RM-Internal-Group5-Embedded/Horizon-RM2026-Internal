## JC24B 交接说明

### 体系概览

**STM32 侧（`Core/Modules/jc24btran.*` + `Core/Src/UserTask.cpp`）**
- `transceiverTask` 静态创建，初始化 `jc24b::DataTransceiver` 并每 50 ms 调一次 `process()`；任务内 `loop_count` 触发 LED0 每秒翻转，作为看门狗指示。
- `DataTransceiver` 用 `HAL_UARTEx_ReceiveToIdle_DMA` 打开 USART2 DMA 接收；在 Rx 回调内校验 `[0xAA, 0xBB, pos1..4, checksum]`，通过 `DEBOUNCE_FRAMES = 3` 去抖后写 `stable_positions_`，刷新 `last_rx_time_`，并回复 `[0xCC, 0xDD, status, checksum]` ACK。
- 指示灯含义（可在调试时观察）：  
  - LED0：运行中（1 Hz 闪烁）。  
  - LED1：进入DMA回调 
  - LED2：/

**PC 侧（`Vision/JC42BSending.py`）**
- `JC24BTransceiver` 封装串口连接、互斥、线程化 `_communication_loop`。默认严格 `time.sleep(0.1)`，每轮构建 `[0xAA 0xBB pos1..4 checksum]` 并写串口。
- `check_for_ack()` 非阻塞读取 4 字节 ACK，若连续 5 次失败打印警告；`last_ack_time` 用于健康检查。

**视觉侧接入（`Vision/block_detect.py` 等）**
- `define_regions.py` 提供交互式绘制 4 个检测区域（或颜色板区域），结果写入 `config/block_regions_view.json`。
- `color_plate.py` 负责颜色掩膜与 ROI。
- `block_detect.py` 主循环：相机帧→二值化→检测矩形→判断中心点是否进入任一区域。当前同学的实现是“仅在某区域从无到有时立即 `transceiver.send_positions()` 单次发包”，并没有把数据挂到 `_communication_loop` 的 0.1 s 回调里。

### 接线与硬件配置

- **目标接口**（STM32G431CB-UART2）：  
  - `PA2` → `USART2_TX`（发往 PC/JC24B 的 TX）。  
  - `PA3` → `USART2_RX`（接收 PC/JC24B 的 RX）。  
  - `PA0` → `PD`（低电平=允许通信/模块掉电信号，参考 JC24B 手册）。  
  - `PA1` → `SET`（高电平=模块开启通信）。  
- **JC24B 模块端**：
    板子上从上往下应该是
  - `JC24B_RXD` 接 `PA2 (USART2_TX)`； 
  - `JC24B_TXD` 接 `PA3 (USART2_RX)`；   
  - `JC24B_PD` 接 `PA0`，拉低通信模式，拉高休眠模式；  
  - `JC24B_SET` 接 `PA1`，上电后拉高通信模式，拉低配置模式，已经配置完成所以只管拉高就好；  
  - GND 共地，VCC 依照模块规格 3.3 V。

### 测试

- 2025-11-14 
    pc端根据视觉端的思路仅在检测到方块进入时发送，stm端接收使用dma中断进入回调，检测数据有效性，发回ack校验包，pc端接收，（在接收无效或错误时报警），（由于仅单次发送防抖会影响位置输出）

### 后续修改思路

如果只在新方块到达时发送一次：

1. **删除 STM32 去抖**：直接把去抖uint8_t debounce_counters_相关逻辑移除。增加去抖是我在测试中观测到拍摄方块是可能会出现很短暂的失去检测，在0.1s的情况下有必要，但仅检测进入或许不需要。
2. **可以引入心跳帧，也可以不要**：  
   - PC 端仍在 `_communication_loop` 中每 0.1 s 发送“心跳包”。心跳包可约定为 `[0xAA, 0xEE, 0x00, 0x00, 0x00, 0x00, checksum]`（仅第二个字节不同，表示这帧不是位置数据）。  
   - 真正的位置信息仍使用第二字节 `0xBB`。在接收时写入position_便于进行位置决策。
   - STM32 侧在 Rx 回调识别帧头：  
       * `0xAA 0xBB` → 正常数据，LED1 点亮表示收到。  
       * `0xAA 0xEE` → 心跳，仅用来刷新看门狗，可让 LED2 维持常亮表示“通信正常”。  
   - 如此兼容事件驱动和去抖（去抖可保留，用于 0xBB 包），并且通过心跳刷新连接状态。
