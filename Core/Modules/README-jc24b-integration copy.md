# JC24B DataTransceiver 对接说明（板内模块）

此文档面向在板上其他模块（FreeRTOS 任务、驱动或应用逻辑）的工程师，描述如何集成并使用 `jc24b::DataTransceiver` 来获取接收到的 `positions`（4 个位置状态）。

文件位置
- 模块实现：`Core/Modules/jc24btran.hpp` 与 `Core/Modules/jc24btran.cpp`
- 示例任务（参考实现）：`Core/Src/transceiver_user_example.cpp`

概述
- 外部通过串口向板子发送 7 字节的位置包（格式详见下文），板子校验并更新内部去抖后的 4 路位置状态。
- 板上模块通过 `DataTransceiver` 提供的 API 读取这些去抖后的稳定状态。

主要 API（简明）
- `void init(UART_HandleTypeDef* huart)` — 在串口初始化后调用以启动 DMA 接收。
- `void process()` — 在周期性任务中调用，用于看门狗检查与重连处理（建议 50~200 ms 周期）。
- `bool getPositions(bool positions[4])` — 读取当前稳定位置（返回 false 表示数据无效或未连接）。
- `bool isConnected() const` — 查询连接状态。
- `uint16_t getReconnectCount() const` — 获取重连计数，用于诊断。

数据与时序要点
- 接收包（Host -> 板）：7 字节
  - header1 = 0xAA
  - header2 = 0xBB
  - position1, position2, position3, position4 （0x00 表示 OFF，非零视为 ON）
  - checksum = sum(bytes[0..5]) & 0xFF
- 确认包（板 -> Host）：4 字节，格式 `0xCC 0xDD status checksum`，checksum = sum(first3) & 0xFF。
- 去抖：默认 `DEBOUNCE_FRAMES = 3`，需要连续 3 帧同一值才被视为稳定。
- 看门狗超时：默认 `WATCHDOG_TIMEOUT = 500 ms`。若超过该时间未接收到有效包，模块视为断连并尝试重连。
- 重连间隔：`RECONNECT_RETRY_INTERVAL = 200 ms`。

线程与中断注意
- `RxEventCallback` 由 HAL UART/DMA 回调触发，属于中断/回调上下文。该函数会更新内部状态并发送 ACK（当前实现中直接调用 `HAL_UART_Transmit`，注意这在某些 HAL 实现中可能阻塞或不适合 ISR 中调用）。
- `getPositions()` 直接拷贝内部 `stable_positions_` 到提供的数组；为避免与回调并发读写导致竞态，建议在读取时使用 FreeRTOS 临界区：`taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`，或把状态通过队列/事件转交到任务处理。

推荐使用模式（示例流程）
1. 在串口 `huartX` 初始化完成后创建并初始化 `DataTransceiver`：`transceiver.init(&huartX);`
2. 创建一个 FreeRTOS 周期性任务，每 50~200 ms 调用 `transceiver.process()`（推荐 100 ms）。
3. 在任务中使用临界区调用 `getPositions()` 并根据变化触发业务逻辑（不要在 ISR 中执行耗时操作）。

示例包（HEX）
- positions = [1,0,1,0]
  - payload: AA BB 01 00 01 00
  - checksum = (AA + BB + 01 + 00 + 01 + 00) & FF = 0x67
  - 完整包: AA BB 01 00 01 00 67
- ACK (status=0x00): CC DD 00 A9

故障排查要点（板内模块角度）
- `getPositions()` 返回 false：检查 `isConnected()`、串口/ DMA 是否启动、Host 是否在 500 ms 内发送了有效包。
- 位置更新滞后：去抖需连续 3 帧，确认 Host 是否连续发送多帧。
- 读取不一致（竞态）：在读取时使用 `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` 或改为通过队列订阅状态变化。
- 重连计数持续增长：表示未收到有效包或有串口错误（帧错误/溢出）；使用 `getReconnectCount()` 诊断并检查串口配置与物理连接。

可选增强（建议）
- 将 ACK 发送改为在任务上下文完成（如使用 DMA 发送或把要发的数据放入发送队列），避免在 ISR 中阻塞。
- 在接收到新稳定状态时，用 FreeRTOS 队列或事件把状态发布给消费者任务，避免轮询和临界区。

示例代码参考 `Core/Src/transceiver_user_example.cpp`。


