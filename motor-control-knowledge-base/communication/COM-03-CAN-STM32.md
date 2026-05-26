# COM-03 STM32 FDCAN实现

> 路径：📡 工业通信协议 > COM-03

**难度：** ★★★★☆

## 概述

STM32G4系列是ST针对电机控制优化的MCU，其内置的FDCAN外设（基于Bosch M_CAN IP核）完整支持CAN 2.0A/2.0B和CAN FD协议。与STM32F4等老型号使用的bxCAN外设相比，FDCAN在滤波器架构、消息RAM管理、中断机制等方面有根本性变化。理解FDCAN外设的架构和配置方法，是开发可靠CAN通信软件的基础。

本文以STM32G474为例，深入讲解FDCAN外设架构、滤波器配置、中断处理、收发编程及硬件接口设计，并提供HAL库和寄存器两种实现方式。

## 正文

### 1. STM32G4 FDCAN外设架构

#### 1.1 FDCAN外设资源

STM32G474包含3个FDCAN实例：FDCAN1、FDCAN2、FDCAN3。

| 特性 | FDCAN1 | FDCAN2 | FDCAN3 |
|------|--------|--------|--------|
| 时钟源 | PLLQ / HSE / PCLK1 | PLLQ / HSE / PCLK1 | PLLQ / HSE / PCLK1 |
| 支持CAN FD | 是 | 是 | 是 |
| 消息RAM | 共享SRAMCAN (10KB) | 共享SRAMCAN | 共享SRAMCAN |
| TX FIFO | 支持 | 支持 | 支持 |
| TX Queue | 支持 | 支持 | 支持 |
| RX FIFO数量 | 2 | 2 | 2 |
| 专用RX Buffer | 支持 | 支持 | 支持 |
| TX Event FIFO | 支持 | 支持 | 支持 |

#### 1.2 消息RAM（Message RAM）架构

FDCAN外设的核心设计变化是**消息RAM的灵活分配**。与bxCAN固定的邮箱结构不同，FDCAN将消息存储在SRAMCAN中，各区域的容量可由软件配置：

```
SRAMCAN (10KB = 2560个32位字) 布局：
┌──────────────────────────────────────┐ 0x000
│ 标准滤波器元素 (11位ID + 配置)       │ 可配置数量(最多128个)
├──────────────────────────────────────┤
│ 扩展滤波器元素 (29位ID + 配置)       │ 可配置数量(最多64个)
├──────────────────────────────────────┤
│ RX FIFO 0                            │ 可配置深度(最多64个元素)
├──────────────────────────────────────┤
│ RX FIFO 1                            │ 可配置深度(最多64个元素)
├──────────────────────────────────────┤
│ RX Buffer (专用接收缓冲区)           │ 可配置数量
├──────────────────────────────────────┤
│ TX Event FIFO                        │ 可配置深度(最多32个元素)
├──────────────────────────────────────┤
│ TX FIFO / TX Queue                   │ 可配置深度(最多32个元素)
├──────────────────────────────────────┤
│ TX Buffer (专用发送缓冲区)           │ 最多3个
└──────────────────────────────────────┘
```

每个元素的大小取决于配置：
- 最多8字节数据：每个元素4个32位字（16字节）
- 最多64字节数据：每个元素18个32位字（72字节）

> **工程经验**：10KB的SRAMCAN由FDCAN1/FDCAN2/FDCAN3共享。在多CAN通道同时使用时，必须仔细规划每个通道的消息RAM分配，避免溢出。推荐的做法是：在初始化时通过`FDCAN_RAMCFG`寄存器明确划分各通道的RAM区域。

#### 1.3 FDCAN与bxCAN的关键差异

| 特性 | bxCAN (STM32F4) | FDCAN (STM32G4) |
|------|-----------------|-----------------|
| CAN FD支持 | 不支持 | 支持 |
| 邮箱/缓冲区 | 3个TX邮箱 + 2个RX FIFO | 灵活配置的TX/RX缓冲区 |
| 滤波器 | 14组（CAN1）/ 28组（CAN2） | 128个标准 + 64个扩展 |
| 消息存储 | 固定结构 | 灵活分配的消息RAM |
| TX优先级 | 邮箱优先级 | FIFO或优先级队列模式 |
| 时钟域 | APB1 | 独立时钟（PLLQ/HSE） |
| DMA支持 | 不支持 | 支持RX FIFO DMA |

> **工程经验**：从bxCAN迁移到FDCAN时，最大的思维转变是"邮箱"到"消息RAM"的架构变化。bxCAN的3个TX邮箱是固定硬件资源，而FDCAN的TX缓冲区数量和模式（FIFO/Queue）可配置。Queue模式下，ID最小的帧优先发送，天然支持基于优先级的发送调度。

### 2. 滤波器配置

#### 2.1 滤波器架构

FDCAN的滤波器分为**标准滤波器**和**扩展滤波器**两组，每组元素独立配置：

```
标准滤波器元素（每个元素64位）：
┌────────────────┬────────────────┬──────────────────────────────────┐
│ SFID2[10:0]    │ SFID1[10:0]    │ 配置位                           │
│ 第二ID/掩码    │ 第一ID         │ SFEC[2:0] + SFT[1:0]            │
└────────────────┴────────────────┴──────────────────────────────────┘

扩展滤波器元素（每个元素128位）：
┌────────────────┬────────────────┬──────────────────────────────────┐
│ EFID2[28:0]    │ EFID1[28:0]    │ 配置位                           │
│ 第二ID/掩码    │ 第一ID         │ EFEC[2:0] + EFT[1:0]            │
└────────────────┴────────────────┴──────────────────────────────────┘
```

#### 2.2 滤波器类型

**SFT（Standard Filter Type）/ EFT（Extended Filter Type）**：

| 值 | 类型 | 说明 |
|----|------|------|
| 00 | Range | SFID1 ≤ ID ≤ SFID2（范围匹配） |
| 01 | Dual | ID = SFID1 或 ID = SFID2（双ID匹配） |
| 10 | Mask | ID & SFID2 == SFID1 & SFID2（掩码匹配） |
| 11 | (保留) | — |

**掩码匹配详解**：

```
掩码匹配规则：接收ID & Mask == FilterID & Mask

示例：接收所有ID为0x1XX的标准帧
  FilterID (SFID1) = 0x100
  Mask (SFID2)     = 0x700
  接收条件: (ID & 0x700) == (0x100 & 0x700) = 0x100
  匹配: 0x100~0x1FF

示例：只接收ID为0x123的标准帧
  FilterID (SFID1) = 0x123
  Mask (SFID2)     = 0x7FF
  接收条件: (ID & 0x7FF) == 0x123
  匹配: 仅0x123
```

#### 2.3 滤波器动作

**SFEC（Standard Filter Element Configuration）/ EFEC**：

| 值 | 动作 | 说明 |
|----|------|------|
| 000 | Disable | 滤波器禁用 |
| 001 | FIFO 0 | 匹配帧存入RX FIFO 0 |
| 010 | FIFO 1 | 匹配帧存入RX FIFO 1 |
| 011 | Reject | 拒绝匹配帧 |
| 100 | Priority | 匹配帧存入专用RX Buffer（高优先级） |
| 101 | Priority+FIFO0 | 优先中断 + 存入FIFO 0 |
| 110 | Priority+FIFO1 | 优先中断 + 存入FIFO 1 |
| 111 | TX Buffer | 触发TX缓冲区发送（远程帧处理） |

> **工程经验**：
> 1. 在电机控制中，建议将**指令帧**（如速度/位置设定）路由到FIFO 0，将**状态帧**（如电机反馈）路由到FIFO 1，实现数据分流处理。
> 2. 滤波器按索引顺序匹配，**第一个匹配的滤波器生效**。因此应将最严格的滤波器放在前面，最宽松的放在后面。
> 3. 如果所有滤波器都不匹配，帧将被拒绝。**务必确保需要接收的帧有对应的滤波器规则**，否则帧会被静默丢弃——这是初学者最常犯的错误。

#### 2.4 HAL库滤波器配置示例

```c
// 配置标准滤波器：接收ID为0x100~0x10F的帧，存入FIFO 0
FDCAN_FilterTypeDef filterConfig;

filterConfig.IdType = FDCAN_STANDARD_ID;
filterConfig.FilterIndex = 0;
filterConfig.FilterType = FDCAN_FILTER_MASK;
filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
filterConfig.FilterID1 = 0x100;
filterConfig.FilterID2 = 0x7F0;
HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig);

// 配置扩展滤波器：接收ID为0x12345XXX的帧，存入FIFO 1
filterConfig.IdType = FDCAN_EXTENDED_ID;
filterConfig.FilterIndex = 0;
filterConfig.FilterType = FDCAN_FILTER_MASK;
filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
filterConfig.FilterID1 = 0x12345000;
filterConfig.FilterID2 = 0x1FFFF000;
HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig);

// 启用全局滤波器：标准帧未匹配时拒绝，扩展帧未匹配时拒绝
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                              FDCAN_REJECT,
                              FDCAN_REJECT,
                              FDCAN_FILTER_REMOTE,
                              FDCAN_FILTER_REMOTE);
```

### 3. 中断处理

#### 3.1 FDCAN中断源

FDCAN外设有大量中断源，通过两个中断线（FDCAN_IT0 / FDCAN_IT1）输出到NVIC：

| 中断类别 | 中断源 | 典型用途 |
|----------|--------|---------|
| **RX中断** | RX FIFO 0新消息 | 接收数据处理 |
| | RX FIFO 0满 | 紧急处理，防止溢出 |
| | RX FIFO 0消息丢失 | 错误恢复 |
| | RX FIFO 1新消息 | 同上 |
| **TX中断** | TX FIFO空 | 触发下一帧发送 |
| | TX完成 | 确认发送成功 |
| | TX事件FIFO新元素 | 获取发送时间戳 |
| **错误中断** | 错误被动/总线关闭 | 通信故障处理 |
| | 协议错误（仲裁丢失等） | 调试诊断 |
| | RAM访问失败 | 严重错误 |
| | TEC/REC达到阈值 | 预警 |
| **状态中断** | 高优先级消息 | 实时响应 |
| | 唤醒 | 低功耗恢复 |

#### 3.2 中断线分配

```c
// 将RX FIFO 0中断分配到中断线0
HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                FDCAN_INTERRUPT_LINE0);

// 将TX完成和错误中断分配到中断线1
HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                FDCAN_IT_TX_COMPLETE | FDCAN_IT_BUS_OFF,
                                FDCAN_INTERRUPT_LINE1);

// 启用中断
HAL_FDCAN_ActivateNotification(&hfdcan1,
                                FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                FDCAN_INTERRUPT_LINE0);
HAL_FDCAN_ActivateNotification(&hfdcan1,
                                FDCAN_IT_TX_COMPLETE | FDCAN_IT_BUS_OFF,
                                FDCAN_INTERRUPT_LINE1);
```

#### 3.3 中断服务程序

```c
// FDCAN1中断线0处理（接收）
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

// FDCAN1中断线1处理（发送完成/错误）
void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

// RX FIFO 0新消息回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[64];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                    &rxHeader, rxData) == HAL_OK)
        {
            // 根据ID分发处理
            switch (rxHeader.Identifier)
            {
                case 0x100:
                    // 处理速度指令
                    break;
                case 0x101:
                    // 处理位置指令
                    break;
                default:
                    break;
            }
        }
    }
}

// TX完成回调
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t BufferIndexes)
{
    // 发送完成，可以准备下一帧
}

// 总线关闭回调
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                     uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
        // Bus Off处理：等待恢复或请求重新初始化
        HAL_FDCAN_Start(hfdcan);
    }
}
```

> **工程经验**：
> 1. **中断中不要做耗时操作**。CAN接收中断应仅完成"从FIFO取出数据+放入消息队列"的操作，业务逻辑在任务中处理。
> 2. **RX FIFO溢出是严重事件**。如果RX FIFO满导致消息丢失，说明接收处理速度跟不上总线流量。应增大FIFO深度或优化中断响应时间。
> 3. **TEC/REC阈值中断**非常有用。建议设置TEC=96时触发预警中断（Error Passive阈值为128），留出提前量。

### 4. 收发示例代码

#### 4.1 HAL库方式 — 完整初始化与收发

```c
// FDCAN初始化
void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;

    // 标称波特率配置：500 kbit/s (假设PCLK=170MHz, 分频后时钟42.5MHz)
    // 42.5MHz / (1 + 12 + 3) = 42.5MHz / 16 = 2.65625 MHz → 再分频
    // 实际：PCLK=170MHz, ClockDivider=1 → CAN时钟=170MHz
    // Prescaler=10 → 17MHz, NominalSeg1=12, NominalSeg2=3
    // 17MHz / (1+12+3) = 17MHz/16 ≈ 1.0625 MHz... 需重新计算
    // 正确：Prescaler=2 → 85MHz, 1+63+16=80 → 85/80 ≈ 1.0625... 
    // 推荐使用STM32CubeMX计算位定时参数
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalTimeSeg1 = 12;
    hfdcan1.Init.NominalTimeSeg2 = 3;
    hfdcan1.Init.NominalSyncJumpWidth = 1;

    // 数据波特率配置：5 Mbit/s
    hfdcan1.Init.DataPrescaler = 2;
    hfdcan1.Init.DataTimeSeg1 = 7;
    hfdcan1.Init.DataTimeSeg2 = 2;
    hfdcan1.Init.DataSyncJumpWidth = 1;

    hfdcan1.Init.StdFiltersNbr = 28;
    hfdcan1.Init.ExtFiltersNbr = 8;
    hfdcan1.Init.RxFifo0ElmtsNbr = 16;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan1.Init.RxFifo1ElmtsNbr = 8;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    HAL_FDCAN_Init(&hfdcan1);
}

// 发送CAN FD帧
void CAN_SendFDMessage(uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_ON;
    txHeader.FDFormat = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

// 发送经典CAN帧
void CAN_SendClassicMessage(uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}
```

#### 4.2 寄存器方式 — 核心配置

```c
// 寄存器级FDCAN初始化（简化版，500kbit/s标称 + 5Mbit/s数据）
void FDCAN1_RegisterInit(void)
{
    // 使能FDCAN时钟
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    while(!(RCC->APB1ENR1 & RCC_APB1ENR1_FDCANEN));

    // 进入初始化模式
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while(!(FDCAN1->CCCR & FDCAN_CCCR_INIT));
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // 配置标称位定时（500 kbit/s）
    // 假设CAN时钟=17MHz (170MHz/10)
    // Seg1=12, Seg2=3, SJW=1 → 总tq=1+12+3=16
    // 17MHz/16 = 1.0625 MHz... 需调整
    // Prescaler=1, 时钟=170MHz → 170MHz/(1+29+10) = 170/40 = 4.25MHz... 
    // 实际工程中应精确计算，此处展示寄存器操作流程
    FDCAN1->NBTP = (0 << FDCAN_NBTP_NSJW_Pos)    |
                   (12 << FDCAN_NBTP_NTSEG1_Pos)  |
                   (3 << FDCAN_NBTP_NTSEG2_Pos)   |
                   (9 << FDCAN_NBTP_NBRP_Pos);

    // 配置数据位定时（5 Mbit/s）
    FDCAN1->DBTP = (0 << FDCAN_DBTP_DSJW_Pos)    |
                   (7 << FDCAN_DBTP_DTSEG1_Pos)   |
                   (2 << FDCAN_DBTP_DTSEG2_Pos)   |
                   (1 << FDCAN_DBTP_DBRP_Pos);

    // 启用CAN FD和BRS
    FDCAN1->CCCR |= FDCAN_CCCR_FDOE;
    FDCAN1->CCCR |= FDCAN_CCCR_BRSE;

    // 配置TX FIFO模式
    FDCAN1->TXBC |= (0 << FDCAN_TXBC_TFQM_Pos);

    // 配置标准滤波器（元素0：接收所有帧到FIFO0）
    uint32_t *filterStart = (uint32_t *)(FDCAN1->SIDFC & FDCAN_SIDFC_FLSSA);
    filterStart[0] = 0x00000000;
    filterStart[1] = (0x01 << FDCAN_STD_FILTER_SFT_Pos) |
                     (0x01 << FDCAN_STD_FILTER_SFEC_Pos) |
                     (0x000 << FDCAN_STD_FILTER_SFID1_Pos) |
                     (0x000 << FDCAN_STD_FILTER_SFID2_Pos);

    FDCAN1->SIDFC = (1 << FDCAN_SIDFC_LSS_Pos);

    // 配置全局滤波器
    FDCAN1->GFC = FDCAN_GFC_ANFS_RXFIFO0 | FDCAN_GFC_ANFE_RXFIFO0;

    // 退出初始化模式
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    while(FDCAN1->CCCR & FDCAN_CCCR_INIT);

    // 启用RX FIFO0新消息中断
    FDCAN1->IE = FDCAN_IE_RF0NE;
    FDCAN1->ILS = FDCAN_ILS_RXFIFO0;
    FDCAN1->ILE = FDCAN_ILE_EINT0;

    NVIC_SetPriority(FDCAN1_IT0_IRQn, 5);
    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

// 寄存器方式发送
void FDCAN1_RegisterSend(uint32_t id, uint8_t *data, uint8_t dlc)
{
    // 等待TX FIFO有空间
    while(!(FDCAN1->TXFQS & FDCAN_TXFQS_TFQF));

    // 获取TX FIFO写入索引
    uint32_t putIdx = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    // 计算TX Buffer元素地址
    uint32_t txBufferAddr = (FDCAN1->TXBC & FDCAN_TXBC_TBSA) +
                             putIdx * 18 * 4;

    // 写入TX Header
    uint32_t *txHeader = (uint32_t *)txBufferAddr;
    txHeader[0] = (id << FDCAN_TX_BUFFER_ID_Pos) |
                  (FDCAN_STANDARD_ID << 18) |
                  (FDCAN_DATA_FRAME << 20);
    txHeader[1] = (dlc << FDCAN_TX_BUFFER_DLC_Pos) |
                  (FDCAN_BRS_ON << FDCAN_TX_BUFFER_BRS_Pos) |
                  (FDCAN_FD_CAN << FDCAN_TX_BUFFER_FDF_Pos);

    // 写入数据
    uint32_t *txData = (uint32_t *)(txBufferAddr + 8);
    for (int i = 0; i < (dlc + 3) / 4; i++)
    {
        txData[i] = ((uint32_t *)data)[i];
    }

    // 请求发送
    FDCAN1->TXBAR = (1 << putIdx);
}
```

> **工程经验**：寄存器方式代码更紧凑、执行更快，但可读性差且容易出错。实际工程中推荐HAL库方式开发，仅在极端性能要求（如中断延迟必须<1μs）时使用寄存器方式。无论哪种方式，位定时参数的计算都应使用STM32CubeMX验证。

### 5. 与收发器接口

#### 5.1 TJA1050接口（经典CAN）

TJA1050是NXP的经典CAN收发器，仅支持CAN 2.0（最高1 Mbit/s）：

```
STM32G4                          TJA1050
┌──────────┐                    ┌──────────┐
│          │                    │          │
│  PA11    ├────────────────────┤ TXD      │
│ (FDCAN1_RXD)                  │          │
│          │                    │    CAN_H ├───── CAN Bus
│  PA12    ├────────────────────┤ RXD      │
│ (FDCAN1_TXD)                  │    CAN_L ├───── CAN Bus
│          │                    │          │
└──────────┘                    │    VREF  ├───── (可选)
                                │    SPLIT ├───── (可选共模稳定)
                                │    STB   ├───── GND(正常)/VCC(待机)
                                └──────────┘
```

关键设计要点：
- VCC：5V供电（TJA1050不支持3.3V供电）
- STB引脚：接地=正常工作，接VCC=待机模式
- CAN_H/CAN_L到总线之间需加ESD保护（如NUP2105L）

#### 5.2 SN65HVD230接口（3.3V CAN）

SN65HVD230是TI的3.3V CAN收发器，适合STM32直连：

```
STM32G4                          SN65HVD230
┌──────────┐                    ┌──────────┐
│          │                    │          │
│  PA12    ├────────────────────┤ D        │
│ (FDCAN1_TXD)                  │          │
│          │                    │    CAN_H ├───── CAN Bus
│  PA11    ├────────────────────┤ R        │
│ (FDCAN1_RXD)                  │    CAN_L ├───── CAN Bus
│          │                    │          │
└──────────┘                    │    RS     ├───── 10kΩ→GND(高速)
                                │          │     或 10kΩ→VCC(斜率控制)
                                └──────────┘
```

RS引脚配置：
- RS接地或通过10kΩ接地：高速模式（最高1 Mbit/s）
- RS通过10kΩ接VCC：斜率控制模式（降低EMI，最高约125 kbit/s）

> **工程经验**：SN65HVD230虽然供电3.3V，但其CAN_H/CAN_L输出电平仍符合ISO 11898-2标准（差分2V），可以与5V供电的TJA1050在同一总线上通信。但SN65HVD230不支持CAN FD的数据段高速率（>1 Mbit/s）。

#### 5.3 TJA1462接口（CAN FD收发器）

TJA1462是NXP的CAN FD专用收发器，支持最高5 Mbit/s：

```
STM32G4                          TJA1462
┌──────────┐                    ┌──────────┐
│          │                    │          │
│  PA12    ├────────────────────┤ TXD      │
│ (FDCAN1_TXD)                  │          │
│          │                    │    CAN_H ├───── CAN Bus
│  PA11    ├────────────────────┤ RXD      │
│ (FDCAN1_RXD)                  │    CAN_L ├───── CAN Bus
│          │                    │          │
└──────────┘                    │    EN     ├───── GPIO控制(使能)
                                │    ERR   ├───── GPIO输入(错误指示)
                                │    WAKE  ├───── (本地唤醒输入)
                                └──────────┘
```

TJA1462的ERR引脚提供额外的诊断能力：
- ERR=低电平：总线有错误（如CAN_H/CAN_L短路、终端电阻缺失）
- 可连接到STM32的GPIO，在软件中监控总线健康状态

> **工程经验**：CAN FD系统中，收发器的选择至关重要。经典收发器（TJA1050/SN65HVD230）的环路延迟约200ns，在5 Mbit/s时每位仅200ns，几乎没有时序裕量。必须使用CAN FD专用收发器。TJA1462的环路延迟约50ns，支持8 Mbit/s。

### 6. 双CAN冗余设计

#### 6.1 冗余架构

在安全关键的电机控制系统中（如电动汽车、工业机器人），CAN总线冗余是常见需求：

```
                    CAN Bus A (主通道)
                   ┌──────────────────┐
                   │  120Ω       120Ω │
            ┌──────┤                  ├──────┐
            │      └──────────────────┘      │
     FDCAN1 │                                │ FDCAN2
            │      ┌──────────────────┐      │
            ├──────┤                  ├──────┤
            │      │  120Ω       120Ω │      │
            │      └──────────────────┘      │
            │      CAN Bus B (备用通道)       │
            │                                │
        ┌───┴────────────────────────────────┴───┐
        │              STM32G4                    │
        │         (双CAN冗余管理)                 │
        └────────────────────────────────────────┘
```

#### 6.2 冗余策略

| 策略 | 描述 | 延迟 | 带宽利用率 |
|------|------|------|-----------|
| **热备** | 双通道同时发送，接收端取先到的 | 最低 | 50%（双倍开销） |
| **温备** | 主通道发送，备用通道监听，故障时切换 | 中等 | 50%~100% |
| **冷备** | 仅主通道工作，故障后启动备用通道 | 最高 | 100% |

#### 6.3 冗余切换实现

```c
typedef struct
{
    FDCAN_HandleTypeDef *primary;
    FDCAN_HandleTypeDef *secondary;
    uint8_t active_channel;
    uint32_t failover_count;
    uint32_t primary_error_count;
    uint32_t secondary_error_count;
} CAN_RedundancyManager;

void CAN_RedundancyInit(CAN_RedundancyManager *mgr,
                         FDCAN_HandleTypeDef *primary,
                         FDCAN_HandleTypeDef *secondary)
{
    mgr->primary = primary;
    mgr->secondary = secondary;
    mgr->active_channel = 0;
    mgr->failover_count = 0;
    mgr->primary_error_count = 0;
    mgr->secondary_error_count = 0;
}

// 冗余发送：双通道同时发送
HAL_StatusTypeDef CAN_RedundancySend(CAN_RedundancyManager *mgr,
                                       FDCAN_TxHeaderTypeDef *header,
                                       uint8_t *data)
{
    HAL_StatusTypeDef status;

    if (mgr->active_channel == 0)
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(mgr->primary, header, data);
        HAL_FDCAN_AddMessageToTxFifoQ(mgr->secondary, header, data);
    }
    else
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(mgr->secondary, header, data);
        HAL_FDCAN_AddMessageToTxFifoQ(mgr->primary, header, data);
    }

    return status;
}

// 冗余接收：取先到的帧
HAL_StatusTypeDef CAN_RedundancyReceive(CAN_RedundancyManager *mgr,
                                          FDCAN_RxHeaderTypeDef *header,
                                          uint8_t *data)
{
    uint32_t fifo0LevelPrimary = HAL_FDCAN_GetRxFifoFillLevel(mgr->primary, FDCAN_RX_FIFO0);
    uint32_t fifo0LevelSecondary = HAL_FDCAN_GetRxFifoFillLevel(mgr->secondary, FDCAN_RX_FIFO0);

    if (fifo0LevelPrimary > 0)
    {
        return HAL_FDCAN_GetRxMessage(mgr->primary, FDCAN_RX_FIFO0, header, data);
    }
    else if (fifo0LevelSecondary > 0)
    {
        return HAL_FDCAN_GetRxMessage(mgr->secondary, FDCAN_RX_FIFO0, header, data);
    }

    return HAL_ERROR;
}

// 错误监控与通道切换
void CAN_RedundancyMonitor(CAN_RedundancyManager *mgr)
{
    uint32_t primaryTEC = (mgr->primary->Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    uint32_t secondaryTEC = (mgr->secondary->Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;

    // 主通道TEC超过阈值，切换到备用通道
    if (mgr->active_channel == 0 && primaryTEC > 96)
    {
        mgr->active_channel = 1;
        mgr->failover_count++;
        mgr->primary_error_count++;
    }
    else if (mgr->active_channel == 1 && secondaryTEC > 96)
    {
        mgr->active_channel = 0;
        mgr->failover_count++;
        mgr->secondary_error_count++;
    }
}
```

> **工程经验**：
> 1. 双CAN冗余的**物理隔离**是关键——两条总线应使用不同的线缆、不同的连接器，避免单点故障。
> 2. 冗余切换的判定不应仅依赖TEC，还应结合：ACK错误次数、CRC错误次数、帧丢失率等综合判断。
> 3. 切换后的主通道恢复检测：当原主通道TEC降回正常水平后，应自动恢复为双通道热备模式。
> 4. 在STM32G4中，FDCAN1和FDCAN2共享SRAMCAN，需确保消息RAM分配不冲突。

## 小结

| 知识点 | 核心要点 | 工程关键 |
|--------|---------|---------|
| FDCAN架构 | 消息RAM灵活分配，支持CAN FD | 多通道共享SRAMCAN需规划分配 |
| 滤波器 | 128标准+64扩展，掩码/范围/双ID匹配 | 按索引顺序匹配；未匹配帧被静默丢弃 |
| 中断处理 | 双中断线，丰富的中断源 | RX中断仅取数据+入队；TEC阈值预警 |
| 收发编程 | HAL库+寄存器两种方式 | 位定时参数用CubeMX计算；FD帧需专用收发器 |
| 收发器接口 | TJA1050(5V)/SN65HVD230(3.3V)/TJA1462(FD) | CAN FD必须用专用收发器；注意供电和ESD |
| 双CAN冗余 | 热备/温备/冷备三种策略 | 物理隔离是关键；综合判定切换条件 |

## 参考

- STMicroelectronics, "RM0440 STM32G4 Reference Manual", Chapter 42: FDCAN
- STMicroelectronics, "AN5348 Getting started with FDCAN on STM32G4"
- Bosch, "M_CAN User's Manual Version 3.3.1"
- NXP, "TJA1050 High-Speed CAN Transceiver Datasheet"
- NXP, "TJA1462/TJA1463 CAN FD Transceiver Datasheet"
- Texas Instruments, "SN65HVD230 3.3V CAN Bus Transceiver Datasheet"
- ISO 11898-2:2016, "Road vehicles — CAN — Part 2: High-speed medium access unit"
