# DShot Beacon 需求文档

> 基于 Betaflight AT32F4 分支代码分析 (`src/main/io/beeper.*`, `src/main/drivers/dshot_command.*`, `src/main/fc/core.c`, `src/main/flight/failsafe.c`, `src/main/pg/beeper.*`)

---

## 1. 概述

DShot Beacon 是利用 DShot 协议向 ESC（电调）发送特定命令，使电机发出蜂鸣音的功能。它作为传统物理蜂鸣器（beeper）的替代或补充，用于在丢失飞行器时定位。

---

## 2. 架构层次

```
┌─────────────────────────────────────────────────────────────────┐
│  触发源 (Trigger Sources)                                       │
│  ├─ failsafe.c:   RX_LOST / RX_LOST_LANDING                    │
│  ├─ core.c:       BOXBEEPERON / BOXHEADADJ (RX_SET)           │
│  └─ core.c:       BOXBEEPERMUTE (静音)                         │
├─────────────────────────────────────────────────────────────────┤
│  beeper.c: beeper() → 设置 currentBeeperEntry (按优先级)        │
│  beeper.c: beeperUpdate() @100Hz → DShot Beacon 输出决策        │
├─────────────────────────────────────────────────────────────────┤
│  dshot_command.c: dshotCommandWrite() → 命令入队                 │
│  dshot_command.c: dshotCommandOutputIsEnabled() → 同步输出       │
├─────────────────────────────────────────────────────────────────┤
│  ESC (电调): 接收 DShot 帧 → 解析 BEACON 命令 → 电机发声        │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Beeper 模式定义

### 3.1 所有 Beeper 模式 (beeperMode_e)

| 枚举值 | 名称 | 优先级 | 说明 |
|--------|------|--------|------|
| 0 | BEEPER_SILENCE | - | 静音 |
| 1 | BEEPER_GYRO_CALIBRATED | 0 | 陀螺仪校准完成 |
| 2 | BEEPER_RX_LOST | 1 | **★ 接收机信号丢失 (DShot Beacon 支持)** |
| 3 | BEEPER_RX_LOST_LANDING | 2 | 信号丢失自动降落 (SOS) |
| 4 | BEEPER_DISARMING | 3 | 解锁 |
| 5 | BEEPER_ARMING | 4 | 加锁 |
| 6 | BEEPER_ARMING_GPS_FIX | 5 | 加锁 + GPS 定位 |
| 7 | BEEPER_BAT_CRIT_LOW | 7 | 电池严重低电压 |
| 8 | BEEPER_BAT_LOW | 8 | 电池低电压 |
| 9 | BEEPER_GPS_STATUS | 9 | GPS 卫星数播报 |
| 10 | BEEPER_RX_SET | 10 | **★ AUX 通道触发蜂鸣 (DShot Beacon 支持)** |
| 11 | BEEPER_ACC_CALIBRATION | 11 | 加速度计校准完成 |
| 12 | BEEPER_ACC_CALIBRATION_FAIL | 12 | 加速度计校准失败 |
| 13 | BEEPER_READY_BEEP | 13 | GPS 就绪 |
| 14 | BEEPER_MULTI_BEEPS | 14 | 多蜂鸣 (内部使用) |
| 15 | BEEPER_DISARM_REPEAT | 15 | 重复解锁音 |
| 16 | BEEPER_ARMED | 16 | 已加锁提醒 |
| 17 | BEEPER_SYSTEM_INIT | 17 | 系统初始化 |
| 18 | BEEPER_USB | 18 | USB 连接 |
| 19 | BEEPER_BLACKBOX_ERASE | 19 | 黑匣子擦除 |
| 20 | BEEPER_CRASH_FLIP_MODE | 20 | 翻摔模式 |
| 21 | BEEPER_CAM_CONNECTION_OPEN | 21 | 摄像头连接打开 |
| 22 | BEEPER_CAM_CONNECTION_CLOSE | 22 | 摄像头连接关闭 |
| 23 | BEEPER_RC_SMOOTHING_INIT_FAIL | 23 | RC 平滑初始化失败 |
| 24 | BEEPER_ALL | 24 | 全部模式开关 |

### 3.2 DShot Beacon 支持的模式

DShot Beacon **仅**在以下两种模式下输出（定义见 `DSHOT_BEACON_ALLOWED_MODES`）：

```c
#define DSHOT_BEACON_ALLOWED_MODES ( \
    BEEPER_GET_FLAG(BEEPER_RX_LOST) \   // bit 1
    | BEEPER_GET_FLAG(BEEPER_RX_SET) )  // bit 10
```

| 模式 | 触发场景 | 物理蜂鸣器行为 | DShot Beacon 行为 |
|------|----------|---------------|-------------------|
| BEEPER_RX_LOST | 接收机信号丢失 | `beep_txLostBeep[]`: 500ms/500ms 交替 | 发送 DShot Beacon 命令到电机 |
| BEEPER_RX_SET | AUX 开关激活蜂鸣 | `beep_shortBeep[]`: 100ms/100ms 短鸣 | 发送 DShot Beacon 命令到电机 |

**其他所有 beeper 模式（ARMING, DISARMING, BAT_LOW 等）均不会触发 DShot Beacon。**

---

## 4. DShot Beacon 命令

### 4.1 可用命令

```c
typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,    // 1 - 默认值
    DSHOT_CMD_BEACON2,    // 2
    DSHOT_CMD_BEACON3,    // 3
    DSHOT_CMD_BEACON4,    // 4
    DSHOT_CMD_BEACON5,    // 5
    // ... 其他命令
} dshotCommands_e;
```

- **5 种可选音调**：BEACON1 ~ BEACON5（对应 DShot 命令值 1~5）
- **默认值**：`dshotBeaconTone = 1` → `DSHOT_CMD_BEACON1`
- **配置参数** (CLI)：`beeper_dshot_beacon_tone`，范围 [1, DSHOT_CMD_BEACON5]
- **有效性校验** (`config.c`)：若配置值超出 [BEACON1, BEACON5] 范围，自动重置为 `DSHOT_CMD_BEACON1`

### 4.2 命令发送参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 发送目标 | `ALL_MOTORS` | 所有电机 |
| 电机数量 | `getMotorCount()` | 配置的电机总数 |
| 命令类型 | `DSHOT_CMD_TYPE_INLINE` | 内联发送（与正常电机信号交替） |
| 重复次数 | `1` | 只发送一次 |
| 命令后延迟 | `DSHOT_BEEP_DELAY_US = 100000µs (100ms)` | 发送后的等待时间 |

注意区别：其他 DShot 命令（如 SPIN_DIRECTION）需要 `repeats = 10`；Beacon 命令只需 `repeats = 1`。

---

## 5. 时间间隔参数

### 5.1 核心时间常量

```c
// 来源: beeper.h
#define DSHOT_BEACON_GUARD_DELAY_US   1200000  // 1.2秒 - Beacon与加锁/解锁之间的保护间隔
#define DSHOT_BEACON_MODE_INTERVAL_US  450000  // 450ms - RX_SET Beacon最小间隔
#define DSHOT_BEACON_RXLOSS_INTERVAL_US 950000 // 950ms - RX_LOST Beacon最小间隔
```

### 5.2 实际生效间隔

由于 `beeperUpdate()` 以 **100Hz** 周期调度（`TASK_PERIOD_HZ(100)`，即每 10ms 调用一次）：

| 参数 | 定义值 | 实际值（取整到10ms边界） |
|------|--------|-------------------------|
| GUARD_DELAY | 1,200,000µs (1.2s) | ~1.2s |
| MODE_INTERVAL (RX_SET) | 450,000µs (450ms) | ~500ms |
| RXLOSS_INTERVAL (RX_LOST) | 950,000µs (950ms) | ~1,000ms |

### 5.3 DShot 命令队列内部延迟

```
命令入队 → IDLEWAIT (等待电机空闲) → STARTDELAY (起始延迟) → ACTIVE (发送命令) → POSTDELAY (命令后延迟)
```

| 阶段 | 延迟 | 说明 |
|------|------|------|
| 起始延迟 | `DSHOT_INITIAL_DELAY_US = 10,000µs (10ms)` | 命令序列开始前的初始延迟 |
| 命令间隔 | `DSHOT_COMMAND_DELAY_US = 1,000µs (1ms)` | 重复发送之间的间隔（Beacon 不重复） |
| 命令后延迟 | `DSHOT_BEEP_DELAY_US = 100,000µs (100ms)` | Beacon 命令后的等待 |

---

## 6. DShot Beacon 输出条件

### 6.1 必要条件（ALL 必须满足）

在 `beeperUpdate()` 中，DShot Beacon 命令发送的**完整条件链**如下：

```
条件1: USE_DSHOT 编译宏已定义
  AND
条件2: !areMotorsRunning()              ← 电机未运行
  AND
条件3: currentBeeperEntry->mode == BEEPER_RX_SET
        OR
        currentBeeperEntry->mode == BEEPER_RX_LOST
  AND
条件4: cmpTimeUs(currentTimeUs, getLastDisarmTimeUs()) > DSHOT_BEACON_GUARD_DELAY_US
         ← 距上次解锁超过 1.2 秒
  AND
条件5: !isTryingToArm()                 ← 未在尝试加锁
  AND
条件6: cmpTimeUs(currentTimeUs, lastDshotBeaconCommandTimeUs) > dShotBeaconInterval
         ← 距上次 Beacon 超过最小间隔
         (RX_SET: 450ms, RX_LOST: 950ms)
```

### 6.2 条件详解

#### 条件2: `areMotorsRunning()` 判定

```c
bool areMotorsRunning(void) {
    if (ARMING_FLAG(ARMED))           // 已加锁 → 电机运行
        return true;
    for (每个电机) {
        if (motor_disarmed[i] != disarmMotorOutput)  // 任一电机非空闲值 → 运行
            return true;
    }
    return false;
}
```

- **已加锁（ARMED）**：电机一定在运行 → 不发送 Beacon
- **已解锁（DISARMED）但是**：有电机输出不为空闲值（如 flipOverAfterCrash 等场景）→ 不发送 Beacon
- **已解锁且所有电机空闲**：允许发送 Beacon

#### 条件4: Guard Delay 保护

- **来源**：`getLastDisarmTimeUs()` 返回最后一次 `disarm()` 的时刻
- **目的**：防止 DShot Beacon 命令与 spin direction 命令（用于 turtle mode）产生冲突
- 解锁后 1.2 秒内 **不发送** Beacon，给 spin direction 命令留出时间窗口

#### 条件5: `isTryingToArm()` 判定

```c
bool isTryingToArm() {
    return (tryingToArm != ARMING_DELAYED_DISARMED);
}
```

- `tryingToArm` 状态值：
  - `ARMING_DELAYED_DISARMED = 0`：空闲（not trying to arm）
  - `ARMING_DELAYED_NORMAL = 1`：等待正常加锁
  - `ARMING_DELAYED_CRASHFLIP = 2`：等待 crash flip 加锁
  - `ARMING_DELAYED_LAUNCH_CONTROL = 3`：等待 launch control 加锁

---

## 7. 加锁时 Beacon 互斥机制

### 7.1 加锁延迟逻辑 (`tryArm()`)

```
用户请求加锁 (ARM switch)
  │
  ├─ 检查: 距上次 Beacon 命令是否 < 1.2秒 ?
  │   ├─ YES → 延迟加锁，设置 tryingToArm = ARMING_DELAYED_*
  │   │        不执行加锁，return
  │   │        下一次 tryArm() 调用时重新检查
  │   │
  │   └─ NO  → 继续正常加锁流程：
  │             1. 发送 DShot spin direction 命令（如有 flipOverAfterCrash）
  │             2. 发送 DShot EDT 使能命令（如适用）
  │             3. 设置 ARMED 标志
  │             4. 发送 BEEPER_ARMING 蜂鸣
  │             5. 重置 tryingToArm
```

### 7.2 OSD 警告显示

在 `osd_warnings.c` 中，当加锁被 Beacon 延迟时：

- Beacon 保护期前 0.5 秒：显示 `" BEACON ON"`
- 之后：显示 `"ARM IN X.X"` 倒计时（单位：秒，1位小数）

```c
int armingDelayTime = (getLastDshotBeaconCommandTimeUs() + DSHOT_BEACON_GUARD_DELAY_US - currentTimeUs) / 1e5;
if (armingDelayTime >= (DSHOT_BEACON_GUARD_DELAY_US / 1e5 - 5)) {
    tfp_sprintf(warningText, " BEACON ON");      // 前0.5秒
} else {
    tfp_sprintf(warningText, "ARM IN %d.%d", ...); // 倒计时
}
```

---

## 8. 触发场景

### 8.1 BEEPER_RX_LOST（接收机信号丢失）

**触发位置**：`src/main/flight/failsafe.c`

```c
if (!receivingRxData && (armed || ARMING_FLAG(WAS_EVER_ARMED))) {
    beeperMode = BEEPER_RX_LOST;
}
...
if (beeperMode != BEEPER_SILENCE) {
    beeper(beeperMode);  // 每帧调用一次
}
```

- **条件**：接收机无有效数据 **且** (已加锁 **或** 曾被加锁过)
- **触发时机**：failsafe 状态机每帧检查
- 注意：`BEEPER_RX_LOST`（优先级=1）会抢占大部分其他蜂鸣

### 8.2 BEEPER_RX_SET（AUX 开关触发）

**触发位置**：`src/main/fc/core.c` (多处)

```c
// 场景1: BOXBEEPERON 开关激活
if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
    beeper(BEEPER_RX_SET);
}

// 场景2: BOXHEADADJ 模式下设置头方向
if (imuQuaternionHeadfreeOffsetSet()) {
    beeper(BEEPER_RX_SET);
}
```

### 8.3 BEEPER_RX_LOST_LANDING（SOS 模式）

```c
// 在 FAILSAFE_LANDING 或 FAILSAFE_GPS_RESCUE 状态中
if (armed) {
    beeperMode = BEEPER_RX_LOST_LANDING;  // SOS 序列
}
```

- 此模式**不**触发 DShot Beacon（不在 `DSHOT_BEACON_ALLOWED_MODES` 中）
- 仅通过物理蜂鸣器输出 SOS 莫尔斯电码

---

## 9. 静音/禁用机制

### 9.1 Beeper 静音条件

当以下任一条件满足时，`beeper(BEEPER_SILENCE)` 被调用，**所有蜂鸣（包括物理蜂鸣器和 DShot Beacon）停止**：

1. **模式为 BEEPER_SILENCE**：直接静音
2. **USB 供电且 BEEPER_USB 在 off_flags 中**：`beeperConfig()->beeper_off_flags & BEEPER_GET_FLAG(BEEPER_USB) && getBatteryState() == BATTERY_NOT_PRESENT`
3. **BOXBEEPERMUTE RC 模式激活**：`IS_RC_MODE_ACTIVE(BOXBEEPERMUTE)`

> 注意：这与 DShot Beacon 的 `dshotBeaconOffFlags` 不同，此处禁用的是整个蜂鸣系统。

### 9.2 DShot Beacon 独立禁用

通过 `dshotBeaconOffFlags` 位掩码控制：

```c
typedef struct beeperConfig_s {
    uint32_t beeper_off_flags;      // 物理蜂鸣器模式开关
    uint8_t dshotBeaconTone;        // Beacon 音调选择 (1-5)
    uint32_t dshotBeaconOffFlags;   // DShot Beacon 模式开关
} beeperConfig_t;
```

- **CLI 设置命令**：`beacon` (类似 `beeper` 命令)
  - `beacon` → 显示已禁用的模式
  - `beacon list` → 显示可用模式 (RX_LOST, RX_SET)
  - `beacon -RX_SET` → 禁用 RX_SET 的 Beacon
  - `beacon RX_SET` → 启用 RX_SET 的 Beacon
- **默认值**：`dshotBeaconOffFlags = DSHOT_BEACON_ALLOWED_MODES`（所有 DShot Beacon 模式默认启用）
- **有效性校验**：若 `dshotBeaconOffFlags` 包含非 DShot Beacon 模式位，重置为 0
- **MSP 接口**：`MSP_BEEPER_CONFIG` / `MSP_SET_BEEPER_CONFIG` 支持读写

### 9.3 物理蜂鸣器与 DShot Beacon 的分离

物理蜂鸣器输出**不受** `dshotBeaconOffFlags` 影响。在 `beeperUpdate()` 中：

```c
// DShot Beacon 输出 (受 dshotBeaconOffFlags 控制？→ 否！)
// 注意：DShot Beacon 的输出检查不包含 dshotBeaconOffFlags
// 它由条件6（间隔）隐性控制 + beeperMode 的 off_flags 控制
```

实际上，DShot Beacon 在 `beeperUpdate()` 中的输出**受物理蜂鸣器的 `beeper_off_flags` 间接控制**：
- 因为 `beeper()` 函数中检查了 `beeperConfig()->beeper_off_flags`
- 如果模式（如 BEEPER_RX_SET）在 beeper_off_flags 中被禁用，`currentBeeperEntry` 不会被设置为该模式
- 因此 DShot Beacon 的条件3 (`currentBeeperEntry->mode == BEEPER_RX_SET`) 不满足

换句话说：**物理蜂鸣器的 `beeper_off_flags` 会同时禁用物理蜂鸣器和 DShot Beacon。**

---

## 10. DShot 协议要求

### 10.1 协议启用条件

```c
case DSHOT_CMD_TYPE_INLINE:
    ret = dshotStreamingCommandsAreEnabled();
    break;
```

`dshotStreamingCommandsAreEnabled()` 要求：
1. `motorIsEnabled()` → 电机已使能
2. `motorGetMotorEnableTimeMs() != 0` → 电机使能时间已记录
3. `millis() > motorGetMotorEnableTimeMs() + DSHOT_PROTOCOL_DETECTION_DELAY_MS`
   → 距电机使能超过 **3000ms** (3秒协议检测延迟)

### 10.2 协议检查

```c
if (!isMotorProtocolDshot() || !dshotCommandsAreEnabled(commandType) 
    || (command > DSHOT_MAX_COMMAND) || dshotCommandQueueFull()) {
    return;  // 不发送命令
}
```

- 仅 DShot 协议支持
- 命令值必须在 [0, 47] 范围内
- 命令队列未满（最多 3 个待发送命令）
- 若为 INLINE 类型，必须满足 streaming 条件

### 10.3 命令同步

DShot 命令输出与 PID 循环（电机输出）同步：
- 通过 `dshotSetPidLoopTime()` 获知 PID 循环周期（默认 125µs @ 8KHz）
- `dshotCommandOutputIsEnabled()` 在每个 PID 循环中被调用
- 按 PID 循环计数计算延迟：`dshotCommandCyclesFromTime(delayUs)`

---

## 11. 命令队列机制

### 11.1 队列结构

```
commandQueue[DSHOT_MAX_COMMANDS + 1] = commandQueue[4]  (4个槽位，最多3个命令)
           │
           ├─ [commandQueueTail]: 当前正在处理的命令
           ├─ [commandQueueTail+1]: 下一个待处理命令
           └─ [commandQueueTail+2]: 第三个待处理命令
```

### 11.2 队列状态机

```
IDLEWAIT ──(电机空闲)──▶ STARTDELAY ──(初始延迟)──▶ ACTIVE ──(发送完成)──▶ POSTDELAY ──(后续延迟)──▶ 出队
   ▲                                                                                                    │
   └────────────────────────────────── 队列为空 ◀─────────────────────────────────────────────────────────┘
```

### 11.3 Beacon 命令处理流程

```
dshotCommandWrite(ALL_MOTORS, motorCount, DSHOT_CMD_BEACONx, DSHOT_CMD_TYPE_INLINE)
  │
  ├─ 创建命令控制块:
  │   .repeats = 1
  │   .delayAfterCommandUs = 100000  (DSHOT_BEEP_DELAY_US)
  │   .command[i] = DSHOT_CMD_BEACONx  (目标电机)
  │   .command[j] = DSHOT_CMD_MOTOR_STOP (非目标电机)
  │
  ├─ 检查电机空闲:
  │   ├─ 空闲 → 状态 = STARTDELAY, 延迟 = 10ms
  │   └─ 不空闲 → 状态 = IDLEWAIT
  │
  └─ dshotCommandOutputIsEnabled() 在每个PID循环被调用:
      │
      ├─ IDLEWAIT: 等待 allMotorsAreIdle()
      ├─ STARTDELAY: 等待 10ms 初始延迟
      ├─ ACTIVE: 发送 1 次 DShot 帧（含 BEACON 命令值）
      └─ POSTDELAY: 等待 100ms 后完成
```

---

## 12. 配置参数汇总

### 12.1 编译时宏

| 宏 | 用途 |
|-----|------|
| `USE_DSHOT` | 启用 DShot 协议支持（必须） |
| `USE_BEEPER` | 启用蜂鸣器功能（必须） |
| `USE_DSHOT_TELEMETRY` | DShot 遥测（Blocking 模式需要） |

### 12.2 运行时参数

| 参数 | CLI 名称 | 类型 | 范围 | 默认值 | 说明 |
|------|----------|------|------|--------|------|
| `dshotBeaconTone` | `beeper_dshot_beacon_tone` | uint8 | [1, 5] | 1 | Beacon 音调选择 |
| `dshotBeaconOffFlags` | (通过 `beacon` 命令设置) | uint32 位掩码 | RX_LOST, RX_SET | `DSHOT_BEACON_ALLOWED_MODES` | Beacon 模式开关 |
| `beeper_off_flags` | (通过 `beeper` 命令设置) | uint32 位掩码 | ALL modes | 0 | 物理蜂鸣器模式开关 |

### 12.3 CLI 命令

| 命令 | 示例 | 说明 |
|------|------|------|
| `beacon` | `beacon` | 显示已禁用的 Beacon 模式 |
| `beacon list` | `beacon list` | 列出所有支持的 Beacon 模式 |
| `beacon -RX_SET` | `beacon -RX_SET` | 禁用 RX_SET 的 DShot Beacon |
| `beacon RX_LOST` | `beacon RX_LOST` | 启用 RX_LOST 的 DShot Beacon |
| `beeper_dshot_beacon_tone` | `set beeper_dshot_beacon_tone = 3` | 设置 Beacon 音调为 3 号 |
| `beeper` | `beeper -RX_LOST` | 禁用物理蜂鸣器的 RX_LOST |

---

## 13. 完整事件时序

### 13.1 正常接收机信号丢失场景

```
时间T0: 接收机信号丢失
  │
  ├─ failsafe 状态机检测到 !receivingRxData
  │   → beeper(BEEPER_RX_LOST)
  │
  ├─ beeperUpdate() @100Hz:
  │   检查条件:
  │   ├─ !areMotorsRunning() → 电机未运行（已加锁状态下 = false，跳过）
  │   │   → 只走物理蜂鸣器路径（beep_txLostBeep 500ms/500ms 交替）
  │   │
  │   └─ 若已解锁:
  │       ├─ 距上次解锁 > 1.2s ?
  │       ├─ !isTryingToArm() ?
  │       └─ 距上次 Beacon > 950ms ?
  │           → 发送 DShot Beacon 命令
  │           → 更新 lastDshotBeaconCommandTimeUs = currentTimeUs
  │
  ├─ ~1000ms后 (实际由于100Hz调度 ≈1000ms):
  │   再次发送 DShot Beacon
  │
  └─ 循环直到:
      ├─ 接收机信号恢复 (failsafe恢复)
      │   → beeper(BEEPER_SILENCE) 或 切换到其他模式
      └─ 或飞行器加锁 (ARMED)
          → areMotorsRunning() = true，不再发送Beacon
```

### 13.2 AUX 开关触发场景

```
用户激活 BOXBEEPERON (或 BOXHEADADJ)
  │
  ├─ core.c: beeper(BEEPER_RX_SET)
  │
  ├─ beeperUpdate() @100Hz:
  │   检查条件:
  │   ├─ !areMotorsRunning() → 电机必须未运行
  │   ├─ 距上次解锁 > 1.2s ?
  │   ├─ !isTryingToArm() ?
  │   └─ 距上次 Beacon > 450ms ?
  │       → 发送 DShot Beacon
  │
  ├─ ~500ms后:
  │   再次发送 DShot Beacon
  │
  └─ 循环直到:
      └─ 用户关闭 BOXBEEPERON 开关
          → beeperMode 不再是 BEEPER_RX_SET
          → 停止发送 Beacon
```

### 13.3 加锁延迟场景

```
用户在 Beacon 刚发送后立即请求加锁 (Beacon ↔ ARM 互斥)
  │
时间T0: lastDshotBeaconCommandTimeUs (刚发送)
  │
用户: ARM switch ON
  │
  ├─ tryArm():
  │   cmpTimeUs(currentTimeUs, lastDshotBeaconCommandTimeUs) < 1.2s ?
  │   → YES: 延迟加锁!
  │   → 设置 tryingToArm = ARMING_DELAYED_*
  │   → return (不加锁)
  │
  ├─ OSD 显示: " BEACON ON" (前0.5s)
  │             "ARM IN 0.7" ... "ARM IN 0.1"
  │
  ├─ ~1.2s后:
  │   tryArm():
  │   cmpTimeUs(...) >= 1.2s → 正常加锁
  │   → ARMED
  │   → areMotorsRunning() = true
  │   → Beacon 自动停止
```

---

## 14. 关键设计约束

1. **Beacon 仅由 RX_LOST 和 RX_SET 触发** — 其他所有蜂鸣模式不会产生 DShot Beacon
2. **电机运行时永不发送 Beacon** — 防止干扰飞行控制信号
3. **解锁后 1.2 秒内不发送 Beacon** — 为 spin direction / turtle mode 命令保留窗口
4. **Beacon 发送中不允许加锁** — 加锁需等待距上次 Beacon ≥ 1.2 秒
5. **加锁过程中不发送 Beacon** — `!isTryingToArm()` 条件
6. **RX_LOST Beacon 间隔 ≈1 秒**，RX_SET Beacon 间隔 ≈0.5 秒 — 给 ESC 足够时间播放音调
7. **Beacon 命令仅发送 1 次**（不重复），与 spin direction 命令的 10 次重复不同
8. **DShot 协议必须已启用 + 检测完成（3 秒延迟）** — 才允许内联命令
9. **`beeper_off_flags` 间接控制 DShot Beacon** — 因为禁用某模式后 `currentBeeperEntry` 不会设为该模式
10. **`BOXBEEPERMUTE` 同时禁用物理蜂鸣器和 DShot Beacon**

---

## 15. 相关源文件索引

| 文件 | 内容 |
|------|------|
| `src/main/io/beeper.h` | 模式枚举、时间常量、DSHOT_BEACON_ALLOWED_MODES |
| `src/main/io/beeper.c` | 核心蜂鸣逻辑、DShot Beacon 输出决策、蜂鸣序列定义 |
| `src/main/drivers/dshot_command.h` | DShot 命令枚举、命令类型枚举 |
| `src/main/drivers/dshot_command.c` | 命令队列管理、命令输出同步 |
| `src/main/pg/beeper.h` | beeperConfig_t 参数组定义 |
| `src/main/pg/beeper.c` | 参数组注册和默认值 |
| `src/main/pg/beeper_dev.h` | 蜂鸣器硬件设备配置 |
| `src/main/drivers/sound_beeper.h` | BEEP_ON/BEEP_OFF 硬件接口宏 |
| `src/main/fc/core.c` | 加锁/解锁逻辑、tryArm() Beacon 互斥、RX_SET 触发 |
| `src/main/fc/tasks.c` | BEEPER 任务调度定义 (100Hz) |
| `src/main/flight/failsafe.c` | failsafe 状态机、RX_LOST 触发 |
| `src/main/flight/mixer_init.c` | areMotorsRunning() 实现 |
| `src/main/osd/osd_warnings.c` | Beacon 导致的加锁延迟 OSD 警告 |
| `src/main/cli/cli.c` | `beacon` CLI 命令、`processBeeperCommand()` |
| `src/main/cli/settings.c` | `beeper_dshot_beacon_tone` 参数定义 |
| `src/main/config/config.c` | 参数有效性校验 |
| `src/main/msp/msp.c` | MSP 协议读写 beeperConfig |
