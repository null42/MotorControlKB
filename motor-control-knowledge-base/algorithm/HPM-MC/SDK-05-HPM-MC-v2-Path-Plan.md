# HPM-MC v2 路径规划模块深度解析 ★★★★☆ 🔰📚🔧

**文档版本：** v1.0
**生成日期：** 2026-05-23
**适用对象：** 电机控制工程师、伺服系统开发者
**前置知识：** C语言编程、电机控制基础、运动学基本概念

---

## 目录

1. [概述](#1-概述)
2. [梯形速度曲线](#2-梯形速度曲线)
3. [S曲线规划](#3-s曲线规划)
4. [配置结构体与API](#4-配置结构体与api)
5. [与控制环的协调](#5-与控制环的协调)
6. [弱磁控制关联](#6-弱磁控制关联)

---

## 1. 概述

### 1.1 路径规划在电机控制中的作用

路径规划（Path Planning）位于控制环的最外层，负责在**位置环之前**生成平滑的运动轨迹，是伺服控制中承上启下的关键模块。

```
┌───────────┐    ┌───────────────┐    ┌───────────┐    ┌───────────┐    ┌──────────┐
│ Path Plan │───▶│ Position Loop │───▶│Speed Loop │───▶│CurrentLoop│───▶│  Motor   │
│  (本模块)  │    │ (位置环)       │    │ (速度环)   │    │ (电流环)   │    │          │
└───────────┘    └───────────────┘    └───────────┘    └───────────┘    └──────────┘
      ↑
  目标位置/速度
```

### 1.2 为何需要路径规划

| 问题 | 无路径规划 | 有路径规划 |
|------|-----------|-----------|
| **加速度管理** | 阶跃目标导致冲击（Jerk∞） | 限制加速度，保护机械结构 |
| **失步风险** | 快速变速可能导致电机失步 | 柔和过渡，跟随能力更强 |
| **运动平滑性** | 速度突变，运行粗糙 | 连续速度剖面，低噪音低振动 |
| **可预测性** | 难以估算运动时间 | 精确给出到达时间的运动轨迹 |

**核心理念：** 将用户给定的离散目标点，转化为连续、受物理约束（最大加速度、最大速度）约束的时域轨迹。

### 1.3 v2 独有特性

v2 的路径规划模块与 v1 相比具有以下特征：

- **双缓冲曲线更新：** 支持 `current`/`next` 两级缓冲，在一条曲线执行期间即可预装载下一条曲线，实现无缝衔接
- **极对数感知：** 直接使用 `mcl_cfg_t.physical.motor.pole_num`，将机械角度增量自动转换为电角度增量
- **角度闭环折叠：** 使用 `MCL_ANGLE_MOD_X` 将累积角度限幅到 `[0, 2π)`，避免浮点溢出
- **与 Hybrid 控制深度集成：** 规划位置直接作为 `hybrid_ctrl` 的 `q_des` 输入

---

## 2. 梯形速度曲线

### 2.1 运动阶段分解

梯形速度曲线是最经典的工程速度剖面，分为**三个阶段**：

```
速度 │
     │    ┌──────────────────┐
     │   ╱                    ╲
     │  ╱                      ╲
     │ ╱                        ╲
     │╱                          ╲
     └──────────────────────────────▶ 时间
        ├── acc ──┼── const ──┼── dec ──┤
        t0        t1          t2
     (加速段)    (匀速段)      (减速段)
```

**各阶段数学描述（机械角度域）：**

**加速段** `0 ≤ t < t_acc`：

```
a(t) = +acc_max                         （恒加速）
v(t) = acc_max · t
θ(t) = 0.5 · acc_max · t²
```

**匀速段** `t_acc ≤ t < t_acc + t_const`：

```
a(t) = 0                                （加速度为零）
v(t) = v_max = acc_max · t_acc
θ(t) = θ_acc + v_max · (t - t_acc)
```

**减速段** `t_acc + t_const ≤ t ≤ t_total`：

```
a(t) = -dec_max                         （恒减速）
v(t) = v_max - dec_max · (t - t1)
θ(t) = θ_const + v_max · (t - t1) - 0.5 · dec_max · (t - t1)²
```

### 2.2 离散化与预计算

由于路径规划在 **PWM 中断上下文中周期性调用**（周期 `loop_ts`），算法需要以离散步长执行。源码在 `hpm_mcl_path_t_cure_generate()` 首次调用（`ts == 0`）时完成全部预计算：

```
t0 = acc_time                                            (加速持续时间)
t2 = dec_time                                            (减速持续时间)  
t1 = total_time - t0 - t2                                (匀速持续时间)

num0 = ⌊ t0 / loop_ts ⌋                                  (加速段步数)
num1 = ⌊ t1 / loop_ts ⌋                                  (匀速段步数)
num2 = ⌊ t2 / loop_ts ⌋                                  (减速段步数)

t0_counter = num0                                        (第一拐点)
t1_counter = num0 + num1                                 (第二拐点)
t2_counter = num0 + num1 + num2                          (终点)
```

**预计算插值系数：**

```
a0 = acc · loop_ts · 0.5                                 (加速段系数)
a1 = (speed · t1) / num1  ≈ speed · loop_ts             (匀速段系数)
a2 = dec · loop_ts · 0.5                                 (减速段系数)
```

这些系数在每次调用 `generate` 时直接查用，无需重复计算乘除法。

### 2.3 逐周期位置增量计算

源码中对三个阶段的处理使用 `if-else` 分支（`ts` 已在当前调用中自增）：

```c
/* 加速段: ts ∈ [1, t0_counter] */
theta_inc = a0 × (loop_ts × ts × 2 - loop_ts)
/* 展开：θ_inc = 0.5 × acc × loop_ts² × (2×ts - 1) */

/* 匀速段: ts ∈ (t0_counter, t1_counter] */
theta_inc = a1  (= speed × loop_ts)

/* 减速段: ts ∈ (t1_counter, t2_counter] */
theta_inc = a2 × (loop_ts × (t2_counter - ts) × 2 - loop_ts)
/* 展开：θ_inc = 0.5 × dec × loop_ts² × (2×(t2-ts) - 1) */
```

**物理含义：** 每个 `theta_inc` 是一个 PWM 周期内电机转过的**机械角度**（弧度）。随后乘以 `pole_num` 转换为电角度，再累加入 `theta_next`。

### 2.4 角度累积与归一化

```c
path->theta.next = MCL_ANGLE_MOD_X(0, MCL_PI * 2, theta_acc)
```

作用：将电角度折叠到 `[0, 2π)` 范围。避免了因长期累积导致浮点精度丢失或溢出。这同时也意味着：**如果目标位置超过一圈，theta 会自动绕回** — 这适用于持续旋转场景。

### 2.5 双缓冲切换机制

```
         ┌──────────────┐         ┌──────────────┐
用户推送 ▶│    next      │────────▶│   current    │──▶ generate() 使用
         │ (预装载区)    │ 自动切换 │ (执行区)      │
         └──────────────┘         └──────────────┘
```

- `hpm_mcl_path_update_t_cure()` 优先写入 `current`（若为空），否则写入 `next`
- 当 `current` 曲线执行完毕（`ts > t2_counter`），自动将 `next` 搬移至 `current`，置 `next_empty = true`
- 若 `next` 也为空，则置 `current_empty = true`，输出保持最后位置不变
- `hpm_mcl_path_t_cure_complete()` 仅在**两个缓冲区均为空**时返回 `true`

### 2.6 轨迹参数约束关系

要使梯形曲线能够正确规划，以下约束必须满足：

```
t_total = t_acc + t_const + t_dec
t_acc = v_max / acc                        (加速到最高速所需时间)
t_dec = v_max / dec                        (从最高速减速到零所需时间)

总位移（机械弧度）：
Δθ = 0.5·acc·t_acc² + v_max·t_const + v_max·t_dec - 0.5·dec·t_dec²
```

若 `Δθ` 过小（目标太近），系统可能无法达到 `v_max`，曲线退化为**三角形**（加速后直接减速，无匀速段）。当前 v2 实现中，用户需自行保证参数合理性，模块内部不做校验。

---

## 3. S曲线规划

### 3.1 S曲线的优势

S曲线在梯形曲线基础上进一步限制了**加加速度（Jerk）**，使得加速度本身也连续变化，消除了加速度突变带来的机械冲击：

```
        梯形曲线                    S曲线
    加速度                        加速度
      │                            │
      │  ┌──┐                       │    ╱‾‾‾╲
      │  │  │                       │   ╱     ╲
      │  │  │                       │  ╱       ╲
      │──┘  └──                     │─╱         ╲──
      └──────────▶ t               └───────────────▶ t
    (不连续跳变)                  (连续变化，无突变)
```

典型七段S曲线：

```
速度 │
     │         ┌───────────────┐
     │        ╱                 ╲
     │       ╱                   ╲
     │      ╱                     ╲
     │     ╱                       ╲
     │    ╱                         ╲
     │   ╱                           ╲
     │  ╱                             ╲
     └─────────────────────────────────────▶ 时间
      │1│ 2 │ 3 │   4   │ 5 │ 6 │ 7 │
      加  匀  减   匀   加  匀  减
      加  加  加   速   减  减  减
      速  速  速        速  速  速
```

### 3.2 v2 中的实现状态

**当前 v2（截止本版分析）仅实现梯形速度曲线，未实现 S 曲线。** `path_plan_t_cure_cfg_t` 中的 `t_cure` 命名即表示 trapezoidal curve，没有 jerk 相关参数。

若未来引入 S 曲线，需要增加：
- `jerk` 参数（加加速度，单位 rad/s³）
- 七阶段时间分段计算
- 三次多项式（而非二次）的位置插值

在需要平滑启停的应用中（如机器人关节），可通过降低 `acc`/`dec` 值来近似获得柔和效果，但其本质仍是梯形曲线。

---

## 4. 配置结构体与API

### 4.1 结构体总览

```c
/* 梯形曲线参数 */
typedef struct {
    float acc;       /* 加速度, rad/s²                  */
    float dec;       /* 减速度, rad/s²                  */
    float speed;     /* 匀速运行速度, rad/s              */
    float time;      /* 总规划时间, s                    */
    float acc_time;  /* 加速持续时间, s                  */
    float dec_time;  /* 减速持续时间, s                  */
} path_plan_t_cure_cfg_t;

/* 路径规划配置 */
typedef struct {
    path_plan_t_cure_cfg_t t_cure;  /* 默认曲线配置     */
    float loop_ts;                   /* 调用周期, s      */
} mcl_path_plan_cfg_t;

/* 路径规划运行数据 */
typedef struct {
    mcl_path_plan_cfg_t *cfg;        /* 指向配置        */
    int32_t *pole_num;               /* 指向极对数      */
    struct {                         /* 梯形曲线运行态  */
        path_plan_t_cure_cfg_t next;
        path_plan_t_cure_cfg_t current;
        bool next_empty;
        bool current_empty;
        uint32_t ts;                 /* 当前步序号      */
        uint32_t t0;                 /* 加速段结束步    */
        uint32_t t1;                 /* 匀速段结束步    */
        uint32_t t2;                 /* 总步数          */
        float a0;                    /* 加速段系数      */
        float a1;                    /* 匀速段系数      */
        float a2;                    /* 减速段系数      */
    } t_cure;
    struct {
        float next;                  /* 下一周期角度    */
        float last;                  /* 上一周期角度    */
        float current;               /* 当前角度        */
    } theta;
} mcl_path_plan_t;
```

### 4.2 常用 API

| API | 说明 | 调用频率 |
|-----|------|---------|
| `hpm_mcl_path_init(path, cfg, mcl)` | 初始化，关联极对数 | 一次性 |
| `hpm_mcl_path_update_t_cure(path, cfg)` | 推送梯形曲线参数 | 按需（运动指令变更时） |
| `hpm_mcl_path_t_cure_generate(path)` | 执行一步规划计算 | **每 PWM 周期调用** |
| `hpm_mcl_path_t_cure_complete(path)` | 查询曲线是否执行完毕 | 按需轮询 |
| `hpm_mcl_path_get_current_theta(path)` | 获取当前电角度 | 控制环使用 |
| `hpm_mcl_path_get_next_theta(path)` | 获取下一周期电角度 | 控制环使用（前馈） |
| `hpm_mcl_path_get_last_theta(path)` | 获取上一周期电角度 | 速度估算 |

### 4.3 参数建议范围

| 参数 | 建议范围 | 说明 |
|------|---------|------|
| `loop_ts` | 50μs ~ 200μs | 与 PWM 频率匹配（如 20kHz → 50μs） |
| `acc` | 10 ~ 10000 rad/s² | 取决于电机转动惯量和驱动器能力 |
| `dec` | 同 acc | 通常与加速度对称设置 |
| `speed` | 0 ~ 电机额定转速（机械） | 留有余量，不超过额定转速 |
| `time` | ≥ acc_time + dec_time | 需确保匀速段时间非负 |
| `acc_time` | ≤ time | 用户自行保证约束 |

**典型配置示例（2000rpm 电机，极对数 4）：**

```c
mcl_path_plan_cfg_t path_cfg = {
    .t_cure = {
        .acc = 500.0f,       /* 500 rad/s² ≈ 79.6 rps²     */
        .dec = 500.0f,       /* 对称减速                     */
        .speed = 100.0f,     /* 100 rad/s ≈ 955 rpm (机械)  */
        .time = 0.5f,        /* 总时间 500ms                 */
        .acc_time = 0.2f,    /* 加速 200ms                   */
        .dec_time = 0.2f,    /* 减速 200ms，匀速 100ms      */
    },
    .loop_ts = 0.00005f,     /* 50μs，对应 20kHz PWM        */
};
```

---

## 5. 与控制环的协调

### 5.1 控制环调用链路

在 HPM_MCL_v2 架构中，路径规划与控制环的整合发生在 **`hpm_mcl_loop()`** 内部。`mcl_loop_t` 持有 `mcl_path_plan_t *path` 指针，调用层次如下：

```
hpm_mcl_loop()                          /* 主循环，PWM 中断中调用 */
  │
  ├── hpm_mcl_path_t_cure_generate(path)  /* 执行一步路径规划       */
  │
  ├── [若 enable_position_loop]
  │     ├── 从 path 读取 theta → 作为位置环输入（角度模式控制）
  │     └── 位置环输出 → 速度环参考
  │
  ├── [若 enable_speed_loop]
  │     └── 速度环计算 → 电流环参考
  │
  └── [电流环]
        └── FOC 输出 → PWM 驱动
```

### 5.2 与 hybrid_ctrl 的配合（位置-力矩混合控制）

Hybrid 控制是 v2 的新增特性，其控制律为：

```
tau = tau_ff + kp · (q_des - q_actual) + kd · (dq_des - dq_actual)
```

路径规划模块与 `hybrid_ctrl` 的协作方式如下：

```
                 Path Plan
                     │
          ┌──────────┼──────────┐
          │          │          │
          ▼          ▼          ▼
       theta      theta      theta
      .current   .next     .last
          │          │          │
          ▼          ▼          ▼
     mcl_hybrid_ctrl_set_position(q_des)
     mcl_hybrid_ctrl_set_velocity(dq_des)    ← (theta_next - theta_current) / loop_ts
          │
          ▼
     mcl_hybrid_ctrl_step(cfg, state)
          │
          ▼
     tau_output → iq_ref → FOC 电流环
```

**关键点：**
- `q_des` = `hpm_mcl_path_get_next_theta(path)` — 使用 **下一周期** 的角度作为目标，提供一周期前馈
- `dq_des` = `(theta_next - theta_current) / loop_ts` — 从角度差反推速度（用户自行计算，模块不直接输出速度）
- `tau_ff` 可用于重力补偿等场景，由用户按需填入

### 5.3 与普通FOC的配合

在非 hybrid 模式下，路径规划遵循标准三环级联：

```
Path Plan ──▶ Position Loop ──▶ Speed Loop ──▶ Current Loop (FOC) ──▶ Motor

 theta_des      (用户设定或规划输出)     iq_ref                  PWM
```

`mcl_loop_t` 中的 `ref_position` 和 `exec_ref.position` 分别代表用户设定位置与实际执行位置。当 `enable_position_loop = true` 时，位置环误差驱动速度环，形成完整的伺服控制链。

---

## 6. 弱磁控制关联

### 6.1 路径规划层对弱磁的定位

路径规划模块**本身不处理弱磁逻辑**。弱磁控制（Field Weakening）发生在 FOC 电流环层面，通过注入负的 `id` 电流来削弱气隙磁场，从而在基速以上维持恒功率运行。

路径规划模块与弱磁的分工：

```
                       基速以下                  基速以上
                       ────────                  ────────
Path Plan (本模块)：    规划位置/速度轨迹         规划位置/速度轨迹
                        （速度 ≤ 基速）            （速度可 > 基速）

FOC 电流环 + 弱磁：     正常 MTPA 控制            注入负 id，限制端电压
                        id = 0 或 MTPA            id < 0（弱磁电流）
```

### 6.2 高速区策略

路径规划在配置 `speed` 参数时：

- **若 `speed ≤ 基速`（机械）：** 电机全程运行在恒转矩区，无需弱磁介入
- **若 `speed > 基速`：** 进入弱磁区后，位置环/速度环输出的 `iq` 需受弱磁约束限制，且实际加速度能力会随转速升高而下降，此时梯形曲线的 `acc`/`dec` 参数应**低于低速区的设定值**

推荐的保守做法是：

```
弱磁区 acc_max(speed) = 低速区 acc_max × (基速 / speed)
```

该系数随速度升高递减，反映弱磁区转矩能力下降的物理事实。当前 v2 路径规划模块未自动实现此逻辑，需用户在应用层根据速度动态调整 `acc`/`dec` 参数。

```c
/* 应用层示例：根据速度动态调整加速度 */
float scale = (fabsf(current_speed) < base_speed) ? 1.0f
                                                   : (base_speed / fabsf(current_speed));
path_cfg.acc = base_acc * scale;
path_cfg.dec = base_dec * scale;
hpm_mcl_path_update_t_cure(&path, &path_cfg);
```

---

## 7. 路径规划代码实现分析

本章节基于 `hpm_mcl_path_plan.c` / `.h` 源码，对路径规划模块的核心算法进行逐段深度解析，建立"原理公式 → 代码实现 → 工程要点"的完整映射。

### 7.1 梯形速度曲线生成 (`hpm_mcl_path_t_cure_generate`)

#### 7.1.1 完整代码

```c
hpm_mcl_stat_t hpm_mcl_path_t_cure_generate(mcl_path_plan_t *path)
{
    float t0, t1, t2;
    int32_t num0, num1, num2;
    float theta;

    MCL_ASSERT_OPT(path != NULL, mcl_invalid_pointer);
    MCL_ASSERT_OPT(path->cfg->loop_ts != 0, mcl_invalid_argument);
    if (path->t_cure.current_empty) {
        path->t_cure.ts = 0;
        return mcl_success;
    }
    if (path->t_cure.ts == 0) {
        if (path->t_cure.current.time != 0) {
            t0 = path->t_cure.current.acc_time;
            t2 = path->t_cure.current.dec_time;
            t1 = path->t_cure.current.time - t0 - t2;
            num0 = (int32_t)(t0 / path->cfg->loop_ts);
            num1 = (int32_t)(t1 / path->cfg->loop_ts);
            num2 = (int32_t)(t2 / path->cfg->loop_ts);

            path->t_cure.t0 = num0;
            path->t_cure.t1 = num0 + num1;
            path->t_cure.t2 = num0 + num1 + num2;
            path->t_cure.a0 = path->t_cure.current.acc * path->cfg->loop_ts * 0.5f;
            path->t_cure.a1 = ((path->t_cure.current.speed * t1) / num1);
            path->t_cure.a2 = path->t_cure.current.dec * path->cfg->loop_ts * 0.5f;
        } else {
            path->t_cure.t0 = 0;
            path->t_cure.t1 = 0;
            path->t_cure.t2 = 0;
            path->t_cure.a0 = 0;
            path->t_cure.a1 = 0;
            path->t_cure.a2 = 0;
        }
    }
    path->theta.last = path->theta.current;
    path->theta.current = path->theta.next;
    path->t_cure.ts += 1;

    if (path->t_cure.ts > path->t_cure.t2) {
        theta = path->theta.current;
        if (path->t_cure.next_empty == false) {
            path->t_cure.current = path->t_cure.next;
            path->t_cure.next_empty = true;
            path->t_cure.ts = 0;
        } else {
            path->t_cure.current_empty = true;
            path->t_cure.current.time = 0;
            path->t_cure.ts = 0;
        }
    } else if (path->t_cure.ts > path->t_cure.t1) {
        theta = path->t_cure.a2 * (path->cfg->loop_ts *(path->t_cure.t2 - path->t_cure.ts) * 2 - path->cfg->loop_ts);
        theta *= *path->pole_num;
        theta += path->theta.next;
    } else if (path->t_cure.ts > path->t_cure.t0) {
        theta = path->t_cure.a1;
        theta *= *path->pole_num;
        theta += path->theta.next;
    } else {
        theta = path->t_cure.a0 * (path->cfg->loop_ts * path->t_cure.ts * 2 - path->cfg->loop_ts);
        theta *= *path->pole_num;
        theta += path->theta.next;
    }
    path->theta.next = MCL_ANGLE_MOD_X(0, MCL_PI * 2, theta);

    return mcl_success;
}
```

#### 7.1.2 逐段解析

**（一）前置校验与空缓冲处理**

```c
MCL_ASSERT_OPT(path != NULL, mcl_invalid_pointer);
MCL_ASSERT_OPT(path->cfg->loop_ts != 0, mcl_invalid_argument);
if (path->t_cure.current_empty) {
    path->t_cure.ts = 0;
    return mcl_success;
}
```

- 空指针和零周期校验是防御性编程的基本要求
- 若 `current_empty == true`，说明当前无曲线可执行，直接返回，并将步计数器 `ts` 归零（保持干净状态）
- 此时 `theta.next` 保持上一周期的值不变，输出角度恒定——电机"停住"

**（二）预计算阶段（ts == 0 首次调用）**

```c
if (path->t_cure.ts == 0) {
    if (path->t_cure.current.time != 0) {
        t0 = path->t_cure.current.acc_time;
        t2 = path->t_cure.current.dec_time;
        t1 = path->t_cure.current.time - t0 - t2;
        num0 = (int32_t)(t0 / path->cfg->loop_ts);
        num1 = (int32_t)(t1 / path->cfg->loop_ts);
        num2 = (int32_t)(t2 / path->cfg->loop_ts);

        path->t_cure.t0 = num0;
        path->t_cure.t1 = num0 + num1;
        path->t_cure.t2 = num0 + num1 + num2;
        path->t_cure.a0 = path->t_cure.current.acc * path->cfg->loop_ts * 0.5f;
        path->t_cure.a1 = ((path->t_cure.current.speed * t1) / num1);
        path->t_cure.a2 = path->t_cure.current.dec * path->cfg->loop_ts * 0.5f;
    } else {
        /* time == 0 的退化情况：所有参数归零 */
    }
}
```

**加速段步数推导：**

$$num_0 = \left\lfloor \frac{t_{acc}}{T_s} \right\rfloor$$

其中 $T_s$ = `loop_ts`（PWM 周期）。`t0` 存储的是加速段结束的**绝对步序号**，即 `num0`。

**匀速段步数推导：**

$$num_1 = \left\lfloor \frac{t_{const}}{T_s} \right\rfloor = \left\lfloor \frac{t_{total} - t_{acc} - t_{dec}}{T_s} \right\rfloor$$

`t1 = num0 + num1` 是匀速段结束的绝对步序号（第二拐点）。

**减速段步数推导：**

$$num_2 = \left\lfloor \frac{t_{dec}}{T_s} \right\rfloor$$

`t2 = num0 + num1 + num2` 是总步数（终点）。

**插值系数推导：**

- **加速段系数 `a0`：**

$$a_0 = \frac{1}{2} \cdot acc \cdot T_s$$

加速段第 $k$ 步的角度增量公式为：

$$\Delta\theta_{mech}(k) = a_0 \cdot (T_s \cdot k \cdot 2 - T_s) = \frac{1}{2} \cdot acc \cdot T_s^2 \cdot (2k - 1)$$

这与连续域公式 $\theta(t) = \frac{1}{2} \cdot acc \cdot t^2$ 的离散差分一致：第 $k$ 步的增量等于 $\theta(k \cdot T_s) - \theta((k-1) \cdot T_s)$。

- **匀速段系数 `a1`：**

$$a_1 = \frac{speed \cdot t_1}{num_1} \approx speed \cdot T_s$$

这是匀速段每步的机械角度增量。代码未直接使用 `speed * loop_ts`，而是用 `(speed * t1) / num1`，这是因为 `num1 = floor(t1/Ts)` 存在截断误差，用总位移除以实际步数可以**均匀分摊截断误差**，避免终点位置偏差。

- **减速段系数 `a2`：**

$$a_2 = \frac{1}{2} \cdot dec \cdot T_s$$

减速段第 $k$ 步（从终点倒数）的角度增量公式为：

$$\Delta\theta_{mech}(k) = a_2 \cdot (T_s \cdot (t2 - ts) \cdot 2 - T_s) = \frac{1}{2} \cdot dec \cdot T_s^2 \cdot (2(t2 - ts) - 1)$$

减速段使用 `(t2 - ts)` 作为递减计数器，使得增量从大到小递减，与加速段形成镜像对称。

**（三）角度历史更新与步进**

```c
path->theta.last = path->theta.current;
path->theta.current = path->theta.next;
path->t_cure.ts += 1;
```

三级角度移位：`last ← current ← next`，然后 `ts` 自增。注意 `ts` 的自增发生在**角度计算之前**，因此后续分支中 `ts` 的值已经是"当前步"的序号（从 1 开始计数）。

**（四）加速段计算**

```c
} else {
    theta = path->t_cure.a0 * (path->cfg->loop_ts * path->t_cure.ts * 2 - path->cfg->loop_ts);
    theta *= *path->pole_num;
    theta += path->theta.next;
}
```

展开后：

$$\theta_{elec} = \underbrace{\frac{1}{2} \cdot acc \cdot T_s^2 \cdot (2 \cdot ts - 1)}_{\Delta\theta_{mech}} \times p + \theta_{prev}$$

其中 $p$ 为极对数。物理含义：第 `ts` 步的机械角度增量乘以极对数后，叠加到上一周期的电角度上。

**（五）匀速段计算**

```c
} else if (path->t_cure.ts > path->t_cure.t0) {
    theta = path->t_cure.a1;
    theta *= *path->pole_num;
    theta += path->theta.next;
}
```

匀速段每步增量恒定：

$$\theta_{elec} = a_1 \times p + \theta_{prev}$$

**（六）减速段计算**

```c
} else if (path->t_cure.ts > path->t_cure.t1) {
    theta = path->t_cure.a2 * (path->cfg->loop_ts *(path->t_cure.t2 - path->t_cure.ts) * 2 - path->cfg->loop_ts);
    theta *= *path->pole_num;
    theta += path->theta.next;
}
```

展开后：

$$\theta_{elec} = \underbrace{\frac{1}{2} \cdot dec \cdot T_s^2 \cdot (2 \cdot (t2 - ts) - 1)}_{\Delta\theta_{mech}} \times p + \theta_{prev}$$

当 `ts` 逐渐趋近 `t2` 时，`(t2 - ts)` 递减至 0，增量递减至 0，速度平滑归零。

**（七）曲线完成与缓冲切换**

```c
if (path->t_cure.ts > path->t_cure.t2) {
    theta = path->theta.current;
    if (path->t_cure.next_empty == false) {
        path->t_cure.current = path->t_cure.next;
        path->t_cure.next_empty = true;
        path->t_cure.ts = 0;
    } else {
        path->t_cure.current_empty = true;
        path->t_cure.current.time = 0;
        path->t_cure.ts = 0;
    }
}
```

当 `ts > t2` 时，当前曲线已执行完毕。此时 `theta = path->theta.current`（保持最终位置不变），然后进入双缓冲切换逻辑（详见 7.2 节）。

#### 7.1.3 工程要点

| 要点 | 说明 |
|------|------|
| **预计算只做一次** | `ts == 0` 分支仅在曲线起始时执行一次，后续周期直接查表计算，CPU 开销极低 |
| **整数截断处理** | `num1 = (int32_t)(t1 / loop_ts)` 向下取整，`a1 = speed * t1 / num1` 分摊误差 |
| **分支顺序** | `ts > t2` → `ts > t1` → `ts > t0` → else，从后往前判断，优先处理完成/减速 |
| **time == 0 退化** | 当 `time == 0` 时所有系数归零，曲线不产生任何角度增量，等效于"空操作" |
| **极对数乘法位置** | 先计算机械角度增量，再乘极对数转电角度，避免在系数中混入极对数导致含义不清 |

---

### 7.2 双缓冲切换机制

#### 7.2.1 相关代码

**推送曲线参数：**

```c
hpm_mcl_stat_t hpm_mcl_path_update_t_cure(mcl_path_plan_t *path, path_plan_t_cure_cfg_t *cfg)
{
    MCL_ASSERT_OPT(path != NULL, mcl_invalid_pointer);

    if (path->t_cure.current_empty) {
        path->t_cure.current_empty = false;
        path->t_cure.current = *cfg;
    } else if (path->t_cure.next_empty) {
        path->t_cure.next_empty = false;
        path->t_cure.next = *cfg;
    } else {
        return mcl_fail;
    }

    return mcl_success;
}
```

**曲线完成时的切换（在 `hpm_mcl_path_t_cure_generate` 内）：**

```c
if (path->t_cure.ts > path->t_cure.t2) {
    theta = path->theta.current;
    if (path->t_cure.next_empty == false) {
        path->t_cure.current = path->t_cure.next;
        path->t_cure.next_empty = true;
        path->t_cure.ts = 0;
    } else {
        path->t_cure.current_empty = true;
        path->t_cure.current.time = 0;
        path->t_cure.ts = 0;
    }
}
```

**完成查询：**

```c
bool hpm_mcl_path_t_cure_complete(mcl_path_plan_t *path)
{
    if ((path->t_cure.current_empty ==  true) &&
        (path->t_cure.next_empty ==  true)) {
        return true;
    } else {
        return false;
    }
}
```

#### 7.2.2 切换逻辑解析

双缓冲机制的核心是一个两级 FIFO，由 `current_empty` 和 `next_empty` 两个布尔标志驱动：

```
写入端 (hpm_mcl_path_update_t_cure)：
  ┌──────────────────────────────────────────────────┐
  │  current_empty?                                   │
  │    YES → current = *cfg, current_empty = false    │
  │    NO  → next_empty?                              │
  │            YES → next = *cfg, next_empty = false   │
  │            NO  → return mcl_fail (缓冲区满)        │
  └──────────────────────────────────────────────────┘

读取端 (hpm_mcl_path_t_cure_generate, ts > t2 时)：
  ┌──────────────────────────────────────────────────┐
  │  next_empty?                                      │
  │    NO  → current = next, next_empty = true, ts=0  │
  │          (无缝衔接下一条曲线)                       │
  │    YES → current_empty = true, ts = 0             │
  │          (无待执行曲线，进入空闲)                    │
  └──────────────────────────────────────────────────┘
```

**状态转移表：**

| 当前状态 | 写入操作 | 写入后状态 |
|---------|---------|-----------|
| current=空, next=空 | 写入 cfg | current=cfg, next=空 |
| current=占用, next=空 | 写入 cfg | current=占用, next=cfg |
| current=占用, next=占用 | 写入 cfg | **mcl_fail**（拒绝） |

| 当前状态 | 曲线完成 | 完成后状态 |
|---------|---------|-----------|
| current=占用, next=占用 | ts > t2 | current=next, next=空 |
| current=占用, next=空 | ts > t2 | current=空, next=空 |

#### 7.2.3 无锁切换的实现原理

该模块**无需互斥锁或临界区保护**，原因如下：

1. **单写单读模型：** `hpm_mcl_path_update_t_cure()` 由应用任务（或上位机指令回调）调用，`hpm_mcl_path_t_cure_generate()` 由 PWM 中断调用。写入端只修改 `current`/`next` 的内容和 `current_empty`/`next_empty` 标志；读取端只在 `ts > t2` 时修改这些变量
2. **时序互斥：** 切换操作发生在 `generate()` 函数内部，此时已处于 PWM 中断上下文，不会被另一个 `generate()` 调用打断
3. **标志位语义保证：** `current_empty` 和 `next_empty` 的修改是原子的（布尔赋值在 32 位 MCU 上是原子操作），不存在读写撕裂风险

**潜在风险：** 若应用任务在 `generate()` 执行切换的瞬间调用 `update_t_cure()`，可能出现竞态。在 RTOS 环境下，建议在 `update_t_cure()` 调用前后关闭 PWM 中断（或使用 `__disable_irq()` / `__enable_irq()`），确保标志位修改的原子性。当前实现依赖用户在应用层保证调用时序。

#### 7.2.4 工程要点

| 要点 | 说明 |
|------|------|
| **缓冲深度为 2** | 最多预存 1 条曲线，若连续推送第 3 条会被拒绝（返回 `mcl_fail`） |
| **无缝衔接** | `next` 搬移到 `current` 后 `ts` 归零，下一周期立即开始新曲线的预计算，无空闲周期 |
| **完成判断** | `t_cure_complete()` 要求两个缓冲区**均为空**才返回 true，确保预装载曲线也被执行 |
| **角度连续性** | 切换时 `theta = path->theta.current`（保持上一曲线终点），新曲线从该位置继续累加，角度不会跳变 |

---

### 7.3 角度累积与归一化

#### 7.3.1 相关代码

**角度移位与累积（在 `hpm_mcl_path_t_cure_generate` 内）：**

```c
path->theta.last = path->theta.current;
path->theta.current = path->theta.next;
path->t_cure.ts += 1;

/* ... 各段计算 ... */

theta *= *path->pole_num;
theta += path->theta.next;

path->theta.next = MCL_ANGLE_MOD_X(0, MCL_PI * 2, theta);
```

**MCL_ANGLE_MOD_X 宏定义（hpm_mcl_common.h）：**

```c
#define MCL_ANGLE_MOD_X(down, up, val)  \
({    \
    float val_; \
    if ((val) > up) { \
        val_ = (val); \
        do {  \
            val_ = (val_) - (up - down);    \
        } while ((val_) > up); \
    } else if ((val) < down) {    \
        val_ = (val); \
        do {  \
            val_ = (val_) + (up - down);   \
        } while ((val_) < down); \
    } else {    \
        val_ = (val); \
    }   \
    (val_);  \
})
```

#### 7.3.2 角度积分过程解析

路径规划模块的角度更新本质上是**前向欧拉积分**：

$$\theta_{elec}[k+1] = \theta_{elec}[k] + \Delta\theta_{mech}[k] \times p$$

其中：
- $\theta_{elec}[k]$ = `path->theta.next`（上一周期计算的电角度）
- $\Delta\theta_{mech}[k]$ = 当前步的机械角度增量（由梯形曲线公式计算）
- $p$ = `*path->pole_num`（极对数）

**三级角度变量的含义与用途：**

| 变量 | 含义 | 典型用途 |
|------|------|---------|
| `theta.last` | 两个周期前的电角度 | 速度估算：$\omega \approx (\theta_{current} - \theta_{last}) / (2 \cdot T_s)$ |
| `theta.current` | 上一周期的电角度 | 当前控制环使用的角度反馈 |
| `theta.next` | 本周期新计算的电角度 | 下一周期的前馈角度，hybrid 控制的 `q_des` |

**移位操作时序：**

```
周期 k 调用 generate()：
  last    ← current    (k-2 周期角度)
  current ← next       (k-1 周期角度)
  next    ← 新计算值    (k 周期角度)
```

#### 7.3.3 MCL_ANGLE_MOD_X 归一化解析

`MCL_ANGLE_MOD_X(0, 2π, theta)` 的作用是将电角度折叠到 $[0, 2\pi)$ 区间：

$$\theta_{norm} = \theta - \left\lfloor \frac{\theta}{2\pi} \right\rfloor \cdot 2\pi$$

代码实现采用**循环减法**而非取模运算：

```c
if (val > up) {
    do { val_ = val_ - (up - down); } while (val_ > up);
} else if (val < down) {
    do { val_ = val_ + (up - down); } while (val_ < down);
}
```

**为什么用循环减法而非 `fmodf`？**

1. **无库依赖：** 嵌入式环境可能没有完整数学库，循环减法仅用加减法
2. **确定性执行：** `fmodf` 在某些工具链上执行时间不确定，循环减法的迭代次数可预估（通常 1~2 次）
3. **精度：** 避免浮点取模的精度陷阱

**归一化的必要性：**

- **防止浮点溢出：** 若电机持续旋转，`theta` 会无限增大。`float` 的有效精度约 7 位十进制数，当角度累积到 $10^4$ rad 量级时，$2\pi \approx 6.28$ 的增量已无法精确表示，导致角度分辨率丢失
- **三角函数输入约束：** FOC 的 Park/Clarke 变换要求角度在 $[0, 2\pi)$ 内，超出范围虽不报错但可能引入额外计算误差
- **支持连续旋转：** 归一化后角度自动绕回，天然适配连续旋转场景

#### 7.3.4 工程要点

| 要点 | 说明 |
|------|------|
| **先乘极对数再累加** | `theta *= pole_num; theta += theta.next;`——增量先转电角度再叠加，避免机械/电角度混用 |
| **归一化在最后一步** | 累加完成后才归一化，确保累加精度不受中间折叠影响 |
| **循环减法的迭代上限** | 正常运行时每步增量远小于 $2\pi$，循环仅执行 0~1 次；若参数异常（如 `acc` 极大），可能多次迭代 |
| **不支持多圈绝对位置** | 归一化后角度被折叠，无法区分第几圈——适用于连续旋转，不适用于需要多圈绝对定位的场合 |

---

### 7.4 S曲线实现

#### 7.4.1 当前实现状态

**v2 路径规划模块当前未实现 S 曲线。** 源码中不存在任何 jerk（加加速度）相关参数或七段式计算逻辑。`path_plan_t_cure_cfg_t` 结构体的命名 `t_cure`（trapezoidal curve）也明确表明仅支持梯形曲线。

#### 7.4.2 梯形曲线与 S 曲线的差异对比

| 特性 | 梯形曲线（当前实现） | S 曲线（未实现） |
|------|-------------------|-----------------|
| 加速度变化 | 阶跃跳变（不连续） | 连续变化（受 Jerk 限制） |
| 位置曲线 | 二次/一次/二次多项式 | 三次/二次/一次/二次/三次多项式 |
| 阶段数 | 3 段（加速/匀速/减速） | 7 段（加加速/匀加速/减加速/匀速/加减速/匀减速/减减速） |
| 机械冲击 | 加速度突变 → 有限 Jerk → 激振 | Jerk 受限 → 平滑启停 → 低振 |
| 计算复杂度 | 低（3 个系数 + if-else） | 高（7 段参数 + 多项式插值） |
| 参数 | acc, dec, speed, time | acc, dec, speed, time, **jerk** |
| 适用场景 | 通用伺服、风机泵类 | 机器人关节、精密定位、人机协作 |

#### 7.4.3 若需引入 S 曲线的扩展方向

若未来需要实现 S 曲线，需在 `path_plan_t_cure_cfg_t` 中增加 `jerk` 参数，并新增七段式预计算逻辑。核心数学关系为：

$$Jerk = \frac{d^3\theta}{dt^3} = \frac{d\alpha}{dt}$$

加速段的加速度不再阶跃，而是以恒定 Jerk 从 0 线性增长至 $acc_{max}$：

$$\alpha(t) = J \cdot t, \quad 0 \le t \le t_j$$

其中 $t_j = acc_{max} / J$ 为加加速段时间。对应的速度和位置需用三次多项式计算：

$$v(t) = \frac{1}{2} J \cdot t^2, \quad \theta(t) = \frac{1}{6} J \cdot t^3$$

当前 v2 的 `a0`/`a1`/`a2` 三系数体系无法直接扩展为七段式，需要重构预计算结构体。

#### 7.4.4 工程要点

| 要点 | 说明 |
|------|------|
| **降低 acc/dec 可近似柔和效果** | 减小加速度值可降低冲击，但本质仍是梯形曲线，Jerk 仍为无穷大 |
| **S 曲线代价** | 计算量增加约 2~3 倍，RAM 占用增加（7 段参数 + 多项式系数），需评估 MCU 算力 |
| **混合策略** | 可在应用层实现 S 曲线位置序列，再逐点喂给路径规划模块作为目标位置 |

---

### 7.5 路径状态机

#### 7.5.1 状态机代码

v2 的路径规划模块**没有显式的状态枚举**，状态机由 `current_empty`、`next_empty` 和 `ts` 三个变量隐式驱动。以下是提取出的状态机逻辑：

```c
/* 初始化 (hpm_mcl_path_init) */
path->t_cure.next_empty = true;
path->t_cure.current = cfg->t_cure;
path->t_cure.ts = 0;
path->t_cure.current_empty = true;

/* 周期执行 (hpm_mcl_path_t_cure_generate) */
if (path->t_cure.current_empty) {       /* 状态: IDLE */
    path->t_cure.ts = 0;
    return mcl_success;
}

if (path->t_cure.ts == 0) {             /* 状态: PRE_COMPUTE (首次调用) */
    /* 预计算 t0, t1, t2, a0, a1, a2 */
}

path->theta.last = path->theta.current;
path->theta.current = path->theta.next;
path->t_cure.ts += 1;

if (path->t_cure.ts > path->t_cure.t2) {  /* 状态: COMPLETE */
    theta = path->theta.current;
    if (path->t_cure.next_empty == false) {
        path->t_cure.current = path->t_cure.next;  /* → PRE_COMPUTE (新曲线) */
        path->t_cure.next_empty = true;
        path->t_cure.ts = 0;
    } else {
        path->t_cure.current_empty = true;          /* → IDLE */
        path->t_cure.current.time = 0;
        path->t_cure.ts = 0;
    }
} else if (path->t_cure.ts > path->t_cure.t1) {  /* 状态: DECELERATION */
    /* 减速段角度计算 */
} else if (path->t_cure.ts > path->t_cure.t0) {  /* 状态: CONSTANT_SPEED */
    /* 匀速段角度计算 */
} else {                                          /* 状态: ACCELERATION */
    /* 加速段角度计算 */
}
```

#### 7.5.2 状态转换图

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    ▼                                         │
              ┌──────────┐   update_t_cure()    ┌───────────────┐
              │   IDLE   │─────────────────────▶│  PRE_COMPUTE  │
              │(current_ │                      │   (ts == 0)   │
              │ empty=T) │◀─────────────────────│               │
              └──────────┘   current_empty=T    └───────┬───────┘
                    ▲          next_empty=T              │
                    │                                     │ ts++ 后进入
                    │                                     ▼
                    │                            ┌───────────────┐
                    │                            │ ACCELERATION  │
                    │                            │ (ts ≤ t0)     │
                    │                            └───────┬───────┘
                    │                                    │ ts > t0
                    │                                    ▼
                    │                            ┌───────────────┐
                    │                            │CONSTANT_SPEED │
                    │                            │ (t0 < ts ≤ t1)│
                    │                            └───────┬───────┘
                    │                                    │ ts > t1
                    │                                    ▼
                    │                            ┌───────────────┐
                    │                            │ DECELERATION  │
                    │                            │ (t1 < ts ≤ t2)│
                    │                            └───────┬───────┘
                    │                                    │ ts > t2
                    │                                    ▼
                    │                            ┌───────────────┐
                    │                            │   COMPLETE    │
                    └────────────────────────────│  (ts > t2)    │
                       next_empty=T              │               │
                       (无待执行曲线)              └───────┬───────┘
                                                         │ next_empty=F
                                                         │ (有待执行曲线)
                                                         ▼
                                                  ┌───────────────┐
                                                  │  PRE_COMPUTE  │
                                                  │ (next→current)│
                                                  └───────────────┘
```

#### 7.5.3 各状态转换条件解析

| 转换 | 条件 | 动作 |
|------|------|------|
| IDLE → PRE_COMPUTE | `update_t_cure()` 写入 `current` | `current_empty = false`，`ts = 0` |
| PRE_COMPUTE → ACCELERATION | `ts` 自增后 `ts ≤ t0` | 执行预计算，进入加速段 |
| ACCELERATION → CONSTANT_SPEED | `ts > t0` 且 `ts ≤ t1` | 加速段结束，进入匀速段 |
| CONSTANT_SPEED → DECELERATION | `ts > t1` 且 `ts ≤ t2` | 匀速段结束，进入减速段 |
| DECELERATION → COMPLETE | `ts > t2` | 减速段结束，曲线执行完毕 |
| COMPLETE → IDLE | `next_empty == true` | 无待执行曲线，进入空闲 |
| COMPLETE → PRE_COMPUTE | `next_empty == false` | 有预装载曲线，搬移 `next → current`，`ts = 0` |

#### 7.5.4 路径完成判断逻辑

```c
bool hpm_mcl_path_t_cure_complete(mcl_path_plan_t *path)
{
    if ((path->t_cure.current_empty ==  true) &&
        (path->t_cure.next_empty ==  true)) {
        return true;
    } else {
        return false;
    }
}
```

**判断条件：`current_empty == true && next_empty == true`**

这意味着：
1. 当前曲线已执行完毕（`current_empty` 在 COMPLETE 状态下被置 true）
2. 没有预装载的曲线等待执行（`next_empty` 仍为 true）

**仅判断 `current_empty` 不够：** 若用户已通过 `update_t_cure()` 推送了下一条曲线到 `next` 缓冲区，`current_empty` 为 true 但 `next_empty` 为 false，此时路径尚未真正完成——还有一条曲线等待执行。

#### 7.5.5 工程要点

| 要点 | 说明 |
|------|------|
| **隐式状态机** | 无显式枚举，状态由变量组合隐含表达，优点是省 RAM，缺点是可读性稍差 |
| **PRE_COMPUTE 只执行一次** | `ts == 0` 分支确保预计算仅在曲线起始时执行，后续周期直接跳过 |
| **ts 从 1 开始计数** | 移位后 `ts += 1`，因此加速段的 `ts` 范围是 `[1, t0]`，与公式中 $k \ge 1$ 对应 |
| **退化曲线处理** | 若 `time == 0`，所有步数为 0，`ts = 1 > t2 = 0` 立即进入 COMPLETE，等效于空操作 |
| **连续运动的保证** | COMPLETE → PRE_COMPUTE 的无缝切换确保两条曲线之间无空闲周期，角度连续不跳变 |

---

## 附录：模块文件索引

| 文件 | 说明 |
|------|------|
| `core/control/hpm_mcl_path_plan.h` | 路径规划接口定义 |
| `core/control/hpm_mcl_path_plan.c` | 梯形曲线生成算法实现 |
| `core/control/hpm_mcl_hybrid_ctrl.h` | 混合力位控制接口（路径规划消费者） |
| `core/control/hpm_mcl_hybrid_ctrl.c` | 混合力位控制算法实现 |
| `core/loop/hpm_mcl_loop.h` | 主循环定义，挂载 path 指针 |