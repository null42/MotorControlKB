# 无感FOC算法集成指南

## 一、文件清单

### 新增文件
```
AxDr/AxDr/User/
├── motor/
│   ├── observer.h/c          # 观测器模块
│   ├── startup.h/c           # 启动策略模块
│   ├── observer_config.h     # 观测器参数配置
│   └── foc_ctrl_new.h/c      # 集成示例代码
└── utils/
    ├── lpf.h/c               # 低通滤波器
    ├── angle.h/c             # 角度处理
    └── table.h/c             # 查表插值
```

## 二、集成步骤

### 步骤1: 添加文件到Keil工程

1. 在Keil工程中创建新的文件组：
   - `User/motor` 组：添加 observer.c, startup.c, foc_ctrl_new.c
   - `User/utils` 组：添加 lpf.c, angle.c, table.c

2. 添加头文件包含路径：
   ```
   ..\..\User\motor
   ..\..\User\utils
   ```

### 步骤2: 修改 foc_ctrl.c

在文件开头添加：

```c
#include "foc_ctrl_new.h"
```

在 `motor_ctrl()` 函数的 `mt_run` 状态中添加观测器更新：

```c
case mt_run:
    foc_pwm_run(&foc);
    foc_ctrl_update();      // 新增：更新观测器和角度融合
    foc_ctrl_mode();
    foc_calc(&foc);
    break;
```

### 步骤3: 初始化

在 `main.c` 的初始化部分添加：

```c
#include "foc_ctrl_new.h"

// 在系统初始化后调用
foc_ctrl_init();
```

### 步骤4: 配置参数

修改 `observer_config.h` 中的参数以匹配您的电机：

```c
#define OBS_MOTOR_RS            0.5f    // 定子电阻 (Ω)
#define OBS_MOTOR_LD            0.001f  // d轴电感 (H)
#define OBS_MOTOR_LQ            0.001f  // q轴电感 (H)
#define OBS_MOTOR_FLUX          0.01f   // 永磁体磁链 (Wb)
#define OBS_MOTOR_TS            0.00005f // 采样周期 (s)

// 观测器增益
#define OBS_SMO_H1_BASE         100.0f
#define OBS_SMO_PLL_KP          500.0f
#define OBS_SMO_PLL_KI          1000.0f

// 启动参数
#define STARTUP_ALIGN_CURRENT   0.5f    // 定位电流 (A)
#define STARTUP_ALIGN_TIME_MS   500.0f  // 定位时间 (ms)
#define STARTUP_START_FREQ      5.0f    // 启动频率 (Hz)
#define STARTUP_END_FREQ        50.0f   // 结束频率 (Hz)
```

## 三、使用方法

### 有感模式（默认）

系统默认使用编码器角度，无需额外操作。

### 无感模式启动

在启动电机时，系统会自动执行无感启动流程：

1. **转子定位阶段**：注入直流电流，定位转子
2. **开环加速阶段**：从低频开环加速到设定频率
3. **切换阶段**：平滑切换到观测器角度
4. **闭环运行**：使用观测器角度进行FOC控制

### 有感/无感切换

```c
// 切换到编码器角度
angle_source_switch(ANGLE_SRC_ENCODER);

// 切换到观测器角度
angle_source_switch(ANGLE_SRC_OBSERVER);

// 使用混合角度（过渡期间）
angle_source_switch(ANGLE_SRC_HYBRID);
```

## 四、调试建议

### 4.1 观测器调试

使用VOFA+或串口绘图工具观测以下变量：

```c
// 观测器输出
float theta_obs = observer_get_angle(&g_observer);
float speed_obs = observer_get_speed(&g_observer);

// 编码器输出
float theta_enc = mt6816.elec_angle;
float speed_enc = mt6816.speed;

// 角度误差
float theta_error = theta_obs - theta_enc;
```

### 4.2 参数整定

**观测器增益调整**：
1. 从较小的H1值开始（如50）
2. 观察角度跟踪误差
3. 逐步增大H1直到误差最小
4. 调整PLL参数优化动态响应

**启动参数调整**：
1. 定位电流：通常为额定电流的30-50%
2. 定位时间：确保转子稳定（通常300-1000ms）
3. 启动频率：根据电机极对数调整（通常5-20Hz）
4. 加速速率：避免失步（通常5-20Hz/s）

### 4.3 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 启动失败 | 定位电流不足 | 增大STARTUP_ALIGN_CURRENT |
| 角度抖动 | 观测器增益过大 | 减小OBS_SMO_H1_BASE |
| 低速失步 | PLL带宽过低 | 增大OBS_SMO_PLL_KP |
| 切换冲击 | 过渡时间过短 | 增大STARTUP_SWITCH_TIME_MS |

## 五、性能优化

### 5.1 使用CORDIC硬件加速

STM32G474内置CORDIC单元，可加速三角函数计算：

```c
// 在angle.c中替换为CORDIC版本
void sincos_calc_cordic(trig_t *trig)
{
    // 使用CORDIC硬件计算sin/cos
    // 参考STM32G4 CORDIC HAL库
}
```

### 5.2 内存优化

将查表数据放入Flash：

```c
// 在table.c中
const float sin_table[SINE_TABLE_SIZE + 1] = { ... };
```

### 5.3 实时性优化

- 将观测器计算放在ADC中断中
- 参数自适应可放在低优先级任务中
- 使用DMA传输ADC数据

## 六、测试验证

### 6.1 功能测试

- [ ] 空载启动测试
- [ ] 负载启动测试
- [ ] 有感/无感切换测试
- [ ] 长时间运行稳定性测试

### 6.2 性能测试

- [ ] 角度跟踪精度测试（误差<5°）
- [ ] 速度控制精度测试（误差<5%）
- [ ] 动态响应测试（阶跃响应）
- [ ] CPU负载测试（<30%）

## 七、故障诊断

### 观测器不收敛

检查项：
1. 电机参数是否正确
2. 电流采样是否准确
3. 电压输出是否正常
4. 观测器增益是否合理

### 启动失败

检查项：
1. 定位电流是否足够
2. 加速速率是否过快
3. 负载是否过大
4. 电源电压是否稳定

## 八、注意事项

1. **首次调试**：建议先在仿真环境测试
2. **安全保护**：确保过流保护功能正常
3. **参数保存**：调试好的参数建议保存到Flash
4. **文档记录**：记录调试过程和参数

## 九、技术支持

如遇问题，请检查：
1. 编译器是否开启FPU支持
2. 头文件路径是否正确
3. 电机参数是否匹配
4. 硬件连接是否正常

---

**版本**: v1.0  
**日期**: 2026-04-10  
**作者**: AI Assistant
