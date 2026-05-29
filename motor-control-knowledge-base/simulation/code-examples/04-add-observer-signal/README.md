# 示例 4：添加自定义观测信号

## 目标
学会在仿真输出中添加新的可观测信号——以 ESO 估算的负载转矩为例。

## 三步法

### Step 1：在 YAML signal_library 中添加信号
在 `user_config.yaml` 的 `signal_library` 末尾添加：
```yaml
  - OFSR.esoaf.xTL  # ESO 估算的负载转矩
```

### Step 2：确认 C 代码已记录该信号
`utility.c` 中 `write_data_to_file()` 的 fprintf 调用了 `DATA_DETAILS` 宏。
`DATA_DETAILS` 由 `super_config.py` 根据 `signal_library` 自动生成。
所以在 Step 1 后点击「Save to C and compile」即可自动完成此步骤。

### Step 3：在 cplot 中添加新子图
在 `user_config.yaml` 的 `cplot.subplot` 中添加：
```yaml
    - title: ESO Load Torque Estimation
      y_title: Torque [Nm]
      y:
        - y_data: ACM.TLoad
          y_label: True $T_L$
        - y_data: OFSR.esoaf.xTL
          y_label: ESO $\hat{T}_L$
```

## 预期效果
- 新增子图显示真实负载 `ACM.TLoad` 和 ESO 估算 `OFSR.esoaf.xTL` 的对比
- 稳态时两条线应重合
- 负载突变时 ESO 估计有一点滞后（取决于观测器带宽 `CAREFUL_ESOAF_OMEGA_OBSERVER`）
