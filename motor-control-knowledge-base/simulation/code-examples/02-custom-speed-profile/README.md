# 示例 2：自定义速度/负载曲线

## 目标
学会修改 `pmsm_comm.c` 的 `_user_commands()` 函数来设计自己的仿真场景。

## 要修改的文件
`E:\new_things\emachinery\emachinery\frameworkCodes\c\pmsm_comm.c`

## 修改方案

在 `_user_commands()` 函数中找到 `#if PC_SIMULATION == TRUE` 块（约第 734 行），修改速度指令和负载转矩。以下是一个典型测试场景：

```
时间轴：
  0.00s - 速度指令 0 rpm，电机静止准备
  0.01s - 速度阶跃到 +400 rpm，正向加速
  0.04s - 突加满载（额定电流 50% 对应转矩）
  0.07s - 撤除负载，恢复到空载
  0.10s - 速度反转到 -300 rpm
```

## 预期效果
- Speed 子图：0→400rpm 阶跃（略有超调后稳定）→ 突加负载掉速 → 恢复 → 反转
- iQ 子图：加速时冲到限幅，稳速下降，加负载时回升
- Torque 子图：Tem 跟随 TLoad，加减速时有惯量转矩叠加

## 修改要点
- `(*CTRL).i->cmd_varOmega = xxx * RPM_2_MECH_RAD_PER_SEC` — 速度指令
- `ACM.TLoad = xxx` — 负载转矩 [Nm]
- `(*CTRL).timebase` — 仿真时间（从 0 开始，单位秒）

## 注意事项
- 必须在 `#if PC_SIMULATION == TRUE` 块中修改
- 不同 `WHO_IS_USER` 的用户有各自的指令块，确保在正确的位置修改
