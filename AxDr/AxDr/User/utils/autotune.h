#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

#include <stdint.h>
#include "serial_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  自包含 PID 结构体 (不依赖任何外部 pid.h)
 * ============================================================ */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float p_term;
    float i_term;
    float d_term;

    float i_term_max;
    float i_term_min;

    float ctrl_period;

    float ref_value;
    float fback_value;

    float error;
    float pre_err;

    float out_min;
    float out_max;

    float out_value;
} at_pid_t;

/* ============================================================
 *  调参方法枚举
 * ============================================================ */
typedef enum
{
    TUNE_METHOD_ZN_PI            = 0,
    TUNE_METHOD_ZN_PID           = 1,
    TUNE_METHOD_ZN_SOME_OVERSHOT = 2,
    TUNE_METHOD_ZN_NO_OVERSHOOT  = 3,
    TUNE_METHOD_TYREUS_LUYBEN    = 4
} tune_method_t;

/* ============================================================
 *  继电器反馈控制器
 * ============================================================ */
typedef struct
{
    float   amplitude;
    float   hysteresis;
    float   last_output;
    uint8_t last_direction;
} relay_ctrl_t;

/* ============================================================
 *  振荡数据采集器
 * ============================================================ */
typedef struct
{
    float   *peak_values;
    uint16_t peak_count;
    uint16_t max_peaks;
    float    last_value;
    uint8_t  direction;
    float    cross_time;
    float   *period_values;
    uint16_t period_count;
    uint16_t max_periods;
    uint32_t start_tick;
} oscillation_capture_t;

/* ============================================================
 *  自动调参引擎
 *
 *  与硬件完全解耦，通过函数指针注入:
 *    get_feedback  → 获取被控量当前值
 *    set_output    → 设置控制输出
 *    get_time      → 获取当前时间(秒, 浮点)
 *    report_func   → 可选, 调试数据回传
 * ============================================================ */
typedef struct
{
    tune_config_t  config;
    tune_state_t   state;
    tune_result_t  result;

    uint32_t start_tick;
    uint32_t osc_start_tick;
    uint16_t cycle_count;
    uint8_t  is_running;

    relay_ctrl_t          relay;
    oscillation_capture_t capture;

    float peak_buf[64];
    float period_buf[64];

    at_pid_t *target_pid;

    void (*report_func)(uint8_t *data, uint16_t len);

    float (*get_feedback)(void);
    void  (*set_output)(float value);
    float (*get_time)(void);
} autotune_t;

/* ============================================================
 *  API: 调参引擎
 * ============================================================ */
void autotune_init(autotune_t *at, at_pid_t *pid);
void autotune_set_config(autotune_t *at, tune_config_t *cfg);
void autotune_set_callbacks(autotune_t *at,
                            float (*get_fb)(void),
                            void  (*set_out)(float),
                            float (*get_t)(void));
void autotune_set_report(autotune_t *at, void (*report)(uint8_t *data, uint16_t len));

uint8_t       autotune_start(autotune_t *at);
uint8_t       autotune_stop(autotune_t *at);
uint8_t       autotune_is_busy(autotune_t *at);
tune_state_t  autotune_get_state(autotune_t *at);
tune_result_t *autotune_get_result(autotune_t *at);
uint8_t       autotune_apply_result(autotune_t *at);

void autotune_run(autotune_t *at);

/* ============================================================
 *  API: PID 工具函数 (自包含, 不依赖外部 pid.c)
 * ============================================================ */
void at_pid_init(at_pid_t *pid);
void at_pid_set_limits(at_pid_t *pid, float i_max, float i_min,
                       float o_max, float o_min);
void at_pid_clear(at_pid_t *pid);
void at_pid_reset(at_pid_t *pid, float kp, float ki, float kd);
float at_pid_serial_ctrl(at_pid_t *pid, float ref, float fb);
float at_pid_parallel_ctrl(at_pid_t *pid, float ref, float fb);

/* ============================================================
 *  命令处理器: 串口协议 ↔ 调参引擎 的桥接器
 *
 *  用户只需:
 *    1. 创建 autotune_cmd_t 实例
 *    2. 填充 ptr 指向各自的 PID / 回调
 *    3. 调用 autotune_cmd_init 注册
 *    4. 串口收到数据后调用 autotune_cmd_feed
 * ============================================================ */
typedef struct
{
    serial_proto_t *proto;
    autotune_t     *autotune;

    at_pid_t *pid_speed;
    at_pid_t *pid_iq;
    at_pid_t *pid_id;

    void  (*get_motor_state)(motor_state_t *s);
    float (*get_speed_fb)(void);
    float (*get_iq_fb)(void);
    float (*get_id_fb)(void);
    void  (*set_speed_ref)(float v);
    void  (*set_iq_ref)(float v);
    void  (*set_id_ref)(float v);
} autotune_cmd_t;

void autotune_cmd_init(autotune_cmd_t *cmd, serial_proto_t *proto,
                       autotune_t *at);
void autotune_cmd_dispatch(uint8_t cmd, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AUTOTUNE_H__ */
