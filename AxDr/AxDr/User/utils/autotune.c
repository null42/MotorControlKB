#include "autotune.h"
#include <string.h>
#include <math.h>

/* ============================================================
 *  内部常量
 * ============================================================ */
#define AT_PI_F            3.14159265358979323846f
#define RELAY_MIN_CYCLE    4
#define MIN_OSC_AMP        1e-6f

/* ============================================================
 *  内部声明
 * ============================================================ */
static void relay_reset(relay_ctrl_t *r, float amp, float hys);
static float relay_update(relay_ctrl_t *r, float error);
static void osc_init(oscillation_capture_t *c, float *peaks, uint16_t max_p,
                     float *periods, uint16_t max_t);
static void osc_reset(oscillation_capture_t *c);
static void osc_feed(oscillation_capture_t *c, float value, float time);
static uint8_t osc_ready(oscillation_capture_t *c);
static void tune_calc(autotune_t *at);

/* ============================================================
 *  PID 工具函数
 * ============================================================ */

void at_pid_init(at_pid_t *pid)
{
    memset(pid, 0, sizeof(at_pid_t));
}

void at_pid_set_limits(at_pid_t *pid, float i_max, float i_min,
                       float o_max, float o_min)
{
    pid->i_term_max = i_max;
    pid->i_term_min = i_min;
    pid->out_max    = o_max;
    pid->out_min    = o_min;
}

void at_pid_clear(at_pid_t *pid)
{
    pid->p_term    = 0.0f;
    pid->i_term    = 0.0f;
    pid->d_term    = 0.0f;
    pid->error     = 0.0f;
    pid->pre_err   = 0.0f;
    pid->out_value = 0.0f;
}

void at_pid_reset(at_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float at_pid_serial_ctrl(at_pid_t *pid, float ref, float fb)
{
    pid->ref_value   = ref;
    pid->fback_value = fb;
    pid->error       = ref - fb;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->p_term;

    if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
    if (pid->i_term < pid->i_term_min) pid->i_term = pid->i_term_min;

    pid->out_value = pid->p_term + pid->i_term;

    if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
    if (pid->out_value < pid->out_min) pid->out_value = pid->out_min;

    return pid->out_value;
}

float at_pid_parallel_ctrl(at_pid_t *pid, float ref, float fb)
{
    pid->ref_value   = ref;
    pid->fback_value = fb;
    pid->error       = ref - fb;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;

    if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
    if (pid->i_term < pid->i_term_min) pid->i_term = pid->i_term_min;

    pid->d_term  = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
    if (pid->out_value < pid->out_min) pid->out_value = pid->out_min;

    return pid->out_value;
}

/* ============================================================
 *  调参引擎
 * ============================================================ */

void autotune_init(autotune_t *at, at_pid_t *pid)
{
    if (at == NULL) return;

    memset(at, 0, sizeof(autotune_t));
    at->target_pid = pid;

    at->config.target_loop     = PID_LOOP_SPEED;
    at->config.tune_method     = TUNE_METHOD_ZN_PI;
    at->config.relay_amplitude = 0.5f;
    at->config.hysteresis      = 0.05f;
    at->config.max_cycles      = 12;
    at->config.timeout_ms      = 30000;

    osc_init(&at->capture, at->peak_buf, 64, at->period_buf, 64);
}

void autotune_set_config(autotune_t *at, tune_config_t *cfg)
{
    if (at && cfg)
        memcpy(&at->config, cfg, sizeof(tune_config_t));
}

void autotune_set_callbacks(autotune_t *at,
                            float (*get_fb)(void),
                            void  (*set_out)(float),
                            float (*get_t)(void))
{
    if (at == NULL) return;
    at->get_feedback = get_fb;
    at->set_output   = set_out;
    at->get_time     = get_t;
}

void autotune_set_report(autotune_t *at, void (*report)(uint8_t *data, uint16_t len))
{
    if (at) at->report_func = report;
}

uint8_t autotune_start(autotune_t *at)
{
    if (at == NULL) return 1;
    if (at->is_running) return 1;
    if (!at->get_feedback || !at->set_output || !at->get_time) return 2;

    at->state      = TUNE_STATE_INIT;
    at->is_running = 1;
    at->start_tick = (uint32_t)(at->get_time() * 1000.0f);

    osc_reset(&at->capture);
    relay_reset(&at->relay, at->config.relay_amplitude, at->config.hysteresis);
    memset(&at->result, 0, sizeof(tune_result_t));

    return 0;
}

uint8_t autotune_stop(autotune_t *at)
{
    if (at == NULL) return 1;
    at->is_running = 0;
    at->state      = TUNE_STATE_IDLE;
    if (at->set_output) at->set_output(0.0f);
    return 0;
}

uint8_t autotune_is_busy(autotune_t *at)
{
    return (at && at->is_running) ? 1 : 0;
}

tune_state_t autotune_get_state(autotune_t *at)
{
    return at ? at->state : TUNE_STATE_IDLE;
}

tune_result_t *autotune_get_result(autotune_t *at)
{
    return at ? &at->result : NULL;
}

uint8_t autotune_apply_result(autotune_t *at)
{
    if (!at || !at->target_pid) return 1;
    if (at->state != TUNE_STATE_DONE) return 2;

    at_pid_t *pid = at->target_pid;
    pid->kp = at->result.tuned_kp;
    pid->ki = at->result.tuned_ki;
    pid->kd = at->result.tuned_kd;
    at_pid_clear(pid);

    at->result.tune_method_used = at->config.tune_method;
    return 0;
}

/*
 *  调参主状态机: INIT → RELAY → ANALYZE → CALCULATE → DONE
 *  每次 FOC 主循环调用一次
 */
void autotune_run(autotune_t *at)
{
    if (!at || !at->is_running) return;

    float now = at->get_time();
    float fb  = at->get_feedback();

    switch (at->state)
    {
    case TUNE_STATE_INIT:
        at->state         = TUNE_STATE_RELAY;
        at->cycle_count   = 0;
        at->osc_start_tick = (uint32_t)(now * 1000.0f);
        osc_reset(&at->capture);
        break;

    case TUNE_STATE_RELAY:
    {
        float ref = (at->config.step_amplitude > 0.001f)
                      ? at->config.step_amplitude : 0.0f;
        float err  = ref - fb;
        float out  = relay_update(&at->relay, err);
        at->set_output(out);

        osc_feed(&at->capture, fb, now);

        uint32_t elapsed = (uint32_t)(now * 1000.0f) - at->start_tick;
        if (at->config.timeout_ms > 0 && elapsed > at->config.timeout_ms)
        {
            at->state        = TUNE_STATE_ERROR;
            at->result.status = ERR_TUNE_TIMEOUT;
            at->is_running   = 0;
            break;
        }

        if (osc_ready(&at->capture))
            at->state = TUNE_STATE_ANALYZE;
        break;
    }

    case TUNE_STATE_ANALYZE:
    {
        at->set_output(0.0f);

        if (at->capture.peak_count < 4 || at->capture.period_count < 2)
        {
            at->state         = TUNE_STATE_ERROR;
            at->result.status = ERR_TUNE_UNSTABLE;
            at->is_running    = 0;
            break;
        }

        /* 平均峰-峰值 */
        float sum_amp  = 0.0f;
        uint16_t n_amp = 0;
        float prev     = 0.0f;
        uint8_t first  = 1;

        for (uint16_t i = 0; i < at->capture.peak_count; i++)
        {
            if (first) { prev = at->capture.peak_values[i]; first = 0; continue; }
            float a = (float)fabs((double)(at->capture.peak_values[i] - prev));
            if (a > MIN_OSC_AMP) { sum_amp += a; n_amp++; }
            prev = at->capture.peak_values[i];
        }

        /* 平均周期 */
        float sum_t  = 0.0f;
        uint16_t n_t = 0;
        for (uint16_t i = 0; i < at->capture.period_count; i++)
        {
            if (at->capture.period_values[i] > 1e-6f)
            { sum_t += at->capture.period_values[i]; n_t++; }
        }

        if (n_amp == 0 || n_t == 0)
        {
            at->state         = TUNE_STATE_ERROR;
            at->result.status = ERR_TUNE_UNSTABLE;
            at->is_running    = 0;
            break;
        }

        float av_amp = sum_amp / (float)n_amp;
        float av_T   = sum_t   / (float)n_t;

        /* Ku = 4d / (pi * a) , a = 单边幅值 = av_amp / 2 */
        at->result.ultimate_gain   = (4.0f * at->config.relay_amplitude)
                                   / (AT_PI_F * av_amp * 0.5f);
        at->result.ultimate_period           = av_T;
        at->result.oscillation_amplitude     = av_amp * 0.5f;
        at->result.oscillation_cycles        = n_t;

        at->state = TUNE_STATE_CALCULATE;
        break;
    }

    case TUNE_STATE_CALCULATE:
        tune_calc(at);
        at->state = TUNE_STATE_DONE;
        break;

    case TUNE_STATE_DONE:
        at->is_running = 0;
        break;

    case TUNE_STATE_ERROR:
        at->is_running = 0;
        if (at->set_output) at->set_output(0.0f);
        break;

    default:
        break;
    }
}

/* ============================================================
 *  继电器反馈
 * ============================================================ */

static void relay_reset(relay_ctrl_t *r, float amp, float hys)
{
    r->amplitude      = amp;
    r->hysteresis     = hys;
    r->last_output    = amp;
    r->last_direction = 1;
}

static float relay_update(relay_ctrl_t *r, float error)
{
    if (error > r->hysteresis)
    {
        r->last_output    =  r->amplitude;
        r->last_direction = 1;
    }
    else if (error < -r->hysteresis)
    {
        r->last_output    = -r->amplitude;
        r->last_direction = 0;
    }
    return r->last_output;
}

/* ============================================================
 *  振荡采集
 * ============================================================ */

static void osc_init(oscillation_capture_t *c, float *peaks, uint16_t max_p,
                     float *periods, uint16_t max_t)
{
    c->peak_values   = peaks;
    c->max_peaks     = max_p;
    c->period_values = periods;
    c->max_periods   = max_t;
    osc_reset(c);
}

static void osc_reset(oscillation_capture_t *c)
{
    c->peak_count   = 0;
    c->period_count = 0;
    c->last_value   = 0.0f;
    c->direction    = 1;
    c->cross_time   = 0.0f;
}

static void osc_feed(oscillation_capture_t *c, float value, float time)
{
    if (c->peak_count == 0)
    {
        c->last_value       = value;
        c->peak_values[0]   = value;
        c->peak_count       = 1;
        return;
    }

    float delta = value - c->last_value;

    /* 趋势反转 → 记录峰值 */
    if ((c->direction == 1 && delta < 0.0f) ||
        (c->direction == 0 && delta > 0.0f))
    {
        if (c->peak_count < c->max_peaks)
            c->peak_values[c->peak_count++] = c->last_value;
        c->direction = (delta > 0.0f) ? 1 : 0;
    }
    else
    {
        c->direction = (delta > 0.0f) ? 1 : 0;
    }

    /* 零点穿越 → 记录周期 */
    if (c->last_value * value <= 0.0f)
    {
        if (c->cross_time > 0.0f)
        {
            float period = time - c->cross_time;
            if (period > 1e-6f && c->period_count < c->max_periods)
                c->period_values[c->period_count++] = period;
        }
        c->cross_time = time;
    }

    c->last_value = value;
}

static uint8_t osc_ready(oscillation_capture_t *c)
{
    return (c->peak_count >= RELAY_MIN_CYCLE * 2
         && c->period_count >= RELAY_MIN_CYCLE) ? 1 : 0;
}

/* ============================================================
 *  Ziegler-Nichols 参数计算
 *
 *  基于临界增益 Ku, 临界周期 Tu:
 *    ZN-PI:            Kp=0.45·Ku   Ti=Tu/1.2
 *    ZN-PID(经典):     Kp=0.60·Ku   Ti=Tu/2.0  Td=Tu/8.0
 *    ZN-有超调:        Kp=0.33·Ku   Ti=Tu/2.0  Td=Tu/5.0
 *    ZN-无超调:        Kp=0.20·Ku   Ti=Tu/3.0  Td=Tu/5.0
 *    Tyreus-Luyben:    Kp=Ku/3.2    Ti=2.2·Tu  Td=Tu/6.3
 * ============================================================ */

static void tune_calc(autotune_t *at)
{
    float Ku = at->result.ultimate_gain;
    float Tu = at->result.ultimate_period;

    if (Ku < 1e-9f || Tu < 1e-9f)
    {
        at->result.status = ERR_TUNE_UNSTABLE;
        return;
    }

    float Kp = 0.0f, Ti = 0.0f, Td = 0.0f;

    switch (at->config.tune_method)
    {
    case TUNE_METHOD_ZN_PI:
        Kp = 0.45f * Ku;  Ti = Tu / 1.2f;  Td = 0.0f;     break;
    case TUNE_METHOD_ZN_PID:
        Kp = 0.60f * Ku;  Ti = Tu / 2.0f;  Td = Tu / 8.0f; break;
    case TUNE_METHOD_ZN_SOME_OVERSHOT:
        Kp = 0.33f * Ku;  Ti = Tu / 2.0f;  Td = Tu / 5.0f; break;
    case TUNE_METHOD_ZN_NO_OVERSHOOT:
        Kp = 0.20f * Ku;  Ti = Tu / 3.0f;  Td = Tu / 5.0f; break;
    case TUNE_METHOD_TYREUS_LUYBEN:
        Kp = Ku / 3.2f;   Ti = 2.2f * Tu;  Td = Tu / 6.3f; break;
    default:
        Kp = 0.45f * Ku;  Ti = Tu / 1.2f;  Td = 0.0f;     break;
    }

    at->result.tuned_kp = Kp;
    at->result.tuned_ki = (Ti > 1e-9f) ? (Kp / Ti) : 0.0f;
    at->result.tuned_kd = Kp * Td;
    at->result.status   = 0;
}

/* ============================================================
 *  命令处理器
 * ============================================================ */

static autotune_cmd_t *g_cmd = NULL;

void autotune_cmd_init(autotune_cmd_t *cmd, serial_proto_t *proto,
                       autotune_t *at)
{
    if (!cmd) return;
    memset(cmd, 0, sizeof(autotune_cmd_t));
    cmd->proto    = proto;
    cmd->autotune = at;
    g_cmd         = cmd;

    serial_proto_set_callback(proto, autotune_cmd_dispatch);
}

void autotune_cmd_dispatch(uint8_t cmd, uint8_t *data, uint8_t len)
{
    if (!g_cmd || !g_cmd->proto) return;

    serial_proto_t *p = g_cmd->proto;

    switch (cmd)
    {
    /* ---- 读 PID ---- */
    case CMD_READ_PID:
    {
        if (len < 1) { serial_proto_send_nack(p, cmd, ERR_INVALID_LEN); break; }

        at_pid_t *pid = NULL;
        if (g_cmd->pid_speed && data[0] == PID_LOOP_SPEED) pid = g_cmd->pid_speed;
        if (g_cmd->pid_iq    && data[0] == PID_LOOP_IQ)    pid = g_cmd->pid_iq;
        if (g_cmd->pid_id    && data[0] == PID_LOOP_ID)    pid = g_cmd->pid_id;
        if (!pid) { serial_proto_send_nack(p, cmd, ERR_INVALID_CMD); break; }

        uint8_t r[28];
        memcpy(&r[0],  &pid->kp,  4);
        memcpy(&r[4],  &pid->ki,  4);
        memcpy(&r[8],  &pid->kd,  4);
        memcpy(&r[12], &pid->i_term_max, 4);
        memcpy(&r[16], &pid->i_term_min, 4);
        memcpy(&r[20], &pid->out_max,    4);
        memcpy(&r[24], &pid->out_min,    4);
        serial_proto_send_response(p, cmd, r, 28);
        break;
    }

    /* ---- 写 PID ---- */
    case CMD_WRITE_PID:
    {
        if (len < 29) { serial_proto_send_nack(p, cmd, ERR_INVALID_LEN); break; }

        at_pid_t *pid = NULL;
        if (g_cmd->pid_speed && data[0] == PID_LOOP_SPEED) pid = g_cmd->pid_speed;
        if (g_cmd->pid_iq    && data[0] == PID_LOOP_IQ)    pid = g_cmd->pid_iq;
        if (g_cmd->pid_id    && data[0] == PID_LOOP_ID)    pid = g_cmd->pid_id;
        if (!pid) { serial_proto_send_nack(p, cmd, ERR_INVALID_CMD); break; }

        memcpy(&pid->kp,         &data[1],  4);
        memcpy(&pid->ki,         &data[5],  4);
        memcpy(&pid->kd,         &data[9],  4);
        memcpy(&pid->i_term_max, &data[13], 4);
        memcpy(&pid->i_term_min, &data[17], 4);
        memcpy(&pid->out_max,    &data[21], 4);
        memcpy(&pid->out_min,    &data[25], 4);
        at_pid_clear(pid);
        serial_proto_send_ack(p, cmd);
        break;
    }

    /* ---- 读电机状态 ---- */
    case CMD_READ_MOTOR_STATE:
    {
        motor_state_t s;
        if (g_cmd->get_motor_state) g_cmd->get_motor_state(&s);
        else memset(&s, 0, sizeof(s));
        serial_proto_send_response(p, cmd, (uint8_t *)&s, sizeof(s));
        break;
    }

    /* ---- 设目标值 ---- */
    case CMD_SET_REFERENCE:
    {
        if (len < 5) { serial_proto_send_nack(p, cmd, ERR_INVALID_LEN); break; }
        float val; memcpy(&val, &data[1], 4);

        if (data[0] == PID_LOOP_SPEED && g_cmd->set_speed_ref)
            { g_cmd->set_speed_ref(val); serial_proto_send_ack(p, cmd); }
        else if (data[0] == PID_LOOP_IQ && g_cmd->set_iq_ref)
            { g_cmd->set_iq_ref(val); serial_proto_send_ack(p, cmd); }
        else if (data[0] == PID_LOOP_ID && g_cmd->set_id_ref)
            { g_cmd->set_id_ref(val); serial_proto_send_ack(p, cmd); }
        else
            serial_proto_send_nack(p, cmd, ERR_INVALID_CMD);
        break;
    }

    /* ---- 启动调参 ---- */
    case CMD_TUNE_START:
    {
        if (!g_cmd->autotune)
            { serial_proto_send_nack(p, cmd, ERR_UNKNOWN); break; }

        if (len >= sizeof(tune_config_t))
        {
            tune_config_t cfg;
            memcpy(&cfg, data, sizeof(tune_config_t));
            autotune_set_config(g_cmd->autotune, &cfg);
        }

        uint8_t ret = autotune_start(g_cmd->autotune);
        if (ret == 0)
            serial_proto_send_ack(p, cmd);
        else
            serial_proto_send_nack(p, cmd,
                (ret == 1) ? ERR_TUNE_BUSY : ERR_MOTOR_OFFLINE);
        break;
    }

    /* ---- 停止调参 ---- */
    case CMD_TUNE_STOP:
        if (!g_cmd->autotune)
            { serial_proto_send_nack(p, cmd, ERR_UNKNOWN); break; }
        autotune_stop(g_cmd->autotune);
        serial_proto_send_ack(p, cmd);
        break;

    /* ---- 查询状态 ---- */
    case CMD_TUNE_STATUS:
    {
        if (!g_cmd->autotune)
            { serial_proto_send_nack(p, cmd, ERR_UNKNOWN); break; }
        tune_state_t st = autotune_get_state(g_cmd->autotune);
        uint8_t r[4] = { (uint8_t)st, g_cmd->autotune->is_running ? 1 : 0, 0, 0 };
        serial_proto_send_response(p, cmd, r, 4);
        break;
    }

    /* ---- 获取结果 ---- */
    case CMD_TUNE_RESULT:
    {
        if (!g_cmd->autotune)
            { serial_proto_send_nack(p, cmd, ERR_UNKNOWN); break; }
        tune_result_t *res = autotune_get_result(g_cmd->autotune);
        serial_proto_send_response(p, cmd, (uint8_t *)res, sizeof(tune_result_t));
        break;
    }

    /* ---- 应用结果 ---- */
    case CMD_TUNE_APPLY:
    {
        if (!g_cmd->autotune)
            { serial_proto_send_nack(p, cmd, ERR_UNKNOWN); break; }
        uint8_t ret = autotune_apply_result(g_cmd->autotune);
        if (ret == 0)
            serial_proto_send_ack(p, cmd);
        else
            serial_proto_send_nack(p, cmd,
                (ret == 1) ? ERR_UNKNOWN : ERR_TUNE_IDLE);
        break;
    }

    /* ---- 阶跃测试 ---- */
    case CMD_STEP_TEST:
    {
        if (len < 5)
            { serial_proto_send_nack(p, cmd, ERR_INVALID_LEN); break; }
        float amp; memcpy(&amp, &data[1], 4);

        if (data[0] == PID_LOOP_SPEED && g_cmd->set_speed_ref)
            g_cmd->set_speed_ref(amp);
        else if (data[0] == PID_LOOP_IQ && g_cmd->set_iq_ref)
            g_cmd->set_iq_ref(amp);
        serial_proto_send_ack(p, cmd);
        break;
    }

    default:
        serial_proto_send_nack(p, cmd, ERR_INVALID_CMD);
        break;
    }
}
