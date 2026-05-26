#ifndef __SERIAL_PROTOCOL_H__
#define __SERIAL_PROTOCOL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  帧格式定义
 *  | 帧头(2B) | 命令(1B) | 数据长度(1B) | 数据(NB) | CRC16(2B) | 帧尾(2B) |
 *  |   AA 55  |   CMD   |    LEN     |  PAYLOAD  |   CRC   |   55 AA  |
 *
 *  帧头: 0xAA 0x55
 *  帧尾: 0x55 0xAA
 *  CRC16: CCITT-FALSE (多项式 0x1021, 初始值 0xFFFF)
 *  最大数据长度: 128 字节
 * ============================================================ */

#define PROTO_FRAME_HEAD1        0xAA
#define PROTO_FRAME_HEAD2        0x55
#define PROTO_FRAME_TAIL1        0x55
#define PROTO_FRAME_TAIL2        0xAA
#define PROTO_HEAD_SIZE          2
#define PROTO_TAIL_SIZE          2
#define PROTO_CMD_SIZE           1
#define PROTO_LEN_SIZE           1
#define PROTO_CRC_SIZE           2
#define PROTO_FRAME_OVERHEAD     (PROTO_HEAD_SIZE + PROTO_CMD_SIZE + PROTO_LEN_SIZE + PROTO_CRC_SIZE + PROTO_TAIL_SIZE)
#define PROTO_MAX_PAYLOAD        128
#define PROTO_RX_BUF_SIZE        (PROTO_HEAD_SIZE + PROTO_CMD_SIZE + PROTO_LEN_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE + PROTO_TAIL_SIZE)
#define PROTO_TX_BUF_SIZE        PROTO_RX_BUF_SIZE

/* ============================================================
 *  命令码定义
 * ============================================================ */
typedef enum
{
    CMD_READ_PID         = 0x01,
    CMD_WRITE_PID        = 0x02,
    CMD_READ_MOTOR_STATE = 0x03,
    CMD_SET_REFERENCE    = 0x04,
    CMD_READ_SYSTEM_INFO = 0x05,

    CMD_TUNE_START       = 0x10,
    CMD_TUNE_STOP        = 0x11,
    CMD_TUNE_STATUS      = 0x12,
    CMD_TUNE_RESULT      = 0x13,
    CMD_TUNE_APPLY       = 0x14,

    CMD_BATCH_READ       = 0x20,
    CMD_BATCH_WRITE      = 0x21,

    CMD_SAVE_PARAMS      = 0x30,
    CMD_LOAD_PARAMS      = 0x31,

    CMD_STEP_TEST        = 0x40,
    CMD_SINE_SWEEP       = 0x41,

    CMD_ACK              = 0xF0,
    CMD_NACK             = 0xF1,
    CMD_ERROR            = 0xFF
} proto_cmd_t;

/* ============================================================
 *  错误码定义
 * ============================================================ */
typedef enum
{
    ERR_NONE             = 0x00,
    ERR_CRC_MISMATCH     = 0x01,
    ERR_INVALID_CMD      = 0x02,
    ERR_INVALID_LEN      = 0x03,
    ERR_PARAM_OUTOFRANGE = 0x04,
    ERR_TUNE_BUSY        = 0x05,
    ERR_TUNE_IDLE        = 0x06,
    ERR_TUNE_TIMEOUT     = 0x07,
    ERR_TUNE_UNSTABLE    = 0x08,
    ERR_MOTOR_OFFLINE    = 0x09,
    ERR_SAVE_FAILED      = 0x0A,
    ERR_LOAD_FAILED      = 0x0B,
    ERR_BUFFER_OVERFLOW  = 0x0C,
    ERR_UNKNOWN          = 0xFF
} proto_error_t;

/* ============================================================
 *  PID环标识
 * ============================================================ */
typedef enum
{
    PID_LOOP_SPEED  = 0x00,
    PID_LOOP_IQ     = 0x01,
    PID_LOOP_ID     = 0x02
} pid_loop_t;

/* ============================================================
 *  电机状态 (CMD_READ_MOTOR_STATE 返回)
 * ============================================================ */
#pragma pack(push, 1)
typedef struct
{
    float    speed;
    float    i_q;
    float    i_d;
    float    v_bus;
    float    temperature;
    uint8_t  motor_state;
    uint8_t  fault_code;
    uint16_t reserved;
} motor_state_t;
#pragma pack(pop)

/* ============================================================
 *  调参状态
 * ============================================================ */
typedef enum
{
    TUNE_STATE_IDLE       = 0x00,
    TUNE_STATE_INIT       = 0x01,
    TUNE_STATE_RELAY      = 0x02,
    TUNE_STATE_ANALYZE    = 0x03,
    TUNE_STATE_CALCULATE  = 0x04,
    TUNE_STATE_VERIFY     = 0x05,
    TUNE_STATE_DONE       = 0x06,
    TUNE_STATE_ERROR      = 0xFF
} tune_state_t;

/* ============================================================
 *  调参配置 (CMD_TUNE_START 下发)
 * ============================================================ */
#pragma pack(push, 1)
typedef struct
{
    pid_loop_t target_loop;
    uint8_t    tune_method;
    float      relay_amplitude;
    float      hysteresis;
    float      step_amplitude;
    uint16_t   max_cycles;
    uint32_t   timeout_ms;
} tune_config_t;
#pragma pack(pop)

/* ============================================================
 *  调参结果 (CMD_TUNE_RESULT 返回)
 * ============================================================ */
#pragma pack(push, 1)
typedef struct
{
    float    ultimate_gain;
    float    ultimate_period;
    float    tuned_kp;
    float    tuned_ki;
    float    tuned_kd;
    float    oscillation_amplitude;
    uint8_t  tune_method_used;
    uint8_t  status;
    uint16_t oscillation_cycles;
} tune_result_t;
#pragma pack(pop)

/* ============================================================
 *  协议句柄
 *  收发完全通过函数指针, 与具体硬件解耦
 *  - tx_func: 发送一包数据, 返回0成功/非0失败
 *  - on_command: 收到完整有效帧时的回调
 *  - user_data: 用户自定义上下文 (可选)
 * ============================================================ */
typedef struct
{
    uint8_t  rx_buf[PROTO_RX_BUF_SIZE];
    uint16_t rx_index;
    uint8_t  rx_state;
    uint8_t  rx_cmd;
    uint8_t  rx_len;

    uint8_t  tx_buf[PROTO_TX_BUF_SIZE];
    uint16_t tx_len;

    int  (*tx_func)(const uint8_t *data, uint16_t len);
    void (*on_command)(uint8_t cmd, uint8_t *data, uint8_t len);
    void *user_data;
} serial_proto_t;

/* ============================================================
 *  API
 * ============================================================ */
void     serial_proto_init  (serial_proto_t *proto,
                             int (*tx)(const uint8_t *data, uint16_t len));
void     serial_proto_set_callback(serial_proto_t *proto,
                             void (*cb)(uint8_t cmd, uint8_t *data, uint8_t len));
void     serial_proto_feed  (serial_proto_t *proto, uint8_t byte);
void     serial_proto_feed_buf(serial_proto_t *proto, const uint8_t *data, uint16_t len);

uint8_t  serial_proto_send_response(serial_proto_t *proto, uint8_t cmd,
                                    uint8_t *data, uint8_t len);
uint8_t  serial_proto_send_ack     (serial_proto_t *proto, uint8_t cmd);
uint8_t  serial_proto_send_nack    (serial_proto_t *proto, uint8_t cmd,
                                    uint8_t error_code);
uint8_t  serial_proto_send_error   (serial_proto_t *proto, uint8_t error_code);

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTOCOL_H__ */
