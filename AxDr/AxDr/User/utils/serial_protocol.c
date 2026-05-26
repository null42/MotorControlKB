#include "serial_protocol.h"
#include <string.h>

/* ============================================================
 *  CRC16 CCITT-FALSE
 *  多项式 0x1021, 初值 0xFFFF, 不反转
 * ============================================================ */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* ============================================================
 *  初始化协议句柄
 *  tx: 发送回调 int tx(const uint8_t *data, uint16_t len)
 *      返回 0 表示成功, 非 0 表示失败
 * ============================================================ */
void serial_proto_init(serial_proto_t *proto,
                       int (*tx)(const uint8_t *data, uint16_t len))
{
    if (proto == NULL)
        return;

    memset(proto, 0, sizeof(serial_proto_t));
    proto->tx_func = tx;
}

void serial_proto_set_callback(serial_proto_t *proto,
                               void (*cb)(uint8_t cmd, uint8_t *data, uint8_t len))
{
    if (proto != NULL)
        proto->on_command = cb;
}

/* ============================================================
 *  内部: 组装发送帧到 tx_buf, 设置 tx_len
 * ============================================================ */
static void build_frame(serial_proto_t *proto, uint8_t cmd,
                        const uint8_t *data, uint8_t len)
{
    uint16_t idx = 0;
    uint16_t crc;

    proto->tx_buf[idx++] = PROTO_FRAME_HEAD1;
    proto->tx_buf[idx++] = PROTO_FRAME_HEAD2;
    proto->tx_buf[idx++] = cmd;
    proto->tx_buf[idx++] = len;

    if (len > 0 && data != NULL)
    {
        memcpy(&proto->tx_buf[idx], data, len);
        idx += len;
    }

    crc = crc16_ccitt(proto->tx_buf, idx);
    proto->tx_buf[idx++] = (uint8_t)(crc >> 8);
    proto->tx_buf[idx++] = (uint8_t)(crc & 0xFF);

    proto->tx_buf[idx++] = PROTO_FRAME_TAIL1;
    proto->tx_buf[idx++] = PROTO_FRAME_TAIL2;

    proto->tx_len = idx;
}

/* ============================================================
 *  内部: 通过 tx_func 发送 tx_buf
 * ============================================================ */
static uint8_t transmit(serial_proto_t *proto)
{
    if (proto == NULL || proto->tx_func == NULL || proto->tx_len == 0)
        return 1;

    int ret = proto->tx_func(proto->tx_buf, proto->tx_len);
    return (ret == 0) ? 0 : 1;
}

/* ============================================================
 *  发送: 通用响应帧
 * ============================================================ */
uint8_t serial_proto_send_response(serial_proto_t *proto, uint8_t cmd,
                                   uint8_t *data, uint8_t len)
{
    build_frame(proto, cmd, data, len);
    return transmit(proto);
}

uint8_t serial_proto_send_ack(serial_proto_t *proto, uint8_t cmd)
{
    return serial_proto_send_response(proto, CMD_ACK, &cmd, 1);
}

uint8_t serial_proto_send_nack(serial_proto_t *proto, uint8_t cmd,
                               uint8_t error_code)
{
    uint8_t payload[2];
    payload[0] = cmd;
    payload[1] = error_code;
    return serial_proto_send_response(proto, CMD_NACK, payload, 2);
}

uint8_t serial_proto_send_error(serial_proto_t *proto, uint8_t error_code)
{
    return serial_proto_send_response(proto, CMD_ERROR, &error_code, 1);
}

/* ============================================================
 *  逐字节喂入解析器 (9 状态状态机)
 *
 *  state 0: 搜索 0xAA
 *  state 1: 搜索 0x55
 *  state 2: 收命令字
 *  state 3: 收数据长度
 *  state 4: 收数据负载
 *  state 5: 收 CRC 高字节
 *  state 6: 收 CRC 低字节
 *  state 7: 收帧尾 0x55
 *  state 8: 收帧尾 0xAA → CRC校验 → 回调
 *
 *  任何一步匹配失败均回退到 state 0
 * ============================================================ */
void serial_proto_feed(serial_proto_t *proto, uint8_t byte)
{
    if (proto == NULL)
        return;

    switch (proto->rx_state)
    {
    case 0:
        if (byte == PROTO_FRAME_HEAD1)
        {
            proto->rx_buf[0] = byte;
            proto->rx_state = 1;
        }
        break;

    case 1:
        if (byte == PROTO_FRAME_HEAD2)
        {
            proto->rx_buf[1] = byte;
            proto->rx_index  = PROTO_HEAD_SIZE;
            proto->rx_state  = 2;
        }
        else
        {
            proto->rx_state = 0;
        }
        break;

    case 2:
        proto->rx_cmd = byte;
        proto->rx_buf[proto->rx_index++] = byte;
        proto->rx_state = 3;
        break;

    case 3:
        proto->rx_len = byte;
        proto->rx_buf[proto->rx_index++] = byte;
        if (proto->rx_len > PROTO_MAX_PAYLOAD)
        {
            proto->rx_state = 0;
        }
        else if (proto->rx_len > 0)
        {
            proto->rx_state = 4;
        }
        else
        {
            proto->rx_state = 5;
        }
        break;

    case 4:
        proto->rx_buf[proto->rx_index++] = byte;
        if (--proto->rx_len == 0)
            proto->rx_state = 5;
        break;

    case 5:
        proto->rx_buf[proto->rx_index++] = byte;
        proto->rx_state = 6;
        break;

    case 6:
        proto->rx_buf[proto->rx_index++] = byte;
        proto->rx_state = 7;
        break;

    case 7:
        if (byte == PROTO_FRAME_TAIL1)
        {
            proto->rx_buf[proto->rx_index++] = byte;
            proto->rx_state = 8;
        }
        else
        {
            proto->rx_state = 0;
        }
        break;

    case 8:
        if (byte == PROTO_FRAME_TAIL2)
        {
            proto->rx_buf[proto->rx_index++] = byte;

            uint16_t data_len = proto->rx_index - PROTO_HEAD_SIZE
                              - PROTO_CMD_SIZE - PROTO_LEN_SIZE
                              - PROTO_CRC_SIZE - PROTO_TAIL_SIZE;
            uint16_t calc_crc = crc16_ccitt(proto->rx_buf,
                                            proto->rx_index - PROTO_TAIL_SIZE
                                                           - PROTO_CRC_SIZE);
            uint16_t rx_crc  = ((uint16_t)proto->rx_buf[proto->rx_index - 4] << 8)
                             |  proto->rx_buf[proto->rx_index - 3];

            if (calc_crc == rx_crc && proto->on_command)
            {
                uint8_t *payload = proto->rx_buf + PROTO_HEAD_SIZE
                                 + PROTO_CMD_SIZE + PROTO_LEN_SIZE;
                proto->on_command(proto->rx_cmd, payload,
                                  (uint8_t)data_len);
            }
        }
        proto->rx_state = 0;
        break;

    default:
        proto->rx_state = 0;
        break;
    }
}

/* ============================================================
 *  批量喂入: 逐字节调用 serial_proto_feed
 *  适用于 UART ISR 一次性收到多个字节的场景
 * ============================================================ */
void serial_proto_feed_buf(serial_proto_t *proto, const uint8_t *data,
                           uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
        serial_proto_feed(proto, data[i]);
}
