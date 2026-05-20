/**
 * @file uart_protocol.h
 * @brief 串口帧协议接口。
 */
#ifndef UART_PROTOCOL_H_
#define UART_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 协议帧头第 1 字节。 */
#define UART_PROTOCOL_SOF1 0xA5U

/** @brief 协议帧头第 2 字节。 */
#define UART_PROTOCOL_SOF2 0x5AU

/** @brief 当前协议版本号。 */
#define UART_PROTOCOL_VERSION 0x01U

/** @brief 响应命令标志位。 */
#define UART_PROTOCOL_RSP_FLAG 0x8000U

/** @brief 最大负载长度，单位字节。 */
#define UART_PROTOCOL_MAX_PAYLOAD_LEN 128U

/**
 * @brief 串口协议帧。
 */
struct uart_protocol_frame
{
    /**< 协议版本号。 */
    uint8_t version;
    /**< 协议标志位。 */
    uint8_t flags;
    /**< 命令号。 */
    uint16_t cmd;
    /**< 帧序号。 */
    uint16_t seq;
    /**< 负载长度，单位字节。 */
    uint16_t payload_len;
    /**< 负载数据。 */
    uint8_t payload[UART_PROTOCOL_MAX_PAYLOAD_LEN];
};

/**
 * @brief 串口命令回调函数类型。
 * @param frame 已完成校验的协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
typedef int (*uart_protocol_handler_t)(const struct uart_protocol_frame *frame,
                                       void *user_data);

/**
 * @brief 串口命令表项。
 */
struct uart_protocol_cmd_entry
{
    /**< 命令号。 */
    uint16_t cmd;
    /**< 命令回调函数。 */
    uart_protocol_handler_t handler;
    /**< 用户参数。 */
    void *user_data;
};

/**
 * @brief 初始化串口协议模块。
 * @param entries 命令表。
 * @param entry_count 命令表项数量。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_init(const struct uart_protocol_cmd_entry *entries,
                       size_t entry_count);

/**
 * @brief 反初始化串口协议模块。
 * @return void
 */
void uart_protocol_deinit(void);

/**
 * @brief 发送串口协议帧。
 * @param cmd 命令号。
 * @param seq 帧序号。
 * @param flags 协议标志位。
 * @param payload 负载数据。
 * @param payload_len 负载长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
/**
 * @brief 发送串口原始调试数据。
 * @param data 待发送数据指针。
 * @param len 待发送数据长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_send_raw(const uint8_t *data, uint16_t len);

int uart_protocol_send_frame(uint16_t cmd,
                             uint16_t seq,
                             uint8_t flags,
                             const uint8_t *payload,
                             uint16_t payload_len);

/**
 * @brief 发送命令响应帧。
 * @param req_cmd 请求命令号。
 * @param seq 请求帧序号。
 * @param status 运行状态。
 * @param payload 响应负载数据。
 * @param payload_len 响应负载长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_send_response(uint16_t req_cmd,
                                uint16_t seq,
                                int16_t status,
                                const uint8_t *payload,
                                uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H_ */
