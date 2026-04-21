/**
 * @file can_protocol.h
 * @brief CAN 传输层接口。
 */
#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 可注册的最大接收过滤器数量。 */
#define CAN_PROTOCOL_MAX_FILTER_COUNT 8U

/**
 * @brief CAN 接收过滤器配置。
 */
struct can_protocol_filter_config
{
    /**< 过滤器 ID。 */
    uint32_t id;
    /**< 过滤器掩码。 */
    uint32_t mask;
    /**< 过滤器标志。 */
    uint8_t flags;
};

/**
 * @brief CAN 接收回调函数类型。
 */
typedef void (*can_protocol_rx_handler_t)(const struct can_frame *frame,
                                          void *user_data);

/**
 * @brief 初始化 CAN 传输层。
 * @param filters 过滤器配置列表。
 * @param filter_count 过滤器数量。
 * @param rx_handler 接收回调函数。
 * @param user_data 传递给接收回调的用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
int can_protocol_init(const struct can_protocol_filter_config *filters,
                      size_t filter_count,
                      can_protocol_rx_handler_t rx_handler,
                      void *user_data);

/**
 * @brief 反初始化 CAN 传输层。
 * @return void
 */
void can_protocol_deinit(void);

/**
 * @brief 发送一个 CAN 报文。
 * @param frame 待发送的 CAN 报文指针。
 * @return int 0 表示成功，负值表示失败。
 */
int can_protocol_send(const struct can_frame *frame);

/**
 * @brief 发送一个标准帧数据报文。
 * @param id 标准帧 ID。
 * @param data 待发送的数据指针。
 * @param dlc 数据长度。
 * @return int 0 表示成功，负值表示失败。
 */
int can_protocol_send_data(uint32_t id,
                           const uint8_t *data,
                           uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H_ */
