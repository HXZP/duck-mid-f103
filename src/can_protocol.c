/**
 * @file can_protocol.c
 * @brief CAN 传输层实现。
 */
#include "can_protocol.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_protocol, LOG_LEVEL_INF);

/** @brief 默认 CAN 波特率，单位 bit/s。 */
#define CAN_PROTOCOL_BAUDRATE 500000U

/** @brief 接收消息队列长度。 */
#define CAN_PROTOCOL_RX_QUEUE_LEN 16

/** @brief 发送消息队列长度。 */
#define CAN_PROTOCOL_TX_QUEUE_LEN 16

/** @brief 发送入队等待时间。 */
#define CAN_PROTOCOL_TX_QUEUE_TIMEOUT K_MSEC(100)

/** @brief 发送超时时间。 */
#define CAN_PROTOCOL_TX_TIMEOUT K_MSEC(100)

/** @brief 无效过滤器 ID。 */
#define CAN_PROTOCOL_INVALID_FILTER_ID (-1)

/** @brief 接收线程栈大小。 */
#define CAN_PROTOCOL_THREAD_STACK_SIZE 1024

/** @brief 发送线程栈大小。 */
#define CAN_PROTOCOL_TX_THREAD_STACK_SIZE 1024

/** @brief 接收线程优先级。 */
#define CAN_PROTOCOL_THREAD_PRIORITY 5

/** @brief 发送线程优先级，数值越大优先级越低。 */
#define CAN_PROTOCOL_TX_THREAD_PRIORITY 6

/** @brief CAN 接收消息队列。 */
CAN_MSGQ_DEFINE(can_protocol_rx_msgq, CAN_PROTOCOL_RX_QUEUE_LEN);

/** @brief CAN 发送消息队列。 */
CAN_MSGQ_DEFINE(can_protocol_tx_msgq, CAN_PROTOCOL_TX_QUEUE_LEN);

/** @brief 当前使用的 CAN 控制器设备。 */
static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

/** @brief 当前安装的过滤器 ID 列表。 */
static int can_protocol_filter_ids[CAN_PROTOCOL_MAX_FILTER_COUNT];

/** @brief 当前安装的过滤器数量。 */
static size_t can_protocol_filter_count;

/** @brief 接收回调函数。 */
static can_protocol_rx_handler_t can_protocol_rx_handler;

/** @brief 接收回调的用户参数。 */
static void *can_protocol_rx_user_data;

/** @brief 传输层是否已经初始化。 */
static bool can_protocol_initialized;

/** @brief 接收线程是否已经启动。 */
static bool can_protocol_thread_started;

/** @brief 发送线程是否已经启动。 */
static bool can_protocol_tx_thread_started;

/** @brief 传输层互斥锁。 */
K_MUTEX_DEFINE(can_protocol_mutex);

/** @brief 接收线程栈。 */
K_THREAD_STACK_DEFINE(can_protocol_thread_stack, CAN_PROTOCOL_THREAD_STACK_SIZE);

/** @brief 接收线程控制块。 */
static struct k_thread can_protocol_thread_data;

/** @brief 发送线程栈。 */
K_THREAD_STACK_DEFINE(can_protocol_tx_thread_stack, CAN_PROTOCOL_TX_THREAD_STACK_SIZE);

/** @brief 发送线程控制块。 */
static struct k_thread can_protocol_tx_thread_data;

/**
 * @brief 重置过滤器记录。
 * @return void
 */
static void can_protocol_reset_filter_records(void)
{
    size_t i;

    for (i = 0U; i < CAN_PROTOCOL_MAX_FILTER_COUNT; i++)
    {
        can_protocol_filter_ids[i] = CAN_PROTOCOL_INVALID_FILTER_ID;
    }

    can_protocol_filter_count = 0U;
}

/**
 * @brief 移除当前已安装的所有过滤器。
 * @return void
 */
static void can_protocol_remove_filters(void)
{
    size_t i;

    for (i = 0U; i < can_protocol_filter_count; i++)
    {
        if (can_protocol_filter_ids[i] >= 0)
        {
            can_remove_rx_filter(can_dev, can_protocol_filter_ids[i]);
            can_protocol_filter_ids[i] = CAN_PROTOCOL_INVALID_FILTER_ID;
        }
    }

    can_protocol_filter_count = 0U;
}

/**
 * @brief 安装 CAN 接收过滤器。
 * @param filters 过滤器配置列表。
 * @param filter_count 过滤器数量。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_protocol_add_filters(const struct can_protocol_filter_config *filters,
                                    size_t filter_count)
{
    size_t i;

    can_protocol_reset_filter_records();

    for (i = 0U; i < filter_count; i++)
    {
        const struct can_filter filter = {
            .id = filters[i].id,
            .mask = filters[i].mask,
            .flags = filters[i].flags,
        };
        int ret;

        ret = can_add_rx_filter_msgq(can_dev, &can_protocol_rx_msgq, &filter);

        if (ret < 0)
        {
            can_protocol_remove_filters();
            return ret;
        }

        can_protocol_filter_ids[i] = ret;
        can_protocol_filter_count++;
    }

    return 0;
}

/**
 * @brief CAN 控制器状态变化回调。
 * @param dev CAN 设备指针。
 * @param state 新的总线状态。
 * @param err_cnt 当前总线错误计数。
 * @param user_data 用户参数。
 * @return void
 */
static void can_protocol_state_change_cb(const struct device *dev,
                                         enum can_state state,
                                         struct can_bus_err_cnt err_cnt,
                                         void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    LOG_WRN("CAN state changed: state=%d tx_err=%u rx_err=%u",
            state,
            err_cnt.tx_err_cnt,
            err_cnt.rx_err_cnt);
}

/**
 * @brief CAN 接收线程入口。
 * @param arg1 线程参数 1。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void can_protocol_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        struct can_frame frame;
        can_protocol_rx_handler_t rx_handler;
        void *user_data;

        if (k_msgq_get(&can_protocol_rx_msgq, &frame, K_FOREVER) != 0)
        {
            continue;
        }

        k_mutex_lock(&can_protocol_mutex, K_FOREVER);

        if (!can_protocol_initialized || (can_protocol_rx_handler == NULL))
        {
            k_mutex_unlock(&can_protocol_mutex);
            continue;
        }

        rx_handler = can_protocol_rx_handler;
        user_data = can_protocol_rx_user_data;
        k_mutex_unlock(&can_protocol_mutex);

        rx_handler(&frame, user_data);
    }
}

/**
 * @brief CAN 发送线程入口。
 * @param arg1 线程参数 1。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void can_protocol_tx_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        struct can_frame frame;
        bool initialized;
        int ret;

        if (k_msgq_get(&can_protocol_tx_msgq, &frame, K_FOREVER) != 0)
        {
            continue;
        }

        k_mutex_lock(&can_protocol_mutex, K_FOREVER);
        initialized = can_protocol_initialized;
        k_mutex_unlock(&can_protocol_mutex);

        if (!initialized)
        {
            continue;
        }

        ret = can_send(can_dev, &frame, CAN_PROTOCOL_TX_TIMEOUT, NULL, NULL);

        if (ret < 0)
        {
            LOG_ERR("CAN send failed: id=0x%03x ret=%d", frame.id, ret);
        }
    }
}

/**
 * @brief 启动 CAN 接收线程。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_protocol_start_thread(void)
{
    if (can_protocol_thread_started)
    {
        return 0;
    }

    k_thread_create(&can_protocol_thread_data,
                    can_protocol_thread_stack,
                    K_THREAD_STACK_SIZEOF(can_protocol_thread_stack),
                    can_protocol_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    CAN_PROTOCOL_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    can_protocol_thread_started = true;
    return 0;
}

/**
 * @brief 启动 CAN 发送线程。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_protocol_start_tx_thread(void)
{
    if (can_protocol_tx_thread_started)
    {
        return 0;
    }

    k_thread_create(&can_protocol_tx_thread_data,
                    can_protocol_tx_thread_stack,
                    K_THREAD_STACK_SIZEOF(can_protocol_tx_thread_stack),
                    can_protocol_tx_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    CAN_PROTOCOL_TX_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    can_protocol_tx_thread_started = true;
    return 0;
}

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
                      void *user_data)
{
    int ret;

    if ((filters == NULL) || (filter_count == 0U) ||
        (filter_count > CAN_PROTOCOL_MAX_FILTER_COUNT) ||
        (rx_handler == NULL))
    {
        return -EINVAL;
    }

    if (!device_is_ready(can_dev))
    {
        return -ENODEV;
    }

    ret = can_protocol_start_thread();

    if (ret < 0)
    {
        return ret;
    }

    ret = can_protocol_start_tx_thread();

    if (ret < 0)
    {
        return ret;
    }

    if (can_protocol_initialized)
    {
        can_protocol_deinit();
    }

    k_mutex_lock(&can_protocol_mutex, K_FOREVER);
    can_protocol_rx_handler = rx_handler;
    can_protocol_rx_user_data = user_data;
    can_protocol_initialized = false;
    k_msgq_purge(&can_protocol_rx_msgq);
    k_msgq_purge(&can_protocol_tx_msgq);
    can_protocol_reset_filter_records();
    k_mutex_unlock(&can_protocol_mutex);

    ret = can_stop(can_dev);

    if ((ret < 0) && (ret != -EALREADY))
    {
        return ret;
    }

    /*
     * CAN_MODE_NORMAL 不包含 CAN_MODE_ONE_SHOT，控制器会保持自动重传。
     */
    ret = can_set_mode(can_dev, CAN_MODE_NORMAL);

    if (ret < 0)
    {
        return ret;
    }

    ret = can_set_bitrate(can_dev, CAN_PROTOCOL_BAUDRATE);

    if (ret < 0)
    {
        return ret;
    }

    can_set_state_change_callback(can_dev, can_protocol_state_change_cb, NULL);

    ret = can_start(can_dev);

    if (ret < 0)
    {
        can_set_state_change_callback(can_dev, NULL, NULL);
        return ret;
    }

    ret = can_protocol_add_filters(filters, filter_count);

    if (ret < 0)
    {
        can_set_state_change_callback(can_dev, NULL, NULL);
        can_stop(can_dev);
        return ret;
    }

    k_mutex_lock(&can_protocol_mutex, K_FOREVER);
    can_protocol_initialized = true;
    k_mutex_unlock(&can_protocol_mutex);

    LOG_INF("CAN transport init done: bitrate=%u filters=%u tx_queue=%u",
            CAN_PROTOCOL_BAUDRATE,
            (unsigned int)filter_count,
            CAN_PROTOCOL_TX_QUEUE_LEN);

    return 0;
}

/**
 * @brief 反初始化 CAN 传输层。
 * @return void
 */
void can_protocol_deinit(void)
{
    k_mutex_lock(&can_protocol_mutex, K_FOREVER);
    can_protocol_initialized = false;
    can_protocol_rx_handler = NULL;
    can_protocol_rx_user_data = NULL;
    k_mutex_unlock(&can_protocol_mutex);

    can_protocol_remove_filters();
    can_set_state_change_callback(can_dev, NULL, NULL);
    can_stop(can_dev);
    k_msgq_purge(&can_protocol_rx_msgq);
    k_msgq_purge(&can_protocol_tx_msgq);

    k_mutex_lock(&can_protocol_mutex, K_FOREVER);
    can_protocol_reset_filter_records();
    k_mutex_unlock(&can_protocol_mutex);
}

/**
 * @brief 将 CAN 报文放入发送队列。
 * @param frame 待发送的 CAN 报文指针。
 * @return int 0 表示成功放入队列，负值表示失败。
 */
int can_protocol_send(const struct can_frame *frame)
{
    bool initialized;
    int ret;

    if (frame == NULL)
    {
        return -EINVAL;
    }

    if (frame->dlc > CAN_MAX_DLC)
    {
        return -EMSGSIZE;
    }

    k_mutex_lock(&can_protocol_mutex, K_FOREVER);
    initialized = can_protocol_initialized;
    k_mutex_unlock(&can_protocol_mutex);

    if (!initialized)
    {
        return -ENODEV;
    }

    ret = k_msgq_put(&can_protocol_tx_msgq,
                     frame,
                     CAN_PROTOCOL_TX_QUEUE_TIMEOUT);

    if (ret < 0)
    {
        LOG_ERR("CAN tx queue put failed: id=0x%03x ret=%d", frame->id, ret);
    }

    return ret;
}

/**
 * @brief 将标准帧数据报文放入发送队列。
 * @param id 标准帧 ID。
 * @param data 待发送的数据指针。
 * @param dlc 数据长度。
 * @return int 0 表示成功放入队列，负值表示失败。
 */
int can_protocol_send_data(uint32_t id,
                           const uint8_t *data,
                           uint8_t dlc)
{
    struct can_frame frame;

    if ((data == NULL) && (dlc > 0U))
    {
        return -EINVAL;
    }

    if (dlc > CAN_MAX_DLC)
    {
        return -EMSGSIZE;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.dlc = dlc;
    frame.flags = 0U;

    if (dlc > 0U)
    {
        memcpy(frame.data, data, dlc);
    }

    return can_protocol_send(&frame);
}
