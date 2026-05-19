/**
 * @file uart_protocol.c
 * @brief 串口帧协议实现。
 */
#include "uart_protocol.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(uart_protocol, LOG_LEVEL_INF);

/** @brief 串口接收环形缓冲区长度，单位字节。 */
#define UART_PROTOCOL_RX_RING_SIZE 256U

/** @brief 串口接收中断单次读取长度，单位字节。 */
#define UART_PROTOCOL_IRQ_READ_SIZE 32U

/** @brief 协议线程栈大小，单位字节。 */
#define UART_PROTOCOL_THREAD_STACK_SIZE 1536U

/** @brief 协议线程优先级。 */
#define UART_PROTOCOL_THREAD_PRIORITY 5

/** @brief 响应状态字段长度，单位字节。 */
#define UART_PROTOCOL_STATUS_SIZE 2U

/** @brief 帧头后固定头部长度，单位字节。 */
#define UART_PROTOCOL_HEADER_SIZE 8U

/** @brief CRC16 初始值。 */
#define UART_PROTOCOL_CRC16_INIT 0xFFFFU

/** @brief CRC16 多项式。 */
#define UART_PROTOCOL_CRC16_POLY 0x1021U

/** @brief 串口协议使用的设备。 */
static const struct device *const uart_protocol_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));

/** @brief 接收环形缓冲区存储。 */
static uint8_t uart_protocol_rx_ring_buffer[UART_PROTOCOL_RX_RING_SIZE];

/** @brief 接收环形缓冲区。 */
static struct ring_buf uart_protocol_rx_ring;

/** @brief 命令表。 */
static const struct uart_protocol_cmd_entry *uart_protocol_entries;

/** @brief 命令表项数量。 */
static size_t uart_protocol_entry_count;

/** @brief 协议模块是否已经初始化。 */
static bool uart_protocol_initialized;

/** @brief 协议线程是否已经启动。 */
static bool uart_protocol_thread_started;

/** @brief 串口协议互斥锁。 */
K_MUTEX_DEFINE(uart_protocol_mutex);

/** @brief 串口协议接收信号量。 */
K_SEM_DEFINE(uart_protocol_rx_sem, 0, 1);

/** @brief 协议线程栈。 */
K_THREAD_STACK_DEFINE(uart_protocol_thread_stack, UART_PROTOCOL_THREAD_STACK_SIZE);

/** @brief 协议线程控制块。 */
static struct k_thread uart_protocol_thread_data;

/**
 * @brief 帧解析状态。
 */
enum uart_protocol_parse_state
{
    /**< 等待帧头第 1 字节。 */
    UART_PROTOCOL_PARSE_WAIT_SOF1 = 0,
    /**< 等待帧头第 2 字节。 */
    UART_PROTOCOL_PARSE_WAIT_SOF2,
    /**< 读取固定头部。 */
    UART_PROTOCOL_PARSE_HEADER,
    /**< 读取负载数据。 */
    UART_PROTOCOL_PARSE_PAYLOAD,
    /**< 读取 CRC16。 */
    UART_PROTOCOL_PARSE_CRC,
};

/**
 * @brief 帧解析器运行数据。
 */
struct uart_protocol_parser
{
    /**< 当前解析状态。 */
    enum uart_protocol_parse_state state;
    /**< 固定头部缓冲区。 */
    uint8_t header[UART_PROTOCOL_HEADER_SIZE];
    /**< 固定头部已接收长度，单位字节。 */
    uint16_t header_pos;
    /**< 负载已接收长度，单位字节。 */
    uint16_t payload_pos;
    /**< CRC16 已接收长度，单位字节。 */
    uint16_t crc_pos;
    /**< 接收到的 CRC16。 */
    uint16_t rx_crc;
    /**< 当前协议帧。 */
    struct uart_protocol_frame frame;
};

/** @brief 帧解析器运行数据。 */
static struct uart_protocol_parser uart_protocol_parser_data;

/**
 * @brief 读取小端 16 位整数。
 * @param data 数据指针。
 * @return uint16_t 读取结果。
 */
static uint16_t uart_protocol_get_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief 写入小端 16 位整数。
 * @param data 数据指针。
 * @param value 待写入的值。
 * @return void
 */
static void uart_protocol_put_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/**
 * @brief 更新 CRC16。
 * @param crc 当前 CRC16。
 * @param data 输入字节。
 * @return uint16_t 更新后的 CRC16。
 */
static uint16_t uart_protocol_crc16_update(uint16_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= (uint16_t)data << 8;

    for (i = 0U; i < 8U; i++)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (crc << 1) ^ UART_PROTOCOL_CRC16_POLY;
        }
        else
        {
            crc <<= 1;
        }
    }

    return crc;
}

/**
 * @brief 计算 CRC16。
 * @param data 数据指针。
 * @param len 数据长度，单位字节。
 * @return uint16_t CRC16。
 */
static uint16_t uart_protocol_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = UART_PROTOCOL_CRC16_INIT;
    size_t i;

    for (i = 0U; i < len; i++)
    {
        crc = uart_protocol_crc16_update(crc, data[i]);
    }

    return crc;
}

/**
 * @brief 重置帧解析器。
 * @param parser 帧解析器。
 * @return void
 */
static void uart_protocol_parser_reset(struct uart_protocol_parser *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = UART_PROTOCOL_PARSE_WAIT_SOF1;
}

/**
 * @brief 查找命令表项。
 * @param cmd 命令号。
 * @return const struct uart_protocol_cmd_entry* 找到的命令表项，未找到返回 NULL。
 */
static const struct uart_protocol_cmd_entry *uart_protocol_find_entry(uint16_t cmd)
{
    size_t i;

    for (i = 0U; i < uart_protocol_entry_count; i++)
    {
        if (uart_protocol_entries[i].cmd == cmd)
        {
            return &uart_protocol_entries[i];
        }
    }

    return NULL;
}

/**
 * @brief 分发已完成校验的协议帧。
 * @param frame 协议帧。
 * @return void
 */
static void uart_protocol_dispatch_frame(const struct uart_protocol_frame *frame)
{
    const struct uart_protocol_cmd_entry *entry;

    k_mutex_lock(&uart_protocol_mutex, K_FOREVER);

    if (!uart_protocol_initialized)
    {
        k_mutex_unlock(&uart_protocol_mutex);
        return;
    }

    entry = uart_protocol_find_entry(frame->cmd);
    k_mutex_unlock(&uart_protocol_mutex);

    if ((entry == NULL) || (entry->handler == NULL))
    {
        LOG_WRN("UART command not found: cmd=0x%04x seq=%u", frame->cmd, frame->seq);
        uart_protocol_send_response(frame->cmd, frame->seq, -ENOENT, NULL, 0U);
        return;
    }

    entry->handler(frame, entry->user_data);
}

/**
 * @brief 校验并提交协议帧。
 * @param parser 帧解析器。
 * @return void
 */
static void uart_protocol_submit_frame(struct uart_protocol_parser *parser)
{
    uint8_t crc_buffer[UART_PROTOCOL_HEADER_SIZE + UART_PROTOCOL_MAX_PAYLOAD_LEN];
    uint16_t calc_crc;

    memcpy(crc_buffer, parser->header, UART_PROTOCOL_HEADER_SIZE);

    if (parser->frame.payload_len > 0U)
    {
        memcpy(&crc_buffer[UART_PROTOCOL_HEADER_SIZE],
               parser->frame.payload,
               parser->frame.payload_len);
    }

    calc_crc = uart_protocol_crc16(crc_buffer,
                                   UART_PROTOCOL_HEADER_SIZE + parser->frame.payload_len);

    if (calc_crc != parser->rx_crc)
    {
        LOG_WRN("UART frame crc mismatch: cmd=0x%04x seq=%u calc=0x%04x rx=0x%04x",
                parser->frame.cmd,
                parser->frame.seq,
                calc_crc,
                parser->rx_crc);
        return;
    }

    uart_protocol_dispatch_frame(&parser->frame);
}

/**
 * @brief 完成固定头部解析。
 * @param parser 帧解析器。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_protocol_finish_header(struct uart_protocol_parser *parser)
{
    parser->frame.version = parser->header[0];
    parser->frame.flags = parser->header[1];
    parser->frame.cmd = uart_protocol_get_le16(&parser->header[2]);
    parser->frame.seq = uart_protocol_get_le16(&parser->header[4]);
    parser->frame.payload_len = uart_protocol_get_le16(&parser->header[6]);

    if (parser->frame.version != UART_PROTOCOL_VERSION)
    {
        LOG_WRN("UART frame version mismatch: version=%u", parser->frame.version);
        return -EPROTO;
    }

    if (parser->frame.payload_len > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        LOG_WRN("UART frame payload too large: len=%u", parser->frame.payload_len);
        return -EMSGSIZE;
    }

    return 0;
}

/**
 * @brief 向帧解析器输入 1 字节。
 * @param parser 帧解析器。
 * @param byte 输入字节。
 * @return void
 */
static void uart_protocol_parse_byte(struct uart_protocol_parser *parser, uint8_t byte)
{
    switch (parser->state)
    {
    case UART_PROTOCOL_PARSE_WAIT_SOF1:
        if (byte == UART_PROTOCOL_SOF1)
        {
            parser->state = UART_PROTOCOL_PARSE_WAIT_SOF2;
        }
        break;

    case UART_PROTOCOL_PARSE_WAIT_SOF2:
        if (byte == UART_PROTOCOL_SOF2)
        {
            parser->state = UART_PROTOCOL_PARSE_HEADER;
            parser->header_pos = 0U;
        }
        else if (byte != UART_PROTOCOL_SOF1)
        {
            parser->state = UART_PROTOCOL_PARSE_WAIT_SOF1;
        }
        break;

    case UART_PROTOCOL_PARSE_HEADER:
        parser->header[parser->header_pos] = byte;
        parser->header_pos++;

        if (parser->header_pos >= UART_PROTOCOL_HEADER_SIZE)
        {
            if (uart_protocol_finish_header(parser) < 0)
            {
                uart_protocol_parser_reset(parser);
            }
            else if (parser->frame.payload_len == 0U)
            {
                parser->state = UART_PROTOCOL_PARSE_CRC;
                parser->crc_pos = 0U;
                parser->rx_crc = 0U;
            }
            else
            {
                parser->state = UART_PROTOCOL_PARSE_PAYLOAD;
                parser->payload_pos = 0U;
            }
        }
        break;

    case UART_PROTOCOL_PARSE_PAYLOAD:
        parser->frame.payload[parser->payload_pos] = byte;
        parser->payload_pos++;

        if (parser->payload_pos >= parser->frame.payload_len)
        {
            parser->state = UART_PROTOCOL_PARSE_CRC;
            parser->crc_pos = 0U;
            parser->rx_crc = 0U;
        }
        break;

    case UART_PROTOCOL_PARSE_CRC:
        if (parser->crc_pos == 0U)
        {
            parser->rx_crc = byte;
            parser->crc_pos = 1U;
        }
        else
        {
            parser->rx_crc |= (uint16_t)byte << 8;
            uart_protocol_submit_frame(parser);
            uart_protocol_parser_reset(parser);
        }
        break;

    default:
        uart_protocol_parser_reset(parser);
        break;
    }
}

/**
 * @brief 串口中断回调。
 * @param dev 串口设备。
 * @param user_data 用户参数。
 * @return void
 */
static void uart_protocol_irq_callback(const struct device *dev, void *user_data)
{
    uint8_t data[UART_PROTOCOL_IRQ_READ_SIZE];

    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
    {
        int rx_len;

        rx_len = uart_fifo_read(dev, data, sizeof(data));

        if (rx_len <= 0)
        {
            continue;
        }

        if (ring_buf_put(&uart_protocol_rx_ring, data, (uint32_t)rx_len) < (uint32_t)rx_len)
        {
            LOG_WRN("UART rx ring overflow");
        }

        k_sem_give(&uart_protocol_rx_sem);
    }
}

/**
 * @brief 串口协议线程入口。
 * @param arg1 线程参数 1。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void uart_protocol_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        uint8_t byte;

        k_sem_take(&uart_protocol_rx_sem, K_FOREVER);

        while (ring_buf_get(&uart_protocol_rx_ring, &byte, 1U) == 1U)
        {
            uart_protocol_parse_byte(&uart_protocol_parser_data, byte);
        }
    }
}

/**
 * @brief 启动协议线程。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_protocol_start_thread(void)
{
    if (uart_protocol_thread_started)
    {
        return 0;
    }

    k_thread_create(&uart_protocol_thread_data,
                    uart_protocol_thread_stack,
                    K_THREAD_STACK_SIZEOF(uart_protocol_thread_stack),
                    uart_protocol_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    UART_PROTOCOL_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    uart_protocol_thread_started = true;
    return 0;
}

int uart_protocol_init(const struct uart_protocol_cmd_entry *entries,
                       size_t entry_count)
{
    int ret;

    if ((entries == NULL) || (entry_count == 0U))
    {
        return -EINVAL;
    }

    if (!device_is_ready(uart_protocol_dev))
    {
        return -ENODEV;
    }

    ret = uart_protocol_start_thread();

    if (ret < 0)
    {
        return ret;
    }

    k_mutex_lock(&uart_protocol_mutex, K_FOREVER);
    uart_protocol_entries = entries;
    uart_protocol_entry_count = entry_count;
    uart_protocol_initialized = true;
    ring_buf_init(&uart_protocol_rx_ring,
                  sizeof(uart_protocol_rx_ring_buffer),
                  uart_protocol_rx_ring_buffer);
    uart_protocol_parser_reset(&uart_protocol_parser_data);
    k_mutex_unlock(&uart_protocol_mutex);

    uart_irq_callback_user_data_set(uart_protocol_dev,
                                    uart_protocol_irq_callback,
                                    NULL);
    uart_irq_rx_enable(uart_protocol_dev);

    LOG_INF("UART protocol init done");
    return 0;
}

void uart_protocol_deinit(void)
{
    uart_irq_rx_disable(uart_protocol_dev);
    uart_irq_callback_user_data_set(uart_protocol_dev, NULL, NULL);

    k_mutex_lock(&uart_protocol_mutex, K_FOREVER);
    uart_protocol_initialized = false;
    uart_protocol_entries = NULL;
    uart_protocol_entry_count = 0U;
    ring_buf_reset(&uart_protocol_rx_ring);
    uart_protocol_parser_reset(&uart_protocol_parser_data);
    k_mutex_unlock(&uart_protocol_mutex);
}

int uart_protocol_send_frame(uint16_t cmd,
                             uint16_t seq,
                             uint8_t flags,
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    uint8_t frame_buffer[2U + UART_PROTOCOL_HEADER_SIZE + UART_PROTOCOL_MAX_PAYLOAD_LEN + 2U];
    uint16_t crc;
    size_t pos = 0U;
    size_t crc_start;
    size_t i;

    if ((payload == NULL) && (payload_len > 0U))
    {
        return -EINVAL;
    }

    if (payload_len > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        return -EMSGSIZE;
    }

    frame_buffer[pos] = UART_PROTOCOL_SOF1;
    pos++;
    frame_buffer[pos] = UART_PROTOCOL_SOF2;
    pos++;

    crc_start = pos;

    frame_buffer[pos] = UART_PROTOCOL_VERSION;
    pos++;
    frame_buffer[pos] = flags;
    pos++;
    uart_protocol_put_le16(&frame_buffer[pos], cmd);
    pos += 2U;
    uart_protocol_put_le16(&frame_buffer[pos], seq);
    pos += 2U;
    uart_protocol_put_le16(&frame_buffer[pos], payload_len);
    pos += 2U;

    if (payload_len > 0U)
    {
        memcpy(&frame_buffer[pos], payload, payload_len);
        pos += payload_len;
    }

    crc = uart_protocol_crc16(&frame_buffer[crc_start], pos - crc_start);
    uart_protocol_put_le16(&frame_buffer[pos], crc);
    pos += 2U;

    for (i = 0U; i < pos; i++)
    {
        uart_poll_out(uart_protocol_dev, frame_buffer[i]);
    }

    return 0;
}

int uart_protocol_send_response(uint16_t req_cmd,
                                uint16_t seq,
                                int16_t status,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    uint8_t rsp_payload[UART_PROTOCOL_MAX_PAYLOAD_LEN];
    uint16_t rsp_payload_len;

    if ((payload == NULL) && (payload_len > 0U))
    {
        return -EINVAL;
    }

    if ((payload_len + UART_PROTOCOL_STATUS_SIZE) > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        return -EMSGSIZE;
    }

    uart_protocol_put_le16(rsp_payload, (uint16_t)status);
    rsp_payload_len = UART_PROTOCOL_STATUS_SIZE;

    if (payload_len > 0U)
    {
        memcpy(&rsp_payload[rsp_payload_len], payload, payload_len);
        rsp_payload_len += payload_len;
    }

    return uart_protocol_send_frame(req_cmd | UART_PROTOCOL_RSP_FLAG,
                                    seq,
                                    0U,
                                    rsp_payload,
                                    rsp_payload_len);
}
