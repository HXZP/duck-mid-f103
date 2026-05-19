/**
 * @file uart_command.h
 * @brief 串口业务命令接口。
 */
#ifndef UART_COMMAND_H_
#define UART_COMMAND_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化串口业务命令。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_command_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_COMMAND_H_ */
