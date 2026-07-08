/**
 * @file can_command.h
 * @brief CAN 业务调试命令接口。
 */
#ifndef CAN_COMMAND_H_
#define CAN_COMMAND_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化 CAN 业务调试命令模块。
 * @return int 0 表示成功，负值表示失败。
 */
int can_command_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMMAND_H_ */
