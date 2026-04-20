/**
 * @file led.h
 * @brief 应用层 LED 控制接口。
 */
#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层 LED。
 * @return int 0 表示成功，负值表示失败。
 */
int led_init(void);

/**
 * @brief 执行开机白灯快速闪烁 5 次。
 * @return void
 */
void led_boot_blink(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */
