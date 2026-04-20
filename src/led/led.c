/**
 * @file led.c
 * @brief 应用层 LED 控制实现。
 */
#include "led.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/** @brief RGB 灯带设备节点。 */
#define LED_RGB_NODE DT_NODELABEL(rgb_led)

/** @brief RGB 灯带灯珠数量。 */
#define LED_RGB_COUNT DT_PROP(LED_RGB_NODE, chain_length)

/** @brief 白灯亮度。 */
#define LED_WHITE_BRIGHTNESS 0x18

/** @brief 开机闪烁次数。 */
#define LED_BOOT_BLINK_COUNT 5

/** @brief 开机亮灯时长，单位毫秒。 */
#define LED_BOOT_ON_MS 50

/** @brief 开机关灯时长，单位毫秒。 */
#define LED_BOOT_OFF_MS 50

/** @brief RGB 灯带设备实例。 */
static const struct device *const led_rgb_dev = DEVICE_DT_GET(LED_RGB_NODE);

/** @brief RGB 灯带像素缓存。 */
static struct led_rgb led_pixels[LED_RGB_COUNT];

/** @brief RGB 灯带是否可用。 */
static bool led_ready;

/**
 * @brief 清空灯带缓存。
 * @return void
 */
static void led_clear_pixels(void)
{
	memset(led_pixels, 0, sizeof(led_pixels));
}

/**
 * @brief 将所有灯珠设置为同一颜色。
 * @param red 红色亮度。
 * @param green 绿色亮度。
 * @param blue 蓝色亮度。
 * @return void
 */
static void led_fill_all(uint8_t red, uint8_t green, uint8_t blue)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(led_pixels); i++)
	{
		led_pixels[i].r = red;
		led_pixels[i].g = green;
		led_pixels[i].b = blue;
	}
}

/**
 * @brief 刷新灯带显示。
 * @return int 0 表示成功，负值表示失败。
 */
static int led_flush(void)
{
	if (!led_ready)
	{
		return -ENODEV;
	}

	return led_strip_update_rgb(led_rgb_dev, led_pixels, ARRAY_SIZE(led_pixels));
}

/**
 * @brief 关闭所有灯珠。
 * @return void
 */
static void led_turn_off(void)
{
	int ret;

	if (!led_ready)
	{
		return;
	}

	led_clear_pixels();
	ret = led_flush();

	if (ret < 0)
	{
		led_ready = false;
	}
}

/**
 * @brief 初始化应用层 LED。
 * @return int 0 表示成功，负值表示失败。
 */
int led_init(void)
{
	int ret;

	if (!device_is_ready(led_rgb_dev))
	{
		return -ENODEV;
	}

	led_ready = true;
	led_clear_pixels();
	ret = led_flush();

	if (ret < 0)
	{
		led_ready = false;
		return ret;
	}

	return 0;
}

/**
 * @brief 执行开机白灯快速闪烁 5 次。
 * @return void
 */
void led_boot_blink(void)
{
	size_t i;
	int ret;

	if (!led_ready)
	{
		return;
	}

	for (i = 0; i < LED_BOOT_BLINK_COUNT; i++)
	{
		led_fill_all(LED_WHITE_BRIGHTNESS,
			     LED_WHITE_BRIGHTNESS,
			     LED_WHITE_BRIGHTNESS);
		ret = led_flush();

		if (ret < 0)
		{
			led_ready = false;
			return;
		}

		k_sleep(K_MSEC(LED_BOOT_ON_MS));
		led_turn_off();

		if (!led_ready)
		{
			return;
		}

		k_sleep(K_MSEC(LED_BOOT_OFF_MS));
	}
}
