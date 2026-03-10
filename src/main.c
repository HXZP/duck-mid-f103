#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>

LOG_MODULE_REGISTER(ws2812, LOG_LEVEL_INF);

// get ws2812 node
#define LED_WS2812_NODE DT_NODELABEL(rgb_led)

/* WS2812 device */
static const struct device *ws2812_dev;
static struct led_rgb *pixels;

/**
 * @brief Initialize the WS2812 LED strip
 */
void Init_ws2812(void)
{
    size_t num_leds;

    ws2812_dev = DEVICE_DT_GET(LED_WS2812_NODE);
    __ASSERT(!device_is_ready(ws2812_dev), "Failed to get WS2812 device");

    num_leds = led_strip_length(ws2812_dev);
    __ASSERT(num_leds != 0, "WS2812 device has zero length");

    /* Allocate memory for LED pixels */
    pixels = k_malloc(sizeof(struct led_rgb) * num_leds);
    __ASSERT(pixels != NULL, "Failed to allocate memory for LED pixels");
}

/**
 * @brief Set LED color
 */
int set_Led_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    size_t num_leds;
    int ret;

    num_leds = led_strip_length(ws2812_dev);

    if (index >= num_leds) {
        LOG_ERR("LED index: %d out of range: %d", index, num_leds);
        return -EINVAL;
    }

    pixels[index].r = red;
    pixels[index].g = green;
    pixels[index].b = blue;

    ret = led_strip_update_rgb(ws2812_dev, pixels, num_leds);
    if (ret < 0) {
        LOG_ERR("Failed to update LED color: %d", ret);
        return ret;
    }
    return 0;
}

int main(void)
{
    LOG_INF("Hello, Duck Bot!");
    Init_ws2812();

    size_t num_leds = led_strip_length(ws2812_dev);
    uint16_t hue = 0;

    while (1) {
        /* Convert HSV to RGB for rainbow effect */
        for (size_t i = 0; i < num_leds; i++) {
            uint16_t current_hue = (hue + i * 360 / num_leds) % 360;
            uint8_t r, g, b;

            /* HSV to RGB conversion */
            uint16_t h = current_hue / 60;
            uint8_t f = (current_hue % 60) * 255 / 60;
            uint8_t p = 0;
            uint8_t q = 255 - f;
            uint8_t t = f;

            switch (h) {
                case 0: r = 255; g = t;   b = p; break;
                case 1: r = q;   g = 255; b = p; break;
                case 2: r = p;   g = 255; b = t; break;
                case 3: r = p;   g = q;   b = 255; break;
                case 4: r = t;   g = p;   b = 255; break;
                case 5: r = 255; g = p;   b = q; break;
                default: r = 0; g = 0; b = 0; break;
            }

            set_Led_color(i, r, g, b);
        }

        hue = (hue + 5) % 360;  /* Rotate hue */
        k_sleep(K_MSEC(50));    /* Delay for smooth animation */
    }

    return 0;
}
