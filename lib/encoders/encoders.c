#include "encoders.h"

#define LEFT_PIN  4   // PTD4
#define RIGHT_PIN 12  // PTA12

static const struct device *gpio_d;
static const struct device *gpio_a;

static struct gpio_callback left_cb;
static struct gpio_callback right_cb;

static volatile uint32_t ticks_left = 0;
static volatile uint32_t ticks_right = 0;

static void left_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ticks_left++;
}

static void right_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ticks_right++;
}

void encoders_init(void) {
    gpio_d = DEVICE_DT_GET(DT_NODELABEL(gpiod));
    gpio_a = DEVICE_DT_GET(DT_NODELABEL(gpioa));

    if (!device_is_ready(gpio_d) || !device_is_ready(gpio_a)) {
        printk("ERRO FATAL: Dispositivo GPIO A ou D não pronto!\n");
        return;
    }

    // Configura Pino Esquerda (PTD4)
    gpio_pin_configure(gpio_d, LEFT_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_d, LEFT_PIN, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&left_cb, left_isr, BIT(LEFT_PIN));
    gpio_add_callback(gpio_d, &left_cb);

    // Configura Pino Direita (PTA12)
    gpio_pin_configure(gpio_a, RIGHT_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_a, RIGHT_PIN, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&right_cb, right_isr, BIT(RIGHT_PIN));
    gpio_add_callback(gpio_a, &right_cb);

    printk("Encoders inicializados com sucesso.\n");
}

void encoders_reset(void) {
    ticks_left = 0;
    ticks_right = 0;
}

uint32_t encoders_get_left_ticks(void) { 
    return ticks_left; 
}

uint32_t encoders_get_right_ticks(void) { 
    return ticks_right; 
}

float encoders_get_distance_cm(void) {
    uint32_t avg_ticks = (ticks_left);
    return avg_ticks * CM_PER_TICK;
}