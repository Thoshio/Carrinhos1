#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define left_pin 4

//static const struct device *gpio_c;
static const struct device *gpio_d;
static struct gpio_callback left_data;

void encoder_init(void) {
    gpio_init();
}

// ISR - encoder da esquerda
void left_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int val = gpio_pin_get(dev, left_pin);
    printk("esquerda: %d\n", val);
}

void gpio_init(void) {
    // Encoder da direita (visão carrinho) GPIOC_9-> trocar para PTA12

    // Encoder da esquerda (visão carrinho) GPIOD_4
    gpio_d = DEVICE_DT_GET(DT_NODELABEL(gpiod));

    if (gpio_d == NULL || !device_is_ready(gpio_d)) {
        printk("ERRO FATAL: Dispositivo GPIOC não encontrado ou não está pronto!\n");
    }

    if (gpio_pin_configure(gpio_d, left_pin, GPIO_INPUT) < 0) { 
        printk("Erro ao configurar PTC7\n");
    }

    // Interrupção esquerda
    gpio_pin_interrupt_configure(gpio_d, left_pin, GPIO_INT_EDGE_FALLING | GPIO_INT_EDGE_RISING);
    gpio_init_callback(&left_data, left_isr, BIT(left_pin));
    gpio_add_callback(gpio_d, &left_data);

}


int main(void)
{   
    printk("Iniciando robô no modo simples...\n");

    gpio_init();

    // Lógica de desvio em loop infinito
    for(;;) {
    }

    return 0;
}