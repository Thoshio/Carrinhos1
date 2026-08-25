#include <pwm_z402.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <ultrassonic.h>               // Biblioteca personalizada

#define TPM_MODULE 22500

uint16_t value_pwm(int duty) {
    return (uint16_t)TPM_MODULE * duty/100;
}

int main(void)
{   
    printk("Iniciando programa...\n");
    ultrassonic_init();

    // 1. Busca o dispositivo usando a macro moderna do Devicetree
    const struct device *gpio_c = DEVICE_DT_GET(DT_NODELABEL(gpioc));

    // 2. VERIFICA O DISPOSITIVO IMEDIATAMENTE (antes de tentar configurar)
    if (gpio_c == NULL || !device_is_ready(gpio_c)) {
        printk("ERRO FATAL: Dispositivo GPIOC não encontrado ou não está pronto!\n");
        return -1; // Para o programa aqui, pois não dá para continuar
    }

    printk("GPIOC encontrado com sucesso!\n");

    // 3. Agora é seguro configurar os pinos
    if (gpio_pin_configure(gpio_c, 7, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Erro ao configurar PTC7\n");
    }
    gpio_pin_configure(gpio_c, 0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(gpio_c, 3, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(gpio_c, 4, GPIO_OUTPUT_ACTIVE);

    // 4. Define os estados iniciais
    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 0);

    printk("Pinos configurados!\n");


    pwm_tpm_Init(TPM0, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM0, 1, TPM_PWM_H, GPIOA, 4);
    pwm_tpm_Ch_Init(TPM0, 2, TPM_PWM_H, GPIOA, 5);

    pwm_tpm_CnV(TPM0, 1, value_pwm(0));
    pwm_tpm_CnV(TPM0, 2, value_pwm(0));
    k_msleep(5000);

    for(;;) {
    print_distance();

    printk("indo\n");
    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 1);
    gpio_pin_set(gpio_c, 4, 0);

    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(50));

    k_msleep(5000);
    printk("parou\n");
    pwm_tpm_CnV(TPM0, 1, value_pwm(1));
    pwm_tpm_CnV(TPM0, 2, value_pwm(1));

    k_msleep(5000);
    printk("voltou\n");
    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));
    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 1);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);
    k_msleep(5000);
    }
    
    return 0;
}