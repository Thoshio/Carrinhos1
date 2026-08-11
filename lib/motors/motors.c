#include "motors.h"

static const struct device *gpio_c;

void motors_init(void){
    gpio_init();
    pwm_init();
    printk("Motores configurados!\n");
}

void gpio_init(void){
    //Busca o dispositivo usando a macro moderna do Devicetree
    gpio_c = DEVICE_DT_GET(DT_NODELABEL(gpioc));

    //VERIFICA O DISPOSITIVO IMEDIATAMENTE (antes de tentar configurar)
    if (gpio_c == NULL || !device_is_ready(gpio_c)) {
        printk("ERRO FATAL: Dispositivo GPIOC não encontrado ou não está pronto!\n");
    }

    printk("GPIOC encontrado com sucesso!\n");

    //Agora é seguro configurar os pinos
    if (gpio_pin_configure(gpio_c, 7, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Erro ao configurar PTC7\n");
    }
    gpio_pin_configure(gpio_c, 0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(gpio_c, 3, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(gpio_c, 4, GPIO_OUTPUT_ACTIVE);

    //Define os estados iniciais
    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 0);
}

void pwm_init(void){
    pwm_tpm_Init(TPM0, TPM_PLLFLL, TPM_MODULE_MOTORS, TPM_CLK, PS_4, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM0, 1, TPM_PWM_H, GPIOA, 4);
    pwm_tpm_Ch_Init(TPM0, 2, TPM_PWM_H, GPIOA, 5);

    pwm_tpm_CnV(TPM0, 1, value_pwm(0));
    pwm_tpm_CnV(TPM0, 2, value_pwm(0));
}

uint16_t value_pwm(int duty) {
    uint32_t calculo = (uint32_t)TPM_MODULE_MOTORS * duty;
    return (uint16_t)(calculo / 100);
}

void go(void){
    printk("indo\n");
    pwm_tpm_CnV(TPM0, 1, value_pwm(50));
    pwm_tpm_CnV(TPM0, 2, value_pwm(50));

    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 1);
    gpio_pin_set(gpio_c, 4, 0);
}

void back(void){
    printk("voltando\n");
    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 1);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);

    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));
}

void left(void){
    printk("direita\n");
    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);

    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));
}

void right(void){
    printk("direita\n");
    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);

    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));
}

void stop(void){
    printk("parando\n");
    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 0);

    pwm_tpm_CnV(TPM0, 1, value_pwm(0));
    pwm_tpm_CnV(TPM0, 2, value_pwm(0));
}