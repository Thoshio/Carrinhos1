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
    //Iniciando TPM0 para a PWM dos motores 
    pwm_tpm_Init(TPM0, TPM_PLLFLL, TPM_MODULE_MOTORS, TPM_CLK, PS_4, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM0, 1, TPM_PWM_H, GPIOA, 4);
    pwm_tpm_Ch_Init(TPM0, 2, TPM_PWM_H, GPIOA, 5);

    pwm_tpm_CnV(TPM0, 1, value_pwm(0));
    pwm_tpm_CnV(TPM0, 2, value_pwm(0));
}


uint16_t value_pwm(int duty) {
    //função que define o valor em função do Duty Cycle que será passado para pwm_tpm_CnV 
    uint32_t calculo = (uint32_t)TPM_MODULE_MOTORS * duty;
    return (uint16_t)(calculo / 100);
}

void go(void){
    // ir para frente com 100% de força dos motores
    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));

    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 1);
    gpio_pin_set(gpio_c, 4, 0);
}

void back(void){
    // ir para trás com 100% de força dos motores
    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));

    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 1);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);
}

void left(void){
    // ir para esquerda com 100% de força dos motores
    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));

    gpio_pin_set(gpio_c, 7, 0);
    gpio_pin_set(gpio_c, 0, 1);
    gpio_pin_set(gpio_c, 3, 1);
    gpio_pin_set(gpio_c, 4, 0);

    k_msleep(1200);
}

void right(void){
    // ir para direita com 100% de força dos motores
    pwm_tpm_CnV(TPM0, 1, value_pwm(100));
    pwm_tpm_CnV(TPM0, 2, value_pwm(100));

    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 0);
    gpio_pin_set(gpio_c, 3, 0);
    gpio_pin_set(gpio_c, 4, 1);

    k_msleep(1200);
}

void stop(void){
    // desligar motores
    pwm_tpm_CnV(TPM0, 1, value_pwm(0));
    pwm_tpm_CnV(TPM0, 2, value_pwm(0));

    gpio_pin_set(gpio_c, 7, 1);
    gpio_pin_set(gpio_c, 0, 1);
    gpio_pin_set(gpio_c, 3, 1);
    gpio_pin_set(gpio_c, 4, 1);
}