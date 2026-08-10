#include "ultrassonic.h"

volatile uint16_t captured = 0;
volatile uint16_t captured_subida = 0; 
volatile bool esperando_descida = false; // Controla se estamos medindo subida ou descida
volatile bool nova_leitura_pronta = false; // Avisa o main que há um dado novo
volatile uint16_t delta = 0;

void trigger_init(void) {
    //trigger TPM2 Ch:0 PTBE22
    pwm_tpm_Init(TPM2, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM2, 0, TPM_PWM_H, GPIOE, 22);
    pwm_tpm_CnV(TPM2, 0, DUTY_CYCLE);
}

void tpm1_isr(void *arg) {
    TPM1->STATUS |= TPM_STATUS_CH0F_MASK; // zerra a flag que gerou a interrupção

    captured = TPM1->CONTROLS[0].CnV; // coloca o valor atual do timer na variável "captured"

    if (!esperando_descida) {
        captured_subida = captured;
        esperando_descida = true;
    } else {
        delta = captured-captured_subida;
        nova_leitura_pronta = true;
        esperando_descida = false;
    }
}

void interrupt_init(void) {
    // Conecta a interrupção via Zephyr
    IRQ_CONNECT(TPM_IRQ_LINE, TPM_IRQ_PRIORITY, tpm1_isr, NULL, 0);
    irq_enable(TPM_IRQ_LINE);
 
}

void tpm1_init(void) {
    //echo TPM1 Ch:1 PTE21
    // Inicializa TPM1 com módulo e prescaler desejado
    pwm_tpm_Init(TPM1, TPM_PLLFLL, 65535, TPM_CLK, PS_128, EDGE_PWM);
    // Configura TPM1_CH0 como input capture na borda de subida e descida em PTB0
    pwm_tpm_Ch_Init(TPM1, 1, TPM_INPUT_CAPTURE_BOTH|TPM_CHANNEL_INTERRUPT, GPIOE, 21);
}

void ultrassonic_init(void) {
    trigger_init();
    interrupt_init();
    tpm1_init();
}

void print_distance(void) {
    if (nova_leitura_pronta) {
            nova_leitura_pronta = false;
            float tempo_segundos = (float)delta * 128.0f / 48000000;
            float dist = tempo_segundos * VELOCIDADE_SOM / 2;
            printk("Objeto a: %f cm\n", (double)dist);
        }
        k_msleep(100);
}