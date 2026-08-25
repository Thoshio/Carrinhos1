#include "ultrassonic.h"

volatile uint16_t captured = 0;
volatile uint16_t captured_subida = 0; 
volatile bool esperando_descida = false;   // Controla se estamos medindo subida ou descida
volatile bool nova_leitura_pronta = false; // Avisa o main que há um dado novo
volatile uint16_t delta = 0;
int ultima_distancia;

void trigger_init(void) {
    // trigger TPM2 Ch:0 PTB2
    pwm_tpm_Init(TPM2, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM2, 0, TPM_PWM_H, GPIOB, 2);
    pwm_tpm_CnV(TPM2, 0, DUTY_CYCLE);
}

void tpm1_isr(void *arg) {
    TPM1->STATUS |= TPM_STATUS_CH1F_MASK; // Zera a flag que gerou a interrupção

    captured = TPM1->CONTROLS[1].CnV; // Coloca o valor atual do timer na variável "captured"

   // Se o bit 21 for 1, a energia está ALTA (borda de subida). Se for 0, está BAIXA.
    bool pino_alto = (GPIOB->PDIR & (1 << 1)) != 0;

    if (pino_alto) {
        // É GARANTIDO que o pulso começou
        captured_subida = captured;
        esperando_descida = true;
    } 
    else {
        // É GARANTIDO que o pulso terminou
        if (esperando_descida) { // Só calcula se realmente viu o início antes
            delta = captured - captured_subida;
            nova_leitura_pronta = true;
            esperando_descida = false;
        }
    }
}

void interrupt_init(void) {
    // Conecta a interrupção via Zephyr
    IRQ_CONNECT(TPM_IRQ_LINE, TPM_IRQ_PRIORITY, tpm1_isr, NULL, 0);
    irq_enable(TPM_IRQ_LINE);
 
}

void tpm1_init(void) {
    // echo TPM1 Ch:1 PTB1
    // Inicializa TPM1 com módulo e prescaler desejado
    pwm_tpm_Init(TPM1, TPM_PLLFLL, 65535, TPM_CLK, PS_128, EDGE_PWM);
    // Configura TPM1_CH1 como input capture na borda de subida e descida em PTB1
    pwm_tpm_Ch_Init(TPM1, 1, TPM_INPUT_CAPTURE_BOTH|TPM_CHANNEL_INTERRUPT, GPIOB, 1);
}

void ultrassonic_init(void) {
    trigger_init();
    interrupt_init();
    tpm1_init();

    printk("Ultrassônico configurado!\n");
}

void print_distance(void) {
    // Se houver delta valido, calcula tempo e, em seguida, a distância
    if (nova_leitura_pronta) {
            nova_leitura_pronta = false;
            float tempo_segundos = (float)delta * 128.0f / 48000000;
            float dist = tempo_segundos * VELOCIDADE_SOM / 2;

            // Mostra a distância que está enxergando
            printk("Objeto a: %f cm\n", (double)dist);
        }
        k_msleep(100);
}

int distance_cm() {
    if (nova_leitura_pronta) {
        // Se houver delta valido, calcula tempo e, em seguida, a distância
        nova_leitura_pronta = false;
        float tempo_segundos = (float)delta * 128.0f / 48000000;
        float dist = tempo_segundos * VELOCIDADE_SOM / 2;
        
        // Retorna a distância calculada ou a última lida no caso da nova leitura ainda estar sendo gerada
        ultima_distancia = (int)dist;
    }
    return ultima_distancia;
}