/*
 * ultrassonic.h
 *
 * Created on: 08/06/2026
 * Author: Thoshio Onuki Neto
 */
#ifndef ULTRASSONIC_H_
#define ULTRASSONIC_H_
 
//#include "externs.h"
#include <pwm_z402.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>


// Define o valor do registrador MOD do TPM para configurar o período do PWM
#define TPM_MODULE 22500             // Define a frequência do PWM fpwm = (TPM_CLK / (TPM_MODULE * PS)) para um periodo de 60ms (evitar sobreposicao de echos)
#define DUTY_CYCLE TPM_MODULE/4500   // Define duty_cycle em 0,022%  ->  pulsos duram 13,2us 

#define TPM_IRQ_LINE TPM1_IRQn  // relaciona a interrupção ao timer TPM1
#define TPM_IRQ_PRIORITY 1      // define a prioridade da interrupção

#define VELOCIDADE_SOM 34300      // cm/s



void trigger_init(void); //trigger TPM2 Ch:0 PTE22

void tpm1_isr(void *arg); //interrupt routine

void interrupt_init(void);

void tpm1_init(void); //echo TPM1 Ch:1 PTE21

void ultrassonic_init(void);

void print_distance(void);

int distance_cm();

#endif /* ULTRASSONIC_H_ */
