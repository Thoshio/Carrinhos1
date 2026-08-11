/*
 * motors.h
 *
 * Created on: 11/08/2026
 * Author: Thoshio Onuki Neto
 */
#ifndef MOTORS_H_
#define MOTORS_H_
 
//#include "externs.h"
#include <pwm_z402.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>


// Define o valor do registrador MOD do TPM para configurar o período do PWM
#define TPM_MODULE_MOTORS 10000



void motors_init(void);

void gpio_init(void);

void pwm_init(void);

uint16_t value_pwm(int duty);

void go(void);

void back(void);

void left(void);

void right(void);

void stop(void);

#endif /* MOTORS_H_ */
