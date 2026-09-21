/*
 * encoders.h
 *
 * Created on: 15/09/2026
 * Author: Thoshio Onuki Neto
 */
#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// Ajuste conforme a estrutura física do seu robô
#define WHEEL_DIAMETER_CM  7.0f   // Diâmetro da roda em cm
#define ENCODER_SLOTS      5.0f  // Quantidade de furos do disco
#define WHEEL_BASE_CM      13.5f  // Distância entre rodas em cm

// Constantes calculadas
#define CM_PER_TICK        (6.5)
#define TICKS_FOR_90_DEG   (uint32_t)(9)

void encoders_init(void);
void encoders_reset(void);
uint32_t encoders_get_left_ticks(void);
uint32_t encoders_get_right_ticks(void);
float encoders_get_distance_cm(void);

#endif /* ENCODERS_H_ */
