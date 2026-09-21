#include <zephyr/kernel.h>
#include "encoders.h"
#include "motors.h"

int main(void) {
    printk("Iniciando Robô com Leitura de Encoders...\n");

    motors_init();
    encoders_init();

    // Teste 1: Anda para frente e imprime distância percorrida
    encoders_reset();
    go();
    
    for (int i = 0; i < 21; i++) {
        k_msleep(200);
        printk("Distancia: %.2f cm\n",
               (double)encoders_get_distance_cm());
    }
    stop();
    k_msleep(1000);

    // Teste 2: Executa Giro 90 graus para Esquerda
    printk("Girando 90 graus para Esquerda...\n");
    //turn_left_deg90();
    left();
    k_msleep(2000);

    // Teste 3: Executa Giro 90 graus para Direita
    printk("Girando 90 graus para Direita...\n");
    //turn_right_deg90();
    right();
    k_msleep(1000);
    stop();

    while (1) {
        k_msleep(1000);
    }

    return 0;
}