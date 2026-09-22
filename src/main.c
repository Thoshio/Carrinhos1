#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "encoders.h"
#include "motors.h"
#include "ultrassonic.h"
#include "nrf24.h"
#include "spi.h"

#define RF_CANAL   76
static const uint8_t endereco[NRF24_ADDR_WIDTH] = { 'C', 'A', 'R', 'R', '1' };

/* Pacote de dados para comunicação rádio */
typedef struct __attribute__((packed)) {
    char comando;
    float distancia;
} radio_packet_t;

/* Máquina de Estados */
typedef enum { STOP, RUN } estado_carro_t;

int main(void) {
    printk("Iniciando Robô Labirinto...\n");

    // Inicialização do Hardware
    motors_init();
    encoders_init();
    ultrassonic_init();
    spi_init(SPI_1, ALT_0, PRESCALE_2, DIVISOR_1, CS_MAN);

    if (!nrf24_init(endereco, RF_CANAL, sizeof(radio_packet_t), NRF24_MODO_RX)) {
        printk("Falha critica no nrf24_init()\n");
        return -1;
    }

    estado_carro_t estado = STOP;
    float distancia_acumulada = 0.0f;

    while (1) {
        // ==========================================
        // 1. PROCESSAMENTO DE COMANDOS WIRELESS
        // ==========================================
        if (nrf24_irq_occurred()) {
            uint8_t r_status = nrf24_status();
            
            if (r_status & (1 << 6)) { // STATUS_RX_DR indicando chegada de dados
                while (nrf24_available()) {
                    radio_packet_t rx;
                    nrf24_read(&rx, sizeof(rx));

                    switch (rx.comando) {
                        case 'R':
                            estado = RUN;
                            encoders_reset();
                            printk("Comando RX: Iniciando Labirinto (RUN)\n");
                            break;
                            
                        case 'S':
                            estado = STOP;
                            stop();
                            printk("Comando RX: Parando (STOP)\n");
                            break;
                            
                        case 'D': {
                            // Calcula distância em tempo real
                            float dist_total = distancia_acumulada;
                            if (estado == RUN) {
                                dist_total += encoders_get_distance_cm();
                            }
                            
                            // A função nrf24_send troca temporariamente para TX e devolve para RX
                            radio_packet_t tx = {'A', dist_total}; // 'A' de Answer
                            nrf24_send(&tx, sizeof(tx));
                            printk("Distancia transmitida ao PC: %.2f cm\n", (double)dist_total);
                            break;
                        }
                        
                        case 'E':
                            distancia_acumulada = 0.0f;
                            encoders_reset();
                            printk("Comando RX: Memoria de distancia zerada!\n");
                            break;
                    }
                }
            }
        }

        // ==========================================
        // 2. NAVEGAÇÃO E DESVIO DE OBSTÁCULOS
        // ==========================================
        if (estado == RUN) {
            int dist_frente = distance_cm();

            // Caminho livre
            if (dist_frente > 15) {
                go();
                k_msleep(50); // Move um pouco e refaz a leitura
            } 
            // Obstáculo detectado
            else {
                stop();
                
                // Salva a distância percorrida na reta antes da curva
                distancia_acumulada += encoders_get_distance_cm();
                printk("Obstaculo a %d cm. Analisando rotas...\n", dist_frente);

                // Analisa a Esquerda
                turn_left_deg90();
                k_msleep(200); // Aguarda estabilização do ultrassônico
                int dist_esq = distance_cm();

                // Analisa a Direita (Gira 180 graus a partir da visão esquerda)
                turn_right_deg90();
                turn_right_deg90();
                k_msleep(200);
                int dist_dir = distance_cm();

                // Tomada de Decisão Lógica
                if (dist_esq > dist_dir && dist_esq > 15) {
                    printk("Direcao escolhida: Esquerda.\n");
                    // O carro está olhando para a direita. Precisa girar 180 para ir à esquerda.
                    turn_left_deg90();
                    turn_left_deg90();
                } 
                else if (dist_dir >= dist_esq && dist_dir > 15) {
                    printk("Direcao escolhida: Direita.\n");
                    // O carro já está apontado para a direita. Apenas mantém a direção.
                } 
                else {
                    printk("Sem saida. Efetuando meia volta (180 graus).\n");
                    // Está apontado para a direita. Gira mais 90 direita para voltar por onde veio.
                    turn_right_deg90();
                }

                // Fim da manobra: Zera os encoders. Os ticks gastos nas curvas são descartados.
                encoders_reset();
            }
        } else {
            // No estado STOP, cede a vez para reduzir o uso da CPU
            k_msleep(50);
        }
    }
    return 0;
}