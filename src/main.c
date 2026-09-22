#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "MKL25Z4.h"
#include "spi.h"
#include "nrf24.h"

#define RF_CANAL   76
static const uint8_t endereco[NRF24_ADDR_WIDTH] = { 'C', 'A', 'R', 'R', '1' };

/* Pacote de dados para comunicação rádio idêntico ao código do carrinho */
typedef struct __attribute__((packed)) {
    char comando;
    float distancia;
} radio_packet_t;

/* Dispositivo UART para o console do computador */
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static void mostra_ajuda(void) {
    printk("\n======================================\n");
    printk("       CONTROLE DO CARRINHO           \n");
    printk("======================================\n");
    printk(" R - Iniciar o Carrinho (RUN)\n");
    printk(" S - Parar o Carrinho (STOP)\n");
    printk(" E - Apagar a Distancia\n");
    printk(" D - Requisitar a Distancia\n");
    printk(" H - Mostrar este menu\n");
    printk("======================================\n\n");
}

static void processa_teclado(uint8_t tecla) {
    // Ignora espaços e line breaks 
    if(tecla == '\r' || tecla == '\n' || tecla == ' ') return;
    
    // Converte caracteres minúsculos para maiúsculos
    if(tecla >= 'a' && tecla <= 'z') tecla -= 32;

    if (tecla == 'H') {
        mostra_ajuda();
        return;
    }

    radio_packet_t tx;
    tx.distancia = 0.0f; // Campo ignorado nos comandos básicos

    if(tecla == 'R' || tecla == 'S' || tecla == 'E' || tecla == 'D') {
        tx.comando = (char)tecla;
        
        // A função nrf24_send troca para TX, envia e volta automaticamente para RX
        if(nrf24_send(&tx, sizeof(tx))) {
            printk("-> Comando '%c' enviado com sucesso (ACK).\n", tx.comando);
            
            if(tecla == 'D') {
                printk("-> Modo de espera: Aguardando interrupcao com a distancia do carrinho...\n");
            }
        } else {
            printk("-> Falha: O carrinho não respondeu ao comando '%c' (Sem ACK).\n", tx.comando);
        }
    } else {
        printk("-> Comando invalido: %c\n", tecla);
    }
}

int main(void) {
    uint8_t tecla;

    k_msleep(500);
    printk("\nInicializando Placa Mestre no PC...\n");

    if(!device_is_ready(uart_dev)) {
        printk("Erro: Controlador UART ausente!\n");
        return -1;
    }

    // Inicializa a comunicação SPI com a mesma configuração do carrinho
    spi_init(SPI_1, ALT_0, PRESCALE_2, DIVISOR_1, CS_MAN);

    // O módulo de rádio é inicializado no modo RX para ficar de escuta continuamente
    if(!nrf24_init(endereco, RF_CANAL, sizeof(radio_packet_t), NRF24_MODO_RX)) {
        printk("Falha critica na inicializacao do NRF24 (nrf24_init).\n");
        for(;;) k_msleep(100);
    }

    printk("Radio da Placa Mestre configurado no Canal %d.\n", RF_CANAL);
    mostra_ajuda();

    while (1) {
        // ==========================================
        // 1. PROCESSA DIGITAÇÃO NO COMPUTADOR
        // ==========================================
        (void)uart_err_check(uart_dev);
        while(uart_poll_in(uart_dev, &tecla) == 0) {
            processa_teclado(tecla);
            (void)uart_err_check(uart_dev);
        }

        // ==========================================
        // 2. RECEBE A DISTÂNCIA DO CARRINHO (VIA IRQ)
        // ==========================================
        // Captura a resposta inteligente assim que o pino de IRQ puxar o semáforo
        if (nrf24_irq_occurred()) {
            uint8_t r_status = nrf24_status();
            
            // Verifica o bit de STATUS_RX_DR (bit 6 indicando chegada de payload)
            if (r_status & (1 << 6)) { 
                while (nrf24_available()) {
                    radio_packet_t rx;
                    nrf24_read(&rx, sizeof(rx));

                    // Confere se o carrinho respondeu à requisição de distância
                    if (rx.comando == 'A') {
                        printk("\n--------------------------------------\n");
                        printk(" DISTANCIA PERCORRIDA: %.2f cm\n", (double)rx.distancia);
                        printk("--------------------------------------\n\n");
                    }
                }
            }
        }
        
        // Reduz a sobrecarga de CPU cravando um pequeno delay
        k_msleep(5);
    }
    return 0;
}