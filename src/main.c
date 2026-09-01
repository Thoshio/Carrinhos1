/*
 * main.c
 * Comunicação wireless entre placas FRDM-KL25Z com nRF24L01 operando via interrupção.
 */

#include "MKL25Z4.h"
#include "spi.h"
#include "nrf24.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define PAPEL_TX             0
#define PAPEL_RX             1
#define PAPEL_BIDIRECIONAL   2

#define PAPEL   PAPEL_BIDIRECIONAL

#define RF_CANAL   76
static const uint8_t endereco[NRF24_ADDR_WIDTH] = { 'C', 'A', 'R', 'R', '1' };

/* Pinos do LED RGB da placa KL25Z */
#define LED_R_PIN   18
#define LED_G_PIN   19
#define LED_B_PIN   1

#define ACAO_APAGA    0
#define ACAO_ACENDE   1
#define ACAO_INVERTE  2

typedef struct __attribute__((packed)) {
    uint8_t  comando;
    uint8_t  estado;
    uint16_t seq;
} comando_t;

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static nrf24_modo_t g_modo_local;
static uint16_t     g_seq;

/* Lógica e hardware dos LEDs originais mantidos */
static void led_init(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	PORTB->PCR[LED_R_PIN] = PORT_PCR_MUX(1);
	PORTB->PCR[LED_G_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[LED_B_PIN] = PORT_PCR_MUX(1);

	GPIOB->PDDR |= (1u << LED_R_PIN) | (1u << LED_G_PIN);
	GPIOD->PDDR |= (1u << LED_B_PIN);

	GPIOB->PSOR = (1u << LED_R_PIN) | (1u << LED_G_PIN);
	GPIOD->PSOR = (1u << LED_B_PIN);
}

static void led_aplica(uint8_t cor, uint8_t acao) {
	uint32_t mascara_b = 0;
	uint32_t mascara_d = 0;

	switch(cor) {
		case 'r': mascara_b = (1u << LED_R_PIN); break;
		case 'g': mascara_b = (1u << LED_G_PIN); break;
		case 'b': mascara_d = (1u << LED_B_PIN); break;
		case 'a':
			mascara_b = (1u << LED_R_PIN) | (1u << LED_G_PIN);
			mascara_d = (1u << LED_B_PIN);
			break;
		default: return;
	}

	switch(acao) {
		case ACAO_ACENDE:
			if(mascara_b) GPIOB->PCOR = mascara_b;
			if(mascara_d) GPIOD->PCOR = mascara_d;
			break;
		case ACAO_APAGA:
			if(mascara_b) GPIOB->PSOR = mascara_b;
			if(mascara_d) GPIOD->PSOR = mascara_d;
			break;
		case ACAO_INVERTE:
			if(mascara_b) GPIOB->PTOR = mascara_b;
			if(mascara_d) GPIOD->PTOR = mascara_d;
			break;
		default: break;
	}
}

static void mostra_ajuda(void) {
	printk("\nComandos (controlam o LED da OUTRA placa):\n");
	printk("  r / g / b  - inverte o LED vermelho / verde / azul\n");
	printk("  1          - acende todos\n");
	printk("  0          - apaga todos\n");
	printk("  t          - troca o papel desta placa (TX <-> RX)\n");
	printk("  h          - mostra esta ajuda\n\n");
}

static bool tecla_para_comando(uint8_t tecla, comando_t *cmd) {
	switch(tecla) {
		case 'r': case 'g': case 'b':
			cmd->comando = tecla;
			cmd->estado  = ACAO_INVERTE;
			return true;
		case '1':
			cmd->comando = 'a';
			cmd->estado  = ACAO_ACENDE;
			return true;
		case '0':
			cmd->comando = 'a';
			cmd->estado  = ACAO_APAGA;
			return true;
		default:
			return false;
	}
}

static void trata_tecla(uint8_t tecla) {
    comando_t cmd;

    if(tecla == '\r' || tecla == '\n') return;
    if(tecla == 'h') { mostra_ajuda(); return; }
    if(tecla == 't') {
        g_modo_local = (g_modo_local == NRF24_MODO_RX) ? NRF24_MODO_TX : NRF24_MODO_RX;
        nrf24_set_modo(g_modo_local);
        printk("Papel agora: %s\n", (g_modo_local == NRF24_MODO_RX) ? "RECEPTOR" : "TRANSMISSOR");
        return;
    }

    if(!tecla_para_comando(tecla, &cmd)) return;

#if PAPEL == PAPEL_RX
    printk("Placa em modo RECEPTOR - tecle 't' para transmitir\n");
#else
    cmd.seq = g_seq++;
    if(nrf24_send(&cmd, sizeof(cmd))) {
        printk("Enviado '%c' ação %u (seq %u) - ACK recebido!\n", cmd.comando, cmd.estado, cmd.seq);
    } else {
        printk("Enviado '%c' (seq %u) - SEM ACK: a outra placa não respondeu.\n", cmd.comando, cmd.seq);
    }
#endif
}

int main(void) {
    uint8_t tecla;
    nrf24_modo_t modo;

    k_msleep(500);
    printk("\n=== FRDM-KL25Z + nRF24L01 (Operação Via IRQ) ===\n");

    led_init();

    if(!device_is_ready(uart_dev)) {
        return -1;
    }

    spi_init(SPI_1, ALT_0, PRESCALE_2, DIVISOR_1, CS_MAN);

    nrf24_diag();

#if PAPEL == PAPEL_TX
    modo = NRF24_MODO_TX;
    printk("Papel Base: TRANSMISSOR\n");
#else
    modo = NRF24_MODO_RX;
    printk("Papel Base: %s\n", (PAPEL == PAPEL_RX) ? "RECEPTOR" : "BIDIRECIONAL");
#endif

    if(!nrf24_init(endereco, RF_CANAL, sizeof(comando_t), modo)) {
        printk("Falha crítica no nrf24_init().\n");
        for(;;) k_msleep(100);
    }

    printk("Rádio pronto! Canal %d.\n", RF_CANAL);
    mostra_ajuda();

    g_modo_local = modo;

    for(;;) {
        (void)uart_err_check(uart_dev);
        while(uart_poll_in(uart_dev, &tecla) == 0) {
            trata_tecla(tecla);
            (void)uart_err_check(uart_dev);
        }

        /* Lógica do Receptor operando por IRQ (Semáforo do Zephyr) */
        if(g_modo_local == NRF24_MODO_RX) {
            if(nrf24_irq_occurred()) {
                uint8_t r_status = nrf24_status();
                if (r_status & STATUS_RX_DR) {
                    while (nrf24_available()) {
                        comando_t rx;
                        nrf24_read(&rx, sizeof(rx));
                        led_aplica(rx.comando, rx.estado);
                        printk("Recebido comando '%c' ação %u (seq %u)\n", rx.comando, rx.estado, rx.seq);
                    }
                }
            }
        }
        
        k_msleep(5);
    }
    return 0;
}