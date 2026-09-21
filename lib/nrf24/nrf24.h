/*
 * nrf24.h
 * Driver nRF24L01 (versão Original) para FRDM-KL25Z.
 * Atualizado para utilizar Interrupção (IRQ) e Semáforos do Zephyr OS.
 */

#ifndef LIB_NRF24_NRF24_H_
#define LIB_NRF24_NRF24_H_

#include <stdbool.h>
#include <stdint.h>

/* --- BITS DE STATUS (Mova para cá para o main.c poder enxergar) --- */
#define STATUS_RX_DR       (1 << 6)
#define STATUS_TX_DS       (1 << 5)
#define STATUS_MAX_RT      (1 << 4)

/* Largura do endereço em bytes */
#define NRF24_ADDR_WIDTH   5

typedef enum {
	NRF24_MODO_TX = 0,   /* Transmissor (PTX) */
	NRF24_MODO_RX = 1    /* Receptor (PRX) - modo de escuta */
} nrf24_modo_t;

bool nrf24_init(const uint8_t *address, uint8_t channel, uint8_t payload_len, nrf24_modo_t modo);
void nrf24_set_modo(nrf24_modo_t modo);
bool nrf24_send(const void *data, uint8_t len);
bool nrf24_irq_occurred(void);
bool nrf24_available(void);
void nrf24_read(void *buf, uint8_t len);
uint8_t nrf24_status(void);
void nrf24_diag(void);

#endif /* LIB_NRF24_NRF24_H_ */