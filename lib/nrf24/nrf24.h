/*
 * nrf24.h
 * Driver nRF24L01+ para FRDM-KL25Z (bare-metal sobre a lib spi.c).
 *
 * Suporta os dois papeis: transmissor (PTX) e receptor (PRX), com troca em
 * tempo de execucao. Duas placas com este mesmo firmware conversam entre si.
 *
 * Ligacao (SPI1 alternativa 0) - identica nas duas placas:
 *   nRF24L01     FRDM-KL25Z
 *   VCC     ->   3.3V   (NUNCA 5V - maximo do modulo e 3.6V)
 *   GND     ->   GND
 *   SCK     ->   PTE2   (SPI1_SCK)
 *   MOSI    ->   PTE1   (SPI1_MOSI)
 *   MISO    ->   PTE3   (SPI1_MISO)
 *   CSN     ->   PTE4   (GPIO)
 *   CE      ->   PTE5   (GPIO)
 *   IRQ     ->   (nao usado - o driver faz polling)
 *
 * O KL25Z ja opera em 3.3V, entao nao precisa de level shifter.
 * Capacitor de 10-100uF entre VCC e GND junto ao modulo: o pico de corrente
 * na transmissao causa brownout e e a causa mais comum de link instavel.
 *
 * OBS: a pinagem do modulo muda entre versoes. Confira o seu antes de ligar.
 */

#ifndef LIB_NRF24_NRF24_H_
#define LIB_NRF24_NRF24_H_

#include <stdbool.h>
#include <stdint.h>

/* Largura do endereco em bytes. */
#define NRF24_ADDR_WIDTH   5

typedef enum
{
	NRF24_MODO_TX = 0,   /* PTX - transmissor */
	NRF24_MODO_RX = 1    /* PRX - receptor, escutando */
} nrf24_modo_t;

/*
 * Inicializa o radio.
 *
 * address     - NRF24_ADDR_WIDTH bytes. As duas placas usam o MESMO endereco.
 * channel     - canal RF (0-125). As duas placas usam o MESMO canal.
 * payload_len - tamanho fixo do payload (1-32), igual nas duas placas.
 * modo        - papel inicial.
 *
 * Retorna false se o radio nao responder no SPI (fiacao ou alimentacao).
 */
bool nrf24_init(const uint8_t *address, uint8_t channel,
                uint8_t payload_len, nrf24_modo_t modo);

/* Troca o papel do radio em tempo de execucao. */
void nrf24_set_modo(nrf24_modo_t modo);

/*
 * Envia um payload e aguarda o auto-ack. Coloca o radio em TX se preciso e
 * devolve ao modo anterior no final.
 * Retorna true se o outro lado confirmou, false em MAX_RT (sem resposta).
 */
bool nrf24_send(const void *data, uint8_t len);

/* true se ha pacote na FIFO de recepcao. So faz sentido em NRF24_MODO_RX. */
bool nrf24_available(void);

/* Copia um pacote recebido para buf. Chamar apos nrf24_available(). */
void nrf24_read(void *buf, uint8_t len);

/* Le o registrador STATUS. Util para debug. */
uint8_t nrf24_status(void);

/*
 * Exercita o barramento SPI e imprime o resultado no console (OpenSDA).
 * Escreve dois valores distintos num registrador e le de volta, o que
 * separa "MISO morto" de "radio respondendo". Chamar antes de nrf24_init().
 */
void nrf24_diag(void);

#endif /* LIB_NRF24_NRF24_H_ */
