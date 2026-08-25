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
 *   CSN     ->   PTD5   (GPIO)
 *   CE      ->   PTA13  (GPIO)
 *   IRQ     ->   PTA16  (GPIO com interrupcao, ativo em BAIXO)
 *
 * O KL25Z ja opera em 3.3V, entao nao precisa de level shifter.
 * Capacitor de 10-100uF entre VCC e GND junto ao modulo: o pico de corrente
 * na transmissao causa brownout e e a causa mais comum de link instavel.
 *
 * OBS: a pinagem do modulo muda entre versoes. Confira o seu antes de ligar.
 *
 * ---------------------------------------------------------------------------
 * SOBRE O PINO IRQ
 * ---------------------------------------------------------------------------
 *
 * O IRQ e uma saida do radio, ativa em BAIXO, que avisa "aconteceu um evento"
 * sem que o KL25Z precise ficar consultando o STATUS pelo SPI.
 *
 * Aqui ele so e usado para RECEPCAO: nrf24_irq_init() mascara os eventos de
 * transmissao no proprio radio, entao a unica coisa capaz de baixar o pino e a
 * chegada de um pacote (RX_DR). Os desfechos de envio (TX_DS / MAX_RT)
 * continuam sendo lidos por polling dentro de nrf24_send(), que e bloqueante.
 *
 * PTA16 foi escolhido porque no KL25Z SO as portas A e D tem hardware de
 * interrupcao por pino - PORTB, PORTC e PORTE nao geram interrupcao nenhuma.
 * Se precisar mudar o pino, mexa apenas no bloco NRF_IRQ_* no topo do nrf24.c
 * e mantenha-se em PORTA ou PORTD, ajustando NRF_IRQ_LINE junto.
 *
 * O mesmo bloco de #define cobre CSN e CE (NRF_CSN_* e NRF_CE_*), que tambem
 * mudaram de porta: os tres sinais de controle ficam fora do PORTE, que agora
 * carrega apenas o barramento SPI.
 *
 * O IRQ e OPCIONAL: sem chamar nrf24_irq_init() o driver continua funcionando
 * por polling com nrf24_available(), exatamente como antes.
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

/*
 * Liga a interrupcao do pino IRQ. Chamar DEPOIS de nrf24_init().
 *
 * Configura PTA16 como entrada com pull-up e interrupcao na borda de DESCIDA,
 * e mascara TX_DS e MAX_RT no radio para que so a chegada de pacote (RX_DR)
 * baixe o pino.
 *
 * A rotina de interrupcao nao toca no SPI: ela apenas marca uma flag interna.
 * Quem le o pacote continua sendo o laco principal, via nrf24_irq_recebido()
 * seguido de nrf24_available() / nrf24_read().
 */
void nrf24_irq_init(void);

/*
 * Consome o aviso deixado pela interrupcao.
 *
 * Retorna true UMA vez por evento sinalizado e ja limpa a flag, entao pode ser
 * chamada direto no if do laco principal.
 *
 * IMPORTANTE: a interrupcao e por borda, e a FIFO do radio guarda ate 3
 * pacotes. Se dois chegarem coladinhos, ha uma unica borda de descida para os
 * dois. Por isso o tratamento tem que ESVAZIAR a FIFO, e nao ler so um pacote:
 *
 *   if(nrf24_irq_recebido())
 *   {
 *       while(nrf24_available()) { nrf24_read(&pkt, sizeof(pkt)); ... }
 *   }
 */
bool nrf24_irq_recebido(void);

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
