/*
 * nrf24.c
 * Driver nRF24L01+ para FRDM-KL25Z.
 */

#include "nrf24.h"
#include "spi.h"
#include "MKL25Z4.h"
#include <zephyr/kernel.h>

/* --- Comandos SPI --- */
#define CMD_R_REGISTER     0x00
#define CMD_W_REGISTER     0x20
#define CMD_R_RX_PAYLOAD   0x61
#define CMD_W_TX_PAYLOAD   0xA0
#define CMD_FLUSH_TX       0xE1
#define CMD_FLUSH_RX       0xE2
#define CMD_NOP            0xFF

/* --- Registradores --- */
#define REG_CONFIG         0x00
#define REG_EN_AA          0x01
#define REG_EN_RXADDR      0x02
#define REG_SETUP_AW       0x03
#define REG_SETUP_RETR     0x04
#define REG_RF_CH          0x05
#define REG_RF_SETUP       0x06
#define REG_STATUS         0x07
#define REG_RX_ADDR_P0     0x0A
#define REG_TX_ADDR        0x10
#define REG_RX_PW_P0       0x11
#define REG_FIFO_STATUS    0x17
#define REG_DYNPD          0x1C
#define REG_FEATURE        0x1D

/* --- Bits do STATUS --- */
#define STATUS_RX_DR       (1 << 6)
#define STATUS_TX_DS       (1 << 5)
#define STATUS_MAX_RT      (1 << 4)

/* --- Bits do CONFIG --- */
#define CONFIG_EN_CRC      (1 << 3)
#define CONFIG_CRCO        (1 << 2)   /* CRC de 16 bits */
#define CONFIG_PWR_UP      (1 << 1)
#define CONFIG_PRIM_RX     (1 << 0)

/* --- Bits do FIFO_STATUS --- */
#define FIFO_RX_EMPTY      (1 << 0)

#define NRF_SPI            SPI_1

/* CSN = PTE4, CE = PTE5 */
#define PIN_CSN            4
#define PIN_CE             5

/* CONFIG base: CRC de 16 bits, radio ligado. */
#define CONFIG_BASE        (CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP)

static uint8_t      g_payload_len = 32;
static nrf24_modo_t g_modo        = NRF24_MODO_TX;

static inline void csn_low(void)  { GPIOE->PCOR = (1u << PIN_CSN); }
static inline void csn_high(void) { GPIOE->PSOR = (1u << PIN_CSN); }
static inline void ce_low(void)   { GPIOE->PCOR = (1u << PIN_CE);  }
static inline void ce_high(void)  { GPIOE->PSOR = (1u << PIN_CE);  }

static void write_reg(uint8_t reg, uint8_t value)
{
	csn_low();
	spi_transfer(NRF_SPI, CMD_W_REGISTER | reg);
	spi_transfer(NRF_SPI, value);
	csn_high();
}

static uint8_t read_reg(uint8_t reg)
{
	uint8_t value;

	csn_low();
	spi_transfer(NRF_SPI, CMD_R_REGISTER | reg);
	value = spi_transfer(NRF_SPI, CMD_NOP);
	csn_high();

	return value;
}

static void write_reg_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
	uint8_t i;

	csn_low();
	spi_transfer(NRF_SPI, CMD_W_REGISTER | reg);
	for(i = 0; i < len; i++)
	{
		spi_transfer(NRF_SPI, buf[i]);
	}
	csn_high();
}

static void send_cmd(uint8_t cmd)
{
	csn_low();
	spi_transfer(NRF_SPI, cmd);
	csn_high();
}

static void pins_init(void)
{
	/* CSN e CE como saida. O clock do PORTE ja e ligado pelo spi_init(). */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[PIN_CSN] = PORT_PCR_MUX(1);
	PORTE->PCR[PIN_CE]  = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1u << PIN_CSN) | (1u << PIN_CE);

	csn_high();
	ce_low();
}

uint8_t nrf24_status(void)
{
	uint8_t status;

	csn_low();
	status = spi_transfer(NRF_SPI, CMD_NOP);
	csn_high();

	return status;
}

void nrf24_set_modo(nrf24_modo_t modo)
{
	if(modo == NRF24_MODO_RX)
	{
		write_reg(REG_CONFIG, CONFIG_BASE | CONFIG_PRIM_RX);
		write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
		send_cmd(CMD_FLUSH_RX);
		/* Em RX o CE fica alto o tempo todo: e ele que mantem o radio ouvindo. */
		ce_high();
	}
	else
	{
		/* Em TX o CE so sobe no pulso que dispara cada pacote. */
		ce_low();
		write_reg(REG_CONFIG, CONFIG_BASE);
	}

	/* Datasheet: 130us para o radio assentar apos trocar de modo. */
	k_busy_wait(150);

	g_modo = modo;
}

void nrf24_diag(void)
{
	uint8_t lido_03, lido_02;

	pins_init();

	/* Datasheet: 100ms de power-on reset antes de falar com o radio. */
	k_msleep(100);

	printk("\n--- diagnostico nRF24 (SCK=PTE2 MOSI=PTE1 MISO=PTE3 CSN=PTE4 CE=PTE5) ---\n");
	printk("STATUS bruto = 0x%02X\n", nrf24_status());

	/*
	 * Escreve dois valores diferentes em SETUP_AW e le de volta. Se o MISO
	 * estiver bom, a leitura acompanha o que foi escrito. Um valor fixo nas
	 * duas leituras significa que nada esta chegando pelo MISO.
	 */
	write_reg(REG_SETUP_AW, 0x03);
	lido_03 = read_reg(REG_SETUP_AW);
	write_reg(REG_SETUP_AW, 0x02);
	lido_02 = read_reg(REG_SETUP_AW);
	write_reg(REG_SETUP_AW, 0x03);   /* volta para 5 bytes de endereco */

	printk("SETUP_AW: escrevi 0x03 li 0x%02X | escrevi 0x02 li 0x%02X\n",
	       lido_03, lido_02);

	if(lido_03 == 0x03 && lido_02 == 0x02)
	{
		printk("=> SPI OK, radio respondendo.\n");
	}
	else if(lido_03 == 0xFF && lido_02 == 0xFF)
	{
		printk("=> MISO preso em 1. PTE3 solto, ou modulo sem 3.3V.\n");
	}
	else if(lido_03 == 0x00 && lido_02 == 0x00)
	{
		printk("=> MISO preso em 0. PTE3 no GND, CSN nao chega ao modulo,\n");
		printk("   ou o modulo nao esta alimentado.\n");
	}
	else
	{
		printk("=> Resposta inconsistente. Fio ruim, mau contato ou ruido.\n");
	}
	printk("------------------------------------------------------------\n");
}

bool nrf24_init(const uint8_t *address, uint8_t channel,
                uint8_t payload_len, nrf24_modo_t modo)
{
	g_payload_len = payload_len;

	pins_init();

	/* Datasheet: 100ms de power-on reset antes de falar com o radio. */
	k_msleep(100);

	/*
	 * Sanidade do SPI: SETUP_AW vale 0x03 apos reset. Se a leitura nao bater
	 * com a escrita, o MISO nao esta chegando e nao adianta seguir.
	 */
	write_reg(REG_SETUP_AW, 0x03);
	if(read_reg(REG_SETUP_AW) != 0x03)
	{
		return false;
	}

	/* Retransmissao: ARD = 1500us, ARC = 15 tentativas. */
	write_reg(REG_SETUP_RETR, (5 << 4) | 15);

	/* PA_LOW + 1 Mbps. A taxa precisa ser igual nas duas placas. */
	write_reg(REG_RF_SETUP, 0x03);

	/* Payload estatico. */
	write_reg(REG_FEATURE, 0x00);
	write_reg(REG_DYNPD, 0x00);

	write_reg(REG_RF_CH, channel);

	/*
	 * Um unico pipe (0) para tudo. Em RX ele recebe; em TX o auto-ack volta
	 * pelo pipe 0, que por isso precisa ter o mesmo endereco do TX_ADDR.
	 * Usar so o pipe 0 deixa a troca de papel sem nenhuma mudanca de endereco.
	 */
	write_reg_buf(REG_TX_ADDR, address, NRF24_ADDR_WIDTH);
	write_reg_buf(REG_RX_ADDR_P0, address, NRF24_ADDR_WIDTH);
	write_reg(REG_RX_PW_P0, payload_len);
	write_reg(REG_EN_AA, 0x01);      /* auto-ack no pipe 0 */
	write_reg(REG_EN_RXADDR, 0x01);  /* pipe 0 habilitado */

	send_cmd(CMD_FLUSH_TX);
	send_cmd(CMD_FLUSH_RX);
	write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

	nrf24_set_modo(modo);

	/* Datasheet: 1.5ms de standby apos PWR_UP. */
	k_msleep(2);

	return true;
}

bool nrf24_send(const void *data, uint8_t len)
{
	const uint8_t *bytes = (const uint8_t *)data;
	nrf24_modo_t modo_anterior = g_modo;
	uint8_t status = 0;
	uint8_t i;
	uint32_t timeout;

	if(g_modo != NRF24_MODO_TX)
	{
		nrf24_set_modo(NRF24_MODO_TX);
	}

	write_reg(REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);

	csn_low();
	spi_transfer(NRF_SPI, CMD_W_TX_PAYLOAD);
	for(i = 0; i < g_payload_len; i++)
	{
		/* Payload estatico: completa com zero se o dado for menor. */
		spi_transfer(NRF_SPI, (i < len) ? bytes[i] : 0x00);
	}
	csn_high();

	/* Pulso em CE dispara a transmissao. Minimo 10us. */
	ce_high();
	k_busy_wait(15);
	ce_low();

	/*
	 * Pior caso: 15 retransmissoes x 1500us = 22.5ms. 50ms da folga.
	 * Polling no STATUS em vez de usar o pino IRQ.
	 */
	for(timeout = 0; timeout < 500; timeout++)
	{
		status = nrf24_status();
		if(status & (STATUS_TX_DS | STATUS_MAX_RT))
		{
			break;
		}
		k_busy_wait(100);
	}

	/*
	 * Qualquer desfecho que nao seja TX_DS deixa o payload preso na FIFO de
	 * transmissao - tanto MAX_RT quanto o estouro do timeout acima. Sem o
	 * flush, tres falhas enchem a FIFO (ela guarda 3 pacotes) e o radio para
	 * de transmitir de vez. E o classico "funciona uma vez e depois nada".
	 */
	if((status & STATUS_TX_DS) == 0)
	{
		send_cmd(CMD_FLUSH_TX);
	}

	write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

	if(modo_anterior != NRF24_MODO_TX)
	{
		nrf24_set_modo(modo_anterior);
	}

	return (status & STATUS_TX_DS) != 0;
}

bool nrf24_available(void)
{
	return (read_reg(REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0;
}

void nrf24_read(void *buf, uint8_t len)
{
	uint8_t *bytes = (uint8_t *)buf;
	uint8_t i;

	csn_low();
	spi_transfer(NRF_SPI, CMD_R_RX_PAYLOAD);
	for(i = 0; i < g_payload_len; i++)
	{
		uint8_t b = spi_transfer(NRF_SPI, CMD_NOP);
		if(i < len)
		{
			bytes[i] = b;
		}
	}
	csn_high();

	write_reg(REG_STATUS, STATUS_RX_DR);
}
