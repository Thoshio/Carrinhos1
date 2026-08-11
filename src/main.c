/*
 * main.c
 * Comunicacao wireless entre duas placas FRDM-KL25Z com nRF24L01+.
 *
 * Pelo terminal do PC (porta COM virtual do OpenSDA) voce controla o LED RGB
 * da OUTRA placa. As duas placas rodam este mesmo firmware.
 *
 * A camada de radio esta em lib/nrf24. A camada SPI, em lib/spi.
 */

#include "MKL25Z4.h"
#include "spi.h"
#include "nrf24.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/* ------------------------------------------------------------------ */
/*  FLAG DE PAPEL DA PLACA                                            */
/*                                                                    */
/*  PAPEL_BIDIRECIONAL - fica escutando e envia quando voce digita.    */
/*                       As duas placas controlam o LED uma da outra.  */
/*                       Grave este mesmo binario nas duas.            */
/*  PAPEL_TX           - so transmite. Placa que manda os comandos.    */
/*  PAPEL_RX           - so recebe. Placa cujo LED sera controlado.    */
/*                                                                    */
/*  Tambem da para trocar em tempo de execucao: tecle 't' no terminal. */
/* ------------------------------------------------------------------ */
#define PAPEL_TX             0
#define PAPEL_RX             1
#define PAPEL_BIDIRECIONAL   2

#define PAPEL   PAPEL_BIDIRECIONAL

/* Canal e endereco precisam ser IGUAIS nas duas placas. */
#define RF_CANAL   76
static const uint8_t endereco[NRF24_ADDR_WIDTH] = { 'C', 'A', 'R', 'R', '1' };

/* LED RGB da FRDM-KL25Z. Todos ativos em nivel BAIXO. */
#define LED_R_PIN   18   /* PTB18 */
#define LED_G_PIN   19   /* PTB19 */
#define LED_B_PIN   1    /* PTD1  */

/* Acoes do campo 'estado'. */
#define ACAO_APAGA    0
#define ACAO_ACENDE   1
#define ACAO_INVERTE  2

/*
 * Payload trocado entre as placas. Tamanho fixo de 4 bytes, igual nos dois
 * lados. 'packed' para o layout nao depender do alinhamento do compilador.
 */
typedef struct __attribute__((packed))
{
	uint8_t  comando;   /* 'r', 'g', 'b' ou 'a' (todos) */
	uint8_t  estado;    /* ACAO_APAGA / ACAO_ACENDE / ACAO_INVERTE */
	uint16_t seq;       /* contador de sequencia, para enxergar perda */
} comando_t;

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static void led_init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	PORTB->PCR[LED_R_PIN] = PORT_PCR_MUX(1);
	PORTB->PCR[LED_G_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[LED_B_PIN] = PORT_PCR_MUX(1);

	GPIOB->PDDR |= (1u << LED_R_PIN) | (1u << LED_G_PIN);
	GPIOD->PDDR |= (1u << LED_B_PIN);

	/* Nivel alto = apagado. */
	GPIOB->PSOR = (1u << LED_R_PIN) | (1u << LED_G_PIN);
	GPIOD->PSOR = (1u << LED_B_PIN);
}

static void led_aplica(uint8_t cor, uint8_t acao)
{
	uint32_t mascara_b = 0;
	uint32_t mascara_d = 0;

	switch(cor)
	{
		case 'r': mascara_b = (1u << LED_R_PIN); break;
		case 'g': mascara_b = (1u << LED_G_PIN); break;
		case 'b': mascara_d = (1u << LED_B_PIN); break;
		case 'a':
			mascara_b = (1u << LED_R_PIN) | (1u << LED_G_PIN);
			mascara_d = (1u << LED_B_PIN);
			break;
		default:
			return;
	}

	/* Logica invertida: PCOR acende, PSOR apaga. */
	switch(acao)
	{
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
		default:
			break;
	}
}

static void mostra_ajuda(void)
{
	printk("\nComandos (controlam o LED da OUTRA placa):\n");
	printk("  r / g / b  - inverte o LED vermelho / verde / azul\n");
	printk("  1          - acende todos\n");
	printk("  0          - apaga todos\n");
	printk("  t          - troca o papel desta placa (TX <-> RX)\n");
	printk("  h          - mostra esta ajuda\n\n");
}

/* Traduz a tecla em um comando. Retorna false se a tecla nao for de LED. */
static bool tecla_para_comando(uint8_t tecla, comando_t *cmd)
{
	switch(tecla)
	{
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

static nrf24_modo_t g_modo_local;
static uint16_t     g_seq;

/* Trata uma tecla vinda do terminal. */
static void trata_tecla(uint8_t tecla)
{
	comando_t cmd;

	/* O terminal manda CR/LF junto com a tecla. Ignora sem reclamar. */
	if(tecla == '\r' || tecla == '\n')
	{
		return;
	}

	if(tecla == 'h')
	{
		mostra_ajuda();
		return;
	}

	if(tecla == 't')
	{
		g_modo_local = (g_modo_local == NRF24_MODO_RX) ? NRF24_MODO_TX
		                                               : NRF24_MODO_RX;
		nrf24_set_modo(g_modo_local);
		printk("papel agora: %s\n",
		       (g_modo_local == NRF24_MODO_RX) ? "RECEPTOR" : "TRANSMISSOR");
		return;
	}

	if(!tecla_para_comando(tecla, &cmd))
	{
		return;
	}

#if PAPEL == PAPEL_RX
	printk("placa em modo RECEPTOR - tecle 't' para transmitir\n");
#else
	cmd.seq = g_seq++;
	if(nrf24_send(&cmd, sizeof(cmd)))
	{
		printk("enviado '%c' acao %u (seq %u) - ACK ok\n",
		       cmd.comando, cmd.estado, cmd.seq);
	}
	else
	{
		printk("enviado '%c' (seq %u) - SEM ACK: a outra placa nao respondeu\n",
		       cmd.comando, cmd.seq);
	}
#endif
}

int main(void)
{
	uint8_t tecla;
	nrf24_modo_t modo;

	/* Zephyr leva um instante para subir o console do OpenSDA. */
	k_msleep(500);
	printk("\n=== FRDM-KL25Z + nRF24L01+ ===\n");

	led_init();

	if(!device_is_ready(uart_dev))
	{
		printk("console UART indisponivel\n");
		return -1;
	}

	/*
	 * SPI1 alternativa 0: PTE2 = SCK, PTE1 = MOSI, PTE3 = MISO.
	 * CS manual porque uma transacao do nRF24 mantem CSN baixo por varios
	 * bytes - o CS automatico do KL25Z sobe entre cada byte.
	 */
	spi_init(SPI_1, ALT_0, PRESCALE_2, DIVISOR_1, CS_MAN);

	nrf24_diag();

#if PAPEL == PAPEL_TX
	modo = NRF24_MODO_TX;
	printk("Papel: TRANSMISSOR\n");
#else
	modo = NRF24_MODO_RX;
	printk("Papel: %s\n", (PAPEL == PAPEL_RX) ? "RECEPTOR" : "BIDIRECIONAL");
#endif

	if(!nrf24_init(endereco, RF_CANAL, sizeof(comando_t), modo))
	{
		printk("nrf24_init falhou - ver diagnostico acima.\n");
		for(;;)
		{
			/* Pisca rapido: radio nao responde no SPI. */
			GPIOD->PTOR = (1u << LED_B_PIN);
			k_msleep(100);
		}
	}

	printk("Radio pronto. Canal %d, payload %d bytes.\n",
	       RF_CANAL, (int)sizeof(comando_t));
	mostra_ajuda();

	g_modo_local = modo;

	for(;;)
	{
		/*
		 * Destrava o receptor do UART antes de ler.
		 *
		 * O LPSCI do KL25Z tem buffer de recepcao de UM byte, e o poll_in do
		 * Zephyr so testa RDRF - nunca limpa OR/FE/PE. Como nrf24_send()
		 * bloqueia por dezenas de ms esperando o ACK, os caracteres que
		 * chegam nesse meio tempo estouram o buffer, OR trava setado e o
		 * receptor fica surdo permanentemente. uart_err_check() limpa essas
		 * flags (o driver da NXP faz isso a cada byte lido; o do Zephyr nao).
		 */
		(void)uart_err_check(uart_dev);

		/* Drena TUDO que chegou, nao so um caractere por volta. */
		while(uart_poll_in(uart_dev, &tecla) == 0)
		{
			trata_tecla(tecla);
			(void)uart_err_check(uart_dev);
		}

		/* --- Pacote recebido --- */
		if(g_modo_local == NRF24_MODO_RX && nrf24_available())
		{
			comando_t rx;

			nrf24_read(&rx, sizeof(rx));
			led_aplica(rx.comando, rx.estado);
			printk("recebido '%c' acao %u (seq %u)\n",
			       rx.comando, rx.estado, rx.seq);
		}

		k_msleep(5);
	}

	return 0;
}
