/*
 * main.c
 * Comunicacao wireless entre duas placas FRDM-KL25Z com nRF24L01+.
 *
 * Pelo terminal do PC (porta COM virtual do OpenSDA) voce controla o LED RGB
 * da OUTRA placa. As duas placas rodam este mesmo firmware.
 *
 * A camada de radio esta em lib/nrf24. A camada SPI, em lib/spi.
 *
 * ============================================================================
 * COMO USAR
 * ============================================================================
 *
 *   1. Grave este mesmo binario nas DUAS placas.
 *   2. Abra um terminal serial (115200 8N1) na porta COM de cada uma.
 *   3. Digite 'r' numa placa: o LED vermelho da OUTRA inverte.
 *
 * O terminal confirma cada envio com ACK ok / SEM ACK, entao da para saber na
 * hora se o link caiu sem precisar olhar a outra placa.
 *
 * ---------------------------------------------------------------------------
 * FLUXO DO PROGRAMA
 * ---------------------------------------------------------------------------
 *
 *   main()
 *     |- led_init()      configura os 3 pinos do LED RGB
 *     |- spi_init()      liga o SPI1 nos pinos PTE1/2/3
 *     |- nrf24_diag()    testa a fiacao do radio e imprime o resultado
 *     |- nrf24_init()      configura o radio (canal, endereco, payload, papel)
 *     |- nrf24_irq_init()  liga a interrupcao do pino IRQ (PTA16)
 *     `- laco infinito:
 *          |- le teclas do UART       -> monta comando -> nrf24_send()
 *          `- se a ISR avisou (IRQ)   -> nrf24_read()  -> led_aplica()
 *
 * RECEPCAO por interrupcao, o resto por polling.
 *
 * O radio baixa o pino IRQ quando um pacote entra na FIFO. A ISR (dentro de
 * lib/nrf24) nao fala com o radio: ela so marca uma flag, e o laco principal
 * ve essa flag em nrf24_irq_recebido() e ai sim faz as transacoes SPI. Manter
 * todo o SPI num contexto so evita que uma leitura disparada por interrupcao
 * caia no meio do nrf24_send(), que segura o barramento por dezenas de ms.
 *
 * O teclado continua sendo lido por polling: o UART do console nao vale a
 * complexidade de uma segunda interrupcao para umas poucas teclas por segundo.
 *
 * ---------------------------------------------------------------------------
 * LIGACAO (identica nas duas placas)
 * ---------------------------------------------------------------------------
 *
 *   nRF24L01+        FRDM-KL25Z
 *     VCC     ->     3.3V     (NUNCA 5V)
 *     GND     ->     GND
 *     SCK     ->     PTE2     (SPI1_SCK)
 *     MOSI    ->     PTE1     (SPI1_MOSI)
 *     MISO    ->     PTE3     (SPI1_MISO)
 *     CSN     ->     PTE4     (GPIO)
 *     CE      ->     PTE5     (GPIO)
 *     IRQ     ->     PTA16    (GPIO com interrupcao, ativo em BAIXO)
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

/*
 * Escolha do papel em tempo de compilacao. Os #if espalhados pelo arquivo
 * usam esta constante para incluir ou remover trechos inteiros de codigo.
 */
#define PAPEL   PAPEL_BIDIRECIONAL

/*
 * Canal e endereco precisam ser IGUAIS nas duas placas.
 *
 * O canal define a frequencia (2400 MHz + canal). O 76 e uma escolha comum
 * porque fica acima da maior parte do trafego de Wi-Fi domestico.
 * O endereco e um identificador de 5 bytes: so radios com o mesmo endereco
 * conversam entre si, o que permite varios pares na mesma sala.
 */
#define RF_CANAL   76
static const uint8_t endereco[NRF24_ADDR_WIDTH] = { 'C', 'A', 'R', 'R', '1' };

/*
 * LED RGB da FRDM-KL25Z. Todos ativos em nivel BAIXO.
 *
 * "Ativo em baixo" porque o anodo comum do LED esta no 3.3V: o pino do
 * microcontrolador puxa o catodo para o GND para acender. Logo, nivel
 * baixo = aceso, nivel alto = apagado. Isso inverte a intuicao em todo
 * o codigo de LED abaixo.
 */
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
 *
 * Sem o 'packed' o compilador poderia inserir um byte de padding antes do
 * uint16_t para alinha-lo, mudando o tamanho da struct. Como o tamanho do
 * payload e fixo e combinado entre as duas placas, qualquer padding
 * inesperado desalinha a interpretacao do outro lado.
 *
 * Nota: as duas placas usam o mesmo compilador e a mesma arquitetura, entao
 * a ordem dos bytes do 'seq' (little-endian no Cortex-M0+) e a mesma nos dois
 * lados e nao precisa de conversao.
 */
typedef struct __attribute__((packed))
{
	uint8_t  comando;   /* 'r', 'g', 'b' ou 'a' (todos) */
	uint8_t  estado;    /* ACAO_APAGA / ACAO_ACENDE / ACAO_INVERTE */
	uint16_t seq;       /* contador de sequencia, para enxergar perda */
} comando_t;

/*
 * Handle do UART do console (a COM virtual do OpenSDA).
 *
 * DT_CHOSEN(zephyr_console) pega do devicetree qual periferico foi escolhido
 * como console para esta placa, entao nao ha nenhum nome de UART hardcoded
 * aqui. O mesmo handle serve para o printk (saida) e para o uart_poll_in
 * (entrada de teclas).
 */
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/*
 * Configura os tres pinos do LED RGB como saida digital, todos apagados.
 *
 * Passos obrigatorios para qualquer GPIO no KL25Z:
 *   1. SCGC5 - liga o clock da porta. Sem isso, escrever nos registradores
 *              da porta simplesmente nao tem efeito (e nao gera erro).
 *   2. PCR   - seleciona a funcao do pino. MUX(1) = GPIO.
 *   3. PDDR  - direcao: bit em 1 = saida.
 */
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

/*
 * Aplica uma acao (acender / apagar / inverter) a uma cor.
 *
 * Duas etapas: primeiro traduz a cor em mascaras de bits, depois aplica a
 * acao. Sao duas mascaras separadas porque o LED RGB da placa esta espalhado
 * em duas portas diferentes - vermelho e verde no PORTB, azul no PORTD.
 *
 * Cor 'a' significa "todos", e por isso monta as duas mascaras de uma vez.
 * Cor desconhecida (pacote corrompido, por exemplo) simplesmente retorna sem
 * fazer nada.
 */
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

	/*
	 * Logica invertida: PCOR acende, PSOR apaga.
	 *
	 * Os tres registradores sao atomicos e afetam apenas os bits da mascara:
	 *   PCOR (Clear)  - zera o pino  -> acende (ativo em baixo)
	 *   PSOR (Set)    - poe em 1     -> apaga
	 *   PTOR (Toggle) - inverte      -> alterna o estado atual
	 *
	 * O "if(mascara)" evita escrever 0 numa porta que nao tem nenhum LED
	 * envolvido nesta acao. Escrever 0 seria inofensivo, mas o teste deixa
	 * explicito que so a porta relevante e tocada.
	 */
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

/* Menu impresso no terminal na inicializacao e ao teclar 'h'. */
static void mostra_ajuda(void)
{
	printk("\nComandos (controlam o LED da OUTRA placa):\n");
	printk("  r / g / b  - inverte o LED vermelho / verde / azul\n");
	printk("  1          - acende todos\n");
	printk("  0          - apaga todos\n");
	printk("  t          - troca o papel desta placa (TX <-> RX)\n");
	printk("  h          - mostra esta ajuda\n\n");
}

/*
 * Traduz a tecla em um comando. Retorna false se a tecla nao for de LED.
 *
 * Preenche apenas 'comando' e 'estado'. O campo 'seq' e responsabilidade de
 * quem envia, em trata_tecla(), para o contador nao ser incrementado por
 * teclas que acabam nao virando pacote.
 */
static bool tecla_para_comando(uint8_t tecla, comando_t *cmd)
{
	switch(tecla)
	{
		/* Cores individuais alternam o estado atual do LED. */
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

/*
 * Estado global do programa.
 *
 * g_modo_local - papel atual desta placa. Precisa ser rastreado aqui, e nao
 *                so dentro do driver, porque o laco principal so consulta o
 *                radio por pacotes recebidos quando esta em RX.
 * g_seq        - contador de sequencia dos pacotes enviados. Serve para ver
 *                buracos na numeracao do lado receptor e detectar perdas.
 */
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

	/*
	 * 't' inverte o papel em tempo de execucao. Util para testar os dois
	 * sentidos do link sem recompilar nem regravar as placas.
	 */
	if(tecla == 't')
	{
		g_modo_local = (g_modo_local == NRF24_MODO_RX) ? NRF24_MODO_TX
		                                               : NRF24_MODO_RX;
		nrf24_set_modo(g_modo_local);
		printk("papel agora: %s\n",
		       (g_modo_local == NRF24_MODO_RX) ? "RECEPTOR" : "TRANSMISSOR");
		return;
	}

	/* Tecla que nao corresponde a nenhum comando de LED e ignorada. */
	if(!tecla_para_comando(tecla, &cmd))
	{
		return;
	}

/*
 * Compilada como PAPEL_RX, a placa nunca transmite - o envio nem entra no
 * binario. Nos outros papeis, envia e reporta o desfecho.
 */
#if PAPEL == PAPEL_RX
	printk("placa em modo RECEPTOR - tecle 't' para transmitir\n");
#else
	cmd.seq = g_seq++;
	/*
	 * nrf24_send() bloqueia ate o hardware confirmar (TX_DS) ou desistir
	 * (MAX_RT). O retorno vem do auto-ack do proprio radio, entao "ACK ok"
	 * significa que a outra placa REALMENTE recebeu o pacote - nao apenas
	 * que ele foi jogado no ar.
	 */
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
	 *
	 * PRESCALE_2 e DIVISOR_1 dividem o clock do barramento (24 MHz) por
	 * 4 * 32 = 128, dando ~187 kHz no SCK. Bem abaixo dos 10 MHz que o nRF24
	 * aceita: e uma escolha conservadora, que tolera jumper solto e
	 * protoboard sem terminacao. Se precisar de mais vazao, e aqui que se
	 * aumenta a velocidade.
	 */
	spi_init(SPI_1, ALT_0, PRESCALE_2, DIVISOR_1, CS_MAN);

	/* Imprime o teste de fiacao antes de tentar configurar o radio. */
	nrf24_diag();

	/*
	 * Papel inicial. Note que BIDIRECIONAL comeca em RX: a placa fica
	 * escutando por padrao e so vira transmissor pelos poucos milissegundos
	 * de cada nrf24_send(), voltando para RX logo em seguida. E isso que
	 * permite as duas placas se controlarem mutuamente com um so binario.
	 */
#if PAPEL == PAPEL_TX
	modo = NRF24_MODO_TX;
	printk("Papel: TRANSMISSOR\n");
#else
	modo = NRF24_MODO_RX;
	printk("Papel: %s\n", (PAPEL == PAPEL_RX) ? "RECEPTOR" : "BIDIRECIONAL");
#endif

	/*
	 * O payload e dimensionado por sizeof(comando_t): mudar a struct ajusta
	 * as duas pontas automaticamente, desde que as duas placas sejam
	 * recompiladas juntas.
	 */
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

	/*
	 * Interrupcao do IRQ. Depois do nrf24_init() porque e ele que escreve o
	 * CONFIG com as mascaras de TX_DS/MAX_RT - sem essa escrita o pino
	 * tambem seria baixado no fim de cada envio.
	 */
	nrf24_irq_init();

	printk("Radio pronto. Canal %d, payload %d bytes. IRQ em PTA16.\n",
	       RF_CANAL, (int)sizeof(comando_t));
	mostra_ajuda();

	g_modo_local = modo;

	/*
	 * Laco principal: alterna entre atender o teclado e atender o radio.
	 * Nenhuma das duas tarefas pode monopolizar o processador.
	 */
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

		/*
		 * Drena TUDO que chegou, nao so um caractere por volta.
		 *
		 * O uart_err_check() dentro do laco e necessario porque cada
		 * trata_tecla() pode bloquear em nrf24_send(), acumulando novos
		 * caracteres (e um novo overrun) antes da proxima leitura.
		 */
		while(uart_poll_in(uart_dev, &tecla) == 0)
		{
			trata_tecla(tecla);
			(void)uart_err_check(uart_dev);
		}

		/*
		 * --- Pacote recebido ---
		 *
		 * Nada de consultar a FIFO a cada volta: so entra aqui quando a ISR
		 * do pino IRQ avisou que chegou alguma coisa. nrf24_irq_recebido()
		 * ja limpa a flag, entao cada evento e tratado uma unica vez.
		 *
		 * O while() e essencial, nao e preciosismo. A interrupcao e por
		 * BORDA e a FIFO do radio guarda ate 3 pacotes: se dois chegarem
		 * colados, o pino desce uma vez so e ha uma unica borda para os
		 * dois. Lendo apenas um, o outro ficaria preso na FIFO ate um
		 * proximo pacote gerar borda nova - e FIFO cheia faz o radio parar
		 * de aceitar recepcao. Drenar ate esvaziar resolve os dois casos.
		 */
		if(nrf24_irq_recebido())
		{
			comando_t rx;

			while(nrf24_available())
			{
				nrf24_read(&rx, sizeof(rx));
				led_aplica(rx.comando, rx.estado);
				printk("recebido '%c' acao %u (seq %u)\n",
				       rx.comando, rx.estado, rx.seq);
			}
		}

		/*
		 * Cede o processador entre as voltas. Quem manda no tempo de
		 * resposta do radio agora e a interrupcao, nao este sleep - ele
		 * governa apenas a leitura do teclado, onde 5ms passa despercebido.
		 */
		k_msleep(5);
	}

	return 0;
}
