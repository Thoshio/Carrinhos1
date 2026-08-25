/*
 * nrf24.c
 * Driver nRF24L01+ para FRDM-KL25Z.
 *
 * ============================================================================
 * COMO O nRF24L01+ FUNCIONA (resumo do que importa para este driver)
 * ============================================================================
 *
 * O modulo e um transceptor de 2.4 GHz controlado por dois caminhos separados:
 *
 *   1) SPI  - barramento de configuracao e de dados. Por ele o KL25Z le e
 *             escreve registradores e enche/esvazia as FIFOs de payload.
 *             Quem controla o SPI e o pino CSN (chip select, ativo em BAIXO).
 *
 *   2) CE   - pino dedicado que liga/desliga o RADIO em si, independente do
 *             SPI. E o CE que diz "comece a ouvir" (em RX) ou "transmita o que
 *             esta na FIFO agora" (em TX).
 *
 * Essa separacao e a fonte de quase toda confusao com esse chip: configurar
 * pelo SPI nao transmite nada; e o CE que dispara. E manter o CE alto em RX
 * e o que mantem o receptor acordado.
 *
 * ---------------------------------------------------------------------------
 * ESTADOS QUE ESTE DRIVER USA
 * ---------------------------------------------------------------------------
 *
 *   PRX (recepcao)     - CONFIG.PRIM_RX = 1, CE alto continuamente.
 *                        O radio escuta e joga o que chegar na FIFO de RX.
 *
 *   PTX (transmissao)  - CONFIG.PRIM_RX = 0, CE normalmente baixo.
 *                        Um pulso de >=10us em CE dispara o envio de UM pacote.
 *
 * ---------------------------------------------------------------------------
 * AUTO-ACK (Enhanced ShockBurst)
 * ---------------------------------------------------------------------------
 *
 * Com EN_AA ligado, o hardware faz sozinho o handshake:
 *
 *   PTX envia o pacote  ->  PRX recebe, valida o CRC e devolve um ACK
 *                       ->  PTX recebe o ACK e seta TX_DS ("data sent")
 *
 * Se o ACK nao vier, o PTX retransmite sozinho ate ARC vezes, esperando ARD
 * entre cada tentativa (configurados em SETUP_RETR). Esgotadas as tentativas
 * ele seta MAX_RT ("max retransmits") e desiste.
 *
 * Ou seja: TX_DS = a outra placa REALMENTE recebeu. MAX_RT = ninguem respondeu.
 * E por isso que nrf24_send() consegue devolver true/false de forma confiavel
 * sem nenhum protocolo de aplicacao por cima.
 *
 * Detalhe importante: para o ACK voltar, o PTX escuta brevemente no endereco
 * do PIPE 0. Por isso RX_ADDR_P0 tem que ser IGUAL a TX_ADDR - ver nrf24_init().
 *
 * ---------------------------------------------------------------------------
 * FORMATO DAS TRANSACOES SPI
 * ---------------------------------------------------------------------------
 *
 * Toda transacao segue o mesmo molde:
 *
 *   CSN baixo -> byte de COMANDO -> N bytes de dado -> CSN alto
 *
 * O CSN precisa ficar baixo durante a transacao INTEIRA. Se subir no meio, o
 * chip aborta e interpreta o resto como um comando novo. E exatamente por isso
 * que o main.c inicializa o SPI com CS_MAN (chip select manual): o CS
 * automatico do KL25Z sobe entre cada byte e quebraria toda transacao de mais
 * de um byte.
 *
 * Outro detalhe do chip: o primeiro byte que ele devolve em QUALQUER transacao
 * e sempre o registrador STATUS. E por isso que nrf24_status() e so um NOP -
 * manda um byte inofensivo e aproveita o STATUS que volta junto.
 */

#include "nrf24.h"
#include "spi.h"
#include "MKL25Z4.h"
#include <zephyr/kernel.h>

/* --- Comandos SPI ---
 * Sao os opcodes do primeiro byte de cada transacao.
 * R_REGISTER e W_REGISTER sao mascaras: o numero do registrador vai somado
 * (OR) nos 5 bits de baixo, por isso "CMD_W_REGISTER | reg" mais abaixo.
 */
#define CMD_R_REGISTER     0x00   /* le registrador  (0x00 | reg) */
#define CMD_W_REGISTER     0x20   /* escreve registrador (0x20 | reg) */
#define CMD_R_RX_PAYLOAD   0x61   /* le e remove o pacote do topo da FIFO RX */
#define CMD_W_TX_PAYLOAD   0xA0   /* escreve um pacote na FIFO TX */
#define CMD_FLUSH_TX       0xE1   /* descarta tudo que esta na FIFO TX */
#define CMD_FLUSH_RX       0xE2   /* descarta tudo que esta na FIFO RX */
#define CMD_NOP            0xFF   /* nao faz nada; serve so para ler o STATUS */

/* --- Registradores ---
 * Enderecos internos do chip, acessados via R_REGISTER / W_REGISTER.
 */
#define REG_CONFIG         0x00   /* liga o radio, escolhe TX/RX, CRC */
#define REG_EN_AA          0x01   /* habilita auto-ack por pipe */
#define REG_EN_RXADDR      0x02   /* habilita cada um dos 6 pipes de recepcao */
#define REG_SETUP_AW       0x03   /* largura do endereco (3, 4 ou 5 bytes) */
#define REG_SETUP_RETR     0x04   /* ARD (espera) e ARC (num. de retentativas) */
#define REG_RF_CH          0x05   /* canal RF: 2400 MHz + canal, em MHz */
#define REG_RF_SETUP       0x06   /* taxa no ar e potencia do amplificador */
#define REG_STATUS         0x07   /* flags de evento; escrever 1 LIMPA o bit */
#define REG_RX_ADDR_P0     0x0A   /* endereco que o pipe 0 escuta */
#define REG_TX_ADDR        0x10   /* endereco para onde este radio transmite */
#define REG_RX_PW_P0       0x11   /* tamanho fixo do payload no pipe 0 */
#define REG_FIFO_STATUS    0x17   /* estado das FIFOs de TX e RX */
#define REG_DYNPD          0x1C   /* payload de tamanho dinamico por pipe */
#define REG_FEATURE        0x1D   /* habilita recursos extras (DPL, ack payload) */

/* --- Bits do STATUS ---
 * Sao flags "grudentas": o hardware seta, e o software precisa limpar
 * escrevendo 1 de volta no bit (write-1-to-clear). Se voce nao limpar, o
 * radio trava no proximo ciclo.
 */
#define STATUS_RX_DR       (1 << 6)  /* Data Ready: chegou pacote na FIFO RX */
#define STATUS_TX_DS       (1 << 5)  /* Data Sent: pacote enviado E confirmado */
#define STATUS_MAX_RT      (1 << 4)  /* estourou o numero de retransmissoes */

/* --- Bits do CONFIG --- */
#define CONFIG_EN_CRC      (1 << 3)  /* liga a verificacao de CRC */
#define CONFIG_CRCO        (1 << 2)  /* CRC de 16 bits (sem este bit, 8) */
#define CONFIG_PWR_UP      (1 << 1)  /* tira o radio do power-down */
#define CONFIG_PRIM_RX     (1 << 0)  /* 1 = receptor (PRX), 0 = transmissor */

/* --- Bits do FIFO_STATUS --- */
#define FIFO_RX_EMPTY      (1 << 0)  /* 1 = nao ha nada para ler na FIFO RX */

/* Qual periferico SPI do KL25Z esta ligado ao radio. */
#define NRF_SPI            SPI_1

/* CSN = PTE4, CE = PTE5 - os dois como GPIO comum, nao como funcao do SPI. */
#define PIN_CSN            4
#define PIN_CE             5

/*
 * CONFIG base: CRC de 16 bits, radio ligado.
 * O bit PRIM_RX e adicionado (ou nao) em nrf24_set_modo(), que e o unico
 * lugar que decide entre TX e RX.
 */
#define CONFIG_BASE        (CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP)

/*
 * Estado do driver.
 *
 * g_payload_len - tamanho fixo do pacote, definido em nrf24_init(). Como o
 *                 modo e de payload ESTATICO, toda transacao de payload tem
 *                 que ter exatamente esse tamanho, nem mais nem menos.
 * g_modo        - papel atual, para nrf24_send() saber se precisa trocar de
 *                 modo antes de transmitir e restaurar depois.
 */
static uint8_t      g_payload_len = 32;
static nrf24_modo_t g_modo        = NRF24_MODO_TX;

/*
 * Manipulacao direta dos pinos via registradores do GPIO do KL25Z:
 *   PCOR (Port Clear Output Register) - zera os bits marcados na mascara
 *   PSOR (Port Set Output Register)   - seta os bits marcados na mascara
 * Sao registradores atomicos: escrever neles nao mexe nos outros pinos da
 * porta, entao nao precisa de leitura-modificacao-escrita.
 *
 * CSN e ativo em BAIXO: baixo = SPI selecionado.
 */
static inline void csn_low(void)  { GPIOE->PCOR = (1u << PIN_CSN); }
static inline void csn_high(void) { GPIOE->PSOR = (1u << PIN_CSN); }
static inline void ce_low(void)   { GPIOE->PCOR = (1u << PIN_CE);  }
static inline void ce_high(void)  { GPIOE->PSOR = (1u << PIN_CE);  }

/*
 * Escreve um byte em um registrador.
 * Transacao: [W_REGISTER|reg] [valor]
 */
static void write_reg(uint8_t reg, uint8_t value)
{
	csn_low();
	spi_transfer(NRF_SPI, CMD_W_REGISTER | reg);
	spi_transfer(NRF_SPI, value);
	csn_high();
}

/*
 * Le um byte de um registrador.
 * Transacao: [R_REGISTER|reg] [NOP para gerar clock]
 *
 * O SPI e full-duplex: para RECEBER um byte e preciso ENVIAR um byte, porque
 * e o mestre quem gera o clock. O NOP e so um byte inofensivo cuja unica
 * funcao e produzir os 8 pulsos de clock que trazem a resposta pelo MISO.
 */
static uint8_t read_reg(uint8_t reg)
{
	uint8_t value;

	csn_low();
	spi_transfer(NRF_SPI, CMD_R_REGISTER | reg);
	value = spi_transfer(NRF_SPI, CMD_NOP);
	csn_high();

	return value;
}

/*
 * Escreve varios bytes num registrador, numa unica transacao com o CSN
 * baixo do inicio ao fim. Usado para os enderecos, que tem 5 bytes.
 */
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

/* Envia um comando avulso, sem dado (FLUSH_TX, FLUSH_RX, ...). */
static void send_cmd(uint8_t cmd)
{
	csn_low();
	spi_transfer(NRF_SPI, cmd);
	csn_high();
}

/*
 * Configura CSN e CE como saidas digitais comuns.
 *
 * PORT_PCR_MUX(1) = funcao ALT1 do pino, que no KL25Z e sempre "GPIO".
 * (ALT2 no PTE4 seria SPI1_PCS0, o chip select automatico - que NAO queremos,
 *  ver o comentario sobre CS_MAN no topo do arquivo.)
 * PDDR = Port Data Direction Register: bit em 1 significa saida.
 *
 * Estado inicial seguro: CSN alto (SPI nao selecionado) e CE baixo (radio
 * parado, nem ouvindo nem transmitindo).
 */
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

/*
 * Le o registrador STATUS.
 *
 * Aproveita a caracteristica do chip de devolver o STATUS no primeiro byte de
 * toda transacao: manda um NOP (que nao altera nada) e fica com a resposta.
 */
uint8_t nrf24_status(void)
{
	uint8_t status;

	csn_low();
	status = spi_transfer(NRF_SPI, CMD_NOP);
	csn_high();

	return status;
}

/*
 * Troca entre PRX (receptor) e PTX (transmissor).
 *
 * Alem de mexer no bit PRIM_RX do CONFIG, esta funcao cuida do CE, que e o
 * que efetivamente coloca o radio para funcionar em cada papel.
 */
void nrf24_set_modo(nrf24_modo_t modo)
{
	if(modo == NRF24_MODO_RX)
	{
		write_reg(REG_CONFIG, CONFIG_BASE | CONFIG_PRIM_RX);
		/* Limpa eventos pendentes do papel anterior para nao ler lixo. */
		write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
		/* Descarta pacotes velhos que sobraram na FIFO de recepcao. */
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

/*
 * Teste de fiacao, para rodar ANTES de nrf24_init().
 *
 * A ideia: escrever dois valores DIFERENTES num registrador inofensivo e ler
 * cada um de volta. Um unico teste de leitura nao distingue "o radio
 * respondeu 0xFF" de "o MISO esta solto e o pull-up devolve 0xFF sempre".
 * Com dois valores distintos, so um MISO realmente funcionando consegue
 * acompanhar a mudanca.
 *
 * SETUP_AW e o registrador escolhido porque so define a largura do endereco:
 * mexer nele temporariamente nao estraga nada, e no fim voltamos para 0x03.
 */
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

	/* As leituras acompanharam as escritas: o caminho SPI inteiro esta bom. */
	if(lido_03 == 0x03 && lido_02 == 0x02)
	{
		printk("=> SPI OK, radio respondendo.\n");
	}
	/* Tudo 1: o pino esta flutuando com pull-up, ninguem dirige o MISO. */
	else if(lido_03 == 0xFF && lido_02 == 0xFF)
	{
		printk("=> MISO preso em 1. PTE3 solto, ou modulo sem 3.3V.\n");
	}
	/* Tudo 0: o MISO esta amarrado ao GND, ou o chip nunca foi selecionado. */
	else if(lido_03 == 0x00 && lido_02 == 0x00)
	{
		printk("=> MISO preso em 0. PTE3 no GND, CSN nao chega ao modulo,\n");
		printk("   ou o modulo nao esta alimentado.\n");
	}
	/* Respondeu, mas errado: contato intermitente ou clock com ruido. */
	else
	{
		printk("=> Resposta inconsistente. Fio ruim, mau contato ou ruido.\n");
	}
	printk("------------------------------------------------------------\n");
}

/*
 * Configura o radio do zero e o deixa pronto no papel pedido.
 *
 * Pre-requisito: spi_init() ja chamado pelo main.
 * Retorna false se o radio nao responder no SPI.
 */
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

	/*
	 * Retransmissao automatica: ARD = 1500us, ARC = 15 tentativas.
	 *
	 * ARD sao os 4 bits de cima, em passos de 250us: (5+1) * 250us = 1500us.
	 * ARC sao os 4 bits de baixo: quantas vezes retransmitir antes de MAX_RT.
	 *
	 * 1500us e folgado de proposito. O ARD precisa ser maior que o tempo do
	 * pacote de ACK completo; com margem grande o link fica estavel mesmo com
	 * alimentacao ruim. O custo e o pior caso de 15 * 1500us = 22.5ms parado
	 * dentro de nrf24_send() quando a outra placa esta desligada.
	 */
	write_reg(REG_SETUP_RETR, (5 << 4) | 15);

	/*
	 * PA_LOW + 1 Mbps. A taxa precisa ser igual nas duas placas.
	 *
	 * Potencia baixa e proposital: os modulos genericos sem capacitor de
	 * desacoplamento sofrem brownout no pico de corrente da transmissao em
	 * potencia alta. Para duas placas na mesma bancada, PA_LOW sobra.
	 * 1 Mbps (em vez de 2 Mbps) tambem melhora o alcance e a robustez.
	 */
	write_reg(REG_RF_SETUP, 0x03);

	/*
	 * Payload estatico: tamanho fixo, sempre g_payload_len bytes.
	 * FEATURE = 0 desliga os recursos estendidos, DYNPD = 0 desliga o
	 * tamanho dinamico por pipe. Assim os dois lados sempre concordam sobre
	 * quantos bytes tem um pacote, sem negociacao.
	 */
	write_reg(REG_FEATURE, 0x00);
	write_reg(REG_DYNPD, 0x00);

	/* Frequencia = 2400 MHz + canal. Canal 76 => 2476 MHz. */
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

	/* Comeca de FIFOs limpas e sem flags de evento penduradas. */
	send_cmd(CMD_FLUSH_TX);
	send_cmd(CMD_FLUSH_RX);
	write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

	nrf24_set_modo(modo);

	/* Datasheet: 1.5ms de standby apos PWR_UP. */
	k_msleep(2);

	return true;
}

/*
 * Envia um pacote e espera a confirmacao do outro lado.
 *
 * Funcao BLOQUEANTE: so retorna quando o hardware sinalizar TX_DS (entregue)
 * ou MAX_RT (desistiu). No pior caso isso leva ~22.5ms - por isso o main.c
 * precisa se preocupar com os caracteres do UART que chegam nesse intervalo.
 *
 * Retorna true se o outro radio confirmou o recebimento.
 */
bool nrf24_send(const void *data, uint8_t len)
{
	const uint8_t *bytes = (const uint8_t *)data;
	nrf24_modo_t modo_anterior = g_modo;
	uint8_t status = 0;
	uint8_t i;
	uint32_t timeout;

	/*
	 * Se a placa estava escutando, vira transmissor so pelo tempo do envio.
	 * O papel anterior e restaurado no fim da funcao - e isso que permite o
	 * modo bidirecional, onde as duas placas ficam em RX quase o tempo todo.
	 */
	if(g_modo != NRF24_MODO_TX)
	{
		nrf24_set_modo(NRF24_MODO_TX);
	}

	/* Limpa resultados do envio anterior; senao lemos o desfecho errado. */
	write_reg(REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);

	/* Carrega o payload na FIFO de transmissao (ainda nao transmite nada). */
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
	 *
	 * (500 voltas x 100us = 50ms de teto. O laco sai antes assim que o
	 *  hardware sinalizar um dos dois desfechos.)
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

	/* Deixa o STATUS limpo para a proxima chamada. */
	write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

	/* Volta a escutar, se era esse o papel antes do envio. */
	if(modo_anterior != NRF24_MODO_TX)
	{
		nrf24_set_modo(modo_anterior);
	}

	return (status & STATUS_TX_DS) != 0;
}

/*
 * Ha pacote esperando para ser lido?
 *
 * Consulta a FIFO diretamente (bit RX_EMPTY) em vez do flag RX_DR do STATUS.
 * A FIFO guarda ate 3 pacotes, e o RX_DR pode ficar dessincronizado dela
 * quando chegam varios pacotes seguidos. Olhar a FIFO e sempre a verdade.
 */
bool nrf24_available(void)
{
	return (read_reg(REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0;
}

/*
 * Le o pacote do topo da FIFO de recepcao.
 *
 * Como o payload e estatico, e OBRIGATORIO drenar os g_payload_len bytes do
 * SPI mesmo que o chamador queira menos (len < g_payload_len). Parar no meio
 * deixaria bytes na FIFO e dessincronizaria todas as leituras seguintes - por
 * isso o laco sempre vai ate o fim e so o "if(i < len)" decide o que copiar.
 */
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

	/* Marca o evento como tratado (write-1-to-clear). */
	write_reg(REG_STATUS, STATUS_RX_DR);
}
