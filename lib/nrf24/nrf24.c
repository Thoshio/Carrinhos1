/*
 * nrf24.c
 * Driver nRF24L01 (Versão Original) refatorado.
 * Utilizando API nativa de GPIO do Zephyr para evitar conflito de IRQ.
 */

#include "nrf24.h"
#include "spi.h"
#include "MKL25Z4.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h> // Incluído para gerenciar o pino IRQ pelo SO

/* --- Comandos e Registradores SPI do NRF24 --- */
#define CMD_R_REGISTER     0x00
#define CMD_W_REGISTER     0x20
#define CMD_R_RX_PAYLOAD   0x61
#define CMD_W_TX_PAYLOAD   0xA0
#define CMD_FLUSH_TX       0xE1
#define CMD_FLUSH_RX       0xE2
#define CMD_NOP            0xFF

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

#define STATUS_RX_DR       (1 << 6)
#define STATUS_TX_DS       (1 << 5)
#define STATUS_MAX_RT      (1 << 4)

#define CONFIG_EN_CRC      (1 << 3)
#define CONFIG_CRCO        (1 << 2)
#define CONFIG_PWR_UP      (1 << 1)
#define CONFIG_PRIM_RX     (1 << 0)
#define FIFO_RX_EMPTY      (1 << 0)

/* --- MAPEAMENTO DE PINOS --- */
#define NRF_SPI            SPI_1
#define PIN_CE             13 // PTA13
#define PIN_CSN            5  // PTD5
#define PIN_IRQ            16 // PTA16 (Porta A, Pino 16)

#define CONFIG_BASE        (CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP)

static uint8_t      g_payload_len = 32;
static nrf24_modo_t g_modo        = NRF24_MODO_TX;

/* Semáforo binário usado para sincronizar a interrupção */
K_SEM_DEFINE(nrf24_irq_sem, 0, 1);

/* Estruturas do Zephyr para o Callback do GPIO */
static const struct device *gpio_a_dev;
static struct gpio_callback nrf24_irq_cb_data;

/* Funções de baixo nível para manuseio dos pinos CE e CSN */
static inline void csn_low(void)  { GPIOD->PCOR = (1u << PIN_CSN); }
static inline void csn_high(void) { GPIOD->PSOR = (1u << PIN_CSN); }
static inline void ce_low(void)   { GPIOA->PCOR = (1u << PIN_CE);  }
static inline void ce_high(void)  { GPIOA->PSOR = (1u << PIN_CE);  }

/* --- Callback Nativo do Zephyr (Substitui o antigo porta_isr) --- */
void nrf24_irq_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&nrf24_irq_sem); // Libera o semáforo para o loop principal
}

static void write_reg(uint8_t reg, uint8_t value) {
    csn_low();
    spi_transfer(NRF_SPI, CMD_W_REGISTER | reg);
    spi_transfer(NRF_SPI, value);
    csn_high();
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t value;
    csn_low();
    spi_transfer(NRF_SPI, CMD_R_REGISTER | reg);
    value = spi_transfer(NRF_SPI, CMD_NOP);
    csn_high();
    return value;
}

static void write_reg_buf(uint8_t reg, const uint8_t *buf, uint8_t len) {
    csn_low();
    spi_transfer(NRF_SPI, CMD_W_REGISTER | reg);
    for(uint8_t i = 0; i < len; i++) {
        spi_transfer(NRF_SPI, buf[i]);
    }
    csn_high();
}

static void send_cmd(uint8_t cmd) {
    csn_low();
    spi_transfer(NRF_SPI, cmd);
    csn_high();
}

static void pins_init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    /* Apenas CE e CSN mantidos via bare-metal */
    PORTA->PCR[PIN_CE] = PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1u << PIN_CE);

    PORTD->PCR[PIN_CSN] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1u << PIN_CSN);

    csn_high();
    ce_low();
}

uint8_t nrf24_status(void) {
    uint8_t status;
    csn_low();
    status = spi_transfer(NRF_SPI, CMD_NOP);
    csn_high();
    return status;
}

void nrf24_set_modo(nrf24_modo_t modo) {
    if(modo == NRF24_MODO_RX) {
        write_reg(REG_CONFIG, CONFIG_BASE | CONFIG_PRIM_RX);
        write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
        send_cmd(CMD_FLUSH_RX);
        ce_high();
    } else {
        ce_low();
        write_reg(REG_CONFIG, CONFIG_BASE);
    }
    k_busy_wait(150);
    g_modo = modo;
}

void nrf24_diag(void) {
    uint8_t lido_03, lido_02;
    pins_init();
    k_msleep(100);

    printk("\n--- Diagnóstico nRF24L01 (SCK=PTE2 MOSI=PTE1 MISO=PTE3 CSN=PTD5 CE=PTA13 IRQ=PTA16) ---\n");
    printk("STATUS bruto = 0x%02X\n", nrf24_status());

    write_reg(REG_SETUP_AW, 0x03);
    lido_03 = read_reg(REG_SETUP_AW);
    write_reg(REG_SETUP_AW, 0x02);
    lido_02 = read_reg(REG_SETUP_AW);
    write_reg(REG_SETUP_AW, 0x03);

    printk("SETUP_AW: escrevi 0x03 li 0x%02X | escrevi 0x02 li 0x%02X\n", lido_03, lido_02);
    
    if(lido_03 == 0x03 && lido_02 == 0x02) {
        printk("=> SPI OK, módulo respondendo.\n");
    } else {
        printk("=> Falha de SPI. Verifique fiação ou alimentação.\n");
    }
}

bool nrf24_init(const uint8_t *address, uint8_t channel, uint8_t payload_len, nrf24_modo_t modo) {
    g_payload_len = payload_len;
    pins_init();
    k_msleep(100);

    /* INICIALIZAÇÃO DA INTERRUPÇÃO PELA API DO ZEPHYR (RESOLVE O ERRO DE MÚLTIPLOS REGISTROS) */
    gpio_a_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa)); // Captura a referência nativa da Porta A
    
    if (device_is_ready(gpio_a_dev)) {
        // Configura o pino 16 como entrada, com resistor pull-up
        gpio_pin_configure(gpio_a_dev, PIN_IRQ, GPIO_INPUT | GPIO_PULL_UP);
        // Solicita interrupção na borda de descida (quando módulo puxa para LOW)
        gpio_pin_interrupt_configure(gpio_a_dev, PIN_IRQ, GPIO_INT_EDGE_FALLING);
        
        // Registra nossa função para ser chamada pelo Zephyr
        gpio_init_callback(&nrf24_irq_cb_data, nrf24_irq_callback, BIT(PIN_IRQ));
        gpio_add_callback(gpio_a_dev, &nrf24_irq_cb_data);
    } else {
        printk("Erro: Controlador GPIOA não está pronto no Zephyr!\n");
        return false;
    }

    write_reg(REG_SETUP_AW, 0x03);
    if(read_reg(REG_SETUP_AW) != 0x03) return false;

    write_reg(REG_SETUP_RETR, (5 << 4) | 15);
    write_reg(REG_RF_SETUP, 0x03); // 1 Mbps
    write_reg(REG_RF_CH, channel);
    
    write_reg_buf(REG_TX_ADDR, address, NRF24_ADDR_WIDTH);
    write_reg_buf(REG_RX_ADDR_P0, address, NRF24_ADDR_WIDTH);
    write_reg(REG_RX_PW_P0, payload_len);
    
    write_reg(REG_EN_AA, 0x01);
    write_reg(REG_EN_RXADDR, 0x01);

    send_cmd(CMD_FLUSH_TX);
    send_cmd(CMD_FLUSH_RX);
    write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    
    nrf24_set_modo(modo);
    k_msleep(2);
    return true;
}

bool nrf24_send(const void *data, uint8_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    nrf24_modo_t modo_anterior = g_modo;
    uint8_t status;
    
    if(g_modo != NRF24_MODO_TX) nrf24_set_modo(NRF24_MODO_TX);
    
    write_reg(REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);
    
    csn_low();
    spi_transfer(NRF_SPI, CMD_W_TX_PAYLOAD);
    for(uint8_t i = 0; i < g_payload_len; i++) {
        spi_transfer(NRF_SPI, (i < len) ? bytes[i] : 0x00);
    }
    csn_high();

    ce_high();
    k_busy_wait(15);
    ce_low();

    if(k_sem_take(&nrf24_irq_sem, K_MSEC(50)) != 0) {
        status = nrf24_status();
    } else {
        status = nrf24_status();
    }

    if((status & STATUS_TX_DS) == 0) {
        send_cmd(CMD_FLUSH_TX);
    }
    
    write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    
    if(modo_anterior != NRF24_MODO_TX) nrf24_set_modo(modo_anterior);
    
    return (status & STATUS_TX_DS) != 0;
}

bool nrf24_irq_occurred(void) {
    return k_sem_take(&nrf24_irq_sem, K_NO_WAIT) == 0;
}

bool nrf24_available(void) {
    return (read_reg(REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0;
}

void nrf24_read(void *buf, uint8_t len) {
    uint8_t *bytes = (uint8_t *)buf;
    
    csn_low();
    spi_transfer(NRF_SPI, CMD_R_RX_PAYLOAD);
    for(uint8_t i = 0; i < g_payload_len; i++) {
        uint8_t b = spi_transfer(NRF_SPI, CMD_NOP);
        if(i < len) bytes[i] = b;
    }
    csn_high();
    
    write_reg(REG_STATUS, STATUS_RX_DR);
}