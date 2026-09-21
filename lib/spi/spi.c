/*
 * spi.c
 * Author: Evandro
 * https://github.com/evandro-teixeira/frdm-kl25z-spi
 * https://embarcados.com.br/biblioteca-spi-para-a-placa-frdm-kl25z/
 */
#include "spi.h"
 
bool spi_init(bool spi,bool alt,uint8_t pre, uint16_t div,bool cs)
{
	switch(spi)
	{
		case SPI_0:
			switch(alt)
			{
				case ALT_0:
					SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;      //Turn on clock to C module
					//SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;  	//Enable SPI0 clock
					if(cs == CS_AUT)						//Chip Select Auto
						PORTC->PCR[4] = PORT_PCR_MUX(0x2);     //Set PTC4 to mux 2 [SPI0_PCS0]
					PORTC->PCR[5] = PORT_PCR_MUX(0x2);    		//Set PTC5 to mux 2 [SPI0_SCK]
					PORTC->PCR[6] = PORT_PCR_MUX(0x2);    		//Set PTC6 to mux 2 [SPI0_MOSI]
					PORTC->PCR[7] = PORT_PCR_MUX(0x2);    		//Set PTC7 to mux 2 [SPIO_MISO]
				break;
 
				case ALT_1:
					SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;      //Turn on clock to A module
					//SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;   	//Enable SPI0 clock
					if(cs == CS_AUT)						//Chip Select Auto
						PORTA->PCR[14] = PORT_PCR_MUX(0x2);   	//Set PTA14 to mux 2 [SPI0_PCS0]
					PORTA->PCR[15] = PORT_PCR_MUX(0x2);    	//Set PTA15 to mux 2 [SPI0_SCK]
					PORTA->PCR[16] = PORT_PCR_MUX(0x2);    	//Set PTA16 to mux 2 [SPI0_MOSI]
					PORTA->PCR[17] = PORT_PCR_MUX(0x2);    	//Set PTA17 to mux 2 [SPIO_MISO]
				break;
 
				default:
					return false;
				break;
			}
			//Enable SPI0 clock
			SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
 
			//Chip Select Auto
			if(cs == CS_AUT)
				// Set SPI0 to Master & SS pin to auto SS & spi mode in 0,0
				SPI0->C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;
			else
				// Set SPI0 to Master
				SPI0->C1 = SPI_C1_MSTR_MASK;
 
			// Configure SPI Register C2
			// MODFEN so faz sentido com CS automatico. Com CS manual o pino SS
			// e um GPIO comum e MODFEN o transformaria em entrada de mode-fault:
			// um nivel baixo espurio limpa MSTR e desliga o SPI sem sinalizar erro.
			SPI0->C2 = (cs == CS_AUT) ? SPI_C2_MODFEN_MASK : 0;
 
			// Set baud rate prescale divisor to 6 & set baud rate divisor to 4 for baud rate of 1 Mhz
			SPI0->BR = (SPI_BR_SPPR(pre) | SPI_BR_SPR(div));    //  Mhz
 
			// Enable SPI0
			SPI0->C1 |= SPI_C1_SPE_MASK;
 
			return true;
		break;
 
		case SPI_1:
			switch(alt)
			{
				case ALT_0:
					SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;      //Turn on clock to E module
					//SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK;   	//Enable SPI1 clock
					if(cs == CS_AUT)						//Chip Select Auto
						PORTE->PCR[4] = PORT_PCR_MUX(0x2);     //Set PTE4 to mux 2 [SPI1_PCS0]
					PORTE->PCR[2] = PORT_PCR_MUX(0x2);    		//Set PTE2 to mux 2 [SPI1_SCK]
					PORTE->PCR[1] = PORT_PCR_MUX(0x2);    		//Set PTE1 to mux 2 [SPI1_MOSI]
					PORTE->PCR[3] = PORT_PCR_MUX(0x2);    		//Set PTE3 to mux 2 [SPI1_MISO]
				break;
 
				case ALT_1:
					SIM->SCGC5  |= SIM_SCGC5_PORTB_MASK;     //Turn on clock to B module
					//SIM->SCGC4  |= SIM_SCGC4_SPI1_MASK;  	//Enable SPI1 clock
					if(cs == CS_AUT)						//Chip Select Auto
						PORTB->PCR[10] = PORT_PCR_MUX(0x2);    //Set PTB10 to mux 2 [SPI1_PCS0]
					PORTB->PCR[11] = PORT_PCR_MUX(0x2);    	//Set PTB11 to mux 2 [SPI1_SCK]
					PORTB->PCR[16] = PORT_PCR_MUX(0x2);    	//Set PTB16 to mux 2 [SPI1_MOSI]
					PORTB->PCR[17] = PORT_PCR_MUX(0x2);    	//Set PTB17 to mux 2 [SPI1_MISO]
				break;
 
				default:
					return false;
				break;
			}
			//Enable SPI1 clock
			SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK;
 
			// Chip Select Auto
			if(cs == CS_AUT)
				// Set SPI0 to Master & SS pin to auto SS & spi mode in 0,0
				SPI1->C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;
			else
				// Set SPI0 to Master
				SPI1->C1 = SPI_C1_MSTR_MASK;
 
			// Configure SPI Register C2 - ver comentario no bloco do SPI_0
			SPI1->C2 = (cs == CS_AUT) ? SPI_C2_MODFEN_MASK : 0;
 
			// Set baud rate prescale divisor to 6 & set baud rate divisor to 4 for baud rate of 1 Mhz
			SPI1->BR = (SPI_BR_SPPR(pre) | SPI_BR_SPR(div));    //  Mhz
 
			// Enable SPI0
			SPI1->C1 |= SPI_C1_SPE_MASK;
 
			return true;
		break;
 
		default:
			return false; // error
		break;
	}
}
 
bool spi_send(bool spi,uint8_t data)
{
	switch(spi)
	{
		case SPI_0:
			//While buffer is not empty do nothing
			while(!(SPI_S_SPTEF_MASK & SPI0->S))
			{
				__asm("nop");
			}
			//Write char to SPI
			SPI0->D = data;
 
			return true;
		break;
 
		case SPI_1:
			//While buffer is not empty do nothing
			while(!(SPI_S_SPTEF_MASK & SPI1->S))
			{
				__asm("nop");
			}
			//Write char to SPI
			SPI1->D = data;
 
			return true;
		break;
 
		default:
			return false; //erro
		break;
	}
}
 
uint8_t spi_transfer(bool spi, uint8_t data)
{
	if(spi == SPI_0)
	{
		// Aguarda buffer de transmissao vazio
		while(!(SPI0->S & SPI_S_SPTEF_MASK))
		{
			__asm("nop");
		}
		SPI0->D = data;
		// Aguarda o byte recebido - le D para limpar SPRF
		while(!(SPI0->S & SPI_S_SPRF_MASK))
		{
			__asm("nop");
		}
		return SPI0->D;
	}
	else
	{
		while(!(SPI1->S & SPI_S_SPTEF_MASK))
		{
			__asm("nop");
		}
		SPI1->D = data;
		while(!(SPI1->S & SPI_S_SPRF_MASK))
		{
			__asm("nop");
		}
		return SPI1->D;
	}
}

uint8_t spi_read(bool spi)
{
	switch(spi)
	{
		case SPI_0:
			  // Wait for receive flag to set
			  while(!(SPI0->S & SPI_S_SPRF_MASK))
			  {
			    __asm("nop");
			  }
			  // SPI0_D;  //Read SPI data from slave
			  return SPI0->D;
		break;
 
		case SPI_1:
			  // Wait for receive flag to set
			  while(!(SPI1->S & SPI_S_SPRF_MASK))
			  {
			    __asm("nop");
			  }
			  // SPI1_D;  //Read SPI data from slave
			  return SPI1->D;
		break;
 
		default:
			return false;
		break;
	}
}