//jj kazazian 2018//

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "applet.h"
#include "registers_name.h"
/*----------------------------------------------------------------------------
 *        modules Headers (clock,...)
 *----------------------------------------------------------------------------*/
#include "clock.h"
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Pins to configure for the application. */
static const Pin spi_pins[] = {
	PIN_SPI_MISO,
	PIN_SPI_MOSI,
	PIN_SPI_SPCK,
	PIN_SPI_NPCS3
};

struct _SPI { // from 301

    /** Input Arguments*/ 
         uint8_t rw;  
         uint8_t addr;    
         uint8_t data;
};


/** SPI Clock setting (Hz) */
static uint32_t spiClock     = 600000;
static uint32_t dbg_baudrate = 115200;


#define SPI0_CS0  0
#define SPI0_CS1  1
#define SPI0_CS2  2
#define SPI0_CS3  3


/** SPI clock configuration */
static const uint32_t clockConfigurations[3] = { 600000, 2000000, 4000000};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

static void SetClockConfiguration(uint8_t configuration)
{
	spiClock = clockConfigurations[configuration];
	printf("-I- Setting SPI master clock #%u ... \n\r",
			(unsigned int)clockConfigurations[configuration]);
}

static void waitKey(void)
{
	printf("-I- Press any key to Continue...\n\r");
	while (1) {
		if (DBG_GetChar()!= 0)
			break;
	}
}



static void Init_SPI(Spi *spi, uint8_t dwNpcs) 
{// from 301    
    /* Disable write protection*/
    /* Enable the SPI clock*/
    PMC_EnablePeripheral(ID_SPI0) ;

    /* Configure SPI in Master Mode with CS selected !!! */
    SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS(dwNpcs));
    SetClockConfiguration(0);
    SPI_ConfigureNPCS(spi, dwNpcs, SPI_SCBR( spiClock, BOARD_MCK));
    //printf("-I- spi clock ratio: %x \n\r", BOARD_MCK/spiClock);
    /* 16 bits mode transfer*/
    spi->SPI_CSR[dwNpcs] |=  SPI_CSR_BITS_16_BIT; 
    
    spi->SPI_CSR[dwNpcs] |= SPI_CSR_DLYBS(1);
    spi->SPI_CSR[dwNpcs] |= SPI_CSR_DLYBCT(64);
    spi->SPI_CSR[dwNpcs] |= SPI_CSR_NCPHA;
    spi->SPI_CSR[dwNpcs] |= SPI_CSR_CPOL;
    
    /* Enable the SPI */
    SPI_Enable(spi) ; 
   
}

static bool check_RDRF(Spi *spi) {
        bool     wait=true;
        bool     status=false;
        uint8_t  rdrf;
        uint32_t n=0;
        
while (wait){
                   //Wait(1);  
                   n++;
                   rdrf = spi->SPI_SR & SPI_SR_RDRF;
                   printf("-I- RDRF: %x \n\r", rdrf);
                   if (n>1) {if (rdrf == 0) wait=true; else wait=false;}
                   if (n>15) {wait=false;} // timeout
         };
 if  (rdrf == 0) status=false; else status=true;      
        
return status;
} 

static uint8_t SENSE_READ_16(Spi *spi, uint8_t dwNpcs, uint8_t addr)
{// jjk
        uint16_t data;
        uint16_t wdata;
        bool  wait=true;
        bool  status;
        uint32_t n=0;
        data = spi->SPI_RDR;  // clear rdrf;
        wdata = (addr|0x80)<< 8 | (0xff); // 1+addr+data

  	while ((spi->SPI_SR & SPI_SR_TXEMPTY) == 0);	
        spi->SPI_TDR = wdata | SPI_PCS(dwNpcs); // write address
        while ((spi->SPI_SR & SPI_SR_TDRE) == 0);
         
        if (check_RDRF(spi)) data = spi->SPI_RDR; // read data
         
        TRACE_INFO("READ COMMAND REG[0x%x]=0x%04x\n\r", addr, data);	  
        return data;  
}



static void SENSE_WRITE_16(Spi *spi, uint8_t dwNpcs, uint8_t addr, volatile  uint8_t Data)
{// jjk
  uint16_t data = 0;
  uint16_t rdata;
  data = (addr&0x7F)<< 8 | Data;
 
        while ((spi->SPI_SR & SPI_SR_TXEMPTY) == 0);	
        spi->SPI_TDR = data | SPI_PCS(dwNpcs);   // Write address+data
        while ((spi->SPI_SR & SPI_SR_TDRE) == 0);
        
        rdata = spi->SPI_RDR;  //dummy read data
        TRACE_INFO("WRITE COMMAND REG[addr/data]=0x%02x\n\r",  data);	      
}



/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu( void )
{
	uint32_t i;

	printf("\n\rMenu :\n\r");
	printf("------\n\r");

	for (i = 0; i < 3; i++) {
		printf("  %u: Set SPCK = %7u Hz\n\r",
               (unsigned int)i, (unsigned int)clockConfigurations[i]);
	}
	printf("  s: Perform SPI transfer start\n\r");
	printf("  d: Perform SPI DMA Transfer (first 30 bytes of Tx buffer)\n\r");
	printf("  h: Display menu \n\r\n\r");
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main (void)  
{
	uint8_t ucKey;
        
        struct _SPI spibox; // from 301  
        uint8_t tmp;        // from 301 
        uint8_t secu = 0xA0|0x01;
       
	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();
        
  
        PIO_Configure(spi_pins, PIO_LISTSIZE(spi_pins));  
	/* Disable watchdog */
	WDT_Disable(WDT);
        /* DBG UART */
        DBG_Configure(dbg_baudrate, BOARD_MCK);
         
	/* Output example information */
	printf("\n\r--- SPI Example %s --\n\r", SOFTPACK_VERSION);
	printf("--- %s\n\r", BOARD_NAME);
	printf("--- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);
        
        Clock_Config();        
     
        /* Display menu */
	//DisplayMenu();
        
 //from301
        
        Init_SPI(SPI0, SPI0_CS3); 
  
        waitKey();
        spibox.addr  = REG_SOFT_NRESET;
        spibox.data = 0x0f;
        
        SENSE_WRITE_16(SPI0, SPI0_CS3, spibox.addr, spibox.data);
       
        spibox.addr  = 0x3e;
        spibox.data  = secu;

        SENSE_WRITE_16(SPI0, SPI0_CS3, spibox.addr, spibox.data);        
        
        spibox.addr = 0x30;
        spibox.data = 0xff;
        
        SENSE_WRITE_16(SPI0, SPI0_CS3, spibox.addr, spibox.data); 
        
    /*
        spibox.addr = REG_ATCFG;
        spibox.addr = REG_ADCV1_TAG;
        spibox.addr = REG_ITOUTCR;
        spibox.addr = VC_ITEM_CHIP_ID;
        spibox.addr = REG_SDI0;
 */
        
      
        
	while (1) {
          
		//ucKey = DBG_GetChar();
                waitKey();
          
        /* access to the sense registers */  // from 301  
        spibox.addr = REG_ADCV3_TAG;
        spibox.data = SENSE_READ_16(SPI0, SPI0_CS3, spibox.addr);  
        spibox.addr = REG_ADCV2_TAG;
        spibox.data = SENSE_READ_16(SPI0, SPI0_CS3, spibox.addr);      
        spibox.addr = REG_ADCV1_TAG;
        spibox.data = SENSE_READ_16(SPI0, SPI0_CS3, spibox.addr);                 
		switch (ucKey) {
		case 'h':
		        DisplayMenu();
			break;
		case 's':
			
			break;

		case 'd':
			
		default:
			/* Set SPI clock configuration #n */
			if ((ucKey >= '0') && (ucKey <= ('0' + 2))) {
				SetClockConfiguration(ucKey - '0');
			}
			break;
		}
	}
}
/** \endcond */
/* ---------------------------------------------------------------------------- */
/*                  Microchip Microcontroller Software Support                  */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Microchip Corporation                                    */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

