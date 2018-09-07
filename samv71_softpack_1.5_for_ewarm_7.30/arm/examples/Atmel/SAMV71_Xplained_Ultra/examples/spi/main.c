/* ---------------------------------------------------------------------------- */
/*                  Microchip Microcontroller Software Support                  */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
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

/**
 *  \page spi SPI example
 *
 *  \section Purpose
 *
 * This example shows control of the SPI in loop back mode.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear (values depend
 *  on the board and chip used):
 *     \code
 *      -- SPI Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Menu :
 *      ------
 *      0: Set SPCK =  500000 Hz
 *      1: Set SPCK = 1000000 Hz
 *      2: Set SPCK = 5000000 Hz
 *      s: Perform SPI transfer start
 *      d: Perform SPI Dma Transfer
 *      h: Display menu
 *     \endcode
 *
 * The user can then choose any of the available options to perform
 * the described action.
 *
 *  \section References
 *  - spi/main.c
 *  - pio.h
 *  - pio_it.h
 *  - board.h
 */

/** \file
 *
 *  This file contains all the specific code for the SPI example.
 *
 */

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

struct _Mailbox { // from 301

    /** Command send to the monitor to be executed. */
    uint32_t command;
    /** Returned status, updated at the end of the monitor execution.*/
    uint32_t status;

    /** Input Arguments*/ 
    uint32_t cmd_addr;    // registers address or VC button ID
    uint32_t cmd_value;
    uint32_t spi_cs;
};

/** Global timestamps in milliseconds since start of application */
volatile uint32_t dwTimeStamp = 0;

/** SPI Clock setting (Hz) */
static uint32_t spiClock = 500000;
static uint32_t dbg_baudrate = 115200;

/** Global DMA driver for all transfer */
static Spid    SpiDma;
static SpidCmd SpiCommand;
static sXdmad  Dma;

#define SPI0_CS0  0
#define SPI0_CS1  1
#define SPI0_CS2  2
#define SPI0_CS3  3


// from 301
unsigned int  mask_spics;
unsigned char reg_value;
unsigned char item_status=0;


/** SPI clock configuration */
static const uint32_t clockConfigurations[3] = { 500000, 1000000, 5000000};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
COMPILER_ALIGNED(32) uint8_t pTxBuffer[] = "This is SPI LoopBack Test Buffer";
COMPILER_ALIGNED(32) uint8_t pRxBuffer[30];

static void Init_SPI(void) 
{// from 301    
    /* Enable the SPI clock*/
    PMC_EnablePeripheral(ID_SPI0) ;

    /* Configure SPI in Master Mode with CS selected !!! */
    SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS(1));

    SPI_ConfigureNPCS(SPI0, 1, SPI_SCBR( 8000000, BOARD_MCK));
    
    /* Enable the SPI */
    SPI_Enable(SPI0) ;    
}

static void SENSE_SPI_CS_SELECT(void) 
{// from 301 
	  REG_PIOA_CODR = mask_spics;
}

static void SENSE_SPI_CS_DESELECT(void) 
{// from 301
	  //PIN_SPI_NPCS3_PA22,PIN_SPI_NPCS2_PA30,PIN_SPI_NPCS1_PA31,PIN_SPI_NPCS0_PA11
	  //(0x1<<22)|(0x1<<30)|(0x1<<31)|(0x1<<11)
	  REG_PIOA_SODR = (0x1<<22)|(0x1<<30)|(0x1<<31)|(0x1<<11);
}

static void SENSE_SPI_READ(unsigned char addr,volatile unsigned char*pData,unsigned char size)
{// from 301
	  unsigned char i;
	  
	  SENSE_SPI_CS_SELECT();
	  
	  SPI_Write(SPI0, 1, (addr|0x80));
	  for(i=0;i<size;i++){
	      SPI_Write(SPI0, 1, 0xFF);
	      pData[i]=SPI_Read(SPI0);
	  }
	      
	  SENSE_SPI_CS_DESELECT(); 	
}

static void SENSE_SPI_WRITE(unsigned char addr,volatile unsigned char*pData,unsigned char size)
{// from 301
	  unsigned char i;
	  
	  SENSE_SPI_CS_SELECT();
	  
	  SPI_Write(SPI0, 1, (addr&0x7F));
	  for(i=0;i<size;i++)
	      SPI_Write(SPI0, 1, pData[i]);
	      
	  SENSE_SPI_CS_DESELECT();    
}

static void waitKey(void)
{
	printf("-I- Press any key to Continue...\n\r");
	while (1) {
		if (DBG_GetChar()!= 0)
			break;
	}
}


/**
 *  \brief Handler for SPI0.
 *
 *  Process SPI interrupts
 */
void SPI0_Handler(void)
{
	printf("%c", (char) SPI_Read(SPI0));
}

/**
 *  \brief Handler for XDMAC.
 *
 *  Process XDAMC interrupts
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&Dma);
}

/**
 * \brief Sets the specified SPI clock configuration.
 * \param configuration  Index of the configuration to set.
 */
static void SetClockConfiguration(uint8_t configuration)
{
	spiClock = clockConfigurations[configuration];
	printf("Setting SPI master clock #%u ... \n\r",
			(unsigned int)clockConfigurations[configuration]);
}

/**
 * \brief Perform SPI transfer with interrupt in SPI loop back mode.
 */
static void SpiLoopBack(void)
{
	uint8_t i;

	printf( "\n\r-I- Configure SPI master\n\r" );
	SPI_Configure(SPI0, ID_SPI0, (SPI_MR_MSTR | SPI_MR_MODFDIS
					| SPI_MR_LLB | SPI_PCS( SPI0_CS3 )));
	SPI_ConfigureNPCS( SPI0,
			SPI0_CS3,
			SPI_DLYBCT( 1000, BOARD_MCK ) |
			SPI_DLYBS(1000, BOARD_MCK) |
			SPI_SCBR( spiClock, BOARD_MCK) );

	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn ,1);
	NVIC_EnableIRQ(SPI0_IRQn);

	SPI_EnableIt(SPI0, SPI_IER_RDRF);
	SPI_Enable(SPI0);

	for (i = 0; ;i++) {
		SPI_Write(SPI0, SPI0_CS3 , (uint16_t)pTxBuffer[i]);
		if (pTxBuffer[i] =='\0')
			break;
	}
	if (SPI_IsFinished(SPI0)) {
		SPI_Disable(SPI0);
	}
}

/**
 * \brief Perform SPI transfer with DMA in SPI loop back mode.
 */
static void SpiLoopBackDma(void)
{
	printf( "\n\r-I- Configure SPI master\n\r" );
	Dma.pXdmacs = XDMAC;

	SpiCommand.TxSize = 30;
	SpiCommand.pTxBuff = (uint8_t *)pTxBuffer;
	SpiCommand.RxSize= 30;
	SpiCommand.pRxBuff = (uint8_t *)pRxBuffer;
	SpiCommand.spiCs = SPI0_CS3;


	SPID_Configure(&SpiDma, SPI0, ID_SPI0, (SPI_MR_MSTR | SPI_MR_MODFDIS
					| SPI_MR_LLB | SPI_PCS( SPI0_CS3 )), &Dma);
	SPI_ConfigureNPCS(SPI0,
					SPI0_CS3,
					SPI_DLYBCT( 1000, BOARD_MCK ) |
					SPI_DLYBS(1000, BOARD_MCK) |
					SPI_SCBR( spiClock, BOARD_MCK));

	SPI_Enable(SPI0);
	SPID_SendCommand(&SpiDma, &SpiCommand);
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
extern int main (int argc, char **argv) // from 301  
{
	uint8_t ucKey;
        
        struct _Mailbox *pMailbox = (struct _Mailbox *) argv; // from 301  
        unsigned char tmp; // from 301 
       
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

        /* access to the sense registers */  // from 301  
       //  pMailbox->cmd_addr = VC_ITEM_CHIP_ID;
       // Init_SPI();
       // SENSE_SPI_READ(pMailbox->cmd_addr,&tmp,1);       
       // pMailbox->cmd_value=tmp;
       // pMailbox->status = APPLET_SUCCESS;   
        
        TRACE_INFO("COMMAND READ REG[0x%x]=0x%x\n\r",
              (unsigned int)pMailbox->cmd_addr,
              (unsigned int)pMailbox->cmd_value);
           
     
        /* Display menu */
	DisplayMenu();

	while (1) {
          
		ucKey = DBG_GetChar();
                SpiLoopBack();
                
		switch (ucKey) {
		case 'h':
			DisplayMenu();
			break;
		case 's':
			SpiLoopBack();
			break;

		case 'd':
			SpiLoopBackDma();
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
