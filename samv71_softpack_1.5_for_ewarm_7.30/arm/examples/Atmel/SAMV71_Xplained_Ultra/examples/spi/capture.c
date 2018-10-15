/* jj kazazaian 2018*/
// PIOA parallel capture with DMA
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "capture.h"
#include "pc_dma.h"

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern struct _MAILBOX mb;    

/** PIO receive buffer. */
uint32_t pio_rx_buffer[SIZE_BUFF_RECEPT];


// to remove later when DMA works
bool pio_rx_buffer_error[SIZE_BUFF_RECEPT];
uint32_t count;

// DMA

/** Global DMA driver for all transfer */
static sXdmad dmad;

/** Global parallel DMA instance */
static PcDma Pc;

/** parallel capture command instance */
static PcCmd PcCommand;


/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/

/* \brief xDMA interrupt handler.*/
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/* \brief Callback function for parallele capture interrupt*/
static void _pc_Callback(int dummy, void* pArg)
{
	dummy = dummy;
	pArg = pArg;
	//Processing the DMA buffer //////////////////////////////////////
        PIO_Print_Buffer(); 
	}

/*\brief Configure parallele capture DMA and start DMA transfer.*/
void _pc_dmaTransfer(void)
{
	PcCommand.RxSize   = SIZE_BUFF_RECEPT;
	PcCommand.pRxBuff  = pio_rx_buffer;
	PcCommand.callback = (PcCallback)_pc_Callback;
        Pc_ConfigureDma(&Pc ,PIOA ,ID_PIOA,&dmad);
	Pc_SendData(&Pc, &PcCommand);
}
/**
 * \brief Read from Capture Reception Holding Register.
 * \note Data presence should be tested before any read attempt.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param pul_data Pointer to store the data.
 *
 * \retval 0 Success.
 * \retval 1 I/O Failure, Capture data is not ready.
 */
static uint32_t pio_capture_read(const Pio *p_pio, uint32_t *pul_data)
{/*
	// Check if the data is ready 
	if ((p_pio->PIO_PCISR & PIO_PCISR_DRDY)== 0) {
		return 1;
	}
*/
	/* Read data */
	*pul_data = p_pio->PIO_PCRHR;
	return p_pio->PIO_PCISR & PIO_PCISR_DRDY; //clear the status
}


static void PIO_Capture_Buffer(const Pio *p_pio)
{
      uint32_t data;

      pio_rx_buffer_error[count] = pio_capture_read(p_pio, &data); 
       
      
      pio_rx_buffer[count] = data & 0x1F; 
      if (count > SIZE_BUFF_RECEPT) count=0; else count++;
     


      
}

void PIO_Print_Buffer(void) {
      uint32_t i;
      printf("\n\r");
       for (i = 0; i < SIZE_BUFF_RECEPT; i++) {
              printf("   Data [%02d] = " , i);  
              Print_int8_to_bin((uint8_t)(pio_rx_buffer[i]& 0x1F));
              printf("\n\r");
       }

}

void PIO_Reset_Buffer(void) {
      uint32_t i;
      printf("\n\r");
       for (i = 0; i < SIZE_BUFF_RECEPT; i++) {
         pio_rx_buffer[i] = 0;
       }

}


void PIO_Capture(void) {
  PIO_Capture_Buffer(PIOA);
 
}

void Enable_Capture(void) {
 /* enable pio capture*/
    PIOA->PIO_PCMR |= PIO_PCMR_PCEN;
 
}

void Disable_Capture(void) {
    /* disable pio capture*/
    PIOA->PIO_PCMR &= ~((uint32_t)PIO_PCMR_PCEN);
}




/** initialize PIO parallel capture function*/
void Capture_Config(Pio *pio)
{
    /* enable periphral clock*/
    PMC_EnablePeripheral(ID_PIOA);

    /* disable pio capture*/
    pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_PCEN);

      /* enable rxbuff interrupt*/
    pio->PIO_PCIER |= PIO_PCIER_RXBUFF;

    /* 8bit width*/
    pio->PIO_PCMR |= PIO_PCMR_DSIZE_BYTE;

    /* always sampling on clock*/
    pio->PIO_PCMR |= PIO_PCMR_ALWYS;
    
    /* initialize PIO DMA mode*/ 
    _pc_dmaTransfer();
    

/*    
 data is stored in PIO_PCRHR and the flag DRDY is set to one in PIO_PCISR.
 If the bit ALWYS is set to one, the Parallel Capture mode samples
 the sensor data at the rising edge of the sensor clock whichever the data enable signals are.
The size of the data which can be read in PIO_PCRHR can be programmed using the DSIZE field in PIO_PCMR.
If this data size is larger than 8 bits, then the Parallel Capture mode samples several sensor data to form a
concatenated data of size defined by DSIZE. Then this data is stored in PIO_PCRHR and the flag DRDY is set to
one in PIO_PCISR.
*/
}



/** \endcond */
/* ---------------------------------------------------------------------------- */
/*                  Microchip Microcontroller Software Support                  */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2018, Microchip Corporation                                    */
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
