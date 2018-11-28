/* jj kazazaian 2018*/
// PIOA parallel capture with DMA
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "capture.h"
#include "pc_dma.h"
#include "dsp_sense.h"
#include "clock.h"
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern MAILBOX *mb;    

struct _UNPACK up;  // probably no more used

/*----------------------------------------------------------------------------
 *        new jjk
 *----------------------------------------------------------------------------*/
void Print_Buffer(int32_t * buf) {
  // print of output buffer
  uint32_t j;
        for (j = 0; j < CIC_NUMBER*BUFFER_NUMBER; j++) {
        printf(" %d \n\r" ,buf[j]);
        }
}

void Unpack_64b_bs0(uint32_t *in) { 
// for runtime measurement
 
  mb->to_bs[0] = 0;
  mb->to_bs[0] =  
                  (in[mb->count]  & B2_msk) << 5
                | (in[mb->count]  & B2_msk) << 4
                | (in[mb->count]  & B2_msk) << 3
                | (in[mb->count]  & B2_msk) << 2
                | (in[mb->count]  & B2_msk) << 1
                | (in[mb->count]  & B2_msk)  
                | (in[mb->count]  & B1_msk)  
                | (in[mb->count]  & B0_msk);
  /*
printf("  in= %x  B2=%x B1=%x B0=%x  bs=%d \r\n"
      ,  in[mb->count]
      ,  in[mb->count]  & B2_msk 
      ,  in[mb->count]  & B1_msk  
      ,  in[mb->count]  & B0_msk
      ,  mb->to_bs[0]
);
*/

}
void Enable_Capture(void) {
 /* enable pio capture*/
    PIOA->PIO_PCMR |= PIO_PCMR_PCEN;
 
}

void Disable_Capture(void) {
    /* disable pio capture*/
    PIOA->PIO_PCMR &= ~((uint32_t)PIO_PCMR_PCEN);
}

void Reset(void) {
        mb->count = 0;        // counting of sample Numbers 
        mb->dmacall = true;   // DMA interupt rise when false
        mb->repeat  = true;   // repeat the io and dma loop
}

 void PIO_Generation(void) {
     // first acquisition before looping
	while (mb->repeat) { 
     // PIO signal generation
        DSP();
        BS_2_IO(); 
 
            // printf(" %d   %x     ", mb->buffer_switch, mb->Pab); 
            // printf(" %d     ", mb->to_bs[0]);
            // printf(" %d \n\r" ,mb->BS0);
           
         // Next_action();     // State Machine next action to do  
         
        if (mb->count == SAMPLES_NUMBER-1) {mb->repeat = false; mb->count=0;}  else mb->count++;
        }
        // wait for end of DMA callback, next buffer available   
        while(mb->dmacall); 
        mb -> buffer_switch = !mb-> buffer_switch;
        Reset(); 
}

 void PIO_DMA_firstbuffer(void) { 
     // first acquisition before looping
     // wait for end of DMA callback, next buffer available   
        while(mb->dmacall); 
        mb -> buffer_switch = !mb-> buffer_switch;
        Reset(); 
}


void PIO_synchro_polling(void) {
  uint32_t j;
  mb->synchro = false;
      while(!mb->synchro) { 
            mb->sync = IO_get_sync();
            if (mb->presync==false && mb->sync==true) {
                 Enable_Capture();
                 mb->synchro=true;
            }
            mb->presync = mb->sync;
      }         
}


void PIO_Capture_DMA(void) {
/* initialize PIO DMA mode and buffer pointer*/ 
  DMA_Buffer_cfg(mb->A, mb->B, ND);
  DMA_PIO_cfg();
  DMA_Start();
  DMA_Enable();
}

/** initialize PIO parallel capture function*/
void Capture_Config(Pio *pio)
{
    /* enable periphral clock*/
    PMC_EnablePeripheral(ID_PIOA);

    /* disable pio capture*/
    pio->PIO_PCMR &= ~((uint32_t)PIO_PCMR_PCEN);

      /* interrupt*/
        PIOA->PIO_IER |= PIO_IER_P10; // synchro pin
        pio->PIO_PCIER |= PIO_PCIER_DRDY;
    //  pio->PIO_PCIDR |= PIO_PCIDR_ALL;
  
    /* 8bit width*/
    pio->PIO_PCMR |= PIO_PCMR_DSIZE_BYTE;

    /* always sampling on clock*/
    pio->PIO_PCMR |= PIO_PCMR_ALWYS;
    
    Enable_Capture(); 
}
/*----------------------------------------------------------------------------*/

    
    
    
    
























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
/*
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
     
}
*/
/* \brief Callback function for parallele capture interrupt*/
static void _pc_Callback(int dummy, void* pArg)
{
	dummy = dummy;
	pArg = pArg;
	//Processing the DMA buffer //////////////////////////////////////
        mb-> dmacall =  false;
	}

/*\brief Configure parallele capture DMA and start DMA transfer.*/
void _pc_dmaTransfer(uint32_t *Pbuffer)
{       
        PcCommand.RxSize   = ND;     
	PcCommand.pRxBuff  = Pbuffer;
	PcCommand.callback = (PcCallback)_pc_Callback;
        Pc_ConfigureDma(&Pc ,PIOA ,ID_PIOA, &dmad);
	Pc_SendData(&Pc, &PcCommand);
}

bool Unpack(uint32_t *in) { 
//
 
  uint32_t j;
  uint8_t data[4];
  
        
        for (j = 0; j < 4; j++) {
              data[j] = (uint8_t)((in[mb->count] >> 8*j) & 0x1F);  // byte data extraction from 32 bits
              up.sync = (uint8_t)((data[j]  >> 4)   & 0x1);        // synchro bit extraction
              // up.csum = up.csum + up.sync;  // cumulate sync to count up to sample number 
              if (up.synchronized && (up.sync == 1) && (up.kase != 3-j)) {up.kase = 4; up.synchronized = false;}// check for synchronization error
              if (up.sync == 1) {up.kase = 3-j; up.synchronized = true;}   // case detection 0:0001 1:0010 2:0100 3:1000   
        }
    
        
          
        switch(up.kase) {
        // Reorder data to fit the synchronization

                  case 0:
                  // 0001, case synchrone
                    mb->to_demux[0] = data[0];
                    mb->to_demux[1] = data[1];
                    mb->to_demux[2] = data[2];
                    mb->to_demux[3] = data[3];
                  break;

                 case 1:
                  // 0010, case one shift left 
                    mb->to_demux[0] = up.predata[3];
                    mb->to_demux[1] = data[0];
                    mb->to_demux[2] = data[1];
                    mb->to_demux[3] = data[2];      
                  break;
                  
                 case 2:
                  // 0100, case one shift left 
                    mb->to_demux[0] = up.predata[2];
                    mb->to_demux[1] = up.predata[3]; 
                    mb->to_demux[2] = data[0];
                    mb->to_demux[3] = data[1];  
                  break;
                  
                 case 3:
                  // 1000, case one shift left 
                    mb->to_demux[0] = up.predata[1]; 
                    mb->to_demux[1] = up.predata[2]; 
                    mb->to_demux[2] = up.predata[3]; 
                    mb->to_demux[3] = data[0]; 
                  break;
                  
                 default:
                  printf("---ERROR Synchro lost \n\r");
       }
      
       
      
     
       demxcode();
          
       up.predata[0] = data[0];
       up.predata[1] = data[1];
       up.predata[2] = data[2];
       up.predata[3] = data[3];
 
       if (up.i == SAMPLES_NUMBER-1) up.i=0;  else up.i++;
     
       return up.status;
       
}

void Unpack_bs0(uint32_t *in) { 
// for runtime measurement
 
  uint32_t j;
  uint8_t data[4];
  
        
        for (j = 0; j < 4; j++) {
              data[j] = (uint8_t)((in[mb->count] >> 8*j) & 0x1F);  // byte data extraction from 32 bits   
        }

                    mb->to_demux[0] = data[0];

       demxcode_bs0();
       
}



/*
void PIO_Capture_DMA(bool switch_buffer) {
  
  
 /// if (switch_buffer) _pc_dmaTransfer(mb->B); else _pc_dmaTransfer(mb->A);

  
}
*/







/*    
 data is stored in PIO_PCRHR and the flag DRDY is set to one in PIO_PCISR.
 If the bit ALWYS is set to one, the Parallel Capture mode samples
 the sensor data at the rising edge of the sensor clock whichever the data enable signals are.
The size of the data which can be read in PIO_PCRHR can be programmed using the DSIZE field in PIO_PCMR.
If this data size is larger than 8 bits, then the Parallel Capture mode samples several sensor data to form a
concatenated data of size defined by DSIZE. Then this data is stored in PIO_PCRHR and the flag DRDY is set to
one in PIO_PCISR.
*/
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
