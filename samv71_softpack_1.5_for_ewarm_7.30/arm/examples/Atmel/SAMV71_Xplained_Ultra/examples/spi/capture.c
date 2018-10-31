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
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern MAILBOX *mb;    

struct _UNPACK up;


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
        //PIO_Copy_Buffer(mb->A, mb->B);
        PIO_Capture_DMA(mb->dmaswitch);  //0 for A, 1 for B
        mb->dmaswitch = !mb->dmaswitch;
        mb->dmacall =  false;
        
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


void PIO_Copy_Buffer(uint32_t *in, uint32_t *out ) {
      uint32_t i;
       for (i = 0; i < SAMPLES_NUMBER; i++) {
              out[i] = in[i];   
       }
}

void PIO_Print_Buffer(uint32_t *in) {
      uint32_t i, j, k;
      uint8_t data;
      printf("\n\r");
       for (i = 0; i < SAMPLES_NUMBER; i++) {
         for (j = 0; j < 4; j++) {
           
              printf("   Data [%02d] = " , k);  
              data = (uint8_t)((in[i] >> 8*j) & 0x1F);
              Print_int8_to_bin(data);
              printf("\n\r");
              k++;
         }
       }
}

bool Capture_Unpack(uint32_t *in) { 
//
  
  uint32_t j;
  uint8_t data[4];
  
        for (j = 0; j < 4; j++) {
              data[j] = (uint8_t)((in[up.i] >> 8*j) & 0x1F);  // byte data extraction from 32 bits
              up.sync = (uint8_t)((data[j]  >> 4)   & 0x1);   // synchro bit extraction
              up.csum = up.csum + up.sync;  // cumulate sync to count up to sample number 
              if (up.synchronized && (up.sync == 1) && (up.kase != j)) {up.kase = 4; up.synchronized = false;}// check for synchronization error
              if (up.sync == 1) {up.kase = j; up.synchronized = true;}   // case detection 0:0001 1:0010 2:0100 3:1000   
        }
        
        switch(up.kase) {
        // Reorder data to fit the synchronization

                  case 0:
                  // 0001, case synchrone
                    mb->to_demux[0] = (data[0] & 0xF);
                    mb->to_demux[1] = (data[1] & 0xF);
                    mb->to_demux[2] = (data[2] & 0xF);
                    mb->to_demux[3] = (data[3] & 0xF);
                  break;

                 case 1:
                  // 0010, case one shift left 
                    mb->to_demux[0] = (data[1] & 0xF);
                    mb->to_demux[1] = (data[2] & 0xF);
                    mb->to_demux[2] = (data[3] & 0xF);
                    mb->to_demux[3] = (up.predata[0] & 0xF);        
                  break;
                  
                 case 2:
                  // 0100, case one shift left 
                    mb->to_demux[0] = (data[2] & 0xF);
                    mb->to_demux[1] = (data[3] & 0xF);
                    mb->to_demux[2] = (up.predata[0] & 0xF); 
                    mb->to_demux[3] = (up.predata[1] & 0xF);  
                  break;
                  
                 case 3:
                  // 1000, case one shift left 
                    mb->to_demux[0] = (data[3] & 0xF);
                    mb->to_demux[1] = (up.predata[0] & 0xF); 
                    mb->to_demux[2] = (up.predata[1] & 0xF); 
                    mb->to_demux[3] = (up.predata[2] & 0xF);  
                  break;
                  
                 default:
                  printf("---ERROR Synchro lost \n\r");
       }
        
       demxcode();

       up.predata[0] = data[0];
       up.predata[1] = data[1];
       up.predata[2] = data[2];
       up.predata[3] = data[3];
       
       if (CIC(0, mb->demux_to_bs[0])) {       
         printf(" %d \n\r" ,mb->CIC0);   
         //mb->CIC2_out[up.k] = mb->CIC2;
         if (up.k== CIC_NUMBER*BUFFER_NUMBER) up.k=0;  else up.k++;
       }
       
       
       if (up.i == SAMPLES_NUMBER-1) up.i=0;  else up.i++;
   
         
       return up.status;
}



bool PIO_Unpack_Buffer( uint32_t *in) {
      uint32_t i, j, k;
      uint32_t n = 0;     // count the frame start after sync
      uint32_t csum = 0;  //Check sum
      uint8_t data;
      uint8_t sync = 0;
      uint8_t  dmx[4];
      bool synchronized = false;
      bool dmx_full     = false;
      bool status       = false; //no error 

       for (i = 0; i < SAMPLES_NUMBER; i++) {
         for (j = 0; j < 4; j++) {
   
              data = (uint8_t)((in[i] >> 8*j) & 0x1F);  // byte data extraction from 32 bits
              sync = (uint8_t)((data  >> 4)   & 0x1);   // synchro bit extraction
                        
              csum = csum + sync;

              if (synchronized) {
              dmx[n] = (uint8_t)(data & 0xF);
              n++;
                if (n == 4) {n=0; dmx_full = true;}
              }
              
              if (sync==1) synchronized = true;  
              
              //printf("   sync [%02d] = %02d \n\r" ,k, sync);
              
              if (dmx_full) {
              //printf("   D0= %02d   D1= %02d   D2= %02d   D3= %02d   \n\r" ,dmx[0],dmx[1],dmx[2],dmx[3]);
              
              mb->to_demux[0] = dmx[0];
              mb->to_demux[1] = dmx[1];
              mb->to_demux[2] = dmx[2];
              mb->to_demux[3] = dmx[3];
              
              demxcode();
              
      
                if (CIC(0, mb->demux_to_bs[0])) printf(" %d \n\r" ,mb->CIC0); 
/*
              mb.BS0rx[i] = mb.demux_to_bs[0];
              mb.BS1rx[i] = mb.demux_to_bs[1];
              mb.BS2rx[i] = mb.demux_to_bs[2];
              mb.BS3rx[i] = mb.demux_to_bs[3];
              mb.BS4rx[i] = mb.demux_to_bs[4];
            */  
              
              
              
             // printf("   B0  [%02d]  = %02d   B1= %02d   B2= %02d   B3= %02d    B4= %02d \n\r" ,i,mb.BS0rx[i],mb.BS1rx[i],mb.BS2rx[i],mb.BS3rx[i],mb.BS4rx[i]);
              dmx_full = false;
              } else { 
             // padding with zeros
             // mb.BS0rx[i] = 0;
             // mb.BS1rx[i] = 0;
             // mb.BS2rx[i] = 0;
             // mb.BS3rx[i] = 0;
             // mb.BS4rx[i] = 0;

              }
              
                 
              k++;
         }
       }
       printf("\n\r");
       if (csum!=SAMPLES_NUMBER) {
           printf("   Check sum  = %d   ERROR \n\r" , csum); 
           status = true;
       }       
       else printf("   Check sum Ok = %d \n\r" , csum); 
       return status;
}

void PIO_Clear_Buffer(uint32_t *in) {
      uint32_t i;
       for (i = 0; i < SAMPLES_NUMBER; i++) {
         in[i] = 0;
       }
}


void PIO_Capture_DMA(bool switch_buffer) {
    /* initialize PIO DMA mode*/ 
  
  if (switch_buffer) _pc_dmaTransfer(mb->B); else _pc_dmaTransfer(mb->A);
 
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

      /* disable all interrupt*/
    //pio->PIO_PCIER |= PIO_PCIER_DRDY;
      pio->PIO_PCIDR |= PIO_PCIDR_ALL;
  
    /* 8bit width*/
    pio->PIO_PCMR |= PIO_PCMR_DSIZE_BYTE;

    /* always sampling on clock*/
    pio->PIO_PCMR |= PIO_PCMR_ALWYS;
    
    
    Enable_Capture(); 

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
