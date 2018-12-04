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
#include "spi_sense.h"
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern MAILBOX *mb;    

struct _UNPACK up;     // probably no more used


/*----------------------------------------------------------------------------
 *        new jjk
 *----------------------------------------------------------------------------*/
static void Print_Buffer_hex(int32_t * buf) {
  // print of output buffer
  uint32_t j;
        for (j = 0; j < CIC_NUMBER*BUFFER_NUMBER; j++) {
        printf(" %x \n\r" ,buf[j]);
        }
}

void Print_Buffer(int32_t * buf) {
  // print of output buffer
  uint32_t j;
        for (j = 0; j < CIC_NUMBER*BUFFER_NUMBER; j++) {
        printf(" %d \n\r" ,buf[j]);
        }
}

void Print_Buffer_bin(int32_t * buf) {
  // print of output buffer
  uint32_t j;
  uint32_t i;
  int8_t bs_data;
  
        for (j = 0; j < CIC_NUMBER*BUFFER_NUMBER; j++) {
          for (i = 0; i < 10; i++) { 
                bs_data = ((buf[j] >> i*3) & 0x7u)<<5;
                bs_data = bs_data >>5;
                printf(" %d"R ,bs_data);
                bs_data = 0;
          }
        }
}


void Unpack_word_bs0(uint32_t *in) { 
// for runtime measurement
   mb->to_bs[0] = 0;
   mb->to_bs[0] =  in[mb->count] << 5;
   mb->to_bs[0] =  mb->to_bs[0]  >> 5;
}

void Unpack_word_bs1( uint32_t *in) { 
  
  mb->to_bs[1] = 0;
  mb->to_bs[1] =  (in[mb->count] & 0x1u << 3)  << 2 | (in[mb->count] & 0x3u << 8) >> 2;
  mb->to_bs[1] =  mb->to_bs[1]  >> 5;
}

void Unpack_word_bs2( uint32_t *in) { 
  
  mb->to_bs[2] = 0;
  mb->to_bs[2] =  (in[mb->count] & 0x3u << 10)  >> 10-5 | (in[mb->count] & 0x1u << 16) >> 14-5;
  mb->to_bs[2] =  mb->to_bs[2]  >> 5;
}

void Unpack_word_bs3( uint32_t *in) { 
  
  mb->to_bs[3] = 0;
  mb->to_bs[3] =  (in[mb->count] & 0x7u << 17)  >> 12;
  mb->to_bs[3] =  mb->to_bs[3]  >> 5;
}

void Unpack_word_bs4( uint32_t *in) { 
  mb->to_bs[4] = 0;
  mb->to_bs[4] =  in[mb->count] >> (24-5);
  mb->to_bs[4] =  mb->to_bs[4]  >> 5;
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

static void Reset_buffout(void)
{
 uint32_t i;  
 mb->kos = 0;
 mb->pos = 0; 
  for (i = 0; i < CIC_NUMBER*BUFFER_NUMBER; i++) {
    mb->CIC_C[i] = 0;
    
  }
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

static void bs_to_bin32(void) {
 int8_t bsin;
   // reset the buffer before 
   // pos is the bitstream position in the 32 bits 0 to 9
   // n is the bitstream number
   // kos is the buffer sample index
  
 #ifdef BUFFOUT
bsin =  mb->to_bs[mb->View_bs];

  if (mb->kos < CIC_NUMBER*BUFFER_NUMBER) {
  
               mb->CIC_C[mb->kos] |= (bsin & 0x7u) << mb->pos*3;
               if (mb->pos == 9) {mb->pos=0; mb->kos++;} else mb->pos++;
         
       }
#endif
}

static void BS_to_bin(void) {

 switch (mb->View_bs) {
                                case 0:
                                     Unpack_word_bs0(mb->Pab); 
                                     bs_to_bin32();
                                    break;
                                case 1:
                                     Unpack_word_bs1(mb->Pab); 
                                     bs_to_bin32();
                                      break;
                                case 2:
                                     Unpack_word_bs2(mb->Pab); 
                                     bs_to_bin32();
                                    break;
                                case 3:
                                     Unpack_word_bs3(mb->Pab); 
                                     bs_to_bin32();
                                    break;
                                case 4:
                                     Unpack_word_bs4(mb->Pab); 
                                     bs_to_bin32();
                                    break;
                          } 
}


/** run capture in condition 01*/
void Capture_01(void)
{
  uint32_t buff_count = 0;
  bool     buff_repeat = true;
  uint32_t k;  
  uint32_t j; 
  
  mb->buffer_switch = false;
  if (mb->buffer_switch) mb->Pab = mb->A; else mb->Pab = mb->B;  
          
  printf(I"START polling Fsync"R);    
  Reset();
  Reset_buffout();
    


#ifdef AUTOTEST
      Enable_Capture(); 
      PIO_Generation(); // Fill the first buffer 
#else
      PIO_synchro_polling(); // enable capture at sync detection  
      PIO_DMA_firstbuffer(); // dmacall, buffer switch and reset
  
#endif
  
  
  buff_repeat = true;
  while (buff_repeat) { // switching between buffer

        buff_count++;   
        if (mb->buffer_switch) mb->Pab = mb->A; else mb->Pab = mb->B; 

     // start buffer loop A B ////////////////////////////////////////
        while (mb->repeat) { 
           k++;
#ifdef AUTOTEST
           DSP();
           BS_2_IO();
#endif          
          if (mb->Ena_cic) View(mb->View_bs,k); else BS_to_bin();
          if (mb->count == SAMPLES_NUMBER-1) {mb->repeat = false; mb->count=0;}  else mb->count++;
        }
      // stop buffer loop A B ////////////////////////////////////////
        
        
     // wait for end of DMA callback, next buffer available   
        while(mb->dmacall){ mb->key = local_GetChar();}   
        Reset();
        mb -> buffer_switch = !mb-> buffer_switch;

      // End of buffering
         if ( buff_count == BUFFER_NUMBER) {
                     //Disable_Capture();   
                     buff_repeat = false;
                     buff_count = 0; 
              #ifdef BUFFOUT 
                     if (mb->Ena_cic) Print_Buffer(mb->CIC_C); else Print_Buffer_bin(mb->CIC_C);
                     Average(mb->CIC_C);
                     Stdev(mb->CIC_C);
                     printf(I"STOP  \n\r");
              #endif
         }
         
  }
 
}

void Capture_console_Init(void)
{
  mb->console = false;
  mb->Ena_bs0 = false;
  mb->Ena_bs1 = false; 
  mb->Ena_bs2 = false;
  mb->Ena_bs3 = false;
  mb->Ena_bs4 = false;
  mb->View_bs =  0;
}

void Capture_console_Print(void)
{
    // bits attribution, Change on posedge
    // Sigma delta order MSB to LSB
    // BS4,  BS3,  BS2,  BS1,  BS0    
    // SDV2, SDI2, SDV1, SDI1, SD0  
printf(R); 
printf(I"BS0=SD0=I0, BS1=I1, BS2=V1 BS3=I2, BS4=V2"R);  
if (mb->Ena_bs0 == false) printf(I"BS0/I0 Disabled\r\n"); else printf(I"BS0/I0 Enabled\r\n");
if (mb->Ena_bs1 == false) printf(I"BS1/I1 Disabled\r\n"); else printf(I"BS1/I1 Enabled\r\n");
if (mb->Ena_bs2 == false) printf(I"BS2/V1 Disabled\r\n"); else printf(I"BS2/V1 Enabled\r\n");
if (mb->Ena_bs3 == false) printf(I"BS3/I2 Disabled\r\n"); else printf(I"BS3/I2 Enabled\r\n");
if (mb->Ena_bs4 == false) printf(I"BS4/V2 Disabled\r\n"); else printf(I"BS4/V2 Enabled\r\n");
printf(I"View BS %d"R, mb->View_bs);
}

static void cic_compute(uint32_t n, uint32_t k) {
   uint32_t j;
   // n is the bitstream number
   // k is the bitstream sample index
   // j decimated sample index
   
       if (CIC(n, mb->to_bs[n])) { 
         j=k/CICOSR-1;
         #ifdef BUFFOUT 
                mb->CIC_C[j] = mb->CIC[n];
         #endif
       }
}






void View(uint32_t n, uint32_t k) {

 switch (n) {
                                case 0:
                                     Unpack_word_bs0(mb->Pab); 
                                     cic_compute(n,k);
                                    break;
                                case 1:
                                     Unpack_word_bs1(mb->Pab); 
                                     cic_compute(n,k);
                                      break;
                                case 2:
                                     Unpack_word_bs2(mb->Pab); 
                                     cic_compute(n,k);
                                    break;
                                case 3:
                                     Unpack_word_bs3(mb->Pab); 
                                     cic_compute(n,k);
                                    break;
                                case 4:
                                     Unpack_word_bs4(mb->Pab); 
                                     cic_compute(n,k);
                                    break;
                          } 
}


static void Switch_Enable(void) {
 switch (mb->key) {
                                case '0':
                                    mb->Ena_bs0 = true;     printf(R"Enable BS 0"R);
                                    mb->prekey =' '; 
                                    mb->key =' ';
                                    break;         
                                case '1': 
                                    mb->Ena_bs1 = true;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Enable BS 1"R);
                                    break;
                                case '2':
                                    mb->Ena_bs2 = true;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Enable BS 2"R); 
                                    break;
                                case '3':
                                    mb->Ena_bs3 = true;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Enable BS 3"R);
                                    break;
                                case '4':
                                    mb->Ena_bs4 = true;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Enable BS 4"R);
                                    break;
                          }

}

static void Switch_View(void) {
 switch (mb->key) {
                                case '0':
                                    mb->View_bs = 0;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"View BS 0"R);
                                    break;
                                case '1':
                                    mb->View_bs = 1;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"View BS 1"R);
                                    break;
                                case '2':
                                    mb->View_bs = 2;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"View BS 2"R); 
                                    break;
                                case '3':
                                    mb->View_bs = 3;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"View BS 3"R); 
                                    break;
                                case '4':
                                    mb->View_bs = 4;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"View BS 4"R); 
                                    break;
                          } 

}

static void Switch_Disable(void) {
 switch (mb->key) {
                                case '0':
                                    mb->Ena_bs0 = false;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Disable BS 0"R);
                                    break;
                                case '1':
                                    mb->Ena_bs1 = false;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Disable BS 1"R);
                                    break;
                                case '2':
                                    mb->Ena_bs2 = false;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Disable BS 2"R); 
                                    break;
                                case '3':
                                    mb->Ena_bs3 = false;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Disable BS 3"R); 
                                    break;
                                case '4':
                                    mb->Ena_bs4 = false;
                                    mb->key =' ';
                                    mb->prekey =' ';        printf(R"Disable BS 4"R); 
                                    break;
                          } 

}

void Capture_console(void)
{
  // ² enter Console
  // s stop console
  // e enable ADC + bs number
  // d disable ADC  + bs number
  // v view bitstream  + bs number
  // i info
  // c to activate the CIC filter
  // b to see the bitstream
  
  
 
  
  if (mb->key == '²') {
              printf(I"Start Console"R);
              mb->key == ' ';
              mb->console =  true;
              while(mb->console) {
                    mb->key = DBG_GetChar();
                    DBG_PutChar(mb->key);
                    
                    switch (mb->key) {
                    case 's': 
                        mb->console = false; printf(R);
                        mb->key =' ';
                        break;
                    case 'e':
                        mb->prekey ='e';
                        mb->key =' ';
                        break;
                    case 'd':
                        mb->prekey ='d'; 
                        mb->key =' ';
                        break;
                    case 'v':
                        mb->prekey ='v'; 
                        mb->key =' ';
                        break;
                    case 'i':
                        Capture_console_Print();
                        mb->prekey =' '; 
                        mb->key =' ';
                        break;                        
                    case 'c':
                        mb->Ena_cic = true; printf("Enable CIC filter osr=64"R);
                        mb->prekey =' '; 
                        mb->key =' ';
                        break;  
                    case 'b':
                        mb->Ena_cic = false; printf("Disable CIC and print bitstream "R);
                        mb->prekey =' '; 
                        mb->key =' ';
                        break;       
                    }
                    
                    
                    switch (mb->prekey) { 
                     case 'e':
                         Switch_Enable();
                         break;
                     case 'd':
                         Switch_Disable();
                         break;
                     case 'v':
                         Switch_View();
                         break;
                    }
              }
         printf(I"Stop Console"R);
         Set_Channels();
  }
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
