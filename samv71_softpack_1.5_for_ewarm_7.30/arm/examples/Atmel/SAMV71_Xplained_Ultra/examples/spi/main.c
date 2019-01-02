// jj kazazian 2018//
// Xult board R225 and wolfson audio chip removed
// activate the speed optimization in the IAR compiler
// jjk set to Dcache 0 for debug in samv71q21.h
// #define __DCACHE_PRESENT 0   /**< SAMV71Q21 does provide a Data Cache   */ 
// disable: /* Select XTAL 32k instead of internal slow RC 32k for slow clock */
// in file system_samv71.c, slowclock set to internal RC32k
// change __SAMV71Q21__ to __SAMV71N21__ in IAR preprocessing and option
// change BOARD_SAMV71_XULT to BOARD_SAMV71_DVB in IAR preprocessing and option
// new definitions NO_PIOC, NO_PIOE, NO_TC1
// add _SAMV71_TC1_INSTANCE_ in samv71q21.h for Q21
// add _SAMV71_PIOC_INSTANCE_ in samv71q21.h for Q21


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
/*----------------------------------------------------------------------------
 *        modules Headers (clock,...)
 *----------------------------------------------------------------------------*/
#include "clock.h"
#include "spi_sense.h"
#include "dsp_sense.h"
#include "mux.h"
#include "capture.h"
#include "pc_dma.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
// see main_config.h  

MAILBOX *mb = (MAILBOX *)0x20000100; // data placed in DTCM

uint32_t doit;
uint32_t k;  

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
// See main_config.c



/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

 extern int main (void)  
{ 
        uint32_t buff_nb_count = 0;
        bool mainloop_repeat = true;
        uint32_t j;
        
 
     // mb = (MAILBOX *)malloc( sizeof(MAILBOX));  no used, changed to DTCM     
     // IO_ctrl(6,1);  
     // IO_ctrl(6,0);  
        Main_Config();
        Clock_Config();        
        Sense_Config();
        DSP_Config();
        Init_state();
        Capture_Config(PIOA);
        Disable_Capture(); // Enable_Capture();  when the sync is detected
        Capture_console_Init();
        Set_Channels();        

        PIO_Capture_DMA(); // DMA configuration
        Sense_Dump_param(); // SPI com and read registers
        mb->synchro = false;
        
    #if defined(BOARD_SAMV71_XULT)
	printf(I"board_v71_xult"R);
    #endif
        
        /*
        while(1) {
          
        IO_ctrl(0,1);  
         for (j = 0; j < 16; j++) __asm("nop");
        IO_ctrl(0,0);  
         for (j = 0; j < 16; j++) __asm("nop");
        
        }
        */
Console: 
        IO_ctrl(0,0);  
        waitKey(); // space bar  to continue or '²' for console
        Capture_console(); 
        //Capture_301();
        //Capture_01(); 
        Capture_02();  
goto Console;

}


       /*
        IO_ctrl(6,0);  
        mb->buffer_switch = false;
        if (mb->buffer_switch) mb->Pab = mb->A; else mb->Pab = mb->B;  
          
        printf(I"START polling Fsync  \n\r");
        
        Reset();
        */


/*
        PIO_synchro_polling_onrise(); // enable capture at sync detection     
         
        
       // PIO_Generation(); // Fill the first buffer 
       PIO_DMA_firstbuffer(); // dmacall, buffer switch and reset
     
doitagain:
  mainloop_repeat = true;

  while (mainloop_repeat) { // switch between buffer

        buff_nb_count++;   

        //PIO_Capture_DMA(mb -> buffer_switch);  // 0 for A, 1 for B
        if (mb->buffer_switch) mb->Pab = mb->A; else mb->Pab = mb->B; 

	while (mb->repeat) { // buffer loop
     // PIO signal generation and buffer A or B filling
           
          k++;
         // DSP();
         // BS_2_IO();
   
          Unpack_word_bs1(mb->Pab); 
             
             if (CIC(1, mb->to_bs[1])) { 
               j=k/CICOSR-1;
               #ifdef BUFFOUT 
                      mb->CIC_C[j] = mb->CIC[1];
               #endif
             }
             
             if (mb->count == SAMPLES_NUMBER-1) {mb->repeat = false; mb->count=0;}  else mb->count++;
            
             
        }
        
      
        // wait for end of DMA callback, next buffer available   
        while(mb->dmacall);   
        Reset();
        mb -> buffer_switch = !mb-> buffer_switch;

         if ( buff_nb_count == BUFFER_NUMBER) {
           Disable_Capture();   
           mainloop_repeat = false;
              buff_nb_count = 0; 
              #ifdef BUFFOUT 
                     Print_Buffer(mb->CIC_C);
                     printf(I"STOP  \n\r");
              #endif
         }
         
  }

  #ifdef BUFFOUT 
        if (doit == 0) { // choose the number of doit to do by increasing the 0
          printf(I"End of Program:  number of samples= %d \r\n", k/CICOSR); 
          while (1); 
        } else doit++;
  #endif
  goto doitagain;
  
  printf(I"End of Program \r\n");
  while (1);
}


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

