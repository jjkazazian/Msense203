//jj kazazian 2018//
// Xult board R225 removed
//samv71q21.h
//#define __DCACHE_PRESENT       0      /**< SAMV71Q21 does provide a Data Cache         */ // jjk set to 0 for debug
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

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
// see main_config.h  
 
  
 MAILBOX *mb = (MAILBOX *)0x20000100;
 
 //  MAILBOX mb @ 0x20000100;
 


uint32_t doit;
uint32_t k;  



/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
// See main_config.c


static void Print_Buffer(int32_t * buf) {// not working
  
  uint32_t j;
        for (j = 0; j < CIC_NUMBER*BUFFER_NUMBER; j++) {
        printf(" %d \n\r" ,buf[j]);
        }
}


static void Reset(void) {
        mb->count = 0;        // counting of sample Numbers 
        mb->dmacall = true;   // DMA interupt rise when false
        mb->repeat  = true;   // repeat the io and dma loop
}

static void PIO_Generation(void) {
  
	while (mb->repeat) { 
          // PIO signal generation
                       DSP();
                       //Copy_BS_to_Buffer(mb.count); //buffer C to check tx values
                       BS_2_IO(); 
             
            // printf(" %d   %x     ", mb->dmaswitch, mb->Pab); 
            // printf(" %d     ", mb->to_bs[0]);
            // printf(" %d \n\r" ,mb->BS0);
           
         // Next_action();     // State Machine next action to do  
         
        if (mb->count == SAMPLES_NUMBER-1) {mb->repeat = false; mb->count=0;}  else mb->count++;
        }
        // wait for end of DMA callback, next buffer available   
        while(mb->dmacall); 
        mb -> dmaswitch = !mb-> dmaswitch;
        Reset(); 
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

  extern int main (void)  
{ 
        uint32_t buff_nb_count = 0;
        bool mainloop_repeat = true;
        uint32_t j;
 
       // mb = (MAILBOX *)malloc( sizeof(MAILBOX)); changed to TCM     
        
        Main_Config();
        Clock_Config();        
        Sense_Config();
        DSP_Config();
        Init_state();
        Capture_Config(PIOA);
        Enable_Capture();  
 
        //Sense_Dump_param(); // SPI com and read registers
         mb->dmaswitch = false;
         IO_ctrl(6,0);  
         waitKey(); 
            
         // IO_ctrl(6,1);  
         // IO_ctrl(6,0);  
         
        // Fill the first buffer 
        printf("   START  \n\r");
        if (mb->dmaswitch) mb->Pab = mb->A; else mb->Pab = mb->B;  
        PIO_Capture_DMA( mb -> dmaswitch);
        Reset();
        PIO_Generation();


    
doitagain:
  mainloop_repeat = true;

  
  while (mainloop_repeat) {

        buff_nb_count++;   
        //printf("   buff: %d  \n\r", buff_nb_count);
        PIO_Capture_DMA(mb -> dmaswitch);  // 0 for A, 1 for B
        if (mb->dmaswitch) mb->Pab = mb->A; else mb->Pab = mb->B; 

	while (mb->repeat) { 
     // PIO signal generation and buffer A or B filling
           
          k++;
             
             DSP();
             BS_2_IO();
             
                IO_ctrl(6,1);  
        
             Unpack_64b_bs0(mb->Pab);
            
             
            // printf(" %d   %x     ", mb->dmaswitch, mb->Pab); 
            // printf(" %d   \n\r  ", mb->to_bs[1]);
            // printf(" %d \n\r" ,mb->BS0);
                
     
             if (CIC(0, mb->to_bs[0])) { 
               j=k/CICOSR-1;
               //mb->CIC_C[j] = mb->CIC0;
               //printf(" %d, %d \n\r" ,j, mb->CIC_C[j]); 
             }
             
      
             
             if (mb->count == SAMPLES_NUMBER-1) {mb->repeat = false; mb->count=0;}  else mb->count++;
               
             IO_ctrl(6,0);  
        }
       
        // wait for end of DMA callback, next buffer available   
        while(mb->dmacall);   
        Reset();
        mb -> dmaswitch = !mb-> dmaswitch;

         if ( buff_nb_count == BUFFER_NUMBER) {
              mainloop_repeat = false;
              buff_nb_count = 0; 
             // printf("   STOP  \n\r");
              //Print_Buffer(mb->CIC_C);
   
         }
         
  }
   /*
        if (doit == 0) { 
          printf("\r\n   End of Program:  k= %d \r\n", k/CICOSR); 
          
                  while (1); 
        } else doit++;
*/
  goto doitagain;
  
  printf("    End of Program \r\n");
  while (1);
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

