//jj kazazian 2018//
// Xult board R225 removed

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
 //struct _MAILBOX mb;// @ 0x20420000;
   
MAILBOX *mb;
uint32_t doit;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
// See main_config.c


static void PIO_Generation(void) {
  
         mb->repeat  = true;

        
	while (mb->repeat) { 
          // PIO signal generation
                       DSP();
                       //Copy_BS_to_Buffer(mb.count); //buffer C to check tx values
                       BS_2_IO(); 
                       mb->count++;
                 
                    
         // Next_action();     // State Machine next action to do  
         
             if ( mb->count == SAMPLES_NUMBER) {
               mb->repeat = false;
               break;
             }
        }
  
  
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

 extern int main (void)  
{
          uint32_t *pDest;
          uint32_t buff_nb_count = 0;
          bool mainloop_repeat = true;
         

       // fill Stack with 0xFEEDFACE Pattern 
       for (pDest = (uint32_t *)ASTACK; pDest < (uint32_t *)(ASTACK+SSTACK);) 
       {
              *pDest++ = 0xFEEDFACE;
       }
 
  mb = (MAILBOX *)malloc( sizeof(MAILBOX)); 

        Main_Config();
        Clock_Config();        
        Sense_Config();
        DSP_Config();
        Init_state();
        Capture_Config(PIOA);
        Enable_Capture();  
 
        //Sense_Dump_param(); // SPI com and read registers
         mb->dmacall   =  true;
         mb->dmaswitch = false;
         
         waitKey(); 
         
        // Fill the first buffer 
        PIO_Capture_DMA( mb->dmaswitch);
        PIO_Generation();
         

    
doitagain:
  mainloop_repeat = true;

         
  while (mainloop_repeat) {
        buff_nb_count++;    

        mb->count = 0; // counting of sample Numbers 
        mb->dmacall = true;
        mb->repeat  = true;

        
	while (mb->repeat) { 
          // PIO signal generation
             DSP();
             BS_2_IO();
             
             if (mb->dmaswitch) Capture_Unpack(mb->A); 
             else Capture_Unpack(mb->B);     
             
             mb->count++;

             if ( mb->count == SAMPLES_NUMBER) {
               mb->repeat = false;
             }
        }
        // wait for end of DMA callback, next buffer available   
        while(mb->dmacall); 
  
         if ( buff_nb_count == BUFFER_NUMBER) {
              mainloop_repeat = false;
              buff_nb_count = 0;  
         }
  
  }

        if (doit == 0) { 
                  printf("\r\n   End of Program \r\n"); 
                  while (1); 
        } else doit++;
  
  goto doitagain;
  
  printf("    End of Program \r\n");
  while (1);
}
/** \endcond */

       //PIO_Print_Buffer(mb.B);
       //Print_TxBS_Buffer(); //C buffer
       //Print_RxBS_Buffer();  // A buffer
       //PIO_Clear_Buffer(mb.C); 
       //PIO_Clear_Buffer(mb.A);
       // printf("    Start unpack \r\n");
       // PIO_Unpack_Buffer(mb->A);
       // printf("    Unpack done \r\n");
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

