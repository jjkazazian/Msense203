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

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
// See main_config.c

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

extern int main (void)  
{
  
  //get ASTACK in memory map
          uint32_t *pDest;
         

                //fill Stack with 0xFEEDFACE Pattern 
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
         mb->dmacall =  true;
            
  while (1) {
             
        waitKey();
        
        Memory_Config(mb);
        waitKey();
        
        PIO_Capture_DMA();
        printf("    DMA restart\r\n");
        
        mb->count = 0; // counting of sample Numbers 
        mb->dmacall = true;
        mb->repeat  = true;

        printf("    boolean reset %d \r\n",mb->count );
	while (mb->repeat) { // PIO signal generation
                       DSP();
                       //Copy_BS_to_Buffer(mb.count); //buffer C to check tx values
                       BS_2_IO(); 
                       mb->count++;
                       // printf("  %d \r\n", mb->count);
                    
         // Next_action();     // State Machine next action to do  
         
             if (mb->count == SAMPLES_NUMBER) {
               mb->repeat = false;
               break;
             }
        }
           
        while(mb->dmacall); // wait for end of DMA callback
        
      
       PIO_Copy_Buffer(mb->A, mb->B);
       
       //PIO_Print_Buffer(mb.C);
       //PIO_Print_Buffer(mb.B);
       
       //Print_TxBS_Buffer(); //C buffer
       PIO_Unpack_Buffer(mb->B);
       printf("    Unpack done");
       //Print_RxBS_Buffer();  // A buffer
      
       
       //PIO_Clear_Buffer(mb.C); 
       //PIO_Clear_Buffer(mb.A);
  }
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

