/* jj kazazaian 2018*/
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "mux.h"

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern MAILBOX *mb;    

struct _MUX mx;
struct _DEMUX demx;
    
/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/

  static void mxcode(void) {
    // D3[0] = SDI1(0), D3(1) = SDV1(1), D3(2) = SDI2(2), D3(3) = 0 parity      
    // D2[0] = SD0(2),  D2(1) = SDV1(0), D2(2) = SDI2(1), D2(3) = SDV2(2)
    // D1[0] = SD0(1),  D1(1) = SDI1(2), D1(2) = SDI2(0), D1(3) = SDV2(1)      
    // D0[0] = SD0(0),  D0(1) = SDI1(1), D0(2) = SDV1(2), D0(3) = SDV2(0)

    // bits attribution, Change on posedge
    // Sigma delta order MSB to LSB
    // BS4,  BS3,  BS2,  BS1,  BS0    
    // SDV2, SDI2, SDV1, SDI1, SD0    
     // Bitstream values   
        //  2 = 010
        //  1 = 001
        //  0 = 000
        // -1 = 111
        // -2 = 110
      mx.D3[0] = mb->BS1 & B0_msk;  // SDI1(0)
      mx.D3[1] = mb->BS2 & B1_msk;  // SDV1(1)
      mx.D3[2] = mb->BS3 & B2_msk;  // SDI2(2)
      mx.D3[3] = false; 
 
      mx.D2[0] = mb->BS0 & B2_msk;  // SD0(2)
      mx.D2[1] = mb->BS2 & B0_msk;  // SDV1(0)
      mx.D2[2] = mb->BS3 & B1_msk;  // SDI2(1)
      mx.D2[3] = mb->BS4 & B2_msk;  // SDV2(2)

      mx.D1[0] = mb->BS0 & B1_msk;  // SD0(1)
      mx.D1[1] = mb->BS1 & B2_msk;  // SDI1(2)
      mx.D1[2] = mb->BS3 & B0_msk;  // SDI2(0)
      mx.D1[3] = mb->BS4 & B1_msk;  // SDV2(1)

      mx.D0[0] = mb->BS0 & B0_msk;  // SD0(0)
      mx.D0[1] = mb->BS1 & B1_msk;  // SDI1(1)
      mx.D0[2] = mb->BS2 & B2_msk;  // SDV1(2)
      mx.D0[3] = mb->BS4 & B0_msk;  // SDV2(0)

      mx.Fsync[0] = false;   
      mx.Fsync[1] = false;   
      mx.Fsync[2] = false;   
      mx.Fsync[3] = true;
  
    } 
 
  int8_t decodeBS(bool *code ) {
    //  2 = 010
    //  1 = 001
    //  0 = 000
    // -1 = 111
    // -2 = 110
   
   if (code[2]==0 && code[1]==1 && code[0]==0) return 2;
   if (code[2]==0 && code[1]==0 && code[0]==1) return 1;
   if (code[2]==0 && code[1]==0 && code[0]==0) return 0;
   if (code[2]==1 && code[1]==1 && code[0]==1) return -1;
   if (code[2]==1 && code[1]==1 && code[0]==0) return -2;
   //Wrong code
   if (code[2]==0 && code[1]==1 && code[0]==1) return 97;
   if (code[2]==1 && code[1]==0 && code[0]==0) return 98;
   if (code[2]==1 && code[1]==0 && code[0]==1) return 99;
   else return 99;
   }
  
  
  
   void demxcode(void) {
     
             
    //SD0 bit stream BS0   
      demx.SD0[0]= mb->to_demux[0] & B0_msk; 
      demx.SD0[1]= mb->to_demux[0] & B1_msk;
      demx.SD0[2]= mb->to_demux[0] & B2_msk;
      
    //SDI1  bit stream BS1   
      demx.SDI1[0]=  mb->to_demux[0] & B3_msk; 
      demx.SDI1[1]=  mb->to_demux[1] & B0_msk; 
      demx.SDI1[2]=  mb->to_demux[1] & B1_msk;  

    //SDV1  bit stream BS2   
      demx.SDV1[0]= mb->to_demux[1] & B2_msk; 
      demx.SDV1[1]= mb->to_demux[1] & B3_msk; 
      demx.SDV1[2]= mb->to_demux[2] & B0_msk;      
      
    //SDI2  bit stream BS3
      demx.SDI2[0]= mb->to_demux[2] & B1_msk;  
      demx.SDI2[1]= mb->to_demux[2] & B2_msk;   
      demx.SDI2[2]=  mb->to_demux[2]& B3_msk;   
      
    //SDV2  bit stream BS4

      demx.SDV2[0] = mb->to_demux[3] & B0_msk; 
      demx.SDV2[1] = mb->to_demux[3] & B1_msk; 
      demx.SDV2[2] = mb->to_demux[3] & B2_msk; 
         
      //mx.BD3[3] = false; 
      mb->to_bs[0] = decodeBS(demx.SD0);
      mb->to_bs[1] = decodeBS(demx.SDI1);
      mb->to_bs[2] = decodeBS(demx.SDV1);
      mb->to_bs[3] = decodeBS(demx.SDI2);
      mb->to_bs[4] = decodeBS(demx.SDV2);
 
    } 
 
 void demxcode_bs0(void) {   
   // for runtime measurement
    //SD0 bit stream BS0   
      demx.SD0[0]= mb->to_demux[0] & B0_msk; 
      demx.SD0[1]= mb->to_demux[0] & B1_msk;
      demx.SD0[2]= mb->to_demux[0] & B2_msk;
    
      //mx.BD3[3] = false; 
      mb->to_bs[0] = decodeBS(demx.SD0);
    } 
  
  
  
 static void Drive_IO(void) {
     uint32_t i; 
     uint32_t j; 
     uint8_t data; 
     
     for (i = 0; i < 4; i++) {
            
                IO_ctrl(0,mx.D0[i]);
                IO_ctrl(1,mx.D1[i]);
                IO_ctrl(2,mx.D2[i]);
                IO_ctrl(3,mx.D3[i]);
                IO_ctrl(4,mx.Fsync[i]);
                IO_clear(5); // Clock capture
                //Wait(1);
                for (j = 0; j < 1000; j++) __asm("nop");
                IO_set(5);  // Clock capture on rise
                //Wait(1);
                for (j = 0; j < 1000; j++) __asm("nop");
                IO_clear(5);
                IO_ctrl(4,0);

                //data = mx.Fsync[i]*16+mx.D3[i]*8+mx.D2[i]*4+mx.D1[i]*2+mx.D0[i];
                // printf("d=   %x\r\n",data);
                // Print_int8_to_bin(data);
                // printf("\r\n");
                //mb.C[mb.count] |= (uint32_t)data << i*8; //Checker
           
      }
     
 }  
  
  
    void BS_2_IO(void ) {
  
      mxcode();
      Drive_IO();
 
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
