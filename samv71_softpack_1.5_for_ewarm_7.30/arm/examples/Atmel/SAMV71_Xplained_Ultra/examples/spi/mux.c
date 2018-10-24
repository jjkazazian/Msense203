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
extern struct _MAILBOX mb;    

struct _MUX mx;
struct _DEMUX demx;
    


/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/

  static void Load_BS(void) {
    mx.sd0  = (int8_t)mb.BS0[0];
    mx.sdI1 = (int8_t)mb.BS1[0];
    mx.sdV1 = (int8_t)mb.BS2[0];
    mx.sdI2 = (int8_t)mb.BS3[0];
    mx.sdV2 = (int8_t)mb.BS4[0];   
        }
  
     
        
  static  void bs_2_bin(struct _MUX * mux) {
      mux->SD0[0] = mux->sd0 & B0_msk;  
      mux->SD0[1] = mux->sd0 & B1_msk;  
      mux->SD0[2] = mux->sd0 & B2_msk; 
      
      mux->SDI1[0] = mux->sdI1 & B0_msk;  
      mux->SDI1[1] = mux->sdI1 & B1_msk;  
      mux->SDI1[2] = mux->sdI1 & B2_msk; 
      
      mux->SDV1[0] = mux->sdV1 & B0_msk;  
      mux->SDV1[1] = mux->sdV1 & B1_msk;  
      mux->SDV1[2] = mux->sdV1 & B2_msk;      
      
      mux->SDI2[0] = mux->sdI2 & B0_msk;  
      mux->SDI2[1] = mux->sdI2 & B1_msk;  
      mux->SDI2[2] = mux->sdI2 & B2_msk; 
      
      mux->SDV2[0] = mux->sdV2 & B0_msk;  
      mux->SDV2[1] = mux->sdV2 & B1_msk;  
      mux->SDV2[2] = mux->sdV2 & B2_msk; 
      
    }
 
 
  static void Load_DMX(void) {
    demx.D0 = (uint8_t)mb.to_demux[0];
    demx.D1 = (uint8_t)mb.to_demux[1];
    demx.D2 = (uint8_t)mb.to_demux[2];
    demx.D3 = (uint8_t)mb.to_demux[3];
    
   // Print_byte("D2", mb.to_demux[2]);
   // Print_byte("D3", mb.to_demux[3]);
        }  
  
  static  void dmx_2_bin(struct _DEMUX * demx) {
      demx->BD0[0] = demx->D0 & B0_msk;  
      demx->BD0[1] = demx->D0 & B1_msk;  
      demx->BD0[2] = demx->D0 & B2_msk; 
      demx->BD0[3] = demx->D0 & B3_msk; 
      
      demx->BD1[0] = demx->D1 & B0_msk;  
      demx->BD1[1] = demx->D1 & B1_msk;  
      demx->BD1[2] = demx->D1 & B2_msk; 
      demx->BD1[3] = demx->D1 & B3_msk; 
      
      demx->BD2[0] = demx->D2 & B0_msk;  
      demx->BD2[1] = demx->D2 & B1_msk;  
      demx->BD2[2] = demx->D2 & B2_msk;      
      demx->BD2[3] = demx->D2 & B3_msk; 
      
      demx->BD3[0] = demx->D3 & B0_msk;  
      demx->BD3[1] = demx->D3 & B1_msk;  
      demx->BD3[2] = demx->D3 & B2_msk; 
      demx->BD3[3] = demx->D3 & B3_msk;
    }
  static void mxcode(void) {
    // D3[0] = SDI1(0), D3(1) = SDV1(1), D3(2) = SDI2(2), D3(3) = 0 parity      
    // D2[0] = SD0(2),  D2(1) = SDV1(0), D2(2) = SDI2(1), D2(3) = SDV2(2)
    // D1[0] = SD0(1),  D1(1) = SDI1(2), D1(2) = SDI2(0), D1(3) = SDV2(1)      
    // D0[0] = SD0(0),  D0(1) = SDI1(1), D0(2) = SDV1(2), D0(3) = SDV2(0)
     // Load_BS();
     // bs_2_bin(&mx);
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
      mx.D3[0] = mb.BS1[0] & B0_msk;
      mx.D3[1] = mb.BS2[0] & B1_msk; //
      mx.D3[2] = mb.BS3[0] & B2_msk;  
      mx.D3[3] = false; 
      //mx.D3[0] = mx.SDI1[0];  mx.D3[1] = mx.SDV1[1]; mx.D3[2] = mx.SDI2[2]; mx.D3[3] = false; 
      mx.D2[0] = mb.BS0[0] & B2_msk; 
      mx.D2[1] = mb.BS2[0] & B0_msk; //
      mx.D2[2] = mb.BS3[0] & B1_msk;  
      mx.D2[3] = mb.BS4[0] & B2_msk; 
      //mx.D2[0] = mx.SD0[2]; mx.D2[1] = mx.SDV1[0]; mx.D2[2] = mx.SDI2[1]; mx.D2[3] = mx.SDV2[2];
      mx.D1[0] = mb.BS0[0] & B1_msk;  
      mx.D1[1] = mb.BS1[0] & B2_msk; 
      mx.D1[2] = mb.BS3[0] & B0_msk;  
      mx.D1[3] = mb.BS4[0] & B1_msk;  
      //mx.D1[0] = mx.SD0[1]; mx.D1[1] = mx.SDI1[2]; mx.D1[2] = mx.SDI2[0]; mx.D1[3] = mx.SDV2[1]; 
      mx.D0[0] = mb.BS0[0] & B0_msk;
      mx.D0[1] = mb.BS1[0] & B1_msk;
      mx.D0[2] = mb.BS2[0] & B2_msk; //
      mx.D0[3] = mb.BS4[0] & B0_msk;  
      //mx.D0[0] = mx.SD0[0];  mx.D0[1] = mx.SDI1[1]; mx.D0[2] = mx.SDV1[2]; mx.D0[3] = mx.SDV2[0];
      mx.Fsync[0] = false;   
      mx.Fsync[1] = false;   
      mx.Fsync[2] = false;   
      mx.Fsync[3] = true;
  
    } 
 
  static  uint8_t decodeBS(bool *code ) {
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
     // Load_DMX();
    //  dmx_2_bin(&demx);
      
    //SD0 bit stream BS0   
  //demx.SD0[0]=  demx.BD0[0]; 
    demx.SD0[0]= mb.to_demux[0] & B0_msk;
  //demx.SD0[1]=  demx.BD1[0]; 
    demx.SD0[1]= mb.to_demux[0] & B1_msk;
  //demx.SD0[2]=  demx.BD2[0];
    demx.SD0[2]= mb.to_demux[0] & B2_msk;
      
      
      
    //SDI1  bit stream BS1   
    //demx.SDI1[0]= demx.BD3[0]; 
      demx.SDI1[0]=  mb.to_demux[0] & B3_msk;
    //demx.SDI1[1]= demx.BD0[1];  
      demx.SDI1[1]=  mb.to_demux[1] & B0_msk; 
    //demx.SDI1[2]= demx.BD1[1];  
      demx.SDI1[2]=  mb.to_demux[1] & B1_msk;  

    //SDV1  bit stream BS2   
    //demx.SDV1[0]= demx.BD2[1];  
      demx.SDV1[0]= mb.to_demux[1] & B2_msk;
    //demx.SDV1[1]= demx.BD3[1];  
      demx.SDV1[1]= mb.to_demux[1] & B3_msk;
    //demx.SDV1[2]= demx.BD0[2];   
      demx.SDV1[2]= mb.to_demux[2] & B0_msk;      
      
      
    //SDI2  bit stream BS3
    //demx.SDI2[0]= demx.BD1[2];   
      demx.SDI2[0]= mb.to_demux[2] & B1_msk;  
    //demx.SDI2[1]= demx.BD2[2];   
      demx.SDI2[1]= mb.to_demux[2] & B2_msk;   
    //demx.SDI2[2]= demx.BD3[2]; 
      demx.SDI2[2]=  mb.to_demux[2]& B3_msk;   
      
    //SDV2  bit stream BS4
    //demx.SDV2[0]= demx.BD0[3];
      demx.SDV2[0] = mb.to_demux[3] & B0_msk; 
    //demx.SDV2[1]= demx.BD1[3]; 
      demx.SDV2[1] = mb.to_demux[3] & B1_msk; 
    //demx.SDV2[2]= demx.BD2[3];
      demx.SDV2[2] = mb.to_demux[3] & B2_msk; 
      
      //mx.BD3[3] = false; 
      mb.demux_to_bs[0] = decodeBS(demx.SD0);
      mb.demux_to_bs[1] = decodeBS(demx.SDI1);
      mb.demux_to_bs[2] = decodeBS(demx.SDV1);
      mb.demux_to_bs[3] = decodeBS(demx.SDI2);
      mb.demux_to_bs[4] = decodeBS(demx.SDV2);
      
     
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
                for (j = 0; j < 100; j++) __asm("nop");
                IO_set(5);  // Clock capture on rise
                //Wait(1);            // try and remove
                //PIO_Capture();      // try and remove
                //Wait(1);
                for (j = 0; j < 100; j++) __asm("nop");
                IO_clear(5);
                IO_ctrl(4,0);

                data = mx.Fsync[i]*16+mx.D3[i]*8+mx.D2[i]*4+mx.D1[i]*2+mx.D0[i];
                // printf("d=   %x\r\n",data);
                // Print_int8_to_bin(data);
                // printf("\r\n");
                mb.C[mb.count] |= (uint32_t)data << i*8; 
           
      }
     
 }  
  
  
    void BS_2_IO(void ) {
  
      mxcode();
      //printf(" SD: %d  hex: %x  ----> %d %d %d", mx.sd0, mx.sd0, mx.SD0[2],mx.SD0[1],mx.SD0[0]); 
      //or// Print_int8_to_bin(mx.sd0);
      //printf( "\r\n");
      
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
