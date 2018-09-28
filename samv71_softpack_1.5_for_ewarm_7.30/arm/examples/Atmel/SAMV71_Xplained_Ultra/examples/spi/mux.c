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
    
// Sigma delta order MSB to LSB
// BS4, BS3, BS2, BS1, BS0    
// SDV2, SDI2, SDV1, SDI1, SD0    
    
    //  2 = 010
    //  1 = 001
    //  0 = 000
    // -1 = 111
    // -2 = 110

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

  static void mxcode(void) {
    // D3[0] = SDI1(0), D3(1) = SDV1(1), D3(2) = SDI2(2), D3(3) = 0 parity      
    // D2[0] = SD0(2),  D2(1) = SDV1(0), D2(2) = SDI2(1), D2(3) = SDV2(2)
    // D1[0] = SD0(1),  D1(1) = SDI1(2), D1(2) = SDI2(0), D1(3) = SDV2(1)      
    // D0[0] = SD0(0),  D0(1) = SDI1(1), D0(2) = SDV1(2), D0(3) = SDV2(0)
      Load_BS();
      bs_2_bin(&mx);
    // Change on posedge
      mx.D3[0] = mx.SDI1[0]; mx.D3[1] = mx.SDV1[1]; mx.D3[2] = mx.SDI2[2]; mx.D3[3] = false; 
      mx.D2[0] = mx.SD0[2];  mx.D2[1] = mx.SDV1[0]; mx.D2[2] = mx.SDI2[1]; mx.D2[3] = mx.SDV2[2];
      mx.D1[0] = mx.SD0[1];  mx.D1[1] = mx.SDI1[2]; mx.D1[2] = mx.SDI2[0]; mx.D1[3] = mx.SDV2[1]; 
      mx.D0[0] = mx.SD0[0];  mx.D0[1] = mx.SDI1[1]; mx.D0[2] = mx.SDV1[2]; mx.D0[3] = mx.SDV2[0];
      mx.Fsync[0] = false; mx.Fsync[1] = false; mx.Fsync[2] = false; mx.Fsync[3] = true;
    } 
 
 static void Drive_IO(void) {
     uint32_t i; 
          for (i = 0; i < 5; i++) {
      MUX_ctrl(0,mx.D0[i]);
      MUX_ctrl(1,mx.D1[i]);
      MUX_ctrl(2,mx.D2[i]);
      MUX_ctrl(3,mx.D3[i]);
      MUX_ctrl(4,mx.Fsync[i]);
      Wait(1);
      }
 }  
  
  
    void Print_bs_2_bin(void ) {
  
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
