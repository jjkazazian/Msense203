/* jj kazazaian 2018*/
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "dsp_sense.h"

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

float pi=3.1415926535897932384626433832795028841971693993751058209749445; // Pi


// Signal input
static uint32_t n;       // 16 bits signed signal full scale
static uint32_t FS;      // +/- dynamic = +/-FS/2
static int32_t amp;      // 16 bits full scale  
static uint32_t fin;     // expected input signal frequency
static float fain;       // actual recomputed signal frequency
static uint32_t N;       // Sinus number of points
static float T;          // Clock period
static float Tin;        // Sinus period

// CIC
//static uint32_t CICScale;          // standard scale not used here
static uint32_t CICBout;           // CIC ouput number of bit
static uint32_t CICScal_sirag;     // CIC output applied scaling at 24 bits

// Modulator
static int32_t lvl1;
static int32_t lvl2;
static float gfb1;
static float gfb2;
static uint32_t gff;

static struct _MOD mod0;
static struct _MOD mod1;
static struct _MOD mod2;
static struct _MOD mod3;
static struct _MOD mod4;

static struct _CIC cic0;
static struct _CIC cic1;
static struct _CIC cic2;
static struct _CIC cic3;
static struct _CIC cic4;


extern struct _MAILBOX mb;
static uint32_t nsinus;   // sinus buffer counter  
static bool flag_osr;
//int32_t sinus[BSBUFFER];        // Input buffer for min 50Hz sinus
//int32_t Dout[BSBUFFER/8];       // Output buffer
//uint32_t idx;                   // buffer index

/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/

void Copy_BS_to_Buffer(uint32_t index) {
  
    // generated bit stream
    mb.BS0tx[index]= mb.BS0[0];
    mb.BS1tx[index]= mb.BS1[0];
    mb.BS2tx[index]= mb.BS2[0];
    mb.BS3tx[index]= mb.BS3[0];
    mb.BS4tx[index]= mb.BS4[0];
  
}

void Print_TxBS_Buffer(void) {
  uint32_t i;
   
  // generated bit stream
  
 for (i = 0; i < SAMPLES_NUMBER; i++) {
   
   printf("TX BS0 [%02d] = %02d BS1= %02d BS2= %02d BS3= %02d  BS4= %02d\r\n" , i, mb.BS0tx[i], mb.BS1tx[i], mb.BS2tx[i], mb.BS3tx[i], mb.BS4tx[i] );  
   
 }
  
}

void Print_RxBS_Buffer(void) {
   uint32_t i;
   printf("\r\n");  
  // generated bit stream
  
 for (i = 0; i < SAMPLES_NUMBER; i++) {
   
   printf("RX BS0 [%02d] = %02d BS1= %02d BS2= %02d BS3= %02d  BS4= %02d\r\n" , i, mb.BS0rx[i], mb.BS1rx[i], mb.BS2rx[i], mb.BS3rx[i], mb.BS4rx[i] );  
   
 }
  
}


static void Sin_Config(void)
{
// Signal input
n   = 16;                     // 16 bits signed signal full scale
FS  = (int)round(pow(2,n));   // +/-2^n dynamic = +/-FS/2
amp = FS/4;                   // 16 bits full scale  
fin = 1200;                   // requested input signal frequency
Tin = M/(float)fin;           // clock period MicroSecond
T   = M/(float)BSCLOCK;       // clock period MicroSecond
N   = (int)round(Tin/T);      // sinus Numbers of point
fain = M/(N*T);               // actual input frequency
printf ("sinus amplitude  = %d \r\n", amp );
printf ("sinus nb of point  = %d \r\n", N );
printf ("sinus frequency = %f \r\n", fain );
}


static void Mod_Config(void)
{
lvl1  = FS/4;
lvl2  = FS/2;
gfb1  = FS*2.4/4;
gfb2  = 2*gfb1;
gff   = 2;

printf ("lvl1  %d \r\n", lvl1 );
printf ("lvl2  %d \r\n", lvl2 );
printf ("gfb1  %d \r\n", (int32_t)gfb1 );
printf ("gfb2  %d \r\n", (int32_t)gfb2 );
}

static void CIC_Config(void)
{
//CICScale = CICOSR^CICORDER;                // standard scale not used here
CICBout  = CICORDER*(int)(log((double)CICOSR)/log(2));    // CIC ouput number of bit
CICScal_sirag = (int)pow(2,NS-(NBS+CICBout-1)); // CIC output applied scaling at 24 bits
//printf ("CICScal_sirag  %d \r\n", CICScal_sirag );

}

void Sinus_Gen(void) 
{// not used anymore
  uint32_t i; 
  float s;  
  
  for (i = 0; i < N; i++) {
  s = amp*sin(i*T*2*pi*fain/M);
  //sinus[i] = (int)round(s);
 // printf ("  %d \r\n", sinus[i] );
  }

}

static int32_t Sinus(uint32_t index, uint32_t nb) 
{
  float s;  
  s = amp*sin(index*T*2*pi*fain/M+2*pi/nb);
  //s = amp*arm_sin_f32(index*T*2*pi*fain/M+2*pi/nb);
  return (int)round(s);
}


static int32_t Mod_Comp(int32_t in) 
{
  // comparator
  int32_t bs;

    if ((in > -lvl1) && (in < lvl1))    bs = 0;    
    if ((in >= lvl1) && (in < lvl2))    bs = 1;
    if ((in >= lvl2))                   bs = 2;      
    if ((in <= -lvl1) && (in > -lvl2))  bs = -1;
    if ((in <= -lvl2))                  bs = -2;
    
return bs;
}


static void Modulator(struct _MOD * mod) 
{
  mod->bs   = Mod_Comp(mod->int2);
  mod->int2 = mod->int2 + mod->int1 + gff*mod->xin - (int32_t)gfb2*mod->bs;
  mod->int1 = mod->int1 + mod->z  - (int32_t)gfb1*mod->bs;
  mod->z    = mod->xin;
}

static void Combfilter(struct _MOD * mod, struct _CIC * cic) 
{
  cic->osr++;
  
// integrator

cic->int3 = cic->int3 + cic->int2;
cic->int2 = cic->int2 + cic->int1;
cic->int1 = cic->int1 + mod->bs;

    if (cic->osr == CICOSR) { // decimator
                      
              cic->osr=0;
              flag_osr = true;
              
              // derivator
              cic->der1    = cic->int3;
              cic->der2    = cic->der1 - cic->zder1;
              cic->der3    = cic->der2 - cic->zder2;
              cic->xout    = cic->der3 - cic->zder3;

              cic->zder1   = cic->der1;
              cic->zder2   = cic->der2;
              cic->zder3   = cic->der3;   

              cic->xout = CICScal_sirag*cic->xout;     
    }   
}

static void Decimation(void)  {
 uint32_t i; 
         if (flag_osr){
                        flag_osr = false;

                        if (mb.idx == BSIZE-1) {
                               mb.buffer_full =  true;
                               for (i = 0; i < BSIZE; i++) printf ("  %d \r\n", cic0.xout);
                               mb.buffer_full=false;  
                               mb.idx=0;    
                        } else mb.idx++;
                         
          }
}

static void BS2mb(void)  {

          mb.BS0[mb.idx] = mod0.bs;
          mb.BS1[mb.idx] = mod1.bs;
          mb.BS2[mb.idx] = mod2.bs;
          mb.BS3[mb.idx] = mod3.bs;
          mb.BS4[mb.idx] = mod4.bs;
          
          if (mb.idx == BSIZE-1) {
                                  mb.buffer_full =  true;
                                  mb.idx=0;    
          } else mb.idx++;
    }


void DSP_Config(void) 
{
Sin_Config();
Mod_Config();
CIC_Config();
flag_osr    = false;
mb.idx = 0;
nsinus = 0;
}

void DSP(void)
{
         int32_t sin;
         
          if (nsinus == N-1) nsinus=0; else nsinus++;
          sin = Sinus(nsinus,1);
          mod0.xin = sin;
          mod1.xin = 0;
          mod2.xin = sin;
          mod3.xin = 0;
          mod4.xin = sin;
       
          Modulator(&mod0);
          Modulator(&mod1);
          Modulator(&mod2);
          Modulator(&mod3);
          Modulator(&mod4);
          
           /*
           Combfilter(&mod0,&cic0);   
           Combfilter(&mod1,&cic1);    
           Combfilter(&mod2,&cic2);   
           Combfilter(&mod3,&cic3);   
           Combfilter(&mod4,&cic4);   
          */
           
          // Decimation();
          
          BS2mb();
          
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
