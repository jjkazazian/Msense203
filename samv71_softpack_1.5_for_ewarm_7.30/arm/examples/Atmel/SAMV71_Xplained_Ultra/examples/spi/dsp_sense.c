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
extern MAILBOX *mb;    
float pi=3.1415926535897932384626433832795028841971693993751058209749445; // Pi

 uint32_t ii;
 
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
//static uint32_t CICScale;        // standard scale not used here
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


extern MAILBOX *mb;
static uint32_t nsinus;   // sinus buffer counter  

//int32_t sinus[BSBUFFER];        // Input buffer for min 50Hz sinus
//int32_t Dout[BSBUFFER/8];       // Output buffer
//uint32_t idx;                   // buffer index

/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/


static void Sin_Config(void)
{
// Signal input
n   = 16;                     // 16 bits signed signal full scale
FS  = (int)round(pow(2,n));   // +/-2^n dynamic = +/-FS/2
amp = FS/4;                   // 16 bits full scale  
fin = 70;                     // requested input signal frequency
Tin = M/(float)fin;           // clock period MicroSecond
T   = M/(float)BSCLOCK;       // clock period MicroSecond
N   = (int)round(Tin/T);      // sinus Numbers of point
fain = M/(N*T);               // actual input frequency
printf (I"sinus amplitude    = %d "R, amp );
printf (I"sinus nb of point  = %d "R, N );
printf (I"sinus frequency    = %f "R, fain );
}


static void Mod_Config(void)
{
lvl1  = FS/4;
lvl2  = FS/2;
gfb1  = FS*2.4/4;
gfb2  = 2*gfb1;
gff   = 2;

printf (I"lvl1  %d"R, lvl1 );
printf (I"lvl2  %d"R, lvl2 );
printf (I"gfb1  %d"R, (int32_t)gfb1 );
printf (I"gfb2  %d"R, (int32_t)gfb2 );
}

static void CIC_Config(void)
{
//CICScale = CICOSR^CICORDER;                // standard scale not used here
CICBout  = CICORDER*(int)(log((double)CICOSR)/log(2));    // CIC ouput number of bit
CICScal_sirag = (int)pow(2,NS-(NBS+CICBout-1)); // CIC output applied scaling at 24 bits
//printf ("CICScal_sirag  %d \r\n", CICScal_sirag );
cic0.id = 0;
cic1.id = 1;
cic2.id = 2;
cic3.id = 3;
cic4.id = 4;
}


void SNR(void)
{
  double snr;
  double amp;
  double a=0.5;
  double fsv=0.6;
  double FS = pow(2,23);
  double b;
  amp = (a*FS/fsv)/sqrt(2); 
  
  switch (CICOSR) {
    
  case 8:
    b=15;
    break;
  case 16:
    b=12;
    break;
  case 32:
    b=9;
    break;
  case 64:
    b=6;
    break;
  }
  
  snr = 20*log10(amp/mb->cic_rms)+b;
    //                             / 
  printf(I"SNR Sinus 0.5V, BW=2kHz = %d dB"R, (int)snr);
}


void Average(int32_t * buf)
{
  uint32_t j;
  double nb;
  
  mb->cic_avg = 0;
  nb = CIC_NUMBER*BUFFER_NUMBER;

for (j = 2; j < nb; j++) {mb->cic_avg = mb->cic_avg + buf[j];}
  mb->cic_avg = mb->cic_avg/(nb-2);
  printf(I"Average                 = %lf"R, mb->cic_avg);
}


void Average_bs(int32_t * buf)
{
  uint32_t j;
  uint32_t i;
  double nb;
  int8_t bs_data;
  
  mb->cic_avg = 0;
  nb = CIC_NUMBER*BUFFER_NUMBER;
  
        for (j = 0; j < nb; j++) {
          for (i = 0; i < 10; i++) { 
                bs_data = ((buf[j] >> i*3) & 0x7u)<<5;
                bs_data = bs_data >> 5;
                mb->cic_avg = mb->cic_avg + bs_data;
                bs_data = 0;
          }
        }
  mb->cic_avg = mb->cic_avg/nb;
  printf(I"Average             = %lf"R, mb->cic_avg);
}


void Stdev(int32_t * buf)
{
  uint32_t j;
  double nb;
  double rms;
  
  mb->cic_rms = 0;
  nb= CIC_NUMBER*BUFFER_NUMBER;
  
for (j = 2; j < nb; j++) {mb->cic_rms = mb->cic_rms + (buf[j]-mb->cic_avg)*(buf[j]-mb->cic_avg);}
  rms =  mb->cic_rms/(nb-2);
  mb->cic_rms = sqrt(rms);
printf(I"Deviation Standard      = %lf"R, mb->cic_rms);
}

void Stdev_bs(int32_t * buf)
{
  uint32_t i;
  uint32_t j;
  double nb;
  double rms;
  int8_t bs_data;
  
  mb->cic_rms = 0;
  nb= CIC_NUMBER*BUFFER_NUMBER;
  
for (j = 0; j < nb; j++) {

for (i = 0; i < 10; i++) { 
                bs_data = ((buf[j] >> i*3) & 0x7u)<<5;
                bs_data = bs_data >>5;
                mb->cic_rms = mb->cic_rms + (bs_data-mb->cic_avg)*(bs_data-mb->cic_avg);
                bs_data = 0;
          }
}

  rms =  mb->cic_rms/nb;
  mb->cic_rms = sqrt(rms);
printf(I"Deviation Standard      = %lf"R, mb->cic_rms);
}






static int32_t Sinus(uint32_t index) 
{
  double s;
  s = randone()+8*amp*sin((double)(index*T*2*pi*fain/M));
  return (int)round(s/8);
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

static int32_t Combfilter(int32_t bs, struct _CIC * cic) 
{
  cic->osr++;
  
// integrator
cic->int3 = cic->int3 + cic->int2;
cic->int2 = cic->int2 + cic->int1;
cic->int1 = cic->int1 + bs;

    if (cic->osr == CICOSR) { // decimator
                      
              cic->osr=0;
              cic->flag_osr = true;
              
              // derivator
              cic->der1    = cic->int3;
              cic->der2    = cic->der1 - cic->zder1;
              cic->der3    = cic->der2 - cic->zder2;
              cic->xout    = cic->der3 - cic->zder3;

              cic->zder1   = cic->der1;
              cic->zder2   = cic->der2;
              cic->zder3   = cic->der3;   

              cic->xout = CICScal_sirag * cic->xout; 
              return cic->xout;
    }   
}

static void reset_combfilter(struct _CIC * cic)
{
    cic->int3  = 0;
    cic->int2  = 0;
    cic->int1  = 0;
    cic->zder1 = 0;
    cic->der1  = 0;
    cic->zder2 = 0;
    cic->der2  = 0;
    cic->zder3 = 0;
    cic->der3  = 0; 
    cic->xout  = 0;

}

void reset_cic(void)

{
  reset_combfilter(&cic0);
  reset_combfilter(&cic1);
  reset_combfilter(&cic2);
  reset_combfilter(&cic3);
  reset_combfilter(&cic4);

}

static bool Decimate(struct _CIC *cic)  {

         if (cic->flag_osr){
              cic->flag_osr = false;
              switch (cic->id) {
              case 0:
                mb->CIC[0] = cic->xout;
              case 1:  
                mb->CIC[1] = cic->xout;
              case 2:  
                mb->CIC[2] = cic->xout;
              case 3:  
                mb->CIC[3] = cic->xout;
              case 4:  
                mb->CIC[4] = cic->xout;
              }
              return true;
          } else return false;
}


static void BS2mb(void)  {

          mb->BS0 = mod0.bs;
          mb->BS1 = mod1.bs;
          mb->BS2 = mod2.bs;
          mb->BS3 = mod3.bs;
          mb->BS4 = mod4.bs;
          
          mb->buffer_full =  true;
    }


bool CIC(uint32_t id, int32_t bs) {

                switch (id) {
              case 0:
                  Combfilter(bs,&cic0);
                  return Decimate(&cic0);
              case 1:  
                  Combfilter(bs,&cic1);
                  return Decimate(&cic1);
              case 2:  
                  Combfilter(bs,&cic2);
                  return Decimate(&cic2);
              case 3:  
                  Combfilter(bs,&cic3);
                  return Decimate(&cic3);
              case 4:  
                  Combfilter(bs,&cic4);
                  return Decimate(&cic4);
              }
}  



void DSP_Config(void) 
{
Sin_Config();
Mod_Config();
CIC_Config();
cic0.flag_osr    = false;
cic1.flag_osr    = false;
cic2.flag_osr    = false;
cic3.flag_osr    = false;
cic4.flag_osr    = false;
//mb->idx = 0; not used
nsinus = 0;
}

void DSP(void)
{
          int32_t sin;
          if (mb->Ena_sin) sin = Sinus(nsinus); else sin=(int)randone();

          if (nsinus == N-1) nsinus=0; else nsinus++;
          
          if (mb->Ena_bs0) {mod0.xin = sin; Modulator(&mod0);} else mod0.bs=0;
          if (mb->Ena_bs1) {mod1.xin = sin; Modulator(&mod1);} else mod1.bs=0;
          if (mb->Ena_bs2) {mod2.xin = sin; Modulator(&mod2);} else mod2.bs=0;
          if (mb->Ena_bs3) {mod3.xin = sin; Modulator(&mod3);} else mod3.bs=0;
          if (mb->Ena_bs4) {mod4.xin = sin; Modulator(&mod4);} else mod4.bs=0;

          BS2mb();
         
          
          
          
  }


           // Check 
/*          if (ii < 65536){
            ii++;
            printf(" %d \n\r" , mod0.bs);
           //if (CIC(2, mod2.bs)) { printf(" %d \n\r" ,mb->CIC2); } 
          }
*/ 

/*
void Sinus_Gen(void) 
{// not used anymore
  uint32_t i; 
  float s;  
  
  for (i = 0; i < N; i++) {
  s = amp*sin(i*T*2*pi*fain/M);

  }

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
