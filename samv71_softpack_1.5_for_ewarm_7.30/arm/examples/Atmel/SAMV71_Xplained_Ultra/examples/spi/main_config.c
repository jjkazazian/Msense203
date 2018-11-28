/* jj kazazaian 2018 */
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"

   
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
static uint32_t dbg_baudrate = 115200;
extern MAILBOX *mb; 
//extern MAILBOX mbox; 

//Pin for muxout
extern const Pin mux_pins[]       = PINS_MUXout;
extern const Pin capture_pins[]   = PINS_capture;
        
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
void waitKey(void)
{
	printf("\n\r-I- Press any key to Continue...\n\r");
	while (1) {
		if (DBG_GetChar()!= 0)
			break;
	}
}
static unsigned int_to_int(unsigned k) {
    if (k == 0) return 0;
    if (k == 1) return 1;                       /* optional */
    return (k % 2) + 10 * int_to_int(k / 2);
}

void Print_int8_to_bin(uint8_t k)
{ // 8 bits only
  
  printf("Code= %08d", (k == 0 || k == 1 ? k : ((k % 2) + 10 * int_to_int(k / 2))));
}

void Print_byte(char *a, uint8_t b)
//Syntax is Print_byte("toto", value);
{ 
printf("%s ",a); 
Print_int8_to_bin(b); 
printf("\r\n");

}



double randone(void) 
{//randf= +/-1
  double  rdf;  
  int32_t r1;
  uint32_t max_rd; 
  r1 = rand()-rand();
  max_rd = MAX(max_rd,abs(r1));
  rdf = (double)r1/(double)max_rd;
  return rdf;
}



void Init_state(void) {
mb->dsp_compute    = false;
mb->buffer_print   = false;
mb->buffer_full    = false;
}

uint32_t Truth_table(void) {
  int i; 
  uint8_t val[8] = {0,1,2,3,4,5,0,0}; 
  i  =  (int)mb->dsp_compute *4+
        (int)mb->buffer_print*2+
        (int)mb->buffer_full *1;
 mb->next_state = val[i];
 return i;
 /*
dsp_compute	
        buffer_print	
                buffer_full	
                        State/next value
0	0	0	0 // init state, will activate the dsp
0	0	1	1 // DSP stoped after buffer is full
0	1	0	2 // buffer transmitted
0	1	1	3 // print bs to uart, end of uart, go to zero state
1	0	0	4 // dsp next step computation
1	0	1	5 // dsp computation done, bs buffer is full, will stop the dsp
1	1	0	6
1	1	1	7
*/
}
void Next_action(void) {
  uint32_t state;
  uint32_t i;
  
  state = Truth_table(); 


  //printf("  %d %d %d  state: %d next: %d  \r\n", mb.dsp_compute, mb.buffer_print, mb.buffer_full, state, mb.next_state);
  
  switch (mb->next_state) {
		case 0: //  reset all bool to init value
                       mb->dsp_compute = true;
		break;               
		case 1:  // do nothing
                        Init_state();
                break;        
		case 2: // print bs buffer to io
                       //for (i = 0; i < BSIZE; i++) printf ("  %d \r\n", mb.BS1[i]);
                  
                       //DSP();
                       BS_2_IO(); 
                       mb->count++;
                       
                       mb->buffer_print = false;
		break;            
                case 3: // do nothing      
                       Init_state();
		break;   
                case 4: // dsp next step computation
                       
                       DSP();
                       mb->buffer_full =  true;
                      
                       
                break;
                case 5: // stop dsp
                       mb->dsp_compute = false;
                       mb->buffer_print = true;
                       mb->buffer_full  = false;
                break;
                case 6: // do nothing
                       Init_state(); 
                break;
                case 7: // do nothing
                       Init_state(); 
                break;        
                  
		default: // do nothing
                        Init_state();
		break;
		} 
}
 

/**
 *  Turns the given mux on if it exists; otherwise does nothing.
 *  \param mux  Number of the D0,1,2,3, Fsync to turn on.
 *  \return 1 if the pin has been turned on; 0 otherwise.
 */


void IO_ctrl(uint32_t pinnb, bool level)
{
      if (level) PIO_Set(&mux_pins[pinnb]); else PIO_Clear(&mux_pins[pinnb]);
}

void IO_set(uint32_t pinnb)
{
 PIO_Set(&mux_pins[pinnb]);      
}
void IO_clear(uint32_t pinnb)
{
 PIO_Clear(&mux_pins[pinnb]);      
}


bool IO_get_sync(void)
{
 uint32_t c;
 c = (PIOA->PIO_PDSR & PIO_PDSR_P10) >> 10;
 if (c==0) return false; else return true;
}

static void Memory_Config_TCM(MAILBOX *pmb){
 /* 384kbyte   = 96000 words of 32 bits*/

  
  uint32_t mb_size;
  uint32_t limit;
  uint32_t *pDestmb;
  
  mb_size = sizeof(MAILBOX);
  limit = (uint32_t)pmb + (uint32_t)mb_size;
  printf("\r\n TCM \r\n");
  printf("        Malloc Mailbox address        = %x   \r\n", (uint32_t)pmb);
  printf("        address limit Mailbox buffer  = %x   \r\n", (uint32_t *)limit); 
  printf("       (bytes) Malloc Mailbox size    = %d   \r\n", mb_size);

  printf("       (bytes) A,B,C buffers          = %d   \r\n", SAMPLES_NUMBER*4*2+CIC_NUMBER*BUFFER_NUMBER); 
       //  fill with 0xDEADFACE Pattern 
       for (pDestmb = (uint32_t *)pmb; pDestmb < (uint32_t *)limit; pDestmb++) 
       {
       *pDestmb = 0xDEADFACE;
       }
  printf("\r\n");
  
}
static void Memory_Config(MAILBOX *pmb){
 
  uint32_t mb_size;
  uint32_t limit;
  uint32_t *pDestmb;
  
  mb_size = sizeof(MAILBOX);
  limit = (uint32_t)&pmb + (uint32_t)mb_size;
  
  printf("\r\n RAM \r\n");
  printf("        Malloc Mailbox address        = %x   \r\n", (uint32_t)&pmb);
  printf("        address limit Mailbox buffer  = %x   \r\n", (uint32_t *)limit); 
  printf("       (bytes) Malloc Mailbox size    = %d   \r\n", mb_size);

  printf("       (bytes) A,B,C buffers          = %d   \r\n", SAMPLES_NUMBER*4*2+CIC_NUMBER*BUFFER_NUMBER); 
       //  fill with 0xDEADFACE Pattern 
       for (pDestmb = (uint32_t *)pmb; pDestmb < (uint32_t *)limit; pDestmb++) 
       {
       *pDestmb = 0xDEADFACE;
       }
  printf("\r\n");



  
}

void Main_Config(void)
{
  /*
       uint32_t *pDest;
  
         // fill Stack with 0xFEEDFACE Pattern 
       for (pDest = (uint32_t *)ASTACK; pDest < (uint32_t *)(ASTACK+SSTACK);) 
       {
              *pDest++ = 0xFEEDFACE;
       }
*/
       /* Enable I and D cache */
	SCB_EnableICache();
	//SCB_EnableDCache(); // if activated: need static cluster optimization 
   
	/* Disable watchdog */
	WDT_Disable(WDT);
        
        /* DBG UART */
        DBG_Configure(dbg_baudrate, BOARD_MCK);
         
	/* Output example information */
	printf("\n\r--- SPI Example %s --\n\r", SOFTPACK_VERSION);
	printf("--- %s\n\r", BOARD_NAME);
	printf("--- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);
        
        // Pin for muxout
        PIO_Configure(mux_pins, PIO_LISTSIZE(mux_pins)); 
        // Pin for capture
        PIO_Configure(capture_pins, PIO_LISTSIZE(capture_pins)); 
        PIOA->PIO_PPDDR |= PIO_PPDDR_P3; // board pullup
        PIOA->PIO_PPDDR |= PIO_PPDDR_P4; // board pullup
        PIOA->PIO_PPDER |= PIO_PPDER_P5;
        PIOA->PIO_PPDER |= PIO_PPDER_P9;
        PIOA->PIO_PPDER |= PIO_PPDER_P10;
        
    //    Memory_Config_TCM(&mbox);
        Memory_Config_TCM(mb);
        
        mb->count=0;
        printf("--- End of Memory configuration \n\r");
        
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
