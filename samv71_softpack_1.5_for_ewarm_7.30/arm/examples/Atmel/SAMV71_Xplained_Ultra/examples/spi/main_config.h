#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "dsp_sense.h"
#include "mux.h"
#include "capture.h"

#define MIN(a,b) (((a)<(b))?(a):(b)) // minus   = MIN(mean, minus);
#define MAX(a,b) (((a)>(b))?(a):(b)) // maximus = MAX(mean, maximus); 

#define BUFFER_NUMBER     2                 // numbers of buffers to acquire
#define CICOSR           64                 // Comb filter decimation factor
#define CIC_NUMBER       107                 // sample number ofter CIC filter
#define SAMPLES_NUMBER   CIC_NUMBER*CICOSR  // numbers of signal Word samples of bitstream 
#define ND               SAMPLES_NUMBER*4   // numbers of byte acquisitions for the DMA

#define ASTACK 0x204000a0  // stack base address
#define SSTACK 0x1000      // stack size
//#define ENABLE_TCM // also made in IAR preprocessing
#define FFT_DEMO   //  128kB

COMPILER_PACK_SET(4)
typedef struct   {
  // Bit stream output from DSP modulator virtual msense 203
    volatile int8_t BS0;
    volatile int8_t BS1;
    volatile int8_t BS2;
    volatile int8_t BS3;
    volatile int8_t BS4;
    
    volatile int32_t CIC0;
    volatile int32_t CIC1;
    volatile int32_t CIC2;
    volatile int32_t CIC3;
    volatile int32_t CIC4;
    
    volatile uint8_t to_demux[4];
    volatile int8_t  to_bs[5];
    
    /** DMA PIO receive buffer. up to 8192 values, cannot be volatile*/
     uint32_t A[SAMPLES_NUMBER]; 
     uint32_t B[SAMPLES_NUMBER];
  
     int32_t CIC_C[CIC_NUMBER*BUFFER_NUMBER];
  
    
    //main loop and DMA control
  //  uint32_t idx;     // DSP decimation counter not used
    bool dmacall;       // dma interupt and callback done
    bool repeat;        // enable the signal generation
    uint32_t count;     // nb of samples counter
    bool dmaswitch;     // DMA switch 0 buffer A to 1 buffer B 
    uint32_t *Pab;      // pointer to buffer A or B
    
    // State machine
    bool cic_ready[5];   // start DSP computation
    bool dsp_compute;    // start DSP computation
    bool buffer_full;    // Bits stream buffer full
    bool buffer_print;   // allow to print bs on uart
    uint8_t next_state;  // truth output table {0,1,2,3.. ,31} 
    
  }  MAILBOX;

COMPILER_PACK_RESET()




#define PIN_D1     {PIO_PA6,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D0     {PIO_PD30, PIOD, ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D2     {PIO_PC19, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D3     {PIO_PA2,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_Fsync  {PIO_PC13, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_clkb   {PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_6      {PIO_PB3,  PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}
// // capture
#define PIN_CD0  {PIO_PA3,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD1  {PIO_PA4,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD2  {PIO_PA5,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD3  {PIO_PA9,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD4  {PIO_PA10, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //Fsync
#define PIN_pclk {PIO_PA22, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/** List of all mux output definitions. */
#define PINS_MUXout  {PIN_D0,  PIN_D1,  PIN_D2,  PIN_D3,  PIN_Fsync, PIN_clkb, PIN_6}
/** List of all PIO Capture */
#define PINS_capture {PIN_CD0, PIN_CD1, PIN_CD2, PIN_CD3, PIN_CD4,   PIN_pclk}

double randone(void) ;
void Init_state(void);
uint32_t Truth_table(void);
void Next_action(void);
uint32_t MUX_Set(uint32_t dw);
uint32_t MUX_Clear(uint32_t dw);
void IO_ctrl(uint32_t pinnb, bool level);
void IO_set(uint32_t pinnb);
void IO_clear(uint32_t pinnb);
void Print_int8_to_bin(uint8_t k);
void waitKey(void);
void Main_Config(void);
void Print_byte(char *a, uint8_t b);
void Memory_Config(MAILBOX *pmb);


#endif /* ! _MAIN_CONFIG_H */
