#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "dsp_sense.h"
#include "mux.h"
#include "capture.h"


// #define BOARD_SAMV71_DVB in IAR setting overwise BOARD_SAMV71_XULT

#define I "<I> "
#define E "<E> "
#define R "\n\r"

//#define AUTOTEST     // Generate sinus and bitstream on muxed IO associate with BUFFOUT
#define BUFFOUT      // Store in an output buffer otherwise continuously sampling


#define MIN(a,b) (((a)<(b))?(a):(b)) // minus   = MIN(mean, minus);
#define MAX(a,b) (((a)>(b))?(a):(b)) // maximus = MAX(mean, maximus); 

#define BUFFER_NUMBER     4                  // numbers of buffers to acquire
#define CICOSR           64                  // Comb filter decimation factor
#define CIC_NUMBER       229                 // sample number after CIC filter
#define SAMPLES_NUMBER   CIC_NUMBER*CICOSR   // numbers of signal Word samples of bitstream 
#define ND               SAMPLES_NUMBER*4    // numbers of byte acquisitions for the DMA

/** type of measurement */
typedef enum _MEAS {
	meas_T ,
	meas_I1,
	meas_V1,
	meas_I2,
	meas_V2,
} measure;

//#define ENABLE_TCM //  made in IAR preprocessing
#define FFT_DEMO     //  floating point unit activated

COMPILER_PACK_SET(4)
typedef struct   {
  // Bit stream output from DSP modulator virtual msense 203
    volatile int8_t BS0;
    volatile int8_t BS1;
    volatile int8_t BS2;
    volatile int8_t BS3;
    volatile int8_t BS4;
  /*  to remove
    volatile int32_t CIC0;
    volatile int32_t CIC1;
    volatile int32_t CIC2;
    volatile int32_t CIC3;
    volatile int32_t CIC4;
*/    
    volatile int32_t CIC[5];
    volatile uint8_t to_demux[4];
    volatile int8_t  to_bs[5];
    
    /** DMA PIO receive buffers*/
     uint32_t A[SAMPLES_NUMBER]; 
     uint32_t B[SAMPLES_NUMBER];
     
    // Signal output buffer 
     int32_t CIC_C[CIC_NUMBER*BUFFER_NUMBER];
     double cic_avg;
     double cic_rms;
     double cic_snr;
  
   // Console
       bool Ena_bs0;  
       bool Ena_bs1;  
       bool Ena_bs2;  
       bool Ena_bs3;  
       bool Ena_bs4; 
       bool Ena_cic; 
       bool Ena_sin; 
       uint32_t View_bs;  
       uint32_t pos;
       uint32_t kos;
       
       uint8_t key;
       uint8_t prekey;
       bool console;     
     
     
  // main loop and DMA control
  //  uint32_t idx;     // DSP decimation counter not used
    bool dmacall;       // dma interupt and callback done
    bool repeat;        // enable the signal generation
    uint32_t count;     // nb of samples counter
    measure meas ;      // type of measurement
    bool buffer_switch; // DMA switch 0 buffer A to 1 buffer B 
    bool DMA_switch;    // DMA switch 0 buffer A to 1 buffer B 
    bool synchro;       // PIO synchro detected
    bool presync;       // PIO synchro previous instant value
    bool sync;          // PIO synchro instant value
    uint32_t *Pab;      // pointer to buffer A or B
    
    // State machine
    bool cic_ready[5];   // start DSP computation
    bool dsp_compute;    // start DSP computation
    bool buffer_full;    // Bits stream buffer full
    bool buffer_print;   // allow to print bs on uart
    uint8_t next_state;  // truth output table {0,1,2,3.. ,31} 
    
  }  MAILBOX;

COMPILER_PACK_RESET()


/** List of all mux output definitions. */
// mux out 
#ifndef	BOARD_SAMV71_DVB
        #define PIN_D0     {PIO_PD30, PIOD, ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_D1     {PIO_PA6,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_D2     {PIO_PC19, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_D3     {PIO_PA2,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_Fsync  {PIO_PC13, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_clkb   {PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PIN_6      {PIO_PB3,  PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

        #define PINS_MUXout  {PIN_D0,  PIN_D1,  PIN_D2,  PIN_D3,  PIN_Fsync, PIN_clkb, PIN_6}
#else
        #define PIN_6        {PIO_PA0,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
        #define PINS_MUXout  {PIN_6}
#endif

// capture
#define PIN_CD0  {PIO_PA3,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD1  {PIO_PA4,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD2  {PIO_PA5,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

#ifdef	BOARD_SAMV71_DVB
         #define PIN_CD3  {PIO_PA13, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
         #define PIN_CD4  {PIO_PA12, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //Fsync not connected
#else
         #define PIN_CD3  {PIO_PA9,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
         #define PIN_CD4  {PIO_PA10, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //Fsync
#endif    

#define PIN_pclk {PIO_PA22, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/** List of all PIO Capture */
#define PINS_capture {PIN_CD0, PIN_CD1, PIN_CD2, PIN_CD3, PIN_CD4, PIN_pclk}

double randone(void) ;
void Init_state(void);
uint32_t Truth_table(void);
void Next_action(void);
uint32_t MUX_Set(uint32_t dw);
uint32_t MUX_Clear(uint32_t dw);
void IO_ctrl(uint32_t pinnb, bool level);
void IO_set(uint32_t pinnb);
void IO_clear(uint32_t pinnb);
bool IO_get_sync(void);
bool IO_get_clk(void);
uint32_t local_GetChar(void);

void Print_int8_to_bin(uint8_t k);
void waitKey(void);
void Main_Config(void);
void Print_byte(char *a, uint8_t b);
void Memory_Config(MAILBOX *pmb);


#endif /* ! _MAIN_CONFIG_H */
