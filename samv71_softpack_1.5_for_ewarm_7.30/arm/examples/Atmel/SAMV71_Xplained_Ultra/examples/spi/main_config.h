#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "dsp_sense.h"
#include "mux.h"
#include "capture.h"


#define BSIZE 1
#define SAMPLES_NUMBER 8




struct _MAILBOX  {
    int32_t BS0[BSIZE];
    int32_t BS1[BSIZE];
    int32_t BS2[BSIZE];
    int32_t BS3[BSIZE];
    int32_t BS4[BSIZE];
    /*
    int32_t D0[4];
    int32_t D1[4];
    int32_t D2[4];
    int32_t D3[4];
    int32_t Fsync[4];
    */
    uint32_t idx; // buffer size counter

    bool repeat;      // stop the program
    uint32_t count;   // nb of samples = count*BSIZE
    
    bool dsp_compute;   // start DSP computation
    bool buffer_full;   // Bits stream buffer full
    bool buffer_print;  // allow to print bs on uart
    uint8_t next_state; // truth output table {0,1,2,3.. ,31} 
    
  } __attribute__((packed)) ;

#define PIN_D1     {PIO_PA6,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D0     {PIO_PD30, PIOD, ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D2     {PIO_PC19, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_D3     {PIO_PA2,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_Fsync  {PIO_PC13, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN_clkb   {PIO_PA19,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
//#define PIN_clkb   {PIO_PB4, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}
// // capture
#define PIN_CD0  {PIO_PA3,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD1  {PIO_PA4,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD2  {PIO_PA5,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD3  {PIO_PA9,  PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define PIN_CD4  {PIO_PA10, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT} //Fsync
#define PIN_pclk {PIO_PA22, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT}

/** List of all mux output definitions. */
#define PINS_MUXout  {PIN_D0,  PIN_D1,  PIN_D2,  PIN_D3,  PIN_Fsync, PIN_clkb}
/** List of all PIO Capture */
#define PINS_capture {PIN_CD0, PIN_CD1, PIN_CD2, PIN_CD3, PIN_CD4,   PIN_pclk}

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



#endif /* ! _MAIN_CONFIG_H */
