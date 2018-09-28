#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#define BSIZE 1
#define SAMPLES_NUMBER 32

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
#define PIN_Fsync  {PIO_PA3,  PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}

/** List of all mux output definitions. */
#define PINS_MUXout  {PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_Fsync}
/** Number of muxout pins */
#define MUX_NUM  5

void Init_state(void);
uint32_t Truth_table(void);
void Next_action(void);
uint32_t MUX_Set(uint32_t dw);
uint32_t MUX_Clear(uint32_t dw);
void MUX_ctrl(uint32_t pinnb, bool level);
void Print_int8_to_bin(uint8_t k);
void waitKey(void);
void Main_Config(void);



#endif /* ! _MAIN_CONFIG_H */
