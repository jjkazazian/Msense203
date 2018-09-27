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
    
    int32_t D0[4];
    int32_t D1[4];
    int32_t D2[4];
    int32_t D3[4];
    int32_t Fsync[4];
    
    uint32_t idx; // buffer size counter

    bool repeat;      // stop the program
    uint32_t count;   // nb of samples = count*BSIZE
    
    bool dsp_compute;   // start DSP computation
    bool buffer_full;   // Bits stream buffer full
    bool buffer_print;  // allow to print bs on uart
    uint8_t next_state;     // truth output table   {0,1,2,3.. ,31} 
    
  } __attribute__((packed)) ;


void Init_state(void);
uint32_t Truth_table(void);
void Next_state(void);
void Print_int8_to_bin(uint8_t k);
void waitKey(void);
void Main_Config(void);



#endif /* ! _MAIN_CONFIG_H */
