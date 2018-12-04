#ifndef _DSP_SENSE_H
#define _DSP_SENSE_H




#define CICORDER 3  // Comb filter Filter order
#define NS 24       // output dynamic twos complement
#define NBS 3       // nb of bit of the SD modulator
#define BSBUFFER 32768       // bit stream buffer size
#define BSCLOCK  1024000     // bit stream clock Hz
#define M  1000000     // Mega
#define K  1000        // Kilo

struct _MOD  {
    int32_t xin;               // Input buffer
    int32_t bs;                // Bit stream buffer
    int32_t int1;              // Integrator 1 memory
    int32_t int2;              // Integrator 2 memory
    int32_t z;                 // unit delay memory
    
  } __attribute__((packed)) ;

struct _CIC  {
    uint32_t id;                // id of the structure 0 for bs0, 1 for bs1, ...  
    int32_t  xout;              // output signal
    uint32_t osr;               // osr counter
    int32_t int1;               // Integrator 1 memory
    int32_t int2;               // Integrator 2 memory
    int32_t int3;               // Integrator 3 memory
    int32_t der1;               // derivative 1 memory
    int32_t der2;               // derivative 2 memory
    int32_t der3;               // derivative 3 memory
    int32_t zder1;              // derivative 1 delay memory
    int32_t zder2;              // derivative 2 delay memory
    int32_t zder3;              // derivative 3 delay memory   
    bool flag_osr;              // true = CIC data ready       
  } __attribute__((packed)) ;    


void Sinus_Gen(void);
void DSP_Config(void);
void DSP(void);
void Average(int32_t * buf);
void Stdev(int32_t * buf);


bool CIC(uint32_t id, int32_t bs); 


#endif /* ! _DSP_SENSE_H */
