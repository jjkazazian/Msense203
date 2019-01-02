#ifndef _CLOCK_H
#define _CLOCK_H


#define PWM_FREQUENCY           50
#define DUTY_CYCLE              50
/** Maximum duty cycle value. */
#define MAX_DUTY_CYCLE              100
/** Minimum duty cycle value. */
#define MIN_DUTY_CYCLE              0

/** Duty cycle buffer length for three channels */
#define DUTY_BUFFER_LENGTH          (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE )


void Clock_Config(void);
void DumpPmcConfiguration(void);
void Configure_Pck0(uint32_t css, uint32_t pres);
void Configure_Pck1(uint32_t css, uint32_t pres);
void Configure_Pck2(uint32_t css, uint32_t pres);
void Configure_Pck0_onthefly(uint32_t css, uint32_t pres);
void Configure_Pck0_enable(void);
void Configure_Pck0_disable(void);
void CalcPmcParam(void);


#endif /* ! _CLOCK_H */
