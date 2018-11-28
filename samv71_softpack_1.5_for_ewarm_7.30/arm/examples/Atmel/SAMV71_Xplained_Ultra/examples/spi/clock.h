#ifndef _CLOCK_H
#define _CLOCK_H





void Clock_Config(void);
void DumpPmcConfiguration(void);
void Configure_Pck0(uint32_t css, uint32_t pres);
void Configure_Pck0_onthefly(uint32_t css, uint32_t pres);
void Configure_Pck0_enable(void);
void Configure_Pck0_disable(void);
void CalcPmcParam(void);


#endif /* ! _CLOCK_H */
