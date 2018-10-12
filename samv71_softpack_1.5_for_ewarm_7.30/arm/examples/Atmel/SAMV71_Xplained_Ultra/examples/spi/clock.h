#ifndef _CLOCK_H
#define _CLOCK_H





void Clock_Config(void);
void DumpPmcConfiguration(void);
void Configure_Pck0(uint32_t css, uint32_t pres);
void CalcPmcParam(void);


#endif /* ! _CLOCK_H */
