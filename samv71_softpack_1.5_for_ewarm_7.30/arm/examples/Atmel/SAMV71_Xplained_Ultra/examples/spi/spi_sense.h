#ifndef _SPI_SENSE_H
#define _SPI_SENSE_H

#include "applet.h"
#include "registers_name.h"


#define SPI0_CS0  0
#define SPI0_CS1  1
#define SPI0_CS2  2
#define SPI0_CS3  3



struct _SPI { 
         volatile uint8_t  addr;    
         volatile uint8_t  data;
                  uint32_t cs;
                  uint32_t id;
                  Spi      *spi;


};


void  Sense_Config(void);
void  Sense_Dump_param(void);
void Sense_Reset_at(uint32_t r);
void Set_Channels(void);


#endif /* ! _SPI_SENSE_H */
