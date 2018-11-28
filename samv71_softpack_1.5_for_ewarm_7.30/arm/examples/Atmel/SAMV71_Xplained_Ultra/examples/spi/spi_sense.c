/* jj kazazaian 2018*/
/*
The SPI Controller is an interface between an SPI port and a register bank of up to 128
read/write 8 bits registers.


A transfer occurs when SCSB signal is low. The incoming stream on MOSI is decoded on sclk
negedge.
The first received bit indicates the operation direction (0 for a write and 1 for a read).
The 7 following bits contain the address of the register to read or write.
The following bytes are data which are either emitted on the MISO line in case of a read operation
or decoded on the MOSI line in case of a write operation.
The first data address corresponds the the first decoded address. The address pointer is then
incremented each time a new byte is read or written.
The operation ends when SCSB goes high.
If SCSCB goes high before the end of a byte transfer, the current byte operation is cancelled.
In case of a read operation, no more data are sent on the MISO line and in case of a write
operation, nothing is written into the currently decoded adress. All previous byte operations
are valid.
*/
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "spi_sense.h"


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/


 uint8_t secu = 0xA0|0x01;


volatile uint16_t registers[128];  // image of the Msense registers 
struct _SPI spibox; 


/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/


static void choose_SPI(void) 
{
            spibox.cs      = SPI0_CS3; // 3
            spibox.id      = ID_SPI0;  // 21
            spibox.spi     = SPI0;     // ((Spi    *)0x40008000U)           
}

static void Init_SPI(void) 
{
     /** SPI Clock setting (Hz) */
    static uint32_t spiClock;

    /** SPI clock configuration */
    static const uint32_t clockConfigurations[3] = { 8000000, 2000000, 4000000};

        /** Pins to configure for the application. */
        static const Pin spi_pins[] = {
                PIN_SPI_MISO,
                PIN_SPI_MOSI,
                PIN_SPI_SPCK,
                PIN_SPI_NPCS3
        };

        choose_SPI();  
        
        spiClock = clockConfigurations[0];
        
        // Enable the SPI clock
        PMC_EnablePeripheral(spibox.id) ;

        // Configure SPI in Master Mode with CS selected !!! 
        SPI_Configure(spibox.spi, spibox.id, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS(spibox.cs));
        
        SPI_ConfigureNPCS(spibox.spi, spibox.cs, SPI_SCBR( spiClock, BOARD_MCK));
      
        // 16 bits mode transfer
        spibox.spi->SPI_CSR[spibox.cs] |=  SPI_CSR_BITS_16_BIT; 
        
        // spi->SPI_CSR[dwNpcs] |= SPI_CSR_DLYBS(1);
        // spi->SPI_CSR[dwNpcs] |= SPI_CSR_DLYBCT(32);
        
        // Enable the SPI 
         SPI_Enable(spibox.spi) ; 
         PIO_Configure(spi_pins, PIO_LISTSIZE(spi_pins)); 
}

static void Print_Registers(uint8_t n) {
  uint32_t i;
  for (i = 0; i < n; i++) {
              printf(I"Register [0x%x] = \t0x%04x ", i, registers [(uint8_t)i]);  
              Print_int8_to_bin((uint8_t)registers [(uint8_t)i]);
              printf("\n\r");
  }
}




static bool check_RDRF(Spi *spi) {
        bool     wait=true;
        bool     status=false;
        uint8_t  rdrf;
        uint32_t n=0;
        
while (wait){
                   n++;
                   rdrf = spi->SPI_SR & SPI_SR_RDRF; 
                   if (n>1) {
                          if (rdrf == 0) wait=true; else wait=false;
                   } 
}
 if  (rdrf == 0) status=false; else status=true;      
        
return status;

}
static uint8_t Sense_Read(uint8_t addr)
{// jjk
        uint16_t data;
        uint16_t wdata;
        bool  wait;
        bool  status;
        uint32_t n=0;
        
        wait  = true;  
        wdata = (addr|0x80)<< 8 | (0xff); // 1+addr+data

  	while ((spibox.spi->SPI_SR & SPI_SR_TXEMPTY) == 0);	
        data = spibox.spi->SPI_RDR;   // to clear the RDRF bit
        spibox.spi->SPI_TDR = wdata | SPI_PCS(spibox.cs); // write address
        while ((spibox.spi->SPI_SR & SPI_SR_TDRE) == 0);   
       // while ((spibox.spi->SPI_SR & SPI_SR_TXEMPTY) == 0);	 
        
        if (check_RDRF(spibox.spi)) data = spibox.spi->SPI_RDR; // read data
        registers[addr] = 0x0000;      // reset register cell
        registers[addr] = (addr)<< 8 | (uint8_t)data; // 0+addr+data  

        return data;  
}



static void Sense_Write( uint8_t addr, volatile  uint8_t Data)
{// jjk
  uint16_t data = 0;

  data = (addr&0x7F)<< 8 | Data;
 
        while ((spibox.spi->SPI_SR & SPI_SR_TXEMPTY) == 0);	
        spibox.spi->SPI_TDR = data | SPI_PCS(spibox.cs);   // Write address+data
        while ((spibox.spi->SPI_SR & SPI_SR_TDRE) == 0);
        
  
        printf(I"WRITE sense register [ addr/data ]=0x%02x\n\r",  data);	
        
}

static void Analog_Config(void)
{
         // clear register
        Sense_Write(Addr(ANA_CTRL), 0x0);
         // write new config
        Sense_Write(Addr(ANA_CTRL), Sense_Read(Addr(ANA_CTRL)) | ANA_CTRL_ONLDO);
        Wait(1);
        Sense_Write(Addr(ANA_CTRL), Sense_Read(Addr(ANA_CTRL)) | ANA_CTRL_ONREF);
        Wait(1);
        Sense_Write(Addr(ANA_CTRL), Sense_Read(Addr(ANA_CTRL)) | ANA_CTRL_ONBIAS);
        Wait(1); 
        
}

static void Set_Channels(void)
{
        Sense_Write(Addr(SDI0),  ONADC | GAIN_ADC_GAINX1 | SDI0_TEMPMEAS);
        Sense_Write(Addr(SDI1),  0 | GAIN_ADC_GAINX1);
        Sense_Write(Addr(SDV1),  0 );
        Sense_Write(Addr(SDI2),  0 | GAIN_ADC_GAINX1);
        Sense_Write(Addr(SDV2),  0 );
}

void Sense_Reset_at(uint32_t r)
{
        // Soft reset
        spibox.addr  = REG_SOFT_NRESET;
        spibox.data  = r;
        Sense_Write(spibox.addr, spibox.data);
}


void  Sense_Config(void) {

        Init_SPI(); 
        // Soft reset
        spibox.addr  = Addr(SOFT_NRESET);
        spibox.data  = 0x0f;
        Sense_Write(spibox.addr, spibox.data);

        // Analog configuration , VDDANA, VREF, VBIAS
        Analog_Config();
        Set_Channels();
}        
        


void  Sense_Dump_param(void){
  /* access to the sense registers */  

        Sense_Read(Addr(ADCV1_TAG)); 
        Sense_Read(Addr(ADCV2_TAG));      
        Sense_Read(Addr(ADCV3_TAG));          
        Sense_Read(Addr(ANA_CTRL)); 
        Sense_Read(Addr(SDI0));
        Sense_Read(Addr(SDI1));
        Sense_Read(Addr(SDV1));
        Sense_Read(Addr(SDI2));
        Sense_Read(Addr(SDV2));  
        Sense_Read(Addr(SOFT_NRESET));  
       
        Print_Registers(46);     
     
}  
        
        
/** \endcond */
/* ---------------------------------------------------------------------------- */
/*                  Microchip Microcontroller Software Support                  */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2018, Microchip Corporation                                    */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */
