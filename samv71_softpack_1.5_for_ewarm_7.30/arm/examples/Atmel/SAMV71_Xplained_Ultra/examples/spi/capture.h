#ifndef _CAPTURE_H
#define _CAPTURE_H


/*----------------------------------------------------------------------------
 *       Defines
 *----------------------------------------------------------------------------*/

#define PIO_PCIDR_ALL (0xFu << 0)
   
void Capture_Config(Pio *pio);
   
void PIO_Print_Buffer(uint32_t *in); 
void PIO_Copy_Buffer(uint32_t *in, uint32_t *out);
void PIO_Clear_Buffer(uint32_t *in);

void PIO_Capture_DMA(void);
void Enable_Capture(void);
void Disable_Capture(void);
void _pc_dmaTransfer(void);


#endif /* ! _CAPTURE_H */
