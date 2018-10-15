#ifndef _CAPTURE_H
#define _CAPTURE_H


/*----------------------------------------------------------------------------
 *       Defines
 *----------------------------------------------------------------------------*/
#define SIZE_BUFF_RECEPT   SAMPLES_NUMBER*4

void Capture_Config(Pio *pio);
void PIO_Print_Buffer(void); 
void PIO_Capture(void);
void Enable_Capture(void);
void Disable_Capture(void);
void _pc_dmaTransfer(void);
void PIO_Reset_Buffer(void);

#endif /* ! _CAPTURE_H */
