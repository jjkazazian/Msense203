#ifndef _CAPTURE_H
#define _CAPTURE_H


/*----------------------------------------------------------------------------
 *       Defines
 *----------------------------------------------------------------------------*/
#define sync_pos    4       
#define sync_msk    (0x1u << sync_pos) 
   
#define PIO_PCIDR_ALL (0xFu << 0)

struct _UNPACK  {
      uint32_t i; 
      uint32_t kase;
      uint64_t Lword;
      uint8_t  predata[4];
      uint32_t csum;  // Check sum
      uint8_t  sync;
      bool     synchronized;
      bool     dmx_full;
      bool     status; // no error 
    
  } __attribute__((packed)) ;
      
      
      
      
void Print_Buffer(int32_t * buf);
void Reset(void);
void PIO_Generation(void);  
void Capture_Config(Pio *pio);
void Unpack_64b_bs0(uint32_t *in);
void PIO_Capture_DMA(void);
void Enable_Capture(void);
void Disable_Capture(void);
void PIO_synchro_polling(void);
void PIO_DMA_firstbuffer(void);
   
bool Unpack(uint32_t *in);
void Unpack_bs0(uint32_t *in);
void _pc_dmaTransfer(uint32_t *Pbuffer);


#endif /* ! _CAPTURE_H */
