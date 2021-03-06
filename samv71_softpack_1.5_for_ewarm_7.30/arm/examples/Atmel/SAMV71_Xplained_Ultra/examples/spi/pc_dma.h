#ifndef _PC_DMA_H
#define _PC_DMA_H

/*----------------------------------------------------------------------------
 *        new jjk
 *----------------------------------------------------------------------------*/
   /** transfer complete callback. */

#define DMA_PMC_POS 26

typedef void (*dmaCallback)(uint32_t, void *);
   
   
   typedef struct {////jjk
  	/** Pointer to the Rx data. */
	uint32_t *pRxBuffA;
	/** Pointer to the Rx data. */
	uint32_t *pRxBuffB;
	/** Rx size in bytes. */
	uint32_t RxSize;
	/** Callback function invoked at the end of transfer. */
	dmaCallback callback;
	/** Callback arguments. */
	void *pArgument;
} DMAconfig; //two buffer A, B acquisition 
   

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

/** transfer complete callback. */
typedef void (*PcCallback)(uint8_t, void *);

/*----------------------------------------------------------------------------
 *       Struct
 *----------------------------------------------------------------------------*/

   
   /**  This structure is sent to the parallel capture function to start the transfer.
 * At the end of the transfer, the callback is invoked by the interrupt handler.
 */
typedef struct {
	/** Pointer to the Rx data. */
	uint32_t *pRxBuff;
	/** Rx size in bytes. */
	uint32_t RxSize;
	/** Callback function invoked at the end of transfer. */
	PcCallback callback;
	/** Callback arguments. */
	void *pArgument;
} PcCmd;


/** Constant structure associated with pio port. This structure prevents
    client applications to have access in the same time. */
typedef struct {
	/** Pointer to parallel capture Hardware registers */
	Pio *pPcHw;
	/** Current SpiCommand being processed */
	PcCmd *pCurrentCommand;
	/** Pointer to DMA driver */
	sXdmad *pXdmad;
	/** parallel capture Id as defined in the product datasheet */
	uint8_t pcId;
	/** Mutual exclusion semaphore. */
	volatile int8_t semaphore;
} PcDma;



/*------------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/
#define PC_OK          0
#define PC_ERROR       1
#define PC_ERROR_LOCK  2
/*------------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/
extern uint32_t Pc_ConfigureDma(PcDma *pPc ,
			         Pio *pPcHw ,
				uint8_t PcId,
				sXdmad *pXdmad);

extern uint32_t Pc_SendData(PcDma *pPc, PcCmd *pCommand);

void DMA_Buffer_cfg( uint32_t *PbufferA, uint32_t *PbufferB, uint32_t size);
void DMA_PIO_cfg(void);
bool DMA_Status(void);
bool DMA_Start(void);
void DMA_Enable(void);
void XDMAC_Handler(void);
#endif /* ! _PC_DMA_H */
