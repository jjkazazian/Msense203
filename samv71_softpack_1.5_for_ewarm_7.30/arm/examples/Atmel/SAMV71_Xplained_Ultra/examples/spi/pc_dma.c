/* jj kazazaian 2018*/
//PIOA parallel capture with DMA

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "pc_dma.h"

/*  DMA driver instance */
static uint32_t pcDmaRxChannel;
static bool dma_initialized =true;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief PcxDMA Rx callback
 * Invoked on Pc DMA reception done.
 * \param channel DMA channel.
 * \param pArg Pointer to callback argument - Pointer to PcDma instance.
 */
static void Pc_Rx_Cb(uint32_t channel, PcDma *pArg)
{
	PcCmd *pPcCmd = pArg->pCurrentCommand;

	if (channel != pcDmaRxChannel)
		return;

	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_DisableIRQ(XDMAC_IRQn);

	/* Release the DMA channels */
	XDMAD_FreeChannel(pArg->pXdmad, pcDmaRxChannel);
	SCB_InvalidateDCache_by_Addr(pPcCmd->pRxBuff, pPcCmd->RxSize);
	/* Release the dataflash semaphore */
	pArg->semaphore++;

	/* Invoke the callback associated with the current command */
	if (pPcCmd && pPcCmd->callback)
		pPcCmd->callback(0, pPcCmd->pArgument);
}

/**
 * \brief Configure the DMA Channels: 0 RX.
 * Channels are disabled after configure.
 * \param pXdmad Pointer to a PcDma instance
 * \returns 0 if the dma channel configuration successfully; otherwise returns
 * PC_ERROR_XXX.
 */
static uint8_t _PcConfigureDmaChannels(PcDma *pPc)
{

	/* Driver initialize */
  if (dma_initialized) {
    XDMAD_Initialize(pPc->pXdmad, 0);
    
    dma_initialized = false;
  } else {
        pPc->pXdmad->pXdmacs = XDMAC;
	pPc->pXdmad->pollingMode = 0;
	pPc->pXdmad->numControllers = XDMAC_CONTROLLER_NUM;
	pPc->pXdmad->numChannels    = (XDMAC_GTYPE_NB_CH(XDMAC_GetType(XDMAC)) + 1);

  }
	XDMAD_FreeChannel(pPc->pXdmad, pcDmaRxChannel);

	/* Allocate a DMA channel for PIOA RX. */
	pcDmaRxChannel =
		XDMAD_AllocateChannel(pPc->pXdmad, pPc->pcId, XDMAD_TRANSFER_MEMORY);

	if (pcDmaRxChannel == XDMAD_ALLOC_FAILED)
		return PC_ERROR;

	/* Setup callbacks for PC RX */
	XDMAD_SetCallback(pPc->pXdmad, pcDmaRxChannel,
					  (XdmadTransferCallback)Pc_Rx_Cb, pPc);

	if (XDMAD_PrepareChannel(pPc->pXdmad, pcDmaRxChannel))
		return PC_ERROR;

	return PC_OK;
}

/**
 * \brief Configure the DMA source and destination with Linker List mode.
 * \param pXdmad   Pointer to a PcDma instance
 * \param pCommand Pointer to PcCmd instance
 * \param PcCmd    Pointer to command
 */

static uint8_t _Pc_configureLinkList(Pio *pPcHw, void *pXdmad, PcCmd *pCommand)
{
	uint32_t xdmaCndc, xdmaInt;
	sXdmadCfg xdmadRxCfg;
	uint32_t pcId;

	if ((unsigned int)pPcHw == (unsigned int)PIOA) pcId = ID_PIOA; /////////////////////

	/* Setup RX Link List */
	xdmadRxCfg.mbr_ubc = XDMA_UBC_NVIEW_NDV0 |
						 XDMA_UBC_NDE_FETCH_DIS |
						 XDMA_UBC_NDEN_UPDATED |
						 pCommand->RxSize;

	xdmadRxCfg.mbr_da = (uint32_t)pCommand->pRxBuff;
	xdmadRxCfg.mbr_sa = (uint32_t) &pPcHw->PIO_PCRHR;  
	xdmadRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
						 XDMAC_CC_MBSIZE_SINGLE |
						 XDMAC_CC_DSYNC_PER2MEM |
						 XDMAC_CC_CSIZE_CHK_1 |
						 XDMAC_CC_DWIDTH_BYTE |
						 XDMAC_CC_SIF_AHB_IF1 |
						 XDMAC_CC_DIF_AHB_IF0 |  // jjk
						 XDMAC_CC_SAM_FIXED_AM |
						 XDMAC_CC_DAM_INCREMENTED_AM |
						 XDMAC_CC_PERID(
						 XDMAIF_Get_ChannelNumber(pcId, XDMAD_TRANSFER_RX));

	xdmadRxCfg.mbr_bc = 0;
	xdmadRxCfg.mbr_sus = 0;
	xdmadRxCfg.mbr_dus = 0;

	xdmaInt =  (XDMAC_CIE_BIE   |
				XDMAC_CIE_DIE   |
				XDMAC_CIE_FIE   |
				XDMAC_CIE_RBIE  |
				XDMAC_CIE_WBIE  |
				XDMAC_CIE_ROIE);
	xdmaCndc = 0;

	if (XDMAD_ConfigureTransfer(pXdmad, pcDmaRxChannel,
				&xdmadRxCfg, xdmaCndc, 0, xdmaInt))
		return PC_ERROR;

	return PC_OK;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/


/**
 * \brief Initializes the PcDma structure and the corresponding Pc & DMA .
 * hardware select value.
 * The driver will uses DMA channel 0 for RX .
 * The DMA channels are freed automatically when no DMA command processing.
 *
 * \param pPc    Pointer to a PcDma instance.
 * \param pPcHw  Associated Pc peripheral.
 * \param PcId   Pc peripheral identifier.
 * \param pDmad  Pointer to a Dmad instance.
 */
uint32_t Pc_ConfigureDma(PcDma *pPc ,
					Pio *pPcHw ,
					uint8_t PcId,
					sXdmad *pXdmad)
{
	/* Initialize the parallel capture structure */
	pPc->pPcHw = pPcHw;
	pPc->pcId  = PcId;
	pPc->semaphore = 1;
	pPc->pCurrentCommand = 0;
	pPc->pXdmad = pXdmad;
	return 0;
}

/**
 * \brief Starts a Pc transfer. This is a non blocking function. It will
 *  return as soon as the transfer is started.
 *
 * \param pPc  Pointer to a PcDma instance.
 * \param pCommand Pointer to the parallel capture command to execute.
 * \returns 0 if the transfer has been started successfully; otherwise returns
 * PC_ERROR_LOCK is the driver is in use, or PC_ERROR if the command is not
 * valid.
 */
uint32_t Pc_SendData(PcDma *pPc, PcCmd *pCommand)
{
	Pio *pPcHw = pPc->pPcHw;

	/* Try to get the dataflash semaphore */
	if (pPc->semaphore == 0)

		return PC_ERROR_LOCK;

	pPc->semaphore--;

	// Initialize the callback
	pPc->pCurrentCommand = pCommand;

	/* Initialize DMA controller using channel 0 for RX. */
	if (_PcConfigureDmaChannels(pPc))
		return PC_ERROR_LOCK;

	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn , 1);
	NVIC_EnableIRQ(XDMAC_IRQn);

	if (_Pc_configureLinkList(pPcHw, pPc->pXdmad, pCommand))
		return PC_ERROR_LOCK;

	  ///////////////////////////////////////////////////

	/* Start DMA 0(RX) */
	if (XDMAD_StartTransfer(pPc->pXdmad, pcDmaRxChannel))
		return PC_ERROR_LOCK;

	return PC_OK;;
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
