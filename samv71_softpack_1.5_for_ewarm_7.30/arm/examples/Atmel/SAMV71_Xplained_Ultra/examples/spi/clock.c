/* jj kazazaian 2018*/
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "include.h"
#include "main_config.h"
#include "clock.h"
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pin PCK0 (PIO_PD13B_PCK0 Peripheral C) */
const Pin pinPCK = PIN_PCK0;

/** Clock backup values */
static uint32_t masterClk;
static uint32_t pllaClk;
static uint32_t masterClkDivision;
static uint32_t pllMultiplier;
static uint32_t masterClkPrescaler;

/*----------------------------------------------------------------------------
 *        module functions
 *----------------------------------------------------------------------------*/

void Clock_Config(void)
{
	/* Configure PCK as peripheral */
	PIO_Configure(&pinPCK, 1);
	TimeTick_Configure();

	/* Configure PCK0 as peripheral */
        //waitKey();
        CalcPmcParam();
	Configure_Pck0(PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES(2));
	printf("\n\r --- Current PMC clock from start-up configuration --- \n\r");
	DumpPmcConfiguration();
}

void DumpPmcConfiguration(void)
{
	uint8_t ucChar[5];
	printf("\n\r========================= PMC ==========================\n\r" );
	printf(" Main clock\n\r");
	ucChar[0] = ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == CKGR_MOR_MOSCSEL) ?
			' ' : 'X';
	ucChar[1] = ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == CKGR_MOR_MOSCSEL) ?
			'X' : ' ';
	printf("\n\r    On-Chip 12MHz RC oscillator [%c] \n\r", ucChar[0]);
        printf("    12MHz crystal oscillator    [%c] \n\r", ucChar[1]);
	printf("\n\r    PLLA Multiplier =   %u  \n\r",  (unsigned int)pllMultiplier);
        printf("    PLLA clock      =   %u MHz\n\r",(unsigned int)(pllaClk/1000000));
        
	printf("\n\r Master clock\n\r");
	ucChar[0] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_SLOW_CLK) ?
			'X' : ' ';
	ucChar[1] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk ) == PMC_MCKR_CSS_MAIN_CLK) ?
			'X' : ' ';
	ucChar[2] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK) ?
			'X' : ' ';
	ucChar[3] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_UPLL_CLK) ?
			'X' : ' ';
	printf("\n\r    SLOW_CLK [%c]\n\r    MAIN_CLK [%c]\n\r    PLLA_CLK [%c]\n\r    UPLL_CLK [%c]\n\r",
			ucChar[0],ucChar[1],ucChar[2],ucChar[3]);
	printf("\n\r    masterClkPrescaler= %u \n\r    Master clock=       %uMHz \n\r",
			(unsigned int)masterClkPrescaler, (unsigned int)masterClk/1000000);
        
	printf("\n\r Programmable clock\n\r");
	ucChar[0] = ((PMC->PMC_PCK[0] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_SLOW_CLK) ?
			'X' : ' ';
	ucChar[1] = ((PMC->PMC_PCK[0] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_MAIN_CLK) ?
			'X' : ' ';
	ucChar[2] = ((PMC->PMC_PCK[0] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_PLLA_CLK) ?
			'X' : ' ';
	ucChar[3] = ((PMC->PMC_PCK[0] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_UPLL_CLK) ?
			'X' : ' ';
	ucChar[4] = ((PMC->PMC_PCK[0] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_MCK) ?
			'X' : ' ';
	printf("\n\r    SLOW_CLK [%c]\n\r    MAIN_CLK [%c]\n\r    PLLA_CLK [%c]\n\r    UPLL_CLK [%c]\n\r     MCK_CLK [%c]\n\r",
			ucChar[0], ucChar[1], ucChar[2], ucChar[3], ucChar[4]);
        printf("\n\r-I- external 4MHz main clock on PCK0 ...\n\r");
	printf("==============================================================\n\r");
}

/*
 *
 * \param css  The master clock divider source.
 * \param pres Master Clock prescaler.
 * \param clk frequency of the master clock (in Hz).
 */
void Configure_Pck0(uint32_t css, uint32_t pres)
{
	/* Disable programmable clock 1 output */
	REG_PMC_SCDR = PMC_SCER_PCK0;
	/* Enable the DAC master clock */
	PMC->PMC_PCK[0] = css | pres;
	/* Enable programmable clock 1 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY1 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0 );
	Wait(50);
}


void CalcPmcParam(void)
{
	uint32_t onChipRC;
	if (PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) {
		pllMultiplier = (PMC->CKGR_PLLAR & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos;
		pllaClk = BOARD_MAINOSC * (pllMultiplier + 1);
		switch (PMC->PMC_MCKR & PMC_MCKR_MDIV_Msk) {
		case PMC_MCKR_MDIV_EQ_PCK:
			masterClkDivision = 1;
			break;
		case PMC_MCKR_MDIV_PCK_DIV2:
			masterClkDivision = 2;
			break;
		case PMC_MCKR_MDIV_PCK_DIV4:
			masterClkDivision = 4;
			break;
		case PMC_MCKR_MDIV_PCK_DIV3:
			masterClkDivision = 3;
			break;
		default:
			masterClkDivision = 3;
		}
		masterClkPrescaler = 1 << ((PMC->PMC_MCKR & PMC_MCKR_PRES_Msk) >>
					PMC_MCKR_PRES_Pos);
		masterClk = pllaClk / masterClkDivision / masterClkPrescaler;
	} else {
		onChipRC = (PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk);
		masterClk = ((onChipRC == CKGR_MOR_MOSCRCF_12_MHz)? 12 :
				((onChipRC == CKGR_MOR_MOSCRCF_8_MHz) ? 8 : 4)) * 1000000;
	}
}


/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Microchip Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
