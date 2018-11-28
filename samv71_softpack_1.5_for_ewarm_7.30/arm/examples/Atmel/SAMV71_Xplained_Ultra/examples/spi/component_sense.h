
/* ============================================================================= */
/**  SOFTWARE API DEFINITION SENSE */
/* ============================================================================= */

/*@{*/

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief Sense hardware registers: MSB_MODE=0: 32 bits mode 
 __O    Write
 __I    Read
 __IO   Read/Write
*/
typedef struct {
  __I  uint8_t ADCI0_TAG;        /**< \brief (Sense Offset: 0x00) I0 tag register*/
  __I  uint8_t ADCI0_23_16;      /**< \brief (Sense Offset: 0x01) I0 adc bit 23 to 16 register*/
  __I  uint8_t ADCI0_15_8;       /**< \brief (Sense Offset: 0x02) I0 adc bit 15 to 8 register*/  
  __I  uint8_t ADCI0_7_0;        /**< \brief (Sense Offset: 0x03) I0 adc bit 7 to 0 register*/
  __I  uint8_t ADCI1_TAG;        /**< \brief (Sense Offset: 0x04) I1 tag register*/
  __I  uint8_t ADCI1_23_16;      /**< \brief (Sense Offset: 0x05) I1 adc bit 23 to 16 register*/
  __I  uint8_t ADCI1_15_8;       /**< \brief (Sense Offset: 0x06) I1 adc bit 15 to 8 register*/  
  __I  uint8_t ADCI1_7_0;        /**< \brief (Sense Offset: 0x07) I1 adc bit 7 to 0 register*/  
  __I  uint8_t ADCV1_TAG;        /**< \brief (Sense Offset: 0x08) V1 tag register*/
  __I  uint8_t ADCV1_23_16;      /**< \brief (Sense Offset: 0x09) V1 adc bit 23 to 16 register*/
  __I  uint8_t ADCV1_15_8;       /**< \brief (Sense Offset: 0x0a) V1 adc bit 15 to 8 register*/  
  __I  uint8_t ADCV1_7_0;        /**< \brief (Sense Offset: 0x0b) V1 adc bit 7 to 0 register*/  
  __I  uint8_t ADCI2_TAG;        /**< \brief (Sense Offset: 0x0c) I2 tag register*/
  __I  uint8_t ADCI2_23_16;      /**< \brief (Sense Offset: 0x0d) I2 adc bit 23 to 16 register*/
  __I  uint8_t ADCI2_15_8;       /**< \brief (Sense Offset: 0x0e) I2 adc bit 15 to 8 register*/  
  __I  uint8_t ADCI2_7_0;        /**< \brief (Sense Offset: 0x0f) I2 adc bit 7 to 0 register*/    
  __I  uint8_t ADCV2_TAG;        /**< \brief (Sense Offset: 0x10) V2 tag register*/
  __I  uint8_t ADCV2_23_16;      /**< \brief (Sense Offset: 0x11) V2 adc bit 23 to 16 register*/
  __I  uint8_t ADCV2_15_8;       /**< \brief (Sense Offset: 0x12) V2 adc bit 15 to 8 register*/  
  __I  uint8_t ADCV2_7_0;        /**< \brief (Sense Offset: 0x13) V2 adc bit 7 to 0 register*/   
  __I  uint8_t ADCI3_TAG;        /**< \brief (Sense Offset: 0x14) I3 tag register*/
  __I  uint8_t ADCI3_23_16;      /**< \brief (Sense Offset: 0x15) I3 adc bit 23 to 16 register*/
  __I  uint8_t ADCI3_15_8;       /**< \brief (Sense Offset: 0x16) I3 adc bit 15 to 8 register*/  
  __I  uint8_t ADCI3_7_0;        /**< \brief (Sense Offset: 0x17) I3 adc bit 7 to 0 register*/     
  __I  uint8_t ADCV3_TAG;        /**< \brief (Sense Offset: 0x18) V3 tag register*/
  __I  uint8_t ADCV3_23_16;      /**< \brief (Sense Offset: 0x19) V3 adc bit 23 to 16 register*/
  __I  uint8_t ADCV3_15_8;       /**< \brief (Sense Offset: 0x1a) V3 adc bit 15 to 8 register*/  
  __I  uint8_t ADCV3_7_0;        /**< \brief (Sense Offset: 0x1b) V3 adc bit 7 to 0 register*/    
  __I  uint8_t A0;               /**< \brief (Sense Offset: 0x1c) not used*/    
  __I  uint8_t A1;               /**< \brief (Sense Offset: 0x1d) not used*/  
  __I  uint8_t A2;               /**< \brief (Sense Offset: 0x1e) not used*/  
  __I  uint8_t A3;               /**< \brief (Sense Offset: 0x1f) not used*/  
  __IO uint8_t SDI0;             /**< \brief (Sense Offset: 0x20) ADC I0 Control register*/    
  __IO uint8_t SDI1;             /**< \brief (Sense Offset: 0x21) ADC I1 Control register*/  
  __IO uint8_t SDV1;             /**< \brief (Sense Offset: 0x22) ADC V1 Control register*/  
  __IO uint8_t SDI2;             /**< \brief (Sense Offset: 0x23) ADC I2 Control register*/  
  __IO uint8_t SDV2;             /**< \brief (Sense Offset: 0x24) ADC V2 Control register*/  
  __IO uint8_t SDI3;             /**< \brief (Sense Offset: 0x25) ADC I3 Control register*/  
  __IO uint8_t SDV3;             /**< \brief (Sense Offset: 0x26) ADC V3 Control register*/    
  __IO uint8_t ANA_CTRL;         /**< \brief (Sense Offset: 0x27) Analog Controls register*/      
  __IO uint8_t ATCFG;            /**< \brief (Sense Offset: 0x28) ATSENSE Configuration Register*/     
  __I  uint8_t A2TSR;            /**< \brief (Sense Offset: 0x29) ATSENSE Status Register*/   
  __IO uint8_t ITOUTCR;           /**< \brief (Sense Offset: 0x2a) Output Interrupt Line Control Register*/
  __IO uint8_t ITCR;             /**< \brief (Sense Offset: 0x2b) Interrupt Control Register*/  
  __I  uint8_t ITSR;             /**< \brief (Sense Offset: 0x2c) Interrupt Status Register*/   
  __O  uint8_t SOFT_NRESET;      /**< \brief (Sense Offset: 0x2d) Software Reset Register*/    
  
} Sense;
#endif /* !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */


/* Various definitions */

#define SENSE   ((Sense *)0x00U) /**< \brief (SENSE  ) Base Address */
#define Addr(member)  (uint8_t)(__INTADDR__((&(SENSE)->member)))  // extract addr value of a field
        //spibox.addr =  (uint8_t)&SENSE->ADCI0_TAG;
        //spibox.addr =  (uint8_t)offsetof(Sense, ADCI0_TAG);

enum gain {
	ADC_GAINX1	= 0x0,
	ADC_GAINX2	= 0x1,
        ADC_GAINX4	= 0x2,
	ADC_GAINX8	= 0x3,
};

enum osr {
	OSR8	= 0x0,
	OSR16	= 0x1,
        OSR32	= 0x2,
	OSR64	= 0x3,
};

/*  end */



/* -------- SDI0 : (Sense Offset:0x20) ADC I0 Control register -------- */
#define ONADC      (0x1u << 0)   /**< \brief enable adc */
#define GAIN_Pos    4            /**< \brief field position */
#define GAIN_Msk   (0x3u << GAIN_Pos) /**< \brief "11" 2 bits mask */
#define GAIN(value) ((GAIN_Msk & ((value) << GAIN_Pos))) // see enum gain for value
#define GAIN_ADC_GAINX1  (0x0u << GAIN_Pos) /**< \brief SDI0_GAINI0(value) gain value 1*/
#define GAIN_ADC_GAINX2  (0x1u << GAIN_Pos) /**< \brief SDI0_GAINI0(value) gain value 2*/
#define GAIN_ADC_GAINX4  (0x2u << GAIN_Pos) /**< \brief SDI0_GAINI0(value) gain value 4*/
#define GAIN_ADC_GAINX8  (0x3u << GAIN_Pos) /**< \brief SDI0_GAINI0(value) gain value 8*/
#define SDI0_TEMPMEAS         (0x1u << 1) /**< \brief Temperature Measurement Activation */




/* -------- ADCI0_TAG : (Sense Offset:0x01) ADCI0 TAG register -------- */
#define ADCI0_TAG_TAGI0_pos       0   /**< \brief field position */
#define ADCI0_TAG_TAGI0_msk       (0xfu << ADCI0_TAG_TAGI0_pos)  /**< \brief "1111" 4 bits mask */
#define ADCI0_TAG_TEMPMEAS_pos    4   /**< \brief field position */
#define ADCI0_TAG_TEMPMEAS_msk    (0x1u << ADCI0_TAG_TEMPMEAS_pos)  /**< \brief "1" 1 bits mask */
#define ADCI0_TAG_DATA_VALID_pos  5   /**< \brief field position */
#define ADCI0_TAG_DATA_VALID_msk (0x1u << ADCI0_TAG_DATA_VALID_pos)  /**< \brief "1" 1 bits mask */

/* -------- ANA_CTRL : (Sense Offset:0x27) Analog controls register -------- */
#define ANA_CTRL_ONBIAS  (0x1u << 0)   /**< \brief Enable of the bias */
#define ANA_CTRL_ONREF   (0x1u << 1)   /**< \brief Enable of the reference */
#define ANA_CTRL_ONLDO   (0x1u << 2)   /**< \brief Enable of the LDO */

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
