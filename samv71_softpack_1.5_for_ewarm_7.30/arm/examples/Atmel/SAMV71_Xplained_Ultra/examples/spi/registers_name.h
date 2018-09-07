#define REG_ADCI0_TAG     0x00 
#define REG_ADCI0_23_16   0x01 
#define REG_ADCI0_15_8    0x02 
#define REG_ADCI0_7_0     0x03 
#define REG_ADCI1_TAG     0x04 
#define REG_ADCI1_23_16   0x05 
#define REG_ADCI1_15_8    0x06 
#define REG_ADCI1_7_0     0x07 
#define REG_ADCV1_TAG     0x08 
#define REG_ADCV1_23_16   0x09 
#define REG_ADCV1_15_8    0x0A 
#define REG_ADCV1_7_0     0x0B 
#define REG_ADCI2_TAG     0x0C 
#define REG_ADCI2_23_16   0x0D 
#define REG_ADCI2_15_8    0x0E 
#define REG_ADCI2_7_0     0x0F 
#define REG_ADCV2_TAG     0x10 
#define REG_ADCV2_23_16   0x11 
#define REG_ADCV2_15_8    0x12 
#define REG_ADCV2_7_0     0x13 
#define REG_ADCI3_TAG     0x14 
#define REG_ADCI3_23_16   0x15 
#define REG_ADCI3_15_8    0x16 
#define REG_ADCI3_7_0     0x17 
#define REG_ADCV3_TAG     0x18 
#define REG_ADCV3_23_16   0x19 
#define REG_ADCV3_15_8    0x1A 
#define REG_ADCV3_7_0     0x1B 
#define REG_SDI0          0x20 
#define REG_SDI1          0x21 
#define REG_SDV1          0x22 
#define REG_SDI2          0x23 
#define REG_SDV2          0x24 
#define REG_SDI3          0x25 
#define REG_SDV3          0x26 
#define REG_ANA_CTRL      0x27 
#define REG_ATCFG         0x28 
#define REG_ATSR          0x29 
#define REG_ITOUTCR       0x2A 
#define REG_ITCR          0x2B 
#define REG_ITSR          0x2C 
#define REG_SOFT_NRESET   0x2D 
#define REG_VER           0x2E 
#define REG_ID            0x2F 
#define REG_BIAS          0x30 
#define REG_PHGEN_ADC     0x31 
#define REG_PHGEN_REF     0x32 
#define REG_ATCFG_CLOCK   0x33 
#define REG_ICTLI0        0x34 
#define REG_ICTLI1        0x35 
#define REG_ICTLV1        0x36 
#define REG_ICTLI2        0x37 
#define REG_ICTLV2        0x38 
#define REG_ICTLI3        0x39 
#define REG_ICTLV3        0x3A 
#define REG_VCM           0x3B 
#define REG_LDO_CTRL      0x3C 
#define REG_FSM_CLOCK     0x3D 
#define REG_SECURITY      0x3E

#define VC_ITEM_CHECKBOX_ADCI0       0x40
#define VC_ITEM_SLIDER_ADCI0_GAIN    0x41
#define VC_ITEM_CHECKBOX_TEMPMEAS    0x42

#define VC_ITEM_CHECKBOX_ADCI1       0x43
#define VC_ITEM_SLIDER_ADCI1_GAIN    0x44

#define VC_ITEM_CHECKBOX_ADCV1       0x45

#define VC_ITEM_CHECKBOX_ADCI2       0x46
#define VC_ITEM_SLIDER_ADCI2_GAIN    0x47

#define VC_ITEM_CHECKBOX_ADCV2       0x48

#define VC_ITEM_CHECKBOX_ADCI3       0x49
#define VC_ITEM_SLIDER_ADCI3_GAIN    0x4A

#define VC_ITEM_CHECKBOX_ADCV3       0x4B

#define VC_ITEM_CHECKBOX_ONLDO       0x4C
#define VC_ITEM_CHECKBOX_ONREF       0x4D
#define VC_ITEM_CHECKBOX_ONBIAS      0x4E

#define VC_ITEM_CHECKBOX_MSBMODE     0x4F
#define VC_ITEM_COMBO_OSR            0x50

#define VC_ITEM_CHECKBOX_ITOUTADCRDY 0x51
#define VC_ITEM_CHECKBOX_ITOUTOVRUN  0x52
#define VC_ITEM_CHECKBOX_ITOUTUNRUN  0x53

#define VC_ITEM_CHECKBOX_ITCRADCRDY  0x54
#define VC_ITEM_CHECKBOX_ITCROVRUN   0x55
#define VC_ITEM_CHECKBOX_ITCRUNRUN   0x56

#define VC_ITEM_BUTTON_NRESET        0x57

#define VC_ITEM_SYSTEM_STATUS        0x58

#define VC_ITEM_SPI_VER              0x59

#define VC_ITEM_CHIP_ID              0x5A

#define VC_ITEM_CHECKBOX_SYSRDYDIS   0x5B
#define VC_ITEM_CHECKBOX_SECPRIV     0x5C
#define VC_ITEM_CHECKBOX_SECRES      0x5D

#define VC_ITEM_CHECKBOX_ONBUF       0x5E
#define VC_ITEM_COMBO_ROUT           0x5F
#define VC_ITEM_COMBO_ICTLBUF        0x60

#define VC_ITEM_COMBO_ADCI0ICTL      0x61
#define VC_ITEM_CHECKBOX_ADCI0DITHER 0x62
#define VC_ITEM_CHECKBOX_ADCI0VCM    0x63

#define VC_ITEM_COMBO_ADCI1ICTL      0x64
#define VC_ITEM_CHECKBOX_ADCI1DITHER 0x65
#define VC_ITEM_CHECKBOX_ADCI1VCM    0x66

#define VC_ITEM_COMBO_ADCI2ICTL      0x67
#define VC_ITEM_CHECKBOX_ADCI2DITHER 0x68
#define VC_ITEM_CHECKBOX_ADCI2VCM    0x69

#define VC_ITEM_COMBO_ADCI3ICTL      0x6A
#define VC_ITEM_CHECKBOX_ADCI3DITHER 0x6B
#define VC_ITEM_CHECKBOX_ADCI3VCM    0x6C

#define VC_ITEM_COMBO_ADCV1ICTL      0x6D
#define VC_ITEM_CHECKBOX_ADCV1DITHER 0x6E
#define VC_ITEM_CHECKBOX_ADCV1VCM    0x6F

#define VC_ITEM_COMBO_ADCV2ICTL      0x70
#define VC_ITEM_CHECKBOX_ADCV2DITHER 0x71
#define VC_ITEM_CHECKBOX_ADCV2VCM    0x72

#define VC_ITEM_COMBO_ADCV3ICTL      0x73
#define VC_ITEM_CHECKBOX_ADCV3DITHER 0x74
#define VC_ITEM_CHECKBOX_ADCV3VCM    0x75

#define VC_ITEM_COMBO_ANALOGMCLK     0x76
#define VC_ITEM_COMBO_DSPMCLK        0x77

#define VC_ITEM_COMBO_CHOPPERF       0x78
#define VC_ITEM_CHECKBOX_CHOPPEREN   0x79
#define VC_ITEM_CHECKBOX_PHGENEN     0x7A

#define VC_ITEM_CHECKBOX_ONBIASLDO   0x7B
#define VC_ITEM_CHECKBOX_PRCHARGE    0x7C
#define VC_ITEM_CHECKBOX_VSEL        0x7D

#define VC_ITEM_CHECKBOX_ANAPOL      0x7E
#define VC_ITEM_COMBO_FSMMCLK        0x7F

#define VC_ITEM_CHECKBOX_ONCHOPBG    0x80
