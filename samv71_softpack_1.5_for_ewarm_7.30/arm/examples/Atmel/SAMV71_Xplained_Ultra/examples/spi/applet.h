
#ifndef APPLET_H
#define APPLET_H

//------------------------------------------------------------------------------
//         Global definitions
//------------------------------------------------------------------------------

/// Applet initialization command code.
#define APPLET_ATSENSE301_INIT              0x81
/// Applet lock command code.
#define APPLET_ATSENSE301_LOCK              0x82
/// Applet unlock command code.
#define APPLET_ATSENSE301_UNLOCK            0x83
/// Applet write command code.
#define APPLET_ATSENSE301_WRITE             0x84
/// Applet read command code.
#define APPLET_ATSENSE301_READ              0x85
/// Applet VC item set status.
#define APPLET_ATSENSE301_VC_SET_STATUS     0x86
/// Applet VC item get status.
#define APPLET_ATSENSE301_VC_GET_STATUS     0x87
/// Applet NOP.
#define APPLET_ATSENSE301_NOP               0x88
/// Applet enable spi-to-ssc bridge.
#define APPLET_ATSENSE301_EN_BRIDGE         0x89
/// Applet VC item set SPI CS.
#define APPLET_ATSENSE301_VC_SET_SPICS      0x8A
/// Applet VC item set SPI VREF.
#define APPLET_ATSENSE301_VC_SET_VREF       0x8B
/// Applet VC item set SPI VTEMP.
#define APPLET_ATSENSE301_VC_SET_VTEMP      0x8C
/// Applet VC item set SPI VGND.
#define APPLET_ATSENSE301_VC_SET_VGND       0x8D
/// Applet VC item set SPI VGND.
#define APPLET_ATSENSE301_VC_SET_4MHZ       0x8E
/// Applet measure temperature .
#define APPLET_ATSENSE301_MEAS_TEMP         0x8F

/// Operation was successful.
#define APPLET_SUCCESS                      0x00

#endif //#ifndef APPLET_H
