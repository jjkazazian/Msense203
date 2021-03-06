#ifndef _MUX_H
#define _MUX_H

#define B0_pos    0   
#define B0_msk    (0x1u << B0_pos) 
#define B1_pos    1     
#define B1_msk    (0x1u << B1_pos)   
#define B2_pos    2       
#define B2_msk    (0x1u << B2_pos) 
#define B3_pos    3       
#define B3_msk    (0x1u << B3_pos) 

struct _MUX  {
   /* int8_t sd0;
    int8_t sdI1;
    int8_t sdV1;
    int8_t sdI2;
    int8_t sdV2;
    
    bool SD0[3];
    bool SDI1[3];
    bool SDV1[3];
    bool SDI2[3];
    bool SDV2[3];
    */
    bool D0[4];
    bool D1[4];
    bool D2[4];
    bool D3[4];
    bool Fsync[4];
    
  } __attribute__((packed)) ;

struct _DEMUX  {
  
    uint32_t synchro_index;  
    /*
    int8_t sd0;
    int8_t sdI1;
    int8_t sdV1;
    int8_t sdI2;
    int8_t sdV2;
    */
    bool SD0 [3];
    bool SDI1[3];
    bool SDV1[3];
    bool SDI2[3];
    bool SDV2[3];
    /*
    bool BD0[4];
    bool BD1[4];
    bool BD2[4];
    bool BD3[4];
 
    uint8_t D0;
    uint8_t D1;
    uint8_t D2;
    uint8_t D3;
*/
    
  } __attribute__((packed)) ;

 void BS_2_IO(void);
 int8_t decodeBS(bool *code ); 
 void demxcode(void);
 void demxcode_bs0(void);
#endif /* ! _MUX_H */

