#include "crc_ccitt.h"


//查表法计算crc
uint16_t do_crc_table(uint8_t *ptr, int len)
{
    uint16_t crc = 0x0000;
    
    while(len--) 
    {
        crc = (crc >> 8) ^ crc_table[(crc ^ *ptr++) & 0xff];
    }
    
    return crc;
}


