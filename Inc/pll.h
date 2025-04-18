#ifndef __PLL_H__
#define __PLL_H__

#include "fm33ld5xx_fl.h"
#include <stdio.h>
void RCC_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLL_R, uint32_t PLL_DB, uint8_t div);

void RCHFInit(uint32_t RCHFclock);
uint32_t SelRCHFToPLL(uint32_t PLLclock, uint8_t div);


#endif
