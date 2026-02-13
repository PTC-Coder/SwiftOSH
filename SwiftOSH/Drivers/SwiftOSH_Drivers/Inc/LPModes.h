/* LPModes.h — SwiftOSH (STM32U545RET6Q)
 * Low-power mode helpers: Sleep and Stop 2 entry/exit.
 */
#ifndef __LPModes_H
#define __LPModes_H

#include "stm32u5xx_hal.h"

void LPModes_Sleep(void);
void LPModes_EnterStop2(void);
void LPModes_RestoreClockAfterStop2(void);

#endif /* __LPModes_H */
