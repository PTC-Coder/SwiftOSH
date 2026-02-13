/* WriteToFlash.h — SwiftOSH (STM32U545RET6Q) */
#ifndef __WriteToFlash_H
#define __WriteToFlash_H

#include <stdint.h>

uint8_t WriteToFlash(uint8_t *buffer, uint32_t offset, uint16_t size);
uint8_t ReadFromFlash(uint8_t *buffer, uint32_t offset, uint16_t size);
uint8_t EraseFlashSector(void);

#endif /* __WriteToFlash_H */
