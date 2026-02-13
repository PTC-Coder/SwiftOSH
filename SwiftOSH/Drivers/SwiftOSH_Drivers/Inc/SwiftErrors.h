/* SwiftErrors.h — SwiftOSH */
#ifndef SWIFT_ERRORS_H
#define SWIFT_ERRORS_H

#include <stdint.h>
#include "ff.h"

typedef enum {
  FUNC_AudioFiles_MountSDCard = 0,
  FUNC_AudioFiles_FormatSDCard,
  FUNC_AudioFiles_CreateNextDirectory,
  FUNC_AudioFiles_CreateNextFile,
  FUNC_Main_FWrite,
  FUNC_AudioFiles_WriteHeader,
  FUNC_AudioFiles_CloseFile,
  DMATransferComplete,
  DMATransferM1Complete,
  DMATransferError
} CALLING_FUNCTIONS;

typedef struct {
  CALLING_FUNCTIONS CallingFunction;
  uint8_t ErrorNumber;
  FRESULT FatFsResult;
} SWIFT_ERRORS;

#endif
