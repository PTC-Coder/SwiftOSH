/**
  ******************************************************************************
  * @file    AudioFiles.h — SwiftOSH
  * @brief   SD card file/directory management for audio recording
  *          Ported from SwiftOne AudioFiles.h
  ******************************************************************************
  */
#ifndef AUDIOFILES_H_
#define AUDIOFILES_H_

#include "integer.h"
#include "ff.h"
#include "SwiftErrors.h"
#include "SwiftSettings.h"

uint8_t AudioFiles_MountSDCard(SWIFT_ERRORS *SwiftError);
uint8_t AudioFiles_WriteHeader(SWIFT_ERRORS *SwiftError, FIL *MyFile);
uint8_t AudioFiles_CloseFile(SWIFT_ERRORS *SwiftError, FIL *MyFile);

uint8_t AudioFiles_GetDiskInfoString(char *DiskInfoString);

uint8_t AudioFiles_NewRecordingDirectory(WAVFile_Attributes *RecordingDirectory);
uint8_t AudioFiles_NewDayDirectory(uint8_t *CustomLabel);
uint8_t AudioFiles_NewAudioFile(uint8_t *CustomLabel, uint8_t *utc_suffix, FIL *MyFile);

uint8_t AudioFiles_ChangeDirectory(void);
uint8_t AudioFiles_NewConfigTextFile(void);

#endif /* AUDIOFILES_H_ */
