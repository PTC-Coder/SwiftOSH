/* RTC_Swift.h — SwiftOSH (STM32U545RET6Q) */
#ifndef __RTC_SWIFT_H
#define __RTC_SWIFT_H

#include "stm32u5xx_hal.h"
#include "SwiftSettings.h"

void RTC_SetDateTime(uint8_t *DateTimeArray);
void RTC_SetDSTActiveFlag(uint8_t DSTFlag);
void RTC_ReadDateTime(uint8_t *buffer, uint32_t BCD_Or_Bin_Format);
void RTC_ReturnDateTimeFileName(char *buff);
void RTC_ReturnDateString(char *buff);
void RTC_ReturnDateTimeString(char *buff);
uint8_t isClockWorking(void);

uint8_t RTC_InitializeScheduleAlarms(Swift_Schedule *schedule);
void SetNextStartTime(Swift_Schedule *schedule);
void SetNextStopTime(Swift_Schedule *schedule);
uint8_t RTC_SetAlarmA(RTC_TimeTypeDef alarmTime);
uint8_t RTC_EnableAlarmB(uint8_t mode);
uint8_t RTC_DeactivateAlarm(uint32_t alarm);

/* Date comparison and DST functions */
uint8_t RTC_CompareDates(RTC_DateTypeDef ScheduleDate);
uint8_t RTC_CompareTimes(RTC_TimeTypeDef ScheduleTime, RTC_TimeTypeDef CurrentTime);
uint8_t RTC_CheckIfBetweenDates(RTC_DateTypeDef StartDate, RTC_DateTypeDef StopDate);
uint8_t RTC_InitiateDST(DST_Settings *DSTSettings);
void RTC_AddTimeAndSetAlarmA(void);
void RTC_ReturnDateTimeStampString(char *buff);

#endif /* __RTC_SWIFT_H */
