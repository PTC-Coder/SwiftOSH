/**
  ******************************************************************************
  * @file    RTC_Swift.c
  * @brief   RTC helper functions — SwiftOSH (STM32U545RET6Q)
  *          Ported from SwiftOne RTC.c.
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "RTC_Swift.h"
#include "GeneralDefines.h"
#include <string.h>
#include <time.h>

extern RTC_HandleTypeDef hrtc;

/**
  * @brief  Set RTC date and time from USB HID buffer.
  *         Buffer layout (BCD):
  *           [0] = report ID (4)
  *           [1] = Year, [2] = Month, [3] = Date, [4] = WeekDay
  *           [5] = Hours, [6] = Minutes, [7] = Seconds
  *           [8] = DST_Active_Flag
  */
void RTC_SetDateTime(uint8_t *DateTimeArray)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  sDate.Year    = DateTimeArray[1];
  sDate.Month   = DateTimeArray[2];
  sDate.Date    = DateTimeArray[3];
  sDate.WeekDay = DateTimeArray[4];
  sTime.Hours   = DateTimeArray[5];
  sTime.Minutes = DateTimeArray[6];
  sTime.Seconds = DateTimeArray[7];
  sTime.SubSeconds = 0;

  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

  /* Write DST_Active_Flag to backup register DR1 */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, (uint32_t)DateTimeArray[8]);
}

/**
  * @brief  Write DST active flag to RTC backup register.
  */
void RTC_SetDSTActiveFlag(uint8_t DSTFlag)
{
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, (uint32_t)DSTFlag);
}

/**
  * @brief  Read RTC date/time into buffer.
  *         buffer[1]=Year, [2]=Month, [3]=Date, [4]=WeekDay,
  *         [5]=Hours, [6]=Minutes, [7]=Seconds
  * @param  BCD_Or_Bin_Format: RTC_FORMAT_BCD or RTC_FORMAT_BIN
  */
void RTC_ReadDateTime(uint8_t *buffer, uint32_t BCD_Or_Bin_Format)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_RTC_WaitForSynchro(&hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  HAL_RTC_GetTime(&hrtc, &sTime, BCD_Or_Bin_Format);
  HAL_RTC_GetDate(&hrtc, &sDate, BCD_Or_Bin_Format);

  buffer[1] = sDate.Year;
  buffer[2] = sDate.Month;
  buffer[3] = sDate.Date;
  buffer[4] = sDate.WeekDay;
  buffer[5] = sTime.Hours;
  buffer[6] = sTime.Minutes;
  buffer[7] = sTime.Seconds;
}

/**
  * @brief  Return date-time string for WAV filename: "YYYYMMDD_hhmmss"
  */
void RTC_ReturnDateTimeFileName(char *buff)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  struct tm now;

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_RTC_WaitForSynchro(&hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  now.tm_year = (sDate.Year + 2000) - 1900;
  now.tm_mon  = sDate.Month - 1;
  now.tm_mday = sDate.Date;
  now.tm_hour = sTime.Hours;
  now.tm_min  = sTime.Minutes;
  now.tm_sec  = sTime.Seconds;
  now.tm_isdst = 0;

  strftime(buff, 16, "%Y%m%d_%H%M%S", &now);
}

/**
  * @brief  Return date string for day directory: "YYYY-MM-DD"
  */
void RTC_ReturnDateString(char *buff)
{
  RTC_DateTypeDef sDate;
  struct tm now;

  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  now.tm_year = (sDate.Year + 2000) - 1900;
  now.tm_mon  = sDate.Month - 1;
  now.tm_mday = sDate.Date;
  now.tm_hour = 0;
  now.tm_min  = 0;
  now.tm_sec  = 0;
  now.tm_isdst = 0;

  strftime(buff, 11, "%Y-%m-%d", &now);
}

/**
  * @brief  Return date-time string: "Date-time: YYYY/MM/DD HH:MM:SS\r\n"
  */
void RTC_ReturnDateTimeString(char *buff)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  struct tm now;

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_RTC_WaitForSynchro(&hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  now.tm_year = (sDate.Year + 2000) - 1900;
  now.tm_mon  = sDate.Month - 1;
  now.tm_mday = sDate.Date;
  now.tm_hour = sTime.Hours;
  now.tm_min  = sTime.Minutes;
  now.tm_sec  = sTime.Seconds;
  now.tm_isdst = 0;

  strftime(buff, 32, "Date-time: %Y/%m/%d %H:%M:%S\r\n", &now);
}

/**
  * @brief  Check if RTC subsecond register is advancing (clock working).
  * @retval 1 if clock is running, 0 if stuck
  */
uint8_t isClockWorking(void)
{
  uint32_t SSR_now = RTC->SSR;
  HAL_Delay(25);
  uint32_t SSR_later = RTC->SSR;

  return (SSR_now != SSR_later) ? 1 : 0;
}

/**
  * @brief  Compare a schedule time against the current RTC time.
  * @retval 0 = equal, 1 = ScheduleTime is in the future, 2 = in the past
  */
uint8_t RTC_CompareTimes(RTC_TimeTypeDef ScheduleTime, RTC_TimeTypeDef CurrentTime)
{
  if (ScheduleTime.Hours > CurrentTime.Hours)
    return 1;
  else if (ScheduleTime.Hours < CurrentTime.Hours)
    return 2;
  else
  {
    if (ScheduleTime.Minutes > CurrentTime.Minutes)
      return 1;
    else if (ScheduleTime.Minutes < CurrentTime.Minutes)
      return 2;
    else
      return 0;
  }
}

/**
  * @brief  Compare a schedule date against the current RTC date.
  * @retval 0 = equal, 1 = future, 2 = past
  */
uint8_t RTC_CompareDates(RTC_DateTypeDef ScheduleDate)
{
  RTC_DateTypeDef CurrentDate;
  HAL_RTC_GetDate(&hrtc, &CurrentDate, RTC_FORMAT_BIN);

  if (ScheduleDate.Year > CurrentDate.Year) return 1;
  if (ScheduleDate.Year < CurrentDate.Year) return 2;
  if (ScheduleDate.Month > CurrentDate.Month) return 1;
  if (ScheduleDate.Month < CurrentDate.Month) return 2;
  if (ScheduleDate.Date > CurrentDate.Date) return 1;
  if (ScheduleDate.Date < CurrentDate.Date) return 2;
  return 0;
}

/**
  * @brief  Check if today is between StartDate and StopDate (BCD input).
  * @retval 0 = not between, 1 = between, 2 = right on stop date
  */
uint8_t RTC_CheckIfBetweenDates(RTC_DateTypeDef StartDate, RTC_DateTypeDef StopDate)
{
  RTC_DateTypeDef BinaryDate;
  uint8_t result;

  /* Convert start date from BCD to binary and compare */
  BinaryDate.Year  = RTC_Bcd2ToByte(StartDate.Year);
  BinaryDate.Month = RTC_Bcd2ToByte(StartDate.Month);
  BinaryDate.Date  = RTC_Bcd2ToByte(StartDate.Date);

  result = RTC_CompareDates(BinaryDate);
  /* Start date must be today or in the past */
  if (result == 0 || result == 2)
  {
    BinaryDate.Year  = RTC_Bcd2ToByte(StopDate.Year);
    BinaryDate.Month = RTC_Bcd2ToByte(StopDate.Month);
    BinaryDate.Date  = RTC_Bcd2ToByte(StopDate.Date);

    result = RTC_CompareDates(BinaryDate);
    if (result == 1)
      return 1;  /* between dates */
    else if (result == 0)
      return 2;  /* right on stop date */
  }
  return 0;
}

/* Helper: return next start time for schedule */
static RTC_TimeTypeDef ReturnNextStartTime(Swift_Schedule *schedule)
{
  RTC_TimeTypeDef t = {0};
  if (schedule->ScheduleType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    uint8_t h = schedule->StartStopTimes[0].RecordingStopTime.Hours * schedule->CurrentRecordNumber;
    uint16_t m = (uint16_t)schedule->StartStopTimes[0].RecordingStopTime.Minutes * schedule->CurrentRecordNumber;
    while (m > 59) { h++; m -= 60; }
    t.Hours = h; t.Minutes = (uint8_t)m;
  }
  else
  {
    t.Hours   = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStartTime.Hours;
    t.Minutes = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStartTime.Minutes;
  }
  return t;
}

/* Helper: return next stop time for schedule */
static RTC_TimeTypeDef ReturnNextStopTime(Swift_Schedule *schedule)
{
  RTC_TimeTypeDef t = {0};
  if (schedule->ScheduleType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    uint8_t h = schedule->StartStopTimes[0].RecordingStopTime.Hours * schedule->CurrentRecordNumber;
    uint16_t m = (uint16_t)schedule->StartStopTimes[0].RecordingStopTime.Minutes * schedule->CurrentRecordNumber;
    h += schedule->StartStopTimes[0].RecordingStartTime.Hours;
    m += (uint16_t)schedule->StartStopTimes[0].RecordingStartTime.Minutes;
    while (m > 59) { h++; m -= 60; }
    t.Hours = h; t.Minutes = (uint8_t)m;
  }
  else
  {
    t.Hours   = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStopTime.Hours;
    t.Minutes = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStopTime.Minutes;
  }
  return t;
}

/**
  * @brief  Full schedule alarm initialization (ported from SwiftOne).
  * @retval 0 = standby/no schedule, 1 = mid-recording, 2 = between records
  */
uint8_t RTC_InitializeScheduleAlarms(Swift_Schedule *schedule)
{
  RTC_TimeTypeDef CurrentTime;
  RTC_TimeTypeDef ScheduleTime;
  RTC_DateTypeDef ScheduleDate;
  uint8_t returnval;

  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

  if (schedule->NumberOfRecords == 0)
  {
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
    schedule->CurrentRecordNumber = 0;
    return 0;
  }

  /* Check stop date */
  ScheduleDate.Year  = RTC_Bcd2ToByte(schedule->StopDate.Year);
  ScheduleDate.Month = RTC_Bcd2ToByte(schedule->StopDate.Month);
  ScheduleDate.Date  = RTC_Bcd2ToByte(schedule->StopDate.Date);

  returnval = RTC_CompareDates(ScheduleDate);
  if (returnval == 2 || returnval == 0)
  {
    /* Stop date is in the past or today — no valid schedule */
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
    schedule->CurrentRecordNumber = 0;
    return 0;
  }

  /* Check start date */
  ScheduleDate.Year  = RTC_Bcd2ToByte(schedule->StartDate.Year);
  ScheduleDate.Month = RTC_Bcd2ToByte(schedule->StartDate.Month);
  ScheduleDate.Date  = RTC_Bcd2ToByte(schedule->StartDate.Date);

  returnval = RTC_CompareDates(ScheduleDate);
  if (returnval == 1)
  {
    /* Start date is in the future — wait for midnight */
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
    RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);
    schedule->CurrentRecordNumber = 0;
    return 0;
  }

  /* We're between start and stop dates. Check time slots. */
  HAL_RTC_GetTime(&hrtc, &CurrentTime, RTC_FORMAT_BIN);
  /* Must also read date to unlock shadow registers */
  {
    RTC_DateTypeDef dummy;
    HAL_RTC_GetDate(&hrtc, &dummy, RTC_FORMAT_BIN);
  }

  for (int i = 0; i < schedule->NumberOfRecords; i++)
  {
    schedule->CurrentRecordNumber = i;

    ScheduleTime = ReturnNextStartTime(schedule);
    returnval = RTC_CompareTimes(ScheduleTime, CurrentTime);
    if (returnval == 1)
    {
      /* Start time is in the future — wait in standby */
      RTC_SetAlarmA(ScheduleTime);
      RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);
      return 2;
    }

    ScheduleTime = ReturnNextStopTime(schedule);
    returnval = RTC_CompareTimes(ScheduleTime, CurrentTime);
    if (returnval == 1)
    {
      /* We're in the middle of a recording period */
      SetNextStopTime(schedule);
      RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);
      return 1;
    }
  }

  /* All time slots done for today — wait for midnight */
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);
  schedule->CurrentRecordNumber = 0;
  return 0;
}

/**
  * @brief  Set RTC Alarm A to the next recording start time.
  *         Full logic ported from SwiftOne — handles duty-cycled and arbitrary.
  */
void SetNextStartTime(Swift_Schedule *schedule)
{
  RTC_TimeTypeDef ScheduleTime = {0};

  schedule->CurrentRecordNumber++;

  if (schedule->CurrentRecordNumber >= schedule->NumberOfRecords)
    schedule->CurrentRecordNumber = 0;

  if (schedule->ScheduleType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    if (schedule->CurrentRecordNumber != 0)
    {
      uint8_t h = schedule->StartStopTimes[0].RecordingStopTime.Hours * schedule->CurrentRecordNumber;
      uint16_t m = (uint16_t)schedule->StartStopTimes[0].RecordingStopTime.Minutes * schedule->CurrentRecordNumber;
      while (m > 59) { h++; m -= 60; }
      ScheduleTime.Hours = h;
      ScheduleTime.Minutes = (uint8_t)m;
      RTC_SetAlarmA(ScheduleTime);
    }
  }
  else if (schedule->ScheduleType == SCHEDULE_TYPE_ARBITRARY)
  {
    if (!((schedule->CurrentRecordNumber == 0) && (schedule->StartsAtMidnightFlag == 1)))
    {
      ScheduleTime.Hours   = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStartTime.Hours;
      ScheduleTime.Minutes = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStartTime.Minutes;
      RTC_SetAlarmA(ScheduleTime);
    }
  }
}

/**
  * @brief  Set RTC Alarm A to the next recording stop time.
  *         Full logic ported from SwiftOne — handles duty-cycled, arbitrary, continuous.
  */
void SetNextStopTime(Swift_Schedule *schedule)
{
  RTC_TimeTypeDef ScheduleTime = {0};

  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

  if (schedule->ScheduleType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    uint8_t h = schedule->StartStopTimes[0].RecordingStopTime.Hours * schedule->CurrentRecordNumber;
    uint16_t m = (uint16_t)schedule->StartStopTimes[0].RecordingStopTime.Minutes * schedule->CurrentRecordNumber;
    h += schedule->StartStopTimes[0].RecordingStartTime.Hours;
    m += (uint16_t)schedule->StartStopTimes[0].RecordingStartTime.Minutes;
    while (m > 59) { h++; m -= 60; }
    ScheduleTime.Hours = h;
    ScheduleTime.Minutes = (uint8_t)m;
    RTC_SetAlarmA(ScheduleTime);
  }
  else if (schedule->ScheduleType == SCHEDULE_TYPE_ARBITRARY)
  {
    if (!(((schedule->CurrentRecordNumber + 1) == schedule->NumberOfRecords) && (schedule->EndsAtMidnightFlag == 1)))
    {
      ScheduleTime.Hours   = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStopTime.Hours;
      ScheduleTime.Minutes = schedule->StartStopTimes[schedule->CurrentRecordNumber].RecordingStopTime.Minutes;
      RTC_SetAlarmA(ScheduleTime);
    }
  }
  else /* CONTINUOUS */
  {
    ScheduleTime.Hours = 23;
    ScheduleTime.Minutes = 59;
    RTC_SetAlarmA(ScheduleTime);
  }
}

/**
  * @brief  Set RTC Alarm A to fire at the specified time.
  */
uint8_t RTC_SetAlarmA(RTC_TimeTypeDef alarmTime)
{
  RTC_AlarmTypeDef sAlarm = {0};

  sAlarm.AlarmTime.Hours   = alarmTime.Hours;
  sAlarm.AlarmTime.Minutes = alarmTime.Minutes;
  sAlarm.AlarmTime.Seconds = alarmTime.Seconds;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;

  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    return 1;

  return 0;
}

/**
  * @brief  Enable Alarm B for midnight (00:00:00) or 2AM (02:00:00).
  * @param  mode: SET_ALARMB_MIDNIGHT or SET_ALARMB_2AM
  */
uint8_t RTC_EnableAlarmB(uint8_t mode)
{
  RTC_AlarmTypeDef sAlarm = {0};

  sAlarm.AlarmTime.Hours   = (mode == SET_ALARMB_2AM) ? 0x02 : 0x00;
  sAlarm.AlarmTime.Minutes = 0x00;
  sAlarm.AlarmTime.Seconds = 0x00;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_B;

  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    return 1;

  return 0;
}

/**
  * @brief  Deactivate an RTC alarm.
  * @param  alarm: RTC_ALARM_A or RTC_ALARM_B
  */
uint8_t RTC_DeactivateAlarm(uint32_t alarm)
{
  HAL_RTC_DeactivateAlarm(&hrtc, alarm);
  return 0;
}


/* ---- DST helper functions ---- */

static uint8_t DST_Active(DST_Settings *DSTSettings)
{
  DSTSettings->DST_Suffix_To_Use = &DSTSettings->DST_Filename_Suffix_Active[0];

  if (DSTSettings->DST_Active_Flag == 1)
  {
    if (DSTSettings->add_or_subtract_time == '+')
    {
      __HAL_RTC_DAYLIGHT_SAVING_TIME_ADD1H(&hrtc, RTC_STOREOPERATION_RESET);
    }
    else
    {
      __HAL_RTC_DAYLIGHT_SAVING_TIME_SUB1H(&hrtc, RTC_STOREOPERATION_RESET);
    }

    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    HAL_RTC_WaitForSynchro(&hrtc);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

    DSTSettings->DST_Active_Flag = 2;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0002);
  }
  return 1;
}

static uint8_t DST_Inactive(DST_Settings *DSTSettings)
{
  DSTSettings->DST_Suffix_To_Use = &DSTSettings->DST_Filename_Suffix_Inactive[0];

  if (DSTSettings->DST_Active_Flag == 2)
  {
    if (DSTSettings->add_or_subtract_time == '+')
    {
      __HAL_RTC_DAYLIGHT_SAVING_TIME_SUB1H(&hrtc, RTC_STOREOPERATION_RESET);
    }
    else
    {
      __HAL_RTC_DAYLIGHT_SAVING_TIME_ADD1H(&hrtc, RTC_STOREOPERATION_RESET);
    }

    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    HAL_RTC_WaitForSynchro(&hrtc);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

    DSTSettings->DST_Active_Flag = 1;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x0001);
  }
  return 1;
}

/**
  * @brief  Evaluate DST state and adjust RTC clock if needed.
  *         Ported from SwiftOne RTC_InitiateDST().
  */
uint8_t RTC_InitiateDST(DST_Settings *DSTSettings)
{
  RTC_DateTypeDef CurrentDate;
  int IsLeapYear;
  static const int days[2][13] = {
    {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
    {0, 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
  };

  HAL_RTC_GetDate(&hrtc, &CurrentDate, RTC_FORMAT_BIN);
  IsLeapYear = (CurrentDate.Year % 4 == 0 && CurrentDate.Year % 100 != 0) ||
               (CurrentDate.Year % 400 == 0);

  DSTSettings->Current_DOY = days[IsLeapYear][CurrentDate.Month] + CurrentDate.Date;
  DSTSettings->TotalDaysInYear = 365 + IsLeapYear;

  if (DSTSettings->DST_Stop_DOY > DSTSettings->DST_Start_DOY)
  {
    if ((DSTSettings->DST_Start_DOY <= DSTSettings->Current_DOY) &&
        (DSTSettings->Current_DOY < DSTSettings->DST_Stop_DOY))
      DST_Active(DSTSettings);
    else
      DST_Inactive(DSTSettings);
  }
  else if ((DSTSettings->Current_DOY >= 1) &&
           (DSTSettings->Current_DOY < DSTSettings->DST_Stop_DOY))
  {
    DST_Active(DSTSettings);
  }
  else if ((DSTSettings->Current_DOY >= DSTSettings->DST_Start_DOY) &&
           (DSTSettings->Current_DOY < (366 + IsLeapYear)))
  {
    DST_Active(DSTSettings);
  }
  else
  {
    DST_Inactive(DSTSettings);
  }

  return 1;
}

/**
  * @brief  Add 2 minutes to current time and set Alarm A (for voice memo timeout).
  */
void RTC_AddTimeAndSetAlarmA(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm = {0};
  struct tm time_info;
  time_t timeNow;
  struct tm *converted_time;

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_RTC_WaitForSynchro(&hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  time_info.tm_year = (sDate.Year + 2000) - 1900;
  time_info.tm_mon  = sDate.Month - 1;
  time_info.tm_mday = sDate.Date;
  time_info.tm_hour = sTime.Hours + 1;
  time_info.tm_min  = sTime.Minutes;
  time_info.tm_sec  = sTime.Seconds;
  time_info.tm_isdst = 0;

  timeNow = mktime(&time_info);
  timeNow += 120;  /* add 2 minutes */

  converted_time = localtime(&timeNow);

  sAlarm.AlarmTime.Hours   = converted_time->tm_hour;
  sAlarm.AlarmTime.Minutes = converted_time->tm_min;
  sAlarm.AlarmTime.Seconds = converted_time->tm_sec;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;

  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
}

/**
  * @brief  Return date-time stamp string: "[ YYYY/MM/DD HH:MM:SS ]"
  */
void RTC_ReturnDateTimeStampString(char *buff)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  struct tm now;

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_RTC_WaitForSynchro(&hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  now.tm_year = (sDate.Year + 2000) - 1900;
  now.tm_mon  = sDate.Month - 1;
  now.tm_mday = sDate.Date;
  now.tm_hour = sTime.Hours;
  now.tm_min  = sTime.Minutes;
  now.tm_sec  = sTime.Seconds;
  now.tm_isdst = 0;

  strftime(buff, 32, "[ %Y/%m/%d %H:%M:%S ]", &now);
}
