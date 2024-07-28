#include "alarm.h"

extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;

extern struct buttons button1, button2;


alarmSettings alarm = {false, 6, 30};

void getAlarm()
{
	 alarm.hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	 alarm.minutes = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
}

void setAlarm(uint16_t hours, uint16_t minutes)
{
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, hours); // в первый регистр запишем число 1234
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, minutes); // во второй регистр запишем число 5678
}

void startAlarm()
{
	static bool checkAlarmPeriod = true;
	if(checkAlarmPeriod == true)
	{
		TIM4 -> ARR = 1000-1;
		checkAlarmPeriod = false;
	}
	else
	{
		TIM4 -> ARR = 50-1;
		checkAlarmPeriod = true;
	}
	HAL_TIM_Base_Start_IT(&htim4);
}
