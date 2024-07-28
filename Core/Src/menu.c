#include "menu.h"

#define TIME_X_POSITION 5
#define TIME_Y_POSITION 0

#define RH_X_POSITION 12
#define RH_Y_POSITION 1

#define TEMPERATURE_X_POSITION 0
#define TEMPERATURE_Y_POSITION 1

#define ALARM_X_POSITION 10
#define ALARM_Y_POSITION 0

#define SET_TIME_X_POSITION 4
#define SET_TIME_Y_POSITION 1

extern alarmSettings alarm;

extern TIM_HandleTypeDef htim1;
extern struct buttons button1, button2;
extern bool updateByEvent;
extern bool startSetting;

const uint8_t alarmRingSigne[8] = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x04, 0x00};
const uint8_t temperatureSigne[8] = {0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E};
const uint8_t humiditySigne[8] = {0x04, 0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E};

RTC_TimeTypeDef sTime = {0};

char trans_str[40] = {0,};

bool timeSettingValueFlag = false;

void showMainPage()
{
	 showTime(TIME_X_POSITION, TIME_Y_POSITION);
	 showRH();
	 showTemperature();
	 if(alarm.enable == true)
		 showAlarm();
	 else
	 {
		 lcd1602_SetCursor(ALARM_X_POSITION, ALARM_Y_POSITION);
		 snprintf(trans_str, 39, " ");
		 lcd1602_Print_text(trans_str);
	 }
	 //button1.longFlag = false;
}

void showTime(uint8_t x, uint8_t y)
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	if(sTime.Hours <= 9)
	{
		snprintf(trans_str, 39, "0%d:", sTime.Hours);
	}
	else
	{
		snprintf(trans_str, 39, "%d:", sTime.Hours);
	}
	lcd1602_SetCursor(x, y);
	lcd1602_Print_text(trans_str);

	if(sTime.Minutes <= 9)
	{
		snprintf(trans_str, 39, "0%d", sTime.Minutes);
	}
	else
	{
		snprintf(trans_str, 39, "%d", sTime.Minutes);
	}

	lcd1602_SetCursor(x+3, y);
	lcd1602_Print_text(trans_str);
}

void setTimeMenu()
{
	lcd1602_SetCursor(2, 0);
	snprintf(trans_str, 39, "TIME SETTING");
	lcd1602_Print_text(trans_str);

	showTime(SET_TIME_X_POSITION, SET_TIME_Y_POSITION);
	updateByEvent = true;
}

void setAlarmMenu()
{
	lcd1602_SetCursor(2, 0);
	sprintf(trans_str, "ALARM SETTING");
	lcd1602_Print_text(trans_str);
	getAlarm();

	lcd1602_SetCursor(4, 1);
	sprintf(trans_str, "%d:%d", alarm.hours, alarm.minutes);
	lcd1602_Print_text(trans_str);
}

void showTemperature()
{
	lcd1602_SetCursor(TEMPERATURE_X_POSITION, TEMPERATURE_Y_POSITION);
	lcd1602_Print_symbol(1);
	sprintf(trans_str, "22C ");
	lcd1602_SetCursor(TEMPERATURE_X_POSITION+1, TEMPERATURE_Y_POSITION);
	lcd1602_Print_text(trans_str);
}

void showRH()
{
	lcd1602_SetCursor(RH_X_POSITION, RH_Y_POSITION);
	lcd1602_Print_symbol(3);
	sprintf(trans_str, "64%% ");
	lcd1602_SetCursor(RH_X_POSITION+1, RH_Y_POSITION);
	lcd1602_Print_text(trans_str);
}

void showAlarm()
{
	lcd1602_SetCursor(ALARM_X_POSITION, ALARM_Y_POSITION);
	lcd1602_Print_symbol(2);
}

void initNewSymbols()
{
	lcd1602_Create_symbol((uint8_t *) temperatureSigne, 1);
	lcd1602_Create_symbol((uint8_t *) alarmRingSigne, 2);
	lcd1602_Create_symbol((uint8_t *) humiditySigne, 3);
}

void setTime()
{
	static bool checkPeriodOfBlink = false;
	uint8_t settingHourse, settingMinutes;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	settingHourse = sTime.Hours;
	settingMinutes = sTime.Minutes;
	if(checkPeriodOfBlink == false)
	{
		if(timeSettingValueFlag == false)
		{
			button2.shortFlag = false;
			if(button2.shortFlag == true || (HAL_GPIO_ReadPin(button2.port, button2.pin) == false && button2.longFlag == true))
			{
				settingHourse++;
				if(settingHourse == 24)
					settingHourse = 0;
				sTime.Hours = settingHourse;
				button2.shortFlag = false;
				HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			}

			lcd1602_SetCursor(SET_TIME_X_POSITION, SET_TIME_Y_POSITION);
			snprintf(trans_str, 39, "  ");
			lcd1602_Print_text(trans_str);
		}
		else if(timeSettingValueFlag == true)
		{
			if(button2.shortFlag == true || (HAL_GPIO_ReadPin(button2.port, button2.pin) == false && button2.longFlag == true))
			{
				settingMinutes++;
				if(settingMinutes == 60)
					settingMinutes = 0;
				sTime.Minutes = settingMinutes;
				button2.shortFlag = false;
				HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			}

			lcd1602_SetCursor(SET_TIME_X_POSITION+3, SET_TIME_Y_POSITION);
			snprintf(trans_str, 39, "  ");
			lcd1602_Print_text(trans_str);
		}
		checkPeriodOfBlink = true;
	}
	else
	{
		showTime(SET_TIME_X_POSITION, SET_TIME_Y_POSITION);
		checkPeriodOfBlink = false;
	}

}

void chooseSettingValue()
{
	if(button1.shortFlag == true)
	{
		button1.shortFlag = false;
		timeSettingValueFlag = !timeSettingValueFlag;
	}
}

void startTim1ForBlinkValue()
{
	HAL_TIM_Base_Start_IT(&htim1);
	//button1.shortFlag = false;
}

