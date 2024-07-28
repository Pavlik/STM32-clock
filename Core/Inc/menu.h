#ifndef __MENU__
#define __MENU__



#include "stm32f1xx.h"
#include "stdio.h"
#include "lcd1602.h"
#include "debounce.h"
#include "alarm.h"

extern RTC_HandleTypeDef hrtc;



void initNewSymbols();

void showAlarm();
void showTime(uint8_t x, uint8_t y);
void showRH();
void showTemperature();
void showMainPage();
void setTimeMenu();
void setAlarmMenu();

void startTim1ForBlinkValue();
void setTime();
void chooseSettingValue();

#endif
