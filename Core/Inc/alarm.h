#ifndef __ALARM__
#define __ALARM__

#include "stm32f1xx.h"
#include "debounce.h"


typedef struct
{
	bool enable;
	uint16_t hours;
	uint16_t minutes;
} alarmSettings;

void setAlarm(uint16_t hours, uint16_t minutes);
void getAlarm();
void initAlarm();
void startAlarm();
void stopAlarm();

#endif
