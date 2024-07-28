#ifndef __DEBOUNCE__
#define __DEBOUNCE__

#include "stm32f1xx.h"
#include "stdbool.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

struct buttons
{
	uint16_t pin;
	GPIO_TypeDef* port;
	volatile bool buttonPressed;
	uint8_t risingBit;
	uint8_t fallingBit;
	bool shortFlag;
	bool longFlag;
};

void getDataFromButton(struct buttons * button);
void handleShortPress(struct buttons ** button);
void handleLongPress(struct buttons ** button);
void callBackButton(struct buttons * button);

#endif
