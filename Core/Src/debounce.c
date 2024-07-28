#include "debounce.h"

volatile bool buttonState = false;
extern volatile uint8_t buttonType;
extern TIM_HandleTypeDef htim4;

void getDataFromButton(struct buttons * button)
{
	buttonState = !HAL_GPIO_ReadPin(button->port, button->pin);
	if(buttonState && !(button->buttonPressed)) //Если кнопка нажата и флаг нажатия не опущен считать кнопку нажатой, но не обрабатывать
	{
		button->buttonPressed = true;
		EXTI->FTSR &= ~(button->fallingBit);  // Отключаем прерывание по нисходящему фронту
		EXTI->RTSR |= button->risingBit;   // Включаем прерывание по восходящему фронту
		TIM3 -> ARR = 20000;		   // Подобрать значение чтобы была 2 секунда
		button->longFlag = false;
	}
	else if(!buttonState && !(button->buttonPressed)) //Если кнопка не нажата и флаг нажатия не опущен
	{
		buttonType = 0;
	}
	else if(buttonState && (button->buttonPressed)) //Если кнопка нажата и флаг нажатия опущен считать кнопку удержанной
	{
		handleLongPress(&button);
	}
	__HAL_GPIO_EXTI_CLEAR_IT(button->pin);  	// очищаем бит EXTI_PR
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);	// очищаем бит NVIC_ICPRx
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);     // включаем внешнее прерывание
}

void handleShortPress(struct buttons ** button)
{
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	(*button)->shortFlag = true;

	HAL_TIM_Base_Start_IT(&htim4);
	(*button)->buttonPressed = false;
	EXTI->FTSR |= (*button)->fallingBit;  // Включаем прерывание по нисходящему фронту
	EXTI->RTSR &= ~((*button)->risingBit);
	TIM3 -> ARR = 100;
	TIM3 -> CNT = 0;

	//HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);

	HAL_TIM_Base_Stop_IT(&htim3);
	buttonType = 0;
	__HAL_GPIO_EXTI_CLEAR_IT((*button)->pin);  		// очищаем бит EXTI_PR
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);	// очищаем бит NVIC_ICPRx
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);     // включаем внешнее прерывание
}

void handleLongPress(struct buttons ** button)
{
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	(*button)->longFlag = true;

	HAL_TIM_Base_Start_IT(&htim4);
	(*button)->buttonPressed = false;
	EXTI->FTSR |= (*button)->fallingBit;  // Включаем прерывание по нисходящему фронту
	EXTI->RTSR &= ~(*button)->risingBit;
	TIM3 -> ARR = 100;

	HAL_TIM_Base_Stop_IT(&htim3);
	buttonType = 0;
	__HAL_GPIO_EXTI_CLEAR_IT((*button)->pin);  		// очищаем бит EXTI_PR
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);	// очищаем бит NVIC_ICPRx
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);     // включаем внешнее прерывание
}

void callBackButton(struct buttons * button)
{
	if(!(button->buttonPressed))
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		HAL_TIM_Base_Start_IT(&htim3);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	}
	else if(button->buttonPressed)
	{
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		handleShortPress(&button);
	}
	else return;
}



