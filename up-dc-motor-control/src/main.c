/* DC motor control based on STM32F103
 *
 * R.Grbic, 2020.
 *
 */


/* Includes */
#include "stddef.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "uart.h"
#include "stm32f103_misc.h"
#include "stm32f10x_util.h"
#include "stm32f10x_ina219.h"
#include "time.h"
#include "stdlib.h"

/* global variables */
volatile char receivedChar;
volatile uint16_t currentPWM;
extern const uint16_t PWM_period;
volatile int noMs = 0;
float voltage, current;
char *buff;
volatile int flagMeasure = 0;
const uint32_t numberOfTicks = 41;
const uint32_t freq = 72000000;

volatile uint16_t pulse_ticks = 0;
volatile unsigned long start_time = 0;
volatile unsigned long end_time = 0;

//variables for moving average filter
uint8_t n = 10;
volatile float sampleSize[10] = { 3000.0 }; //previous samples
volatile uint16_t filteredRPMSample = 3000; //filtered sample
volatile float sum = 0.0;

volatile uint32_t RPM;
volatile uint32_t RPM_Ts;

//PRBS variables
volatile int amplitude = 0;
volatile int cycleSize = 100;
volatile int PRBSFlag = 0;
volatile uint32_t start = 0;

volatile uint32_t RPM_Ref = 3500;

volatile int16_t e0 = 0;
volatile int16_t e1 = 0;
volatile int16_t e2 = 0;
volatile int16_t u0 = 0;
volatile int16_t u1 = 0;
volatile int16_t u2 = 0;




/* UART receive interrupt handler */

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		//echo character
		receivedChar = USART_GetChar();
		if(receivedChar == 'u')
		{
			currentPWM = Get_PWM();
			currentPWM += (uint16_t)500;
			if(currentPWM < PWM_period)
				Set_PWM(currentPWM);
		}
		else if(receivedChar == 'd')
		{

			currentPWM = Get_PWM();
			if(currentPWM < PWM_period)
			{
				currentPWM -= (uint16_t)500;
				Set_PWM(currentPWM);
			}
		}

		else if(receivedChar == 'q')
		        {
					start = 1;
		        }

		USART_PutChar(receivedChar);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}


/* TIM2 input capture interrupt */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC4))
	{
		end_time = TIM2->CCR4;
		pulse_ticks = end_time - start_time;
		start_time = end_time;

		for(int i=n-1; i >= 1; i--)
		{
			sampleSize[i] = sampleSize[i-1];
		}

		sampleSize[0] = RPM;

		sum = 0.0;
		for(int i = 0; i < n; i++)
		{
			sum += sampleSize[i];
		}

		//average of current and previous 9
		filteredRPMSample=(uint16_t)(sum/(float)n);


		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	}

}


/* TIMER4 every 0.1 second interrupt --> sending data to PC */
void TIM4_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM4, TIM_IT_Update))
	{
			RPM = (uint32_t)((float)freq*60.0)/(numberOfTicks*pulse_ticks);
			//RPM_Ts = (uint32_t)((numberOfTicks/pulse_ticks)/(float)Ts) * 60;

			u2 = u1;
			u1 = u0;
			e2 = e1;
			e1 = e0;

			e0 = (int16_t)(RPM_Ref - filteredRPMSample);

			//PI reg
			//u0 = 2.744*e0 - 2.201*e1 + u1;
			//u0 = 7.846*e0 - 7.052*e1 + u1;

			//PID reg
			//u0 = 6.017*e0 - 9.394*e1 + 3.658*e2 + 1.606*u1 - 0.606*u2;
			//u0 = 8.207*e0 - 11.77*e1 + 3.941*e2 + 1.563*u1 - 0.5634*u2;
			u0 = 9.892*e0 - 15.99*e1 + 6.465*e2 + 1.546*u1 - 0.5457*u2;


			if (u0 > 13000)
			{
				u0 = 13000;
			}


		USART_PutChar('p');
		USART_PutChar('m');

		USART_SendUInt_32(RPM);
		//USART_SendUInt_32(amplitude);
		USART_SendUInt_32(RPM_Ref);
		//USART_SendUInt_32(Get_PWM());
		//USART_SendUInt_32(filteredRPMSample);
		//USART_SendUInt_32((int32_t)u0);

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/*
void runPRBS()
{
    if(PRBSFlag == 1)
        {
            Set_PWM(8000u);
        }
        else if(PRBSFlag == 0)
        {
            Set_PWM(10000u);
        }
}
*/


/* systick timer for periodic tasks */
void SysTick_Handler(void)
{

	noMs++;

	if(noMs > 15 * 1000){
		RPM_Ref = 5500;
	}
	else if(noMs > 10 * 1000){
		RPM_Ref = 3500;
	}
	else if(noMs > 5 * 1000){
		RPM_Ref = 5000;
	}

	Set_PWM(u0);


/*	if(noMs < period)
	    {
	        noMs++;
	    }
	    else{
	        noMs = 0;
	        if(PRBSFlag == 1)
	        {
	            amplitude = 10000u;
	            PRBSFlag = !PRBSFlag;
	        }
	        else{
	            amplitude = 8000u;
	            PRBSFlag = !PRBSFlag;
	        }
	        cycleSize = 100 + (rand()%500);
	    }
	    if(start == 1)
	    {
	    	runPRBS();
	    }
*/

}


/* main program - init peripherals and loop */
int main(void)
{
	NVIC_SetPriorityGrouping(0u);
	Systick_init();
	Output_setup();
	USART1_PC_Init();
	Timer_setup();
	Button_init();

	while (1)
	{
		// read push button - stop motor
		if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14))
		{
			Set_PWM(0u);
			Delay_ms(50);
		}
		// read push button - turn on motor
		if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15))
		{
			Set_PWM(5000u);
			Delay_ms(50);
		}

	}
}

