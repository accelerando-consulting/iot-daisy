#if defined(CH32V00X)
#include <ch32v00x.h>
#elif defined(CH32V10X)
#include <ch32v10x.h>
#elif defined(CH32V20X)
#include <ch32v20x.h>
#elif defined(CH32V30X)
#include <ch32v30x.h>
#endif

#include <debug.h>
#include <stdlib.h>
#include <math.h> 

#define LED_PORT GPIOD
#define LED_PIN  GPIO_Pin_6

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void Delay_Init(void);
void Delay_Ms(uint32_t n);

void GPIO_Config(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode) 
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

void winky_loop(void) 
{
	unsigned int cycle = 0;

	while (1)
	{
		++cycle;
#ifdef LED_PIN
		GPIO_WriteBit(LED_PORT, LED_PIN, cycle & 0x01);
#endif
		Delay_Ms(500);
	}
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
  	USART_Printf_Init( 115200 );
	printf( "\r\nBlinky\r\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_Config(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);

	winky_loop();

}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
	while (1)
	{
	}
}
