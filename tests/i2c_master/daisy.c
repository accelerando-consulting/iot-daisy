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

#define LED_PORT GPIOC
#define LED_PIN  GPIO_Pin_4

#define PWM_PORT   GPIOA
#define PWM_PIN    GPIO_Pin_2

#define TX_PORT GPIOD
#define TX_PIN  GPIO_Pin_5

#define SDA_PORT   GPIOC
#define SDA_PIN    GPIO_Pin_1
#define SCL_PORT   GPIOC
#define SCL_PIN    GPIO_Pin_2

#define BASE_ADDR 0x50



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

void spkr_test(void) 
{
	for (int cycle=1;cycle<200;cycle++)
	{
		GPIO_WriteBit(PWM_PORT, PWM_PIN, cycle & 0x01);
		Delay_Ms(5);
	}
}

/*
 * The TIM1_PWMOut function initialises Timer 1 and outputs a
 * variable duty cycle PWM signal on GPIO pin D2.
 *
 * For example arr=100 psc=23 ccp=10 gives 19.2khz at 10% duty cycle
 *             arr=100 psc=2 gives 150khz approx
 **/
void TIM1_PWMOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init( TIM1, &TIM_OCInitStructure );

    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
  	USART_Printf_Init( 115200 );
	printf( "\r\nAccelerando Daisy I2C Chiptune\r\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_Config(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
	GPIO_Config(PWM_PORT, PWM_PIN, GPIO_Mode_Out_PP);

	spkr_test();
	winky_loop();

}

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
	while (1)
	{
	}
}
