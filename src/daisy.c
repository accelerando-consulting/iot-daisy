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
#include <string.h>
#include <math.h>

#include "config.h"
#include "i2c_target.h"

/*@****************************** Constants *********************************/

#define LED_PORT GPIOC
#define LED_PIN  GPIO_Pin_4

#define SPK_PORT GPIOD
#define SPK_PIN  GPIO_Pin_6

#define AUX_PORT GPIOA
#define AUX_PIN  GPIO_Pin_2

#define SDA_PORT GPIOC
#define SDA_PIN  GPIO_Pin_1
#define SCL_PORT GPIOC
#define SCL_PIN  GPIO_Pin_2

#define BASE_ADDR 0x50

/*@******************************* Globals **********************************/

uint8_t i2c_addr = BASE_ADDR;

unsigned long tone_period = 0;
unsigned long tone_cycles = 0;
unsigned long tone_cycles_remaining = 0;

/*@**************************** forward decls *******************************/


void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void Delay_Init(void);
void Delay_Ms(uint32_t n);
void update_tone();
void start_tone();
void stop_tone();

/*@************************** Interrupt handlers ****************************/

void NMI_Handler(void) {}
void HardFault_Handler(void)
{
    while (1)
    {
    }
}

/*@**************************** I2C Registers *******************************/


enum reg_id {
    REG_SIG0 = 0,
    REG_SIG1,
    REG_SIG2,
    REG_TYPE,
    REG_ADDR, 
    REG_VERSION, // 5
    REG_BUILD,
    REG_CONFIG,
    REG_STATUS,
    REG_CMD,
    REG_FREQ, // 10
    REG_DURATION,
    REG_COUNT
};

enum cmd {
    CMD_STOP = 0,
    CMD_PLAY = 1
};


const char *reg_name[]={
	"sig0",
	"sig1",
	"sig2",
	"dev_type",
	"dev_addr",
	"dev_vers",
	"dev_build",
	"config",
	"status",
	"cmd",
	"freq",
	"duration"
};


u16 reg[REG_COUNT];

#define REG(id) (reg[REG_##id])
#define REG_INT(id) ((int)reg[REG_##id])
#define REG_BIT(id,b) (((int)reg[REG_##id])&(1<<(BIT_##id##_##b)))

u16 i2c_reg_read(u16 reg_addr) 
{
    if (reg_addr >= REG_COUNT) {
	LOG("ERR %s invalid reg_addr 0x%04x\n", __func__, (int)reg_addr);
	return 0;
    }
    LOG("%s 0x%04x <= 0x%04x (%s)\n", __func__, (int)reg_addr, reg[reg_addr], reg_name[reg_addr]);
    return reg[reg_addr];
}


void i2c_reg_write(u16 reg_addr, u16 reg_data)
{
    LOG("%s 0x%04x <= 0x%04x (%s)\n", __func__, (int)reg_addr, (int)reg_data,
	   (reg_addr < REG_COUNT)?reg_name[reg_addr]:"INVALID");
    if (reg_addr >= REG_COUNT) {
	LOG("ERR %s invalid reg_addr 0x%04x\n", __func__, (int)reg_addr);
	return;
    }
    u8 err_ro=0;
    //u16 reg_data_old = reg[reg_addr];
    
    switch (reg_addr) {
    case REG_SIG0:
    case REG_SIG1:
    case REG_SIG2:
    case REG_TYPE:
    case REG_VERSION:
    case REG_BUILD:
	err_ro=1;
	break;
    case REG_ADDR:
	REG(ADDR) = i2c_addr = reg_data;
	I2C_Config(I2C_BUS_SPEED_DEFAULT, i2c_addr, i2c_reg_write, i2c_reg_read);
	break;
    case REG_CONFIG:
	REG(CONFIG) = reg_data;
	break;
    case REG_STATUS:
	err_ro=1;
	break;
    case REG_CMD:
	switch (reg_data) {
	case CMD_STOP:
	    stop_tone();
	    break;
	case CMD_PLAY:
	    start_tone();
	    break;
	}
	break;
    case REG_FREQ:
	REG(FREQ) = reg_data;
	update_tone();
	break;
    case REG_DURATION:
	REG(DURATION) = reg_data;
	update_tone();
	break;
    }
    if (err_ro) {
	LOG("ERR %s write attempt to readonly reg_addr 0x%04x\n", __func__, (int)reg_addr);
    }
}

void i2c_init(void) 
{
    /* 
     * Initialise default state of i2c registers
     */
    memset(reg, 0, sizeof(reg));
    // SIG0..2 are manufacturer id
    REG(SIG0) = 0x4163; // Ac
    REG(SIG1) = 0x6365; // ce
    REG(SIG2) = 0x6c72; // lr
    REG(TYPE) = 2; // 2 == daisy
    REG(VERSION) = VERSION;
    REG(BUILD) = BUILD_NUMBER;

    REG(FREQ) = 800;
    REG(DURATION) = 100;

    I2C_Config(I2C_BUS_SPEED_DEFAULT, i2c_addr, i2c_reg_write, i2c_reg_read);
}

    
/*@****************************** Functions *********************************/

void GPIO_Config(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode) 
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &GPIO_InitStructure);
}

void update_tone() 
{
    LOG("< %s\n", __func__);
    tone_period = 1000000/REG(FREQ);
    tone_cycles = REG(DURATION)*1000/tone_period;
    LOG("  %s freq=%dHz tone_period=%lu duration=%dms tone_cycles=%lu\n",
	   __func__, REG_INT(FREQ), tone_period, REG_INT(DURATION), tone_cycles);
}

void start_tone() 
{
    LOG("< %s\n", __func__);
    tone_cycles_remaining = tone_cycles;
}

void stop_tone() 
{
    LOG("< %s\n", __func__);
    tone_cycles_remaining = 0;
}

void winky_loop(void) 
{
    LOG("< %s\n", __func__);
    unsigned int cycle = 0;

    while (1)
    {
	++cycle;
	LOG("%-8u\n", cycle);
	if ((cycle % 2)==0) {
	    GPIO_WriteBit(LED_PORT, LED_PIN, 1);
	    GPIO_WriteBit(SPK_PORT, SPK_PIN, 0);
	    Delay_Ms(500);
	}
	else {
	    GPIO_WriteBit(LED_PORT, LED_PIN, 0);
	    for (int i=0; i<250; i++) {
		GPIO_WriteBit(SPK_PORT, SPK_PIN, 1);
		Delay_Ms(1);
		GPIO_WriteBit(SPK_PORT, SPK_PIN, 0);
		Delay_Ms(1);
	    }
	}
    }
}

void spkr_test(void) 
{
	//LOG("< %s\n", __func__);
    GPIO_WriteBit(LED_PORT, LED_PIN, 1);
    for (int cycle=1;cycle<100;cycle++)
    {
	GPIO_WriteBit(SPK_PORT, SPK_PIN, cycle%2);
	Delay_Us(500);
    }
    GPIO_WriteBit(LED_PORT, LED_PIN, 0);
    Delay_Ms(100);
    GPIO_WriteBit(LED_PORT, LED_PIN, 1);
    Delay_Ms(200);
    GPIO_WriteBit(LED_PORT, LED_PIN, 0);
    Delay_Ms(250);
}

void spkr_loop() 
{
    LOG("< %s\n", __func__);
    update_tone();
    unsigned long loops=0;
    
    while (1) {
	I2C_Poll();

	if (tone_cycles_remaining > 0) {
	    GPIO_WriteBit(SPK_PORT, SPK_PIN, 1);
	    Delay_Us(tone_period/2);
	    GPIO_WriteBit(SPK_PORT, SPK_PIN, 0);
	    Delay_Us(tone_period/2);
	    --tone_cycles_remaining;
	}
	else {
	    GPIO_WriteBit(LED_PORT, LED_PIN, ((loops++)<10));
	    if (loops>=1000) loops=0;
	    Delay_Ms(1);
	}
    }
}


/*
 * The TIM1_PWMOut function initialises Timer 1 and outputs a
 * variable duty cycle PWM signal on GPIO pin A2.
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
    GPIO_Init( GPIOA, &GPIO_InitStructure );

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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();
#if SDI_PRINT
    SDI_Printf_Enable();
#elif DEBUG
    #error uart
    USART_LOG_Init( 115200 );
#else
    // no logging
#endif

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PA1_2, ENABLE);

    GPIO_Config(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
    GPIO_WriteBit(LED_PORT, LED_PIN, 0);

    GPIO_Config(AUX_PORT, AUX_PIN, GPIO_Mode_Out_PP);
    GPIO_WriteBit(AUX_PORT, AUX_PIN, 0);

    GPIO_Config(SPK_PORT, SPK_PIN, GPIO_Mode_Out_PP);
    GPIO_WriteBit(SPK_PORT, SPK_PIN, 0);

    Delay_Ms(100);
    //LOG( "\n\n");
    //LOG( "= Accelerando Daisy I2C Chiptune =\n\n");
    //LOG( "  SystemClk:%dHz\n", (int)SystemCoreClock );

    /* 
     * Initialise default state
     */

    i2c_init();

    spkr_test();
    //winky_loop();
    spkr_loop();

}


// Local Variables:
// mode: C
// c-basic-offset: 4
// End:
