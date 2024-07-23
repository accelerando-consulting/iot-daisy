/*
 *@Note
 *I2C bus scanner
 *
 */

#include "debug.h"
#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>

/* Global define */
#define LED_PORT GPIOD
#define LED_PIN  GPIO_Pin_6

int wait_count;
int timeout_idle=10000;
int timeout_start=1000;
int timeout_addr_ack=1000;

#define WAITFOR(cond, timeout, message) {	\
    for (wait_count=0; (wait_count<(timeout))&&!(cond); wait_count++ ); \
    if (wait_count>=(timeout)) { \
	if ((message) != NULL) printf("TIMEOUT (%d) waiting for %s\n",p, (message)); \
	goto stop; \
    }									\
    if ((wait_count>100) && ((message) != NULL)) { \
	    printf("  waited %d polls for %s\n", wait_count, (message)); \
    } \
    }


/* Global Variable */
vu8 val;

// A 128-bit bitmap for remembering known device presence
uint8_t bus_presence[128/8];
#define IS_PRESENT(x)  (bus_presence[(x)/8] &   (1<<((x)%8)))
#define SET_PRESENT(x) bus_presence[(x)/8] |=  (1<<((x)%8))
#define CLR_PRESENT(x) bus_presence[(x)/8] &= ~(1<<((x)%8))

u8 scan_min=8;
u8 scan_max=120;

void GPIO_CFG(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode) 
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    //GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/* Global define */
#define Size   6
#define RXAddress   0x02
#define TxAddress   0x62

/* Global Variable */
u8 TxData[Size] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
u8 RxData[5][Size];

/*********************************************************************
 * @fn      I2C_CFG
 *
 * @brief   Initializes the I2C peripheral.
 *
 * @return  none
 */
void I2C_CFG(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitTStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    /// SDA : ///
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    /// SCL : ///
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    I2C_InitTStructure.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTStructure.I2C_OwnAddress1 = address;
    I2C_InitTStructure.I2C_ClockSpeed  = bound;
    I2C_InitTStructure.I2C_DutyCycle   = I2C_DutyCycle_16_9;
    I2C_InitTStructure.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitTStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTStructure );

    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );
}

//

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_CFG(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
    GPIO_WriteBit(LED_PORT, LED_PIN, 0);

    USART_Printf_Init(115200);
    printf("\r\n\r\n: %s : \r\n",__FILE__);
    printf(" SystemClk:%lu\r\n", SystemCoreClock);
    printf(" ChipID:%08lx\r\n", DBGMCU_GetDEVID() );

    USARTx_CFG();
    for (int i=0; i < sizeof(bus_presence); i++) {
	bus_presence[i]=0;
    }
    

    //
    // u8  i = 0;
    u8  j = 0;
    // u8  p = 0;
    u8  res ;

    // __IO uint32_t  i2creg=0 ;
    __IO uint32_t   i2cxbase=0 , i2cxbase1=0 , i2cxbase2=0 ;
    i2cxbase = (uint32_t)I2C1 ;  // = (uint32_t)I2Cx;
    //  i2creg = I2C_FLAG >> 28;
    //  I2C_FLAG &= FLAG_Mask;
    //  if(i2creg != 0)
    //  {
    //    i2cxbase += 0x18;
    i2cxbase1 = i2cxbase + 0x14;
    //  }
    //  else
    //  {
    //    I2C_FLAG = (uint32_t)(I2C_FLAG >> 16);
    //    i2cxbase += 0x18;
    i2cxbase2 = i2cxbase + 0x18;
    //  }

    printf("> main(): I2C_Init TxAddress=0x%02x\r\n", (int)TxAddress);
    I2C_CFG( 80000, TxAddress);



    u8 present;
    u32 loops = 0;
    
    while( 1 )
    {
	u8 changed = 0;
	
	for( j=scan_min; j <= scan_max ; j++ )
	{
	    int p; // poll counter
	    
	    // Wait for bus idle
	    WAITFOR(I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) == RESET, timeout_idle, "bus idle");
	    
	    //printf("= main(): ->I2C:Start()\r\n");
	    I2C_GenerateSTART( I2C1, ENABLE );
	    WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT),timeout_start,"start sent");

	    //printf("  = main(): ->I2C:SendAdrId() = x%02X p=%d\r\n" , j, p );
	    present = 0;
	    I2C_Send7bitAddress( I2C1, j<<1, I2C_Direction_Transmitter );

	    //WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), timeout_addr_ack, "address acknowledge");
	    WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), timeout_addr_ack, NULL);
	    present = 1;

	    //printf("  %3d (0x%02x)  => p=%d MTM=%d ", (int)j, (int)j, p, res );
	    res = I2C_CheckEvent( I2C1, I2C_Direction_Transmitter ) ;
	    //printf(", DirT=%d " , res );
	    Delay_Ms(1);

	    res = I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ;
	    //printf(", MTM=%d " , res );
	    //      res = I2C_GetFlagStatus( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) ;
	    //res = (*(__IO uint32_t *) i2cxbase1);
	    //printf(", St=0x%02x " , res );
	    //res = (*(__IO uint32_t *) i2cxbase2);
	    //printf("0x%02x " , res );

	    // while( I2C_GetFlagStatus( I2C1, I2C_FLAG_TXE ) ==  RESET ) {}
	    // Delay_Ms(100);
	    // I2C_SendData( I2C1, TxData[i] );
	    // while( ! I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) )  {}

	stop:
		
	    //printf("\r\n= main(): ->I2C:Stop() \r\n");
	    I2C_GenerateSTOP( I2C1, ENABLE );
	    // res = I2C_GetFlagStatus( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) ;
	    res = (*(__IO uint32_t *) i2cxbase1);
	    //printf("      , St=%d " , res );
	    res = (*(__IO uint32_t *) i2cxbase2);
	    //printf("%d \r\n" , res );

	    if (present && !IS_PRESENT(j)) {
		printf("** NEW DEVICE 0x%02x (%d) ** \n", (int)j, (int)j);
		SET_PRESENT(j);
		++changed;
	    }
	    else if (!present && IS_PRESENT(j)) {
		printf("** BYE DEVICE 0x%02x (%d) **\n", (int)j, (int)j);
		CLR_PRESENT(j);
		++changed;
	    }


	    Delay_Ms(1);
	}

	//printf("\r\n---\r\n\r\n");
	
	GPIO_WriteBit(LED_PORT, LED_PIN, 1);

	if (changed || ((++loops%20)==0)) {
	    int n = 0;
	    printf("Present:");
	    for (int i=0;i<128;i++) {
		if (IS_PRESENT(i)) {
		    printf(" 0x%02x", i);
		    n++;
		}
	    }
	    if (n==0) printf(" NONE");
	    printf("\n");
	}
	Delay_Ms(100);
	GPIO_WriteBit(LED_PORT, LED_PIN, 0);

    }  // - while(1) .


    
    while( 1 )
    {
	;
    }

    /*
      while(1)
      {
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
      {
      // waiting for receiving finish //
      }
      val = (USART_ReceiveData(USART1));
      USART_SendData(USART1, ~val);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      {
      // waiting for sending finish //
      }
      }
    */
}

// local Variables:
// mode: C
// c-basic-offset: 4
// End:
