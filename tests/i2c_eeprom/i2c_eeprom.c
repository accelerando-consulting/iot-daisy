#include "debug.h"
#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>

#define WKEY_PORT GPIOC
#define WKEY_PIN GPIO_Pin_0

#define RKEY_PORT GPIOD
#define RKEY_PIN GPIO_Pin_0

#define I2C_PORT GPIOC
#define SDA_PIN  GPIO_Pin_1
#define SCL_PIN  GPIO_Pin_2

u8 dev_addr = 0x50;
u8 address_size = 2;

int timeout_idle=100;
int timeout_start=100;
int timeout_addr_ack=5000;
int timeout_data_write=500;
int timeout_read_data_ready=5000;
int timeout_stop=500;


#define WAITFOR(cond, timeout, message) {				\
	int waited;							\
	for (waited=0; (waited < (timeout)) && !(cond); waited++);	\
	if (waited >= (timeout)) {					\
	    if (message && (message[0] != '@')) {			\
		printf("** TIMEOUT (%d) for %s\n", waited, (message));	\
	    }								\
	    break;							\
	}								\
	if ((waited >= (timeout/2)) &&					\
	    (message && (message[0] != '@'))) {				\
	    printf("  waited %dx for %s\n", waited, (message));		\
	}								\
    }


void GPIO_Config(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode) 
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}


void I2C_Config(u32 bound, u16 address)
{
    I2C_InitTypeDef I2C_InitTStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Set up PC1 SDA as open collector, left high
    GPIO_Config(I2C_PORT, SDA_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SDA_PIN, 1);

    // Set up SCL as open collector, left high
    GPIO_Config(I2C_PORT, SCL_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SCL_PIN, 1);

    I2C_InitTStructure.I2C_ClockSpeed = bound;
    I2C_InitTStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTStructure.I2C_OwnAddress1 = address;
    I2C_InitTStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTStructure);

    I2C_Cmd(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
}



void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
    //printf("%s 0x%04x <= 0x%02x '%c'\n", __func__, (int)WriteAddr, (int)DataToWrite, DataToWrite);
    // turn auto-ack back on
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    do {
	// wait for bus idle
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET, timeout_idle, "bus idle");

	// Send START, Wait for master mode ready
	I2C_GenerateSTART(I2C1, ENABLE);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT), timeout_start, "start sent");

	// Send the device address, wait for ACK
	I2C_Send7bitAddress(I2C1, dev_addr<<1|1, I2C_Direction_Transmitter);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), timeout_addr_ack, "address ack");

	// Send the memory address (one or two bytes), waiting for ack of each
	if (address_size == 2) {
	    // high byte of register address
	    I2C_SendData(I2C1, (u8)(WriteAddr>>8));
	    WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeout_data_write, "register msb ack");
	}

	// Low byte of regisgter address
	I2C_SendData(I2C1, (u8)(WriteAddr&0x00FF));
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeout_data_write, "register lsb ack");

	// Wait for data ready
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) !=  RESET, timeout_data_write, "data ready");

	// Send data, wait for byte to drain
        I2C_SendData(I2C1, DataToWrite);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeout_data_write, "data byte ack");

    } while (0);  // this while loop allows WAITFOR to use 'break'

    do {
	// Send STOP, wait for bus to settle
	I2C_GenerateSTOP(I2C1, ENABLE);
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET, timeout_stop, "stop");
    } while (0);
    
}

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
    int temp=-1;
    //printf("  %s @ 0x%04x <= ", __func__, (int)ReadAddr);

    // turn auto-ack back on
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    do {
	// Wait for bus to be idle
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET, timeout_idle, "bus idle");

	// Send start, wait for start condition
	I2C_GenerateSTART(I2C1, ENABLE);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT), timeout_start, "start sent");

	// Send device address, wait for ack
	I2C_Send7bitAddress(I2C1, (dev_addr<<1)|1, I2C_Direction_Transmitter);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), timeout_addr_ack, "address ack");

	// Send register MSB, wait for ACK
	if (address_size > 1) {
	    I2C_SendData(I2C1, (u8)(ReadAddr>>8));
	    WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeout_data_write, "register msb ack");
	}

	// Send register LSB, wait for ack
	I2C_SendData(I2C1, (u8)(ReadAddr&0x00FF));
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeout_data_write, "register lsb ack");

	// Send re-START for read mode
	I2C_GenerateSTART(I2C1, ENABLE);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT), timeout_start, "second start sent");


	// Disable auto ack (tell the device we want only one byte)
	// (Do this before sending the address because logic capture shows the
	// device may spam us with two or more bytes until we stop ACKing)
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	// Send device ID again, but read mode
	I2C_Send7bitAddress(I2C1, (dev_addr<<1), I2C_Direction_Receiver);
	WAITFOR(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), timeout_addr_ack, "address ack");


	// Wait for data ready condition
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE), timeout_read_data_ready, "data ready");


	// Read the byte
	temp = I2C_ReceiveData(I2C1);

    } while (0);
    
    do {
	I2C_GenerateSTOP(I2C1, ENABLE);
	WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET, timeout_stop, "stop");
    } while (0);

    if (temp >= 0) {
	//printf(" 0x%02x '%c'\n", temp, temp);
    }
    else {
	temp=0;
    }
    
    return temp;
}


/*********************************************************************
 * @fn      AT24CXX_Read
 *
 * @brief   Read multiple data from EEPROM.
 *
 * @param   ReadAddr - Read frist address. (AT24c02: 0~255)
 *          pBuffer - Read data.
 *          NumToRead - Data number.
 *
 * @return  none
 */
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{
    
    while(NumToRead)
    {
        *pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);
        NumToRead--;
        Delay_Us(100); // give the scope trace a bit of margin
    }
}

/*********************************************************************
 * @fn      AT24CXX_Write
 *
 * @brief   Write multiple data to EEPROM.
 *
 * @param   WriteAddr - Write frist address. (AT24c02: 0~255)
 *          pBuffer - Write data.
 *          NumToWrite - Data number.
 *
 * @return  none
 */
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
{
    while(NumToWrite--)
    {
        AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
        WriteAddr++;
        pBuffer++;
        Delay_Ms(2); // writing to eeprom is rather slow!
    }
}

int main(void)
{
    u16 initial_pos = 100;
    u16 pos = initial_pos;
    u8 data[33]="DEADBEAFDEADBEEFDEADBEEFDEADBEEF";
    char payload[]="Hello, World!";
    int len = 0;
    while (payload[len]) len++;
    int write_len = len;
    int read_len = len;
    
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("\033[2J\033[H"); // ANSI clear screen, home cursor
    //USARTx_CFG();
    printf(": %s : \n", __FILE__);
    printf("   SystemClk:%lu\n", SystemCoreClock);
    printf("   DevID:%08lx\n", DBGMCU_GetDEVID());
    printf("\n");
    Delay_Ms(1000);

    printf("= GPIO Config\n");
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_Config(WKEY_PORT, WKEY_PIN, GPIO_Mode_IPU);
    GPIO_Config(RKEY_PORT, RKEY_PIN, GPIO_Mode_IPU);

    printf("= I2C Config\n");
    I2C_Config(100000, 0x50);

    int loops=0;
    // Wait for bus idle
    while (1) {
	char scl = GPIO_ReadInputDataBit(I2C_PORT, SCL_PIN)?'C':'c';
	char sda = GPIO_ReadInputDataBit(I2C_PORT, SDA_PIN)?'D':'d';
	printf("= Wait for key PC0=rd PC1=wr (loop %d, %c%c)\n", ++loops, scl, sda);
	int r = 1;
	int w = 1;

	while ((r=GPIO_ReadInputDataBit(WKEY_PORT, WKEY_PIN)) &&
	       (w=GPIO_ReadInputDataBit(RKEY_PORT, RKEY_PIN))
	    ); // wait for one of the two command buttons to be pressed
	Delay_Ms(100);
	while (!GPIO_ReadInputDataBit(WKEY_PORT, WKEY_PIN) &&
	       !GPIO_ReadInputDataBit(RKEY_PORT, RKEY_PIN)
	    ); // wait for release of all buttons


	printf("  wait for bus idle\n");
	do {
	    WAITFOR(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == RESET, timeout_idle, "bus idle");

	    if (w == 0) {
		// write button was pressed
		printf("\nAT24CXX_Write %d:[%s] @ 0x%04X\n", write_len, payload, (int)pos);
		AT24CXX_Write(pos, (u8*)payload, write_len);
		Delay_Ms(10);
	    }


	    if (r == 0) {
		// read button was pressed
		printf("\nAT24CXX_Read %d @ 0x%04x\n", read_len, (int)pos);
		AT24CXX_Read(pos, (u8*)data, read_len);

		data[read_len] = '\0';
		printf("  result: %s\n", data);
	    }
	    
	} while (0);
	printf("\n\n");
	++pos; // operate one char further in next time

    }
}

// Local Variables:
// mode: C
// c-basic-offset: 4
// End:
