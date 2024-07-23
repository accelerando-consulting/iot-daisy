/*
 *@Note
 *
 * I2C target (sl*v*) example
 *
 * Pretends to be a 32-byte I2C EEPROM, with 2-byte addressing
 *
 *
 */

#include "debug.h"

#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>

/*@****************************** Constants *********************************/

#define I2C_PORT GPIOC
#define SDA_PIN  GPIO_Pin_1
#define SCL_PIN  GPIO_Pin_2

#define LED_PORT GPIOC
#define LED1 GPIO_Pin_3
#define LED2 GPIO_Pin_4
#define LED3 GPIO_Pin_5
#define LED4 GPIO_Pin_6
#define LED5 GPIO_Pin_7

#define EEPROM_SIZE 16

const u16 dev_addr = 0x0;
const u32 bus_speed = 20000;
u8 address_size = 1;

int timeout_idle=100;
int timeout_start=100;
int timeout_addr_ack=5000;
int timeout_data_write=500;
int timeout_read_data_ready=5000;
int timeout_stop=500;

/*@******************************* Globals **********************************/

u8 fake_eeprom[EEPROM_SIZE] = "DEADBEEFDEADBEEF";
u16 eeprom_addr = 0;


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
	
void LED_ON (int pin) { GPIO_WriteBit(LED_PORT, pin, 0); }
void LED_OFF(int pin) { GPIO_WriteBit(LED_PORT, pin, 1); }

volatile int event_count = 0;
#define EVENT_MAX 8
uint32_t event_buf[EVENT_MAX];

volatile int error_count = 0;
//volatile int last_event_count = 0;
volatile int last_error_count = 0;
#define EVENT_MAX 8
uint32_t error_buf[EVENT_MAX];

void I2C1_EV_IRQHandler(void) {
    LED_ON(LED1);
    uint32_t event = I2C_GetLastEvent(I2C1);
    if (event_count < EVENT_MAX) {
	event_buf[event_count] = event;
    }
    ++event_count;
}

void I2C1_ER_IRQHandler(void) {
    LED_ON(LED5);
    uint32_t event = I2C_GetLastEvent(I2C1);
    if (error_count < EVENT_MAX) {
	error_buf[error_count] = event;
    }
    ++error_count;
}


/*@****************************** Functions *********************************/

/*@@---------------------------- GPIO_Config -------------------------------*/
void GPIO_Config(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode) 
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

/*@@----------------------------- I2C_Config -------------------------------*/
/* @fn      I2C_Config
 *
 * @brief   Configures the IIC peripheral.
 *
 * @return  none
 */
void I2C_Config(u32 busfreq, u16 address)
{
    printf("> %s %luHz 0x%04x\n", __func__, busfreq, (int)address);
    I2C_InitTypeDef  I2C_InitTStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Set up PC1 SDA as open collector, left high
    GPIO_Config(I2C_PORT, SDA_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SDA_PIN, 1);

    // Set up SCL as open collector, left high
    GPIO_Config(I2C_PORT, SCL_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SCL_PIN, 1);

    I2C_InitTStructure.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTStructure.I2C_OwnAddress1 = address<<1;
    I2C_InitTStructure.I2C_ClockSpeed  = busfreq;
    //I2C_InitTStructure.I2C_DutyCycle   = I2C_DutyCycle_16_9;
    I2C_InitTStructure.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitTStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTStructure);

    I2C_StretchClockCmd(I2C1, ENABLE);
#if 0
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    NVIC_EnableIRQ(I2C1_EV_IRQn); // Event interrupt
    NVIC_SetPriority(I2C1_EV_IRQn, 2 << 4);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
    NVIC_EnableIRQ(I2C1_ER_IRQn); // Error interrupt
    NVIC_SetPriority(I2C1_ER_IRQn, 2 << 4);
#endif


    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE); // auto ack our target address

    printf("< %s\n", __func__);
}

void print_event(const char *msg, uint32_t event) 
{
    printf("%s 0x%08lx", msg, event);
#if 0
#define ISIT(f) if (event & I2C_IT_##f) printf(" i%s", #f)    
    ISIT(PECERR);
    ISIT(OVR);
    ISIT(AF);
    ISIT(ARLO);
    ISIT(BERR);
    ISIT(TXE);
    ISIT(RXNE);
    ISIT(STOPF);
    ISIT(ADD10);
    ISIT(BTF);
    ISIT(ADDR);
    ISIT(SB);
#endif
#define ISFLAG(f) if (event & I2C_FLAG_##f) printf(" %s", #f)    
    ISFLAG(DUALF);
    ISFLAG(GENCALL);
    ISFLAG(TRA);
    ISFLAG(BUSY);
    ISFLAG(PECERR);
    ISFLAG(OVR);
    ISFLAG(AF);
    ISFLAG(ARLO);
    ISFLAG(BERR);
    ISFLAG(TXE);
    ISFLAG(RXNE);
    ISFLAG(STOPF);
    ISFLAG(ADD10);
    ISFLAG(BTF);
    ISFLAG(ADDR);
    ISFLAG(SB);
    printf("\n");
}
    


/*@@-------------------------- i2c_handle_read -----------------------------*/
void i2c_handle_read(uint32_t event) 
{
    //printf("< %s(%08lx)\n", __func__, event);
    // Process a read transaction
    u8 ready = 1;
    uint32_t event_was=0;
    int bytes_sent = 0;
    while (1) {
	if (error_count > last_error_count) {
	    print_event("ERROR", error_buf[error_count-1]);
	    last_error_count = error_count;
	}

	if (!event) event = I2C_GetLastEvent(I2C1);
	if (event != event_was) {
	    //print_event("RX EVENT ", event);
		event_was = event;
	}

	if (event & I2C_FLAG_AF) {
	    // byte was not acked, stop transmitting
	    //printf("  %s received NAK bytes_sent=%d\n", __func__, bytes_sent);
	    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	    break;
	}

	if (ready) {
	    u8 b = fake_eeprom[eeprom_addr];
	    //printf("  %s transmit byte @0x%04x 0x%02x (%d) '%c'\n", __func__, (int)eeprom_addr, (int)b, (int)b,((b>=' ') && (b<='~'))?b:'?');
	    I2C_SendData(I2C1, b);
	    eeprom_addr++;
	    bytes_sent++;
	}

#if 0
	if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_STOP_DETECTED)) {
	    printf("  %s received STOP\n", __func__);
	    break;
	}
	else
#endif
        if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_BYTE_TRANSMITTED)) {
	    printf("  %s BTF\n", __func__);
	    I2C_ClearFlag(I2C1, I2C_FLAG_TXE);
	    ready = 1;
	}
	else if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_BYTE_TRANSMITTING)) {
	    // still going
	    ready = 0;
	}
	else {
	    // uh, can this happen?
	    ready = 0;
	}
	event = 0;
    }
}

/*@@-------------------------- i2c_handle_write ----------------------------*/
// @brief Process a write transaction
void i2c_handle_write(uint32_t event) 
{
    //printf("< %s(0x%08lx)\n", __func__, event);

    int byte_count = 0;
    //int recv_max = 8;
    //char recv_buf[9]="DECAFBAD";
    int handled = 0;
    int done = 0;

    do {
	if (error_count > last_error_count) {
	    print_event("ERROR", error_buf[error_count-1]);
	    last_error_count = error_count;
	}

	if (!event) event = I2C_GetLastEvent(I2C1);
	//printf("  %2d ", byte_count);
	//print_event("EVENT ", event);
	handled = 0;

	if (event & I2C_FLAG_RXNE) {
	    //if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_BYTE_RECEIVED)) {
	    
	    u8 b = I2C_ReceiveData(I2C1);
	    //if (byte_count < recv_max) recv_buf[byte_count]=b;
	    //printf("  %2d RCVD 0x%02x (%d) '%c'\n", byte_count, (int)b, (int)b,((b>=' ') && (b<='~'))?b:'?');
	    if ((address_size == 2) && (byte_count == 0)) {
		// high byte of memory address
		eeprom_addr &= 0x00FF;
		eeprom_addr |= (b<<8);
		printf("  %2d AMSB eeprom_addr=0x%04x\n", (int)byte_count, (int)eeprom_addr);
	    }
	    else if ((address_size==2) && (byte_count == 1)) {
		// low byte of memory address
		eeprom_addr &= 0xFF00;
		eeprom_addr |= (b&0xFF);
		if (eeprom_addr > EEPROM_SIZE) {
		    eeprom_addr = eeprom_addr % EEPROM_SIZE;
		}
		printf("  %2d ALSB eeprom_addr=0x%04x\n", (int)byte_count, (int)eeprom_addr);
	    }
	    else if ((address_size==1) && (byte_count == 0)) {
		eeprom_addr = b;
		if (eeprom_addr > EEPROM_SIZE) {
		    eeprom_addr = eeprom_addr % EEPROM_SIZE;
		}
		//printf("  %2d ADDR eeprom_addr=0x%04x\n", (int)byte_count, (int)eeprom_addr);
		LED_ON(LED3);
	    }
	    else {
		// Subsequent bytes, write into memory
		while (eeprom_addr > EEPROM_SIZE) eeprom_addr-=EEPROM_SIZE;
		//printf("  %2d DATA eeprom_addr=0x%04x\n", (int)byte_count, eeprom_addr);
		fake_eeprom[eeprom_addr++] = b;
		LED_ON(LED4);
	    }
	    ++byte_count;
	    handled++;
	}

	if ((event & I2C_FLAG_TRA) && (event & I2C_FLAG_ADDR)) {
	    // a second-start switch to read mode
	    LED_ON(LED5);
	    i2c_handle_read(event);
	    handled++;
	    done++;
	}

	if ((event & I2C_FLAG_STOPF) && !(event & I2C_FLAG_BUSY)) {
	    //else if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_STOP_DETECTED)) {
	    //printf("  %2d STOP eeprom_addr=0x%04x\n", (int)byte_count, (int)eeprom_addr);
	    I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
#if 0

	    printf("     fake_eeprom=");
	    for (int i=0; i<EEPROM_SIZE; i++) {
		char c = fake_eeprom[i];
		printf("%c", ((c>=' ') && (c<='~'))?fake_eeprom[i]:'.');
	    }
	    printf(" recv_buf=");
	    for (int i=0; (i<byte_count) && (i<recv_max);i++) {
		printf(" %02X", (int)recv_buf[i]);
	    }
	    printf("\n");
#endif
	    handled++;
	    done++;
	}

	if (!handled) {
	    //print_event("WTF", event);
	}

	event=0;
    } while (!done);
}
    

/*@@-------------------------------- main ----------------------------------*/
/* @fn      main
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

    USART_Printf_Init(115200);
    //printf("\n\n\n");
    printf("\033[2J\033[H"); // ANSI clear screen, home cursor
    printf(": %s : \n", __FILE__);
    printf("  SystemClk:%lu\n", SystemCoreClock);
    printf("  DevID:%08lx\n\n", DBGMCU_GetDEVID());

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_Config(LED_PORT, LED1, GPIO_Mode_Out_PP);
    GPIO_Config(LED_PORT, LED2, GPIO_Mode_Out_PP);
    GPIO_Config(LED_PORT, LED3, GPIO_Mode_Out_PP);
    GPIO_Config(LED_PORT, LED4, GPIO_Mode_Out_PP);
    GPIO_Config(LED_PORT, LED5, GPIO_Mode_Out_PP);
    LED_OFF(LED1);
    LED_OFF(LED2);
    LED_OFF(LED3);
    LED_OFF(LED4);
    LED_OFF(LED5);

    // blink led1 on to confirm we've booted, then turn off when ready
    LED_ON(LED1);
    LED_ON(LED2);
    Delay_Ms(1000);
    I2C_Config(bus_speed, dev_addr);

    LED_OFF(LED1);
    Delay_Ms(500);
    LED_OFF(LED2);

    int loops = 0;
    int last_write=0;
    int last_read=0;
    int led_delay = 500;

    char scl;
    char sda;
    

    uint32_t event_was=0;
    
    while(1)
    {
	LED_OFF(LED3);
	LED_OFF(LED4);
	LED_OFF(LED5);

	if (error_count > last_error_count) {
	    print_event("ERROR", error_buf[error_count-1]);
	    last_error_count = error_count;
	}

	// Check for event
	uint32_t event = I2C_GetLastEvent(I2C1);

	if ((loops==0) ||
	    ((last_write && (loops == last_write+1)) || (last_read && (loops == last_read+1)))
		) {
	    scl = GPIO_ReadInputDataBit(I2C_PORT, SCL_PIN)?'C':'c';
	    sda = GPIO_ReadInputDataBit(I2C_PORT, SDA_PIN)?'D':'d';
	    printf("= target waiting (%c%c)", scl, sda);
	    printf(" fake_eeprom=");
	    for (int i=0; i<EEPROM_SIZE; i++) {
		    char c = fake_eeprom[i];
		    printf("%c", ((c>=' ') && (c<='~'))?fake_eeprom[i]:'.');
	    }
	    printf(" addr=%d",eeprom_addr);
	    print_event(" event", event);
	    event_was=event;
	}

	if (event != event_was) {
	    //print_event("EVENT", event);
	    event_was=event;
	}

	if (event & I2C_FLAG_STOPF) {
	    //printf(" clear stop\n");
	    //I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
	    event &= ~ I2C_FLAG_STOPF;
	}
	
	// Check for read
	//if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)) {
	if ((event & I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)==I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) {
	    LED_ON(LED2);
	    i2c_handle_write(event); // expect an address and data
	    last_write = loops;
	}
	else if ((event & I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) {
	    //if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED)) {
	    LED_ON(LED1);
	    i2c_handle_read(event);
	    last_read = loops;
	}
	else if (event == I2C_FLAG_BUSY) {
	    // busis in use, address not complete
	}
	else if (event == 0x10) {
	    //printf("STAHP\n");
	    //I2C_ClearFlag(I2C1, I2C_FLAG_SB);
	}
	else if (event && (event != event_was)) {
	    //printf("Unhandled I2C event mask 0x%08lx\n", event);
	}
	    
	++loops;
	if (last_read && (loops >= (last_read  + led_delay))) GPIO_WriteBit(LED_PORT, LED1, 1);
	if (last_write && (loops >= (last_write + led_delay))) GPIO_WriteBit(LED_PORT, LED2, 1);
    }

}

// Local Variables:
// mode: C
// c-basic-offset: 4
// End:
