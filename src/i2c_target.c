/*
 *@Note
 *
 * Polled-mode I2C target device
 *
 * Supports read and write of 8-bit register space
 *
 */

#include "debug.h"

#include <ch32v00x_i2c.h>
#include <ch32v00x_rcc.h>

#include "i2c_target.h"

/*@****************************** Constants *********************************/

#define I2C_PORT GPIOC
#define SDA_PIN  GPIO_Pin_1
#define SCL_PIN  GPIO_Pin_2

/*@******************************* Globals **********************************/

/* 
 * Configuration values
 */
static u32 bus_speed = I2C_BUS_SPEED_DEFAULT;
static u8 dev_addr = 0x00;
static u8 address_size = 1;

#if 0
static int timeout_idle=100;
static int timeout_start=100;
static int timeout_addr_ack=5000;
static int timeout_data_write=500;
static int timeout_read_data_ready=5000;
static int timeout_stop=500;
#endif

static reg_write_cb_t _reg_write_cb=NULL;
static reg_read_cb_t  _reg_read_cb=NULL;

/* 
 * Ephemeral state values
 */
u16 reg_addr = 0;
u16 reg_value = 0;

/*@******************************* Macros  **********************************/

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
	
/*@****************************** Interrupts ********************************/

#if 0
volatile int event_count = 0;
//volatile int last_event_count = 0;
#define EVENT_MAX 8
uint32_t event_buf[EVENT_MAX];

void I2C1_EV_IRQHandler(void) {
    uint32_t event = I2C_GetLastEvent(I2C1);
    if (event_count < EVENT_MAX) {
	event_buf[event_count] = event;
    }
    ++event_count;
}
#endif

#if 0
volatile int error_count = 0;
volatile int last_error_count = 0;
#define EVENT_MAX 8
uint32_t error_buf[EVENT_MAX];


void I2C1_ER_IRQHandler(void) {
    uint32_t event = I2C_GetLastEvent(I2C1);
    if (error_count < EVENT_MAX) {
	error_buf[error_count] = event;
    }
    ++error_count;
}
#endif

/*@****************************** Functions *********************************/
//
// extern and forward decls:
extern void GPIO_Config(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode);

/*@@----------------------------- I2C_Config -------------------------------*/
/* @fn      I2C_Config
 *
 * @brief   Configures the IIC peripheral.
 *
 * @return  none
 */
void I2C_Config(u32 busfreq, u8 device_address, reg_write_cb_t write_cb, reg_read_cb_t read_cb)
{
    printf("> %s %luHz 0x%02x\n", __func__, busfreq, (int)device_address);
    I2C_InitTypeDef  I2C_InitTStructure = {0};

    _reg_write_cb = write_cb;
    _reg_read_cb = read_cb;
    bus_speed = busfreq;
    dev_addr = device_address;
    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Set up PC1 SDA as open collector, left high
    GPIO_Config(I2C_PORT, SDA_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SDA_PIN, 1);

    // Set up SCL as open collector, left high
    GPIO_Config(I2C_PORT, SCL_PIN, GPIO_Mode_AF_OD);
    GPIO_WriteBit(I2C_PORT, SCL_PIN, 1);

    I2C_InitTStructure.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitTStructure.I2C_OwnAddress1 = dev_addr<<1;
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

#if 0
static void print_event(const char *msg, uint32_t event) 
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
#endif

/*@@-------------------------- i2c_handle_read -----------------------------*/
static void i2c_handle_read(uint32_t event) 
{
    //printf("< %s(%08lx)\n", __func__, event);
    // Process a read transaction
    u8 ready = 1;
    uint32_t event_was=0;
    int bytes_sent = 0;
    u8 b;
    

    reg_value = _reg_read_cb(reg_addr);

    while (1) {
#if 0
	if (error_count > last_error_count) {
	    print_event("ERROR", error_buf[error_count-1]);
	    last_error_count = error_count;
	}
#endif
	
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
	    if (bytes_sent == 0) {
		b = reg_value&0xFF;
	    }
	    else if (bytes_sent == 1) {
		b = (reg_value>>8)&0xFF;
	    }
	    else {
		printf("ERR i2c read underflow");
		b=0;
	    }
	    //printf("  %s transmit byte @0x%04x 0x%02x (%d) '%c'\n", __func__, (int)reg_addr, (int)b, (int)b,((b>=' ') && (b<='~'))?b:'?');
	    I2C_SendData(I2C1, b);
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
	    //printf("  %s BTF\n", __func__);
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
static void i2c_handle_write(uint32_t event) 
{
    //printf("< %s(0x%08lx)\n", __func__, event);

    int byte_count = 0;
    int data_count = 0;
    //int recv_max = 8;
    //char recv_buf[9]="DECAFBAD";
    int handled = 0;
    int done = 0;
    reg_value = 0;

    do {
#if 0
	if (error_count > last_error_count) {
	    print_event("ERROR", error_buf[error_count-1]);
	    last_error_count = error_count;
	}
#endif
	
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
		reg_addr &= 0x00FF;
		reg_addr |= (b<<8);
		printf("  %2d AMSB reg_addr=0x%04x\n", (int)byte_count, (int)reg_addr);
	    }
	    else if ((address_size==2) && (byte_count == 1)) {
		// low byte of memory address
		reg_addr &= 0xFF00;
		reg_addr |= (b&0xFF);
		printf("  %2d ALSB reg_addr=0x%04x\n", (int)byte_count, (int)reg_addr);
	    }
	    else if ((address_size==1) && (byte_count == 0)) {
		reg_addr = b;
		//printf("  %2d ADDR reg_addr=0x%04x\n", (int)byte_count, (int)reg_addr);
	    }
	    else {
		// Subsequent bytes, pass to write callback
		//printf("  %2d DATA reg_addr=0x%04x\n", (int)byte_count, reg_addr);
		if (data_count == 0) {
		    reg_value = b;
		}
		else if (data_count == 1) {
		    reg_value |= (b<<8);
		}
		else {
		    printf("ERR i2c write overflow");
		}
		++data_count;
	    }
	    ++byte_count;
	    handled++;
	}

	if ((event & I2C_FLAG_TRA) && (event & I2C_FLAG_ADDR)) {
	    // a second-start switch to read mode
	    i2c_handle_read(event);
	    handled++;
	    done++;
	}

	if ((event & I2C_FLAG_STOPF) && !(event & I2C_FLAG_BUSY)) {
	    //else if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_STOP_DETECTED)) {
	    //printf("  %2d STOP reg_addr=0x%04x\n", (int)byte_count, (int)reg_addr);
	    I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
	    if (byte_count > 1) {
		// don't trigger callback for a zero byte write (often used for bus probing)
		_reg_write_cb(reg_addr, reg_value);
	    }
	    else if(!done) {
		// a no-op write happened (no read)
		//printf("Acknowledge I2C bus probe\n");
	    }
	    
	    handled++;
	    done++;
	}

	if (!handled) {
	    //print_event("WTF", event);
	}

	event=0;
    } while (!done);
}
    
void I2C_Poll(void)
{
    if (!dev_addr) return; // not configured
    
#if 0
    if (error_count > last_error_count) {
	print_event("I2C_ERROR", error_buf[error_count-1]);
	last_error_count = error_count;
    }
#endif
    
    // Check for event
    uint32_t event = I2C_GetLastEvent(I2C1);
    
    // Check for read
    //if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)) {
    if (_reg_write_cb &&
	((event & I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)==I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)) {
	i2c_handle_write(event);
    }
    else if (_reg_read_cb &&
	     ((event & I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)) {
	//if (I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED)) {
	i2c_handle_read(event);
    }
}

// Local Variables:
// mode: C
// c-basic-offset: 4
// End:
