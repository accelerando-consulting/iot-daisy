#pragma once

#define I2C_BUS_SPEED_DEFAULT 80000

typedef void (*reg_write_cb_t)(u16 reg_addr, u16 data);
typedef u16 (*reg_read_cb_t)(u16 reg_addr);

extern void I2C_Config(u32 busfreq, u8 device_address, reg_write_cb_t write_cb, reg_read_cb_t read_cb);
extern void I2C_Poll(void);

