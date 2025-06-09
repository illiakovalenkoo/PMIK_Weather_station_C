#ifndef BME280_STM32_HAL_H
#define BME280_STM32_HAL_H

#include "bme280.h"

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

#endif // BME280_STM32_HAL_H
