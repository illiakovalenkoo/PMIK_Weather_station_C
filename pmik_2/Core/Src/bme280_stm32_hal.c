#include "bme280_stm32_hal.h"
#include "stm32f4xx_hal.h"  // <- używasz STM32F4, nie F1!

extern I2C_HandleTypeDef hi2c1;

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *(uint8_t *)intf_ptr;
    if (HAL_I2C_Mem_Read(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
        return 0;
    return -1;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *(uint8_t *)intf_ptr;
    if (HAL_I2C_Mem_Write(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100) == HAL_OK)
        return 0;
    return -1;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    HAL_Delay((period + 999) / 1000); // zamiana us → ms
}
