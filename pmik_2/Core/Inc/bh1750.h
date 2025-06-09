#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "stm32f4xx_hal.h" // Biblioteka HAL dla konkretnej rodziny STM32 (tu: F4)

// Adresy I2C czujnika BH1750, zależne od poziomu na pinie ADDR
#define BH1750_ADDR_LOW  (0x23 << 1)   // ADDR do GND – adres 0x46
#define BH1750_ADDR_HIGH (0x5C << 1)   // ADDR do VCC – adres 0xB8

// Dostępne tryby pracy BH1750
typedef enum {
    BH1750_CONT_HIGH_RES_MODE     = 0x10, // Ciągły pomiar – wysoka rozdzielczość (1 lx)
    BH1750_CONT_HIGH_RES_MODE_2   = 0x11, // Ciągły pomiar – wysoka rozdzielczość (0.5 lx)
    BH1750_CONT_LOW_RES_MODE      = 0x13, // Ciągły pomiar – niska rozdzielczość (4 lx)
    BH1750_ONE_TIME_HIGH_RES_MODE = 0x20, // Jednorazowy pomiar – wysoka rozdzielczość
    BH1750_ONE_TIME_HIGH_RES_MODE_2 = 0x21, // Jednorazowy – 0.5 lx
    BH1750_ONE_TIME_LOW_RES_MODE  = 0x23  // Jednorazowy – 4 lx
} BH1750_Mode;

// Inicjalizacja czujnika BH1750 (podanie uchwytu I2C i trybu pracy)
void BH1750_Init(I2C_HandleTypeDef *hi2c, BH1750_Mode mode);

// Wywołanie rozpoczęcia pomiaru w trybie ONE_TIME_*
void BH1750_StartMeasurement(void);

// Odczyt danych z czujnika (wartość w luxach)
uint16_t BH1750_ReadLux(void);

#endif /* INC_BH1750_H_ */
