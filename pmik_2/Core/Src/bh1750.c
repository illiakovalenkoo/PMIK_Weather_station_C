#include "bh1750.h"

// Wewnętrzne wskaźniki do konfiguracji czujnika
static I2C_HandleTypeDef *bh1750_i2c;     // Wskaźnik na uchwyt I2C
static BH1750_Mode bh1750_mode;          // Tryb pracy wybrany przy inicjalizacji
static uint8_t bh1750_address = BH1750_ADDR_LOW;  // Domyślny adres I2C (ADDR = GND)

/**
 * @brief Inicjalizacja czujnika BH1750.
 * @param hi2c - uchwyt do magistrali I2C
 * @param mode - tryb pracy (ciągły / jednorazowy / rozdzielczość)
 */
void BH1750_Init(I2C_HandleTypeDef *hi2c, BH1750_Mode mode) {
    bh1750_i2c = hi2c;
    bh1750_mode = mode;

    uint8_t power_on = 0x01;  // Komenda "Power ON"
    uint8_t reset = 0x07;     // Komenda "Reset"

    HAL_I2C_Master_Transmit(bh1750_i2c, bh1750_address, &power_on, 1, 100);
    HAL_Delay(10); // Czekaj po włączeniu zasilania
    HAL_I2C_Master_Transmit(bh1750_i2c, bh1750_address, &reset, 1, 100);
}

/**
 * @brief Rozpoczęcie pomiaru w trybie jednorazowym (ONE_TIME_*)
 */
void BH1750_StartMeasurement(void) {
    uint8_t cmd = bh1750_mode;
    HAL_I2C_Master_Transmit(bh1750_i2c, bh1750_address, &cmd, 1, 100);
}

/**
 * @brief Odczyt danych świetlnych z czujnika.
 * @return Wartość oświetlenia w luxach (16-bit)
 */
uint16_t BH1750_ReadLux(void) {
    uint8_t data[2]; // Bufor na dane: MSB, LSB
    HAL_Delay(180);  // Maksymalny czas konwersji dla High-Res mode
    HAL_I2C_Master_Receive(bh1750_i2c, bh1750_address, data, 2, 100);
    return (data[0] << 8) | data[1]; // Połączenie bajtów w wartość 16-bitową
}
