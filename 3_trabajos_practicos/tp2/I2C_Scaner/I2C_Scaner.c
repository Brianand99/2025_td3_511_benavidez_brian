#include "stdio.h"
#include "stdlib.h"
#include "hardware/i2c.h"


// Configuración I2C
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5
#define LCD_ADDR 0x27

// Configuración LCD
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE_BIT 0x04
#define LCD_COMMAND 0
#define LCD_CHARACTER 1
#define LCD_CLEAR 0x01
#define LCD_FUNCTIONSET 0x20
#define LCD_2LINE 0x08
#define LCD_ENTRYMODESET 0x04
#define LCD_ENTRYLEFT 0x02
#define LCD_DISPLAYCONTROL 0x08
#define LCD_DISPLAYON 0x04
#define DELAY_US 600

 ---------------- LCD FUNCIONES ---------------- 

void lcd_i2c_write(uint8_t data) {
    i2c_write_blocking(I2C_PORT, LCD_ADDR, &data, 1, false);
}

void lcd_toggle_enable(uint8_t data) {
    sleep_us(DELAY_US);
    lcd_i2c_write(data  LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    lcd_i2c_write(data & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

void lcd_send_byte(uint8_t data, int mode) {
    uint8_t high = mode  (data & 0xF0)  LCD_BACKLIGHT;
    uint8_t low = mode  ((data  4) & 0xF0)  LCD_BACKLIGHT;
    lcd_i2c_write(high);
    lcd_toggle_enable(high);
    lcd_i2c_write(low);
    lcd_toggle_enable(low);
}

void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, LCD_COMMAND);
}

void lcd_char(char val) {
    lcd_send_byte(val, LCD_CHARACTER);
}

void lcd_string(const char str) {
    while (str) {
        lcd_char(str++);
    }
}

void lcd_set_cursor(int row, int col) {
    int pos = (row == 0)  0x80 + col  0xC0 + col;
    lcd_command(pos);
}

void lcd_clear() {
    lcd_command(LCD_CLEAR);
}

void lcd_init() {
    sleep_ms(50);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);
    lcd_command(LCD_FUNCTIONSET  LCD_2LINE);
    lcd_command(LCD_ENTRYMODESET  LCD_ENTRYLEFT);
    lcd_command(LCD_DISPLAYCONTROL  LCD_DISPLAYON);
    lcd_clear();
}

 ---------------- I2C SCAN FUNCION ---------------- 

void scan_i2c_devices() {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string(I2C Scan...);

    sleep_ms(1000);
    uint8_t count = 0;
    char buffer[16];

    for (uint8_t addr = 1; addr  127 && count  2; addr++) {
        uint8_t dummy = 0;
        int result = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, true);

        if (result = 0) {
            snprintf(buffer, sizeof(buffer), Found 0x%02X, addr);
            lcd_set_cursor(count, 0);
            lcd_string(buffer);
            count++;
            sleep_ms(1500);   Mostrar cada dirección por separado
        }
    }

    if (count == 0) {
        lcd_set_cursor(1, 0);
        lcd_string(No devices found);
    }