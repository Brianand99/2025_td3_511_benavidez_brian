#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

// Definir los pines para el L298N (control de motores)
#define IN1_PIN 7   // Motor A - Pin IN1
#define IN2_PIN 8   // Motor A - Pin IN2
#define IN3_PIN 9   // Motor B - Pin IN3
#define IN4_PIN 10  // Motor B - Pin IN4

// Configuración UART para el módulo Bluetooth
#define UART_ID uart0
#define BAUD_RATE 9600
#define TX_PIN 17  // UART0 TX (GPIO 17)
#define RX_PIN 16  // UART0 RX (GPIO 16)

void setup_motor_pins() {
    gpio_init(IN1_PIN);
    gpio_init(IN2_PIN);
    gpio_init(IN3_PIN);
    gpio_init(IN4_PIN);
    gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_set_dir(IN2_PIN, GPIO_OUT);
    gpio_set_dir(IN3_PIN, GPIO_OUT);
    gpio_set_dir(IN4_PIN, GPIO_OUT);
}

void move_forward() {
    gpio_put(IN1_PIN, 1);
    gpio_put(IN2_PIN, 0);
    gpio_put(IN3_PIN, 1);
    gpio_put(IN4_PIN, 0);
}

void move_backward() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 1);
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 1);
}

void turn_left() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 1);
    gpio_put(IN3_PIN, 1);
    gpio_put(IN4_PIN, 0);
}

void turn_right() {
    gpio_put(IN1_PIN, 1);
    gpio_put(IN2_PIN, 0);
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 1);
}

void stop_moving() {
    gpio_put(IN1_PIN, 0);
    gpio_put(IN2_PIN, 0);
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 0);
}

void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(RX_PIN);  // Activar resistores pull-up en RX
}

void process_command(char command) {
    switch (command) {
        case 'F':
            move_forward();
            break;
        case 'B':
            move_backward();
            break;
        case 'L':
            turn_left();
            break;
        case 'R':
            turn_right();
            break;
        case 'S':
            stop_moving();
            break;
        default:
            stop_moving();
            break;
    }
}

int main() {
    stdio_init_all();      // Inicializar la comunicación serial
    setup_uart();          // Inicializar UART para Bluetooth
    setup_motor_pins();    // Configurar los pines de los motores

    char received_char;

    while (1) {
        if (uart_is_readable(UART_ID)) {
            received_char = uart_getc(UART_ID);  // Leer el comando del Bluetooth

            // Filtrar caracteres no deseados como saltos de línea o espacios
            if (received_char == '\n' || received_char == '\r' || received_char == ' ') {
                continue;  // Ignorar estos caracteres
            }

            printf("Comando recibido: %c\n", received_char);  // Mostrar el comando recibido
            process_command(received_char);  // Procesar el comando

            sleep_ms(200);  // Añadir un pequeño retardo para evitar lecturas repetidas
        }
    }

    return 0;
}

*/

// Programa para utilizar MPU6050
/*
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lcd.h"
#include "helper.h"
#include "FreeRTOS.h"

#define I2C_DEFAULT i2c0

// Definir pines para I2C
#define SDA_PIN 4    // Pin SDA (GPIO4)
#define SCL_PIN 5    // Pin SCL (GPIO5)

// Dirección del MPU-6050
static int MPU6050_ADDR = 0x69;  // Dirección I2C del MPU-6050 (por defecto 0x68)

uint16_t accel[3];
uint16_t gyro[3];


//  ------- Funciones --------//

// Inicializar el sensor
static void mpu6050_init() {
    // Despertar el sensor (escribir 0x00 en el registro PWR_MGMT_1)
    uint8_t power_mgmt_1 = 0x00;  
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &power_mgmt_1, 1, false);
    sleep_ms(100);  // Esperar 100 ms para asegurarnos de que el sensor esté listo

    // Configurar el registro 0x1C (ACONFIG) para establecer el rango del acelerómetro (ejemplo ±2g)
    uint8_t accel_config = 0x00;  // ±2g
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &accel_config, 1, false);

    // Configurar el registro 0x1B (GYRO_CONFIG) para establecer el rango del giroscopio (ejemplo ±250°/s)
    uint8_t gyro_config = 0x00;  // ±250°/s
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &gyro_config, 1, false);

    // Configurar el registro 0x19 (SMPLRT_DIV) para configurar la tasa de muestreo (ejemplo 100Hz)
    uint8_t smplrt_div = 0x07;  // 100Hz de frecuencia de muestreo
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &smplrt_div, 1, false);
}


void mpu6050_read_raw() {   // Función para leer los datos crudos
   
    uint8_t buffer[6];

    // Acelerómetro: Registros 0x3B (X), 0x3D (Y), 0x3F (Z)
    
    uint8_t accel_reg = 0x3B;
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &accel_reg, 1, false);
    i2c_read_blocking(I2C_DEFAULT, MPU6050_ADDR, buffer, 6, false);
    
    // Convertir los valores de 2 bytes a enteros de 16 bits para el acelerómetro
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i*2] << 8) | buffer[(i*2) + 1];
    }

    // Giroscopio: Registros 0x43 (X), 0x45 (Y), 0x47 (Z)
    uint8_t gyro_reg = 0x43;
    i2c_write_blocking(I2C_DEFAULT, MPU6050_ADDR, &gyro_reg, 1, false);
    i2c_read_blocking(I2C_DEFAULT, MPU6050_ADDR, buffer, 6, false);

    // Convertir los valores de 2 bytes a enteros de 16 bits para el giroscopio
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i*2] << 8) | buffer[(i*2) + 1];
    }


}

int main() {

    stdio_init_all();
    printf("Hola, MPU6050! Leyendo datos en raw desde el registro...\n");

    i2c_init(I2C_DEFAULT, 100 * 1000);  // Iniciar I2C con una velocidad de 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    mpu6050_init();  // Inicializar el sensor


    while (1) {

        mpu6050_read_raw();  // Leer los datos crudos

        // Imprimir los valores convertidos (de 16 bits)
        printf("Acelerómetro: ax=%d, ay=%d, az=%d \n", accel[0], accel[1], accel[2]);
        printf("Giroscopio: Gx=%d, Gy=%d, Gz=%d \n", gyro[0], gyro[1], gyro[2]);

        sleep_ms(1000);  // Esperar 1 segundo antes de leer los datos nuevamente
    }
}
*/




// Configuracion del bluetooh funcionando

/*      
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

#define UART_ID uart0         // UART0 para comunicación serial
#define BAUD_RATE 9600        // Tasa de baudios estándar para el HC-05

// Asignación de pines UART según el pinout
#define TX_PIN 17  // GPIO 17 (Pin 22) de la Raspberry Pi Pico (TX)
#define RX_PIN 16  // GPIO 16 (Pin 21) de la Raspberry Pi Pico (RX)

int main() {
    stdio_init_all();  // Inicializa la comunicación serial para la consola
    uart_init(UART_ID, BAUD_RATE);  // Inicializa UART0 a 9600 baudios
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);  // Configura GPIO 17 como TX de UART0
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);  // Configura GPIO 16 como RX de UART0

    char received_char;

    while (1) {
        // Verificar si hay datos disponibles en UART
        if (uart_is_readable(UART_ID)) {
            // Leer el carácter recibido
            received_char = uart_getc(UART_ID);

            // Imprimir el carácter recibido
            printf("Comando recibido: %c\n", received_char);  // Imprimir en la consola

            // Limpiar el buffer UART (leer cualquier dato residual en el buffer)
            while (uart_is_readable(UART_ID)) {
                uart_getc(UART_ID);  // Leer y descartar cualquier dato adicional
            }
        }
    }

    return 0;
}

*/
