#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"

#include "lcd.h"
#include "helper.h"

// Configuraciones
#define PWM_TAKE_PIN 15
#define PWM_GEN_PIN 16

#define PERIODO_MEDICION_MS    1000
#define I2C         i2c0

#define SDA_GPIO    8
#define SCL_GPIO    9


#define LCD_ADDR        0x27

// Definición de colas
QueueHandle_t cola_flancos;
QueueHandle_t cola_frecuencia;

// ISR: Detecta flanco ascendente y manda un 1 a la cola
void gpio_callback(uint gpio, uint32_t events) {
    
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;   // toma control para que ninguna tarea de mayor prioridad se ejecute, extremadamente necesario para mantener predecible el flujo
        uint8_t flanco = 1;   // se crea variable flanco y se le coloca un 1

    if (gpio == PWM_TAKE_PIN && (events & GPIO_IRQ_EDGE_RISE)) {   // ingresa al IF si el puerto es el correcto y si es por un flanco ascendente
        

        xQueueSendFromISR(cola_flancos, &flanco, &xHigherPriorityTaskWoken);   // se carga el valor 1 en la cola_flancos desde el ISR y esta API pone la variable xHigherPriorityTaskWoken en True para que la tarea que est bloqueada esperando el dato de la cola y permita que tome el control 
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);   // es necesario para que salga del ISR automaticamente

    }
}

// Tarea para contar ls flancos
void tarea_frecuencimetro(void *pvParameters){
    uint32_t contador = 0;
    uint8_t dummy;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {

        // Contar flancos durante PERIODO_MEDICION_MS
        while (xTaskGetTickCount() < xLastWakeTime + pdMS_TO_TICKS(PERIODO_MEDICION_MS)) {

            if (xQueueReceive(cola_flancos, &dummy, pdMS_TO_TICKS(10)) == pdPASS) {

                contador++;

            }
        }

        // Enviar resultado a cola de impresión
        xQueueSend(cola_frecuencia, &contador, portMAX_DELAY);

        contador = 0;
        xLastWakeTime += pdMS_TO_TICKS(PERIODO_MEDICION_MS);
    }
}

// Tarea para mostrar n el LCD
void tarea_lcd(void *pvParameters){
    uint32_t frecuencia;
    char buffer1[16];

    while (1){

        if (xQueueReceive(cola_frecuencia, &frecuencia, portMAX_DELAY) == pdPASS){
            printf("Frecuencia medida: %lu Hz\n", frecuencia);                  //Muestro el valor en consola
            
            
            // Muestreo en LCD 

            lcd_set_cursor(0, 0);
            lcd_string("Frecuencia");

            lcd_set_cursor(1, 0);
            lcd_string("Medida:");

            lcd_set_cursor(1, 8);
            sprintf(buffer1, "%lu Hz", frecuencia);
            lcd_string(buffer1);

        }
    }
}


// Función principal
int main() {

    stdio_init_all();


    gpio_init(PWM_TAKE_PIN);          
    gpio_set_dir(PWM_TAKE_PIN, GPIO_IN); 
    gpio_pull_down(PWM_TAKE_PIN);               // por defecto toma valores 0 si no hay señal

    pwm_user_init(PWM_GEN_PIN, 7500);   // inicio PWM 

    i2c_init(I2C, 100*1000);                          // Inicializo los puertos del I2C con un clock de 100 KHz

    
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);
    
    // Inicializo LCD
    lcd_init(I2C, LCD_ADDR);

    // Crear colas
    cola_flancos = xQueueCreate(1024, sizeof(uint8_t));    // 1024 flancos posibles por segundo
    cola_frecuencia = xQueueCreate(4, sizeof(uint32_t));   // Buffer pequeño, 1 dato por segundo

    // Registrar interrupción
    gpio_set_irq_enabled_with_callback(PWM_TAKE_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);   // interfaz de entrada: PWM_TAKE_PIN, event_mask: (GPIO_IRQ_EDGE_RISE: IRQ cuando la señal transiciona desde 0 a 1), enabled: (true: habilitado), callback: (gpio_callback: funcion que se llama cuando se detecta el evento)

    xTaskCreate(tarea_frecuencimetro, "Contador_Flancos", 1024, NULL, 2, NULL);
    xTaskCreate(tarea_lcd, "Impresion_LCD", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}


