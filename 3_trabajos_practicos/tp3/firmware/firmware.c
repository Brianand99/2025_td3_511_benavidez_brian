#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lcd.h"
#include "helper.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments


#define PWM_GEN_PIN 16
#define PWM_TAKE_PIN 15

#define I2C_SDA 8
#define I2C_SCL 9

SemaphoreHandle_t r_counting;



//---------- Tareas ----------- //

void task_semaforo(void *pvparam)
{

    while(1)
    {
        
        if(gpio_get(PWM_TAKE_PIN))
        {
            xSemaphoreGive(r_counting);
            while(gpio_get(PWM_TAKE_PIN));
        }
    }
    
}

void task_frecuency(void *pvparam)
{
    while(1)
    {

    uint32_t count = uxSemaphoreGetCount(r_counting);               // Creacion de variable y obtencion del valor
    xQueueReset(r_counting);                                        // Reseteo de la cuenta del semaforo 
    printf("La frecuencia es %d \n",count*10);                    // Imprimo por consola el valor de la frecuencia obtenida 

    vTaskDelay(pdMS_TO_TICKS(100));                                 // Bloque la funcion para que tenga mayor prioridad la otra
    
    }

}

int main()
{
    stdio_init_all();
    
    // Inicializacion de puerto para lectura 
    gpio_init(PWM_TAKE_PIN);
    gpio_set_dir(PWM_TAKE_PIN,false);
    
    // Generacion de pwm mediante software 
    pwm_user_init(PWM_GEN_PIN, 7654);

    // Inicializacion de I2C
    i2c_init(i2c_default, 100*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    // Creacion del semaforo 

    r_counting=xSemaphoreCreateCounting(10000,0);

    xTaskCreate(task_semaforo, "Semforo",128*3,NULL,1,NULL);
    xTaskCreate(task_frecuency, "Frecuencia",128*3,NULL,2,NULL);

    vTaskStartScheduler();


    while (true) {

    }
}
