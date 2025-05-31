#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t q_ADC;

// ISR ADC
void adc_ISR(void)
{
    static BaseType_t xHPTW = pdFALSE;
    static uint16_t adc_value;

    // Deshabilito interrupcion y conversion
    adc_irq_set_enabled(false);
    adc_run(false);

    // Leo valor del FIFO
    adc_value = adc_fifo_get();
    // Limpio el FIFO
    adc_fifo_drain();
    // Envio desde la ISR
    xQueueSendFromISR(q_ADC, &adc_value, &xHPTW);
}

static void task_ADC(void *pvParam)
{
    uint16_t result;

    while(1){


        adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);

        adc_irq_set_enabled(true);                                      // Habilito la irq y conversion
        adc_run(true);
        // Delay de 1 seg
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

static void task_Print(void *pvParam)
{
    uint16_t ADC_RAW;
    const int ADC_RESOLUTION = 4095;
    const float ADC_VREF = 3.3f;
    const float ADC_FACTOR = ADC_VREF / ADC_RESOLUTION;
    float ADC_VOLTAGE;
    float TEMPERATURA;

    while(1){

        xQueueReceive(q_ADC, &ADC_RAW, portMAX_DELAY);
        
        ADC_VOLTAGE = ADC_RAW * ADC_FACTOR;
        TEMPERATURA = 27 - (ADC_VOLTAGE - 0.706f) / 0.001721f;
        printf("Temperatura = %.2f C\n", TEMPERATURA);

        }
}


int main()
{
    stdio_init_all();                                       //Inicializacion
    adc_init();
    adc_set_temp_sensor_enabled(true);


    adc_fifo_setup(true, false, 1, false, false);           // Seteo el FIFO del ADC (size 1)

    adc_irq_set_enabled(true);                              // Habilito irq del adc
    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_ISR);       // Asocio handler de IRQ del FIFO a la funcion ISR (asociada al core en ejecucion)
    irq_set_enabled(ADC_IRQ_FIFO, true);                    // Habilito IRQ del FIFO (en el core en ejecucion)


//    adc_run(true);                                          // ADC en modo free running

    
    q_ADC = xQueueCreate(1, sizeof(uint16_t));              // Creo la cola para la temperatura

    xTaskCreate(task_ADC, "ADC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_Print, "Print", 2*configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true);
}
