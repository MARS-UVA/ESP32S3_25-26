#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"

#define ESP_INR_FLAG_DEFAULT 0
#define LED_PIN  36
#define PUSH_BUTTON_PIN  18

TaskHandle_t ISR = NULL;

void IRAM_ATTR button_isr_handler(void *arg){
  xTaskResumeFromISR(ISR);
}

void interrupt_task(void *arg){
  bool led_status = false;
  while(1){
    vTaskSuspend(NULL);
    led_status = !led_status;
    gpio_set_level(LED_PIN, led_status);
    printf("Button pressed!\n");
  }
}

void app_main(void)
{
  esp_rom_gpio_pad_select_gpio(PUSH_BUTTON_PIN);
  esp_rom_gpio_pad_select_gpio(LED_PIN);

  gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_direction(LED_PIN ,GPIO_MODE_OUTPUT);

  gpio_set_intr_type(PUSH_BUTTON_PIN, GPIO_INTR_POSEDGE);

  gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);

  gpio_isr_handler_add(PUSH_BUTTON_PIN, button_isr_handler, NULL);

  xTaskCreate(interrupt_task, "interrupt_task", 4096, NULL, 10, &ISR);
}