#include <firmware/rtos.hpp>

#include <firmware/device.hpp>

#include <cstdint>
#include <FreeRTOS.h>
#include <gpio.h>
#include <task.h>
#include <stm32f4xx_hal.h>

TaskHandle_t blink1_task_handle, blink2_task_handle;
StaticTask_t blink1_task_tcb, blink2_task_tcb;
uint32_t blink1_task_stack[64], blink2_task_stack[64];

void blink1_task(void* params)
{
  while (1) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    vTaskDelay(500);
  }
}

void blink2_task(void* params)
{
  while (1) {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    vTaskDelay(550);
  }
}

extern "C" void rtos_main()
{
  HAL_Delay(500);
  lg::Device::get().initializeDrivers();

  blink1_task_handle = xTaskCreateStatic(
    blink1_task /* Task function */,
    "BLINK1" /* Task name */,
    64 /* Stack size */,
    NULL /* parameters */,
    1 /* Prority */,
    blink1_task_stack /* Task stack address */,
    &blink1_task_tcb /* Task control block */
  );

  blink2_task_handle = xTaskCreateStatic(
    blink2_task /* Task function */,
    "BLINK2" /* Task name */,
    64 /* Stack size */,
    NULL /* parameters */,
    1 /* Prority */,
    blink2_task_stack /* Task stack address */,
    &blink2_task_tcb /* Task control block */
  );

  vTaskStartScheduler();
}
