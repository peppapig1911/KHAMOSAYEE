#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

extern "C"
{
    void vApplicationMallocFailedHook(void)
    {
        panic("FreeRTOS malloc failed");
    }

    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
    {
        (void)xTask;
        panic("Stack overflow in task: %s", pcTaskName);
    }
}