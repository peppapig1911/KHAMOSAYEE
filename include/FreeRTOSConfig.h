#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCPU_CLOCK_HZ 125000000UL
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 256
#define configTOTAL_HEAP_SIZE (128 * 1024)
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1

#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 1

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0

#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1

#define configQUEUE_REGISTRY_SIZE 8

/* SMP — RP2040 dual-core */
#define configNUMBER_OF_CORES 2
#define configTICK_CORE 0
#define configUSE_CORE_AFFINITY 1
#define configRUN_MULTIPLE_PRIORITIES 1

#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY 2
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 1024

#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_xTimerPendFunctionCall 1
#define INCLUDE_xSemaphoreGetMutexHolder 1
#define INCLUDE_vTaskPrioritySet 1

#define configKERNEL_INTERRUPT_PRIORITY 255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 16
#define configUSE_PASSIVE_IDLE_HOOK 0

#define configUSE_TRACE_FACILITY 1
#define configUSE_EVENT_GROUPS 1
#define INCLUDE_xEventGroupSetBitsFromISR 1
#define configSUPPORT_STATIC_ALLOCATION 0

#define configUSE_TIMERS 1
#endif