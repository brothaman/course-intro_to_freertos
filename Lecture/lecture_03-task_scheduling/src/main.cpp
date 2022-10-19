#include <Arduino.h>
#include "Log.h"

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// some string to print
const char msg[] = "Hello world i am here but i need a longer sentence to cause the task preempting";

// task handles
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

// task definition
// task 1 print to serial terminal with lower priority
void startTask1(void * parameters)
{
  // count the number of characters in the string
  int msg_len = strlen(msg);

  // print string to terminal
  while(1)
  {
    Serial.println();
    for (int i=0; i<msg_len; i++)
      Serial.print(msg[i]);
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void startTask2(void * parameter)
{
  // print nonsense to the terminal
  while (1)
  {
    Serial.print('*');
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void setup() {
  // configure the serial port for 300 baud so we can see whats going on
  Serial.begin(300);

  // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n---FreeRTOS Task Scheduling Demo---");

  // print self priority
  Serial.print("Setup and loop task running on core");
  Serial.print(xPortGetCoreID());
  Serial.print(" with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  // Task to run forever
  xTaskCreatePinnedToCore(
    startTask1,
    "Task 1",
    1024,
    NULL,
    1,
    &task_1,
    app_cpu
  );

  // Task to run forever
  xTaskCreatePinnedToCore(
    startTask2,
    "Task 2",
    1024,
    NULL,
    2,
    &task_2,
    app_cpu
  );
}

void loop() {
  // Suspend the higher priority task for some intervals
  for (int i=0; i<3; i++)
  {
    vTaskSuspend(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  // Delete the lower priority task
  if (task_1 != NULL)
  {
    vTaskDelete(task_1);
    task_1 = NULL;
  }
}