#include <Arduino.h>
#include "Log.h"

// Initialize the logger
Log<HardwareSerial> * serial_logger;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif


// Task: do something
void testTask(void * parameters)
{
  while (1)
  {
    int a = 1;
    int b[100];

    // do something with array so it doesnt disappear
    for (int i=0; i<100; i++)
      b[i] = a + i;

    // print a value to the console
    Serial.println(b[0]);

    // Print out the remaining stack memory in words
    Serial.print("High water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));

    // Print out numbe ro f free heap memory bytes before malloc
    Serial.print("Heap before malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());

    int * ptr = (int*)pvPortMalloc(1024 * sizeof(int));
    if (ptr)
    {
      for (int i=0; i<1024; i++)
        ptr[i] = 3;
    }
    else 
    {
      Serial.println("Not enough heap available");
    }

    // print out the number of free heap memory bytes after malloc
    Serial.print("Heap after malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());


    // free the previously allocated memory
    vPortFree(ptr);

    // slow down this process for observation
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // configure the serial port
  Serial.begin(115200);

  // wait a moment for the port to initialize
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n---FreeRTOS Memory Management Demo---");

  // initialize the task and pin it to the core
  xTaskCreatePinnedToCore(
    testTask,
    "Test Task",
    1500,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // delete the setup and loop task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}