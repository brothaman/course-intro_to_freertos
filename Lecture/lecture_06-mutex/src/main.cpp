#include <Arduino.h>
#include "Log.h"

#define ITER 1

// instantiate the logger
Log<HardwareSerial> * serial_logger = NULL;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

int shared_var = 0;
static SemaphoreHandle_t mutex;

void incTask(void * parameters)
{
#if ITER == 1
  int local_var;

  while (1)
  {

    // take  the mutex
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
      // take a round about way to perform shared_var++ randomly and poorly
      local_var = shared_var;
      local_var++;
      vTaskDelay(random(100, 500) / portTICK_PERIOD_MS);
      shared_var = local_var;

      // print out the shared variable to console
      Serial.println(shared_var);

      // once done with the critical task give back the mutex
      xSemaphoreGive(mutex);
    }

    // wait for just a bit before attempting to take the mutex again
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
#elif ITER == 2
#endif
}

void setup() {
  // initialize the serail port
  Serial.begin(115200);

  // initialize random seed at 1
  randomSeed(1);

  // wait for a bit for the serial port to be set up
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n---FreeRTOS Demo - Mutexes---");

  // initilize and configure the serial logger
  serial_logger = new Log<HardwareSerial>(&Serial);
  serial_logger->setLogLevel(LogLevel::kInfo);

  // create the mutex
  mutex = xSemaphoreCreateMutex();

  // create two identical task to cause a race condition
  xTaskCreatePinnedToCore(
    incTask,
    "Bad incrementor 1",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );
  xTaskCreatePinnedToCore(
    incTask,
    "Bad incrementor 2",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // delete the set up and loop task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}