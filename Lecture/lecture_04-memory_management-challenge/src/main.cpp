#include <Arduino.h>
#include "Log.h"


// instantiate the serial logger object
Log<HardwareSerial> * serial_logger = NULL;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// create handles for the two task
TaskHandle_t serial_monitor_listen = NULL;
TaskHandle_t serial_monitor_reply = NULL;

// variable to notify task 2 that the buffer is ready
bool b_buffer_is_ready = false;
char * buffer;
int i_buffer_len;
/**
 * The goal is to create 2 task
 * 
 * Task 1:
 * - listen to the serial port
 * - when input recieved from serial monitor store input in variable and send to task 2
 * 
 * Task 2:
 * - wait for input from task 2
 * - when input recieved send output to serial monitor
 */

void task1(void * parameters)
{
  char sIntBuffer[32];
  while (1)
  {
    if (Serial.available() > 0)
    {
      // peek for a newline and if found    // Print out the remaining stack memory in words
    serial_logger->log("High water mark (words): ", LogLevel::kTrace);
    serial_logger->logln(itoa(uxTaskGetStackHighWaterMark(NULL), sIntBuffer, 10), LogLevel::kTrace);

    // Print out numbe ro f free heap memory bytes before malloc
    serial_logger->log("Heap before malloc (bytes): ", LogLevel::kTrace);
    serial_logger->logln(itoa(xPortGetFreeHeapSize(), sIntBuffer, 10), LogLevel::kTrace);

      //  find out how many bytes are available
      i_buffer_len = Serial.available() + 1;
      buffer = (char *) pvPortMalloc(sizeof(char) * i_buffer_len);
      if (buffer)
      {
        size_t i = Serial.readBytesUntil('\n', buffer, i_buffer_len);
        buffer[i] = '\0';
        b_buffer_is_ready = true;
      }
      else
      {
        Serial.println("ERROR: Unable to allocate memory for char buffer");
      }
      //  allocate enough storage into the bytes
      //  read the data and store it in the buffer
      //  send the data to task 2
      //  deallocate the storage
      // otherwise continue waiting
    }
    // add a little delay to allow othe rtask to use the cpu
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void task2(void * parameters)
{
  char sIntBuffer[32];
  while (1)
  {
    if (b_buffer_is_ready)
    {
      Serial.println(buffer);
      b_buffer_is_ready = false;

      // deallocate the memory from the buffer
      vPortFree(buffer);
      
      // print out the number of free heap memory bytes after malloc
      serial_logger->log("Heap after malloc (bytes): ", LogLevel::kTrace);
      serial_logger->logln(itoa(xPortGetFreeHeapSize(), sIntBuffer, 10), LogLevel::kTrace);

    }
    // add a little delay to allow other task to use the cpu resources
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // initialize the serial port
  Serial.begin(115200);

  // INITIALIZE the serial logger
  serial_logger = new Log<HardwareSerial>(&Serial);

  // wait a second for the serial port to become ready
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n--- FreeRTOS Challenge: Memory Management ---\n");

  // create the serial monitor task and pin it to one core
  xTaskCreatePinnedToCore(
    task1,
    "Task: Serial Monitor",
    2048,
    NULL,
    1,
    &serial_monitor_listen,
    app_cpu
  );

  // create the serial replay task and pin it to core 1
  xTaskCreatePinnedToCore(
    task2,
    "Task: Serial Output",
    2048,
    NULL,
    1,
    &serial_monitor_reply,
    app_cpu
  );

  // delete the setup and loop task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}