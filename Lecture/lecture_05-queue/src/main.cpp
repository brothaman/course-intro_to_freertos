#include <Arduino.h>
#include "Log.h"

// instantiate the logger
Log<HardwareSerial> * serial_logger = NULL;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Configuration settings for queue
static const uint8_t msg_queue_len = 5;

// Globals
static QueueHandle_t msg_queue;

// Task definition

// task: wait for item on queue aand print it 
void printMessages(void * parameters) {
  int item;

  // loop forever
  while (1)
  {
    // see if there's a message in the queue
    if(xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE)
      Serial.println(item);
    
    // wait for a bit to allow the other task to use the cpu
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n--- FreeRTOS Queue Demo ---");

  // initialize and configure the queue
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

  // create the task and attach to core 1
  xTaskCreatePinnedToCore(
    printMessages,
    "Task: Print messages",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

}

void loop() {
  // try to send a value to the queue and print that queue is full if unsuccessful
  static int num = 0;
  if (xQueueSend(msg_queue, (void *)&num, 10) !=pdTRUE)
    Serial.println("Queue full");
  num++;
  vTaskDelay(500 / portTICK_PERIOD_MS);
}