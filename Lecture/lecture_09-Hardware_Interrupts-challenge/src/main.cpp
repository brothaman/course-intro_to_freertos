#include <Arduino.h>
#include "Log.h"

// instantiate the logger
Log<HardwareSerial> * serial_logger = NULL;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

/**
 * Hardware interrupts challenge
 * 
 * DESCRIPTION:
 * create a hardware timer that samples the ADC at 10Hz
 * then store the values in a buffer 
 * once 10 sampes are collected
 * the ISR should run
 * use circular buffer
 * the ISR should wake up task A
 * task A will:
 * - compute the average of the 10 values
 * - then and store it in a global variable
 * - assume this value cannot be read from or written to in a single cycle
 * task B will:
 * - echo back any character it sees from the serial terminal
 * - if it recieves the command "avg" it should print the average
 */

// Macros 
#define BAUDRATE 115200

// GLOBAL VARIABLES
static float g_average;
static SemaphoreHandle_t mutex = NULL;
volatile int adc_pin = A0;
volatile uint16_t adc_value;

// variables for timer
static const uint16_t timer_divider = 80;
static const uint64_t timer_max_count = 1000000;
static hw_timer_t *timer = NULL; 
static TimerHandle_t adc_update_timer = NULL;

// variables for circular buffer
static QueueHandle_t adc_update_queue1 = NULL;
static QueueHandle_t adc_update_queue2 = NULL;
static QueueHandle_t avg_update_queue = NULL;
static const uint8_t queue_len = 10;

// global variable to store the average
// create a buffer to store 10 values from task A (reading the ADC)
// create a buffer to store 10 values from task B (calculating the average)
// create an array of function pointers that the terminal has available (avg)

// functions
void commandAvg();
float getAverage();
void setAverage(float);
uint8_t interpretInput(char *, int);
void sampleADCCallback(TimerHandle_t xTimer);
bool parseTerminalInput(const char *);

// ISR --- Configure interrupt service routine
void IRAM_ATTR onTimer() 
{
  BaseType_t xTaskWoken = pdFALSE;

  //perform action (read from ADC)
  uint16_t val = analogRead(adc_pin);

  // store the value in the queue
  xQueueSendFromISR(adc_update_queue1, (void*)&val, &xTaskWoken);

  // Exit from ISR
  if (xTaskWoken)
    portYIELD_FROM_ISR();
}

/**
 * Task A
 * 
 * @brief after 10 samples have been collected wake up and compute the average
 */
void vTaskA_CalculateAvg(void * parameters)
{
  // a buffer to store all data from the the queue
  int adc_values[10] = {0};

  // variable for local average
  float _average;

  // Print to terminal that task handler is running
  serial_logger->logln("Task A 'Calculating Average' is running", LogLevel::kInfo);
  
  // print to the terminal that the task to calculate the average is running
  while (1)
  {
    // check to see if there are 10 items in queue
    if (uxQueueMessagesWaiting(adc_update_queue2) == 10)
    {
      for (int i = 0; i<10; i++)
        xQueueReceive(adc_update_queue2, (void *)(adc_values + i), portMAX_DELAY);
      
      // calculate the average
      _average = 0;
      for (int i = 0; i < 10; i++)
        _average += adc_values[i];
      _average = _average / 10;

      // gain protected access to the global average variable
      xSemaphoreTake(mutex, 0);
      g_average = _average;
      xSemaphoreGive(mutex);
      serial_logger->logln("Finished calculating the average", LogLevel::kInfo);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// task B - serial terminal
void vTaskB_SerialTerminal(void * parameters)
{
  // character buffer for serial input
  char buf[64] = {0};
  int length;

  // Print to terminal that task handler is running
  serial_logger->logln("Task B 'Serial Terminal' is running", LogLevel::kInfo);
  
  // run continuously
  while (1)
  {
    // wait for input
    if (Serial.available() > 0)
    {
      // add the adc value to the array
      length = Serial.readBytesUntil('\n', buf, 64);
      buf[length] = '\0';

      // pass the string into the parser
      if (!parseTerminalInput(buf))
      {
        Serial.println(buf);
      }
    }

    // wait for a bit and allow other task to run
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// TASk C queue shuffler
void vTaskC_CircularBuffer(void * parameters)
{
  // variable to store the value being shuffled
  uint16_t uiVal;
  BaseType_t handshake = pdFALSE;

  // Print to terminal that task handler is running
  serial_logger->logln("Task 'Queue Handler' is running", LogLevel::kInfo);
  
  // Loop continuously
  while (1)
  {
    if (xQueueReceive(adc_update_queue1, (void *) &uiVal, 0) == pdTRUE)
    {
      xQueueSend(adc_update_queue2, (void*)&uiVal, portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

// function to parse the terminal input
bool parseTerminalInput(const char * string)
{
  // return variable initialized as false
  bool bRet =  false;

  // check if the string is a command
  if (strcmp(string, "AVG") == 0)
  {
    // return true since command was found
    bRet =  pdTRUE;

    // run the command to get the averate
    commandAvg();
  }  
  return bRet;
}

void setup() {
  // Configure the adc pin for input
  pinMode(adc_pin, INPUT);

  // set up the serial port
  Serial.begin(BAUDRATE);

   // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("--- FreeRTOS Hardware Interrupts Challenge ---");

  // initialize the serail logger
  serial_logger = new Log<HardwareSerial>(&Serial);
  serial_logger->setLogLevel(LogLevel::kWarn);

 // create the ADC queue
  adc_update_queue1 = xQueueCreate(queue_len, sizeof(uint16_t));
  adc_update_queue2 = xQueueCreate(queue_len, sizeof(uint16_t));

  // create the queue to hold the average values
  avg_update_queue = xQueueCreate(queue_len, sizeof(float));

  // initialize the semaphore
  mutex = xSemaphoreCreateMutex();

  // create task A
  xTaskCreatePinnedToCore(
    vTaskA_CalculateAvg,
    "Task A: Calculate Average",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // create task B: serial terminal
  xTaskCreatePinnedToCore(
    vTaskB_SerialTerminal,
    "Task B: Serial Terminal",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // create task C: circular buffer
  xTaskCreatePinnedToCore(
    vTaskC_CircularBuffer,
    "Task C: Circular Buffer",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // create and start timer (Num, divider, count up)
  timer = timerBegin(0, timer_divider, true);

  // Provide  ISR to timer (timer, function, edge)
  timerAttachInterrupt(timer, &onTimer, true);

  // At what count shoud ISR trigger (timer, count, autoreload)
  timerAlarmWrite(timer, timer_max_count, true);

  // Allow ISR to trigger
  timerAlarmEnable(timer);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void commandAvg()
{
  float fAvg = getAverage();
  Serial.print("Average: ");
  Serial.println(fAvg);
}

float getAverage()
{
  return g_average;
}