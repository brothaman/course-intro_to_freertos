#include <Arduino.h>

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
#define DEBUG 1

// GLOBAL VARIABLES
float average;
volatile int adc_pin = A0;
uint16_t adc_value;
static const uint16_t timer_divider = 80;
static const uint64_t timer_max_count = 1000000;
static hw_timer_t *timer = NULL; 
static TimerHandle_t adc_update_timer = NULL;
static QueueHandle_t adc_update_queue1 = NULL;
static QueueHandle_t adc_update_queue2 = NULL;
static QueueHandle_t avg_update_queue = NULL;
static const uint8_t queue_len = 10;

// global variable to store the average
// create a semaphore with a maximum value of 10
// create a buffer to store 10 values from task A (reading the ADC)
// create a buffer to store 10 values from task B (calculating the average)
// create an array of function pointers that the terminal has available (avg)

// functions
void commandAvg();
float getAverage();
void setAverage(float);
uint8_t interpretInput(char *, int);
void sampleADCCallback(TimerHandle_t xTimer);
bool parseTerminalInput(String);

// // callback
// void sampleADCCallback(TimerHandle_t xTimer)
// {
//   if (DEBUG)
//     Serial.println("Getting ADC value");

//   // read the ADC value and update the array
//   adc_value = analogRead(adc_pin);

//   // store the value in the queue
//   if (xQueueSend(adc_update_queue1, (void*)&adc_value, 10) != pdTRUE && DEBUG)
//   {
//     // once queue is full call task A to calculate the average
//     Serial.println("DEBUG WARN: Queue full");
//   }
// }

// test
volatile int isrbufpos = 0;
int taskbufpos = 0;
volatile uint16_t buf[10];
static SemaphoreHandle_t sem[10] = {NULL}; // semaphore with 10 spots
// ISR --- Configure interrupt service routine
void IRAM_ATTR onTimer() {
  //
  BaseType_t xTaskWoken = pdFALSE;

  //perform action (read from ADC)
  // uint16_t val = analogRead(adc_pin);

  // // store the value in the queue
  // if (xQueueSendFromISR(adc_update_queue1, (void*)&val, &xTaskWoken) != pdTRUE && DEBUG)
  // {
  //   // once queue is full call task A to calculate the average
  //   Serial.println("DEBUG WARN: Queue 1 full");
  // }
  buf[isrbufpos] = analogRead(adc_pin);
  xSemaphoreGiveFromISR(sem[isrbufpos], &xTaskWoken);
  isrbufpos++;
  if (isrbufpos == 10)
    isrbufpos = 0;
  

  // try to add to a buffer
  // Exit from ISR
  if (xTaskWoken)
    portYIELD_FROM_ISR();
}

// task A
void taskCalculateAverage(void * parameters)
{     
  // create a variable to stroe the adc value
  float avg_adc_value;
  uint16_t adc_value;
  int i = 0;

  // print to the terminal that the task to calculate the average is running
  if (DEBUG)
    Serial.println("Task 'Calculate Average' is running");

  while (i < 10)
  {
    // get the data stored in the queue
    if (xQueueReceive(adc_update_queue2, (void*)&adc_value, 0) == pdTRUE)
    {
      avg_adc_value+=(float)adc_value;
      i++;
    }
  }
  // take semaphore
  average = avg_adc_value / 10;
  // give semaphore
  // exit task
}

// task B - serial terminal
void taskSerialTerminal(void * parameters)
{
  // print to terminal that the serial terminal is running
  Serial.println("Task 'Serial Terminal' is running");

  // run continuously
  while (1)
  {
    // wait for input
    if (Serial.available() > 0)
    {
      // add the adc value to the array
      String strBuf = Serial.readString();

      // pass the string into the parser
      if (!strBuf.isEmpty() && !parseTerminalInput(strBuf))
      {
        Serial.println(strBuf);
      }
    }
  }
}

// TASk C queue shuffler
void taskQueueHandler(void * parameters)
{
  // variable to store the value being shuffled
  uint16_t uiVal;
  BaseType_t handshake = pdFALSE;

  // Print to terminal that task handler is running
  Serial.println("Task 'Queue Handler' is running");
  // Loop continuously
  while (1)
  {
    // take from queue 1 without blocking
    Serial.print("Queue has:  ");
    UBaseType_t uxMsgsInQ = uxQueueMessagesWaiting(adc_update_queue1);
    Serial.print(uxMsgsInQ);
    Serial.println("   messages in queue.");
    if (!handshake && xSemaphoreTake(sem[taskbufpos], portMAX_DELAY))
    {
      // copy the value from the buffer
      uiVal = buf[taskbufpos];
      handshake = !handshake;
    }
    if (handshake && xQueueSend(adc_update_queue2, (void*)&uiVal, 0))
    {
      handshake = !handshake;

      // move the buffer position forward
      taskbufpos++;
    }
    // if (xQueueReceive(adc_update_queue1, (void *) &uiVal, 0) == pdTRUE)
    // {
    //   if (xQueueSend(adc_update_queue2, (void*)&uiVal, 10) != pdTRUE)
    //   {
    //     Serial.println("Queue 2 Full");
    //   }
    //   else
    //   {
    //     i++;
    //   }
    // }
    if (taskbufpos == 10)
    {
      Serial.println("Starting up task to calculate average");
      // start Task A to calculate the average
      xTaskCreatePinnedToCore(taskCalculateAverage,
                              "Calculate Average",
                              1024,
                              NULL,
                              2,
                              NULL,
                              app_cpu
      );
      taskbufpos = 0;
    }
  }
}

// function to parse the terminal input
bool parseTerminalInput(String strBuf)
{
  // return variable initialized as false
  bool bRet =  false;

  // convert the string to upper case (!!!modifies string in place!!!) 
  strBuf.toUpperCase();
  strBuf.trim();

  // check if the string is a command
  if (strBuf.compareTo("AVG") == 0)
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

 // create the ADC queue
  adc_update_queue1 = xQueueCreate(queue_len, sizeof(uint16_t));
  adc_update_queue2 = xQueueCreate(queue_len, sizeof(uint16_t));

  // create the queue to hold the average values
  avg_update_queue = xQueueCreate(queue_len, sizeof(float));

  // create the necessary semaphores
  for (SemaphoreHandle_t s : sem)
  {
    s = xSemaphoreCreateBinary();
  }

  // // Start task to handle the serial terminal
  // if (DEBUG)
  // {
  //   Serial.print("Pinning Serial Terminal task to core: ");
  //   Serial.println(app_cpu);
  // }
  // xTaskCreatePinnedToCore(taskSerialTerminal,
  //                         "Serial Terminal",
  //                         1024,
  //                         NULL,
  //                         2,
  //                         NULL,
  //                         app_cpu
  // );

  // Start task to handle the double buffer
  if (DEBUG)
  {
    Serial.print("Pinning Queue Handler task to core: ");
    Serial.println(app_cpu);
  }
  xTaskCreatePinnedToCore(taskQueueHandler,
                          "Queue handler",
                          1024,
                          NULL,
                          2,
                          NULL,
                          app_cpu
  );

  // create and start timer (Num, divider, count up)
  if (DEBUG)
    Serial.println("Initializing timer 0");
  timer = timerBegin(0, timer_divider, true);

  // Provide  ISR to timer (timer, function, edge)
  if (DEBUG)
    Serial.println("Attaching ISR to timer");
  timerAttachInterrupt(timer, &onTimer, true);

  // At what count shoud ISR trigger (timer, count, autoreload)
  if (DEBUG)
    Serial.println("Configure timer to trigger 10 times per second 10Hz");
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
  return average;
}