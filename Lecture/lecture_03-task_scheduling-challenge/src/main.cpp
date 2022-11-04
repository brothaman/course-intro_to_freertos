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

// task handles
static TaskHandle_t t_serial_terminal = NULL;
static TaskHandle_t t_blink_led = NULL;

// led pins to be used
static const int r_led_pin = GPIO_NUM_0;
static const int g_led_pin = GPIO_NUM_2;
static const int b_led_pin = GPIO_NUM_4;

// a delay value for blinkrgb
static int g_delay;

// a blink led function to use
void blinkRGB(int delay = 500)
{
	if (delay > 1000)
		delay = 1000;
	else if (delay < 50)
		delay = 50;
	digitalWrite(r_led_pin, HIGH);
	vTaskDelay(delay / portTICK_PERIOD_MS);
	digitalWrite(r_led_pin, LOW);
	digitalWrite(g_led_pin, HIGH);
	vTaskDelay(delay / portTICK_PERIOD_MS);
	digitalWrite(g_led_pin, LOW);
	digitalWrite(b_led_pin, HIGH);
	vTaskDelay(delay / portTICK_PERIOD_MS);
	digitalWrite(b_led_pin, LOW);
}


// task definition
// task 1 print to serial terminal with lower priority
void taskSerialMonitor(void * parameters)
{
  // string to store the input
  char input_buffer[32] = {'\0'};
  int inc = 0;
  // String output_buffer;
  int delay = 0; 
  bool b_updated;

  // print string to terminal
  while(1)
  {
    while (Serial.available() > 0 && inc<32)
    {
      input_buffer[inc] = Serial.read();
      inc++;
    }
    if (inc>0)
    {
      input_buffer[inc] = '\0';
      delay = atoi(input_buffer);
      g_delay = delay;
      // output_buffer = String("Updated delay to: "); + delay + String("ms");
      Serial.print("Updated delay to: ");
      Serial.print(delay);
      Serial.println("ms");
      b_updated = false;
    }
    inc = 0;

  }
}

// task 2 - blink led
void taskBlinkLED(void * parameters)
{
  g_delay = 500;
  // blink the led
  while (1)
  {
    blinkRGB(g_delay);
  }
}

void setup() 
{
  // configure pins for output
  pinMode(r_led_pin, OUTPUT);
  pinMode(g_led_pin, OUTPUT);
  pinMode(b_led_pin, OUTPUT);

  // configure the serial port for 300 baud so we can see whats going on
  Serial.begin(115200);

  // initialize the serial port
  serial_logger = new Log<HardwareSerial>(&Serial);

  // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("\n---FreeRTOS Task Scheduling Demo---\n");

  // print self priority
  Serial.print("Setup and loop task running on core '");
  Serial.print(xPortGetCoreID());
  Serial.print("' with priority: ");
  Serial.println(uxTaskPriorityGet(NULL));

  // Task to run forever
  xTaskCreatePinnedToCore(
    taskBlinkLED,
    "Task 1",
    1024,
    NULL,
    1,
    &t_blink_led,
    app_cpu
  );

  // Task to run forever
  xTaskCreatePinnedToCore(
    taskSerialMonitor,
    "Task: Serial Monitor",
    1024,
    NULL,
    1,
    &t_serial_terminal,
    app_cpu
  );
  vTaskDelete(NULL);
}

void loop() {
}