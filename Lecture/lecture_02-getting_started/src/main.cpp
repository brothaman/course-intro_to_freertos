#include <Arduino.h>
#include "Log.h"

// instantiate the logger
Log<HardwareSerial> * serial_logger;

// alias the logger type
using LogLevel = Log<HardwareSerial>::eLogLevel;
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif


// pins to use
static const int r_led_pin = GPIO_NUM_0;
static const int g_led_pin = GPIO_NUM_2;
static const int b_led_pin = GPIO_NUM_4;

// our task blink an led
void toggleLED(void * parameters)
{
	// echo a debug message to the  console
	if (serial_logger)
		serial_logger->log("Initializing task 'Toggle LED'", LogLevel::kTrace);
	while (1)
	{
		digitalWrite(r_led_pin, HIGH);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		digitalWrite(r_led_pin, LOW);
		digitalWrite(g_led_pin, HIGH);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		digitalWrite(g_led_pin, LOW);
		digitalWrite(b_led_pin, HIGH);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		digitalWrite(b_led_pin, LOW);
	}
}
void setup() {
	// initialize the serial port
	Serial.begin(115200);

	// wait for serial port to get up and running
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
  // initialize the logger
	serial_logger = new Log<HardwareSerial>(&Serial);

	// configure the serial logger to report all
	serial_logger->setLogLevel(LogLevel::kAll);

	// log initialization message to console at trace level
	serial_logger->log("--- FreeRTOS Demo for getting started video ---", LogLevel::kTrace);

	// create the toggleLED task
	xTaskCreatePinnedToCore(
		toggleLED,
		"Task: Toggle LED",
		1024,
		NULL,
		1,
		NULL,
		app_cpu
	);
	delete serial_logger;
	vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}