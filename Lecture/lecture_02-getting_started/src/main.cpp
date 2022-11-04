#include <Arduino.h>
#include "Log.h"

// instantiate the logger
Log<HardwareSerial> *serial_logger;

// alias the logger type
using LogLevel = Log<HardwareSerial>::eLogLevel;
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// delay value
volatile static unsigned long _delay;

// pins to use
static const int r_led_pin = GPIO_NUM_0;
static const int g_led_pin = GPIO_NUM_2;
static const int b_led_pin = GPIO_NUM_4;
static const int pot_pin = A0;

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

// our task blink an led
void toggleLED(void *parameters)
{
	// echo a debug message to the  console
	if (serial_logger)
		serial_logger->log("Initializing task 'Toggle LED'", LogLevel::kTrace);
	while (1)
	{
		blinkRGB(_delay);
	}
}

// task to update delay with potentiometer value
void updateDelay(void *parameters)
{
	String str_output;
	while (1)
	{
		// update the delay value
		_delay = analogRead(pot_pin) / 4;
		str_output = String("ADC Value: ") + _delay;
		serial_logger->log(str_output.c_str(), LogLevel::kTrace);

		// delay for a time to allow the blink task to run
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}

void setup()
{
	// initialize the pins for output
	pinMode(r_led_pin, OUTPUT);
	pinMode(g_led_pin, OUTPUT);
	pinMode(b_led_pin, OUTPUT);
	pinMode(pot_pin, INPUT);

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

	// initialize the delay to 500
	_delay = 500UL;
	serial_logger->log("Initializing delay to 500", LogLevel::kInfo);

	// create the toggleLED task
	xTaskCreatePinnedToCore(
		toggleLED,
		"Task: Toggle LED",
		1024,
		NULL,
		1,
		NULL,
		app_cpu);

	// create the toggleLED task
	xTaskCreatePinnedToCore(
		updateDelay,
		"Task: Update Delay",
		1024,
		NULL,
		1,
		NULL,
		app_cpu);

	vTaskDelete(NULL);
}

void loop()
{
	// put your main code here, to run repeatedly:
}