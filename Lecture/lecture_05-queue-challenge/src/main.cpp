#include <Arduino.h>
#include "Log.h"

#define DELAY_MIN 50

// instantiate the logger
Log<HardwareSerial> * serial_logger = NULL;
using LogLevel = Log<HardwareSerial>::eLogLevel;

// Use only one core for demo
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

/**
 * Command parser
 * 
 * @brief search input string for command and extract parameters
 * 
 * @param input_string the input string which may contain a command
 * 
 * @return char array containing the value parsed from the input string 
*/
bool pszCommandParser(char * input_string, const int input_string_length, const char * command, const int command_length, char * value)
{
	bool b_ret = false;
	if (input_string)
	{
		serial_logger->log("Command parser called on input: \"", LogLevel::kInfo);
		serial_logger->log(input_string, LogLevel::kInfo);
		serial_logger->log("\"\n",LogLevel::kInfo);

		// incrementor
		int i = 0;
		int len = strlen(input_string);

		// forward search for a white space charater (\s or \t)
		for (i=0; i<len && i<input_string_length; i++)
		{
			if (input_string[i] == ' ')
				break;
		}
		i++;

		// check to see if there is a match
		if (strcmp(command, input_string) == 0)
		{
			strcpy(value, input_string + i);
			b_ret = true;
			serial_logger->log("Command parser found value: \"", LogLevel::kInfo);
			serial_logger->log(value, LogLevel::kInfo);
			serial_logger->log("\"\n",LogLevel::kInfo);
		}
	}
	return b_ret;
}

// global integer to store delay value
int g_delay = DELAY_MIN;
int led_pin = GPIO_NUM_0;

// queues
QueueHandle_t queue_1; int queue_1_length = 10;
QueueHandle_t queue_2; int queue_2_length = 10;

/**
 * Task 1
 * 
 * @brief handles serial input
 *	- prints any new messages to the console
 *	- reads from serial input
 *	- echo input back to serial terminal
 *	- if delay command recieved update the delay
*/
void vTask1(void * parameters)
{
	// char array buffer for taking input
	char buffer[64] = {0x00};
	char delay_buffer[64] = {0x00};
	int delay = DELAY_MIN;

	// loop continuously
	while (1)
	{
		// wait for serial input
		while (Serial.available() == 0){}

		// parse serial input - look for delay command
		Serial.readBytesUntil('\n', buffer, 64);

		// otherwise echo serial input back to console
		if (pszCommandParser(buffer, 64, "delay", 6, delay_buffer))
		{
			// try to convert value to int
			delay = atoi(delay_buffer);
			delay = delay < DELAY_MIN ? DELAY_MIN : delay;
			Serial.print("After conversion delay is: ");
			Serial.println(delay);
			xQueueSend(queue_1, &delay, 0);
		}
		else
		{
			// echo input back to console
			Serial.println(buffer);
		}

		// wait for a bit so you dont block the other task
		vTaskDelay(250 * portTICK_PERIOD_MS);
	}
}

/**
 * Task 2
 * 
 * @brief waits for data from task1 and updates the delay 
*/
void vTask2(void * parameters)
{
	char buffer[32];
	int delay;
	while (1)
	{
		if (xQueueReceive(queue_1, &delay, 0) == pdTRUE)
		{
			// update the blink rate
			g_delay = delay < DELAY_MIN ? DELAY_MIN : delay;

			// log the blink rate to the console
			serial_logger->log("Updating blink delay to: ", LogLevel::kTrace);
			itoa(delay, buffer, 32);
			serial_logger->logln(buffer, LogLevel::kTrace);
		}

		// wait for a bit before checking again
		vTaskDelay(250 * portTICK_PERIOD_MS);
	}
}

void setup() {
	// configure the serial port
	Serial.begin(115200);

	// wait for a second for the serial port
	vTaskDelay(1000 * portTICK_PERIOD_MS);
	Serial.println("\n---FreeRTOS Queue Challenge---");

	// configure the pin for output
	pinMode(led_pin, OUTPUT);

	// initialize the serial logger
	serial_logger = new Log<HardwareSerial>(&Serial);
	serial_logger->setLogLevel(LogLevel::kAll);
	serial_logger->log("Logger Initialized\n", LogLevel::kInfo);

	// Initialize the queue
	queue_1 = xQueueCreate(queue_1_length, sizeof(int));
	queue_2 = xQueueCreate(queue_2_length, sizeof(int));

	// initialize the task
	xTaskCreate(
		vTask1,
		"Task 1",
		1024,
		NULL,
		1,
		NULL
	);

	xTaskCreate(
		vTask2,
		"Task 2",
		1024,
		NULL,
		1,
		NULL
	);
}

void loop() {
	// blink the led
	digitalWrite(led_pin, HIGH);
	vTaskDelay(g_delay * portTICK_PERIOD_MS);
	digitalWrite(led_pin, LOW);
	vTaskDelay(g_delay * portTICK_PERIOD_MS);
}
