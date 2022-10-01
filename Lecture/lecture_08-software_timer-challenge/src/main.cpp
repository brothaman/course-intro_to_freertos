#include <Arduino.h>

/*
  Timers in freertos and now for your 
  challenge a common example of timers in
  microcontrollers is the lcd backlight
  auto dim feature let's pretend that the
  led on your esp32 is the backlight for
  an lcd so long as you're using the
  serial terminal and entering characters
  which will pretend is a menu on the lcd
  the led should turn on and stay on
  however after five seconds of inactivity
  the led should turn off your job is to
  echo characters to the serial terminal
  in a new task turn on the led when these
  characters are being entered and then
  use a software timer to turn off the led
  after five seconds from when the last
  character was entered
  here's a hint for you the x timer start
  function will restart a counter if it's
  called before the timer has expired on
  the next episode we'll look at
  integrating hardware interrupts with
  rtos tasks good luck and see you then
*/


/**
 * This program will check to see if any 
 * keys have been typed. each time a key
 * is typed it will reset turn on the led
 * and reset the led timeout timer. if
 * the led timeout timer lapses the led
 * time will turn off.
 */

// Use only core 1 for demo
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Globals
static TimerHandle_t led_refresh_timer = NULL;

enum eTIMER_ID {
  LED_REFRESH_TIMER
};

// Callback function
void refreshTimerCallback(TimerHandle_t xTimer) {
  // Turn LED off
  Serial.println("Turning LED off");
  digitalWrite(LED_BUILTIN, LOW);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Enable the led pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Wait a moment to start
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println("\n--- FreeRTOS Timer Demo ---");


  // Create a one-shot timer
  led_refresh_timer = xTimerCreate(
    "One-shot timer",             // Name of timer
    5000 / portTICK_PERIOD_MS,    // Period of timer (in ticks)
    pdFALSE,                      // Auto-reload
    (void *)0,                    // Timer ID
    refreshTimerCallback          // Callback function attached to timer
  );

  // Check to make sure the timers were created
  if (led_refresh_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    // Wait and then print out a message that we're starting the timer
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers... ");
  }

}

void loop() {
  // check the serial port
  if (Serial.read() > 0)
  {
    // Turn LED on
    Serial.println("Turning LED on");
    digitalWrite(LED_BUILTIN, HIGH);
    xTimerStart(led_refresh_timer, portMAX_DELAY);
  }
}