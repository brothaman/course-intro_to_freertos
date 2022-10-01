#include <Arduino.h>

// Use only core 1 for demo
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Globals
static TimerHandle_t one_shot_timer = NULL;
static TimerHandle_t auto_reload_timer = NULL;
// Callback function
void myTimerCallback(TimerHandle_t xTimer) {
  // Print message if timer 0 expired
  if((uint32_t) pvTimerGetTimerID(xTimer) == 0)
    Serial.println("One-shot timer expired");
  
  // Print message if timer 0 expired
  if((uint32_t) pvTimerGetTimerID(xTimer) == 1)
    Serial.println("Auto-reload timer expired");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Wait a moment to start
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println("\n--- FreeRTOS Timer Demo ---");

  // Create a one-shot timer
  one_shot_timer = xTimerCreate(
    "One-shot timer",             // Name of timer
    2000 / portTICK_PERIOD_MS,    // Period of timer (in ticks)
    pdFALSE,                      // Auto-reload
    (void *)0,                    // Timer ID
    myTimerCallback               // Callback function attached to timer
  );

  // Create a one-shot timer
  auto_reload_timer = xTimerCreate(
    "One-shot timer",             // Name of timer
    1000 / portTICK_PERIOD_MS,    // Period of timer (in ticks)
    pdTRUE,                       // Auto-reload
    (void *)1,                    // Timer ID
    myTimerCallback               // Callback function attached to timer
  );

  // Check to make sure the timers were created
  if (one_shot_timer == NULL || auto_reload_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    // Wait and then print out a message that we're starting the timer
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers... ");

    // Start timer (max block time if command queue is full)
    xTimerStart(one_shot_timer, portMAX_DELAY);
    xTimerStart(auto_reload_timer, portMAX_DELAY);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}