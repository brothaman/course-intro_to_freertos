#include <Arduino.h>

// Use only one core for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
static const uint16_t timer_divider = 8;
static const uint64_t timer_max_count = 1000000;
static const TickType_t task_delay = 2000 / portTICK_PERIOD_MS;

// Pins 
// static const int led_pin = LED_BUILTIN;
static const int adc_pin = A0;

// Globals
static hw_timer_t *timer = NULL;
// static volatile int isr_counter;
// static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t val;
static SemaphoreHandle_t bin_sem = NULL;

// --- Configure interrupt service routine
void IRAM_ATTR onTimer() {
  BaseType_t task_woken = pdFALSE;

  //perform action (read from ADC)
  val = analogRead(adc_pin);

  // Give semaphore to tell task that new falue is ready
  xSemaphoreGiveFromISR(bin_sem, &task_woken);

  // Exit from ISR
  if (task_woken)
    portYIELD_FROM_ISR();
}

// ---------- TASKS -----------
// Wait for semaphore and print out ADC value when recieved
void printValues(void * parameters) {
  // Loop forever and wait for the semaphore to become available
  while (1)
  {
    xSemaphoreTake(bin_sem, portMAX_DELAY);
    Serial.println(val);
  }
}
void setup() {
  // put your setup code here, to run once:
  // Configure LED Pin
  pinMode(adc_pin, INPUT);

  // Start the task
  Serial.begin(115200);

  // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("--- FreeRTOS ISR Critical Section Demo ---");

  // create the binary semaphore
  bin_sem = xSemaphoreCreateBinary();

  // Force reboot if we cant create the semaphore
  if (bin_sem == NULL)
  {
    Serial.println("Could not create semaphore");
    ESP.restart();
  }

  // Start task to print out results
  xTaskCreatePinnedToCore(printValues,
                          "Print Values",
                          1024,
                          NULL,
                          2,
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
  // do nothing
}