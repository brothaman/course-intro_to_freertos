#include <Arduino.h>


// which iteration of the demo is this
#define ITER 3

// Use only one core for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
#if ITER == 1 || ITER == 3
  // timer counts at 1 MHz
  static const uint16_t timer_divider = 80;
#elif ITER == 2
  // timer counts at 10 MHz 
  static const uint16_t timer_divider = 8;
#endif
  static const uint64_t timer_max_count = 1000000;
  static const TickType_t task_delay = 2000 / portTICK_PERIOD_MS;

// Pins 
#if ITER == 1
  static const int led_pin = GPIO_NUM_0;
#elif ITER == 3
  static const int adc_pin = A0;
#endif

// Globals
static hw_timer_t *timer = NULL;

#if ITER == 2
  static volatile int isr_counter;
  static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#elif ITER == 3
  static volatile uint16_t val;
  static SemaphoreHandle_t bin_sem = NULL;
#endif

// --- Configure interrupt service routine
void IRAM_ATTR onTimer() {
  #if ITER == 1
    // Toggle LED
    int pin_state = digitalRead(led_pin);
    digitalWrite(led_pin, !pin_state);
  #elif ITER == 2
    portENTER_CRITICAL_ISR(&spinlock);
    isr_counter++;
    portEXIT_CRITICAL_ISR(&spinlock);
  #elif ITER == 3
    BaseType_t task_woken = pdFALSE;

    //perform action (read from ADC)
    val = analogRead(adc_pin);

    // Give semaphore to tell task that new falue is ready
    xSemaphoreGiveFromISR(bin_sem, &task_woken);

    // Exit from ISR
    if (task_woken)
      portYIELD_FROM_ISR();
  #endif
}


// ---------- TASKS -----------
#if ITER == 2 || ITER == 3
// Wait for semaphore and print out ADC value when recieved
void printValues(void * parameters) {
  // Loop forever and wait for the semaphore to become available
  while (1)
  {
    #if ITER == 2
      while (isr_counter > 0)
      {
        // print the values of the counter
        Serial.println(isr_counter);

        // decrement the counter and protect the operatioon with the spinlock
        portENTER_CRITICAL(&spinlock);
        isr_counter--;
        portEXIT_CRITICAL(&spinlock);
      }

      // wait 2 seconds while the isr increments the counter a few times
      vTaskDelay(task_delay);
    #elif ITER == 3
      xSemaphoreTake(bin_sem, portMAX_DELAY);
      Serial.println(val);
    #endif
  }
}
#endif

void setup() {
      // Initialize the serial port
    Serial.begin(115200);

    // Wait a moment to start
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println();
    Serial.println("--- FreeRTOS ISR Critical Section Demo ---");


  #if ITER == 1
    // Configure LED Pin
    pinMode(led_pin, OUTPUT);
  #elif ITER == 3
    // configure ADC pin
    pinMode(adc_pin, INPUT);

    // create the binary semaphore
    bin_sem = xSemaphoreCreateBinary();

    // Force reboot if we cant create the semaphore
    if (bin_sem == NULL)
    {
      Serial.println("Could not create semaphore");
      ESP.restart();
    }

  #endif

    #if ITER == 2 || ITER == 3
    // Start task to print out results
    xTaskCreatePinnedToCore(printValues,
                            "Print Values",
                            1024,
                            NULL,
                            1,
                            NULL,
                            app_cpu
    );
  #endif

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