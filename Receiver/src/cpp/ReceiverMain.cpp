#include <RHReliableDatagram.h>
#include <Adafruit_NeoPixel.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_system.h"
#include <RH_RF69.h>
#include "esp_err.h"
#include "esp_log.h"
#include <stdlib.h>
#include "SD_MMC.h"
#include <stdio.h>
#include <SPI.h>
#include "FS.h"
#include <map>
#include <EEPROM.h>

#include "lib/Receiver.h"

#define ESP_EXCEPTION_DECODER

Receiver receiver("/config/README.txt");

SemaphoreHandle_t dataMutex;  // Mutex to protect shared data

void CANHandlerTask(void *pvParameters) {
  for (;;) {
    // Handle button press events here
    receiver.receiveCAN(dataMutex, true);
    //receiver.displayUpdatedParameters();
    vTaskDelay(pdMS_TO_TICKS(0));  // Adjust delay according to your application requirements
  }
}

void setup() {
  Serial.begin(115200);


  delay(3000);  // delay to let serial begin have a breather huff huff

  receiver.begin(true);

  receiver.printParameters();
  //receiver.initLapFence(51.5280372, -0.1426583, 51.5280462, -0.1425165, 51.5279792, -0.1425112, 51.5279757, -0.1426627);    // NW1 3QS Compton Close Park
  //receiver.initRadioFence(51.5280363, -0.1430492, 51.5280545, -0.1422529, 51.5279662, -0.1422228, 51.5279683, -0.1430449);  // NW1 3QS Compton Close Park

  //receiver.initLapFence(51.4199761, 0.3482651, 51.4200143, 0.3483264, 51.4198693, 0.3488128, 51.4198191, 0.3487607);    // CycloPark
  //receiver.initRadioFence(51.4205795, 0.3484007, 51.4206552, 0.3484280, 51.4202241, 0.3496966, 51.4201799, 0.3496453);  // CycloPark

  receiver.initLapFence(43.7708310, -0.0408539, 43.7709265, -0.0406479, 43.7706997, -0.0404429, 43.7706075, -0.0406456);    // Nogaro track
  receiver.initRadioFence(43.7719449, -0.0423221, 43.7723014, -0.0417674, 43.7712505, -0.0403314, 43.7707955, -0.0409531);  // Nogaro track

  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    printf("Failed to create mutex\n");
    // Handle the error appropriately, e.g., stop the program
    while (1)
      ;
  }

  xTaskCreatePinnedToCore(
    CANHandlerTask,    // Task function
    "CANHandlerTask",  // Task name
    4096,              // Stack size (bytes)
    NULL,              // Task parameters
    3,                 // Task priority
    NULL,              // Task handle
    0                  // Core to run the task (0 or 1)
  );
}

void loop() {}