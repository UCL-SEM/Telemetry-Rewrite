// ModeConfig.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

#include "Config.h"

class Receiver;

class ModeConfig {
public:
  ModeConfig(Receiver& receiver);
  void begin();
  static void handleButtonInterrupt();

private:
  Receiver& receiver;
  static void buttonTask(void* param);
  void handleButtonPress();
  static ModeConfig* instance;

  TaskHandle_t buttonTaskHandle;
  SemaphoreHandle_t buttonSemaphore;

  // Button press states
  enum ButtonState {
    IDLE,
    SINGLE_PRESS_DETECTED,
    WAITING_FOR_DOUBLE_PRESS,
    DOUBLE_PRESS_DETECTED,
    LONG_PRESS_DETECTED,
    VERY_LONG_PRESS_DETECTED
  };

  unsigned long pressStartTime = 0;
  unsigned long pressEndTime = 0;
  unsigned long lastPressEndTime = 0;
  int pressCount = 0;
  bool waitingForDoublePress = false;
  bool isButtonPressed;

  void singlePressFunction();
  void doublePressFunction();
  void longPressFunction();
  void veryLongPressFunction();
};