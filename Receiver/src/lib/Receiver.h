// Receiver.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "DataProcessing.h"
#include "ConfigFileParser.h"
#include "CANMessageParser.h"
#include "SDCardHandler.h"
#include "RadioHandler.h"
#include "ModeConfig.h"

class ParameterUpdateManager {
public:
  void markParameterUpdated(uint16_t paramId) {
    if (paramId < MAX_PARAMETERS) {
      updatedParameters[paramId] = true;
      lastUpdateTimes[paramId] = millis();
    }
  }

  bool isParameterUpdated(uint16_t paramId) const {
    return paramId < MAX_PARAMETERS && updatedParameters[paramId];
  }

  unsigned long getLastUpdateTime(uint16_t paramId) const {
    return paramId < MAX_PARAMETERS ? lastUpdateTimes[paramId] : 0;
  }

  void resetParameterUpdate(uint16_t paramId) {
    if (paramId < MAX_PARAMETERS) {
      updatedParameters[paramId] = false;
    }
  }

  void resetAllParameterUpdates() {
    for (int i = 0; i < MAX_PARAMETERS; i++) {
      updatedParameters[i] = false;
    }
  }

private:
  static const int MAX_PARAMETERS = 100;  // Adjust based on your needs
  bool updatedParameters[MAX_PARAMETERS] = { false };
  unsigned long lastUpdateTimes[MAX_PARAMETERS] = { 0 };
};

class Receiver {
public:
  Receiver(const String& configFilePath);
  void begin(bool liveSendingMode = false);
  void printParameters();
  void displayUpdatedParameters();

  char filename[40];

  void togglePrintStatements();
  bool printStatementsEnabled;

  bool liveSendingMode;

  void updateLapCount();

  twai_message_t message;
  twai_status_info_t can_status;
  void receiveCAN(SemaphoreHandle_t& dataMutex, bool displayParameters = false);
  void initLapFence(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD);
  void initRadioFence(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD);
  RadioHandler radioHandler;
public:
  Geofence lapFence;
  Geofence batchDataFence;
  SDCardHandler sdHandler;
  ConfigFileParser configParser;
  CANMessageParser canParser;
  ParameterUpdateManager updateManager;
  ModeConfig modeConfig;
  //DataProcessor dataProcessor;

  bool CANInit();
  void handleCANMessage(const twai_message_t& message);
  //void saveToSDCard(const char* filename, const std::vector<uint8_t>& data, uint16_t messageId);
  void syncGPSTime();
  void syncTime(uint32_t seconds, uint32_t microseconds);
  uint64_t getCurrentUnixTimeMicroseconds();
  //void transmitOverRadio(const std::vector<uint8_t>& data, uint16_t messageId);
};