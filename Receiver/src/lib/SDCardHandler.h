// SDCardHandler.h
#pragma once

#include <FS.h>
#include <SD_MMC.h>
#include <EEPROM.h>
#include <vector>

//#include "ConfigFileParser.h"
#include "Config.h"


class SDCardHandler {
public:
  SDCardHandler();
  bool begin(bool liveSendingMode);
  //bool createNewFile(char* filename);
  bool appendToFile(const CANMessageConfig& config);
  bool createDir(const CANMessageConfig& config);
  File openFile(const char* filename, const char* mode);
  void resetCounter();

  bool liveSendingMode;

private:
  uint8_t counterBytes[sizeof(uint16_t)];  // Add this line

  void loadCounterFromEEPROM();
  void incrementAndStoreCounter();
  void saveMarkedParameters(const CANMessageConfig& config, uint32_t lapNumber);
};