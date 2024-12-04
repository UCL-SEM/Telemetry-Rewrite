// CANMessageParser.h
#pragma once

#include <map>
#include <vector>
#include "esp_err.h"
#include "esp_log.h"

#include "Config.h"
//#include "ConfigFileParser.h"

class CANMessageParser {
public:
  CANMessageParser(const std::map<uint16_t, CANMessageConfig>& messageConfigs);
  bool parseMessage(const twai_message_t& message);

  // Getters for parsed data
  const std::vector<uint8_t>& getDataForMessage(uint16_t messageId) const;
  void displayDataForMessage(uint16_t messageId);

private:
  std::map<uint16_t, std::vector<uint8_t>> parsedData;
  const std::map<uint16_t, CANMessageConfig>& messageConfigs;

  template<typename T>
  void byte2Type(T& v, const byte b[4]);
};