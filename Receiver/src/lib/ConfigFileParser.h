// ConfigFileParser.h
#pragma once

class SDCardHandler; // Forward declaration

#include "Config.h"
#include "SDCardHandler.h"

#include <map>
#include <string>
#include <vector>

class ConfigFileParser {
public:
  ConfigFileParser(const String& configFilePath, SDCardHandler& sdHandler);
  bool parseConfigFile();
  std::map<uint16_t, CANMessageConfig>& getMessageConfigs();

private:
  String configFilePath;
  std::map<uint16_t, CANMessageConfig> messageConfigs;
  SDCardHandler& sdHandler;  // Reference to SDCardHandler instance
  std::vector<String> splitLine(const String& line, char delimiter);
  bool parseConfigLine(const std::string& line);
};