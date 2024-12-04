// ConfigFileParser.cpp
#include <FS.h>
#include <SD_MMC.h>
#include <sstream>

//#include "Config.h"
#include "lib/ConfigFileParser.h"
//#include "SDCardHandler.h"

/**
 * Constructs a ConfigFileParser object with the specified configuration file path and SDCardHandler reference.
 *
 * @param configFilePath the path to the configuration file
 * @param sdHandler a reference to the SDCardHandler instance
 */
ConfigFileParser::ConfigFileParser(const String& configFilePath, SDCardHandler& sdHandler)
  : configFilePath(configFilePath), sdHandler(sdHandler) {}

/**
 * Parses the configuration file at the specified path.
 *
 * Opens the file, skips the first line, and then reads the file line by line, parsing each line using the parseConfigLine function.
 * If any line fails to parse, an error message is printed and the function continues with the next line.
 *
 * @return True if the file was successfully parsed, false otherwise.
 */
bool ConfigFileParser::parseConfigFile() {

  File configFile = sdHandler.openFile(configFilePath.c_str(), FILE_READ);
  if (!configFile) {
    printf("Failed to open config file!\n");
    return false;
  }

  printf("Opened config file");

  // Skip the first line
  if (configFile.available()) {
    configFile.readStringUntil('\n');
  }

  while (configFile.available()) {
    std::string line = configFile.readStringUntil('\n').c_str();
    if (!parseConfigLine(line.c_str())) {
      printf("Failed to parse config line: %s\n", line.c_str());
    }
  }

  configFile.close();
  return true;
}

std::map<uint16_t, CANMessageConfig>& ConfigFileParser::getMessageConfigs() {
  return messageConfigs;
}

/**
 * Parses a single line from the configuration file.
 *
 * The line is split into fields by commas, and the fields are used to populate a CANMessageConfig object.
 * The object is then added to the messageConfigs map.
 *
 * @param line the line to parse from the configuration file
 *
 * @return true if the line was successfully parsed, false otherwise
 */
bool ConfigFileParser::parseConfigLine(const std::string& line) {
  std::stringstream ss(line);
  std::string field;
  std::vector<String> fields;

  // Split the line by commas and extract the fields
  while (std::getline(ss, field, ',')) {
    fields.push_back(String(field.c_str()));
  }

  // Check if the line has the correct number of fields
  if (fields.size() < 6 || fields.size() > 10 || (fields.size() > 6 && fields.size() < 10)) {  // Adjusted to allow for single parameters
    return false;
  }

  String messageName = fields[0];
  bool isRadioMessage = messageName.startsWith("@");  // Correct usage
  if (isRadioMessage) {
    messageName.remove(0, 1);  // Remove the "@" from the message name
  }
  // Create a new CANMessageConfig object and populate it with the extracted data
  CANMessageConfig config;
  config.name = messageName;
  config.isRadioMessage = isRadioMessage;
  if (config.name == "GPST") {
    config.isGPSTimeMessage = true;
  } else {
    config.isGPSTimeMessage = false;
  }
  printf("Raw ID from config file: %s\n", fields[1].c_str());
  config.id = std::stoul(fields[1].c_str(), nullptr, 10);
  printf("Converted ID (decimal): %d\n", config.id);

  // Determine if there's only one parameter
  bool singleParameter = (fields.size() == 6);

  if (singleParameter) {
    // Handle single parameter cas
    config.param1.name = fields[2];
    config.param1.unit = fields[3];
    config.param1.type = fields[4][0];
    try {
      if (!fields[5].isEmpty()) {
        int bits = std::stoi(fields[5].c_str(), nullptr, 10);  // Use std::stoi for signed integers
        if (bits >= 0 && bits <= UINT8_MAX) {
          config.param1.bits = static_cast<uint8_t>(bits);
        } else {
          printf("Bits value out of range for uint8_t: %d\n", bits);
          return false;
        }
      } else {
        printf("Empty string for bits. Skipping.\n");
        return false;
      }
    } catch (const std::invalid_argument& e) {
      printf("Invalid argument for bits: %s\n", e.what());
      return false;
    } catch (const std::out_of_range& e) {
      printf("Out of range for bits: %s\n", e.what());
      return false;
    }
    config.param1.lastReadingTime = 0;

    config.param2.name = "null";
    config.param2.unit = "null";
    config.param2.type = 'N';  // Placeholder for null type
    config.param2.bits = 0;
    config.param2.lastReadingTime = 0;
    // Assuming no timestamp for single parameter case
  } else {
    // Handle two parameters case
    config.param1.name = fields[2];
    config.param1.unit = fields[3];
    config.param1.type = fields[4][0];

    try {
      if (!fields[5].isEmpty()) {
        int bits = std::stoi(fields[5].c_str(), nullptr, 10);  // Use std::stoi for signed integers
        if (bits >= 0 && bits <= UINT8_MAX) {
          config.param1.bits = static_cast<uint8_t>(bits);
        } else {
          printf("Bits value out of range for uint8_t: %d\n", bits);
          return false;
        }
      } else {
        printf("Empty string for bits. Skipping.\n");
        return false;
      }
    } catch (const std::invalid_argument& e) {
      printf("Invalid argument for bits: %s\n", e.what());
      return false;
    } catch (const std::out_of_range& e) {
      printf("Out of range for bits: %s\n", e.what());
      return false;
    }
    config.param1.lastReadingTime = 0;
    config.param2.name = fields[6];
    config.param2.unit = fields[7];
    config.param2.type = fields[8][0];
    try {
      if (!fields[9].isEmpty()) {
        int bits = std::stoi(fields[9].c_str(), nullptr, 10);  // Use std::stoi for signed integers
        if (bits >= 0 && bits <= UINT8_MAX) {
          config.param2.bits = static_cast<uint8_t>(bits);
        } else {
          printf("Bits value out of range for uint8_t: %d\n", bits);
          return false;
        }
      } else {
        printf("Empty string for bits. Skipping.\n");
        return false;
      }
    } catch (const std::invalid_argument& e) {
      printf("Invalid argument for bits: %s\n", e.what());
      return false;
    } catch (const std::out_of_range& e) {
      printf("Out of range for bits: %s\n", e.what());
      return false;
    }
    config.param2.lastReadingTime = 0;
  }

  // Add the new CANMessageConfig object to the messageConfigs map
  messageConfigs[config.id] = config;

  sdHandler.createDir(config);

  return true;
}

/**
 * Splits a given line into components based on a specified delimiter.
 *
 * @param line the input line to be split
 * @param delimiter the character used to split the line
 *
 * @return a vector of strings representing the components of the input line
 */
std::vector<String> ConfigFileParser::splitLine(const String& line, char delimiter) {
  std::vector<String> components;
  int start = 0;
  int end = line.indexOf(delimiter);
  while (end != -1) {
    components.push_back(line.substring(start, end - start));
    start = end + 1;
    end = line.indexOf(delimiter, start);
  }
  components.push_back(line.substring(start));  // Add the last component
  return components;
}
