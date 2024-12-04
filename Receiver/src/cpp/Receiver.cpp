// Receiver.cpp
#include "lib/Receiver.h"

uint32_t lapNumber = 1; // Starts on lap 1
uint32_t loggedRadioPoints = 0;
uint32_t sentRadioPoints = 0;
SemaphoreHandle_t transmitDataSemaphore = xSemaphoreCreateBinary();
SemaphoreHandle_t sdCardMutex;

char radioPacket[128];  // // Define a static buffer large enough to hold the packet. Adjust size as needed.

/**
 * Constructs a new Receiver instance.
 *
 * @param configFilePath Path to the configuration file.
 *
 * @throws std::runtime_error If the configuration file cannot be parsed.
 */
Receiver::Receiver(const String& configFilePath)
  : sdHandler(), configParser(configFilePath, sdHandler), canParser(configParser.getMessageConfigs()), modeConfig(*this) /*, radioHandler() */ {
}

/**
 * Initializes the Receiver object by setting up the SD card, radio, configuration parser, and CAN interface.
 *
 * @param liveSendingMode Flag indicating whether the receiver should operate in live sending mode.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::begin(bool liveSendingMode) {

  this->liveSendingMode = liveSendingMode;

  sdCardMutex = xSemaphoreCreateMutex();
  if (sdCardMutex == NULL) {
    ;
    printf("sd mutex failed to creat\n");
  }

  if (!sdHandler.begin(liveSendingMode)) {
    printf("Failed to initialize SD card\n");
    return;
  }

  if (!radioHandler.begin(liveSendingMode)) {
    Serial.println("Failed to initialize radio");
    return;
  }

  if (!configParser.parseConfigFile()) {
    printf("Failed to parse configuration file\n");
    return;
  }

  if (!CANInit()) {
    printf("CAN initialisation failed. Please verify the CANInit() function in the Receiver.cpp file.\n");
    return;
  }

  syncGPSTime();

  modeConfig.begin();

  printStatementsEnabled = EEPROM.read(PRINT_FLAG_EEPROM_ADDRESS);
}

/**
 * Receives a CAN message and handles it if the message is valid.
 *
 * @param dataMutex A semaphore handle used to protect access to shared data.
 * @param displayParameters A boolean indicating whether to display (serial port) updated parameters after handling the message.
 *
 * @return None
 */
void Receiver::receiveCAN(SemaphoreHandle_t& dataMutex, bool displayParameters) {
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK && !(message.rtr)) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    handleCANMessage(message);
    if (displayParameters) {
      displayUpdatedParameters();
    }
    xSemaphoreGive(dataMutex);
  }
}

/**
 * Formats parameter data based on its type.
 *
 * @param data The vector of bytes containing the parameter data.
 * @param start The starting index of the parameter data in the vector.
 * @param param The parameter object containing information about the data type and size.
 *
 * @return A string representation of the formatted parameter data.
 */
String formatParameterData(const std::vector<uint8_t>& data, size_t start, const Parameter& param) {
  String formattedData;
  size_t end = start + param.bits / 8;

  switch (param.type) {
    case 'U':
      {
        uint32_t value = 0;
        for (size_t i = start; i < end; i++) {
          value = (value << 8) | data[i];
        }
        formattedData = String(value);
        break;
      }
    case 'I':
      {
        int32_t value = 0;
        for (size_t i = start; i < end; i++) {
          value = (value << 8) | data[i];
        }
        formattedData = String(value);
        break;
      }
    case 'F':
      {
        float value = 0;
        memcpy(&value, &data[start], sizeof(float));
        formattedData = String(value);
        break;
      }
  }

  return formattedData;
}

/**
 * Handles a CAN message by parsing its data and updating the corresponding parameter values.
 * It also checks for lap increments, batch data sending, and logs data onto the SD card.
 *
 * @param message The CAN message to be handled.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::handleCANMessage(const twai_message_t& message) {
  if (!canParser.parseMessage(message)) {
    printf("Unknown message identifier: %d\n", message.identifier);
    return;
  }

  const std::vector<uint8_t>& data = canParser.getDataForMessage(message.identifier);
  CANMessageConfig& config = configParser.getMessageConfigs().at(message.identifier);

  // Update the last value of the parameters and set newValueReceived to true
  switch (config.param1.type) {
    case 'U':
      {
        uint32_t value = 0;
        for (size_t i = 0; i < config.param1.bits / 8; i++) {
          value = (value << 8) | data[i];
        }
        config.param1.lastValue = String(value);
        break;
      }
    case 'I':
      {
        int32_t value = 0;
        for (size_t i = 0; i < config.param1.bits / 8; i++) {
          value = (value << 8) | data[i];
        }
        config.param1.lastValue = String(value);
        break;
      }
    case 'F':
      {
        float value = 0;
        memcpy(&value, &data[0], sizeof(float));
        config.param1.lastValue = String(value);
        break;
      }
  }
  // Update lastReadingTime with the current Unix time in microseconds
  uint64_t currentTimeMicroseconds = getCurrentUnixTimeMicroseconds();
  config.param1.lastReadingTime = currentTimeMicroseconds;
  config.param1.newValueReceived = true;

  if (config.param2.type != 'N') {
    switch (config.param2.type) {
      case 'U':
        {
          uint32_t value = 0;
          for (size_t i = 0; i < config.param2.bits / 8; i++) {
            value = (value << 8) | data[i + 4];
          }
          config.param2.lastValue = String(value);
          break;
        }
      case 'I':
        {
          int32_t value = 0;
          for (size_t i = 0; i < config.param2.bits / 8; i++) {
            value = (value << 8) | data[i + 4];
          }
          config.param2.lastValue = String(value);
          break;
        }
      case 'F':
        {
          float value = 0;
          memcpy(&value, &data[4], sizeof(float));
          config.param2.lastValue = String(value);
          break;
        }
    }
    config.param2.lastReadingTime = currentTimeMicroseconds;
    config.param2.newValueReceived = true;
  }

  if (config.name == "GPS1") {
    static uint32_t lastLapIncrementTime = 0;
    if (lapFence.isInside(config.param1.lastValue.toFloat() / 1E7, config.param2.lastValue.toFloat() / 1E7)) {
      uint32_t currentTime = millis();
      if (currentTime - lastLapIncrementTime > LAP_INCREMENT_THRESHOLD) {  // Define LAP_INCREMENT_THRESHOLD as per your requirement in Config.h
        lapNumber++;
        lastLapIncrementTime = currentTime;
        printf("Lap number incremented to: %d\n", lapNumber);
      }
    } else {
      printf("not inside lap fence\n");
    }
    // Check if the current position is inside the batch data sending geofence
    //uint32_t currentTime = millis();
    //if (currentTime - radioHandler.lastCompletionTime > 10000) {
    printf("logged radio point#%d\n", loggedRadioPoints);
    printf("sent radio point#%d\n", sentRadioPoints);
    if (loggedRadioPoints - sentRadioPoints > 100) {
      if ((batchDataFence.isInside(config.param1.lastValue.toFloat() / 1E7, config.param2.lastValue.toFloat() / 1E7)) && (!liveSendingMode)) {
        xSemaphoreGive(transmitDataSemaphore);  // Send batch data
      }
    }
  }

  updateManager.markParameterUpdated(message.identifier);

  if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    // Mutex taken, safe to access the SD card
    sdHandler.appendToFile(config);
    // Release the mutex to allow other tasks to access the SD card
    xSemaphoreGive(sdCardMutex);
  } else {
    // Failed to take the mutex within the timeout period
    printf("failed to log data onto sd card\n");
  }
}

/**
 * Prints the CAN message parameters stored in the configuration parser.
 *
 * This function retrieves the map of CAN message configurations from the configuration parser,
 * then iterates over the map to print each configuration, including the message name, ID,
 * and parameter details.
 *
 * @return None
 */
void Receiver::printParameters() {
  std::map<uint16_t, CANMessageConfig> parameters = configParser.getMessageConfigs();
  printf("Parameter number: %d\n", parameters.size());
  // Iterate over the map and print each CANMessageConfig
  for (const auto& entry : parameters) {
    const CANMessageConfig& config = entry.second;
    printf("Message Name: %s\n", config.name.c_str());
    printf("Message ID: %d\n", config.id);
    printf("Parameter 1: %s (%s) - Type: %c, Bits: %d\n", config.param1.name.c_str(), config.param1.unit.c_str(), config.param1.type, config.param1.bits);
    printf("Parameter 2: %s (%s) - Type: %c, Bits: %d\n\n", config.param2.name.c_str(), config.param2.unit.c_str(), config.param2.type, config.param2.bits);
  }
}


/**
 * Displays the updated parameters by iterating over the CAN message configurations and checking for updates.
 *
 * @return None
 */
void Receiver::displayUpdatedParameters() {
  for (const auto& entry : configParser.getMessageConfigs()) {
    const CANMessageConfig& config = entry.second;
    if (updateManager.isParameterUpdated(config.id)) {
      printf("Parameter Updated: %s (Type: %c)\n", config.param1.name.c_str(), config.param1.type);
      printf("Last Value: %s\n", config.param1.lastValue.c_str());
      printf("Last Update Time: %" PRId64 "\n", config.param1.lastReadingTime);
      if (config.param2.type != 'N' && config.param2.newValueReceived) {
        printf("Parameter Updated: %s (Type: %c)\n", config.param2.name.c_str(), config.param2.type);
        printf("Last Value: %s\n", config.param2.lastValue.c_str());
      }
      // Optionally, reset the update status here if you want to display each update only once
      updateManager.resetParameterUpdate(config.id);
    }
  }
}

/**
 * Initializes the CAN driver and starts it.
 *
 * This function sets up the CAN pin modes, initializes the TWAI driver configuration,
 * installs the driver, and starts it. If any of these steps fail, it prints an error
 * message and returns false.
 *
 * @return true if the CAN driver is successfully initialized and started, false otherwise
 */
bool Receiver::CANInit() {
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, LOW);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);  // LISTEN ONLY mode cannot be used alone on a CAN bus.
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    printf("Failed to install driver\n");
    delay(1000);
    return false;
    //esp_restart(); // DON'T OTMQ{E"LTNH MQ{LERH<{"Q:LERGHQ:<E"}}}}... forget about these i mean
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    if (printStatementsEnabled) {
      printf("Failed to start driver\n");
    }
    delay(1000);
    return false;
    //esp_restart(); // DON'T OTN#WO$PWPRGJMQ{E"LTNH MQ{LERH<{"Q:LERGHQ:<E"}}}}... forget about these i mean
  }
  return true;
}

/**
 * Synchronizes the system time with GPS time.
 *
 * This function checks for the presence of a GPS time message configuration,
 * waits for up to 3 seconds to receive a GPS time message, and updates the system time accordingly.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::syncGPSTime() {
  uint32_t startTime = millis();
  bool hasGPSTimeMessageConfig = false;
  for (const auto& configPair : configParser.getMessageConfigs()) {
    if (configPair.second.isGPSTimeMessage) {
      hasGPSTimeMessageConfig = true;
      break;  // No need to continue checking once a match is found
    }
  }
  if (!hasGPSTimeMessageConfig) {  // If no CANMessageConfig instances with isGPSTimeMessage set to true are found, return
    printf("No GPS time message configuration found\n");
    syncTime(0, 0);
    return;
  }
  while (millis() - startTime < 3000) {  // Wait for 3 seconds
    if (twai_receive(&message, pdMS_TO_TICKS(3000)) == ESP_OK && !(message.rtr)) {
      // Iterate through all message configurations to find a match
      for (const auto& configPair : configParser.getMessageConfigs()) {
        const auto& config = configPair.second;
        if (config.isGPSTimeMessage && configPair.first == message.identifier) {
          // Handle GPS time message
          // Assuming the message contains two uint32_t values for seconds and microseconds
          uint32_t seconds = 0;
          uint32_t microseconds = 0;
          memcpy(&seconds, &message.data[0], sizeof(uint32_t));
          memcpy(&microseconds, &message.data[4], sizeof(uint32_t));
          syncTime(seconds, microseconds);
          printf("Synced with GPS time\n");
          return;
        }
      }
    }
  }
  printf("GPS time not received\n");  // If GPS time message is not received within 3 seconds, continue with the MCU's own microsecond timer
  syncTime(0, 0);
}

/**
 * Synchronizes the system time with the provided time values.
 *
 * This function takes seconds and microseconds as input, converts them to a timeval structure,
 * and uses the settimeofday function to update the system time.
 *
 * @param seconds The number of seconds since the Unix epoch.
 * @param microseconds The number of microseconds since the last second.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::syncTime(uint32_t seconds, uint32_t microseconds) {
  uint64_t unixTimeMicroseconds = (static_cast<uint64_t>(seconds) * 1000000) + microseconds;
  time_t tv_sec = static_cast<time_t>(unixTimeMicroseconds / 1000000);
  suseconds_t tv_usec = static_cast<suseconds_t>(unixTimeMicroseconds % 1000000);
  struct timeval tv = { .tv_sec = tv_sec, .tv_usec = tv_usec };
  settimeofday(&tv, NULL);
  printf("Time synchronized to: %ld seconds and %ld microseconds\n", tv_sec, tv_usec);
}

/**
 * Retrieves the current system time in microseconds since the Unix epoch.
 *
 * @return The current system time in microseconds since the Unix epoch.
 */
uint64_t Receiver::getCurrentUnixTimeMicroseconds() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t unixTimeMicroseconds = static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
  return unixTimeMicroseconds;
}

/**
 * Toggles the state of print statements.
 *
 * BROKEN, DOESNT UPDATE EEPROM
 * 
 * This function reads the current state from EEPROM, toggles it, writes the new state back to EEPROM, 
 * and updates the global flag accordingly.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::togglePrintStatements() {
  // Read the current state from EEPROM
  bool currentState = EEPROM.read(PRINT_FLAG_EEPROM_ADDRESS) == 1;

  // Toggle the state
  bool newState = !currentState;

  // Write the new state to EEPROM
  if (newState) {
    EEPROM.write(PRINT_FLAG_EEPROM_ADDRESS, 1);
  } else if (!newState) {
    EEPROM.write(PRINT_FLAG_EEPROM_ADDRESS, 0);
  }
  EEPROM.commit();

  // Update the global flag
  printStatementsEnabled = newState;
  printf("Print statements %d\n", newState);
}

/**
 * Initializes the lap fence with the given coordinates.
 *
 * @param latA The latitude of the first corner of the lap fence.
 * @param lngA The longitude of the first corner of the lap fence.
 * @param latB The latitude of the second corner of the lap fence.
 * @param lngB The longitude of the second corner of the lap fence.
 * @param latC The latitude of the third corner of the lap fence.
 * @param lngC The longitude of the third corner of the lap fence.
 * @param latD The latitude of the fourth corner of the lap fence.
 * @param lngD The longitude of the fourth corner of the lap fence.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::initLapFence(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD) {
  lapFence.init(latA, lngA, latB, lngB, latC, lngC, latD, lngD);
}
/**
 * Initializes the radio fence with the given coordinates.
 *
 * @param latA The latitude of the first corner of the fence.
 * @param lngA The longitude of the first corner of the fence.
 * @param latB The latitude of the second corner of the fence.
 * @param lngB The longitude of the second corner of the fence.
 * @param latC The latitude of the third corner of the fence.
 * @param lngC The longitude of the third corner of the fence.
 * @param latD The latitude of the fourth corner of the fence.
 * @param lngD The longitude of the fourth corner of the fence.
 *
 * @return None
 *
 * @throws None
 */
void Receiver::initRadioFence(float latA, float lngA, float latB, float lngB, float latC, float lngC, float latD, float lngD) {
  batchDataFence.init(latA, lngA, latB, lngB, latC, lngC, latD, lngD);
}