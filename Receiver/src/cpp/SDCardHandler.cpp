// SDCardHandler.cpp


//#include "Config.h"
#include "lib/SDCardHandler.h"

uint32_t fileCounter;
extern char radioPacket[128];  // Adjust size as needed
extern uint32_t loggedRadioPoints;

/**
 * Default constructor for SDCardHandler class.
 *
 * Initializes the file counter to 0.
 *
 * @return None
 */
SDCardHandler::SDCardHandler() {
  fileCounter = 0;
}

/**
 * Initializes the SDCardHandler object and prepares the SD card for use.
 *
 * This function sets the live sending mode, configures the SD card pins, mounts the SD card,
 * and initializes the EEPROM to store the file counter.
 *
 * @param liveSendingMode a boolean indicating whether to enable live sending mode
 *
 * @return true if the initialization is successful, false otherwise
 */
bool SDCardHandler::begin(bool liveSendingMode) {
  this->liveSendingMode = liveSendingMode;
  if (!SD_MMC.setPins(CLK_PIN, CMD_PIN, D0_PIN, D1_PIN, D2_PIN, D3_PIN)) {
    printf("Pin change failed!\n");
    return false;
  }

  if (!SD_MMC.begin("/sdcard", false, false, 40000)) {
    printf("ERROR: Card Mount Failed.\n");
    return false;
  }

  EEPROM.begin(sizeof(uint32_t));
  loadCounterFromEEPROM();
  incrementAndStoreCounter();

  return true;
}

/**
 * Loads the file counter from the EEPROM.
 *
 * This function reads the stored counter value from the EEPROM, updates the fileCounter variable,
 * and initializes the EEPROM with a default value if it is not already initialized.
 *
 * @return None
 */
void SDCardHandler::loadCounterFromEEPROM() {
  for (int i = 0; i < sizeof(uint16_t); i++) {
    EEPROM.get(EEPROM_COUNTER_ADDRESS_START + i, counterBytes[i]);
  }
  memcpy(&fileCounter, counterBytes, sizeof(uint16_t));
  if (fileCounter == 0 || fileCounter == 0xFFFF) {
    // EEPROM is not initialized or counter is at max, set a default value
    fileCounter = 1;
    memcpy(counterBytes, &fileCounter, sizeof(uint16_t));
    for (int i = 0; i < sizeof(uint16_t); i++) {
      EEPROM.put(EEPROM_COUNTER_ADDRESS_START + i, counterBytes[i]);
    }
    EEPROM.commit();
  }
}

/**
 * Increments the file counter and stores it in the EEPROM.
 *
 * This function increments the fileCounter variable, resets it to 0 if it reaches the maximum limit,
 * and then stores the updated counter value in the EEPROM.
 *
 * @return None
 */
void SDCardHandler::incrementAndStoreCounter() {
  fileCounter++;
  if (fileCounter >= MAX_FILE_COUNT) {
    fileCounter = 0;  // Reset counter if it reaches the maximum limit
  }
  memcpy(counterBytes, &fileCounter, sizeof(uint16_t));
  for (int i = 0; i < sizeof(uint16_t); i++) {
    EEPROM.put(EEPROM_COUNTER_ADDRESS_START + i, counterBytes[i]);
  }
  EEPROM.commit();
}

/**
 * Resets the file counter to its initial value and stores it in the EEPROM.
 *
 * @return None
 */
void SDCardHandler::resetCounter() {
  // Reset the file counter to 1
  fileCounter = 1;
  memcpy(counterBytes, &fileCounter, sizeof(uint16_t));
  for (int i = 0; i < sizeof(uint16_t); i++) {
    EEPROM.put(EEPROM_COUNTER_ADDRESS_START + i, counterBytes[i]);
  }
  EEPROM.commit();
  printf("File counter reset.\n");
}

/**
 * Opens a file on the SD card.
 *
 * @param filename The name of the file to open.
 * @param mode The mode in which to open the file.
 *
 * @return A File object representing the opened file.
 */
File SDCardHandler::openFile(const char* filename, const char* mode) {
  return SD_MMC.open(filename, mode);
}

/**
 * Appends a CAN message configuration to a file on the SD card.
 *
 * The file is opened in append mode, and the message configuration is written to it in a formatted string.
 * If the file cannot be opened, the function returns false.
 * If the message configuration is a radio message and live sending mode is enabled, the function sets a flag to send the packet immediately.
 *
 * @param config The CAN message configuration to append to the file.
 *
 * @return True if the message was successfully appended to the file, false otherwise.
 */
bool SDCardHandler::appendToFile(const CANMessageConfig& config) {
  char filename[128];
  sprintf(filename, "/%s%s%05d.txt", config.name.c_str(), FILENAME_BASE, fileCounter);

  File file = SD_MMC.open(filename, FILE_APPEND);
  if (!file) {
    printf("Failed to open file: %s\n", filename);
    return false;
  }

  file.printf("%d,%" PRId64 ",%s,%s\n",
              lapNumber,
              config.param1.lastReadingTime,
              config.param1.lastValue.c_str(),
              config.param2.lastValue.c_str());
  file.close();

  if (config.isRadioMessage && !liveSendingMode) {
    saveMarkedParameters(config, lapNumber);
    loggedRadioPoints++;
  } else if (config.isRadioMessage && liveSendingMode) {
    // Set the flag for the radio task to send the packet immediately
    if (config.param2.type != 'N') {
      snprintf(radioPacket, sizeof(radioPacket), "%d, %" PRId64 ", %s, %s, %s, %s, %s, %s\n",
               lapNumber,
               config.param1.lastReadingTime,
               config.param1.name.c_str(),
               config.param1.lastValue.c_str(),
               config.param1.unit.c_str(),
               config.param2.name.c_str(),
               config.param2.lastValue.c_str(),
               config.param2.unit.c_str());
    } else {
      snprintf(radioPacket, sizeof(radioPacket), "%d, %" PRId64 ", %s, %s, %s\n",
               lapNumber,
               config.param1.lastReadingTime,
               config.param1.name.c_str(),
               config.param1.lastValue.c_str(),
               config.param1.unit.c_str());
    }
    printf("wanna send live\n");
    xSemaphoreGive(transmitDataSemaphore);
  }
  return true;
}

/**
 * Creates a directory on the SD card based on the provided CAN message configuration.
 *
 * The function creates a directory with the name specified in the config, and a file within that directory.
 * If the directory or file cannot be created, the function returns false.
 * If the config is a radio message and the "/Radio" directory does not exist, it is created.
 *
 * @param config The CAN message configuration used to determine the directory and file names.
 *
 * @return True if the directory and file were successfully created, false otherwise.
 */
bool SDCardHandler::createDir(const CANMessageConfig& config) {
  static bool isRadioDir = false;
  char filepath[128];
  char filename[128];

  sprintf(filepath, "/%s", config.name.c_str());
  sprintf(filename, "/%s%s%05d.txt", config.name.c_str(), FILENAME_BASE, fileCounter);

  printf("Creating Dir: %s\n", filepath);
  if (SD_MMC.mkdir(filepath)) {
    printf("Dir created\n");
    File file = SD_MMC.open(filename, FILE_APPEND);
    if (!file) {
      printf("Failed to open file: %s\n", filename);
      return false;
    }
    file.printf("Lap,Unix (us),%s (%s),%s (%s)\n",
                config.param1.name.c_str(),
                config.param1.unit.c_str(),
                config.param2.name.c_str(),
                config.param2.unit.c_str());
    file.close();

    if (config.isRadioMessage && !isRadioDir) {
      SD_MMC.mkdir("/Radio");
      isRadioDir = true;
      sprintf(filename, "/Radio/data_%05d.txt", fileCounter);
      File file = SD_MMC.open(filename, FILE_APPEND);
      if (!file) {
        printf("Failed to open file: %s\n", filename);
        return false;
      }
      file.close();
    }
  } else {
    printf("mkdir failed\n");
    return false;
  }
  return true;
}

/**
 * Saves marked parameters from a CAN message configuration to a file on the SD card.
 *
 * The function opens a file in the "/Radio" directory and appends the marked parameters to it.
 * If the file cannot be opened, an error message is printed and the function returns.
 *
 * @param config The CAN message configuration containing the marked parameters to be saved.
 * @param lapNumber The current lap number to be included in the saved data.
 *
 * @return None
 */
void SDCardHandler::saveMarkedParameters(const CANMessageConfig& config, uint32_t lapNumber) {
  char filename[128];
  sprintf(filename, "/Radio/data_%05d.txt", fileCounter);
  File file = SD_MMC.open(filename, FILE_APPEND);
  if (!file) {
    printf("Failed to open file: %s\n", filename);
    return;
  }
  if (config.param2.type != 'N') {
    file.printf("%d, %" PRId64 ", %s, %s, %s, %s, %s, %s\n",
                lapNumber,
                config.param1.lastReadingTime,
                config.param1.name.c_str(),
                config.param1.lastValue.c_str(),
                config.param1.unit.c_str(),
                config.param2.name.c_str(),
                config.param2.lastValue.c_str(),
                config.param2.unit.c_str());
  } else {
    file.printf("%d, %" PRId64 ", %s, %s, %s\n",
                lapNumber,
                config.param1.lastReadingTime,
                config.param1.name.c_str(),
                config.param1.lastValue.c_str(),
                config.param1.unit.c_str());
  }
  file.close();
}