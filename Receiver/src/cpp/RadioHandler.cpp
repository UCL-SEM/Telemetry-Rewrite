#include <map>

#include "lib/RadioHandler.h"
#include "lib/SDCardHandler.h"

extern char radioPacket[128];  // Adjust size as needed
extern uint32_t sentRadioPoints;

/**
 * Default constructor for RadioHandler class.
 * Initializes the radio and radio_reliable objects with the specified pins and address.
 * Sets the radioActive flag to false.
 *
 * @return None
 */
RadioHandler::RadioHandler()
  : radio(RFM69_CS, RFM69_INT), radio_reliable(radio, MY_ADDRESS), radioActive(false) {
}

/**
 * Initializes the radio module and sets up the communication parameters.
 *
 * This function initializes the radio module, sets the frequency, transmission power, and encryption key.
 * It also configures the modem and sets up the reliable datagram protocol.
 * If the initialization fails, it returns false.
 *
 * @param liveSendingMode a flag indicating whether to enable live sending mode
 *
 * @return true if the initialization is successful, false otherwise
 */
bool RadioHandler::begin(bool liveSendingMode) {
  this->liveSendingMode = liveSendingMode;
  SPI.begin(12, 13, 11);

  pinMode(RFM69_RST, OUTPUT);

  // Radio manual reset
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!radio_reliable.init()) {
    printf("Radio init failed\n");
    return false;
  }
  if (!radio.setFrequency(RF69_FREQ)) {
    return false;
  }

  radio.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the client
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  radio.setEncryptionKey(key);

  radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250);

  radio_reliable.setRetries(4);
  radio_reliable.setTimeout(50);
  radioActive = true;

  xTaskCreatePinnedToCore(transmitDataTask, "TransmitDataTask", 4096, this, 1, NULL, 1);

  return true;
}

/**
 * Sends data over the radio module.
 *
 * @param data The data to be sent.
 * @param len The length of the data to be sent.
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::sendData(const uint8_t* data, uint8_t len) {
  if (!radioActive) return;
  // Copy data to a non-const buffer if necessary
  uint8_t* nonConstData = const_cast<uint8_t*>(data);
  if (radio_reliable.sendtoWait(nonConstData, len, DEST_ADDRESS)) {
  } else {
    printf("Radio sending failed. No acknowledgment.\n");
  }
  //radio.waitPacketSent();
}

/**
 * Receives data from the radio module.
 *
 * This function checks if the radio is active and if there is any available data.
 * If data is available, it reads the data into a buffer and handles it accordingly.
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::receiveData() {
  if (!radioActive) return;
  if (radio.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);
    if (radio.recv(buf, &buflen)) {
      // Handle received data
    }
  }
}

/**
 * Deactivates the radio module by putting it into sleep mode to save power.
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::deactivateRadio() {
  // Put the radio into sleep mode to save power
  if (!radio.sleep()) {
    printf("Radio failed to go to sleep.\n");
  } else {
    radioActive = false;
  }
}

/**
 * Activates the radio module by waking it from sleep mode and setting it to idle.
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::activateRadio() {
  radio.setModeIdle();  // Wake the radio from sleep mode
  radioActive = true;
}

/**
 * Checks if the radio module is currently active.
 *
 * @return true if the radio is active, false otherwise
 */
bool RadioHandler::isRadioActive() const {
  return radioActive;
}

/**
 * Transmits data from a file over radio to the radio station.
 *
 * This function opens the specified file, reads its contents line by line, and sends each line over radio to the radio station.
 * It also keeps track of the last lap sent and the number of radio points sent.
 *
 * @param filename the name of the file to read from
 * @param lastLapSent a reference to the last lap sent, updated by this function
 * @param lapLineTracker a reference to a map of lap numbers to last line sent, updated by this function
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::transmitDataFromFile(const char* filename, uint32_t& lastLapSent, std::map<uint32_t, uint32_t>& lapLineTracker) {
  //if (millis() - lastCompletionTime > 10000) {
    if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      bool semaphoreTaken = true;
      if (!radioActive) {
        activateRadio();
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      File file = SD_MMC.open(filename, FILE_READ);
      if (!file) {
        printf("Failed to open file: %s\n", filename);
        xSemaphoreGive(sdCardMutex);
        return;
      }

      int linesRead = 0;

      while (file.available()) {
        if (!semaphoreTaken) {
          if (xSemaphoreTake(sdCardMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            semaphoreTaken = true;
          } else {
            semaphoreTaken = false;
          }
        }
        String line = file.readStringUntil('\n');
        uint32_t lapNumberInFile;
        sscanf(line.c_str(), "%d,", &lapNumberInFile);
        if (lapNumberInFile <= lapNumber) {                                                           // Adjusted condition to continue sending data for the current lap
          if (lapLineTracker[lapNumberInFile] == 0 || lapLineTracker[lapNumberInFile] < linesRead) {  // Check if this line has already been sent for this lap
            printf("Sending data for lap: %d, Sent radio point #%d\n", lapNumberInFile, sentRadioPoints);                                    // Debugging output
            sendData((const uint8_t*)line.c_str(), line.length());
            lapLineTracker[lapNumberInFile] = linesRead;  // Update the last line sent for this lap
            lastLapSent = lapNumberInFile;                // Update lastLapSent after sending
            sentRadioPoints++;
            if (semaphoreTaken) {
              xSemaphoreGive(sdCardMutex);
              semaphoreTaken = false;
            }
          }
        } else {
          break;  // We've reached laps that have not been completed yet, stop transmitting
        }
        linesRead++;
        if (linesRead % 5 == 0) {  // Yield control back to the FreeRTOS scheduler periodically
          vTaskDelay(pdMS_TO_TICKS(0));  // Yield control
        }
      }

      file.close();
      printf("Finished sending radio data\n");
      //xSemaphoreGive(transmitDataSemaphore);
      //lastCompletionTime = millis();
      if (semaphoreTaken) {
        xSemaphoreGive(sdCardMutex);
        semaphoreTaken = false;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      deactivateRadio();
    } else {
      printf("Failed to take the mutex, failed sending radio data\n");
    }
  //}
}

/**
 * Starts the radio transmission task, which continuously sends data over radio.
 *
 * If live sending mode is disabled, it reads data from a file and sends it over radio.
 * If live sending mode is enabled, it sends radio packets directly.
 *
 * @param param a pointer to the RadioHandler instance
 *
 * @return None
 *
 * @throws None
 */
void RadioHandler::transmitDataTask(void* param) {
  RadioHandler* radioHandlerInstance = static_cast<RadioHandler*>(param);
  if (!radioHandlerInstance->liveSendingMode) {
    char filename[128];
    sprintf(filename, "/Radio/data_%05d.txt", fileCounter);
    for (;;) {
      static uint32_t lastLapSent = 0;
      static std::map<uint32_t, uint32_t> lapLineTracker;
      if (xSemaphoreTake(transmitDataSemaphore, 1000) == pdTRUE) {
        radioHandlerInstance->transmitDataFromFile(filename, lastLapSent, lapLineTracker);
        //xSemaphoreGive(transmitDataSemaphore); // If you uncomment this line, it will trigger a continous loop of giving and taking the semaphore for every new value...
      }
    }
  } else if (radioHandlerInstance->liveSendingMode) {
    for (;;) {
      if (xSemaphoreTake(transmitDataSemaphore, 1000) == pdTRUE) {
        if (!radioHandlerInstance->radioActive) {
          radioHandlerInstance->activateRadio();
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        radioHandlerInstance->sendData((uint8_t*)radioPacket, strlen(radioPacket));
        printf("sent radio packet\n");
        //xSemaphoreGive(transmitDataSemaphore);
      }
    }
  }
}