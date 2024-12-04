// CANMessageParser.cpp
#include "lib/CANMessageParser.h"

/**
 * Constructor for CANMessageParser.
 *
 * @param messageConfigs A map of CAN message ID to CANMessageConfig, which contains information
 * about how to parse the CAN message.
 */
CANMessageParser::CANMessageParser(const std::map<uint16_t, CANMessageConfig>& messageConfigs)
  : messageConfigs(messageConfigs) {}

/**
 * Displays the parsed data for a given CAN message ID.
 *
 * @param messageId The CAN message ID to retrieve the parsed data for.
 */
void CANMessageParser::displayDataForMessage(uint16_t messageId) {
  // Retrieve the parsed data for the given message ID
  const std::vector<uint8_t>& data = getDataForMessage(messageId);

  // Check if data was found for the given message ID
  if (data.empty()) {
    printf("No data found for message ID: %d\n", messageId);
    return;
  }

  // Print the message ID and the corresponding data
  printf("Message ID: %d\n", messageId);
  printf("Data: ");
  for (size_t i = 0; i < data.size(); ++i) {
    printf("%02X", data[i]);
    if (i < data.size() - 1) {
      printf(" ");
    }
  }
  printf("\n");
}

/**
 * Parses a CAN message and extracts its data according to a given configuration.
 *
 * The function takes a CAN message and a configuration map as input. It first
 * finds the configuration for the given message ID in the map. Then, it
 * extracts the data from the message according to the configuration.
 *
 * The data is stored in a vector and returned. If the message ID is not found in
 * the configuration, an empty vector is returned.
 *
 * @param message The CAN message to parse.
 *
 * @return The parsed data from the message.
 */
bool CANMessageParser::parseMessage(const twai_message_t& message) {
  auto it = messageConfigs.find(message.identifier);
  if (it == messageConfigs.end()) {
    // Unknown message identifier
    return false;
  }

  const CANMessageConfig& config = it->second;
  std::vector<uint8_t> data;

  // Parse parameter 1
  switch (config.param1.type) {
    case 'U':
      {
        uint32_t value = 0;
        // Read the value from the message data
        byte2Type(value, message.data);
        // Store the value in the data vector
        data.push_back((value >> 24) & 0xFF);
        data.push_back((value >> 16) & 0xFF);
        data.push_back((value >> 8) & 0xFF);
        data.push_back(value & 0xFF);
        break;
      }
    case 'I':
      {
        int32_t value = 0;
        // Read the value from the message data
        byte2Type(value, message.data);
        // Store the value in the data vector
        data.push_back((value >> 24) & 0xFF);
        data.push_back((value >> 16) & 0xFF);
        data.push_back((value >> 8) & 0xFF);
        data.push_back(value & 0xFF);
        break;
      }
    case 'F':
      {
        float value = 0;
        // Read the value from the message data
        byte2Type(value, message.data);
        // Store the value in the data vector
        data.push_back(reinterpret_cast<uint8_t*>(&value)[0]);
        data.push_back(reinterpret_cast<uint8_t*>(&value)[1]);
        data.push_back(reinterpret_cast<uint8_t*>(&value)[2]);
        data.push_back(reinterpret_cast<uint8_t*>(&value)[3]);
        break;
      }
      // Add more cases for other data types if needed
  }
  // Parse parameter 2 if non-null
  if (config.param2.type != 'N') {

    switch (config.param2.type) {
      case 'U':
        {
          uint32_t value = 0;
          // Read the value from the message data
          byte2Type(value, &message.data[4]);
          // Store the value in the data vector
          data.push_back((value >> 24) & 0xFF);
          data.push_back((value >> 16) & 0xFF);
          data.push_back((value >> 8) & 0xFF);
          data.push_back(value & 0xFF);
          break;
        }
      case 'I':
        {
          int32_t value = 0;
          // Read the value from the message data
          byte2Type(value, &message.data[4]);
          // Store the value in the data vector
          data.push_back((value >> 24) & 0xFF);
          data.push_back((value >> 16) & 0xFF);
          data.push_back((value >> 8) & 0xFF);
          data.push_back(value & 0xFF);
          break;
        }
      case 'F':
        {
          float value = 0;
          // Read the value from the message data
          byte2Type(value, &message.data[4]);
          // Store the value in the data vector
          data.push_back(reinterpret_cast<uint8_t*>(&value)[0]);
          data.push_back(reinterpret_cast<uint8_t*>(&value)[1]);
          data.push_back(reinterpret_cast<uint8_t*>(&value)[2]);
          data.push_back(reinterpret_cast<uint8_t*>(&value)[3]);
          break;
        }
    }
  }
  parsedData[message.identifier] = data;
  return true;
}

const std::vector<uint8_t>& CANMessageParser::getDataForMessage(uint16_t messageId) const {
  auto it = parsedData.find(messageId);
  if (it == parsedData.end()) {
    static std::vector<uint8_t> emptyVector;
    return emptyVector;
  }
  return it->second;
}



/**
 * Template function to convert a byte array to a given type.
 *
 * This function takes a pointer to a byte array and a reference to a variable of type T.
 * It converts the byte array to the given type and assigns the result to the variable.
 *
 * The type T must be a 4-byte type.
 *
 * @param [in,out] v The variable to store the result in.
 * @param [in] b The byte array to convert.
 */
template<typename T>
void CANMessageParser::byte2Type(T& v, const byte b[4]) {
  static_assert(sizeof(T) == 4, "byte2Type: T must be 4 bytes");
  // Use a union to convert the byte array to the given type.
  union {
    T v;
    byte b[4];
  } u;
  // Copy the byte array to the union.
  u.b[0] = b[0];
  u.b[1] = b[1];
  u.b[2] = b[2];
  u.b[3] = b[3];
  // Copy the result from the union to the variable.
  v = u.v;
}
