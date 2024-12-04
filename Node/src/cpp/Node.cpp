#include "lib/Node.h"

Node::Node() {}

/**
 * Initializes the Node with the specified CAN bus pins and mode.
 *
 * @param CAN_RX_PIN The GPIO pin number for the CAN bus receive pin. Defaults to GPIO_NUM_NC.
 * @param CAN_TX_PIN The GPIO pin number for the CAN bus transmit pin. Defaults to GPIO_NUM_NC.
 * @param CAN_STDBY_PIN The GPIO pin number for the CAN bus standby pin. Defaults to 0.
 * @param listenMode A boolean indicating whether the Node should operate in listen mode on the CAN bus. Defaults to false.
 *
 * @return None.
 *
 * @throws None.
 */
void Node::begin(gpio_num_t CAN_RX_PIN, gpio_num_t CAN_TX_PIN, uint8_t CAN_STDBY_PIN, bool listenMode)
{
  canRxPin = CAN_RX_PIN;
  canTxPin = CAN_TX_PIN;
  canStdbyPin = CAN_STDBY_PIN;
  initializeCANBus(canRxPin, canTxPin, canStdbyPin, listenMode);
}

/**
 * Initializes the CAN bus with the specified pins and mode.
 *
 * @param canRxPin The GPIO pin number for the CAN bus receive pin.
 * @param canTxPin The GPIO pin number for the CAN bus transmit pin.
 * @param canStdbyPin The GPIO pin number for the CAN bus standby pin.
 * @param listenMode A boolean indicating whether the Node should operate in listen mode on the CAN bus.
 *
 * @throws None.
 */
void Node::initializeCANBus(gpio_num_t canRxPin, gpio_num_t canTxPin, uint8_t canStdbyPin, bool listenMode)
{
  pinMode(canStdbyPin, OUTPUT);
  digitalWrite(canStdbyPin, LOW);                                                                                                          // Activate transceiver chip
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(canTxPin, canRxPin, listenMode ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL); // Initialize configuration structures using macro initializers
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  { // Install TWAI driver
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }
  if (twai_start() == ESP_OK)
  { // Start TWAI driver
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }

  // Reconfigure alerts to detect Error Passive and Bus-Off error states
  uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS; // | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    printf("Alerts reconfigured\n");
  }
  else
  {
    printf("Failed to reconfigure alerts");
  }

  printf("CAN_RX pin: %d  CAN_TX pin: %d  CAN_STDBY pin: %d\n", canRxPin, canTxPin, canStdbyPin);
}

/**
 * Initializes a message container with the specified ID and data length. Only done once per message ID that is used (i.e. to be sent or received).
 *
 * @param id The ID of the message to be initialized.
 * @param dataLength The expected length of the data in the message.
 *
 * @throws None.
 */
void Node::initializeMessage(uint32_t id, uint8_t dataLength)
{
  twai_message_t message;
  message.identifier = id;
  message.data_length_code = dataLength;
  messages[id] = message;
  printf("message ID %d initialised\n", message.identifier);
}

/**
 * Deletes a message from the `messages` container. Used if the message is no longer needed.
 *
 * @param id The ID of the message to be deleted.
 *
 * @throws None.
 */
void Node::deleteMessage(uint32_t id)
{
  messages.erase(id);
  printf("message ID %d deleted\n", id);
  for (const auto &pair : messages)
  {
    const twai_message_t &message = pair.second;
    printf("Existing Message ID: %d", message.identifier);
  }
}

/**
 * Converts a value of type T to a byte array.
 *
 * @tparam T The type of the value to be converted.
 * @param value The value to be converted.
 * @param buffer The byte array to store the converted value.
 *
 * @throws None.
 */
template <typename T>
void Node::convertToBytes(T value, byte *buffer)
{
  static_assert(sizeof(T) <= 4, "Type size exceeds maximum allowed size");
  memcpy(buffer, &value, sizeof(value));
}

/**
 * Updates the data of a message with the given ID in the Node's message map.
 *
 * @tparam T1 The type of the first variable to be stored in the message data.
 * @tparam T2 The type of the second variable to be stored in the message data.
 *            If T2 is std::nullptr_t, only the first variable will be stored.
 * @param id The ID of the message to update.
 * @param var1 The first variable to store in the message data.
 * @param var2 The second variable to store in the message data, or nullptr.
 *
 * @throws None.
 */
template <typename T1, typename T2>
void Node::updateMessageData(uint32_t id, T1 var1, T2 var2)
{
  auto it = messages.find(id);
  if (it != messages.end())
  {
    twai_message_t &message = it->second;
    // Clear the existing data
    memset(message.data, 0, sizeof(message.data));
    // Convert and store var1
    union
    {
      T1 var;
      byte bytes[sizeof(T1)];
    } u1;
    u1.var = var1;
    memcpy(message.data, u1.bytes, sizeof(T1));
    // Convert and store var2 if it's not nullptr
    if (!std::is_same<T2, std::nullptr_t>::value)
    {
      union
      {
        T2 var;
        byte bytes[sizeof(T2)];
      } u2;
      u2.var = var2;
      memcpy(message.data + sizeof(T1), u2.bytes, sizeof(T2));
      // Update the data length code
      message.data_length_code = sizeof(T1) + sizeof(T2);
    }
    else
    {
      // Update the data length code for var1 only
      message.data_length_code = sizeof(T1);
    }
  }
  else
  {
    printf("Message with ID %d not found\n", id);
  }
}

// Explicit template instantiation for all combinations of uint32_t, int32_t, and float (not very pretty, but it works)
template void Node::updateMessageData<uint32_t, uint32_t>(uint32_t, uint32_t, uint32_t);
template void Node::updateMessageData<uint32_t, int32_t>(uint32_t, uint32_t, int32_t);
template void Node::updateMessageData<uint32_t, float>(uint32_t, uint32_t, float);
template void Node::updateMessageData<uint32_t, std::nullptr_t>(uint32_t, uint32_t, std::nullptr_t);

template void Node::updateMessageData<int32_t, uint32_t>(uint32_t, int32_t, uint32_t);
template void Node::updateMessageData<int32_t, int32_t>(uint32_t, int32_t, int32_t);
template void Node::updateMessageData<int32_t, float>(uint32_t, int32_t, float);
template void Node::updateMessageData<int32_t, std::nullptr_t>(uint32_t, int32_t, std::nullptr_t);

template void Node::updateMessageData<float, uint32_t>(uint32_t, float, uint32_t);
template void Node::updateMessageData<float, int32_t>(uint32_t, float, int32_t);
template void Node::updateMessageData<float, float>(uint32_t, float, float);
template void Node::updateMessageData<float, std::nullptr_t>(uint32_t, float, std::nullptr_t);

/**
 * Transmits all messages in the Node's message map to the CAN bus. If `lowPowerMode` is true,
 * the function enters light sleep for a duration set by the user after all messages
 * have been transmitted. Otherwise, it delays for the specified `sleepDurationMs`.
 *
 * @param lowPowerMode A boolean indicating whether to enter light sleep after
 *                     transmitting all messages.
 * @param sleepDurationMs The duration to delay in milliseconds if `lowPowerMode`
 *                        is false.
 *
 * @throws None.
 */
void Node::transmitAllMessages(bool lowPowerMode, uint32_t sleepDurationMs)
{
  twai_status_info_t can_status;
  uint32_t sleepTime;
  uint32_t alerts_triggered;
  if (sleepDurationMs < 100)
  {
    sleepTime = 100;
  }
  else
  {
    sleepTime = sleepDurationMs;
  }
  for (const auto &pair : messages)
  {
    const twai_message_t &message = pair.second;
    if (twai_transmit(&message, pdMS_TO_TICKS(sleepTime)) == ESP_OK)
    {
      printf("MSG ID %d sent\n", message.identifier);
    }
    else
    {
      printf("FAIL MSG ID %d\n", message.identifier);
    }
    twai_get_status_info(&can_status);
    printf("tx error count = %d\n", can_status.tx_error_counter);
    delay(2); // THIS MIGHT BE WHAT IS NEEDED TO GIVE EACH MESSAGE SOME TIME ON THE BUS!!!!
  }

  twai_read_alerts(&alerts_triggered, portMAX_DELAY);
  if (lowPowerMode && alerts_triggered == TWAI_ALERT_TX_SUCCESS)
  {
    printf("sleepeing");
    esp_sleep_enable_timer_wakeup(sleepTime * 1000); // Enter light sleep for a duration set by the user
    esp_light_sleep_start();
  }
  else
  {
    delay(sleepTime);
  }
}

/**
 * Transmits a message with the specified ID and waits for the specified number of ticks.
 *
 * @param id The ID of the message to transmit.
 * @param ticks_to_wait The number of ticks to wait for the transmission to complete.
 *
 * @throws None.
 */
void Node::transmitMessage(uint32_t id, TickType_t ticks_to_wait)
{
  twai_status_info_t can_status;
  const twai_message_t &message = messages[id];
  if (twai_transmit(&message, ticks_to_wait) == ESP_OK)
  {
    printf("MSG ID %d sent\n", message.identifier);
  }
  else
  {
    printf("FAIL MSG ID %d\n", message.identifier);
  }
  twai_get_status_info(&can_status);
  printf("tx error count = %d\n", can_status.tx_error_counter);
}

/**
 * Displays the data of the latest message in the Node's message map.
 *
 * This function iterates through the messages in the Node's message map to find
 * the message with the highest ID, which is assumed to be the latest message.
 * It then prints the ID and data of the latest message.
 *
 * @return None.
 *
 * @throws None.
 */
void Node::displayLatestMessageData()
{
  if (messages.empty())
  {
    printf("No messages have been initialized.\n");
    return;
  }

  // Assuming the latest message is the one with the highest ID
  // This might need adjustment based on how messages are updated
  uint32_t latestId = 0;
  for (const auto &pair : messages)
  {
    if (pair.first > latestId)
    {
      latestId = pair.first;
    }
  }

  const twai_message_t &latestMessage = messages[latestId];
  printf("Latest message ID: %d   Data: ", latestId);
  for (uint8_t i = 0; i < latestMessage.data_length_code; i++)
  {
    printf("%2X ", latestMessage.data[i]);
  }
  printf("\n");
}

/**
 * @brief Adds a single expected message with only one variable.
 *
 * This function creates an ExpectedMessage object with the given ID and variable
 * type, and adds it to the expectedMessages map.
 *
 * @param id The ID of the expected message.
 * @param var1Type The type of the variable in the message payload.
 *
 * @throws None.
 */
void Node::addExpectedMessage(uint32_t id, VariableType var1Type)
{
  // Create an ExpectedMessage object with the given ID and variable type
  ExpectedMessage message = {id, var1Type, NONE}; // Use NONE to indicate "no second variable"
  // Add the ExpectedMessage object to the expectedMessages map
  expectedMessages[id] = message;
}

/**
 * Adds an expected message with two variables to the expectedMessages map.
 *
 * @param id The ID of the expected message.
 * @param var1Type The type of the first variable in the message payload.
 * @param var2Type The type of the second variable in the message payload.
 *
 * @throws None.
 */
void Node::addExpectedMessage(uint32_t id, VariableType var1Type, VariableType var2Type)
{
  ExpectedMessage message = {id, var1Type, var2Type};
  expectedMessages[id] = message;
}

/**
 * Adds a collection of expected messages to the internal map.
 *
 * @param messages A vector of ExpectedMessage objects to be added.
 *
 * @throws None.
 */
void Node::addExpectedMessages(const std::vector<ExpectedMessage> &messages)
{
  for (const auto &message : messages)
  {
    expectedMessages[message.id] = message;
  }
}

/**
 * Parses the received CAN message and returns the message ID and string representations of the
 * first and second variables in the message payload.
 *
 * @return A pair containing the message ID and a pair of strings representing the first and second
 *         variables in the message payload. If no message is received or not found, returns a pair
 *         with message ID 0 and empty strings for the variables.
 */
std::pair<uint32_t, std::pair<String, String>> Node::parseReceivedMessage()
{
  // Receive the CAN message
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK)
  {
    // Find the expected message with the received message ID
    auto it = expectedMessages.find(message.identifier);
    if (it != expectedMessages.end())
    {
      ExpectedMessage expected = it->second;
      String var1Str, var2Str;
      // Temporary buffers to hold the float values
      float floatValue1 = 0.0f, floatValue2 = 0.0f;
      // Buffer to hold the formatted string
      char formattedValue[10]; // Adjust the size as needed

      // Parse the first variable in the message payload
      switch (expected.var1Type)
      {
      case INT32:
        var1Str = String(*reinterpret_cast<int32_t *>(message.data));
        break;
      case UINT32:
        var1Str = String(*reinterpret_cast<uint32_t *>(message.data));
        break;
      case FLOAT:
        floatValue1 = *reinterpret_cast<float *>(message.data);
        break;
      case NONE:
        // Handle NONE case, if necessary
        break;
      default:
        // Handle unexpected cases
        break;
      }

      // Parse the second variable in the message payload
      switch (expected.var2Type)
      {
      case INT32:
        var2Str = String(*reinterpret_cast<int32_t *>(message.data + sizeof(int32_t)));
        break;
      case UINT32:
        var2Str = String(*reinterpret_cast<uint32_t *>(message.data + sizeof(uint32_t)));
        break;
      case FLOAT:
        floatValue2 = *reinterpret_cast<float *>(message.data + sizeof(float));
        break;
      case NONE:
        // Handle NONE case, if necessary
        break;
      default:
        // Handle unexpected cases
        break;
      }

      // Convert float values to strings with 2 decimal places
      if (expected.var1Type == FLOAT)
      {
        dtostrf(floatValue1, 4, 2, formattedValue); // Format to 2 decimal places
        var1Str = String(formattedValue);
      }
      if (expected.var2Type == FLOAT)
      {
        dtostrf(floatValue2, 4, 2, formattedValue); // Format to 2 decimal places
        var2Str = String(formattedValue);
      }

      // Return the message ID and string representations of the variables
      return {message.identifier, {var1Str, var2Str}};
    }
  }

  // Return an empty pair if no message is received or not found
  return {0, {"", ""}};
}
