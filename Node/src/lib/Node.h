#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <unordered_map>
#include <vector>

/**
 * @enum VariableType
 * @brief Enumerates the possible types of variables that can be included in a message.
 *
 * This enumeration defines the possible types of variables that can be included in a CAN message.
 * Each variable type has a corresponding size in bytes.
 *
 * - INT32: 32-bit signed integer.
 * - UINT32: 32-bit unsigned integer.
 * - FLOAT: Floating-point number.
 * - NONE: No variable (used when there is only one variable).
 */
enum VariableType
{
  INT32,  //!< 32-bit signed integer
  UINT32, //!< 32-bit unsigned integer
  FLOAT,  //!< Floating-point number
  NONE    //!< No variable (used when there is only one variable)
};

/**
 * @struct ExpectedMessage
 * @brief Structure representing an expected message with its identifier and variable types.
 *
 * @var ExpectedMessage::id
 *      The identifier of the expected message.
 * @var ExpectedMessage::var1Type
 *      The type of the first variable in the message payload.
 * @var ExpectedMessage::var2Type
 *      The type of the second variable in the message payload.
 *
 * @note This structure is used to define the expected messages received by the node.
 *       It's recommended to use `enum VariableType` for the variable types to ensure
 *       the correct interpretation of the message payload.
 */
struct ExpectedMessage
{
  uint32_t id;           ///< The identifier of the expected message.
  VariableType var1Type; ///< The type of the first variable in the message payload.
  VariableType var2Type; ///< The type of the second variable in the message payload.
};

class Node
{
public:
  Node();
  void begin(gpio_num_t canRxPin = GPIO_NUM_NC, gpio_num_t canTxPin = GPIO_NUM_NC, uint8_t canStdbyPin = 0, bool listenMode = false);
  void initializeMessage(uint32_t id, uint8_t dataLength);
  void deleteMessage(uint32_t id);
  template <typename T1, typename T2 = std::nullptr_t>
  void updateMessageData(uint32_t id, T1 var1, T2 var2 = nullptr);
  void transmitAllMessages(bool lowPowerMode = false, uint32_t sleepDurationMs = 1000);
  void transmitMessage(uint32_t id, TickType_t ticks_to_wait = pdMS_TO_TICKS(1000));

  void addExpectedMessage(uint32_t id, VariableType var1Type);
  void addExpectedMessage(uint32_t id, VariableType var1Type, VariableType var2Type);
  void addExpectedMessages(const std::vector<ExpectedMessage> &messages);

  std::pair<uint32_t, std::pair<String, String>> parseReceivedMessage();
  void displayLatestMessageData();

private:
  gpio_num_t canRxPin;
  gpio_num_t canTxPin;
  uint8_t canStdbyPin;
  std::unordered_map<uint32_t, twai_message_t> messages;
  std::unordered_map<uint32_t, ExpectedMessage> expectedMessages;
  template <typename T>
  void convertToBytes(T value, byte *buffer);
  void initializeCANBus(gpio_num_t canRxPin, gpio_num_t canTxPin, uint8_t canStdbyPin, bool listenMode);
};

#endif // NODE_H