// #include <Arduino.h>
// #include <SPI.h>
// #include <RH_RF69.h>
// #include <RHReliableDatagram.h>
// #include <iostream>
// #include <string.h>
// #include <map>
// #include <string>
// #include <vector>
// #include <sstream>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// /************ Radio Setup ***************/

// #define RF69_FREQ 433.0
// #define MY_ADDRESS 1
// #define DEST_ADDRESS 2

// #define RFM69_CS 20   // "B"
// #define RFM69_INT 5   // "C"
// #define RFM69_RST 21  // "A"
// #define RFM69_IRQN digitalPinToInterrupt(RFM69_INT)

// RH_RF69 rf69(RFM69_CS, RFM69_INT);
// RHReliableDatagram rf69_reliable(rf69, MY_ADDRESS);

// // Define a union to hold different types of data
// union Value {
//   int intValue;
//   float floatValue;
// };

// // Define a structure to hold variable information
// struct VariableInfo {
//   std::string type;
//   Value value;
// };

// // Map to store variables dynamically
// std::map<std::string, VariableInfo> variables;

// // Mutex for protecting the variables map
// portMUX_TYPE variablesMutex = portMUX_INITIALIZER_UNLOCKED;

// // Define a fixed-size buffer for messages
// #define MESSAGE_BUFFER_SIZE 128

// // Declare buf and messageQueue before they are used
// uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
// QueueHandle_t messageQueue = xQueueCreate(100, MESSAGE_BUFFER_SIZE);  // Adjust the size as needed

// std::vector<std::string> split(const std::string& str, char delimiter) {
//   std::vector<std::string> tokens;
//   std::string token;
//   std::istringstream tokenStream(str);
//   while (std::getline(tokenStream, token, delimiter)) {
//     tokens.push_back(token);
//   }
//   return tokens;  // Ensure there's a return statement
// }

// // Modify parseMessage to accept a char array instead of std::string
// void parseMessage(const char* message) {
//   std::vector<std::string> tokens = split(message, ',');

//   if (tokens.size() >= 5) {  // At least one complete variable definition
//                              // Process the first variable
//     std::string lapNum = tokens[0];
//     std::string time = tokens[1];
//     std::string varName1 = tokens[2];
//     std::string varValueStr1 = tokens[3];
//     std::string varUnit1 = tokens[4];

//     //uint32_t unixTime = std::stoi(time)/1E6; // Removing microsenconds from Unix time

//     VariableInfo varInfo1;
//     varInfo1.type = varUnit1;                             // Assuming the unit is the type for simplicity
//     varInfo1.value.floatValue = std::stof(varValueStr1);  // Assuming all values are floats
//     printf(">%s:%s§#\n", "Lap #",lapNum.c_str());
//     printf(">%s:%s§%s\n", varName1.c_str()/*, unixTime*/, varValueStr1.c_str(), varUnit1.c_str());
//     variables[varName1] = varInfo1;

//     if (tokens.size() == 8) {  // At least two complete variable definitions
//       // Process the second variable
//       std::string varName2 = tokens[5];
//       std::string varValueStr2 = tokens[6];
//       std::string varUnit2 = tokens[7];

//       VariableInfo varInfo2;
//       varInfo2.type = varUnit2;                             // Assuming the unit is the type for simplicity
//       varInfo2.value.floatValue = std::stof(varValueStr2);  // Assuming all values are floats
//       printf(">%s:%s§%s\n", varName2.c_str() /*, time.c_str()*/, varValueStr2.c_str(), varUnit2.c_str());
//       variables[varName2] = varInfo2;
//     }

//   } else {
//     printf("Incomplete message received\n");
//   }
//   /*
//   // Display the parsed message using printf
//   for (const auto& var : variables) {
//     printf(">%s%f\n", var.first.c_str(), var.second.value.floatValue);
//   }
//   */
// }

// void receiveTask(void* pvParameters) {
//   for (;;) {  // Infinite loop
//     if (rf69.available()) {
//       uint8_t len = sizeof(buf);
//       uint8_t from;

//       if (rf69_reliable.recvfromAck(buf, &len, &from)) {
//         //RH_RF69::printBuffer("Received: ", buf, len); // PRINTING THE BUFFER
//         printf(">RSSI:%d§dBm\n", rf69.lastRssi());
//         // Copy the received message into a fixed-size buffer
//         char message[MESSAGE_BUFFER_SIZE];
//         strncpy(message, (char*)buf, len);
//         message[len] = '\0';  // Ensure null-termination
//         // Send the message to the parsing task
//         if (xQueueSend(messageQueue, &message, portMAX_DELAY) != pdPASS) {
//           printf("failed to send to parser\n");
//         }
//       }
//     }
//     vTaskDelay(1 / portTICK_PERIOD_MS);  // Delay to allow other tasks to run
//   }
// }

// void parseTask(void* pvParameters) {
//   for (;;) {  // Infinite loop
//     char message[MESSAGE_BUFFER_SIZE];
//     if (xQueueReceive(messageQueue, &message, portMAX_DELAY)) {
//       parseMessage(message);
//     }
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   // while (!Serial) delay(1);

//   printf("ESP32C3 RFM69 RX antenna Test!\n");

//   pinMode(RFM69_RST, OUTPUT);
//   digitalWrite(RFM69_RST, LOW);

//   printf("ESP32C3 RFM69 RX antenna Test!\n");

//   digitalWrite(RFM69_RST, HIGH);
//   delay(10);
//   digitalWrite(RFM69_RST, LOW);
//   delay(10);

//   if (!rf69_reliable.init()) {
//     printf("RFM69 radio init failed\n");
//     while (1)
//       ;
//   }
//   Serial.println("RFM69 radio init OK!");

//   if (!rf69.setFrequency(RF69_FREQ)) {
//     printf("setFrequency failed\n");
//   }

//   rf69.setTxPower(20, true);

//   uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
//                     0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
//   rf69.setEncryptionKey(key);
//   rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
//   printf("RFM69 radio @%d MHz\n", (int)RF69_FREQ);

//   // Create the receiving task
//   xTaskCreate(
//     receiveTask,   /* Task function. */
//     "ReceiveTask", /* name of task. */
//     4096,          /* Stack size of task */
//     NULL,          /* parameter of the task */
//     2,             /* priority of the task */
//     NULL           /* Task handle to keep track of created task */
//   );

//   // Create the parsing task
//   xTaskCreate(
//     parseTask,   /* Task function. */
//     "ParseTask", /* name of task. */
//     4096,        /* Stack size of task */
//     NULL,        /* parameter of the task */
//     1,           /* priority of the task */
//     NULL         /* Task handle to keep track of created task */
//   );
// }

// void loop() {
//   // Empty. Everything is done in tasks.
// }