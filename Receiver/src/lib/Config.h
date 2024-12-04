// Config.h

#ifndef CONFIG_H
#define CONFIG_H

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include <Arduino.h>

#define LAP_INCREMENT_THRESHOLD 30000 // the minimum time required to count a new lap, in ms

extern uint32_t lapNumber;
extern SemaphoreHandle_t transmitDataSemaphore;
extern SemaphoreHandle_t sdCardMutex;

// SD Card Configuration
#define FILENAME_BASE "/telemetry_session_"
#define MAX_FILE_COUNT 10000
#define EEPROM_COUNTER_ADDRESS_START 0
extern uint32_t fileCounter;

// CAN Configuration
#define TX_GPIO_NUM GPIO_NUM_18
#define RX_GPIO_NUM GPIO_NUM_8
#define MSG_ID 0x1
#define STANDBY_PIN 35
extern twai_message_t message;
extern twai_status_info_t can_status;

// Radio Configuration
#define RF69_FREQ 433.0
#define MY_ADDRESS 2
#define DEST_ADDRESS 1
#define RFM69_CS 47
#define RFM69_INT 3
#define RFM69_RST 48
#define RFM69_IRQN digitalPinToInterrupt(RFM69_INT)

// SD Card Pins
#define CLK_PIN 7
#define CMD_PIN 15
#define D0_PIN 6
#define D1_PIN 5
#define D2_PIN 17
#define D3_PIN 16

#define BUTTON_PIN 14
#define PRINT_FLAG_EEPROM_ADDRESS 50

struct Parameter {
  String name;
  String unit;
  char type;  // 'U' for unsigned int, 'I' for signed int, 'F' for float
  uint8_t bits;
  uint64_t lastReadingTime;
  String lastValue;  // Store the last value as a string
  bool newValueReceived;
};

struct CANMessageConfig {
  String name;
  uint16_t id;
  Parameter param1;
  Parameter param2;
  bool isGPSTimeMessage;
  bool isRadioMessage;
};

#endif  // CONFIG_H