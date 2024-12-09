#define currentNode
// #define voltage

#include "lib/Node.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

#ifdef currentNode

#define ID 21
#define ID2 22

// For ESP32-C3 or ESP32-S3, the pins will be auto-assigned based on the MCU type.
// Input pin types are GPIO_NUM_X, e.g. GPIO_NUM_1 for pin 1.

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
Node node;

const int NUM_MEASUREMENTS = 10;  // Number of measurements to average
int calibrationValue;             // Variable to store the calibration value
bool isCalibrated = false;        // Flag to indicate if calibration is done

void setup() {
  Serial.begin(115200);
  delay(1000);  // Keep this delay to not cause issues with serial printing.
  Serial.println("Online");
  node.begin(GPIO_NUM_4, GPIO_NUM_3, 2);
  node.initializeMessage(ID, 4);
  // node.initializeMessage(ID2, 4);

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  ads.setGain(GAIN_FOUR);  // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV (USE THIS GAIN IN DIFFERENTIAL MODE)
  // ads.setGain(GAIN_EIGHT);  // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //ads.setGain(GAIN_SIXTEEN);  // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  delay(500);
  ads.setDataRate(RATE_ADS1115_8SPS);

  // Calibration process
  int sum = 0;
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    sum += ads.readADC_Differential_2_3();  // Read ADC value
    delay(130);                             // Delay to ensure stability between measurements
  }
  calibrationValue = sum / NUM_MEASUREMENTS;  // Calculate average
  isCalibrated = true;
  printf("Sensor calibrated to %d as 0.00A\n", calibrationValue);
}

void loop() {
  //uint16_t adc2 = ads.readADC_SingleEnded(2);
  int32_t adc23 = ads.readADC_Differential_2_3();

  // Map the current ADC value using the average of the first NUM_MEASUREMENTS measurements
  float current;
  if (isCalibrated) {
    current = map(adc23, calibrationValue, 15430, 0, 3000) / 1E2;
    printf("%d, %f A\n", adc23, current);
    
  } else {
    // If measurements array is not filled yet, use a default value for mapping
    current = map(adc23, 5, 3866, 0, 3000) / 1E2;
  }
  printf("%d, %f A\n", adc23, current);
  node.updateMessageData(ID, current);
  // node.updateMessageData(ID2, current);
  node.transmitAllMessages();
  //float current = map(adc23, 5, 6472, 0, 5000) / 1E2;
  //printf("0-1: %d  2-3: %d\n", results01, results23);

  delay(500);
}

#elif voltage

#define ID 16
#define ID2 17

/**
 * Pin Configuration for XIAO ESP32-C3 and XIAO ESP32-S3
 * 
 * This section defines the GPIO pins used for TX, RX, and controlling
 * the sleep mode of the CAN transceiver chip based on the specific 
 * board variant being used. Ensure that you select the correct pin 
 * configuration for your hardware setup.
 * 
 * TX_GPIO_NUM: Transmit Pin
 * - XIAO ESP32-C3: GPIO_NUM_3
 * - XIAO ESP32-S3 (e.g. Screen Node): GPIO_NUM_2
 * 
 * RX_GPIO_NUM: Receive Pin
 * - XIAO ESP32-C3: GPIO_NUM_4
 * - XIAO ESP32-S3 (e.g. Screen Node): GPIO_NUM_3
 * 
 * sleepPin: Sleep Control Pin for the CAN Transceiver
 * - XIAO ESP32-C3: GPIO_NUM_2
 * - XIAO ESP32-S3: GPIO_NUM_1
 * 
 * Note:
 * - Make sure to define the correct board type before setting these pins.
 * - These configurations are critical for proper communication over the CAN bus.
 */

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

Node node;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Keep this delay to not cause issues with serial printing.
  node.begin(GPIO_NUM_4, GPIO_NUM_3, 2);
  node.initializeMessage(ID, 4);  // Voltage meter 1
  node.initializeMessage(ID2, 4);  // Voltage meter 1
  // node.initializeMessage(12, 8); // Voltage meter 2

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_TWO);  // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  //ads.setGain(GAIN_FOUR);  // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  delay(500);
  ads.setDataRate(RATE_ADS1115_8SPS);
}

void loop() {

  int32_t results1 = ads.readADC_Differential_0_1();
  int32_t results2 = ads.readADC_Differential_2_3();
  float mapped1 = map(results1, 0, 16593, 0, 3240) / 1E2;
  float mapped2 = map(results2, 0, 16535, 0, 3240) / 1E2;
  printf("First: %fV\n", mapped1);
  printf("Second: %fV\n", mapped2);
  node.updateMessageData(ID, mapped1);
  node.updateMessageData(ID2, mapped2);
  node.transmitAllMessages(false, 500);
}

#else

#define ID 18
#define ID2 19

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// For ESP32-C3 or ESP32-S3, the pins will be auto-assigned based on the MCU type.
// Input pin types are GPIO_NUM_X, e.g. GPIO_NUM_1 for pin 1.

SFE_UBLOX_GNSS myGNSS;

Node node;

int32_t latitude;
int32_t longitude;
uint32_t unixTime;
uint32_t unixTimeMicro;
int32_t velocity;

int64_t start_time;
int64_t end_time;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Keep this delay to not cause issues with serial printing.
  Wire.begin();
  delay(50);

  while (myGNSS.begin() == false) {
    printf("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
    delay(1);
    //esp_restart();  // restart node because it gets stuck because connection with gps fails on power on
  }
  myGNSS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5);  // Produce ten solutions per second

  //myGNSS.setAutoPVT(true);  // Breaks down with the GPS we have, do not turn ON!
  unixTime = myGNSS.getUnixEpoch(unixTimeMicro);
  printf("unix: %d us: %d\n", unixTime, unixTimeMicro);

  node.begin(GPIO_NUM_4, GPIO_NUM_3, 2);
  node.initializeMessage(1, 8);
  node.initializeMessage(ID, 8);
  node.initializeMessage(ID2, 4);

  node.updateMessageData(1, unixTime, unixTimeMicro);
  node.transmitMessage(1, pdMS_TO_TICKS(3000));
  node.deleteMessage(1);
  // Add multiple expected messages
}

void loop() {
  latitude = myGNSS.getLatitude();
  longitude = myGNSS.getLongitude();
  velocity = myGNSS.getGroundSpeed();
  node.updateMessageData(ID, latitude, longitude);
  node.updateMessageData(ID2, velocity);
  node.transmitAllMessages(false, 200);
  //delay(2000);
}

#endif