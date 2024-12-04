#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SPI.h>
#include "Config.h"

class RadioHandler {
public:
  RadioHandler();
  bool begin(bool liveSendingMode);
  void sendData(const uint8_t* data, uint8_t len);
  void receiveData();
  void deactivateRadio();
  void activateRadio();
  bool isRadioActive() const;
  RH_RF69 radio;
  void transmitDataFromFile(const char* filename, uint32_t& lastLapSent, std::map<unsigned int, unsigned int>& lapLineTracker);
  bool liveSendingMode;
  uint32_t lastCompletionTime = 0;
private:
  static void transmitDataTask(void* param);
  RHReliableDatagram radio_reliable;
  bool radioActive;
};