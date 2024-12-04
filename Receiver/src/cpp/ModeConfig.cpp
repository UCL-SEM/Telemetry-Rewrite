// ModeConfig.cpp

#include "lib/ModeConfig.h"
#include "lib/Receiver.h"

#define DEBOUNCE_TIME 50           // Debounce time in milliseconds
#define NORMAL_PRESS_TIME 150      // Time for a normal press in milliseconds
#define DOUBLE_PRESS_TIME 500      // Time window for a double press in milliseconds
#define LONG_PRESS_TIME 3000       // Time for a long press in milliseconds
#define VERY_LONG_PRESS_TIME 6000  // Time for a very long press in milliseconds

ModeConfig* ModeConfig::instance = nullptr;

/**
 * Constructs a ModeConfig object with a reference to a Receiver object.
 *
 * @param receiver Reference to a Receiver object.
 */
ModeConfig::ModeConfig(Receiver& receiver)
  : receiver(receiver) {
  // Constructor implementation...
}

/**
 * Initializes the ModeConfig object by setting up the instance pointer, 
 * configuring the button pin, and creating a semaphore and task for button handling.
 *
 * @return None
 */
void ModeConfig::begin() {
  instance = this;  // Set the instance pointer
  pinMode(BUTTON_PIN, INPUT);
  isButtonPressed = false;
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ModeConfig::handleButtonInterrupt, CHANGE);
  buttonSemaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(buttonTask, "buttonTask", 2048, this, 1, &buttonTaskHandle, 0);
}

/**
 * Task function that runs indefinitely, waiting for the button semaphore to be taken.
 * When the semaphore is taken, it calls the handleButtonPress function to handle the button press.
 *
 * @param param A pointer to the ModeConfig object that this task belongs to.
 *
 * @return None
 */
void ModeConfig::buttonTask(void* param) {
  ModeConfig* modeConfig = static_cast<ModeConfig*>(param);
  for (;;) {
    if (xSemaphoreTake(modeConfig->buttonSemaphore, portMAX_DELAY) == pdTRUE) {
      modeConfig->handleButtonPress();
    }
  }
}

/**
 * Handles a button interrupt by giving a semaphore to wake up the button task.
 *
 * This function is called from an interrupt context and should be as short as possible.
 * It checks if the ModeConfig instance is valid and then gives the semaphore to wake up the button task.
 *
 * @return None
 */
void ModeConfig::handleButtonInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (instance != nullptr) {
    xSemaphoreGiveFromISR(instance->buttonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


/**
 * Handles a button press event, determining whether it's a single press, double press, long press, or very long press.
 *
 * This function is called when the button task is woken up by the button interrupt handler.
 * It checks the current state of the button and the time elapsed since the last press to determine the type of press.
 * Depending on the type of press, it calls the corresponding function (singlePressFunction, doublePressFunction, longPressFunction, or veryLongPressFunction).
 *
 * @return None
 */
void ModeConfig::handleButtonPress() {
  unsigned long currentTime = millis();
  bool pressed = digitalRead(BUTTON_PIN) == HIGH;

  if (pressed && !isButtonPressed) {
    // Button was just pressed
    isButtonPressed = true;
    pressStartTime = currentTime;
  } else if (!pressed && isButtonPressed) {
    // Button was just released
    isButtonPressed = false;
    unsigned long pressDuration = currentTime - pressStartTime;

    if (pressDuration < NORMAL_PRESS_TIME) {
      // This could be a single press or the first press of a double press
      if (waitingForDoublePress && ((currentTime - lastPressEndTime) < DOUBLE_PRESS_TIME)) {
        // This is the second press of a double press
        doublePressFunction();
        printf("double press\n");
        waitingForDoublePress = false;
        pressCount = 0;
      } else {
        // This could be a single press, start waiting for a possible second press
        waitingForDoublePress = true;
        lastPressEndTime = currentTime;
        pressCount++;
      }
    }
    /*
    else if (waitingForDoublePress && ((currentTime - lastPressEndTime) >= DOUBLE_PRESS_TIME)) {
      // No second press occurred, so it's a single press
      singlePressFunction();
      printf("single press\n");
      waitingForDoublePress = false;
      pressCount = 0;
    }
  */
    else if ((pressDuration >= LONG_PRESS_TIME) && (pressDuration < VERY_LONG_PRESS_TIME)) {
      // This is a long press
      longPressFunction();
      printf("long press\n");
    } else if (pressDuration >= VERY_LONG_PRESS_TIME) {
      // This is a very long press
      veryLongPressFunction();
      printf("very long press\n");
    }
  }
}


/**
 * Handles the action for a single button press event.
 *
 * @return None
 */
void ModeConfig::singlePressFunction() {
  // Implement the action for a single press
}

/**
 * Increments the lap number and prints the updated value to the console.
 *
 * @return None
 */
void ModeConfig::doublePressFunction() {
  lapNumber++;
  printf("Lap number incremented to: %d\n", lapNumber);
}

/**
 * Performs an action when a long press is detected.
 *
 * @return None
 */
void ModeConfig::longPressFunction() {
  // Implement the action for a long press
  //xSemaphoreGive(transmitDataSemaphore);
}

/**
 * Handles the action for a very long press event.
 *
 * @return None
 */
void ModeConfig::veryLongPressFunction() {
  // Implement the action for a very long press
}