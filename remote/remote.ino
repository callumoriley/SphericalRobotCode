#include <RH_ASK.h>
#include <SPI.h>

#define MESSAGE_LEN 6
#define MSG_SEND_DELAY 50

RH_ASK driver;

void setup() {
  driver.init();
  Serial.begin(9600); // comment out when done testing
}

void loop() {
  uint8_t data[MESSAGE_LEN] = {45, (analogRead(0)/4), (analogRead(1)/4), 46, 0, 0}; // sends a 1 and a 2 at the beginning and the end to check the messsage
  driver.send(data, MESSAGE_LEN); // send data
  Serial.println(data[1]);
  Serial.println(data[2]);
  Serial.println("------");
  delay(MSG_SEND_DELAY);
}

// http://cdn.sparkfun.com/datasheets/Wireless/General/TWS-BS-3_433.92MHz_ASK_RF_Transmitter_Module_Data_Sheet.pdf
