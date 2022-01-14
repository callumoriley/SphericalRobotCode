#include <RH_ASK.h>
#include <SPI.h>

#define MESSAGE_LEN 4
#define MOTOR 9
#define SERVO 10
#define LEFT_STICK_OFFSET 132
#define RIGHT_STICK_OFFSET 123
#define CUTOFF_TIME 1000
#define MOTOR_PERIOD 7000

//RH_ASK driver; // pin 11 is the radio rx pin and pin 12 is the radio tx pin (unused)
RH_ASK driver(2000, 7, 8); // pin 7 is the radio rx pin and pin 8 is the radio tx pin (unused)

unsigned long lastServoRise = 0;
unsigned long lastMotorRise = 0;
unsigned long currMicros;
unsigned long lastRadioComm = 0;
//unsigned long lastLEDChange = 0;
float motorHighTime = 0; // both in milliseconds
float servoHighTime = 1.5;
boolean motorPinState = false;
boolean servoPinState = false;
//boolean ledState = false;

void setup() {
  Serial.begin(9600); // serial stuff here for debugging, remove when done
  if (!driver.init())
  {
    Serial.println("Initialization failed!");
  }
  pinMode(MOTOR, OUTPUT);
  pinMode(SERVO, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  uint8_t inBuffer[MESSAGE_LEN];
  uint8_t bufLen = sizeof(inBuffer);

  if (driver.recv(inBuffer, &bufLen))
  {
    Serial.println(inBuffer[0]);
    Serial.println(inBuffer[1]);
    Serial.println(inBuffer[2]);
    Serial.println(inBuffer[3]);
    Serial.println("------");

    if (inBuffer[0] == 45 && inBuffer[3] == 46)
    {
      //digitalWrite(MOTOR, (inBuffer[1] - LEFT_STICK_OFFSET < 0) ? 0 : (inBuffer[1] - LEFT_STICK_OFFSET));
      
      //analogWrite(LEFTMOTOR, (inBuffer[1] - LEFT_STICK_OFFSET < 0) ? 0 : 2*(inBuffer[1] - LEFT_STICK_OFFSET));
      //analogWrite(RIGHTMOTOR, (inBuffer[2] - RIGHT_STICK_OFFSET < 0) ? 0 : 2*(inBuffer[2] - RIGHT_STICK_OFFSET));
      
      // left stick is for motor control right now, right stick will be for servo control
      servoHighTime = inBuffer[2]/(float)255.0 + 1.0; // at 1.5 when the stick is neutral, 2.0 when the stick is all the way forward, and 1.0 when the stick is all the way back. Need to add a constant to make it less sensitive
      motorHighTime = ((inBuffer[1] - LEFT_STICK_OFFSET < 0) ? 0 : (inBuffer[1] - LEFT_STICK_OFFSET))/((float)255.0 - LEFT_STICK_OFFSET) * 20.0;
      lastRadioComm = millis();
      Serial.println("VALID");

      //motorHighTime = (inBuffer[1] - LEFT_STICK_OFFSET < 0) ? 0 : 20;
    }  
  }
  // software PWM timing
  currMicros = micros();
  if (currMicros - lastServoRise >= 20000) // 20000 microseconds = 20 milliseconds, period is 50 Hz
  {
    lastServoRise = currMicros;
    digitalWrite(SERVO, HIGH);
    servoPinState = true;
  }
  if (currMicros - lastMotorRise >= MOTOR_PERIOD)
  {
    lastMotorRise = currMicros;
    digitalWrite(MOTOR, HIGH);
    motorPinState = true;
  }
  if (currMicros - lastServoRise >= servoHighTime * 1000 && servoPinState == true) // the reason these are not else ifs is that I want them all to be checked every time the code runs through them
  {
    digitalWrite(SERVO, LOW);
    servoPinState = false;
  }
  if (currMicros - lastServoRise >= motorHighTime * 1000 && motorPinState == true)
  {
    digitalWrite(MOTOR, LOW);
    motorPinState = false;
  }
  /*
  if (millis() - lastLEDChange >= 1000) // this is for blinking an LED on boards which have a built in LED
  {
    lastLEDChange = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState); 
  }
  */
  if (millis() - lastRadioComm > CUTOFF_TIME) // stop the robot if it goes out of radio range, which will happen a lot with these terrible RF links
  {
    motorHighTime = 0;
    //servoHighTime = 1.5;
    digitalWrite(MOTOR, LOW); // remove at some point
  }
}
