/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/
#include "BluetoothSerial.h"   //added BT function

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//#define LEDC_CHANNEL_0    0
//#define LEDC_TIMER_13_BIT  13
//#define LEDC_BASE_FREQ    5000


BluetoothSerial SerialBT;

const int tbpot1 = 38;
const int tbpot2 = 39;
const int accpot1 = 36;
const int accpot2 = 37;
const int dirpin1 = 32;
const int dirpin2 = 33;
const int enpin1 = 14;
const int enpin2 = 26;
const int senspin = 27;
const int freq = 10000;
const int res = 13;
const int pwmchannel = 0;

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
int brightness  = 0;
int fadeAmount  = 5;
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255){
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);  // initialize serial communication at 115200
  SerialBT.begin("ESP32test"); //BT name
  Serial.println("The device is started, now you can pair with BT");
  pinMode(dirpin1, OUTPUT);
  pinMode(dirpin2, OUTPUT);
  pinMode(enpin1, OUTPUT);
  pinMode(enpin2, OUTPUT);
  ledcSetup(pwmchannel, freq, res);
  ledcAttachPin(enpin2, pwmchannel);
}

// the loop routine runs over and over again forever:
void loop() {

  int dir = 0;
  
  digitalWrite(enpin1, LOW);
  digitalWrite(dirpin1, LOW);
  digitalWrite(dirpin2, HIGH);
  
  ledcAnalogWrite(pwmchannel, brightness);
  brightness = brightness + fadeAmount;
  Serial.println(brightness);

  if (brightness <= 0 || dir == 1){
    digitalWrite(dirpin2, LOW);
    digitalWrite(dirpin1, HIGH);
    fadeAmount = -fadeAmount;
    dir = 0;
  }
  delay(50);
  if (brightness >= 255) {
    digitalWrite(dirpin1, LOW);
    digitalWrite(dirpin2, HIGH);
    fadeAmount = -fadeAmount;
    dir = 1;

  }
}
