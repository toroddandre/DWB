/*
  Throttle.ino

** 15.05.21 - Removed delay in sketch, adjusted analog pin.**
** 17.05.32 - Adding use of both bridge on pcb.

Hardware Connections:
-TPS 0:                   Pin A6 (Blue Wire)
-TPS 1:                   Pin A7 (Thin White Wire)
-Throttle Input 0:        Pin A2 (White Wire)
-Throttle Input 1:        Pin A3 (White Marked Wire)
-Error LED                Pin A5 (Blue Wire)
-L298N H-Bridge Enable A: Pin 3  (N/A)
-L298N H-Bridge Input 1:  Pin 12  (Other Speaker Wire)
-L298N H-Bridge Input 2:  Pin 11 (Gray Line Wire)
*/


//PID Library
#include <PID_v1.h>
#include "BluetoothSerial.h"  //BT Function
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Pins assignments
const int LEDPin = 9;
const int TPSinput = 34;
const int pinI1 = 32;
const int pinI2 = 33;
const int speedPin1 = 26;
const int enPin1 = 14;
const int sensPin = 27;
const int tbpot1 = 36;
const int tbpot2 = 37;
const int accpot1 = 38;
const int accpot2 = 39;


// Function Variables
int TPS0MIN = 170;
int TPS0MAX = 957;
int TPS1MIN = 845;
int TPS1MAX = 60;
int Pedal0MIN = 44;
int Pedal1MIN = 90;
int Pedal0MAX = 468;
int Pedal1MAX = 938;
int currentmillis = millis();
int freshstart = 0;
int serialStart = 0;
int interval = 2000;
int errorset = 0;
int calfail = 0;
const int freq = 10000;
const int resolution = 16;
const int pwmchannel = 0;
const int MAX_DUTY_CYCLE = (int)(pow(2, resolution) - 1);

BluetoothSerial SerialBT;



//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);

void setup()
{
  //Set PWM frequency to 31.37255 kHz
  //TCCR2B = TCCR2B & B11111000 | B00000001;  //Timer 2 for pair D3/D11
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;    //Timer 1 for pair D9/D10

  ledcSetup(pwmchannel, freq, resolution);
  ledcAttachPin(speedPin1, pwmchannel);
  
  Input = 0;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedPin1,OUTPUT);
  pinMode(LEDPin,OUTPUT);
  pinMode(enPin1,OUTPUT);
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,LOW);
  digitalWrite(enPin1,LOW);
  Serial.begin(115200); 
  SerialBT.begin("ESP32test");
  Serial.println("BT ready to pair");
}

void loop()
{
  //Uncomment to tune the PID loop
  
/*  float Kp = mapfloat(analogRead(4), 0, 1023, 0, 5);
  Serial.print(Kp, DEC);
  Serial.print("  ");
  float Ki = mapfloat(analogRead(5), 0, 1023, 0, 5);
  Serial.print(Ki, DEC);
  Serial.print("  ");
  myPID.SetTunings(Kp, Ki, 0.00);
*/  
 
  digitalWrite(LEDPin, LOW);

  //Self calibrate throttlebody
  if (freshstart == 0) {
    delay(2000);
    ledcWrite(pwmchannel, -MAX_DUTY_CYCLE);
    TPS0MIN = analogRead(tbpot1);
    TPS1MIN = analogRead(tbpot2);
    Serial.println("Calibrating minimum position");
    SerialBT.println("Calibrating minimum position");
    delay(1000);
    ledcWrite(pwmchannel, MAX_DUTY_CYCLE);
    delay(1000);
    TPS0MAX = analogRead(tbpot1);
    TPS1MAX = analogRead(tbpot2);    
    Serial.println("Calibrating maximum position");
    SerialBT.println("Calibrating maximum position");
    freshstart = 1;
    delay(1000);
    if (TPS0MAX == TPS0MIN) {
      Serial.println("Calibration failed!");
      SerialBT.println("Calibratione failed!");
      ledcWrite(pwmchannel, 0);
      digitalWrite(LEDPin, HIGH);
      calfail = 1;
      if (freshstart == 1 && calfail == 1) {
        Serial.println("Sweeping");
        SerialBT.println("Sweeping");
        digitalWrite(pinI1, LOW);
        digitalWrite(pinI2, HIGH);
        for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){
          ledcWrite(speedPin1, dutyCycle);
          delay(15);
        }
        for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
          ledcWrite(speedPin1, dutyCycle);
          delay(15);
        }
        delay(1000);
        digitalWrite(pinI2, LOW);
        digitalWrite(pinI1, HIGH);
        for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){
          ledcWrite(speedPin1, dutyCycle);
          delay(15);
        }
        for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
          ledcWrite(speedPin1, dutyCycle);
          delay(15);
        }
    
      }
    }
  }


  //PID Loop tunings
  myPID.SetTunings(0.15, 2.00, 0.00); //ORIGINAL: 0.15, 2.00, 0.00
  int TPS0TRAVEL = TPS0MAX-TPS0MIN;
  int TPS1TRAVEL = TPS1MIN-TPS1MAX;
  int TPS0 = constrain(analogRead(tbpot1), TPS0MIN, TPS0MAX);  //Range: 220 - 38 ORIGINAL:663 - 54
  int TPS1 = constrain(analogRead(tbpot2), TPS1MIN, TPS1MAX);  //Range: 79 - 220 ORIGINAL: 164 - 970
  int diff0 = (TPS0-TPS0MIN)+map((TPS1-TPS1MAX), 0, TPS0TRAVEL, 0, TPS1TRAVEL); //ORIGINAL: 0,806,0,609
  while(analogRead(tbpot1) < 40)
  {
    if (errorset = 0){
    ledcWrite(pwmchannel,0);
    digitalWrite(LEDPin, HIGH);
    analogRead(tbpot1);
    Serial.println("ERROR 1 - Throttlesensor 0 out of range - Too low");
    SerialBT.println("ERROR 1 - Throttlesensor 0 out of range - Too low");
    errorset = 1;
    }
    
  }
  while(analogRead(tbpot2) > 1550)
  {
    if (errorset = 0){
    ledcWrite(pwmchannel,0);
    digitalWrite(LEDPin, HIGH);
    analogRead(tbpot2);
    Serial.println("ERROR 2 - Throttlesensor 1 out of range - Too high");
    SerialBT.println("ERROR 2 - Throttlesensor 1 out of range - Too high");
    errorset = 1;
    }
  }
  
 
  //PID Input from TPS
  Input = map(constrain((TPS0 - TPS0MIN), 0, TPS0TRAVEL), 0, TPS0TRAVEL, 0, Pedal1MAX);
 
  int Pedal0 = constrain(analogRead(accpot1), Pedal0MIN, Pedal0MAX);  //Range: 39 - 423 ORIGINAL:0 - 180
  int Pedal1 = constrain(analogRead(accpot2), Pedal1MIN, Pedal1MAX);  //Range: 83 - 850 ORIGINAL:0 - 380
  int diff1 = (Pedal0-Pedal0MIN) - map(Pedal1, 0, Pedal1MAX, 0, Pedal0MAX);
  int TPSIN = analogRead(1);
  while(analogRead(accpot1) > 500 || analogRead(accpot1) < 15)  //added analogRead under known value
  {
    if (errorset = 0){
    ledcWrite(pwmchannel,0);
    digitalWrite(LEDPin, HIGH);
    analogRead(2);
    Serial.println("ERROR 3 - Pedalsensor 0 out of range");
    SerialBT.println("ERROR 3 - Pedalsensor 0 out of range");
    errorset = 1;
    }
  }
  while(analogRead(accpot2) > 1000 || analogRead(accpot2) < 40)  //Added analogRead under known value
  {
    if (errorset = 0){
    ledcWrite(pwmchannel,0);
    digitalWrite(LEDPin, HIGH);
    analogRead(3);
    Serial.println("ERROR 4 - Pedalsensor 1 out of range");
    SerialBT.println("ERROR 4 - Pedalsensor 1 out of range");
    errorset = 1;
    }
  }
  while(diff1 > 70)
  {
    if (errorset = 0){
    ledcWrite(pwmchannel,0);
    digitalWrite(LEDPin, HIGH);
    int Pedal0 = analogRead(accpot1);  //Range: 39-423 ORIG:0 - 180
    int Pedal1 = analogRead(accpot2);  //Range: 83-850 ORIG:0 - 380
    int diff1 = (Pedal0-Pedal0MIN) - map(Pedal1, 0, Pedal1MAX, 0, Pedal0MAX);
    Serial.println("ERROR 5 - Abnormal difference between pedal sensors");
    SerialBT.println("ERROR 5 - Abnormal difference between pedal sensors");
    errorset = 1;
    }
  }

  //PID Setpoint from Throttle Pedal
  Setpoint = (Pedal1 + TPSIN);
  
  //Set throttle to 0
  if(Setpoint <= 5)
  {
   ledcWrite(pwmchannel,0);
  }
  else
  {
    myPID.Compute();
    ledcWrite(pwmchannel,Output);
  }

      Serial.print(diff0, DEC);
      Serial.print("  ");
      Serial.print(diff1, DEC);
      Serial.print("  ");
      Serial.print(TPS0, DEC);
      Serial.print("  ");
      Serial.print(TPS1, DEC);
      Serial.print("  ");
      Serial.print(Pedal0, DEC);
      Serial.print("  ");
      Serial.print(Pedal1, DEC);
      Serial.print("  ");
      Serial.print(TPSIN, DEC);
      Serial.print("  ");
      Serial.print(Input, DEC);
      Serial.print("  ");
      Serial.print(Setpoint, DEC);
      Serial.print("  ");
      Serial.println(Output, DEC);  
      SerialBT.print(diff0, DEC);
      SerialBT.print("  ");
      SerialBT.print(diff1, DEC);
      SerialBT.print("  ");
      SerialBT.print(TPS0, DEC);
      SerialBT.print("  ");
      SerialBT.print(TPS1, DEC);
      SerialBT.print("  ");
      SerialBT.print(Pedal0, DEC);
      SerialBT.print("  ");
      SerialBT.print(Pedal1, DEC);
      SerialBT.print("  ");
      SerialBT.print(TPSIN, DEC);
      SerialBT.print("  ");
      SerialBT.print(Input, DEC);
      SerialBT.print("  ");
      SerialBT.print(Setpoint, DEC);
      SerialBT.print("  ");
      SerialBT.println(Output, DEC); 
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
