

//INCLUDES
#include <PID_v1.h> //pid-library
#include "BluetoothSerial.h"  //Bluetoothe library, for serial communication to phone
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//PIN DEFINES
const int dirpin1 = 32;
const int dirpin2 = 33;
const int enpin1 = 14;
const int enpin2 = 26;
const int senspin = 27;
const int calpin = 19;
const int cal_led_pin = 18;
const int tbpot1 = 36;
const int tbpot2 = 37;
const int accpot1 = 36;  //38 original.. 36 for testing
const int accpot2 = 39;

//PWM Config
const int freq = 20000;
const int res = 16;
const int pwmchannel = 0;

//VARIABLES
int TPS0MIN = 170;
int TPS0NULL = 200;
int TPS0MAX = 957;
int val0 = 0;
int TPS1MIN = 845;
int TPS1NULL = 750;
int TPS1MAX = 60;
int val1 = 0;
int Pedal0MIN = 44;
int Pedal0MAX = 468;
int val2 = 0;
int Pedal1MIN = 90;
int Pedal1MAX = 938;
int val3 = 0;
int potmin = 100;
int potmax = 3000;
int potval = 0;
int start_calibration_done = 0;

BluetoothSerial SerialBT;     //Set bluetooth function_name

//PID Setup
double Setpoint, Input, Output;
double Kp=1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {

  Serial.begin(115200);
  Serial.println("Up and running");
  pinMode(tbpot1, INPUT);
  pinMode(tbpot2, INPUT);
  pinMode(accpot1, INPUT);
  pinMode(accpot2, INPUT);
  pinMode(dirpin1, OUTPUT);
  pinMode(dirpin2, OUTPUT);
  pinMode(enpin1, OUTPUT);
  pinMode(enpin2, OUTPUT);
  pinMode(senspin, INPUT);
  pinMode(calpin, INPUT);
  pinMode(cal_led_pin, OUTPUT);

  //PID-controller setup
  Input = analogRead(accpot1);
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  
  // PWM setup
  ledcSetup(pwmchannel, freq, res);
  ledcAttachPin(enpin2, pwmchannel);
  
}

void loop() {
  
  digitalWrite(enpin1, LOW);
  digitalWrite(dirpin1, LOW);
  digitalWrite(dirpin2, HIGH);

  while (start_calibration_done == 0) {
    initial_cal();
  }

  while (digitalRead(calpin) == HIGH) {
    calibrate_pedal();
  }

  Input = analogRead(accpot1);
  Input = map(Input, Pedal0MIN, Pedal0MAX, 0, 65535);
/*int duty = 0;
  duty = analogRead(potpin);
  duty = map(duty, potmin, potmax, 0, 65535);
  duty = constrain(duty, 0, 65535);*/
  myPID.Compute();
  ledcWrite(pwmchannel, Output);
  Serial.println(Output);
  delay(15);

  while (Output <= Setpoint){
     Serial.println("Going up");
     delay(15);
  }

  while (Output >= Setpoint) {
    Serial.println("Going down");
    delay(15);
  }
}

void initial_cal() {

  digitalWrite(enpin2, LOW);
  digitalWrite(dirpin1, LOW);
  digitalWrite(dirpin2, HIGH);
  
  val0 = analogRead(tbpot1);
  val1 = analogRead(tbpot2);

  TPS0NULL = val0;
  TPS1NULL = val1;
  Serial.println("New standby pot values set");
  delay(200);

  ledcWrite(pwmchannel, 65535);
  delay(1000);

  TPS0MAX = val0;
  TPS1MAX = val1;
  Serial.println("New max throttlebody pot value set");
  delay(500);
  ledcWrite(pwmchannel, 0);
  delay(500);

  digitalWrite(dirpin2, LOW);
  digitalWrite(dirpin1, HIGH);
  ledcWrite(pwmchannel, 65535);
  delay(500);

  TPS0MIN = val0;
  TPS1MIN = val1;
  Serial.println("New min throttlebody pot value set");
  delay(500);

  start_calibration_done++;
  String p1=";";
  Serial.println(TPS0NULL + p1 + TPS1NULL + p1 + TPS0MIN + p1 + TPS1MIN + p1 + TPS0MAX + p1 + TPS1MAX);
  
}

void calibrate_pedal() {

  digitalWrite(cal_led_pin, HIGH);
  Serial.println("Ready to calibrate pedal");
//  SerialBT.println("Ready to calibrare pedal");

  val2 = analogRead(accpot1);
  val3 = analogRead(accpot2);

  if(val2 > Pedal0MAX){
    Pedal0MAX = val2;
    Serial.println("New pedal max calibration set");
  }

  if(val3 > Pedal1MAX){
    Pedal1MAX = val3;
    Serial.println("New pedal max calibration set");
  }
  
  if(val2 < Pedal0MIN){
    Pedal0MIN = val2;
    Serial.println("New pedal min calibration set");
  }
  if(val3 < Pedal1MIN){
    Pedal1MIN = val3;
    Serial.println("New pedal min calibration set");
  }
  
}

void pot_mapping(){

  
  /*
  int TPS0TRAVEL = TPS0MAX-TPS0MIN;
  int TPS1TRAVEL = TPS1MIN-TPS1MAX;
  int TPS0 = constrain(analogRead(tbpot1), TPS0MIN, TPS0MAX);  //Range: 220 - 38 ORIGINAL:663 - 54
  int TPS1 = constrain(analogRead(tbpot2), TPS1MIN, TPS1MAX);  //Range: 79 - 220 ORIGINAL: 164 - 970
  int diff0 = (TPS0-TPS0MIN)+map((TPS1-TPS1MAX), 0, TPS0TRAVEL, 0, TPS1TRAVEL); //ORIGINAL: 0,806,0,609

  int Pedal0 = constrain(analogRead(accpot1), Pedal0MIN, Pedal0MAX);  //Range: 39 - 423 ORIGINAL:0 - 180
  int Pedal1 = constrain(analogRead(accpot2), Pedal1MIN, Pedal1MAX);  //Range: 83 - 850 ORIGINAL:0 - 380
  int diff1 = (Pedal0-Pedal0MIN) - map(Pedal1, 0, Pedal1MAX, 0, Pedal0MAX);
  //int TPSIN = analogRead(potpin);
  */
}
