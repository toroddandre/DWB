#include <PID_v1.h>
#include <BluetoothSerial.h>

/*** CONSTANTS ***************************************************************/
// Using enum instead of separate "const int"s to ensure the same pin cannot
// get multiple names:
enum Pins_t
{
  LED_PIN = 9,
  TPS_INPUT = 34, // NB! In v2 this definition was not used (used 1 instead)
  PIN_I1 = 32,
  PIN_I2 = 33,
  SPEED_PIN1 = 26,
  EN_PIN1 = 14,
  TBPOT1 = 38,
  TBPOT2 = 39,
  ACCPOT1 = 36,
  ACCPOT2 = 37,
};

enum BridgeDirection_t
{
  OPEN,
  CLOSE,
};

const int PWM_FREQ_HZ = 10000;
const int PWM_BIT_RESOLUTION = 16;
const int PWM_CHANNEL = 0;
const uint32_t PWM_MAX_VALUE = (1U << PWM_BIT_RESOLUTION) - 1;

const int NUM_CALIBRATION_STEPS = 256;
const int THROTTLEBODY_SETTLE_DELAY_MS = 15;

const int MIN_ALLOWED_TBPOT_RANGE = 500;
const int MAX_ALLOWED_THROTTLE_DEVIATION = 0.10; // 10%
const int MAX_ALLOWED_PEDAL_DEVIATION = 0.10; // 10%

const int PID_SAMPLE_TIME_MS = 100;
const int REPORT_INTERVAL_MS = 1000;

/*** TYPES *******************************************************************/
struct range_t
{
  int Min;
  int Max;
};

/*** GLOBALS *****************************************************************/
int Error = 0;

struct
{
  range_t Tbpot[2];
  range_t Speedpin;
  range_t Accpot[2];
  range_t Tpsin;
} Calibration;

struct
{
  int Tbpot[2];
  int Accpot[2];
  int Tpsin;
} Samples;

struct
{
  double Setpoint;
  double Input;
  double Output;
} Regulator;

BluetoothSerial SerialBT;
PID Pid(&Regulator.Input, &Regulator.Output, &Regulator.Setpoint,
        0.15, 2.00, 0.00, DIRECT);

/******************************************************************************
 * Print a message on serial and bluetooth.
 */
template <typename... Ts>
void message(const char* Fmt, Ts&&... Args)
{
  static char Buffer[256];

  snprintf(Buffer, sizeof(Buffer), Fmt, Args...);
  Serial.println(Buffer);
  SerialBT.println(Buffer);
}

/******************************************************************************
 * Report an error.
 */
template <typename... Ts>
void error(int Code, const char* Fmt, Ts&&... Args)
{
  if (Error == 0) {
    Error = Code;
  }
  digitalWrite(LED_PIN, HIGH);

  message(Fmt, Args...);
}

/******************************************************************************
 * Set H-bridge direction.
 * 
 * NB! May need to swap settings here! Did not have documentation.
 */
void setBridgeDirection(BridgeDirection_t Direction)
{
  switch (Direction)
  {
    case OPEN:
      digitalWrite(PIN_I2, LOW);
      digitalWrite(PIN_I1, HIGH);
      break;

    case CLOSE:
      digitalWrite(PIN_I1, LOW);
      digitalWrite(PIN_I2, HIGH);
      break;
  }
}

/******************************************************************************
 * Calibrate throttle potmeters, and required PWM strength.
 * 
 * Finds the maximum range of observed values for TBPOT1, TBPOT2 and
 * SPEED_PIN1.
 */
void calibrateThrottle()
{
  int LastValues[2] = {-1, -1};
  
  setBridgeDirection(OPEN);
  for (int i = 0; i < NUM_CALIBRATION_STEPS; ++i)
  {
    uint32_t PwmVal = static_cast<uint32_t>(i*PWM_MAX_VALUE/NUM_CALIBRATION_STEPS);
    
    ledcWrite(PWM_CHANNEL, PwmVal);
    delay(THROTTLEBODY_SETTLE_DELAY_MS);
    Calibration.Tbpot[0].Min = analogRead(TBPOT1);
    Calibration.Tbpot[1].Min = analogRead(TBPOT2);

    if (   (Calibration.Tbpot[0].Min != LastValues[0])
        || (Calibration.Tbpot[1].Min != LastValues[1]))
    {
      Calibration.Speedpin.Min = -PwmVal;
      LastValues[0] = Calibration.Tbpot[0].Min;
      LastValues[1] = Calibration.Tbpot[1].Min;
    }
    else
    {
      break;
    }
  }

  setBridgeDirection(CLOSE);
  for (int i = 0; i < NUM_CALIBRATION_STEPS; ++i)
  {
    uint32_t PwmVal = static_cast<uint32_t>(i*PWM_MAX_VALUE/NUM_CALIBRATION_STEPS);
    
    ledcWrite(PWM_CHANNEL, PwmVal);
    delay(THROTTLEBODY_SETTLE_DELAY_MS);
    Calibration.Tbpot[0].Max = analogRead(TBPOT1);
    Calibration.Tbpot[1].Max = analogRead(TBPOT2);

    if (   (Calibration.Tbpot[0].Max != LastValues[0])
        || (Calibration.Tbpot[1].Max != LastValues[1]))
    {
      Calibration.Speedpin.Max = PwmVal;
      LastValues[0] = Calibration.Tbpot[0].Max;
      LastValues[1] = Calibration.Tbpot[1].Max;
    }
    else
    {
      break;
    }
  }

  // Set back to neutral position:
  ledcWrite(PWM_CHANNEL, 0);
  setBridgeDirection(OPEN);

  for (int i = 0; i < 2; ++i)
  {
    if (Calibration.Tbpot[i].Max - Calibration.Tbpot[i].Min
        < MIN_ALLOWED_TBPOT_RANGE)
    {
      error(1, "ERROR - Throttle sensor %d malfunction [Min: %d, Max: %d]",
            i, Calibration.Tbpot[i].Min, Calibration.Tbpot[i].Max); 
    }
  }
}

/******************************************************************************
 * Set up bridge and PWM to control valve in throttle body. 
 * 
 * Position is expected to be in the range [0.0, 1.0].
 * Assumes calibration values are correct.
 */
void setThrottle(double Position)
{
  int Pwm = Position*(Calibration.Speedpin.Max-Calibration.Speedpin.Min)
    + Calibration.Speedpin.Min;

  if (Pwm >= 0)
  {
    setBridgeDirection(OPEN);
    ledcWrite(PWM_CHANNEL, static_cast<uint32_t>(Pwm));
  }
  else
  {
    setBridgeDirection(CLOSE);
    ledcWrite(PWM_CHANNEL, static_cast<uint32_t>(-Pwm));
  }
}

/******************************************************************************
 * Read current throttle position.
 * 
 * Verify consistency and return normalized position in the range [0.0, 1.0].
 * Assumes calibration values are correct.
 */
double getThrottle()
{
  double Throttle[2];
  
  Samples.Tbpot[0] = analogRead(TBPOT1);
  Samples.Tbpot[1] = analogRead(TBPOT2);

  for (int i = 0; i < 2; ++i)
  {
    Throttle[i] = static_cast<double>((Samples.Tbpot[i] - Calibration.Tbpot[i].Min))
      / (Calibration.Tbpot[i].Max - Calibration.Tbpot[i].Min);
  }

  if (abs(Throttle[0] - Throttle[1]) > MAX_ALLOWED_THROTTLE_DEVIATION)
  {
    error(2, "ERROR - Abnormal difference between throttle sensors "
          "[%d->%.3f, %d->%.3f]",
          Samples.Tbpot[0], Throttle[0], Samples.Tbpot[1], Throttle[1]);
  }

  return (Throttle[0] + Throttle[1])/2.0;
}

/******************************************************************************
 * Read current pedal position.
 * 
 * Verify consistency and return normalized position in the range [0.0, 1.0].
 */
double getPedal()
{
  double Pedal[2];
  
  Samples.Accpot[0] = analogRead(ACCPOT1);
  Samples.Accpot[1] = analogRead(ACCPOT2);
  
  for (int i = 0; i < 2; ++i)
  {
    Pedal[i] = static_cast<double>((Samples.Accpot[i] - Calibration.Accpot[i].Min))
      / (Calibration.Accpot[i].Max - Calibration.Accpot[i].Min);
  }

  if (abs(Pedal[0] - Pedal[1]) > MAX_ALLOWED_PEDAL_DEVIATION)
  {
    error(2, "ERROR - Abnormal difference between pedal sensors "
          "[%d->%.3f, %d->%.3f]",
          Samples.Accpot[0], Pedal[0], Samples.Accpot[1], Pedal[1]);
  }

  return (Pedal[0] + Pedal[1])/2.0;
}

/******************************************************************************
 * Read current external input.
 * 
 * Return normalized in the range [0.0, 1.0].
 */
double getExternalInput()
{  
  Samples.Tpsin = analogRead(TPS_INPUT);
  
  return static_cast<double>((Samples.Tpsin - Calibration.Tpsin.Min))
      / (Calibration.Tpsin.Max - Calibration.Tpsin.Min);
}

/******************************************************************************
 * Report system status.
 */
void reportStatus()
{
  static unsigned long lastreport = 0;
  unsigned long now = millis();

  if (now - lastreport >= REPORT_INTERVAL_MS)
  {
    lastreport = now;
    message("TB: %4d/%4d, ACC: %4d/%4d, TPSIN: %4d "
            "Setpt: %6.3f, In: %6.3f, Out: %6.3f.",
            Samples.Tbpot[0], Samples.Tbpot[1],
            Samples.Accpot[0], Samples.Accpot[1],
            Samples.Tpsin,
            Regulator.Setpoint, Regulator.Input, Regulator.Output);
  }
}

/******************************************************************************/
void setup()
{
  pinMode(PIN_I1, OUTPUT);
  pinMode(PIN_I2, OUTPUT);
  pinMode(SPEED_PIN1, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(EN_PIN1, OUTPUT);

  digitalWrite(PIN_I2, LOW);
  digitalWrite(PIN_I1, LOW);
  digitalWrite(EN_PIN1, LOW);
  digitalWrite(LED_PIN, LOW);

  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BIT_RESOLUTION);
  ledcAttachPin(SPEED_PIN1, PWM_CHANNEL);

  Error = 0;

  Regulator.Setpoint = 0.0;
  Regulator.Input = 0.0;
  Regulator.Output = 0.0;
  Pid.SetSampleTime(PID_SAMPLE_TIME_MS);
  Pid.SetOutputLimits(0.0, 1.0);
  Pid.SetMode(AUTOMATIC);
  
  Serial.begin(115200);
  SerialBT.begin("ESP32test");
  Serial.println("Bluetooth ready.");

  Calibration.Tbpot[0].Min = 0;
  Calibration.Tbpot[0].Max = 1023;
  Calibration.Tbpot[1].Min = 0;
  Calibration.Tbpot[1].Max = 1023;
  Calibration.Accpot[0].Min = 44;
  Calibration.Accpot[0].Max = 468;
  Calibration.Accpot[1].Min = 90;
  Calibration.Accpot[1].Max = 938;
  Calibration.Tpsin.Min = 0;
  Calibration.Tpsin.Max = 1023;
  calibrateThrottle();

  message("Calibration: TB1 %4d/%4d, TB2 %4d/%4d, ACC1 %4d/%4d, ACC2 %4d/%4d,"
          "TPSIN %4d/%4d.",
          Calibration.Tbpot[0].Min, Calibration.Tbpot[0].Max,
          Calibration.Tbpot[1].Min, Calibration.Tbpot[1].Max,
          Calibration.Accpot[0].Min, Calibration.Accpot[0].Max,
          Calibration.Accpot[1].Min, Calibration.Accpot[1].Max,
          Calibration.Tpsin.Min, Calibration.Tpsin.Max);
}

/******************************************************************************/
void loop()
{
  Regulator.Setpoint = getPedal() + getExternalInput();
  Regulator.Input = getThrottle();
  Pid.Compute();
  setThrottle(Regulator.Output);
  
  reportStatus();
}
