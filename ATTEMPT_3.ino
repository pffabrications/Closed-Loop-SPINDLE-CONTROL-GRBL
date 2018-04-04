/*
  PID spindle speed controller
  Closed-loop RPM control for spindles/motors that can be controlled in their speed by a PWM signal.
  Tries to hold the spindle's RPM constant, even when external loads are applied/released.
  
  This code is using the Arduino PID library by Brett Beauregard,
  you can find it at http://playground.arduino.cc/Code/PIDLibrary
  
  In order for the PID to know the actual RPM of your spindle, you need a pickup on the spindle's shaft. 
  This can be for axample an optical pickup, hall sensors, etc..
  The pickup's signal goes to digital Pin 3. The desired speed is set with a potentiometer 
  wired between +5V and GND with its wiper connected to analog Pin A0. The PWM signal that goes to
  the spindle's motor controller is output on Pin 9.
  Variables Kp, Ki, Kd can be changed for tuning the PID control to your setup. The maximum possible
  RPM of your spindle is set with variable maxrpm.
*/  


#include <PID_v1.h>

const int pulserev = 1;            // the pickup's number of impulses per revolution
const int maxrpm = 12000;          // Max.RPM of the spindle
const int stablerpm = 50;         // Min. RPM for your setup to run stable (Set to 50rpm to turn this feature off)
int rpm;                           // measured RPM value
long time = 0;                     // time between two interrupts
long timeOld = 0;                  // absolute time of last interrupt
float frequency;                   // frequency of the interrupts in Hz

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

double Kp = 0.02, Ki = 0.05, Kd = 0.0005;

double Max = 241.0; //max allowed PWM value tuned to suit my speed control which doesnt accept 100% pwm duty
double Min = 2.0; //min allowed PWM value

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//=====================================================================================================================================================================================
// PWM INPUT INTERRUPT
//===================================================================================================================================================================================== 
#define GRBL_PWM 3 
int CNC_RPM;
int PWMINPUT;

// Work around bug in Arduino 1.0.6
#define NOT_AN_INTERRUPT (-1)

uint16_t pwm_value = 0;

void change() {
  static unsigned long prev_time = 0;

  if (digitalRead(GRBL_PWM))
    prev_time = micros();
  else
    pwm_value = micros() - prev_time;
}


//==================================================================================================================================================
void setup()
//==================================================================================================================================================
{
  // Turn serial comunication on for monitoring purposes
  Serial.begin(115200);

   pinMode(GRBL_PWM, INPUT_PULLUP); // SET PINMODE for GRBL PWM INPUT
  attachInterrupt(digitalPinToInterrupt(GRBL_PWM), change, CHANGE); // GBBL PWM INTERRUPT
  
  // Sample Time of 5 ms
  myPID.SetSampleTime(5);

  // Interrupt on Pin 3 (Interrupt 0 on Nano pin D2)
  attachInterrupt(0, measurerpm, RISING);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}
//==================================================================================================================================================
void loop()
//==================================================================================================================================================

{
  //=====================================================================================================================
  //        SERIAL PRINT DIAGNOSTICS
  //=====================================================================================================================

Serial.print("Target RPM: ");
    Serial.print(Setpoint);
    Serial.print("   Spindle RPM: ");
    Serial.print(Input);
    Serial.print("   PWM: ");
    Serial.println(Output);
  
  //=====================================================================================================================
  // GRBL PWM Interrupt stuff
  //=====================================================================================================================
  uint16_t pwmin;

  noInterrupts();
  pwmin = pwm_value;
  interrupts();  

PWMINPUT = pwmin/10; // Calculating RPM from GRBL PWM INPUT DUTY CYCLE
CNC_RPM = PWMINPUT*(maxrpm/100);

//========================================================================================================================
// Calculating the PID RPM SETPOINT
//========================================================================================================================
//  Setpoint = map(analogRead(A0), 0, 1023, 0, maxrpm);   // Reading the pot for setting the target RPM

Setpoint = CNC_RPM;
//========================================================================================================================

  rpm = 60 * (frequency / pulserev);                    // Calculating the RPM of the spindle

  //======================================================================================================================
  //        PID LOOP
  //======================================================================================================================
  
  Input = rpm;                                          // Input variable for PID control

  myPID.SetTunings(Kp, Ki, Kd);

  myPID.Compute();

  if(Setpoint <= stablerpm){                            // No PWM output on Pin 9 if the desired RPM isn't running stable
    analogWrite(9, 0);
  }
  
  else{
  analogWrite(9, Output);                               // PWM Output to motorcontroller on Pin 9
  } 
}


//==================================================================================================================================================
void measurerpm()                                       // Interrupt for measuring the SPINDLE RPM IR SENSOR frequency
//==================================================================================================================================================
{
  time = micros() - timeOld;
  timeOld = micros();
  frequency = time;
  frequency = 1000000 / frequency;
}
