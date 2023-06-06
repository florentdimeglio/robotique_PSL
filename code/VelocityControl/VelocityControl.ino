/* Test code for implementing a PI controller on the velocity measured by the encoders

  Note: this code requires the following libraries (install them through the library manager):
     - SparkFun I2C Mux Arduino Library
     - AS5600 library
*/

// Include the current library
#include "MecatroUtils.h"

// Include the AS5600 library (for the encoders) and Sparkfun I2C Mux (for muliplexer)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5

//Controller parameters
float Kp = 0.1;
float Ki = 1;

// Initialise the integrators
float IR = 0;
float IL = 0;
int time = 0;
float dt;
float uR, uL;
QWIICMUX muliplexer;
AS5600 rightEncoder, leftEncoder;

// This function is called periodically, every CONTROL_LOOP_PERIOD ms.
// Put all your code here.
void mecatro::controlLoop()
{
  dt = (millis() - time) * 1e-3;
  time = millis();
  
  // Set muliplexer to use port 0 to talk to right encoder.
  muliplexer.setPort(7);
  // Raw encoder measurement - from 0 to 360 degrees
  float wR = rightEncoder.getAngularSpeed(AS5600_MODE_RADIANS);

  // Set muliplexer to use port 3 to talk to left encoder.
   muliplexer.setPort(4);
  // Raw encoder measurement - from 0 to 360 degrees
  float wL = leftEncoder.getAngularSpeed(AS5600_MODE_RADIANS);
  
  //Reference value for angular velocities (rad/s)
  float wRref = -20;
  float wLref = -20;

  //Error variables
  float eR = (wR - wRref);
  float eL = (wL - wLref);
  //Update integrators
  IR += dt * eR;
  IL += dt * eL;
  //Control inputs
  uR =  - Kp * eR - Ki * IR;
  uL =  - Kp * eL - Ki * IL;

  // Keep the motor off, i.e. at 0 duty cycle (1 is full forward, -1 full reverse)
  mecatro::setMotorDutyCycle(-uL, -uR);

  mecatro::log("wR",wR);
  mecatro::log("wL",wL);

  // Serial.print("Right speed ");
  // Serial.println(wR);
//  Serial.print("Left speed ");
//  Serial.print(wL)
}


void setup()
{
  // Setup serial communication with the PC - for debugging and logging.
  Serial.begin(1000000);
  // Start I2C communication
  Wire.begin();
  // Set I2C clock speed to 400kHz (fast mode)
  Wire.setClock(400000);

  // Init muliplexer
  if (!muliplexer.begin())
  {
    Serial.println("Error: I2C muliplexer not found. Check wiring.");
  }
  else
  {
    bool isInit = true;
    // Set muliplexer to use port 0 to talk to right encoder.
    muliplexer.setPort(7);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }
    // Set muliplexer to use port 3 to talk to left encoder.
    muliplexer.setPort(4);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      Serial.println("Error: could not connect to left encoder. Check wiring.");
      isInit = false;
    }

    if (isInit)
    {
      // Configure motor control and feedback loop call.
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }
  }
}


void loop()
{
  // Don't forget to call this, otherwise nothing will happen !
  // This function never returns, put all your code inside mecatro::controlLoop.
  mecatro::run();
}
