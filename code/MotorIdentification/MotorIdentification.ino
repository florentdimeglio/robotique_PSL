/* Identification of the motor response: velocity versus input duty cycle with a sawtooth input signal
*/

// Include the current library
#include "MecatroUtils.h"


// Include the AS5600 library (for the encoders) and Sparkfun I2C Mux (for muliplexer)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

QWIICMUX multiplexer;
AS5600 rightEncoder, leftEncoder;

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5


// Target signal: slow sawtooth ; period in ms
int const PERIOD = 5000;
float const QUATER_PERIOD = PERIOD / 4.0;

void mecatro::controlLoop()
{
  int time = millis();
  while (time > PERIOD)
    time -= PERIOD;
  
  double target;
  if (time < QUATER_PERIOD)
    target = time / QUATER_PERIOD;
  else if (time < 3 * QUATER_PERIOD)
    target = 1 - (time - QUATER_PERIOD) / QUATER_PERIOD;
    target = -1.;
  else
    target = -1 + (time - 3 * QUATER_PERIOD) / QUATER_PERIOD;

  // Log duty cycle, right angular speed and right angle. 6 characters max for variable names!
  mecatro::log("dutyCy",  target);
  multiplexer.setPort(4);
  mecatro::log("thetaL",  leftEncoder.rawAngle() * AS5600_RAW_TO_RADIANS);
  multiplexer.setPort(7);
  mecatro::log("thetaR",  rightEncoder.rawAngle() * AS5600_RAW_TO_RADIANS);


  // Make the right motor spin
  mecatro::setMotorDutyCycle(-target,-target);
}


void setup()
{
  // Setup serial communication with the PC - for debugging and logging. Max 1Mbps, then you lose threads.
  Serial.begin(1000000);

  // Start I2C communication  
  Wire.begin();
  // Set I2C clock speed to 400kHz (fast mode)
  Wire.setClock(400000);

  // Init muliplexer
  if (!multiplexer.begin())
  {
    Serial.println("Error: I2C muliplexer not found. Check wiring.");
  }
  else
  {
    bool isInit = true;
    // Set muliplexer to use port 0 to talk to right encoder.
    multiplexer.setPort(7);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }
    multiplexer.setPort(4);
    leftEncoder.begin();
    if (!leftEncoder.isConnected())
    {
      Serial.println("Error: could not connect to left encoder. Check wiring.");
      isInit = false;
    }
    else 
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
