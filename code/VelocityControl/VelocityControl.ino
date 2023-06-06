/* Code to control the velocity of both motors using a PI controller  
*/

// Include the current library
#include "MecatroUtils.h"


// Include the AS5600 library (for the encoders) and Sparkfun I2C Mux (for muliplexer)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Header for I2C communication
#include "Wire.h"

QWIICMUX muliplexer;
AS5600 rightEncoder, leftEncoder;

// Define the control loop period, in ms.
#define CONTROL_LOOP_PERIOD 5

// Create a PID object
class PID{
  public:
    PID(float const& Kp, float const& Ki, float const& antiWindup) :
      Kp_(Kp),
      Ki_(Ki),
      antiWindup_(antiWindup),
    { 
      // Empty on purpose
    }
    
    // Compute next PID command based on current state and target
    float compute(float position, float velocity, float targetPosition, float targetVelocity, float dt)
    {
      // The integral is simply the position, while the velocity is handled by the proportional term..
      float integral = (position - targetPosition);
      if (integral > antiWindup_)
        integral = antiWindup_;
      if (integral < -antiWindup_)
        integral = -antiWindup_;
      
      return - Kp_ * (velocity - targetVelocity) - Ki_ * integral;
    }

  private:
    float Kp_;
    float Kd_;
    float Ki_;
    float antiWindup_;
};

PID rightPID(0.02, 0.15, 1.0);

PID leftPID(0.02, 0.15, 1.0);

// Target signal: sinusoids
float const A = 1.0;  // rad/s
float const f = 1.0;  // Hz
float const w = 2 * PI * f;

void mecatro::controlLoop()
{
  // Right motor
  float const targetPosition = A * (cos(w * time) - 1);
  float const targetVelocity = -A * w * sin(w * time);

  muliplexer.setPort(0);
  float position = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
  float velocity = rightEncoder.getAngularSpeed(AS5600_MODE_RADIANS);

  // Empyrically determined feedforard.
  float const feedforward =  0.165 / 2 / 3.14159 * targetVelocity;
  float const rightCommand = rightPID.compute(position, velocity, targetPosition, targetVelocity) + feedforward;

  // Log
  mecatro::log("rightPosition", position);
  mecatro::log("rightVelocity", velocity);
  mecatro::log("rightPositionTarget",  targetPosition);
  mecatro::log("rightVelocityTarget",  targetVelocity);
  mecatro::log("rightCommand",  rightCommand);

  
  muliplexer.setPort(2);
  position = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
  velocity = leftEncoder.getAngularSpeed(AS5600_MODE_RADIANS);

  float const leftCommand = leftPID.compute(position, velocity, targetPosition, targetVelocity);

  // Log
  //mecatro::log("leftPosition", position);
  //mecatro::log("leftVelocity", velocity);
  //mecatro::log("leftPositionTarget",  targetPosition);
  //mecatro::log("leftVelocityTarget",  targetVelocity);

  // Make the right motor spin
  mecatro::setMotorDutyCycle(-rightCommand, leftCommand);
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
    muliplexer.setPort(0);
    rightEncoder.begin();
    if (!rightEncoder.isConnected())
    {
      Serial.println("Error: could not connect to right encoder. Check wiring.");
      isInit = false;
    }
    // Set muliplexer to use port 3 to talk to left encoder.
    muliplexer.setPort(2);
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
