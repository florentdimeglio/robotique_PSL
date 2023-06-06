// MOtor PWM pin direction
#define DIRECTION_MOTOR_RIGHT 12
#define DIRECTION_MOTOR_LEFT 8

#include <Arduino.h>
#include "MecatroUtils.h"

bool wasTelemetryTimeSent;
unsigned long tickTime;

// Forward interrupt call to user function.
bool shouldRun = false;
unsigned int nIter = 0;
unsigned int targetIter = 0;

ISR(TIMER1_COMPA_vect)
{
  nIter ++;
  if (nIter == targetIter)
  {
    shouldRun = true;
    nIter = 0;
  }
}


namespace mecatro{
    void configureArduino(unsigned char const controlPeriodMs)
    {
        // Stop interrupts while configuring registers.
        cli();

        // Motor signal generation: we need configure the Arduino to generate a high-frequency signal
        // (typically 5-20kHz).
        // This is done through harware PWM, described in Chapter 13 of the documentation of the ATMega328p
        // microcontroller.

        // We use the 16-bit TIMER1 to generate the signal. This timer is mapped to pin 9 and 10 (only specific
        // pins can perform hardware PWM).
        // First, we set these pins as output.
        pinMode(9, OUTPUT);
        pinMode(10, OUTPUT);

        // Now we must configure the signal to output on OC1A. The working principle is as follow: internally, TIMER1 has a 16-bit counter (i.e. it counts upward from 0 to 65535)
        // To generate a signal, a specific register OCR1A (output compare register for port 1A) gives a treshold value. We then set the output to toogle on the threshold:
        // this means that, when the counter is below the threshold, OC1A is set to 1 (high, i.e. 5V), and when it is above, the output becomes 0.
        // Thus, by changing the value of OCR1A, we can change the duty cycle, i.e. the amount of time that the signal spends high.
        // Now we need a way to change the frequency. This can be done through two mechanisms:
        // - first, changing how high the counter goes. Instead of counting up to 65535, we can decide to loop before: this is determined by the ICR1 register.
        // - second, we can choose at what speed the timer goes up. This timer is directly connected to the 16Mhz oscillator on the Arduino board. However, we can choose a
        // multiplicative coefficient, called a prescaler, to devide this frequency by a certain power of two. This is done using the TCCR1 register.
        // Basically ICR1 allows for small, almost continuous change in the frequency, whereas the prescaler is useful for brutal variations. Note that a low value of
        // ICR1 means less resolution (because OCR1A has to be smaller than ICR1), so it's important to choose the right prescaler to have maximum range.
        // Finally, OC1A and OC1B can have different duty cycle, through the OCR1A and OCR1B registers, but they share the same frequency (which is suitable for most cases,
        // motor control for instance).
        // As a final detail, note that TIMER1 is the only one where you can set the frequency continuously: for the other timers, you can only use the prescaler (i.e. there is no ICR0).
        // This is not really a problem for motor control, as you don't need to vary the frequency: using a prescaler of 2 for instance gives a signal at 31kHz, perfect for motor control

        // Let's put all this in practice.
        // The clock frequency is 16Mhz. Counting up to 65535 gives a (16000000 / 65536) = 244Hz frequency. So we don't need a specific prescaler in this case, and we just set it to 1.
        // We first configure OC1A in the correct mode (fast PWM, with ICR1 as TOP value): see page 108 onward.
        TCCR1A = 0b10100010;
        TCCR1B = 0b00011001;

        // Now we need to set ICR1A for a 10kHz signal.
        // The frequency is given by CLK / ICR1A, so we have ICR1A = CLK/10kHz = 1600
        // WARNING: this frequency is also used for the control loop (see below) and MUST NOT BE CHANGED
        ICR1 = 1600;

        // Make sure the motors start of
        OCR1A = 0;
        OCR1B = 0;

        // Now we want to generate a periodic call to a motor control function, every n ms.
        // Timer0 and Timer2 are 8-bit timers and thus won't be very accurate: they require a prescaler
        // of 1024 to be in the correct range, resulting in a 62us precision only.
        // Instead, we use TIMER1 as well: we rely on the fact that ICR1 is set to have a 10kHz period,
        // thus one every 10 calls 1ms has elasped.
        // The drawback of this approach is to create a coupling between both features: the motor signal
        // PWM frequency must not be changed, else the control loop frequency will change as well.

        // All we need to do is enable the interrupt, and store the user-required period.
        TIMSK1 = (1 << OCIE1A); 
        targetIter = 10 * controlPeriodMs;

        // For reference, this is the code to make it work off TIMER2, removing this coupling.
        // Motor control loop frequency: we want a control function to be called at regular intervals.
        // For that purpose, we will use TIMER2 in Clear Timer on Compare Match Mode
        // The objective is to enable the user to specify a control period between 1ms and 5ms.
        // Since the timer is an 8-bit counter, a prescaler of 1024 must be used to enable a 5ms duration.
        // TCCR2A = 0b00000010;
        // TCCR2B = 0b00000111;
        
        // TCNT2  = 0;//initialize counter value to 0

        // The counter thus updates at 16MHz / 1024 = 15625 Hz. 
        // We now set the value of OCRA to have the desired frequency: an interrupt is generate every (OCRA + 1)
        // 1ms occurs every 15.625 tick, so we have our target tick value.
        // OCR2A = round(controlPeriodMs * 15.6) - 1;

        // The correspoding periods are expressed here
        // 1ms -> 1.024ms
        // 2ms -> 1.984ms
        // 3ms -> 3.008ms
        // 4ms -> 3.968ms
        // 5ms -> 4.992ms
        // Now we actually activate the time-based interrupt.
        // TIMSK2 = (1 << OCIE2A);

        // Finially, we enable all interrupts.
        sei();
    }

    void setMotorDutyCycle(float const& rightMotorDC, float const& leftMotorDC)
    {
        // Set direction
        digitalWrite(DIRECTION_MOTOR_RIGHT, rightMotorDC > 0);
        digitalWrite(DIRECTION_MOTOR_LEFT, leftMotorDC > 0);
        // Set duty cycle: the timer counts up to ICR1.
        int counterValue = floor(abs(rightMotorDC) * ICR1);
        // Clamp to ICR1 aka duty cycle of 1
        if (counterValue > ICR1)
          counterValue = ICR1;
        OCR1B = counterValue;

        counterValue = floor(abs(leftMotorDC) * ICR1);
        // Clamp to ICR1 aka duty cycle of 1
        if (counterValue > ICR1)
          counterValue = ICR1;
        OCR1A = counterValue;
    }

    void log(const char* variableName, float const variableValue)
    {
      // To keep processing simple, telemetry is performed by sending fixed-length packets of 11 bytes:
      //  - the first byte, use a delimiter, is the character '@'in ascii (0x40)
      //  - the next 6 bytes are the variable name, in ascii ; the variable is end-padded with spaces to fit in 6 bytes exactly.
      //  - the last 4 bytes are the float32 value (or uint32 for time, see below), little endian.
      // The timestamp is sent automatically exactly once per frame ; the associated variable name (reserved) is TIMESTAMP.
      // Timestamp is the only variable which is not a float32 but a uint32, corresponding to the elapsed time since the start
      // of the log, in micro-seconds. The timestamp is thus to be applied to each subsequent point until the next timestamp.
      // 
      // Here is an example. Note that the line feed \n are not present and used only for readability here.
      // To represent numeric values sent over 4 bytes, the syntax <value> is used.
      //
      // @TIMESTAMP           <0>@sin                 <0.0>@cos                 <0.0>
      // @TIMESTAMP           <1000>@sin                 <1.0>@cos                 <0.5>
      // @TIMESTAMP           <2000>@sin                 <0.5>@cos                 <1.0>

      // These messages correspond to the following CSV table:
      // Time (s) | sin | cos
      // 0.0      | 0.0 | 0.0
      // 0.001    | 1.0 | 0.5
      // 0.002    | 0.5 | 1.0

      // Send timestamp once per loop only
      if (!wasTelemetryTimeSent)
      {
        // Send @ and variable name -> 7 bytes.
        Serial.write("@TS    ", 7);
        // Send the current time, this is a uint32, little endian.
        Serial.write((uint8_t*)(&tickTime), 4);
        wasTelemetryTimeSent = true;
      }
      // Fit the variable name in a fixed-sized array of size 6.
      unsigned char name[7] = "@      ";
      unsigned char i = 0;
      while (i < 6 && variableName[i] != '\0')
      {
        name[i + 1] = variableName[i];
        i++;
      }
      Serial.write(name, 7);
      // Send variable value, float32, little endian
      Serial.write((uint8_t*)(&variableValue), 4);
    }

    void run()
    {
      while (true)
      {
        // Wait for tick from interrupt
        while (!shouldRun) 
          __asm__("nop\n\t");   
        // Configure telemetry
        tickTime = micros();
        shouldRun = false; 
        wasTelemetryTimeSent = false;
        // Run control loop iteself
        mecatro::controlLoop();
      }
    }
};