#include <SimpleFOC.h>
#include <math.h>

// Magnetic sensor instance - SPI
MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 3, 11, 7);

// Inline current sensor setup
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, A0, A1, A2);

// Define neutral position, amplitude scalars, and derivative gain
const float neutral_position = 1.1; // in radians
const float max_voltage = 10; // Maximum voltage to apply
const float scalar_inside = 400; // Scalar for positions within [0.8, 1.4]
const float scalar_outside = 25; // Scalar for positions outside [0.8, 1.4]
const float derivative_gain = 20; // Gain for the derivative term

// Variable to store the previous position
float previous_position = 0;

void setup() {
  // Initialization as before
  sensor.init();
  motor.linkSensor(&sensor);

  current_sense.linkDriver(&driver);
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    while (1); // Halt if initialization fails
  }

  motor.linkCurrentSense(&current_sense);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 5;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  //Serial.println("Motor ready. Adjusting to maintain neutral position.");
}

void loop() {
  motor.loopFOC();

  PhaseCurrent_s current = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

  float position = motor.shaft_angle; // Current position of the motor

  // Choose appropriate amplitude scalar based on position
  float amplitude_scalar;
  if (position >= 0.8 && position <= 1.4) {
    amplitude_scalar = scalar_inside; // Use scalar when position is within range
  } else {
    amplitude_scalar = scalar_outside; // Use scalar when position is outside range
  }

  // Calculate cubic output using the current position
  float cubic_output = (position * position * position) 
                      - (3.3 * position * position) 
                      + (3.54 * position) 
                      - 1.232;

  // Derivative calculation: rate of change of position
  float derivative = (position - previous_position); // Calculate derivative

  // Calculate target voltage using the selected amplitude scalar
  float target_voltage = amplitude_scalar * cubic_output - (derivative_gain * derivative);

  // Constrain the target voltage to a defined range
  target_voltage = constrain(target_voltage, -max_voltage, max_voltage);

  // Apply the calculated target voltage to move the motor
  motor.move(target_voltage);

  // Update the previous position for the next iteration
  previous_position = position;

  // Serial output for the motor position and current
  Serial.print(target_voltage);
  Serial.print(",");
  Serial.print(position);
  Serial.print(",");
  Serial.println(current_magnitude);
}
