#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 3, 11, 7);

// Inline current sensor setup
// Parameters: shunt resistor value, gain, phase A, B, (and C if applicable) ADC pins
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, A0, A1, A2);


// Define neutral position (e.g., in radians or degrees, depending on the sensor unit)
const float neutral_position = 1.1; // Example: 0 radians or degrees
const float stable_offset = 0.5;    // Offset for stable positions on either side
const float stable_position_1 = neutral_position - stable_offset;
const float stable_position_2 = neutral_position + stable_offset;

// Maximum voltage to apply
const float max_voltage = 10;
// Proportional gain for calculating voltage from position error
const float kP = 6;
// Dead zone range (e.g., in radians or degrees, depending on the sensor unit)
const float dead_zone = 0.05; // Adjust this value based on your requirements

void setup() {
  // Initialise magnetic sensor hardware
  sensor.init();
  // Link the motor to the sensor
  motor.linkSensor(&sensor);

  // Initialize current sensor
  current_sense.linkDriver(&driver); // Link current sense and driver for synchronization
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    while(1); // Halt if current sense failed to initialize
  }
  motor.linkCurrentSense(&current_sense); // Link current sense with the motor

  // Power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Aligning voltage 
  motor.voltage_sensor_align = 5;
  // Choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // Set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // Use monitoring with serial 
  Serial.begin(115200);
  // Comment out if not needed
  motor.useMonitoring(Serial);

  // Initialize motor
  motor.init();
  // Align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready. Adjusting to maintain neutral position."));
}

void loop() {
  // Main FOC algorithm function
  motor.loopFOC();
  PhaseCurrent_s  current = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

 // Get the current position of the motor in radians
    float position = motor.shaft_angle;

    // Determine the closest stable point
    float distance_to_1 = fabs(position - stable_position_1);
    float distance_to_2 = fabs(position - stable_position_2);

    // Correct for the wrapping of angular values
    if (distance_to_1 > PI) distance_to_1 = 2 * PI - distance_to_1;
    if (distance_to_2 > PI) distance_to_2 = 2 * PI - distance_to_2;

    // Find the closest stable position
    float target_position = (distance_to_1 < distance_to_2) ? stable_position_1 : stable_position_2;

    // Calculate the position error
    float position_error = position - target_position;

    if (position_error > PI) {
        position_error -= 2 * PI;
    } else if (position_error < -PI) {
        position_error += 2 * PI;
    }

    // Calculate the required voltage to move the motor towards the target position
    float target_voltage = constrain(position_error * kP, -max_voltage, max_voltage);

    // Move the motor with the calculated voltage
    motor.move(-target_voltage);

  // Serial output for the motor currents
  //Serial.print("Current A: ");
  //Serial.println(current.a);
  //Serial.println(" A");
  Serial.print(position);
  Serial.print(",");
  Serial.println(current_magnitude);

}
