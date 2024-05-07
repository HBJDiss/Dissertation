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

  // Calculate deviation from neutral position with shortest path consideration
  float position = motor.shaft_angle; // Get current position in radians or your unit of choice
  // Ensure both angles are in the same range (e.g., -PI to PI or 0 to 2*PI)
  float position_error = position - neutral_position;

  // Adjust position_error to find the shortest path
  // This checks if going the 'long way' around the circle is actually shorter
  if (position_error > PI) {
    position_error -= 2 * PI; // Adjust error to go the shorter way around
  } else if (position_error < -PI) {
    position_error += 2 * PI; // Adjust error to go the shorter way around
  }

  // Check if the position error is within the dead zone
  if (abs(position_error) < dead_zone) {
    // Within dead zone, no movement required
    motor.move(0);
  } else {
    // Outside dead zone, calculate and apply target voltage
    // Notice the negation is removed from target_voltage as we already adjust position_error direction
    float target_voltage = constrain(position_error * kP, -max_voltage, max_voltage);
    motor.move(-target_voltage);
  }

  // Serial output for the motor currents
  //Serial.print("Current A: ");
  //Serial.println(current.a);
  //Serial.println(" A");
  Serial.print(position);
  Serial.print(",");
  Serial.println(current_magnitude);

}