#include <SimpleFOC.h>

// Magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 3, 11, 7);

// Inline current sensor setup
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, A0, A1, A2);

// Control constants
const float neutral_position = 1.1; // Example: 0 radians
const float max_voltage = 10; // Max voltage to apply
const float kP = 10; // Proportional gain
const float kD = 1; // Damping gain for derivative control
const float dead_zone = 0.05; // Dead zone range

float previous_position = 0; // Variable to store the previous position
unsigned long previous_time = 0; // Variable to store the previous loop time

void setup() {
  // Initialize sensor, motor, and driver
  sensor.init();
  motor.linkSensor(&sensor);

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense); // Link current sense with the motor

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Aligning voltage 
  motor.voltage_sensor_align = 5;
  // Choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // Set motion control loop to be used
  motor.controller = MotionControlType::torque;

  Serial.begin(115200);
  
  motor.init();
  motor.initFOC();
  
  previous_position = motor.shaft_angle;
  previous_time = millis();
}

void loop() {
  // Main FOC algorithm function
  motor.loopFOC();

  PhaseCurrent_s  current = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();
  
  // Get current position and time
  float position = motor.shaft_angle;
  unsigned long current_time = millis();
  
  // Calculate position error
  float position_error = position - neutral_position;
  if (position_error > PI) {
    position_error -= 2 * PI;
  } else if (position_error < -PI) {
    position_error += 2 * PI;
  }

  // Calculate derivative (velocity)
  float delta_time = (current_time - previous_time) / 1000.0; // Convert to seconds
  float angular_velocity = (position - previous_position) / delta_time;

  // Update previous position and time
  previous_position = position;
  previous_time = current_time;

  // Check if within dead zone
  if (abs(position_error) < dead_zone) {
    motor.move(0);
  } else {
    // Calculate target voltage with proportional and derivative control
    float proportional_term = kP * position_error;
    float derivative_term = kD * angular_velocity; // Damping term
    
    float target_voltage = constrain(proportional_term - derivative_term, -max_voltage, max_voltage);
    
    motor.move(-target_voltage); // Apply the calculated voltage

    // Serial output for the motor currents
    //Serial.print(-target_voltage);
    //Serial.print(",");
    Serial.print(position);
    Serial.print(",");
    Serial.println(current_magnitude);
  }


}
