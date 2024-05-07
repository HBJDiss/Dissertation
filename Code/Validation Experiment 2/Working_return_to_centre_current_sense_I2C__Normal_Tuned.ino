#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

// magnetic sensor instance - I2C
MagneticSensorI2C sensor_motor1 = MagneticSensorI2C(0x41, 14, 0xFE, 8);
MagneticSensorI2C sensor_motor2 = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 5, 6, 8);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(10, 3, 11, 7);


// Inline current sensor setup
// Parameters: shunt resistor value, gain, phase A, B, (and C if applicable) ADC pins
InlineCurrentSense current_sense_motor1 = InlineCurrentSense(0.01, 20, A0, A1, A2);
InlineCurrentSense current_sense_motor2 = InlineCurrentSense(0.01, 20, A3, A2, A1); // Example ADC pins for the second motor

// Define neutral position for each motor
const float neutral_position_motor1 = 0; // Example: 0 radians or degrees for motor 1
const float neutral_position_motor2 = 1.1; // Example: 0 radians or degrees for motor 2

// Maximum voltage to apply
const float max_voltage = 10;
// Proportional gain for calculating voltage from position error
const float kP = 6;
// Dead zone range (e.g., in radians or degrees, depending on the sensor unit)
const float dead_zone = 0.05; // Adjust this value based on your requirements

// Variables for oscillation
// Oscillation Parameters
float amplitude = 0.5; // Amplitude of the oscillation in radians
float frequency = 2; // Frequency of the oscillations per second
unsigned long startTime = millis(); // Record start time for oscillation

// angle set point variable
float target_angle = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
  // Initialise magnetic sensor hardware for both motors
  sensor_motor1.init();
  sensor_motor2.init();

  // Link each motor to its respective sensor
  motor1.linkSensor(&sensor_motor1);
  motor2.linkSensor(&sensor_motor2);

  // Initialize current sensors for both motors
  current_sense_motor1.linkDriver(&driver1); // Link current sense and driver for synchronization for motor 1
  current_sense_motor2.linkDriver(&driver2); // Link current sense and driver for synchronization for motor 2
  
  // Initialize current sense for motor 1
  if (current_sense_motor1.init()) {
    Serial.println("Current sense for motor 1 init success!");
  } else {
    Serial.println("Current sense for motor 1 init failed!");
    while(1); // Halt if current sense for motor 1 failed to initialize
  }

  // Initialize current sense for motor 2
  if (current_sense_motor2.init()) {
    Serial.println("Current sense for motor 2 init success!");
  } else {
    Serial.println("Current sense for motor 2 init failed!");
    while(1); // Halt if current sense for motor 2 failed to initialize
  }

  // Link current sense with the respective motors
  motor1.linkCurrentSense(&current_sense_motor1);
  motor2.linkCurrentSense(&current_sense_motor2);

  // Power supply voltage
  driver1.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;
  driver1.init();
  driver2.init();

  // Link each motor to the driver
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  // Aligning voltage 
  motor1.voltage_sensor_align = 5;
  motor2.voltage_sensor_align = 5;

  // Choose FOC modulation (optional)
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Set motion control loop to be used
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor1.PID_velocity.P = 2.0f;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor1.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor1.P_angle.P = 40;
  // maximal velocity of the position control
  motor1.velocity_limit = 40;


  // Use monitoring with serial 
  Serial.begin(115200);
  // Comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // Initialize both motors
  motor1.init();
  motor2.init();

  // Align sensor and start FOC for both motors
  motor1.initFOC();
  motor2.initFOC();

 // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motors ready. Adjusting to maintain neutral position."));
}



void loop() {
  // Main FOC algorithm function for motor 2
  motor2.loopFOC();

  PhaseCurrent_s current_motor2 = current_sense_motor2.getPhaseCurrents();
  float current_magnitude_motor2 = current_sense_motor2.getDCCurrent();

  // Update sensor for motor 2
  sensor_motor2.update();

  // Calculate position error and adjust motor voltages for motor 2
  float position_motor2 = motor2.shaft_angle; // Get current position in radians or your unit of choice for motor 2
  float position_error_motor2 = position_motor2 - neutral_position_motor2;

  // Adjust position error to find the shortest path for motor 2
  if (position_error_motor2 > PI) {
    position_error_motor2 -= 2 * PI; // Adjust error to go the shorter way around for motor 2
  } else if (position_error_motor2 < -PI) {
    position_error_motor2 += 2 * PI; // Adjust error to go the shorter way around for motor 2
  }

  // Check if the position error is within the dead zone for motor 2
  if (abs(position_error_motor2) < dead_zone) {
    // Within dead zone for motor 2, no movement required
    motor2.move(0);
  } else {
    // Outside dead zone for motor 2, calculate and apply target voltage
    float target_voltage_motor2 = constrain(position_error_motor2 * kP, -max_voltage, max_voltage);
    motor2.move(-target_voltage_motor2);
  }

  // Main FOC algorithm function for motor 1
  motor1.loopFOC();

  PhaseCurrent_s current_motor1 = current_sense_motor1.getPhaseCurrents();
  float current_magnitude_motor1 = current_sense_motor1.getDCCurrent();

  // Update sensor for motor 1
  sensor_motor1.update();

   float position_motor1 = motor1.shaft_angle; // Get current position in radians or your unit of choice for motor 1

 // Calculate the current phase of the sine wave
  float time = (millis() - startTime) / 1000.0; // Time in seconds since start
  float phase = TWO_PI * frequency * time; // Phase of the sine wave
  float targetAngle = (amplitude * sin(phase)) + neutral_position_motor1; // Calculate target angle

  motor1.move(targetAngle); // Move motor to target angle

  // Serial output for the motor currents



  
  // Print position errors for both motors
  //Serial.print("Position Error (Motor 1): ");
  //Serial.println(position_error_motor1);
  //Serial.print("Position Error (Motor 2): ");
  //Serial.println(position_error_motor2);
  //Serial.println(motor1.shaft_angle);
  //Serial.println(sensor_motor1.getAngle());
  //Serial.print("P1");
  Serial.print(position_motor1);
  Serial.print(",");
  //Serial.print("C1");
  Serial.print(current_magnitude_motor1);
  Serial.print(",");
  //Serial.print("P2");
  Serial.print(position_motor2);
  Serial.print(",");
  //Serial.print("C2");
  Serial.println(current_magnitude_motor2);
    command.run();
}

