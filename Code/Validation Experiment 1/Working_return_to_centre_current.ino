#include <SimpleFOC.h>

// Magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Inline current sensor setup
// Parameters: shunt resistor value, gain, phase A, B, (and C if applicable) ADC pins
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, A0, A1, A2);

// Define neutral position (e.g., in radians or degrees, depending on the sensor unit)
const float neutral_position = 0; // Example: 0 radians or degrees

// Current control parameters
const float target_current = 3; // Target current in Amperes
const float P_gain = 5; // Proportional gain for current control
const float I_gain = 300; // Integral gain for current control

// Dead zone range (e.g., in radians or degrees, depending on the sensor unit)
const float dead_zone = 0.02; // Adjust this value based on your requirements

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

  // Motor control setup
  motor.controller = MotionControlType::torque; // Use torque for current control
  motor.torque_controller = TorqueControlType::foc_current; // Use FOC for precise current control
  
  // Current control gains
  motor.PID_current_q.P = P_gain;
  motor.PID_current_q.I = I_gain;
  motor.PID_current_q.D = 0;
  motor.PID_current_d.P = P_gain;
  motor.PID_current_d.I = I_gain;
  motor.PID_current_d.D = 0;
  motor.LPF_current_q.Tf = 0.01; 
  motor.LPF_current_d.Tf = 0.01;

  // Use monitoring with serial 
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  // Initialize motor
  motor.init();
  // Align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready. Adjusting to maintain neutral position."));
}

void loop() {
  motor.loopFOC();

  float position = motor.shaft_angle;
  float position_error = position - neutral_position;

 // Adjust position_error to find the shortest path
  // This checks if going the 'long way' around the circle is actually shorter
  if (position_error > PI) {
    position_error -= 2 * PI; // Adjust error to go the shorter way around
  } else if (position_error < -PI) {
    position_error += 2 * PI; // Adjust error to go the shorter way around
  }

  // Ensure the system is responsive and doesn't stall at -PI
  position_error = fmod(position_error + PI, 2 * PI) - PI;

  if (abs(position_error) < dead_zone) {
    motor.move(0); // No current demand
  } else {
    float target_current_adjusted = target_current * position_error;
    target_current_adjusted = constrain(target_current_adjusted, -1.5, 1.5);
    motor.move(target_current_adjusted);
    //Serial.println(target_current_adjusted);
  }
  //Serial.println(position_error);

  PhaseCurrent_s  current = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();
  Serial.println(current.a);
}