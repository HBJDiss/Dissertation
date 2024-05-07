#include <SimpleFOC.h>

// Magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// BLDC motor and driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 3, 11, 7);

// Inline current sensor setup
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 20, A0, A1, A2);

// Define stability points
const float neutral_position = 1.1; // Center
const float stable_position_1 = 0.8; // Lower stability point
const float stable_position_2 = 1.4; // Upper stability point

// Maximum voltage to apply
const float max_voltage = 6;


// Store the previous position for derivative calculation
float previous_position = 0;

// Define amplitude constants
const float AmplitudeInner = 2.5; // Amplitude for inner regions
const float AmplitudeOuter = 100; // Amplitude for outer regions
const float kD = 300;

// Low-pass filter configuration
float alpha = 0; // Smoothing factor
float derivative_filtered = 0; // Smoothed derivativative

void setup() {
    Serial.begin(115200);

    // Basic initialization
    sensor.init();
    motor.linkSensor(&sensor);

    // Initialize current sensor
    current_sense.linkDriver(&driver); // Link current sense and driver for synchronization
    if (current_sense.init()) {
        Serial.println("Current sense init success!");
    } else {
        Serial.println("Current sense init failed!");
        while (1); // Halt if current sense failed to initialize
    }
    motor.linkCurrentSense(&current_sense); // Link current sense with the motor

    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;

    motor.useMonitoring(Serial);
    
    motor.init();
    motor.initFOC();
}



void loop() {
    motor.loopFOC();
    
    PhaseCurrent_s current = current_sense.getPhaseCurrents();
    float current_magnitude = current_sense.getDCCurrent();

    // Current motor position
    float position = motor.shaft_angle;

    float derivative = (position - previous_position);
    

    // Apply exponential moving average to smooth the derivative
    derivative_filtered = (alpha * derivative) + ((1 - alpha) * derivative_filtered);

    // Identify the region
    int region;
    if (position < stable_position_1) {
        region = 1; // Region 1: Below low stability point
    } else if (position >= stable_position_1 && position < neutral_position) {
        region = 2; // Region 2: Between low stability point and center
    } else if (position >= neutral_position && position < stable_position_2) {
        region = 3; // Region 3: Between center and upper stability point
    } else {
        region = 4; // Region 4: Beyond upper stability point
    }

    float target_voltage = 0;

    switch (region) {
        case 1:
            // Control logic for Region 1
            target_voltage = 1* AmplitudeOuter*(position-0.8)*(position-0.8);
            break;
        case 2:
            // Control logic for Region 2
            target_voltage = -1*((AmplitudeInner * sin(20.94 * (position + 0.027))) + AmplitudeInner);
            break;
        case 3:
            // Control logic for Region 3
            target_voltage = 1 * ((AmplitudeInner * sin(21.05 * (position + 2.11))) + AmplitudeInner);
            break;
        case 4:
            // Control logic for Region 4
            target_voltage = -1* AmplitudeOuter*(position-1.4)*(position-1.4);;
            break;
    }
    float derivative_term = (kD*derivative);

     // Apply damping with the low-pass-filtered derivative
    if (region == 1 || region == 3) {
        target_voltage -= derivative_term; // Directional damping with filtering
    } else if (region == 2 or  4) {
        target_voltage += derivative_term; // Directional damping with filtering
    }
    
    // Constrain the target voltage
    target_voltage = constrain(target_voltage, -max_voltage, max_voltage);

    motor.move(target_voltage);

    previous_position = position;

  // Serial output for the motor position and current
  //Serial.print(target_voltage);
  //Serial.print(",");
  //Serial.print(derivative);
  //Serial.print(",");
  //Serial.print(derivative_term);
  //Serial.print(",");
  Serial.print(position);
  Serial.print(",");
  Serial.println(current_magnitude);
}

