#ifndef ESP32_WIFI_CONFIG_H
#define ESP32_WIFI_CONFIG_H

#define LED_PIN LED_BUILTIN //used for debugging status

#define LINO_BASE MECANUM
#define USE_GENERIC_2_IN_MOTOR_DRIVER

#define USE_GY85_IMU

#define K_P 0.6
#define K_I 0.8
#define K_D 0.5

#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.001, 0.001, 0.001 }
#define ORI_COV { 0.01, 0.01, 0.01 }
#define MAG_COV { 1e-12, 1e-12, 1e-12 }
#define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

//robot specs
#define MOTOR_MAX_RPM 110                   // motor's max RPM
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 16.4        // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 16.4   // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 121                 // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 121                 // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 121                 // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 121                 // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.060                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.145            // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency
#define SERVO_BITS 11                       // Servo PWM resolution
#define SERVO_FREQ 100000                   // Servo PWM frequency

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV true
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV true
#define MOTOR3_INV false
#define MOTOR4_INV false

// ENCODER PINS
#define MOTOR1_ENCODER_A 36
#define MOTOR1_ENCODER_B 39

#define MOTOR2_ENCODER_A 34
#define MOTOR2_ENCODER_B 35

#define MOTOR3_ENCODER_A 19
#define MOTOR3_ENCODER_B 18

#define MOTOR4_ENCODER_A 17
#define MOTOR4_ENCODER_B 4

// MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 27
  #define MOTOR1_IN_A 32
  #define MOTOR1_IN_B 33

  #define MOTOR2_PWM 14
  #define MOTOR2_IN_A 25
  #define MOTOR2_IN_B 26

  #define MOTOR3_PWM 23
  #define MOTOR3_IN_A 12
  #define MOTOR3_IN_B 13

  #define MOTOR4_PWM 16
  #define MOTOR4_IN_A 2
  #define MOTOR4_IN_B 15

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#include "wifi_ap.h"
#include "ip_settings.h"
#define AGENT_PORT 8888
#define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
#define USE_ARDUINO_OTA
#define USE_SYSLOG
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "R2D32"
#define APP_NAME "hardware"
#define BAUDRATE 921600
#define SDA_PIN 21
#define SCL_PIN 22
#define NODE_NAME "R2D32"
#define USE_SHORT_BRAKE
#define BOARD_INIT { \
    Wire.begin(SDA_PIN, SCL_PIN); \
    Wire.setClock(400000); \
}

#ifdef USE_SYSLOG
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    syslog(LOG_ERR, "%s RCCHECK failed %d", __FUNCTION__, temp_rc); \
    return false; }}
#else
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    flashLED(3); \
    return false; }} // do not block
#endif

#endif
