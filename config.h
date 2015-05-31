#ifndef DEFS_h
#define DEFS_h

#include <AP_Math.h>


// Version of the EEPROM parameter table
#define EEPROM_FORMAT_VS     121

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
  #define A_LED_PIN          37
  #define B_LED_PIN          36
  #define C_LED_PIN          35
  #define LED_ON             HIGH
  #define LED_OFF            LOW
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
  #define A_LED_PIN          27
  #define B_LED_PIN          26
  #define C_LED_PIN          25
  #define LED_ON             LOW
  #define LED_OFF            HIGH
#endif
//////////////////////////////////////////////////////////////////////////////////////////
// Battery statistics
//////////////////////////////////////////////////////////////////////////////////////////
#define BATT_MIN_VOLTAGE     9.0    // 3 cells @ 3.0 V
#define BATT_MAX_VOLTAGE     25.2   // 6 cells @ 3.7 V
#define BATT_T_MS            100
#define BATT_CAP_mAh         10000

//////////////////////////////////////////////////////////////////////////////////////////
// General settings
//////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG_OUT            0
#define DBGRC_OUT            0
#define BENCH_OUT            0

#define NR_OF_PIDS           10
// PID indices, 
// first the rates
#define PID_PIT_RATE         0
#define PID_ROL_RATE         1
#define PID_YAW_RATE         2
#define PID_THR_RATE         3
#define PID_ACC_RATE         4
// then the stabs
#define PID_PIT_STAB         5
#define PID_ROL_STAB         6
#define PID_YAW_STAB         7
#define PID_THR_STAB         8
#define PID_ACC_STAB         9

// Motor numbers definitions for X configuration
#define MOTOR_FR             0      // Front right  (CW)
#define MOTOR_BL             1      // back left    (CW)
#define MOTOR_FL             2      // Front left   (CCW)
#define MOTOR_BR             3      // back right   (CCW)

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_OFF           1000   // Motors completely off
#define RC_THR_ACRO          1225   // Minimum throttle to begin with stabilization
#define RC_THR_MAX           2000   // Maximum throttle bias
#define RC_THR_80P           1675   // Maximum allowed throttle value, settable by user

// Degree range for remote control
#define RC_YAW_MIN           -180
#define RC_YAW_MAX           +180
#define RC_PIT_MIN           -45
#define RC_PIT_MAX           +45
#define RC_ROL_MIN           -45
#define RC_ROL_MAX           +45

//////////////////////////////////////////////////////////////////////////////////////////
// Setup serial ports
//////////////////////////////////////////////////////////////////////////////////////////
#define BAUD_RATE_A          115200  // IO USB .. (maybe: 57600)
#define BAUD_RATE_C          57600  // 3DR RADIO

//////////////////////////////////////////////////////////////////////////////////////////
// Main loop
//////////////////////////////////////////////////////////////////////////////////////////
#define RCVR_T_MS            20     // Update frequency of the receiver loop: 50 Hz
#define INAV_T_MS            20     // Update frequency for the auto navigation system: 50 Hz
#define COMP_T_MS            150000 // Save compass offsets every 2.5 min

//////////////////////////////////////////////////////////////////////////////////////////
// Scheduler module
//////////////////////////////////////////////////////////////////////////////////////////
#define NO_PRC_SCHED         16     // Maximum number of processes in scheduler

//////////////////////////////////////////////////////////////////////////////////////////
// Receiver module
//////////////////////////////////////////////////////////////////////////////////////////
#define IN_BUFFER_S          384      // keep it small (SRAM is precious)
#define MAX_TOKEN_S          16       // MAX_TOKEN_S * 16 bytes: a lot of memory

#define USE_CRC16            0        // Force a CRC16 check for every JSON

#define RC_ROL               0
#define RC_PIT               1
#define RC_THR               2
#define RC_YAW               3

#define RADIO_MSG_LENGTH     7        // Maximum length of radio control message without stop byte
#define APM_IOCHAN_CNT 	     8

#define UART_A               0
#define UART_B               1
#define UART_C               2

#define MAV_TELEM_PORT       UART_C

#define USE_RCIN             0        // This firmware is not using any PPM radio as default
#define USE_UART_A           1        // This is the standard input source (USB from Raspberry Pi)
#define USE_UART_C           1        // And this is the fall-back option (3DR Radio 433 or 900 MHz)

#define COM_PKT_TIMEOUT      500      // Time-out in ms; If the time-out is triggered the machine will go down
#define RCIN_TIMEOUT         250      // Time-out of the ppm radio in ms; If time-out is triggered the firmware tries to receive packets via the USB port on uartA
#define UART_A_TIMEOUT       250      // Time-out of the console serial port in ms; If time-out is triggered the firmware tries to receive packets via the 3DR radio on uartC

//////////////////////////////////////////////////////////////////////////////////////////
// Device module
//////////////////////////////////////////////////////////////////////////////////////////
#define INERT_LOWPATH_FILT   20.f     // Filter constant for the accelerometer
#define INERT_ANGLE_BIAS     60
#define INERT_G_CONST        9.81f

#define COMPASS_INSTALLED             // There is a bug/feature in the scheduler ('bool init()' the compass without anyone installed leads the program into an endless loop
#define COMPASS_CURR_SENS    1
#define COMPASS_UPDATE_T     100

#define COMPASS_EXTERN       1
#define COMPASS_ORIENTATION  ROTATION_ROLL_180

#define RANGE_FINDER_PIN     13
#define RANGE_FINDER_SCALE   1.0f

//////////////////////////////////////////////////////////////////////////////////////////
// Error handling
//////////////////////////////////////////////////////////////////////////////////////////
#define THR_MOD_STEP_S       1.25f
#define THR_TAKE_OFF         1300
#define THR_MIN_STEP_S       25.f
#define MAX_FALL_SPEED_MS    0.833f

//////////////////////////////////////////////////////////////////////////////////////////
// Auto navigation
//////////////////////////////////////////////////////////////////////////////////////////
#define HLD_ALTITUDE_TIMER   20     // 50 Hz
#define HLD_ALTITUDE_ZGBIAS  0.25f  // in g
#define HLD_ALTITUDE_ZTBIAS  25     // in ms

#endif /*DEFS_h*/
