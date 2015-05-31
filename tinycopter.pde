////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
// Ardu pilot library includes
////////////////////////////////////////////////////////////////////////////////
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>
#include <AP_Buffer.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_GPS.h>
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux.h>
#endif

#include <AP_HAL_AVR.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_Math.h>
#include <AP_Mission.h>
#include <AP_Notify.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_RangeFinder.h>
#include <AP_Terrain.h>
#include <AP_SerialManager.h>
#include <AP_Vehicle.h>
#include <AP_Param.h>
#include <AC_PID.h>

#include <RC_Channel.h>

#include <DataFlash.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <jsmn.h>
#include <StorageManager.h>

////////////////////////////////////////////////////////////////////////////////
// Own includes
////////////////////////////////////////////////////////////////////////////////
#include "output_debug.h"
#include "global.h"
#include "arithmetics.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
// Settings stored in the flash
void load_settings();
// Tasks
void rcvr_loop(int);
void inav_loop(int);
void batt_loop(int);
void comp_loop(int); // compass loop for saving the offsets periodically

Task taskRCVR(&rcvr_loop, 0);
Task taskINAV(&inav_loop, 0);
Task taskRBat(&batt_loop, 0);
Task taskCOMP(&comp_loop, 0);


void load_settings() {
  if (!AP_Param::check_var_info() ) {
    #ifdef __AVR__
    hal.console->printf_P(PSTR("Bad var table\n") );
    #else
    hal.console->printf("Bad var table\n");
    #endif
  }

  // disable centrifugal force correction, it will be enabled as part of the arming process
  _AHRS.set_correct_centrifugal(false);
  //hal.util->set_soft_armed(false);

  if (!g.format_version.load() ||
    g.format_version != Parameters::k_format_version) {
    // erase all parameters
    #ifdef __AVR__
    hal.console->printf_P(PSTR("Firmware change: erasing EEPROM...\n") );
    #else
    hal.console->printf("Firmware change: erasing EEPROM...\n");
    #endif
    AP_Param::erase_all();
    // save the current format version
    g.format_version.set_and_save(Parameters::k_format_version);
    #ifdef __AVR__
    hal.console->printf_P(PSTR("done\n") );
    #else
    hal.console->printf("done\n");
    #endif
  } else {
    uint32_t before = hal.scheduler->micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    #ifdef __AVR__
    hal.console->printf_P(PSTR("Load all settings took %d us\n"), hal.scheduler->micros() - before);
    #else
    hal.console->printf("Load all settings took %d us\n", hal.scheduler->micros() - before);
    #endif
  }
}

// Altitude estimation and AHRS system (yaw correction with GPS, barometer, ..)
void inav_loop(int) {
  _HAL_BOARD.update_inav();
  _HAL_BOARD.read_rf_cm();
}

// Receiver thread (remote control), which should be limited to 50 Hz
void rcvr_loop(int) {
  _RECVR.try_any();
}

// save offsets if automatic offset learning is on
void comp_loop(int) {
  if(_HAL_BOARD.m_pComp->learn_offsets_enabled() ) {
    _HAL_BOARD.m_pComp->save_offsets();
  }
}

// Read the battery.
// The voltage is used for adjusting the motor speed,
// as the speed is dependent on the voltage
void batt_loop(int) {
  _HAL_BOARD.read_bat();
  _MODEL.calc_batt_comp();
}

void setup() {
  // Prepare scheduler for the main loop ..
  _SCHED_NAV.add_task(&taskRCVR, RCVR_T_MS);
  _SCHED_NAV.add_task(&taskINAV, INAV_T_MS);
  _SCHED_NAV.add_task(&taskRBat, BATT_T_MS);
  _SCHED_NAV.add_task(&taskCOMP, COMP_T_MS);
  // .. and the sensor output functions
  #if !DBGRC_OUT
  _SCHED_OUT.add_task(&dbgOutAtti,   50);
  _SCHED_OUT.add_task(&dbgOutBaro,  500);
  _SCHED_OUT.add_task(&dbgOutComp,  500);
  _SCHED_OUT.add_task(&dbgOutBat,   500);
  _SCHED_OUT.add_task(&dbgOutGPS,  1000);
  _SCHED_OUT.add_task(&dbgOutPIDP, 1000);
  _SCHED_OUT.add_task(&dbgOutPIDR, 1000);
  _SCHED_OUT.add_task(&dbgOutPIDY, 1000);
  _SCHED_OUT.add_task(&dbgOutPIDT, 1000);
  _SCHED_OUT.add_task(&dbgOutPIDA, 1000);
  _SCHED_OUT.add_task(&dbgOutPIDS, 1000);
  #endif

  // Wait for one second
  hal.scheduler->delay(1000);
  // Set baud rate when connected to RPi
  hal.uartA->begin(BAUD_RATE_A, 256, 256);  // USB
  hal.uartC->begin(BAUD_RATE_C, 128, 128);  // RADIO

  #ifdef __AVR__
  hal.console->printf_P(PSTR("Setup device ..\n") );
  #else
  hal.console->printf("Setup device ..\n");
  #endif

  // Enable the motors and set at 490Hz update
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Set ESC refresh rate to 490 Hz\n"), tiny::progress_f(1, 9) );
  #else
  hal.console->printf("%.1f%%: Set ESC refresh rate to 490 Hz\n", tiny::progress_f(1, 9) );
  #endif
  for(uint_fast16_t i = 0; i < 8; i++) {
    hal.rcout->enable_ch(i);
  }
  hal.rcout->set_freq(0xFF, 490);

  // Load settings from EEPROM
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Load settings from EEPROM\n"), tiny::progress_f(2, 9) );
  #else
  hal.console->printf("%.1f%%: Load settings from EEPROM\n", tiny::progress_f(2, 9) );
  #endif
  load_settings();

  // Init the barometer
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init barometer\n"), tiny::progress_f(3, 9) );
  #else
  hal.console->printf("%.1f%%: Init barometer\n", tiny::progress_f(3, 9) );
  #endif
  _HAL_BOARD.init_barometer();

  // Init the accelerometer and gyrometer
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init inertial sensor\n"), tiny::progress_f(4, 9) );
  #else
  hal.console->printf("%.1f%%: Init inertial sensor\n", tiny::progress_f(4, 9) );
  #endif
  _HAL_BOARD.init_inertial();

  // Init the compass
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init compass: \n"), tiny::progress_f(5, 9) );
  #else
  hal.console->printf("%.1f%%: Init compass: \n", tiny::progress_f(5, 9) );
  #endif
  #ifdef COMPASS_INSTALLED
  _HAL_BOARD.init_compass();
  #endif

  // Init the GPS
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init GPS"), tiny::progress_f(6, 9) );
  #else
  hal.console->printf("%.1f%%: Init GPS", tiny::progress_f(6, 9) );
  #endif
  _HAL_BOARD.init_gps();

  // Init the battery monitor
  #ifdef __AVR__
  hal.console->printf_P(PSTR("\n%.1f%%: Init battery monitor\n"), tiny::progress_f(7, 9) );
  #else
  hal.console->printf("\n%.1f%%: Init battery monitor\n", tiny::progress_f(7, 9) );
  #endif
  _HAL_BOARD.init_batterymon();

  // Init the range finder
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init range finder\n"), tiny::progress_f(8, 9) );
  #else
  hal.console->printf("%.1f%%: Init range finder\n", tiny::progress_f(8, 9) );
  #endif
  //_HAL_BOARD.init_rf();

  // Init the inav system
  #ifdef __AVR__
  hal.console->printf_P(PSTR("%.1f%%: Init inertial navigation\n"), tiny::progress_f(9, 9) );
  #else
  hal.console->printf("%.1f%%: Init inertial navigation\n", tiny::progress_f(9, 9) );
  #endif
  _HAL_BOARD.init_inertial_nav();
}

void loop() {
  // send some json formatted information about the model over serial port
  _SCHED_NAV.run();
  _SCHED_OUT.run();

  // Attitude-, Altitude and Navigation control loop
  _MODEL.run();
}

AP_HAL_MAIN();












