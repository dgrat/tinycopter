#include <AP_AHRS.h>
#include <AP_BattMonitor.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_RangeFinder_MaxsonarI2CXL.h>
#include <AP_GPS.h>
#include <AP_RangeFinder.h>
#include <AP_BoardLED.h>
#include <AP_SerialManager.h>

#include <LowPassFilter.h>

#include "arithmetics.h"
#include "config.h"
#include "device.h"


// create board led object
AP_BoardLED board_led;

const AP_Param::GroupInfo DeviceInit::var_info[] PROGMEM = {
    AP_GROUPEND
};

void display_offsets_and_scaling(const AP_HAL::HAL *pHAL, AP_InertialSensor *pInert) {
  Vector3f accel_offsets = pInert->get_accel_offsets();
  Vector3f accel_scale = pInert->get_accel_scale();
  Vector3f gyro_offsets = pInert->get_gyro_offsets();

  // display results
  #ifdef __AVR__
  pHAL->console->printf_P(PSTR("\nAccel offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                  accel_offsets.x,
                  accel_offsets.y,
                  accel_offsets.z);
  pHAL->console->printf_P(PSTR("Accel scale \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                  accel_scale.x,
                  accel_scale.y,
                  accel_scale.z);
  pHAL->console->printf_P(PSTR("Gyro offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                  gyro_offsets.x,
                  gyro_offsets.y,
                  gyro_offsets.z);
  #else
  pHAL->console->printf("\nAccel offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                  accel_offsets.x,
                  accel_offsets.y,
                  accel_offsets.z);
  pHAL->console->printf("Accel scale \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                  accel_scale.x,
                  accel_scale.y,
                  accel_scale.z);
  pHAL->console->printf("Gyro offsets \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                  gyro_offsets.x,
                  gyro_offsets.y,
                  gyro_offsets.z);
    #endif
}

void display_offsets_and_field(const AP_HAL::HAL *pHAL, Compass *pComp) {
  Vector3f comp_offsets = pComp->get_offsets();
  Vector3f comp_field = pComp->get_field();
  
  // display results
  #ifdef __AVR__
  // display results
  pHAL->console->printf_P(PSTR("\nCompass offsets  X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                  comp_offsets.x,
                  comp_offsets.y,
                  comp_offsets.z);
  pHAL->console->printf_P(PSTR("Compass field \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                  comp_field.x,
                  comp_field.y,
                  comp_field.z);
  #else
  pHAL->console->printf("\nCompass offsets  X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                  comp_offsets.x,
                  comp_offsets.y,
                  comp_offsets.z);
  pHAL->console->printf("Compass field \t X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                  comp_field.x,
                  comp_field.y,
                  comp_field.z);
  #endif
}

///////////////////////////////////////////////////////////
// DeviceInit
///////////////////////////////////////////////////////////
void DeviceInit::init_rf() {
  // TODO: needs revision
  #if defined(RNGFND_TYPE) && defined(RNGFND_PIN) && defined(RNGFND_SCALING)
  enum ap_var_type type;
  AP_Int32 *rf_type   = (AP_Int32 *)AP_Param::find("RNGFND_TYPE",    &type);
  AP_Int32 *rf_pin    = (AP_Int32 *)AP_Param::find("RNGFND_PIN",     &type);
  AP_Int32 *rf_acalde = (AP_Int32 *)AP_Param::find("RNGFND_SCALING", &type);

  rf_type->set_and_save(RangeFinder::RangeFinder_TYPE_ANALOG);
  rf_pin->set_and_save(RANGE_FINDER_PIN);
  rf_acalde->set_and_save(RANGE_FINDER_SCALE);
  #endif

  m_pHAL->scheduler->delay(1000);
  m_pRF->init();
}

void DeviceInit::init_inertial_nav() {
  m_pAHRS->set_compass(m_pComp);
  m_t32Compass = m_t32Inertial = m_pHAL->scheduler->millis();
}

void DeviceInit::init_barometer() {
  m_pBaro->init();
  m_pBaro->calibrate();
}

void DeviceInit::init_compass() {
  // Maybe we have a crappy APM2 with external compass and know about the orientation already
  #if defined(COMPASS_EXTERN) && defined(COMPASS_EXTERN)
  enum ap_var_type type;
  AP_Int32 *comp_ext = (AP_Int32 *)AP_Param::find("COMPASS_EXTERNAL", &type);
  AP_Int32 *comp_ori = (AP_Int32 *)AP_Param::find("COMPASS_ORIENT",   &type);
  
  bool bIsCompassExternal = (bool)*comp_ext;
  if(bIsCompassExternal != COMPASS_EXTERN) {
    #ifdef __AVR__
    m_pHAL->console->printf_P(PSTR("  * Init compass: register compass as external: %d and set orientation: %d\n"), COMPASS_EXTERN, COMPASS_ORIENTATION);
    #else
    m_pHAL->console->printf("  * Init compass: register compass as external: %d and set orientation: %d\n", COMPASS_EXTERN, COMPASS_ORIENTATION);
    #endif
    comp_ext->set_and_save(COMPASS_EXTERN);
    comp_ori->set_and_save(COMPASS_ORIENTATION);
  }
  else {
    #ifdef __AVR__
    m_pHAL->console->printf_P(PSTR("  * Init compass: compass already registered as external: %d and with orientation: %d\n"), COMPASS_EXTERN, COMPASS_ORIENTATION);
    #else
    m_pHAL->console->printf("  * Init compass: compass already registered as external: %d and with orientation: %d\n", COMPASS_EXTERN, COMPASS_ORIENTATION);
    #endif
  }
  #endif

  if(!m_pComp->init() ) {
    #ifdef __AVR__
    m_pHAL->console->printf_P(PSTR("  * Init compass failed!\n") );
    #else
    m_pHAL->console->printf("  * Init compass failed!\n");
    #endif
    return;
  }
  
  if(COMPASS_CURR_SENS) {
    m_pComp->motor_compensation_type(2); // current sensing
  }
  else {
    m_pComp->motor_compensation_type(1); // throttle as input
  }
  
  #ifdef __AVR__
  m_pHAL->console->printf_P(PSTR("  * init done - %u compasses detected\n"), m_pComp->get_count() );
  #else
  m_pHAL->console->printf("  * init done - %u compasses detected\n", m_pComp->get_count() );
  #endif

  // Display offsets
  display_offsets_and_field(m_pHAL, m_pComp);

  m_t32Compass = m_pHAL->scheduler->millis();
}

void DeviceInit::init_inertial() {
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  m_pHAL->gpio->pinMode(40, HAL_GPIO_OUTPUT);
  m_pHAL->gpio->write(40, HIGH);
#endif
  // Turn on MPU6050
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_400HZ);
#else
  m_pInert->init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
#endif
  // Some output
  display_offsets_and_scaling(m_pHAL, m_pInert);
  // Timer init
  m_t32Inertial = m_pHAL->scheduler->millis();
}

void DeviceInit::init_gps() {
  m_pSerMan->init();
  // Initialise the LEDs
  board_led.init();
  // Init the GPS
  m_pGPS->init(NULL, reinterpret_cast<const AP_SerialManager &>(*m_pSerMan) );
}

void DeviceInit::init_batterymon() {
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
  m_pBat->set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_SMBUS);
#else
  m_pBat->set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
#endif
  m_pBat->init();
}

DeviceInit::DeviceInit( const AP_HAL::HAL *pHAL, AP_SerialManager *pSerManager, AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, AP_GPS *pGPS, AP_BattMonitor *pBat, RangeFinder *pRF, AP_AHRS *pAHRS, AC_PID *pPIDs)
{
  m_pPIDs             = pPIDs;
  
  m_pHAL              = pHAL;
  m_pSerMan           = pSerManager;
  m_pInert            = pInert;
  m_pComp             = pComp;
  m_pBaro             = pBar;
  m_pGPS              = pGPS;
  m_pBat              = pBat;
  m_pRF               = pRF;
  m_pAHRS             = pAHRS;
  m_eErrors           = NOTHING_F;

  m_t32Compass = m_t32Inertial = m_pHAL->scheduler->millis();
}

///////////////////////////////////////////////////////////
// Device
///////////////////////////////////////////////////////////
void Device::load_pids() {
  for(int i = 0; i < NR_OF_PIDS; i++) {
    m_pPIDs[i].load_gains();
    #ifdef __AVR__
    m_pHAL->console->printf_P(PSTR("Load PID P %f  I %f  D %f  imax %f\n"), (double)m_pPIDs[i].kP(), (double)m_pPIDs[i].kI(), (double)m_pPIDs[i].kD(), (double)m_pPIDs[i].imax());
    #else
    m_pHAL->console->printf("Load PID P %f  I %f  D %f  imax %f\n", (double)m_pPIDs[i].kP(), (double)m_pPIDs[i].kI(), (double)m_pPIDs[i].kD(), (double)m_pPIDs[i].imax());
    #endif
  }
}

void Device::save_pids() {
  for(int i = 0; i < NR_OF_PIDS; i++) {
    m_pPIDs[i].save_gains();
  }
}

AC_PID &Device::get_pid(uint_fast8_t index) {
  if(index >= NR_OF_PIDS) {
    return m_pPIDs[NR_OF_PIDS-1];
  }
  return m_pPIDs[index];
}

float Device::get_pid_val(uint_fast8_t index) {
  if(index >= NR_OF_PIDS) {
    return m_pPIDs[NR_OF_PIDS-1].get_pid();
  }
  return m_pPIDs[index].get_pid();
}

void Device::set_pid(uint_fast8_t index, const AC_PID &pid) {
  if(index >= NR_OF_PIDS) {
    m_pPIDs[NR_OF_PIDS-1] = pid;
  }
  m_pPIDs[index] = pid;
}

void Device::set_pids_dt(const float &dt) {
  // Rate PIDs
  m_pPIDs[PID_PIT_RATE].set_dt(dt);
  m_pPIDs[PID_ROL_RATE].set_dt(dt);
  m_pPIDs[PID_YAW_RATE].set_dt(dt);
  m_pPIDs[PID_THR_RATE].set_dt(dt);
  m_pPIDs[PID_ACC_RATE].set_dt(dt);
  // STAB PIDs
  m_pPIDs[PID_PIT_STAB].set_dt(dt);
  m_pPIDs[PID_ROL_STAB].set_dt(dt);
  m_pPIDs[PID_YAW_STAB].set_dt(dt);
  m_pPIDs[PID_THR_STAB].set_dt(dt);
  m_pPIDs[PID_ACC_STAB].set_dt(dt);
}

void Device::update_attitude() {
  // Update the inertial and calculate attitude
  m_pAHRS->update();
  // Calculate the attitude based on the accelerometer
  calc_acceleration();

#if BENCH_OUT
  static int iBCounter = 0;
  int iBCurTime = m_pHAL->scheduler->millis();
  ++iBCounter;
  if(iBCurTime - m_t32Inertial >= 1000) {
    #ifdef __AVR__
    m_pHAL->console->printf_P(PSTR("Benchmark - update_attitude(): %d Hz\n"), iBCounter);
    #else
    m_pHAL->console->printf("Benchmark - update_attitude(): %d Hz\n", iBCounter);
    #endif
    iBCounter = 0;
    m_t32Inertial = iBCurTime;
  }
#endif

  m_vAtti_deg.x = ToDeg(m_pAHRS->pitch);
  m_vAtti_deg.y = ToDeg(m_pAHRS->roll);
  m_vAtti_deg.z = ToDeg(m_pAHRS->yaw);
}

Device::Device( const AP_HAL::HAL *pHAL, AP_SerialManager *pSerManager, AP_InertialSensor *pInert, Compass *pComp, AP_Baro *pBar, AP_GPS *pGPS, AP_BattMonitor *pBat, RangeFinder *pRF, AP_AHRS *pAHRS, AC_PID *pPIDs) :
DeviceInit(pHAL, pSerManager, pInert, pComp, pBar, pGPS,  pBat, pRF, pAHRS, pPIDs)
{
  m_iAltitude_cm      = 0;

  m_fInertRolCor      = 0.f;
  m_fInertPitCor      = 0.f;

  m_vAtti_deg.x       = 0.f;
  m_vAtti_deg.y       = 0.f;
  m_vAtti_deg.z       = 0.f;

  m_fCmpH             = 0.f;
  m_fGpsH             = 0.f;

  m_AccelLPF = LowPassFilterVector3f(INERT_LOWPATH_FILT);
}

/*TODO needs revision*/
int_fast32_t Device::read_rf_cm() {
  m_pRF->update();
  if(m_pRF->status() == RangeFinder::RangeFinder_Good) {
    m_iAltitude_cm = m_pRF->distance_cm();
  }
  return m_iAltitude_cm;
}

int_fast32_t Device::get_rf_cm() {
  return m_iAltitude_cm;
}

void Device::set_trims(float fRoll_deg, float fPitch_deg) {
  m_fInertRolCor = fRoll_deg;
  m_fInertPitCor = fPitch_deg;
}

Vector3f Device::get_atti_cor_deg() {
  return Vector3f(m_vAtti_deg.x - m_fInertPitCor, // Pitch correction for imbalances
                  m_vAtti_deg.y - m_fInertRolCor, // Roll correction for imbalances
                  m_vAtti_deg.z);                 // Yaw is without correction on that point, because compass/GPS is thought to do that job, but not here
}

Vector3f Device::get_atti_raw_deg() {
  return m_vAtti_deg;
}

Vector3f Device::get_gyro_degps() {
  if(!m_pInert->healthy() ) {
    #ifdef __AVR__
    //m_pHAL->console->printf_P(PSTR("read_gyro_deg(): Inertial not healthy\n") );
    m_pHAL->console->printf("read_gyro_deg(): Inertial not healthy\n");
    #else
    m_pHAL->console->printf("read_gyro_deg(): Inertial not healthy\n");
    #endif
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, GYROMETER_F) );
    return m_vGyro_deg;
  }

  // Read the current gyrometer value
  m_vGyro_deg = m_pInert->get_gyro();

  // Save values
  float fRol = ToDeg(m_vGyro_deg.x); // in comparison to the accelerometer data swapped
  float fPit = ToDeg(m_vGyro_deg.y); // in comparison to the accelerometer data swapped
  float fYaw = ToDeg(m_vGyro_deg.z);

  // Put them into the right order
  m_vGyro_deg.x = fPit; // PITCH
  m_vGyro_deg.y = fRol; // ROLL
  m_vGyro_deg.z = fYaw; // YAW

  return m_vGyro_deg;
}

void Device::update_inav() {
  if(!m_pGPS) {
    return;
  }

  read_gps();
  read_comp_deg();
}

void Device::calc_acceleration() {
  if(!m_pInert->healthy() ) {
    #ifdef __AVR__
    //m_pHAL->console->printf_P(PSTR("calc_acceleration(): Inertial not healthy\n") );
    m_pHAL->console->printf("calc_acceleration(): Inertial not healthy\n");
    #else
    m_pHAL->console->printf("calc_acceleration(): Inertial not healthy\n");
    #endif
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, ACCELEROMETR_F) );
  }
  // Low Pass SFilter
  Vector3f vAccelCur_cmss = m_pAHRS->get_accel_ef() * 100.f;
  m_vAccelPG_cmss = m_AccelLPF.apply(vAccelCur_cmss, m_t32Inertial);

  // Calculate G-const. corrected acceleration
  m_vAccelMG_cmss = vAccelCur_cmss - m_vAccelPG_cmss;
}

Vector3f Device::get_accel_mg_cmss() {
  return m_vAccelMG_cmss;
}

Vector3f Device::get_accel_pg_cmss() {
  return m_vAccelPG_cmss;
}

/*
 * Return true if compass was healthy
 * In heading the heading of the compass is written.
 * All units in degrees
 */
float Device::read_comp_deg() {
  if (!m_pComp->use_for_yaw() ) {
    //m_pHAL->console->printf("read_comp_deg(): Compass not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, COMPASS_F) );
    return m_fCmpH;
  }

  // accumulate compass values
  m_pComp->accumulate();

  // Update the compass readout maximally ten times a second
  if(m_pHAL->scheduler->millis() - m_t32Compass <= COMPASS_UPDATE_T) {
    return m_fCmpH;
  }

  // After some time read the compass
  m_pComp->read();
  m_fCmpH = m_pComp->calculate_heading(m_pAHRS->get_dcm_matrix() );
  m_fCmpH = ToDeg(m_fCmpH);
  m_pComp->learn_offsets();

  return m_fCmpH;
}

GPSData Device::read_gps() {
  static bool bSetCompLocation = false;

  if(!m_pGPS) {
    return m_ContGPS;
  }

  m_pGPS->update();
  m_ContGPS.status = static_cast<uint_fast32_t>(m_pGPS->status() );

  if(m_ContGPS.status > AP_GPS::NO_FIX) {
    m_ContGPS.latitude    = m_pGPS->location().lat;
    m_ContGPS.longitude   = m_pGPS->location().lng;
    m_ContGPS.altitude_cm = m_pGPS->location().alt;

    m_ContGPS.gspeed_cms  = m_pGPS->ground_speed_cm();

    m_ContGPS.gcourse_cd  = m_pGPS->ground_course_cd();
    m_ContGPS.satelites   = m_pGPS->num_sats();
    m_ContGPS.time_week   = m_pGPS->time_week();
    m_ContGPS.time_week_s = m_pGPS->time_week_ms() / 1000.0;

    // If the GPS is working and the compass initiated,
    // set the location once
    if(m_pComp->healthy() && !bSetCompLocation) {
      m_pComp->set_initial_location(m_ContGPS.latitude, m_ContGPS.longitude);
      bSetCompLocation = true;
    }
  } else {
    //m_pHAL->console->printf("read_gps(): GPS not healthy\n");
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, GPS_F) );
  }

  return m_ContGPS;
}

BaroData Device::read_baro() {
  if (!m_pBaro->healthy() ) {
    #ifdef __AVR__
    //m_pHAL->console->printf_P(PSTR("read_baro(): Barometer not healthy\n") );
    m_pHAL->console->printf("read_baro(): Barometer not healthy\n");
    #else
    m_pHAL->console->printf("read_baro(): Barometer not healthy\n");
    #endif
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, BAROMETER_F) );
    return m_ContBaro;
  }

  m_pBaro->update();

  m_ContBaro.pressure_pa      = m_pBaro->get_pressure();
  m_ContBaro.temperature_deg  = m_pBaro->get_temperature();

  m_ContBaro.altitude_cm      = m_pBaro->get_altitude();
  m_ContBaro.climb_rate_cms   = m_pBaro->get_climb_rate();

  return m_ContBaro;
}

BattData Device::read_bat() {
  const unsigned int iRefVoltSamples = 25;
  static unsigned int iRefVoltCounter = 0;
  static float fRefVolt_V = 0.f;

  m_pBat->read();

  m_ContBat.voltage_V    = m_pBat->voltage();
  m_ContBat.current_A    = m_pBat->current_amps();
  m_ContBat.power_W      = m_ContBat.voltage_V * m_ContBat.current_A;
  m_ContBat.consumpt_mAh = m_pBat->current_total_mah();

  // Only perform current sensing for the compass
  // if there is a ATTO/3DR sensor in use
  if(m_pComp->healthy() ) {
    // set the current in A
    m_pComp->set_current(m_ContBat.current_A);
  }

  if(m_ContBat.voltage_V < BATT_MIN_VOLTAGE) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, VOLTAGE_LOW_F) );
  }
  if(m_ContBat.voltage_V > BATT_MAX_VOLTAGE) {
    m_eErrors = static_cast<DEVICE_ERROR_FLAGS>(tiny::add_flag(m_eErrors, VOLTAGE_HIGH_F) );
  }

  // Collect samples (iRefVoltSamples) for reference voltage (but only if readouts are valid)
  if(iRefVoltCounter < iRefVoltSamples) {
    if(m_ContBat.voltage_V <= BATT_MAX_VOLTAGE && m_ContBat.voltage_V >= BATT_MIN_VOLTAGE) {
      iRefVoltCounter++;
      fRefVolt_V += m_ContBat.voltage_V;
    }
  }
  // Only update reference voltage if necessary
  else if(m_ContBat.refVoltage_V < 0) {
    m_ContBat.refVoltage_V = fRefVolt_V / iRefVoltSamples;
  }

  return m_ContBat;
}

float Device::get_comp_deg() {
  return m_fCmpH;
}

BaroData Device::get_baro() {
  return m_ContBaro;
}

GPSData Device::get_gps() {
  return m_ContGPS;
}

BattData Device::get_bat() {
  return m_ContBat;
}

Vector3f Device::get_accel_mg_g() {
  static float fGForce_x = 0.f;
  static float fGForce_y = 0.f;
  static float fGForce_z = 0.f;
  
  // Sanity check
  if(abs(get_atti_raw_deg().x) > INERT_ANGLE_BIAS || abs(get_atti_raw_deg().y) > INERT_ANGLE_BIAS) {
    return Vector3f(fGForce_x, fGForce_y, fGForce_z);
  }

  float fCFactor = 100.f * INERT_G_CONST;
  Vector3f v3fCurAccel = get_accel_mg_cmss() / fCFactor;
  fGForce_x = v3fCurAccel.x;
  fGForce_y = v3fCurAccel.y;
  fGForce_z = v3fCurAccel.z;
  
  return v3fCurAccel;
}

Vector3f Device::get_accel_pg_g() {  
  static float fGForce_x = 0.f;
  static float fGForce_y = 0.f;
  static float fGForce_z = 0.f;
  
  // Sanity check
  if(abs(get_atti_raw_deg().x) > INERT_ANGLE_BIAS || abs(get_atti_raw_deg().y) > INERT_ANGLE_BIAS) {
    return Vector3f(fGForce_x, fGForce_y, fGForce_z);
  }

  float fCFactor = 100.f * INERT_G_CONST;
  Vector3f v3fCurAccel = get_accel_pg_cmss() / fCFactor;
  fGForce_x = v3fCurAccel.x;
  fGForce_y = v3fCurAccel.y;
  fGForce_z = v3fCurAccel.z;
  
  return v3fCurAccel;
}