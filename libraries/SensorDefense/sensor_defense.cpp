#include "AP_HAL/system.h"
#include "AP_Logger/AP_Logger.h"
#include "AP_Math/vector2.h"
#include "SensorDefense/sensor_defense_structs.h"
#include <SensorDefense/sensor_defense.h>

//----   Helper Functions  ----//
Vector3f SensorDefense::abs(const Vector3f &x) {
  return Vector3f{std::abs(x.x), std::abs(x.y), std::abs(x.z)};
};
Vector2f SensorDefense::abs(const Vector2f &x) {
  return Vector2f{std::abs(x.x), std::abs(x.y)};
};


//----General Functions----//

void SensorDefense::initialize() {
  // Reset sensors
  sensors.currAccel.reset();
  sensors.nextAccel.reset();
  sensors.currGyro.reset();
  sensors.nextGyro.reset();
  sensors.currGps.reset();
  sensors.prevGps.reset();
  sensors.currOF.reset();
  sensors.nextOF.reset();
  sensors.rangefinder.reset();
  sensors.mag.reset();
  sensors.bar.reset();

  // Zero framework readings
  framework.gpsAvail = false;
  framework.ofAvail = false;
  framework.init = true;
  framework.lastUpdate = 0;
}

void SensorDefense::update() {
  const AP_InertialSensor *IMU = AP_InertialSensor::get_singleton();
  const OpticalFlow *AP_OF = OpticalFlow::get_singleton();

  // Magnetometer
  sensors.mag.update();
  sensors.bar.update();

  // ACCOF Sensors
  //  RF
  sensors.rangefinder.update();

  framework.gpsAvail = sensors.currGps.update(AP::gps().get_singleton(), sensors.prevGps);

  // IMU Sensors
  // Updating Current and Previous based on GPS Update rate
  uint32_t ts_imu = IMU->get_last_update_usec() / 1000;
  if ((ts_imu > sensors.currGps.Timestamp) &&
      (ts_imu - sensors.currGps.Timestamp) >= GPS_RATE) {
    sensors.nextAccel.update(IMU);
    sensors.nextGyro.update(IMU);
  } else {
    sensors.currAccel.update(IMU);
    sensors.currGyro.update(IMU);
  }

  uint32_t ts_of = sensors.currOF.Timestamp;
  if ((ts_of > sensors.currGps.Timestamp) &&
      (ts_of - sensors.currGps.Timestamp) >= GPS_RATE) {
    sensors.nextOF.update(AP_OF, sensors.rangefinder);
  } else {
    sensors.currOF.update(AP_OF, sensors.rangefinder);
  }
}

void SensorDefense::run() {
  if (framework.gpsAvail) {
    AP_Logger::get_singleton()->Write_CNFR(
        sensors.prevOF, sensors.currOF, sensors.prevGps, sensors.currGps,
        sensors.currAccel, sensors.nextAccel);

    // GPS Confirmations
    yawGyroGPS();
    velOFGPS();

    // Switch to next Gyroscope data
    sensors.currGyro = sensors.nextGyro;
    sensors.nextGyro.reset(sensors.currGyro.Timestamp);

    // Switch to next OF data
    sensors.prevOF = sensors.currOF;
    sensors.currOF = sensors.nextOF;
    sensors.nextOF.reset(sensors.currOF.Timestamp);

    framework.gpsAvail = false;
  }
}

void SensorDefense::log() {
  uint32_t ts = AP_HAL::micros64();
  if (ts != framework.lastUpdate) {
    framework.lastUpdate = ts;
    AP_Logger::get_singleton()->Write_SNSR(
        // ACO Data
        sensors.cOFAccel, sensors.cOF,
        // CNF Data
        sensors.currAccel, sensors.currGps, sensors.rangefinder,
        // Auxiliary Data
        sensors.mag, sensors.currGyro, sensors.bar);
  }
}

void SensorDefense::checkSensors() {
  // Initialize struct data
  if (!framework.init) {
    initialize();
  }

  // Update sensor values
  update();
  // Log them for offline analysis
  log();
  // Run sensor check
  run();
}

void SensorDefense::yawGyroGPS(){
  // If we are below minimum speed we can't perform a check here
  if(sensors.currGps.Airspeed.xy().length() < GPS_MIN_SPD){
    if(framework.gyr_gps_init){
      framework.gyr_gps_init = false;
      framework.gyr_yaw = 0;
      framework.gps_gc = 0;
    }
    return;
  } else{
    //Align Gyroscope and GPS on first valid reading
    if(!framework.gyr_gps_init){
      // Align gyroscope to GPS at first valid point
      framework.gps_gc = ToDeg(sensors.currGps.Airspeed.xy().angle());
      framework.gyr_yaw = framework.gps_gc;
      framework.gyr_gps_init = true;
    } else{
      framework.gps_gc += ToDeg(sensors.currGps.Airspeed.xy().angle() - sensors.prevGps.Airspeed.xy().angle());
      framework.gyr_yaw +=  sensors.currGyro.TrapInt.z;
      float disagreement = fabs(framework.gps_gc - framework.gyr_yaw);
      if(disagreement >= BENIGN_ANG){
        if((AP_HAL::micros64() - framework.lastAlert[0]) > 1E6){
          gcs().send_text(MAV_SEVERITY_CRITICAL, "Gyroscope and GPS Disagreement out of bounds: %f", disagreement);
          framework.lastAlert[0] = AP_HAL::micros64();
        }
      }
    }
  }
}

void SensorDefense::velOFGPS(){
  Vector2f gps_vel = sensors.currGps.Airspeed.xy();
  Vector2f of_vel = sensors.currOF.VelNE;

  //If no optical flow or rangefinder then there's no disagreement, just return
  if(!(AP_OPTICALFLOW_ENABLED && AP_RANGEFINDER_ENABLED)){
    return;
  } else{
    // The OF is in North-West frame instead of North-East.
    //    Normally I would adjust this in the sensor readings
    //    but the offline analysis already adjusts from West to
    //    East, so I'll make the fix here
    Vector2f fix_of = Vector2f(of_vel[0], -of_vel[1]);

    // Combined velocity disagreement outside allowed disagreement
    float disagreement = abs(gps_vel - fix_of).length();
    if(disagreement >= BENIGN_VEL){
      if((AP_HAL::micros64() - framework.lastAlert[1]) > 1E6){
        gcs().send_text(MAV_SEVERITY_CRITICAL, "OF and GPS Disagreement out of bounds: %f", disagreement);
        framework.lastAlert[1] = AP_HAL::micros64();
      }
    }
  }
}