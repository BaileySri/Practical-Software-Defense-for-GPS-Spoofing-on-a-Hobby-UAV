#pragma once

// Libraries
#include "../AP_Logger/AP_Logger.h"
#include "AP_HAL/system.h"
#include "sensor_defense_structs.h"
#include <cstdint>

class SensorDefense {
public:
  SensorDefense() {}
  ~SensorDefense() {}

  //----   Helper Functions  ----//
  static bool confirm(const float a, const float a_err, const float b,
                      const float b_err, bool);
  static Vector3f abs(const Vector3f &x);
  static Vector2f abs(const Vector2f &x);

  // Sensor Data
  struct {
    Accel currAccel; // Acceleration data within GPS update frame
    Accel nextAccel; // Acceleration data that happens after expected GPS update
    Gyro currGyro;   // Roll,Pitch,Yaw (Rates in radians/sec)
    Gyro nextGyro;   // Roll,Pitch,Yaw (Rates in radians/sec)
    GPS currGps;     // Most recent GPS reading
    GPS prevGps;     // Previous GPS reading
    OF prevOF;       // Previous Optical Flow Data
    OF currOF;       // Current Optical Flow Data
    OF nextOF;       // Optical Flow Data that occurs after expected GPS update
    RF rangefinder;  // Rangefinder Data

    // Data for Accelerometer confirming to Optical Flow
    Accel pOFAccel;
    Accel cOFAccel;
    Accel nOFAccel;
    OF pOF;
    OF cOF;

    // ETC, Sensors I'm recording for auxiliary use
    Mag mag; // Magnetometer instantaneous reading
    Baro bar; //Barometer readings
  } sensors;

  // Data book keeping
  struct {
    bool init = false;   // Initialized Structs
    bool gpsAvail;       // GPS Data has been updated
    bool ofAvail;        // OF Data has been updated
    bool gyr_gps_init;   // GPS has met minimum speed before
    float gyr_yaw = 0;   // Gyroscope yaw for Gyro GPS check
    float gps_gc = 0;    // GPS gc for Gyro GPS check
    uint32_t lastAlert[2] = {0, 0}; //Avoid spamming alerts so only send 1 for each check every 1 second.
    uint64_t lastUpdate; // Last time an SNS log was written
  } framework;

  //----General Functions----//
  void initialize();
  void update();
  void run();
  void log();
  void checkSensors();

  //----Sensor Check Functions----//
  // Check disagreement in yaw of GPS and Gyro
  void yawGyroGPS();
  // Check disagreement in velocity of GPS and OF
  void velOFGPS();


private:
};