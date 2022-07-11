#pragma once

//Libraries
#include <cstdint>
#include "../AP_Logger/AP_Logger.h"
#include "sensor_confirmation_structs.h"

class SensorConfirmation{
    public:
        SensorConfirmation()
        {}
        ~SensorConfirmation()
        {}
        
                //----   Helper Functions  ----//
        static bool confirm(const float a, const float a_err, const float b, const float b_err, bool);
        static Vector3f abs(const Vector3f &x);
        static Vector2f abs(const Vector2f &x);

                //Sensor Data
        struct
        {
            Accel currAccel; //Acceleration data within GPS update frame
            Accel nextAccel; //Acceleration data that happens after expected GPS update
            Gyro currGyro;   //Roll,Pitch,Yaw (Rates in radians/sec)
            Gyro nextGyro;   //Roll,Pitch,Yaw (Rates in radians/sec)
            GPS currGps;     //Most recent GPS reading
            GPS prevGps;     //Previous GPS reading
            OF prevOF;       //Previous Optical Flow Data
            OF currOF;       //Current Optical Flow Data
            OF nextOF;       //Optical Flow Data that occurs after expected GPS update
            RF rangefinder;  //Rangefinder Data

            //Data for Accelerometer confirming to Optical Flow
            Accel pOFAccel;
            Accel cOFAccel;
            Accel nOFAccel;
            OF pOF;
            OF cOF;
        } sensors;

        //Confirmation Data
        struct
        {
            bool init = false; //Initialized Structs
            bool gpsAvail;     //GPS Data has been updated
            bool ofAvail;      // OF Data has been updated
        } framework;

        //----General Functions----//
        void initialize();
        void update();
        bool run();
        void alert();
        void log();
        void confirmation();
        // The max velocity allowed before confirmation fails
        float NetGpsLimit() const;
        // The max flowrate with bodyrate allowed before confirmation fails
        float NetOFLimit() const;
        // Depending on selected mode return the limited GPS value
        //  1: Acc
        //  2: OF
        //  3: Mag 
        //  4: Acc/OF
        //  5: Acc/Mag
        //  6: OF/Mag
        float GCGpsLimit(const int mode) const;

        //----Confirmation Functions----//
        // Confirm change in velocity of GPS and Accelerometer
        bool AccGPS();
        // Confirm change in velocity of Accelerometer and OF
        bool AccOF();
        // Confirm velocity of GPS and Optical Flow Sensor
        bool GpsOF();
        // Confirm ground course based on GPS movement and Magnetometer heading
        bool GpsMagGC();
        // Confirm ground course based on GPS movement and Optical Flow movement with Magnetometer for rotation
        bool GpsOFGC();
        // Confirm ground course based on GPS movement and Accelerometer movement with Magnetometer for rotation
        bool AccGpsGC();
        // Confirm ground course based on Accelerometer and Optical Flow movement with Magnetometer for rotation
        bool AccOFGC();
    private:
};