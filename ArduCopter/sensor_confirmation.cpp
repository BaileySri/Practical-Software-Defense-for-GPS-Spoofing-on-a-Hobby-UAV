//PADLOCK
//Sensor Confirmation source code
//1/27/2021

#include "Copter.h"

//Function delcarations
void initialize();
void update();
bool confirm();
void recover();
void debug();

//Structs for easier naming
struct NED{
    float North;
    float East;
    float Down;

    NED& zero(){
        North = 0.0;
        East = 0.0;
        Down = 0.0;
        return *this;
    }

    float magnitude(){
        return(sqrt(North*North + East*East + Down*Down));
    }

    NED& operator-(const NED &rhs){
        North -= rhs.North;
        East -= rhs.East;
        Down -= rhs.Down;
        return *this;
    }

    NED& operator+(const Vector3f &rhs){
        North += rhs.x;
        East += rhs.y;
        Down += rhs.z;
        return *this;
    }

    NED& operator+(const NED &rhs){
        North += rhs.North;
        East += rhs.East;
        Down += rhs.Down;
        return *this;
    }

    NED& operator=(const Vector3f &rhs){
        North = rhs.x;
        East = rhs.y;
        Down = rhs.z;
        return *this;
    }

    NED& operator=(const NED &rhs){
        North = rhs.North;
        East = rhs.East;
        Down = rhs.Down;
        return *this;
    }

    NED& operator*(const float &A){
        North *= A;
        East *= A;
        Down *= A;
        return *this;
    }
};

struct Accel{
    NED Readings; //m/s/s
    NED Velocity; //m/s
    NED Error; //m/s
    uint32_t Timestamp; //ms
    uint32_t DeltaT; //ms (Time frame for accel data)

    uint32_t update(const AP_InertialSensor *frontend){
        if(DeltaT >= static_cast<uint32_t>(200)){
            //This Accelerometer already has enough data to match GPS
            return DeltaT;
        }
        if(Timestamp != frontend->get_last_update_usec() / 1000){
            NED newReading;
            newReading = frontend->get_accel();
            //Trapezoidal Integration
            Velocity.North += ((newReading.North + Readings.North) / 2) * frontend->get_delta_time();
            Velocity.East += ((newReading.East + Readings.East) / 2) * frontend->get_delta_time();
            Velocity.Down += ((newReading.Down + Readings.Down) / 2) * frontend->get_delta_time();
            Readings = newReading;
            DeltaT += (frontend->get_last_update_usec() / 1000) - Timestamp;
            Timestamp = frontend->get_last_update_usec() / 1000;
        }
        return DeltaT;
    }

    void reset(){
        Readings.zero();
        Velocity.zero();
        Error.zero();
        Timestamp = 0;
        DeltaT = 0;
    }

    Accel& operator=(const Accel &rhs){
        Readings = rhs.Readings;
        Velocity = rhs.Velocity;
        Timestamp = rhs.Timestamp;
        DeltaT = rhs.DeltaT;
        Error = rhs.Error;
        return *this;
    }
};

struct Gyro{
    float Roll;
    float Pitch;
    float Yaw;

    Gyro& operator=(const Vector3f &rhs){
        Roll = rhs.x;
        Pitch = rhs.y;
        Yaw = rhs.z;
        return *this;
    }

    void reset(){
        Roll = 0;
        Pitch = 0;
        Yaw = 0;
    }
};

struct Mag{
    float x; //Milligauss
    float y; //Milligauss
    float z; //Milligauss

    Mag& operator=(const Vector3f &rhs){
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    void reset(){
        x = 0;
        y = 0;
        z = 0;
    }
};

struct GPS{
    int32_t Latitude; //Degrees 1E-7
    int32_t Longitude; //Degrees 1E-7
    int32_t Altitude;
    NED Airspeed; //m/s
    float Yaw;
    float Yaw_Error;
    uint32_t Timestamp; //ms

    void update_location(const Location &rhs){
        Latitude = rhs.lat;
        Longitude = rhs.lng;
        Altitude = rhs.alt;
    }

    bool update(const AP_GPS *rhs){
        if(rhs->last_fix_time_ms() != Timestamp){
            update_location(rhs->location());
            Airspeed = rhs->velocity();
            if(!rhs->gps_yaw_deg(Yaw, Yaw_Error)){
                Yaw = -1;
                Yaw_Error = -1;
            }
            Timestamp = rhs->last_fix_time_ms();
            return true;
        }
        return false;
    }

    void reset(){
        Latitude = 0;
        Longitude = 0;
        Altitude = 0;
        Airspeed.zero();
        Yaw = -1;
        Yaw_Error = -1;
        Timestamp = 0;
    }
};

//Sensor Data
static struct{
    Accel currAccel; //North,East,Down (Acceleration in m/s/s)
    Accel nextAccel; //North,East,Down (Acceleration in m/s/s)
    Gyro gyro; //Roll,Pitch,Yaw (Rates in radians/sec)
    /* A Note for the future
        The mag orientation (+/-) and axis descriptions are provided by part.
        I couldn't find any documentation in source code but it is worth
        understanding that the pre-flight callibration process most likely
        allows for ignorance of the compass orientation as it will just be
        offset so just assume the mag recorded is correctly set in x,y, and z.
    */
    Mag mag; //Mag field about the X,Y, and Z axis (Milligauss)
    GPS gps; //Latitude,Longitude,Altitude (Degrees 1E-7 and Altitude in cm)
} sensors;

//Confirmation Data
static struct{
    bool init = false; //Initialized Structs
    bool gpsAvail; //GPS Data has been updated
    NED lastGPS; //m/s/s (average acceleration observed by GPS)
    NED lastAcc; //m/s/s (average acceleration observed by Accelerometer)
} framework;

void Copter::sensor_confirmation()
{
        if(!framework.init){
            initialize();
        }
        update();
        if(!confirm()){
            recover();
        }
}

void initialize(){
    sensors.currAccel.reset();
    sensors.nextAccel.reset();
    sensors.mag.reset();
    sensors.gps.reset();
    sensors.gyro.reset();
    framework.lastGPS.zero();
    framework.lastAcc.zero();
    framework.gpsAvail = false;
    framework.init = true;
}

void update(){
    //IMU Sensors
    if(sensors.currAccel.update(AP_InertialSensor::get_singleton()) >= static_cast<uint32_t>(200)){
        sensors.nextAccel.update(AP_InertialSensor::get_singleton());
    }
    sensors.gyro = AP_InertialSensor::get_singleton()->get_gyro();
    sensors.mag = AP::compass().get_field();

    //GPS
    framework.gpsAvail = sensors.gps.update(AP::gps().get_singleton());
}

bool confirm(){
    if(framework.gpsAvail){
        debug();
        if((sensors.gps.Airspeed - framework.lastGPS).magnitude() - \
            (sensors.currAccel.Velocity - framework.lastAcc).magnitude() > 0.3){
            gcs().send_text(MAV_SEVERITY_WARNING,"Confirmation Error");
            gcs().send_text(MAV_SEVERITY_WARNING,"GPS: %f | Acc: %f",
                            (sensors.gps.Airspeed - framework.lastGPS).magnitude(),
                            (sensors.currAccel.Velocity - framework.lastAcc).magnitude());
        }
        framework.lastGPS = sensors.gps.Airspeed;
        framework.lastAcc = sensors.currAccel.Velocity;

        //Accumulate velocity
        sensors.nextAccel.Velocity = sensors.currAccel.Velocity + sensors.nextAccel.Velocity;

        // sensors.nextAccel.Error = (sensors.gps.Airspeed - sensors.currAccel.Velocity) + sensors.currAccel.Error;
        // //Clamp Velocity to GPS
        // sensors.nextAccel.Velocity = sensors.nextAccel.Velocity + sensors.nextAccel.Error;

        //Switch to next Accel data
        sensors.currAccel = sensors.nextAccel;
        sensors.nextAccel.reset();

        //Retain timestamp for consistency
        sensors.nextAccel.Timestamp = sensors.currAccel.Timestamp;

        framework.gpsAvail = false;
    }
    return true;
}

void recover(){
    gcs().send_text(MAV_SEVERITY_WARNING,"Recovery Error");
}

void debug(){
    gcs().send_text(MAV_SEVERITY_INFO,"CURRENT (%u) DeltaT: %u | Vel: (%f, %f, %f) | GPS: (%f, %f, %f)",\
                    sensors.currAccel.Timestamp,
                    sensors.currAccel.DeltaT,
                    sensors.currAccel.Velocity.North,
                    sensors.currAccel.Velocity.East,
                    sensors.currAccel.Velocity.Down,
                    sensors.gps.Airspeed.North,
                    sensors.gps.Airspeed.East,
                    sensors.gps.Airspeed.Down);
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
        // for logging
        uint64_t timestamp = AP_HAL::micros64();
        FILE *out_file = fopen("sim_log.csv", "a");
        fprintf(out_file, "%" PRIu64 ", %f, %f, %f, %f, %f, %f\n", \
                timestamp,
                framework.accel_vel.North,
                framework.accel_vel.East,
                framework.accel_vel.Down,
                sensors.gps.Airspeed.North,
                sensors.gps.Airspeed.East,
                sensors.gps.Airspeed.Down);
        fclose(out_file);
    #endif
}