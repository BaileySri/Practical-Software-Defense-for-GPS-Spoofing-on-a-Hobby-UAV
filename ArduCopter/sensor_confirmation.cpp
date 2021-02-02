//PADLOCK
//Sensor Confirmation source code
//1/27/2021

#include "Copter.h"

//Function delcarations
void update();
bool confirm();
void recover();

//Structs for easier naming
struct NED{
    float North;
    float East;
    float Down;

    NED& operator=(const Vector3f rhs){
        North = rhs.x;
        East = rhs.y;
        Down = rhs.z;
        return *this;
    }
};

struct Accel{
    NED readings;
    
    Accel& operator=(const Vector3f rhs){
        readings = rhs;
        return *this;
    }
};

struct Gyro{
    float Roll;
    float Pitch;
    float Yaw;

    Gyro& operator=(const Vector3f rhs){
        Roll = rhs.x;
        Pitch = rhs.y;
        Yaw = rhs.z;
        return *this;
    }
};

struct Mag{
    float x;
    float y;
    float z;

    Mag& operator=(const Vector3f rhs){
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }
};

struct GPS{
    int32_t Latitude;
    int32_t Longitude;
    int32_t Altitude;
    NED Airspeed;
    float Yaw;
    float Yaw_Error;
    uint32_t Timestamp;

    void update_location(const Location rhs){
        Latitude = rhs.lat;
        Longitude = rhs.lng;
        Altitude = rhs.alt;
    }

    GPS& update(const AP_GPS *rhs){
        update_location(rhs->location());
        Airspeed = rhs->velocity();
        if(!rhs->gps_yaw_deg(Yaw, Yaw_Error)){
            Yaw = -1;
            Yaw_Error = -1;
        }
        return *this;
    }
};

//Sensor Data
static struct{
    Accel accel; //North,East,Down (Acceleration in m/s/s)
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

void Copter::sensor_confirmation()
{
    update();
    if(!confirm()){
        recover();
    }
}

void update(){
    //IMU Sensors
    sensors.accel = AP_InertialSensor::get_singleton()->get_accel();
    sensors.gyro = AP_InertialSensor::get_singleton()->get_gyro();
    sensors.mag = AP::compass().get_field();

    //GPS
    sensors.gps.update(AP::gps().get_singleton());
}

bool confirm(){
    return true;
}

void recover(){
}

