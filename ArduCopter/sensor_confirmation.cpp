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

    NED& zero(){
        North = 0.0;
        East = 0.0;
        Down = 0.0;
        return *this;
    }

    NED& operator-(const NED rhs){
        North -= rhs.North;
        East -= rhs.East;
        Down -= rhs.Down;
        return *this;
    }

    NED& operator+(const Vector3f rhs){
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

    NED& operator=(const Vector3f rhs){
        North = rhs.x;
        East = rhs.y;
        Down = rhs.z;
        return *this;
    }

    NED& operator*(const float A){
        North *= A;
        East *= A;
        Down *= A;
        return *this;
    }
};

struct Accel{
    NED readings; //m/s/s
    uint32_t Timestamp; //ms
    NED bias; //m/s/s
    bool biased = false;

    bool update(const AP_InertialSensor *frontend){
        if(!biased && AP_HAL::micros64() > 30000000){
            bias = readings;
            biased = true;
        }
        if(Timestamp != frontend->get_last_update_usec() / 1000){
            readings = bias * (-1.0) + frontend->get_accel();
            return true;
        }
        return false;
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
};

struct GPS{
    int32_t Latitude; //Degrees 1E-7
    int32_t Longitude; //Degrees 1E-7
    int32_t Altitude;
    NED Airspeed; //m/s
    float Yaw;
    float Yaw_Error;
    uint32_t Timestamp; //ms
    float DeltaT; //s

    void update_location(const Location &rhs){
        Latitude = rhs.lat;
        Longitude = rhs.lng;
        Altitude = rhs.alt;
    }

    bool update(const AP_GPS *rhs){
        if(rhs->last_fix_time_ms() != Timestamp){
            DeltaT = (rhs->last_fix_time_ms() - Timestamp) / 1000.0;
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

//Confirmation Data
static struct{
    NED accel_accum; //m/s/s
    NED accel_vel; //m/s
    uint16_t accel_count;
    bool gps_avail;
} framework;

void Copter::sensor_confirmation()
{
    update();
    if(!confirm()){
        recover();
    }
}

void update(){
    //IMU Sensors
    if(sensors.accel.update(AP_InertialSensor::get_singleton())){
        framework.accel_count++;
        framework.accel_accum = framework.accel_accum + sensors.accel.readings;
    }
    sensors.gyro = AP_InertialSensor::get_singleton()->get_gyro();
    sensors.mag = AP::compass().get_field();

    //GPS
    if(sensors.gps.update(AP::gps().get_singleton())){
        framework.gps_avail = true;
    }
}

bool confirm(){
    static uint32_t delay = 3000000;
    uint32_t now = AP_HAL::micros64();
    if(sensors.accel.biased && framework.gps_avail && framework.accel_count > 0 && (now - delay) > 3000000){
        delay = now;
        framework.accel_vel = framework.accel_vel + ((framework.accel_accum * (1.0/framework.accel_count)) * sensors.gps.DeltaT);
        gcs().send_text(MAV_SEVERITY_WARNING,"Accel(N,E,D): %f, %f, %f",
                        framework.accel_vel.North,
                        framework.accel_vel.East,
                        framework.accel_vel.Down);
        gcs().send_text(MAV_SEVERITY_WARNING,"Gps(N,E,D): %f, %f, %f",
                        sensors.gps.Airspeed.North,
                        sensors.gps.Airspeed.East,
                        sensors.gps.Airspeed.Down);
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 1
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
        framework.accel_accum.zero();
        framework.gps_avail = false;
        framework.accel_count = 0;
        framework.accel_vel = sensors.gps.Airspeed;
    }
    return true;
}

void recover(){
}

