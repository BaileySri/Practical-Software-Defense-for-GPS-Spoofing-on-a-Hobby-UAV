//PADLOCK
//Sensor Confirmation source code
//2/22/2021

//Constant declarations
const unsigned int DELAY = 45000000U; //Start-Up delay to allow sensors to settle
const float ACC_NOISE = 0.3;//1.471E-3; // (m/s/s)/(sqrt(Hz)) Accelerometer noise (As defined in LSM303D Datasheet)

#include "Copter.h"

//----Function delcarations----//
void initialize();
void bias();
void update();
bool confirm();
void recover();
void debug();
//----Specific Confirmations----//
bool confirmVel();
bool confirmAlt();

typedef Vector3f NED;
typedef Vector3f BF;

struct Accel{
    NED Readings; //m/s/s
    NED Velocity; //m/s
    float Noise; //m/s/s
    uint32_t Timestamp; //ms

    bool update(const AP_InertialSensor *frontend){
        if(Timestamp != frontend->get_last_update_usec() / 1000){
            NED newReading = AP_AHRS_DCM::get_singleton()->body_to_earth(frontend->get_accel());
            //Trapezoidal Integration
            Velocity += ((newReading + Readings) / 2.0F) * frontend->get_delta_time();
            Readings = newReading;
            Noise += ACC_NOISE * sqrtf( 1E-3/( ( frontend->get_last_update_usec()/1000 ) - ( Timestamp ) ) );
            Timestamp = frontend->get_last_update_usec() / 1000;
            return true;
        }
        return false;
    }

    void reset(){
        Readings.zero();
        Velocity.zero();
        Noise = 0;
        Timestamp = 0;
    }

    Accel& operator=(const Accel &rhs){
        Readings = rhs.Readings;
        Velocity = rhs.Velocity;
        Timestamp = rhs.Timestamp;
        Noise = rhs.Noise;
        return *this;
    }
};

struct Gyro{
    BF Readings; //x = Roll, y = Pitch, z = Yaw

    void reset(){
        Readings.zero();
    }
};

struct Mag{
    Vector3f Readings; //milligauss

    void reset(){
        Readings.zero();
    }
};

struct GPS{
    int32_t Latitude; //Degrees 1E-7
    int32_t Longitude; //Degrees 1E-7
    int32_t Altitude;
    NED Airspeed; //m/s
    float Sacc; //m/s GPS 3D RMS Speed Accuracy
    float Hacc; //m GPS 3D RMS Horizontal Position Accuracy
    float Vacc; //m GPS 3D RMS Vertical Position Accuracy 
    float Yaw;
    float Yaw_Error;
    uint32_t Timestamp; //ms
    uint32_t lastTimestamp; //ms

    void update_location(const Location &rhs){
        Latitude = rhs.lat;
        Longitude = rhs.lng;
        Altitude = rhs.alt;
    }

    bool update(const AP_GPS *frontend){
        if(frontend->last_fix_time_ms() != Timestamp){
            update_location(frontend->location());
            Airspeed = frontend->velocity();
            if(!frontend->gps_yaw_deg(Yaw, Yaw_Error)){
                Yaw = -1;
                Yaw_Error = -1;
            }
            lastTimestamp = Timestamp;
            Timestamp = frontend->last_fix_time_ms();
            if(!frontend->speed_accuracy(Sacc)){
                Sacc = -1;
            }
            if(!frontend->vertical_accuracy(Vacc)){
                Vacc = -1;
            }
            if(!frontend->horizontal_accuracy(Hacc)){
                Hacc = -1;
            }
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
    uint32_t lastConfirm; //ms (Time that the last confirmation occurred)
    NED lastGPS; //m/s (Velocity provided by last GPS Sensor readings)
    NED lastAcc; //m/s (Velocity calculated from previous Acc Sensor data)
} framework;

void Copter::sensor_confirmation()
{
    //Initialize struct data
    if(!framework.init){
        initialize();
    }
    if(AP_HAL::micros64() >= DELAY){
        //Update sensor values 
        update();
            
        //If confirmation returns False, Recover
        //If confirmation returns True, Nothing
        if(!confirm()){
            recover();
        }
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
    framework.lastConfirm = 0;
}

void update(){
    //IMU Sensors
    if((sensors.currAccel.Timestamp - sensors.gps.Timestamp) >= 200){
        sensors.nextAccel.update(AP_InertialSensor::get_singleton());
    } else{
        sensors.currAccel.update(AP_InertialSensor::get_singleton());
    }
    sensors.gyro.Readings = AP_InertialSensor::get_singleton()->get_gyro();
    sensors.mag.Readings = AP::compass().get_field();

    //GPS
    framework.gpsAvail = sensors.gps.update(AP::gps().get_singleton());
}

bool confirm(){
    if(framework.gpsAvail){
        #if CONFIG_HAL_BOARD == HAL_BOARD_SITL && 0
            debug();
        #endif

        if(!confirmVel()){
            gcs().send_text(MAV_SEVERITY_WARNING,"Velocity outside of error ranges.");
        }
        if(!confirmAlt()){
            gcs().send_text(MAV_SEVERITY_WARNING,"Altitude outside of error ranges.");            
        }
        
        //Record previous measurements
        framework.lastGPS = sensors.gps.Airspeed;
        framework.lastAcc = sensors.currAccel.Velocity;
        framework.lastConfirm = sensors.currAccel.Timestamp;
        //Accumulate velocity
        sensors.nextAccel.Velocity += sensors.currAccel.Velocity;
        //Switch to next Accel data
        sensors.currAccel = sensors.nextAccel;
        sensors.nextAccel.reset();
        //Retain timestamp for consistency

        framework.gpsAvail = false;
    }
    return true;
}

void recover(){
    gcs().send_text(MAV_SEVERITY_WARNING,"Recovery");
}

void debug(){
    gcs().send_text(MAV_SEVERITY_INFO,"CURRENT [%lu] currAcc: %u | nextAcc: %u | GPS: %u",
                    AP_HAL::micros64(),
                    sensors.currAccel.Timestamp,
                    sensors.nextAccel.Timestamp,
                    sensors.gps.Timestamp);
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

bool confirmVel(){
    //Total possible error resulting from accelerometer noise
    float acc_velError = ((sensors.currAccel.Timestamp - framework.lastConfirm) * sensors.currAccel.Noise * 3) / 1000; //Multiplied by 3 to account for noise in 3-Axis
    //Compare the change in Airspeed from GPS to change in
    //Dead Reckoning velocity from Accelerometer
    float gps_change = (sensors.gps.Airspeed - framework.lastGPS).length();
    float acc_change = (sensors.currAccel.Velocity - framework.lastAcc).length(); 
    if(gps_change >= acc_change){
        return((acc_change + acc_velError) - (gps_change - sensors.gps.Sacc) >= 0);
    } else{
        return((gps_change + sensors.gps.Sacc) - (acc_change - acc_velError) >= 0);
    }
}

bool confirmAlt(){
    //TODO: Confirmation between Barometer, GPS, and Acc
    return true;
}