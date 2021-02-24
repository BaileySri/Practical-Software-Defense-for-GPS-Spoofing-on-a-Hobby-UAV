//PADLOCK
//Sensor Confirmation source code
//2/22/2021

//Constant declarations
const unsigned int DELAY = 45000000U; //Start-Up delay to allow sensors to settle
const unsigned int BIAS_DELAY = DELAY + 30000000U; //Delay for biasing
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

//Structs for easier naming
struct NED{
    float North;
    float East;
    float Down;

    constexpr NED( const NED& rhs ) = default;

    NED(){
        North = 0;
        East = 0;
        Down = 0;
    }

    NED& zero(){
        North = 0.0;
        East = 0.0;
        Down = 0.0;
        return *this;
    }

    float magnitude() const {
        return(sqrt(North*North + East*East + Down*Down));
    }

    NED operator-(const NED &rhs) const {
        NED ned;
        ned.North = this->North - rhs.North;
        ned.East = this->East - rhs.East;
        ned.Down = this->Down - rhs.Down;
        return ned;
    }

    NED operator+(const Vector3f &rhs) const {
        NED ned;
        ned.North = this->North + rhs.x;
        ned.East = this->East + rhs.y;
        ned.Down = this->Down + rhs.z;
        return ned;
    }

    NED operator+(const NED &rhs) const {
        NED ned;
        ned.North = this->North + rhs.North;
        ned.East = this->East + rhs.East;
        ned.Down = this->Down + rhs.Down;
        return ned;
    }

    NED operator*(const float &A) const {
        NED ned;
        ned.North = this->North * A;
        ned.East = this->East * A;
        ned.Down = this->Down * A;
        return ned;
    }

    NED operator/(const float &A) const {
        NED ned;
        ned.North = this->North / A;
        ned.East = this->East / A;
        ned.Down = this->Down / A;
        return ned;
    }

    NED& operator=(const Vector3f &rhs) {
        North = rhs.x;
        East = rhs.y;
        Down = rhs.z;
        return *this;
    }

    NED& operator=(const NED &rhs) {
        North = rhs.North;
        East = rhs.East;
        Down = rhs.Down;
        return *this;
    }

    NED& operator+=(const NED &rhs) {
        this->North += rhs.North;
        this->East += rhs.East;
        this->Down += rhs.Down;
        return *this;
    }

};

struct Accel{
    NED Reading; //m/s/s
    NED Velocity; //m/s
    float Noise; //m/s/s
    uint32_t Timestamp; //ms

    bool update(const AP_InertialSensor *frontend, const NED& bias){
        if(Timestamp != frontend->get_last_update_usec() / 1000){
            NED newReading;
            newReading = (bias*static_cast<float>(-1)) + frontend->get_accel();
            //Trapezoidal Integration
            Velocity += ((newReading + Reading) / 2.0F) * frontend->get_delta_time();
            Reading = newReading;
            Noise += ACC_NOISE * sqrt( 1E-3/( ( frontend->get_last_update_usec()/1000 ) - ( Timestamp ) ) );
            Timestamp = frontend->get_last_update_usec() / 1000;
            return true;
        }
        return false;
    }

    void reset(){
        Reading.zero();
        Velocity.zero();
        Noise = 0;
        Timestamp = 0;
    }

    Accel& operator=(const Accel &rhs){
        Reading = rhs.Reading;
        Velocity = rhs.Velocity;
        Timestamp = rhs.Timestamp;
        Noise = rhs.Noise;
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
    NED bias; //m/s/s (Bias of accelerometer, after DELAY but before BIAS)
    uint32_t bias_count; //int (Number of measurements taken during bias)
    uint8_t biased; //0,1,2 (0:No Biasing, 1:Currently Biasing, 2:Done Biasing)
} framework;

void Copter::sensor_confirmation()
{
        //Initialize struct data
        if(!framework.init){
            initialize();
        }

        //Bias the accelerometer readings and reset used structs
        if(framework.biased == 1 && AP_HAL::micros64() >= BIAS_DELAY){
            framework.bias = framework.bias / static_cast<float>(framework.bias_count);
            sensors.currAccel.reset();
            if(framework.biased == 1){
                gcs().send_text(MAV_SEVERITY_INFO,"Sensors Biased");
                framework.biased = 2;
            }
        }
        
        if(AP_HAL::micros64() >= BIAS_DELAY){
            //Update sensor values 
            update();
            
            //If confirmation returns False, Recover
            //If confirmation returns True, Nothing
            if(!confirm()){
                recover();
            }
        } else if(AP_HAL::micros64() >= DELAY){
            if(framework.biased == 0){
                gcs().send_text(MAV_SEVERITY_INFO,"Start Biasing");
                framework.biased = 1;
            }
            //After 45 seconds start collecting biasing data
            bias();
            framework.bias += sensors.currAccel.Reading;
            framework.bias_count++;
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
    framework.biased = 0;
    framework.bias.zero();
    framework.bias_count = 0;
    framework.lastConfirm = 0;
}

void bias(){
    //During this time I only want raw measurements
    sensors.currAccel.Reading = AP_InertialSensor::get_singleton()->get_accel();
}

void update(){
    //IMU Sensors
    sensors.currAccel.update(AP_InertialSensor::get_singleton(), framework.bias);
    sensors.gyro = AP_InertialSensor::get_singleton()->get_gyro();
    sensors.mag = AP::compass().get_field();

    //GPS
    framework.gpsAvail = sensors.gps.update(AP::gps().get_singleton());
}

bool confirm(){
    if(framework.gpsAvail){
        debug();

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
        sensors.nextAccel.Velocity = sensors.currAccel.Velocity + sensors.nextAccel.Velocity;
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
    gcs().send_text(MAV_SEVERITY_WARNING,"Recovery");
}

void debug(){
    // gcs().send_text(MAV_SEVERITY_INFO,"CURRENT [%lu] ACC: (%f, %f, %f) | GPS: (%f, %f, %f)",
    //                 AP_HAL::micros64(),
    //                 sensors.currAccel.Velocity.North,
    //                 sensors.currAccel.Velocity.East,
    //                 sensors.currAccel.Velocity.Down,
    //                 sensors.gps.Airspeed.North,
    //                 sensors.gps.Airspeed.East,
    //                 sensors.gps.Airspeed.Down);
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
    float gps_change = (sensors.gps.Airspeed - framework.lastGPS).magnitude();
    float acc_change = (sensors.currAccel.Velocity - framework.lastAcc).magnitude(); 
    if(gps_change >= acc_change){
        return((acc_change + acc_velError) - (gps_change - sensors.gps.Sacc) >= 0);
    } else{
        return((gps_change + sensors.gps.Sacc) - (acc_change - acc_velError) >= 0);
    }
    // if((sensors.gps.Airspeed - framework.lastGPS).magnitude() - (sensors.currAccel.Velocity - framework.lastAcc).magnitude() >= 0){
    //     gcs().send_text(MAV_SEVERITY_WARNING,"Confirmation Error");
    //     gcs().send_text(MAV_SEVERITY_WARNING,"GPS: %f | Acc: %f",
    //                     (sensors.gps.Airspeed - framework.lastGPS).magnitude(),
    //                     (sensors.currAccel.Velocity - framework.lastAcc).magnitude());
    // }
}

bool confirmAlt(){
    //TODO: Confirmation between Barometer, GPS, and Acc
    return true;
}