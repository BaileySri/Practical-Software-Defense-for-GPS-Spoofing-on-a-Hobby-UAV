/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL/system.h"
#include "AP_RangeFinder.h"
#include "AP_RangeFinder/AP_RangeFinder_Params.h"
#include "AP_RangeFinder_Backend.h"
//PADLOCK
// Gets the sign for the attack
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
#include "GCS_MAVLink/GCS.h"
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_Backend::AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        state(_state),
		params(_params)
{
    _backend_type = type();
}

MAV_DISTANCE_SENSOR AP_RangeFinder_Backend::get_mav_distance_sensor_type() const {
    if (type() == RangeFinder::Type::NONE) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return _get_mav_distance_sensor_type();
}

RangeFinder::Status AP_RangeFinder_Backend::status() const {
    if (type() == RangeFinder::Type::NONE) {
        // turned off at runtime?
        return RangeFinder::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_RangeFinder_Backend::has_data() const {
    return ((state.status != RangeFinder::Status::NotConnected) &&
            (state.status != RangeFinder::Status::NoData));
}

// update status based on distance measurement
void AP_RangeFinder_Backend::update_status()
{
    //PADLOCK
    static bool atk_started = false;
    static bool final_msg = false;
    static float rf_init = 0; //m
    static float time = 0; //s
    static float time_elapsed = 0; //s
    bool attack = params.CHANNEL > 0 ? (RC_Channels::rc_channel(params.CHANNEL - 1)->get_radio_in() > 1600) || (params.ATK == 1) : (params.ATK == 1);
    static float t1 = 0; //s

    state.real_distance_m = state.distance_m; //m
    if( !atk_started && attack ) {
        rf_init = state.distance_m;
        atk_started = true;
        time = AP_HAL::micros64()/1.0E6;
        if( fabs(params.RATE) < 0.0001 ){
            // Assume such a small RATE is 0
            t1 = 0;
        } else{
            t1 = sqrtf((abs(params.DIST) / 1.0E2) / params.RATE);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "RF: Attack Started, Time Expected %.2f s", t1*2);
    }
    if( atk_started && attack ) {
        if( params.SIMPLE_ATTACK == 1 ){
            state.distance_m = rf_init + (params.DIST / 100.0f);
            if( !final_msg ){
                gcs().send_text(MAV_SEVERITY_INFO, "RF: One-Step Offset %.2f m", params.DIST/1E2);
                final_msg = true;
            }
        } else{
            time_elapsed += ((AP_HAL::micros64()/1.0E6) - time);
            const int sign = sgn(params.DIST.get());
            if( time_elapsed <= t1 ){ // t <= t1
                state.distance_m = rf_init + (0.5)*sign*(params.RATE)*sq(time_elapsed); // d_0 + 1/2 * a * t^2
            } else if(time_elapsed <= (t1 * 2)){ // t1 < t <= t2
                state.distance_m = rf_init + (0.5)*sign*(params.RATE)*sq(t1) + // d_0 + d_1
                                   sign*(params.RATE)*(t1)*(time_elapsed - t1) - // v_1*t
                                   (0.5)*sign*(params.RATE) * sq(time_elapsed - t1); // 1/2 * a * (t-t1)^2 
            } else{
                state.distance_m = rf_init + params.DIST/1.0E2;
            }
            if( !final_msg ){
                if( (time_elapsed) < (t1 * 2) ){
                gcs().send_text(MAV_SEVERITY_INFO, "RF: Time Elapsed (%.2f/%.2f)s, Offset %.2f m",
                                time_elapsed, t1*2, state.distance_m - state.real_distance_m);
                } else{
                    gcs().send_text(MAV_SEVERITY_INFO, "RF: Finished offsetting %.2f m", state.distance_m - state.real_distance_m);
                    final_msg = true;
                }
            } 
        }
    }

    // check distance
    if (state.distance_m > max_distance_cm() * 0.01f) {
        set_status(RangeFinder::Status::OutOfRangeHigh);
    } else if (state.distance_m < min_distance_cm() * 0.01f) {
        set_status(RangeFinder::Status::OutOfRangeLow);
    } else {
        set_status(RangeFinder::Status::Good);
    }
}

// set status and update valid count
void AP_RangeFinder_Backend::set_status(RangeFinder::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == RangeFinder::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

