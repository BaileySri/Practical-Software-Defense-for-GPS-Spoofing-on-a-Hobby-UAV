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

#include "AP_OpticalFlow.h"
//PADLOCK
// Needed for advanced attacker
#include <SensorConfirmation/sensor_confirmation.h>
#include <AP_Vehicle/AP_Vehicle.h>

#if AP_OPTICALFLOW_ENABLED

extern const AP_HAL::HAL& hal;

OpticalFlow_backend::OpticalFlow_backend(OpticalFlow &_frontend) :
    frontend(_frontend)
{
}

OpticalFlow_backend::~OpticalFlow_backend(void)
{
}

// update the frontend
void OpticalFlow_backend::_update_frontend(const struct OpticalFlow::OpticalFlow_state &state)
{
    //PADLOCK
    if(frontend._pdlk_attack_enable == 1){
        if(frontend._pdlk_adv_atk == 1){
            // Advanced Attacker only attacks East-West for simplicity
            const SensorConfirmation &PDLK = AP_Vehicle::get_singleton()->PDLK;
            const float attack_limit = PDLK.NetOFLimit() - 0.01;
            if(attack_limit <= 0){
                gcs().send_text(MAV_SEVERITY_INFO, "PDLK: Attack Limit is 0");
                pdlkState = state;
            } else{
                Vector2f attack{attack_limit + state.bodyRate.x,
                                state.bodyRate.y};
                pdlkState = OpticalFlow::OpticalFlow_state{ state.surface_quality,
                                                            attack,
                                                            state.bodyRate};
            }
        } else{
        Vector2f attack{frontend._pdlk_attack_x + state.bodyRate.x,
                        frontend._pdlk_attack_y + state.bodyRate.y};
        pdlkState = OpticalFlow::OpticalFlow_state{ state.surface_quality,
                                                    attack,
                                                    state.bodyRate};
        }
        frontend.update_state(pdlkState);
    }else{
        frontend.update_state(state);
    }
}

// apply yaw angle to a vector
void OpticalFlow_backend::_applyYaw(Vector2f &v)
{
    float yawAngleRad = _yawAngleRad();
    if (is_zero(yawAngleRad)) {
        return;
    }
    float cosYaw = cosf(yawAngleRad);
    float sinYaw = sinf(yawAngleRad);
    float x = v.x;
    float y = v.y;
    v.x = cosYaw * x - sinYaw * y;
    v.y = sinYaw * x + cosYaw * y;
}

#endif
