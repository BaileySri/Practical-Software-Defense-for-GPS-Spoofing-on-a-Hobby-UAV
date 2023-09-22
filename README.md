# Practical Software Defense for GPS Spoofing on a Hobby-UAV

## Paper Abstract
Autonomous systems, such as unmanned aerial vehicles (UAVs)
and self driving cars, operate by reacting to physical phenomena
captured by onboard sensors. Current UAVs rely on the Global Positioning
System (GPS), or other global navigation satellite system, to
determine their location on Earth. Consequently, the GPS receiver
can be used as an implicit control channel for autonomous systems.
We propose a software defense that uses observations from
independent sensor systems to detect GPS spoofing. We modify an
open source UAV control program to incorporate our defense and
evaluate our defense on hobby-grade drone hardware using simulated
GPS spoofing. In our field tests we demonstrate that built-in
sensor fusion mechanisms were unable to detect GPS spoofing and
that our method could detect subtle GPS spoofing using multiple
different sensors.

## How to install
The modifications to ArduPilot for our defense can be built using the normal [ArduPilot install instructions](https://ardupilot.org/dev/docs/building-the-code.html).

Use this repository instead of the ArduPilot repository in the instructions.

## Enabling defense
Parameters for the defense and GPS spoofing code are controlled by ArduPilot parameters we've added.

An additional trigger for the attack is set to RC Channel 7, enabling the attack if the channel signal is greater than 1600.

For GPS spoofing the relevant parameters are:
* GPS_PDLK_ATK - (0 or 1) Disable or Enable the attack
* GPS_PDLK_N - (centimeters) How far to spoof the UAV north
* GPS_PDLK_E - (centimeters) How far to spoof the UAV east
* GPS_PDLK_SLW_RAT - (meter/second<sup>2</sup>) Acceleration to apply to UAV with spoofing
* GPS_PDLK_ONE_ATK - (0 or 1) Disable or Enable Overt attack

For the defense or logging:
*  PDLK_SNSR_CONF - (0 or 1) Disable or Enable defense and logging
*  PDLK_CHOI_CI - (0 or 1) Disable or Enable the [Choi](https://doi.org/10.1145/3243734.3243752) control invariant approach

For the fence using real GPS values even during spoofing:
*  GPS_PDLK_FEN - (0 or 1) Disable or Enable fence, disables attack when leaving area
*  GPS_PDLK_FEN_ALT - (meters) Allowed change in altitude
*  GPS_PDLK_FEN_SIZ - (centimeters) A square fence of size GPS_PDLK_FEN_SIZ cm<sup>2</sup>

## Files for quick reference
These links will not work while anonymized
*  [Defense files](/libraries/SensorDefense/)
    * /libraries/SensorDefense/
*  [GPS Spoofing](/libraries/AP_GPS/AP_GPS.cpp#L954)
    * Updates are made to /libraries/AP_GPS/AP_GPS.cpp#L954
*  [Choi model](/ArduCopter/copter_invariants.cpp)
    * /ArduCopter/copter_invariants.cpp
