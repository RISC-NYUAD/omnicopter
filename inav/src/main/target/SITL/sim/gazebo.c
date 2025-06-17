/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <math.h>

#include "platform.h"

#include "target.h"
#include "target/SITL/udplink.h"
#include "target/SITL/sim/gazebo.h"
#include "target/SITL/dyad.h"
#include "target/SITL/sim/realFlight.h"
#include "target/SITL/sim/simple_soap_client.h"
#include "target/SITL/sim/xplane.h"
#include "target/SITL/sim/simHelper.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "sensors/battery_sensor_fake.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "io/rangefinder.h"
#include "common/utils.h"
#include "common/maths.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "rx/sim.h"

#define GZ_PORT 18083
#define GZ_MAX_CHANNEL_COUNT 16
// RealFlight scenerys doesn't represent real landscapes, so fake some nice coords: Area 51 ;)
#define FAKE_LAT 37.277127f
#define FAKE_LON -115.799669f

static char simulator_ip[32] = "127.0.0.1";

#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In
#define RAD2DEG (180.0 / M_PI)

static fdm_packet fdmPkt;
static rc_packet rcPkt;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt;

static bool rc_received = false;
static bool fdm_received = false;

static bool workerRunning = true;
static pthread_t tcpWorker, udpWorker, udpWorkerRC;
static udpLink_t stateLink, pwmLink, pwmRawLink, rcLink;
static uint8_t pwmMapping[GZ_MAX_PWM_OUTS];
static uint8_t mappingCount;
//static pthread_mutex_t updateLock;

static bool isInitalised = false;
static bool useImu = false;

typedef struct 
{
    double m_channelValues[GZ_MAX_PWM_OUTS];
    double m_altitudeASL_MTR;
    double m_groundspeed_MPS;
    double m_pitchRate_DEGpSEC;
    double m_rollRate_DEGpSEC;
    double m_yawRate_DEGpSEC;
    double m_azimuth_DEG;
    double m_pitch_DEG;
    double m_roll_DEG;
    double m_orientationQuaternion_X;
    double m_orientationQuaternion_Y;
    double m_orientationQuaternion_Z;
    double m_orientationQuaternion_W;
    double m_aircraftPositionX_MTR;
    double m_aircraftPositionY_MTR;
    double m_velocityNEDX_MPS;
    double m_velocityNEDY_MPS;
    double m_velocityNEDZ_MPS;
    double m_accelerationBodyAX_MPS2;
    double m_accelerationBodyAY_MPS2;
    double m_accelerationBodyAZ_MPS2;
} gzValues_t;

gzValues_t gzValues; 

/*
static bool getChannelValues(const char* response, uint16_t* channelValues)
{
    if (!response) {
        return false;
    }
    
    const char* channelValueTag = "<m-channelValues-0to1 xsi:type=\"SOAP-ENC:Array\" SOAP-ENC:arrayType=\"xsd:double[12]\">";
    char* pos = strstr(response, channelValueTag);
    if (!pos){
        return false;
    }

    pos += strlen(channelValueTag);
    for (size_t i = 0; i < RF_MAX_CHANNEL_COUNT; i++) {
        char* end = strstr(pos, "</");
        if (!end) {
            return false;
        }

        channelValues[i] = FLOAT_0_1_TO_PWM((float)atof(pos + 6));
        pos = end + 7;   
    }

    return true;
}
*/

static void fakeCoords(double posX, double posY, double distanceX, double distanceY, double *lat, double *lon)
{
    double m = 1 / (2 * (double)M_PIf / 360 * EARTH_RADIUS) / 1000;
    *lat = (double)(posX + (distanceX * m));
    *lon = (double)(posY + (distanceY * m) / cos(posX * ((double)M_PIf / 180)));
} 

static double convertAzimuth(double azimuth)
{
    if (azimuth < 0) {
        azimuth += 360;
    }
    return 360 - fmod(azimuth + 90, 360.0f);
}

/*
static void exchangeData(void)
{
    double servoValues[RF_MAX_PWM_OUTS] = { 0 };    
    for (int i = 0; i < mappingCount; i++) {
        if (pwmMapping[i] & 0x80){ // Motor
            servoValues[i] = PWM_TO_FLOAT_0_1(motor[pwmMapping[i] & 0x7f]);
        } else { 
            servoValues[i] = PWM_TO_FLOAT_0_1(servo[pwmMapping[i]]);
        }
    }

    startRequest("ExchangeData", "<ExchangeData><pControlInputs><m-selectedChannels>%u</m-selectedChannels><m-channelValues-0to1><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item></m-channelValues-0to1></pControlInputs></ExchangeData>",
        0xFFF, servoValues[0], servoValues[1], servoValues[2], servoValues[3], servoValues[4], servoValues[5], servoValues[6], servoValues[7], servoValues[8], servoValues[9], servoValues[10], servoValues[11]);
    char* response = endRequest();

    //rfValues.m_currentPhysicsTime_SEC = getDoubleFromResponse(response, "m-currentPhysicsTime-SEC");
    //rfValues.m_currentPhysicsSpeedMultiplier = getDoubleFromResponse(response, "m-currentPhysicsSpeedMultiplier");
    rfValues.m_airspeed_MPS = getDoubleFromResponse(response, "m-airspeed-MPS");
    rfValues.m_altitudeASL_MTR = getDoubleFromResponse(response, "m-altitudeASL-MTR");
    rfValues.m_altitudeAGL_MTR = getDoubleFromResponse(response, "m-altitudeAGL-MTR");
    rfValues.m_groundspeed_MPS = getDoubleFromResponse(response, "m-groundspeed-MPS");
    rfValues.m_pitchRate_DEGpSEC = getDoubleFromResponse(response, "m-pitchRate-DEGpSEC");
    rfValues.m_rollRate_DEGpSEC = getDoubleFromResponse(response, "m-rollRate-DEGpSEC");
    rfValues.m_yawRate_DEGpSEC = getDoubleFromResponse(response, "m-yawRate-DEGpSEC");
    rfValues.m_azimuth_DEG = getDoubleFromResponse(response, "m-azimuth-DEG");
    rfValues.m_inclination_DEG = getDoubleFromResponse(response, "m-inclination-DEG");
    rfValues.m_roll_DEG = getDoubleFromResponse(response, "m-roll-DEG");
    //rfValues.m_orientationQuaternion_X = getDoubleFromResponse(response, "m-orientationQuaternion-X");
    //rfValues.m_orientationQuaternion_Y = getDoubleFromResponse(response, "m-orientationQuaternion-Y");
    //rfValues.m_orientationQuaternion_Z = getDoubleFromResponse(response, "m-orientationQuaternion-Z");
    //rfValues.m_orientationQuaternion_W = getDoubleFromResponse(response, "m-orientationQuaternion-W");
    rfValues.m_aircraftPositionX_MTR = getDoubleFromResponse(response, "m-aircraftPositionX-MTR");
    rfValues.m_aircraftPositionY_MTR = getDoubleFromResponse(response, "m-aircraftPositionY-MTR");
    //rfValues.m_velocityWorldU_MPS = getDoubleFromResponse(response, "m-velocityWorldU-MPS");
    //rfValues.m_velocityWorldV_MPS = getDoubleFromResponse(response, "m-velocityWorldV-MPS");
    //rfValues.m_velocityWorldW_MPS = getDoubleFromResponse(response, "m-velocityWorldW-MPS");
    //rfValues.m_velocityBodyU_MPS = getDoubleFromResponse(response, "m-velocityBodyU-MPS");
    //rfValues.m_velocityBodyV_MPS = getDoubleFromResponse(response, "mm-velocityBodyV-MPS");
    //rfValues.m_velocityBodyW_MPS = getDoubleFromResponse(response, "m-velocityBodyW-MPS");
    //rfValues.m_accelerationWorldAX_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAX-MPS2");
    //rfValues.m_accelerationWorldAY_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAY-MPS2");
    //rfValues.m_accelerationWorldAZ_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAZ-MPS2");
    rfValues.m_accelerationBodyAX_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAX-MPS2");
    rfValues.m_accelerationBodyAY_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAY-MPS2");
    rfValues.m_accelerationBodyAZ_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAZ-MPS2");
    //rfValues.m_windX_MPS = getDoubleFromResponse(response, "m-windX-MPS");
    //rfValues.m_windY_MPS = getDoubleFromResponse(response, "m-windY-MPS");
    //rfValues.m_windZ_MPSPS = getDoubleFromResponse(response, "m-windZ-MPS");
    //rfValues.m_propRPM = getDoubleFromResponse(response, "m-propRPM");
    //rfValues.m_heliMainRotorRPM = getDoubleFromResponse(response, "m-heliMainRotorRPM");
    rfValues.m_batteryVoltage_VOLTS = getDoubleFromResponse(response, "m-batteryVoltage-VOLTS");
    rfValues.m_batteryCurrentDraw_AMPS = getDoubleFromResponse(response, "m-batteryCurrentDraw-AMPS");
    //rfValues.m_batteryRemainingCapacity_MAH = getDoubleFromResponse(response, "m-batteryRemainingCapacity-MAH");
    //rfValues.m_fuelRemaining_OZ = getDoubleFromResponse(response, "m-fuelRemaining-OZ");
    //rfValues.m_isLocked = getBoolFromResponse(response, "m-isLocked");
    //rfValues.m_hasLostComponents = getBoolFromResponse(response, "m-hasLostComponents");
    //rfValues.m_anEngineIsRunning = getBoolFromResponse(response, "m-anEngineIsRunning");
    //rfValues.m_isTouchingGround = getBoolFromResponse(response, "m-isTouchingGround");
    //rfValues.m_flightAxisControllerIsActive= getBoolFromResponse(response, "m-flightAxisControllerIsActive");
    rfValues.m_currentAircraftStatus = getStringFromResponse(response, "m-currentAircraftStatus");

    
    uint16_t channelValues[RF_MAX_CHANNEL_COUNT];
    getChannelValues(response, channelValues);
    rxSimSetChannelValue(channelValues, RF_MAX_CHANNEL_COUNT);
    
    double lat, lon;
    fakeCoords(FAKE_LAT, FAKE_LON, rfValues.m_aircraftPositionX_MTR, -rfValues.m_aircraftPositionY_MTR, &lat, &lon);
    
    int16_t course = (int16_t)round(convertAzimuth(rfValues.m_azimuth_DEG) * 10);
    int32_t altitude = (int32_t)round(rfValues.m_altitudeASL_MTR * 100);
    gpsFakeSet(
        GPS_FIX_3D,
        16,
        (int32_t)round(lat * 10000000),
        (int32_t)round(lon * 10000000),
        altitude,
        (int16_t)round(rfValues.m_groundspeed_MPS * 100),
        course,
        0, 
        0,
        0,
        0
    );

    int32_t altitudeOverGround = (int32_t)round(rfValues.m_altitudeAGL_MTR * 100);
    if (altitudeOverGround > 0 && altitudeOverGround <= RANGEFINDER_VIRTUAL_MAX_RANGE_CM) {
        fakeRangefindersSetData(altitudeOverGround);
    } else {
        fakeRangefindersSetData(-1);
    }

    const int16_t roll_inav = (int16_t)round(rfValues.m_roll_DEG * 10);
    const int16_t pitch_inav = (int16_t)round(-rfValues.m_inclination_DEG * 10);
    const int16_t yaw_inav = course;
    if (!useImu) {
        imuSetAttitudeRPY(roll_inav, pitch_inav, yaw_inav);
        imuUpdateAttitude(micros());
    }

    // RealFlights acc data is weird if the aircraft has not yet taken off. Fake 1G in horizontale position
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    if (rfValues.m_currentAircraftStatus && strncmp(rfValues.m_currentAircraftStatus, "CAS-WAITINGTOLAUNCH", strlen(rfValues.m_currentAircraftStatus)) == 0) {
        accX = 0;
        accY = 0;
        accZ = (int16_t)(GRAVITY_MSS * 1000.0f);
    } else {
         accX = constrainToInt16(rfValues.m_accelerationBodyAX_MPS2 * 1000);
         accY = constrainToInt16(-rfValues.m_accelerationBodyAY_MPS2 * 1000);
         accZ = constrainToInt16(-rfValues.m_accelerationBodyAZ_MPS2 * 1000);
    }

    fakeAccSet(accX, accY, accZ);

    fakeGyroSet(
        constrainToInt16(rfValues.m_rollRate_DEGpSEC * (double)16.0),
        constrainToInt16(-rfValues.m_pitchRate_DEGpSEC * (double)16.0),
        constrainToInt16(rfValues.m_yawRate_DEGpSEC * (double)16.0)
    );

    fakeBaroSet(altitudeToPressure(altitude), DEGREES_TO_CENTIDEGREES(21));
    fakePitotSetAirspeed(rfValues.m_airspeed_MPS * 100);

    fakeBattSensorSetVbat((uint16_t)round(rfValues.m_batteryVoltage_VOLTS * 100));
    fakeBattSensorSetAmperage((uint16_t)round(rfValues.m_batteryCurrentDraw_AMPS * 100)); 

    fpQuaternion_t quat;
    fpVector3_t north;
    north.x = 1.0f;
    north.y = 0;
    north.z = 0;
    computeQuaternionFromRPY(&quat, roll_inav, pitch_inav, yaw_inav);
    transformVectorEarthToBody(&north, &quat);
    fakeMagSet(
        constrainToInt16(north.x * 16000.0f),
        constrainToInt16(north.y * 16000.0f),
        constrainToInt16(north.z * 16000.0f)
    );
}
*/

//=============================BELOW==============================//
void updateState(const fdm_packet* pkt)
{

//    uint16_t channelValues[GZ_MAX_CHANNEL_COUNT];
//    getChannelValues(response, channelValues);
    rxSimSetChannelValue(rcPkt.channels, GZ_MAX_CHANNEL_COUNT);

    double servoValues[GZ_MAX_PWM_OUTS] = { 0 };    
    for (int i = 0; i < mappingCount; i++) {
        if (pwmMapping[i] & 0x80){ // Motor
            servoValues[i] = PWM_TO_FLOAT_0_1(motor[pwmMapping[i] & 0x7f]);
        } else { 
            servoValues[i] = PWM_TO_FLOAT_0_1(servo[pwmMapping[i]]);
        }
    }

//    double outScale = 1000.0;

    pwmPkt.motor_speed[3] = servoValues[0] ;/// outScale;
    pwmPkt.motor_speed[0] = servoValues[1] ;/// outScale;
    pwmPkt.motor_speed[1] = servoValues[2] ;/// outScale;
    pwmPkt.motor_speed[2] = servoValues[3] ;/// outScale;
//    pwmPkt.motor_speed[3] = 100.0 / outScale;
//    pwmPkt.motor_speed[0] = 100.0 / outScale;
//    pwmPkt.motor_speed[1] = 100.0 / outScale;
//    pwmPkt.motor_speed[2] = 100.0 / outScale;




    gzValues.m_aircraftPositionX_MTR = pkt->position_xyz[0];
    gzValues.m_aircraftPositionY_MTR = pkt->position_xyz[1];
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;

	gzValues.m_roll_DEG = xf;
	gzValues.m_pitch_DEG = yf;
	gzValues.m_azimuth_DEG = zf;
    gzValues.m_altitudeASL_MTR = -pkt->position_xyz[2];
	gzValues.m_velocityNEDX_MPS = pkt->velocity_xyz[0];
	gzValues.m_velocityNEDY_MPS = pkt->velocity_xyz[1];
	gzValues.m_groundspeed_MPS = sqrt(gzValues.m_velocityNEDX_MPS*gzValues.m_velocityNEDX_MPS + gzValues.m_velocityNEDY_MPS*gzValues.m_velocityNEDY_MPS); 
	gzValues.m_accelerationBodyAX_MPS2 = pkt->imu_linear_acceleration_xyz[0];
	gzValues.m_accelerationBodyAY_MPS2 = pkt->imu_linear_acceleration_xyz[1];
	gzValues.m_accelerationBodyAZ_MPS2 = pkt->imu_linear_acceleration_xyz[2];
	gzValues.m_rollRate_DEGpSEC = pkt->imu_angular_velocity_rpy[0];
	gzValues.m_pitchRate_DEGpSEC = pkt->imu_angular_velocity_rpy[1];
	gzValues.m_yawRate_DEGpSEC = pkt->imu_angular_velocity_rpy[2];
    gzValues.m_orientationQuaternion_X = qx;
    gzValues.m_orientationQuaternion_Y = qy;
    gzValues.m_orientationQuaternion_Z = qz;
    gzValues.m_orientationQuaternion_W = qw;

    double lat, lon;
    fakeCoords(FAKE_LAT, FAKE_LON, gzValues.m_aircraftPositionX_MTR, -gzValues.m_aircraftPositionY_MTR, &lat, &lon);
    
    int16_t course = (int16_t)round(convertAzimuth(gzValues.m_azimuth_DEG) * 10);
    int32_t altitude = (int32_t)round(gzValues.m_altitudeASL_MTR * 100);
    gpsFakeSet(
        GPS_FIX_3D,
        16,
        (int32_t)round(lat * 10000000),
        (int32_t)round(lon * 10000000),
        altitude,
        (int16_t)round(gzValues.m_groundspeed_MPS * 100),
        course,
        0, 
        0,
        0,
        0
    );

    const int16_t roll_inav = (int16_t)round(gzValues.m_roll_DEG * 10);
    const int16_t pitch_inav = (int16_t)round(-gzValues.m_pitch_DEG * 10);
    const int16_t yaw_inav = course;
    if (!useImu) {
        imuSetAttitudeRPY(roll_inav, pitch_inav, yaw_inav);
        imuUpdateAttitude(micros());
    }

    // Gazebo acc data is weird if the aircraft has not yet taken off due to ground collisions. Fake 1G in horizontale position
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    if (gzValues.m_altitudeASL_MTR < 0.02) {
        accX = 0;
        accY = 0;
        accZ = (int16_t)(GRAVITY_MSS * 1000.0f);
    } else {
         accX = constrainToInt16(gzValues.m_accelerationBodyAX_MPS2 * 1000);
         accY = constrainToInt16(-gzValues.m_accelerationBodyAY_MPS2 * 1000);
         accZ = constrainToInt16(-gzValues.m_accelerationBodyAZ_MPS2 * 1000);
    }

    fakeAccSet(accX, accY, accZ);
    
    fakeGyroSet(
        constrainToInt16(gzValues.m_rollRate_DEGpSEC * (double)16.0),
        constrainToInt16(-gzValues.m_pitchRate_DEGpSEC * (double)16.0),
        constrainToInt16(gzValues.m_yawRate_DEGpSEC * (double)16.0)
    );

    fakeBaroSet(altitudeToPressure(altitude), DEGREES_TO_CENTIDEGREES(21));
    fakePitotSetAirspeed(gzValues.m_groundspeed_MPS * 100);

    fakeBattSensorSetVbat((uint16_t)round(16.0 * 100));
    fakeBattSensorSetAmperage((uint16_t)round(20.0 * 100)); 

    fpQuaternion_t quat;
    fpVector3_t north;
    north.x = 1.0f;
    north.y = 0;
    north.z = 0;
    computeQuaternionFromRPY(&quat, roll_inav, pitch_inav, yaw_inav);
    transformVectorEarthToBody(&north, &quat);
    fakeMagSet(
        constrainToInt16(north.x * 16000.0f),
        constrainToInt16(north.y * 16000.0f),
        constrainToInt16(north.z * 16000.0f)
    );

	if(!isInitalised){
		printf("initialized\n");
        ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
        // Aircraft can wobble on the runway and prevents calibration of the accelerometer
        ENABLE_STATE(ACCELEROMETER_CALIBRATED);
		isInitalised = true;
	}
    unlockMainPID();

//    pthread_mutex_unlock(&updateLock); // can send PWM output now

    udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
    printf("[pwm]:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", servoValues[0], servoValues[1], servoValues[2], servoValues[3],servoValues[4],servoValues[5],servoValues[6],servoValues[7],servoValues[8],servoValues[9],servoValues[10],servoValues[11]);
//    udpSend(&pwmRawLink, &pwmRawPkt, sizeof(servo_packet_raw));

}


static void* tcpThread(void* data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.5f);

    while (workerRunning) {
        dyad_update();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

static void* udpThread(void* data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
        if (n == sizeof(fdm_packet)) {
            if (!fdm_received) {
                printf("[SITL] new fdm %d t:%f from %s:%d\n", n, fdmPkt.timestamp, inet_ntoa(stateLink.recv.sin_addr), stateLink.recv.sin_port);
                fdm_received = true;
            }
            updateState(&fdmPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

static float readRCSITL(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    return rcPkt.channels[channel];
}

static uint8_t rxRCFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    return RX_FRAME_COMPLETE;
}

static void *udpRCThread(void *data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&rcLink, &rcPkt, sizeof(rc_packet), 100);
        if (n == sizeof(rc_packet)) {
            if (!rc_received) {
                printf("[SITL] new rc %d: t:%f AETR: %d %d %d %d AUX1-4: %d %d %d %d\n", n, rcPkt.timestamp,
                    rcPkt.channels[0], rcPkt.channels[1],rcPkt.channels[2],rcPkt.channels[3],
                    rcPkt.channels[4], rcPkt.channels[5],rcPkt.channels[6],rcPkt.channels[7]);
                rc_received = true;
            }
        }
    }

    printf("udpRCThread end!!\n");
    return NULL;
}


bool simGazeboInit(char* ip, uint8_t* mapping, uint8_t mapCount, bool imu)
{
	int ret;

    memcpy(pwmMapping, mapping, mapCount);
    mappingCount = mapCount;
    useImu = imu;

//    if (pthread_mutex_init(&updateLock, NULL) != 0) {
//        printf("Create updateLock error!\n");
//        exit(1);
//    }

    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);

    ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);

    ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);

    ret = udpInit(&rcLink, NULL, PORT_RC, true);
    printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);

//    ret = udpInit(&stateLink, NULL, 9003, true);
//    printf("start UDP server...%d\n", ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    ret = pthread_create(&udpWorkerRC, NULL, udpRCThread, NULL);
    if (ret != 0) {
        printf("Create udpRCThread error!\n");
        exit(1);
    }

    // Wait until the connection is established, the interface has been initialised 
    // and the first valid packet has been received to avoid problems with the startup calibration.   
    while (!isInitalised) {
        delay(250);
    }

    return true;
}
