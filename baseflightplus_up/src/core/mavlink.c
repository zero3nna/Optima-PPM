#include "board.h"
#include "core/mavlink.h"
#include "actuator/mixer.h"

uint8_t mavlinkMode = false;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define comm_send_ch(chan, ch) uartWrite(ch)
#include <mavlink/mavlink_types.h>

extern mavlink_system_t mavlink_system;

#include <mavlink/common/mavlink.h>

mavlink_system_t mavlink_system;

#define MAVLINK_PARAMID_LEN                     16

#define MAVLINK_HEARTBEAT_INTERVAL              (1000)	    //  1Hz
#define MAVLINK_PARAM_INTERVAL                  (1000/50)   // 50Hz
#define MAVLINK_DS_RAW_SENSORS_INTERVAL         (1000/100)   // 50Hz
#define MAVLINK_DS_RAW_SENSORS_POSITION         (1000/2)    //  5Hz
#define MAVLINK_DS_RAW_SENSORS_RC_CHANNELS      (1000/20)   // 20Hz
#define MAVLINK_DS_RAW_SENSORS_RAW_CONTROLLER   (1000/20)   // 20Hz

#define IS_STREAM_TRIGGERED(id) \
    (mavlinkData.streamInterval[id] && mavlinkData.streamNext[id] < _millis)

typedef struct {
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;
    uint16_t streamInterval[MAV_DATA_STREAM_ENUM_END];
    unsigned long streamNext[MAV_DATA_STREAM_ENUM_END];
    uint16_t packetDrops;
    uint8_t mode;
    uint8_t nav_mode;
    uint8_t status;
} mavlinkStruct_t;

static mavlinkStruct_t mavlinkData;

extern uint32_t cycleTime;
extern int32_t b5, p; // temp / pressure - bmp085.c
uint16_t acc_1G = 265; // 3.3V operation

// ==============================================================================================
// Parameters mapping
// ==============================================================================================

typedef enum  {
    _uint8_t,
    _uint16_t,
    _uint32_t,
    _float_t,
} type_t;

typedef struct _param_t {
    const char* name;
    void* ptr;
    type_t type;
} param_t;

uint16_t param_test1;
float param_test2;

static param_t configParameters[] = {
    { "Parameter1", &param_test1, _uint16_t},
    { "Parameter2", &param_test2, _float_t},
};

#define PARAM_COUNT (sizeof(configParameters) / sizeof(configParameters[0]))

static float getConfigParameterValue(int param)
{
    switch(configParameters[param].type)
    {
    case _uint8_t:
        return (float)*(uint8_t*)configParameters[param].ptr;
    case _uint16_t:
        return (float)*(uint16_t*)configParameters[param].ptr;
    case _uint32_t:
        return (float)*(uint32_t*)configParameters[param].ptr;
    case _float_t:
        return (float)*(float*)configParameters[param].ptr;
    }
    return 0;
}

static void setConfigParameterValue(int param, float paramValue)
{
    switch(configParameters[param].type)
    {
    case _uint8_t:
        *(uint8_t*)configParameters[param].ptr = (uint8_t)paramValue;
    case _uint16_t:
        *(uint16_t*)configParameters[param].ptr = (uint16_t)paramValue;
    case _uint32_t:
        *(uint32_t*)configParameters[param].ptr = (uint32_t)paramValue;
    case _float_t:
        *(float*)configParameters[param].ptr = (float)paramValue;
    }
}

// ==============================================================================================
// mavlink implementation
// ==============================================================================================

void mavlinkInit(void) 
{
    mavlink_system.sysid = 20;
    mavlink_system.compid = MAV_COMP_ID_IMU;
    mavlink_system.type = MAV_TYPE_QUADROTOR;

    memset(&mavlinkData, 0, sizeof(mavlinkData));

    mavlinkData.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    mavlinkData.nav_mode = MAV_STATE_STANDBY;
    mavlinkData.status = MAV_STATE_ACTIVE;

    mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS] = MAVLINK_DS_RAW_SENSORS_INTERVAL;
    mavlinkData.streamInterval[MAV_DATA_STREAM_POSITION] = MAVLINK_DS_RAW_SENSORS_POSITION;
    mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS] = MAVLINK_DS_RAW_SENSORS_RC_CHANNELS;
    mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER] = MAVLINK_DS_RAW_SENSORS_RAW_CONTROLLER;
}

// ==============================================================================================

void mavlinkSendData(void)
{
    static unsigned long lastMillis = 0;
    unsigned long _millis;

    _millis = millis();

    // handle rollover
    if (_millis < lastMillis) {
	mavlinkData.nextHeartbeat = 0;
	mavlinkData.nextParam = 0;
	memset(mavlinkData.streamNext, 0, sizeof(mavlinkData.streamNext));
    }

    // heartbeat & status
    if (mavlinkData.nextHeartbeat < _millis)
    {
	mavlink_msg_heartbeat_send(
            MAVLINK_COMM_0, 
            mavlink_system.type,
            MAV_AUTOPILOT_GENERIC, 
            mavlinkData.mode, 
            mavlinkData.nav_mode, 
            mavlinkData.status
            );

        mavlink_msg_sys_status_send(
            MAVLINK_COMM_0, 
            0,                  // onboard_control_sensors_present
            0,                  // onboard_control_sensors_enabled
            0,                  // onboard_control_sensors_health
            1000 ,          // load
            0,                  // voltage_battery
            -1,                 // current_battery
            -1,                 // battery_remaining
            0,                  // drop_rate_comm
            mavlinkData.packetDrops,    // errors_comm
            0,                  // errors_count1 
            0,                  // errors_count2
            0,                  // errors_count3
            0                   // errors_count4
            );

        mavlinkData.nextHeartbeat = _millis + MAVLINK_HEARTBEAT_INTERVAL;
    }

    // raw sensors
    if (IS_STREAM_TRIGGERED(MAV_DATA_STREAM_RAW_SENSORS))
    {
        // acceleration in mg
        float acc_scale = (1.0f/(float)acc_1G *1000.0f);
        // Angular speed in millirad /sec
        float gyro_scale = (1.0f/16.4f*DEG2RAD *1000.0f); // MPU3050: 16.4 LSB/Â°/sec at range +-2000Â°/sec
        // Magnetic field in milli tesla
        float mag_scale = (1.0f/1090.f *0.1f); // 1090LSB/Gauss - 1 gauss = 0.1 millitesla
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, _millis, 
            (int16_t)((float)rawAccel[0]*acc_scale), (int16_t)((float)rawAccel[1]*acc_scale), (int16_t)((float)rawAccel[2]*acc_scale),
            (int16_t)((float)rawGyro[0]*gyro_scale), (int16_t)((float)rawGyro[1]*gyro_scale), (int16_t)((float)rawGyro[2]*gyro_scale),
            (int16_t)((float)rawMag[0]*mag_scale), (int16_t)((float)rawMag[1]*mag_scale), (int16_t)((float)rawMag[2]*mag_scale)
            );
        // Absolute pressure (hectopascal)
       // mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, _millis, sensors.baroPressure*0.01f, 0, 0);
        mavlinkData.streamNext[MAV_DATA_STREAM_RAW_SENSORS] = _millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS];;
    }

    // position
    if (IS_STREAM_TRIGGERED(MAV_DATA_STREAM_POSITION))
    {
	//mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, _millis, 3, GPS_coord[LAT]*1e7, GPS_coord[LON]*1e7, GPS_altitude*1e3, 65535, 65535, GPS_speed, GPS_ground_course, GPS_numSat);
	mavlinkData.streamNext[MAV_DATA_STREAM_POSITION] = _millis + mavlinkData.streamInterval[MAV_DATA_STREAM_POSITION];;
    }

    // extended status
    if (IS_STREAM_TRIGGERED(MAV_DATA_STREAM_EXTENDED_STATUS))
    {
        mavlinkData.streamNext[MAV_DATA_STREAM_EXTENDED_STATUS] = _millis + mavlinkData.streamInterval[MAV_DATA_STREAM_EXTENDED_STATUS];;
    }

    // rc channels
    if (IS_STREAM_TRIGGERED(MAV_DATA_STREAM_RC_CHANNELS))
    {
	mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, _millis, 0,
            rcData[THROTTLE], 
            rcData[ROLL],
            rcData[PITCH],
            rcData[YAW],
            rcData[AUX1],
            rcData[AUX2],
            rcData[AUX3],
            rcData[AUX4],
            255);
	mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, _millis, 0,
            command[THROTTLE],
            command[ROLL],
            command[PITCH],
            command[YAW],          
            rcData[AUX1],
            rcData[AUX2],
            rcData[AUX3],
            rcData[AUX4],
            255);
        mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] = _millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS];;
    }

    // raw controller
    if (IS_STREAM_TRIGGERED(MAV_DATA_STREAM_RAW_CONTROLLER))
    {
        // Angular speed in millirad /sec
        float gyro_scale = (1.0f/16.4f*DEG2RAD *1000.0f); // MPU3050: 16.4 LSB/Â°/sec at range +-2000Â°/sec
	mavlink_msg_attitude_send(MAVLINK_COMM_0, _millis, 
            -sensors.attitude[ROLL], sensors.attitude[PITCH], sensors.attitude[YAW], 
            -(int16_t)((float)rawGyro[0]*gyro_scale), (int16_t)((float)rawGyro[1]*gyro_scale), (int16_t)((float)rawGyro[2]*gyro_scale));
	mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, _millis, 0, motor[0], motor[1], motor[2], motor[3], motor[4], motor[5], motor[6], motor[7]);
	mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] = _millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER];;
    }

    // send pending notices
    //mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *)msg);

    // list all parameters
    if (mavlinkData.currentParam < PARAM_COUNT && mavlinkData.nextParam < _millis) {
	mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameters[mavlinkData.currentParam].name, getConfigParameterValue(mavlinkData.currentParam), MAVLINK_TYPE_FLOAT, PARAM_COUNT, mavlinkData.currentParam);
	mavlinkData.currentParam++;
	mavlinkData.nextParam = _millis + MAVLINK_PARAM_INTERVAL;
    }

    lastMillis = _millis;
}

static void mavlinkDoCommand(mavlink_message_t *msg) {
    uint16_t command;
    uint8_t ack = MAV_CMD_ACK_ERR_NOT_SUPPORTED;

    command = mavlink_msg_command_long_get_command(msg);

    switch (command) {
	case MAV_CMD_PREFLIGHT_STORAGE:
            ack = MAV_CMD_ACK_OK;
	    break;

	default:
	    break;
    }

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, ack);
}

/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/   
void mavlinkComm(void)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    char paramId[MAVLINK_PARAMID_LEN];
    int32_t i;

    while(uartAvailable())
    {
        uint8_t c = uartRead();
        // Try to get a new message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    if (mavlink_msg_command_long_get_target_system(&msg) == mavlink_system.sysid) {
                        mavlinkDoCommand(&msg);
                    }
                    break;

                case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
                    if (mavlink_msg_change_operator_control_get_target_system(&msg) == mavlink_system.sysid) {
                        mavlink_msg_change_operator_control_ack_send(MAVLINK_COMM_0, msg.sysid, mavlink_msg_change_operator_control_get_control_request(&msg), 0);
                    }
                    break;

                case MAVLINK_MSG_ID_SET_MODE:
                    if (mavlink_msg_set_mode_get_target_system(&msg) == mavlink_system.sysid) {
                        mavlinkData.mode = mavlink_msg_set_mode_get_base_mode(&msg);
                        mavlinkData.nextHeartbeat = 0; // force heartbeat/status update
                    }
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    if (mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) {
                        uint16_t paramIndex;

                        paramIndex = mavlink_msg_param_request_read_get_param_index(&msg);
                        if (paramIndex < PARAM_COUNT)
                            mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameters[paramIndex].name, getConfigParameterValue(paramIndex), MAVLINK_TYPE_FLOAT, PARAM_COUNT, paramIndex);
                    }
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    if (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) {
			    mavlinkData.currentParam = 0;
			    mavlinkData.nextParam = 0;
                    }
                    break;

                case MAVLINK_MSG_ID_PARAM_SET:
                    if (mavlink_msg_param_set_get_target_system(&msg) == mavlink_system.sysid) {
                        int paramIndex = -1;

                        mavlink_msg_param_set_get_param_id(&msg, paramId);
                        for (i = 0; i < PARAM_COUNT; i++) 
                        {
                            if(strcmp(paramId, configParameters[i].name) == 0) 
                            {
                                paramIndex = i;
                                break;
                            }
                        }

                        if (paramIndex >= 0 && paramIndex < PARAM_COUNT) {
                            float paramValue = mavlink_msg_param_set_get_param_value(&msg);
                            if (!isnan(paramValue) && !isinf(paramValue) /*&& !(supervisorData.state & STATE_FLYING)*/)
                                setConfigParameterValue(paramIndex, paramValue);
                            // send back what we have no matter what
                            mavlink_msg_param_value_send(MAVLINK_COMM_0, paramId, getConfigParameterValue(paramIndex), MAVLINK_TYPE_FLOAT, PARAM_COUNT, paramIndex);
                        }
                    }
                    break;

                case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                    if (mavlink_msg_request_data_stream_get_target_system(&msg) == mavlink_system.sysid) {
                        uint16_t rate;
                        uint8_t stream_id;

                        stream_id = mavlink_msg_request_data_stream_get_req_stream_id(&msg);
                        rate = mavlink_msg_request_data_stream_get_req_message_rate(&msg);

                        if (rate > 0 && rate < 200 && stream_id < MAV_DATA_STREAM_ENUM_END && mavlink_msg_request_data_stream_get_start_stop(&msg)) {
                            if(stream_id == MAV_DATA_STREAM_ALL) {
                                for(i=1;i<MAV_DATA_STREAM_ENUM_END;i++)
                                    mavlinkData.streamInterval[i] = rate;
                            }
                            else
                                mavlinkData.streamInterval[stream_id] = rate;
                        }
                        else {
                            if(stream_id == MAV_DATA_STREAM_ALL) {
                                for(i=1;i<MAV_DATA_STREAM_ENUM_END;i++)
                                    mavlinkData.streamInterval[i] = 0;
                            }
                            else
                                mavlinkData.streamInterval[stream_id] = 0;
                        }
                    }
                    break;

                default:
                    // Do nothing
                    break;
            }

        }

        else
        {
            mavlink_status_t* status = mavlink_get_channel_status(MAVLINK_COMM_0); ///< The current decode status
            if( status->parse_state <= MAVLINK_PARSE_STATE_IDLE )
            {
                // last char was not for mavlink 
                if( c == '~' )
                {
                    mavlinkMode = false; // switch back to baseflightplus serial communications
                }
            }
        }

        // Update global packet drops counter
        mavlinkData.packetDrops += status.packet_rx_drop_count;   
    }
}