/*
 *  @file canDef.h
 *  @brief definition of constants used by CAN API
 */

#ifndef __ALLEGROHAND_CANDEF_H__
#define __ALLEGROHAND_CANDEF_H__

#ifdef USING_NAMESPACE_CANAPI
#define CANAPI_BEGIN \
    namespace CANAPI \
    {
#define CANAPI_END \
    }              \
    ;
#else
#define CANAPI_BEGIN
#define CANAPI_END
#define CANAPI
#endif

CANAPI_BEGIN

////////////////////////////////////////////////
//  Define CAN Command
#define ID_CMD_SYSTEM_ON 0x40
#define ID_CMD_SYSTEM_OFF 0x41
#define ID_CMD_SET_TORQUE 0x60
#define ID_CMD_SET_TORQUE_1 (ID_CMD_SET_TORQUE + 0)
#define ID_CMD_SET_TORQUE_2 (ID_CMD_SET_TORQUE + 1)
#define ID_CMD_SET_TORQUE_3 (ID_CMD_SET_TORQUE + 2)
#define ID_CMD_SET_TORQUE_4 (ID_CMD_SET_TORQUE + 3)
#define ID_CMD_SET_POSE_1 0xE0
#define ID_CMD_SET_POSE_2 0xE1
#define ID_CMD_SET_POSE_3 0xE2
#define ID_CMD_SET_POSE_4 0xE3
#define ID_CMD_SET_PERIOD 0x81
#define ID_CMD_CONFIG 0x68

////////////////////////////////////////////////
//  Define CAN Data Reqeust (RTR)
#define ID_RTR_HAND_INFO 0x80
#define ID_RTR_SERIAL 0x88
#define ID_RTR_FINGER_POSE 0x20
#define ID_RTR_FINGER_POSE_1 (ID_RTR_FINGER_POSE + 0)
#define ID_RTR_FINGER_POSE_2 (ID_RTR_FINGER_POSE + 1)
#define ID_RTR_FINGER_POSE_3 (ID_RTR_FINGER_POSE + 2)
#define ID_RTR_FINGER_POSE_4 (ID_RTR_FINGER_POSE + 3)
#define ID_RTR_IMU_DATA 0x30
#define ID_RTR_TEMPERATURE 0x38
#define ID_RTR_TEMPERATURE_1 (ID_RTR_TEMPERATURE + 0)
#define ID_RTR_TEMPERATURE_2 (ID_RTR_TEMPERATURE + 1)
#define ID_RTR_TEMPERATURE_3 (ID_RTR_TEMPERATURE + 2)
#define ID_RTR_TEMPERATURE_4 (ID_RTR_TEMPERATURE + 3)

CANAPI_END

#endif // __ALLEGROHAND_CANDEF_H__
