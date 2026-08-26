/*
 *\brief API for communication over CAN bus
 *\detailed The API for communicating with the various motor controllers
 *          over the CAN bus interface on the robot hand
 */

#ifndef _CANDAPI_H
#define _CANDAPI_H

#include "canDef.h"

CANAPI_BEGIN

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

/*=====================*/
/*       Defines       */
/*=====================*/
// constants
#define TX_QUEUE_SIZE (32)
#define RX_QUEUE_SIZE (32)
#define TX_TIMEOUT (5)
#define RX_TIMEOUT (5)
#define MAX_BUS (256)

/******************/
/* CAN device API */
/******************/

/**
 * @brief command_can_open
 * @param ch
 * @return
 */
int command_can_open(int ch);

/**
 * @brief command_can_open_ex
 * @param ch
 * @param type
 * @param index
 * @return
 */
int command_can_open_ex(int ch, int type, int index);

/**
 * @brief command_can_reset
 * @param ch
 * @return
 */
int command_can_reset(int ch);

/**
 * @brief command_can_close
 * @param ch
 * @return
 */
int command_can_close(int ch);

/**
 * @brief command_can_set_id
 * @param ch
 * @param can_id
 * @return
 */
int command_can_set_id(int ch, unsigned char can_id);

/**
 * @brief command_servo_on
 * @param ch
 * @return
 */
int command_servo_on(int ch);

/**
 * @brief command_servo_off
 * @param ch
 * @return
 */
int command_servo_off(int ch);

/**
 * @brief command_set_torque
 * @param ch
 * @param findex
 * @param pwm
 * @return
 */
int command_set_torque(int ch, int findex, short *pwm);

/**
 * @brief command_set_pose
 * @param ch
 * @param findex
 * @param jposition
 * @return
 */
int command_set_pose(int ch, int findex, short *jposition);

/**
 * @brief command_set_period
 * @param ch
 * @param period Data period in millisecond. It is a array of which dimesion is 3. [0]:Joint positions, [1]:IMU, [2]temperature. If period is set to zero, periodic read will be stopped for that item.
 *               If it is null, all periodic communication will stop.
 * @return
 */
int command_set_period(int ch, short *period);

/**
 * @brief command_set_device_id
 * @param ch
 * @param did
 * @return
 */
int command_set_device_id(int ch, unsigned char did);

/**
 * @brief command_set_rs485_baudrate
 * @param ch
 * @param baudrate
 * @return
 */
int command_set_rs485_baudrate(int ch, unsigned int baudrate);

/**
 * @brief request_hand_information
 * @param ch
 * @return
 */
int request_hand_information(int ch);

/**
 * @brief request_hand_serial
 * @param ch
 * @return
 */
int request_hand_serial(int ch);

/**
 * @brief request_finger_pose
 * @param ch
 * @param findex [0,3]
 * @return
 */
int request_finger_pose(int ch, int findex);

/**
 * @brief request_imu_data
 * @param ch
 * @return
 */
int request_imu_data(int ch);

/**
 * @brief request_temperature
 * @param ch
 * @param sindex sensor index [0,3]
 * @return
 */
int request_temperature(int ch, int sindex);

/**
 * @brief can_read_message
 * @param ch
 * @param id
 * @param len
 * @param data
 * @return
 */
int can_read_message(int ch,
                     int *id,
                     int *len,
                     unsigned char *data);

/**
 * @brief can_write_message
 * @param ch
 * @param id
 * @param len
 * @param data
 * @return
 */
int can_write_message(int ch,
                      int id,
                      int len,
                      unsigned char *data);

CANAPI_END

#endif
