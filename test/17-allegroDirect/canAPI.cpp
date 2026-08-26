/*======================*/
/*       Includes       */
/*======================*/
// System headers
#include <errno.h>
#include <stdio.h>
#ifndef _WIN32
#include <fcntl.h>
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <unistd.h>
#else
#include <windows.h>
#endif
#include <assert.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <malloc.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "canAPI.h"
#include "canDef.h"

//=========================================
//  Logging macros
//=========================================
#define CAN_LOG_INFO(fmt, ...) fprintf(stdout, "[INFO] " fmt "\n", ##__VA_ARGS__)
#define CAN_LOG_WARN(fmt, ...) fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#define CAN_LOG_ERROR(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

CANAPI_BEGIN

/*=====================*/
/*       Defines       */
/*=====================*/
#define NUM_OF_FINGERS 4
#define NUM_OF_TEMP_SENSORS 4

// structure for period-setting command
typedef struct __attribute__((packed))
{
    uint16_t position;
    uint16_t imu;
    uint16_t temp;
} can_period_msg_t;

// structure for config command (device ID / RS-485 baudrate)
typedef struct __attribute__((packed))
{
    uint8_t set;
    uint8_t did;
    uint32_t baudrate;
} can_config_msg_t;

/*=========================================*/
/*       Global variables (file-scope)     */
/*=========================================*/
static unsigned char CAN_ID = 0;
static int socket_;

/*========================================*/
/*    Private function prototypes         */
/*========================================*/
static int raw_read(can_frame &msg);
static int raw_write(const can_frame &msg);

/*========================================*/
/*    Private CAN operations              */
/*========================================*/
static int canSendMsg(int ch, int Txid, const void *payload, uint8_t len)
{
    (void)ch; // placeholder for multi-channel support
    can_frame msg{};
    msg.can_id = ((uint32_t)Txid << 2) | CAN_ID;
    msg.can_dlc = len;
    if (payload && len)
        memcpy(msg.data, payload, len);
    return raw_write(msg);
}

static int canSentRTR(int ch, int Txid)
{
    (void)ch;
    can_frame msg{};
    msg.can_id = ((uint32_t)Txid << 2) | CAN_ID | CAN_RTR_FLAG;
    msg.can_dlc = 0;
    return raw_write(msg);
}

static int canReadMsg(int ch, int &out_id, uint8_t &out_len, void *out_payload)
{
    (void)ch;
    can_frame msg{};
    int n = raw_read(msg);
    if (n < 0)
        return n;
    if (msg.can_id & CAN_ERR_FLAG)
    {
        CAN_LOG_ERROR("CAN error frame received (0x%X)", msg.can_id);
        return -1;
    }
    out_id = (msg.can_id & ~CAN_ERR_FLAG) >> 2;
    out_len = msg.can_dlc;
    memcpy(out_payload, msg.data, msg.can_dlc);
    return 0;
}

/*========================================*/
/*    Public functions (CAN API)          */
/*========================================*/

// Print a frame (for errors)
void printMsg(const can_frame &msg)
{
    CAN_LOG_WARN("msg: {0x%X}, dlc=%d", msg.can_id, msg.can_dlc);
}

// Initialize CAN socket on "can<bus>"
int canInit(int bus)
{
    int i;
    char iface[IFNAMSIZ] = {0};
    snprintf(iface, IFNAMSIZ, "can%d", bus);
    CAN_LOG_INFO("CAN: Initializing bus %d ('%s')", bus, iface);

    struct sockaddr_can addr
    {
    };
    struct ifreq ifr
    {
    };

    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1)
    {
        CAN_LOG_ERROR("Failed to open CAN socket: %s", strerror(errno));
        return -1;
    }

    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) == -1)
    {
        CAN_LOG_ERROR("Trouble finding CAN bus %d ('%s'): %s",
                      bus, iface, strerror(errno));
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == -1)
    {
        CAN_LOG_ERROR("Error binding CAN socket: %s", strerror(errno));
        return -1;
    }

    int flags = fcntl(socket_, F_GETFL);
    if (flags == -1 || fcntl(socket_, F_SETFL, flags | O_NONBLOCK) != 0)
    {
        CAN_LOG_ERROR("Error setting CAN socket non-blocking: %s", strerror(errno));
        return -1;
    }

    CAN_LOG_INFO("CAN: Clearing the CAN buffer");
    can_frame dummy;
    for (i = 0; i < 100; ++i)
        read(socket_, &dummy, sizeof(dummy));
    usleep(500000);
    return 0;
}

// Low-level read
static int raw_read(can_frame &msg)
{
    int n = read(socket_, &msg, sizeof(msg));
    if (n == -1 && errno == EAGAIN)
        return -1;
    if (n != sizeof(msg))
    {
        CAN_LOG_ERROR("Read incomplete CAN frame.");
        return -1;
    }
    return n;
}

// Low-level write
static int raw_write(const can_frame &msg)
{
    int n = write(socket_, &msg, sizeof(msg));
    if (n != sizeof(msg))
    {
        CAN_LOG_WARN("CAN_Write failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

// API: open
int command_can_open(int bus)
{
    return canInit(bus);
}

// API: close
int command_can_close(int ch)
{
    (void)ch;
    if (close(socket_) != 0)
    {
        CAN_LOG_ERROR("CAN: Error in command_can_close()");
        return -1;
    }
    return 0;
}

// API: set CAN ID
int command_can_set_id(int ch, unsigned char can_id)
{
    (void)ch;
    CAN_ID = can_id;
    return 0;
}

// API: servo on/off
typedef long Txid_t;
int command_servo_on(int ch)
{
    return canSendMsg(ch, ID_CMD_SYSTEM_ON, nullptr, 0);
}
int command_servo_off(int ch)
{
    return canSendMsg(ch, ID_CMD_SYSTEM_OFF, nullptr, 0);
}

// API: set torque
int command_set_torque(int ch, int findex, short* pwm)
{
    // Validate finger index
    assert(findex >= 0 && findex < NUM_OF_FINGERS);

    // Copy PWM values into a 16-bit duty array for readability
    short duty[4];
    duty[0] = pwm[0];
    duty[1] = pwm[1];
    duty[2] = pwm[2];
    duty[3] = pwm[3];

    // Transmit the 4 duty values (8 bytes total)
    int Txid = ID_CMD_SET_TORQUE_1 + findex;
    return canSendMsg(ch, Txid, duty, sizeof(duty));
}

// API: set pose
int command_set_pose(int ch, int findex, short* jpos)
{
    // Validate finger index
    assert(findex >= 0 && findex < NUM_OF_FINGERS);

    // Copy joint positions into a 16-bit array for clarity
    short pose[4];
    pose[0] = jpos[0];
    pose[1] = jpos[1];
    pose[2] = jpos[2];
    pose[3] = jpos[3];

    // Transmit position command for finger 'findex' (8 bytes)
    int Txid = ID_CMD_SET_POSE_1 + findex;
    return canSendMsg(ch, Txid, pose, sizeof(pose));
}


// API: set period
int command_set_period(int ch, short *period)
{
    can_period_msg_t msg{};
    if (period)
    {
        msg.position = period[0];
        msg.imu = period[1];
        msg.temp = period[2];
    }
    return canSendMsg(ch, ID_CMD_SET_PERIOD, &msg, sizeof(msg));
}

// API: set config
int command_set_device_id(int ch, unsigned char did)
{
    can_config_msg_t msg{};
    msg.set = 0x01;
    msg.did = did;
    msg.baudrate = 0;
    return canSendMsg(ch, ID_CMD_CONFIG, &msg, sizeof(msg));
}
int command_set_rs485_baudrate(int ch, unsigned int baudrate)
{
    can_config_msg_t msg{};
    msg.set = 0x02;
    msg.did = 0;
    msg.baudrate = baudrate;
    return canSendMsg(ch, ID_CMD_CONFIG, &msg, sizeof(msg));
}

// API: RTR requests
int request_hand_information(int ch)
{
    assert(ch >= 0 && ch < MAX_BUS);
    return canSentRTR(ch, ID_RTR_HAND_INFO);
}
int request_hand_serial(int ch)
{
    assert(ch >= 0 && ch < MAX_BUS);
    return canSentRTR(ch, ID_RTR_SERIAL);
}
int request_finger_pose(int ch, int f)
{
    assert(ch >= 0 && ch < MAX_BUS);
    return canSentRTR(ch, ID_RTR_FINGER_POSE + f);
}
int request_imu_data(int ch)
{
    assert(ch >= 0 && ch < MAX_BUS);
    return canSentRTR(ch, ID_RTR_IMU_DATA);
}
int request_temperature(int ch, int sindex)
{
    assert(ch >= 0 && ch < MAX_BUS);
    assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);
    return canSentRTR(ch, ID_RTR_TEMPERATURE + sindex);
}

// API: raw wrappers
int can_write_message(int ch, int id, int len, unsigned char *data)
{
    return canSendMsg(ch, id, data, len);
}
int can_read_message(int ch, int *id, int *len, unsigned char *data)
{
    uint8_t dlc;
    int rc = canReadMsg(ch, *id, dlc, data);
    *len = dlc;
    return rc;
}

CANAPI_END
