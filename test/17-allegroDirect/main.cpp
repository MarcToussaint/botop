#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // strtol
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <cctype>
#include <iostream>
#include <string>

#include "RockScissorsPaper.h"
#include "canAPI.h"
#include "canDef.h"
#include "rDeviceAllegroHandCANDef.h" // get MAX_DOF

#define PEAKCAN 1

// =================================================
// Global variable declarations
// =================================================

#include <BHand/BHand.h>

BHand *pBHand = nullptr;              // Pointer to BHand algorithm object
AllegroHand_DeviceMemory_t vars = {}; // Device memory structure for Allegro Hand

double q[MAX_DOF] = {0};       // Measured joint positions (radians)
double q_des[MAX_DOF] = {0};   // Desired joint positions (radians)
double tau_des[MAX_DOF] = {0}; // Computed joint torques
double cur_des[MAX_DOF] = {0}; // Clamped torque demands

static int can_handle = 0;        // CAN bus number
static bool ioThreadRun = false;  // Flag to control I/O thread execution
static pthread_t ioThread;        // I/O thread identifier
static double curTime = 0.0;      // Accumulated control loop time (seconds)
int recvNum = 0;                  // Count of received CAN messages
int sendNum = 0;                  // Count of sent CAN messages
static const double delT = 0.003; // Control loop period (seconds)

const bool RIGHT_HAND = true;           // True for right hand, false for left
const int HAND_VERSION = 4;             // Hand firmware version
const double tau_cov_const_v4 = 1200.0; // Torque conversion constant for v4 firmware

void ComputeTorque(); // Forward declaration of torque computation function

// =================================================
// CAN I/O thread function
// =================================================
static void *ioThreadProc(void *)
{
    int id, len;
    unsigned char data[8];
    unsigned char data_return = 0;

    printf("[I/O Thread] Start: waiting for CAN messages...\n");

    while (ioThreadRun)
    {
        while (can_read_message(can_handle, &id, &len, data) == 0)
        {
            switch (id)
            {
            case ID_RTR_HAND_INFO: {
                // Print hardware and firmware information

                printf("> CAN(%d): AllegroHand hardware version: 0x%02x%02x\n",
                       can_handle, data[1], data[0]);
                printf("                      firmware version: 0x%02x%02x\n", data[3],
                       data[2]);
                printf("                      hardware type: %d(%s)\n", data[4],
                       (data[4] == 0 ? "right" : "left"));
                printf("                      temperature: %d (celsius)\n", data[5]);
                printf("                      status: 0x%02x\n", data[6]);
                printf("                      servo status: %s\n",
                       (data[6] & 0x01 ? "ON" : "OFF"));
                printf("                      high temperature fault: %s\n",
                       (data[6] & 0x02 ? "ON" : "OFF"));
                printf("                      internal communication fault: %s\n",
                       (data[6] & 0x04 ? "ON" : "OFF"));
            }
            break;

            case ID_RTR_SERIAL:
                // Print serial number

                printf("[INFO] Serial: SAH0%d0 %c%c%c%c%c%c%c%c\n", HAND_VERSION,
                       data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                break;

            case ID_RTR_FINGER_POSE_1:
            case ID_RTR_FINGER_POSE_2:
            case ID_RTR_FINGER_POSE_3:
            case ID_RTR_FINGER_POSE_4: {
                int findex = id & 0x07; // Finger index
                // Read encoder counts for this finger
                for (int j = 0; j < 4; ++j)
                {
                    vars.enc_actual[findex * 4 + j] =
                        (short)(data[2 * j] | (data[2 * j + 1] << 8));
                }
                data_return |= (1 << findex);
                recvNum++;

                // Once all four fingers' data received
                if (data_return == 0x0F)
                {
                    // Convert encoder counts to radians
                    for (int i = 0; i < MAX_DOF; ++i)
                        q[i] = vars.enc_actual[i] * (333.3 / 65536.0) * (3.141592 / 180.0);

                    // Compute desired torques using BHand algorithm
                    ComputeTorque();

                    // Clamp torque demands between -1.0 and 1.0
                    for (int i = 0; i < MAX_DOF; ++i)
                    {
                        if (tau_des[i] > 1.0)
                            cur_des[i] = 1.0;
                        else if (tau_des[i] < -1.0)
                            cur_des[i] = -1.0;
                        else
                            cur_des[i] = tau_des[i];
                    }

                    // Send PWM torque demands for each finger segment
                    for (int i = 0; i < 4; ++i)
                    {
                        for (int j = 0; j < 4; ++j)
                            vars.pwm_demand[i * 4 + j] =
                                (short)(cur_des[i * 4 + j] * tau_cov_const_v4);
                        command_set_torque(can_handle, i, &vars.pwm_demand[4 * i]);
                    }
                    sendNum++;
                    curTime += delT;
                    data_return = 0;
                }
            }
            break;

            case ID_RTR_IMU_DATA:
                // Print IMU Roll, Pitch, Yaw
                {
                    printf(" Roll : 0x%02x%02x\n", data[0], data[1]);
                    printf(" Pitch: 0x%02x%02x\n", data[2], data[3]);
                    printf(" Yaw  : 0x%02x%02x\n", data[4], data[5]);
                }
                break;

            case ID_RTR_TEMPERATURE_1:
            case ID_RTR_TEMPERATURE_2:
            case ID_RTR_TEMPERATURE_3:
            case ID_RTR_TEMPERATURE_4: {
                int sindex = (id & 0x00000007); // Temperature sensor index
                // int celsius = (int)(data[0]) |
                //               (int)(data[1] << 8) |
                //               (int)(data[2] << 16) |
                //               (int)(data[3] << 24);
                printf("> Temperature[%d]: %d %d %d %d (celsius)\n",
                       sindex, data[0], data[1], data[2], data[3]);
            }
            break;

            default:
                printf("[WARN] Unknown ID(%d), len=%d\n", id, len);
            }
        }
        usleep(1000); // Sleep for 1ms before next read
    }
    printf("[I/O Thread] End\n");
    return nullptr;
}

// =================================================
// Print user instructions
// =================================================
void PrintInstruction()
{
    printf("--------------------------------------------------\n");
    printf("myAllegroHand: ");
    if (RIGHT_HAND)
        printf("Right Hand, v%i.x\n\n", HAND_VERSION);
    else
        printf("Left Hand, v%i.x\n\n", HAND_VERSION);

    printf("Keyboard Commands:\n");
    printf("H: Home Position (PD control)\n");
    printf("R: Ready Position (used before grasping)\n");
    printf("G: Three-Finger Grasp\n");
    printf("K: Four-Finger Grasp\n");
    printf("P: Two-Finger Pinch (Index–Thumb)\n");
    printf("M: Two-Finger Pinch (Middle–Thumb)\n");
    printf("E: Envelop Grasp (All fingers)\n");
    printf("A: Gravity Compensation (Prints current pose data)\n");
    printf("F: Servos OFF (Checks temperature; any grasp command reactivates them)\n");
    printf("1: Rock Motion (Defined in RockScissorsPaper.cpp)\n");
    printf("2: Scissors Motion (Defined in RockScissorsPaper.cpp)\n");
    printf("3: Paper Motion (Defined in RockScissorsPaper.cpp)\n");
    printf("S: Show Keyboard Commands List\n");
    printf("Q: Quit this program\n");
    printf("--------------------------------------------------\n\n");
}

// =================================================
// OpenCAN: parse and initialize CAN interface 'canX'
// =================================================
static bool OpenCAN(const char *ifname)
{
    if (strncmp(ifname, "can", 3) != 0)
    {
        fprintf(stderr, "[ERROR] Interface format 'canX' expected\n");
        return false;
    }
    long bus = strtol(ifname + 3, nullptr, 10);
    if (bus < 0)
    {
        fprintf(stderr, "[ERROR] Invalid bus: %s\n", ifname);
        return false;
    }
    can_handle = (int)bus;
    if (command_can_open(can_handle) < 0)
    {
        fprintf(stderr, "[ERROR] Failed to open CAN: can%d\n", can_handle);
        return false;
    }
    ioThreadRun = true;
    pthread_create(&ioThread, nullptr, ioThreadProc, nullptr);

    if (request_hand_information(can_handle) < 0 ||
        request_hand_serial(can_handle) < 0)
    {
        fprintf(stderr, "[ERROR] Initial information request failed\n");
        command_can_close(can_handle);
        return false;
    }

    short period[3] = {3, 0, 0};
    if (command_set_period(can_handle, period) < 0)
    {
        fprintf(stderr, "[ERROR] Failed to set period\n");
        command_can_close(can_handle);
        return false;
    }
    printf("[OpenCAN] Period pos=%d imu=%d temp=%d\n", period[0], period[1],
           period[2]);

    if (command_servo_on(can_handle) < 0)
    {
        fprintf(stderr, "[ERROR] Servo ON failed\n");
        command_set_period(can_handle, nullptr);
        command_can_close(can_handle);
        return false;
    }
    return true;
}

// =================================================
// CloseCAN: stop I/O and close CAN channel
// =================================================
static void CloseCAN()
{
    printf("[CloseCAN] Shutting down...\n");
    command_set_period(can_handle, nullptr);
    ioThreadRun = false;
    pthread_join(ioThread, nullptr);
    command_can_close(can_handle);
    printf("[CloseCAN] Done\n");
}

// =================================================
// Initialize BHand algorithm
// =================================================
static bool CreateBHandAlgorithm()
{
    printf("[Init] Creating BHand algorithm...\n");
    pBHand = RIGHT_HAND ? bhCreateRightHand() : bhCreateLeftHand();
    if (!pBHand)
    {
        fprintf(stderr, "[ERROR] BHand creation failed\n");
        return false;
    }
    pBHand->SetMotionType(eMotionType_NONE);
    pBHand->SetTimeInterval(delT);
    printf("[Init] Ready\n\n");
    return true;
}
// =================================================
// Destroy BHand algorithm
// =================================================
static void DestroyBHandAlgorithm()
{
    printf("[Exit] Destroying BHand algorithm\n");
    delete pBHand;
    pBHand = nullptr;
}

// =================================================
// Main command processing loop
// =================================================
static void MainLoop()
{
    bool run = true;
    std::string line;
    while (run && std::getline(std::cin, line))
    {
        if (line.empty())
            continue;
        char cmd = std::toupper((unsigned char)line[0]);
        switch (cmd)
        {
        case 'H':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_HOME);
            printf("[CMD] Home position set\n");
            break;
        case 'R':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_READY);
            printf("[CMD] Ready position set\n");
            break;
        case 'G':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_GRASP_3);
            printf("[CMD] Three-finger grasp\n");
            break;
        case 'K':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_GRASP_4);
            printf("[CMD] Four-finger grasp\n");
            break;
        case 'P':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_PINCH_IT);
            printf("[CMD] Index-thumb pinch\n");
            break;
        case 'M':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_PINCH_MT);
            printf("[CMD] Middle-thumb pinch\n");
            break;
        case 'E':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_ENVELOP);
            printf("[CMD] Envelop grasp\n");
            break;
        case 'A':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
            printf("[CMD] Gravity compensation\n");

            printf("> Encoder Count:\n");
            for (int f = 0; f < 4; f++)
            {
                request_finger_pose(can_handle, f);
                printf(" %6d %6d %6d %6d\n",
                       vars.enc_actual[f * 4 + 0], vars.enc_actual[f * 4 + 1],
                       vars.enc_actual[f * 4 + 2], vars.enc_actual[f * 4 + 3]);
            }
            printf("\n");
            usleep(10 * 1000); // 10ms

            printf("> Current Joint Positions:\n");
            for (int i = 0; i < MAX_DOF; i++)
            {
                double rad = q[i];
                double deg = q[i] * RAD2DEG;
                printf(" J[%2d]: %7.3f rad (%7.2f deg) \n", i, rad, deg);

                // Easy to copy and paste to Motion list (e.g. RockScissorsPaper.cpp)
                // printf("%7.3f,", rad);
                // if ((i + 1) % 4 == 0)
                //     printf("\n");
            }
            printf("\n");

            break;
        case 'F':
            if (pBHand)
                pBHand->SetMotionType(eMotionType_NONE);
            printf("[CMD] Servo OFF\n");
            for (int s = 0; s < 4; s++)
            {
                request_temperature(can_handle, s);
            }
            break;
        case '1':
            if (pBHand)
                MotionRock();
            printf("[CMD] Rock motion executed\n");
            break;
        case '2':
            if (pBHand)
                MotionScissors();
            printf("[CMD] Scissor motion executed\n");
            break;
        case '3':
            if (pBHand)
                MotionPaper();
            printf("[CMD] Paper motion executed\n");
            break;
        case 'Q':
            run = false;
            printf("[CMD] Quit command received\n");
            break;
        case 'S':
            PrintInstruction();
            break;
        default:
            printf("[CMD] Undefined Command '%c'\n", cmd);
        }
    }
}

// =================================================
// ComputeTorque: wrapper for BHand control update
// ===============================================
void ComputeTorque()
{
    if (!pBHand)
        return;
    pBHand->SetJointPosition(q);
    pBHand->SetJointDesiredPosition(q_des);
    pBHand->UpdateControl(0);
    pBHand->GetJointTorque(tau_des);
}

// =================================================
// main: entry point
// =================================================
int main(int argc, char **argv)
{
    // if (argc < 2)
    // {
    //     fprintf(stderr, "Usage %s canX\n", argv[0]);
    //     return 1;
    // }
    printf("[Start] AllegroHand Control starting...\n");
    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));

    if (!CreateBHandAlgorithm())
        return 1;
    if (!OpenCAN("can0"))
        return 1;
    PrintInstruction();
    MainLoop();
    CloseCAN();
    DestroyBHandAlgorithm();
    printf("[End] AllegroHand Control ended.\n");
    return 0;
}
