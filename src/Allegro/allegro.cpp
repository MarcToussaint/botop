#include "allegro.h"

#if 1//def RAI_ALLEGRO

#include <unistd.h>
#include <Control/CtrlMsgs.h>

#include "canAPI.h"
#include "canDef.h"
#include "rDeviceAllegroHandCANDef.h" // get MAX_DOF

#include <BHand/BHand.h>

//globals, as in original code
BHand *pBHand = nullptr;              // Pointer to BHand algorithm object
AllegroHand_DeviceMemory_t vars = {}; // Device memory structure for Allegro Hand
double q[MAX_DOF] = {0};       // Measured joint positions (radians)
double q_des[MAX_DOF] = {0};   // Desired joint positions (radians)
double tau_des[MAX_DOF] = {0}; // Computed joint torques
double cur_des[MAX_DOF] = {0}; // Clamped torque demands
static int can_handle = 0;        // CAN bus number
// static bool ioThreadRun = false;  // Flag to control I/O thread execution
// static pthread_t ioThread;        // I/O thread identifier
static double curTime = 0.0;      // Accumulated control loop time (seconds)
int recvNum = 0;                  // Count of received CAN messages
int sendNum = 0;                  // Count of sent CAN messages
static const double delT = 0.003; // Control loop period (seconds)
const bool RIGHT_HAND = true;           // True for right hand, false for left
const int HAND_VERSION = 4;             // Hand firmware version
const double tau_cov_const_v4 = 1200.0; // Torque conversion constant for v4 firmware

bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();

AllegroThread::AllegroThread(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state)
    : rai::RobotAbstraction(cmd, state), Thread("AllegroThread"){

  ctrlTime = 0.;
  t_prev = rai::clockTime();

  Kp = rai::getParameter<arr>("Allegro/Kp");
  Kd = rai::getParameter<arr>("Allegro/Kd");
  canName = "can0";
  LOG(0) << "Allegro: Kp:" << Kp<< " Kd:" << Kd;

  LOG(0) <<"Launching Allegro";
  CreateBHandAlgorithm();
  OpenCAN(canName); //calls threadLoop()
  rai::wait(.5);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
  pBHand->SetGainsEx(Kp.p, Kd.p);
  LOG(0) <<"..DONE" <<endl;
}

AllegroThread::~AllegroThread(){
  LOG(0) <<"shutting down Allegro -- " <<timer.report();
  CloseCAN(); //calls threadClose();
  DestroyBHandAlgorithm();
  LOG(0) <<"..DONE" <<endl;
}

void AllegroThread::step() {
  ioThreadProc();
}

void AllegroThread::publish_state_and_compute_torques(double* _q, double* _tau_des){

  //-- compute filters and velocity
  double t_now = rai::clockTime();
  double t_delta = t_now - t_prev;
  t_prev = t_now;
  q_real_prev = q_real;
  q_filtered_prev = q_filtered;

  q_real.setCarray(_q, 16);
  if(!q_filtered.N){ q_filtered=q_real; q_filtered_prev=q_real; }
  double alpha=.1;
  q_filtered = (1.-alpha)*q_filtered+alpha*q_real;

  q_vel = (q_filtered-q_filtered_prev)/t_delta;

  //-- publish state & INCREMENT CTRL TIME
  {
    auto stateSet = state.set();
    if(!stateSet->stall) stateSet->ctrlTime += t_delta; //HARD CODED: 1kHz
    else stateSet->stall--;
    ctrlTime = stateSet->ctrlTime;
    stateSet->q = q_real;
    stateSet->qDot = q_vel;
    stateSet->tauExternalIntegral.clear();
    stateSet->tauExternalCount++;
  }

  //-- get current ctrl command
  rai::ControlType controlType;
  arr q_ref, qDot_ref, qDDot_ref;
  {
    auto cmdGet = cmd.get();

    controlType = cmdGet->controlType;

    //get commanded reference from the reference callback (e.g., sampling a spline reference)
    if(cmdGet->ref){
      cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, q_vel, ctrlTime);
    }else{
      q_ref = q_real;
    }
  }

  cout <<ctrlTime <<" q_real: " <<q_real <<" q_ref: " <<q_ref <<endl;

  // //-- compute torques from control message depending on the control type
  // arr u;
  // u.resize(16).setZero();
  // u += Kp % (q_ref - q_filtered);
  // u += Kd % (qDot_ref - q_vel);

  // // u.setZero(); //DEBUG!!
  // u *= 0.01;
  // cout <<u <<endl;

  // //copy back
  // for(uint i=0;i<u.N;i++) _tau_des[i] = u.elem(i);

  pBHand->SetJointPosition(_q);
  pBHand->UpdateControl(0);
  pBHand->SetJointDesiredPosition(q_ref.p);
  pBHand->GetJointTorque(_tau_des);
}

bool CreateBHandAlgorithm()
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

void DestroyBHandAlgorithm()
{
  printf("[Exit] Destroying BHand algorithm\n");
  delete pBHand;
  pBHand = nullptr;
}

bool AllegroThread::OpenCAN(const char *ifname)
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
  // ioThreadRun = true;
  // pthread_create(&ioThread, nullptr, ioThreadProc, nullptr);

  threadLoop();

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
void AllegroThread::CloseCAN()
{
  printf("[CloseCAN] Shutting down...\n");
  command_set_period(can_handle, nullptr);
  // ioThreadRun = false;
  // pthread_join(ioThread, nullptr);
  threadClose();
  command_can_close(can_handle);
  printf("[CloseCAN] Done\n");
}


void AllegroThread::ioThreadProc(){
  int id, len;
  unsigned char data[8];
  unsigned char data_return = 0;

  // printf("[I/O Thread] Start: waiting for CAN messages...\n");

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
	    // ComputeTorque();
	    publish_state_and_compute_torques(q, tau_des);

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

#else //RAI_Allegro

AllegroThread::~AllegroThread(){ NICO }
void AllegroThread::init(uint _robotID, const uintA& _qIndices) { NICO }
void AllegroThread::step(){ NICO }

#endif
