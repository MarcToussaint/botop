#pragma once

//low-level communication interface, for which there are multiple implementations (hidraw, hidapi, modbus)
struct SimplexMotion_Communication{
  enum RegType { uns16, uns32, int16, int32, string };

  SimplexMotion_Communication(const char* devPath = "/dev/hidraw0", unsigned short vendor_id = 0x04d8, unsigned short product_id = 0xf79a);
  ~SimplexMotion_Communication();

  void writeRegister(int regNumber, RegType regType, int data);
  int readRegister(int regNumber, RegType regType);
  const char* readString(int regNumber, int n);

private:
  unsigned char buf[64];
  // interface/protocol dependent - with several implementations
  struct SimplexMotion_Communication_Self* self=0;
  bool writeBuf(int len);
  bool readBuf(int len);
};

//===========================================================================

//registers

#define	VER_PARAMETERS			0x0106		// Version of this register set
#define	REG_VER_PARAMETERS		1
#define	REG_VER_FIRMWARE		2
#define	REG_VER_HARDWARE		3
#define	REG_MODEL_NAME			10
#define	REG_SERIAL_NUMBER		20
#define REG_USER_STRING1		30
#define REG_USER_STRING2		40
#define	REG_ADDRESS			50
#define REG_IDENTIFICATION		51
#define	REG_MODBUS_CONTROL		52
#define	REG_SUPPLY			100
#define	REG_TEMP_ELECTRONICS		101
#define	REG_TEMP_MOTOR			102
#define	REG_SPREADSPECTRUM		120
#define	REG_SPEED_FILTER		121
#define	REG_TRIMANGLE			122
#define REG_INPUT_POLARITY		140
#define	REG_INPUT_THRESHOLD		141
#define	REG_INPUT			145
#define REG_OUTPUT_CONTROL1		150
#define REG_OUTPUT_CONTROL2		151
#define REG_OUTPUT_CONTROL3		152
#define REG_OUTPUT_CONTROL4		153
#define	REG_OUTPUT1			160
#define	REG_OUTPUT2			161
#define	REG_OUTPUT3			162
#define	REG_OUTPUT4			163
#define	REG_ANALOG1			170
#define	REG_ANALOG2			171
#define	REG_ANALOG3			172
#define	REG_ANALOG4			173
#define REG_ENCODER_CONTROL		180
#define REG_ENCODER			184
#define REG_MAGNETIC_A			190
#define REG_MAGNETIC_B			191
#define	REG_MOTOR_POSITION		200
#define	REG_MOTOR_SPEED			202
#define REG_MOTOR_TORQUE		203
#define	REG_MOTOR_TORQUE_MAX		204
#define REG_MOTOR_TORQUE_STOP		205
#define REG_MOTOR_VD			206
#define REG_MOTOR_VQ			207
#define REG_MOTOR_ANGLE			208
#define	REG_CURR_IA			220
#define	REG_CURR_IB			221
#define	REG_CURR_ID			222
#define	REG_CURR_IQ			223
#define	REG_CURR_ID_FILTER		224
#define	REG_CURR_IQ_FILTER		225
#define	REG_CURR_ID_KP			226
#define	REG_CURR_ID_KI			227
#define	REG_CURR_IQ_KP			228
#define	REG_CURR_IQ_KI			229
#define REG_REG_KP			300
#define	REG_REG_KI			301
#define	REG_REG_KD			302
#define	REG_REG_LIMIT			303
#define	REG_REG_DELAY			304
#define	REG_REG_FRICTION		305
#define	REG_REG_INERTIA			306
#define	REG_REG_DEADBAND		307
#define	REG_REG_ERROR			308
#define	REG_REG_ERROR_MAX		309
#define	REG_REG_OUTPUT			310
#define	REG_RAMP_SPEED			350
#define	REG_RAMP_SPEED_MAX		351
#define	REG_RAMP_ACC			352
#define	REG_RAMP_ACC_MAX		353
#define	REG_RAMP_DEC_MAX		354
#define	REG_RAMP_JERK			355
#define REG_MODE			400
#define REG_STATUS			410
#define REG_STATUS_LATCHED		411
#define REG_STATUS_INPUTS		412
#define REG_MASK_QUICKSTOP		413
#define REG_MASK_SHUTDOWN		414
#define REG_ERROR			415
#define REG_STOP_CONFIG			416
#define REG_TIME			420
#define REG_TARGET_INPUT		450
#define REG_TARGET_SELECT		452
#define REG_TARGET_MUL			453
#define REG_TARGET_DIV			454
#define REG_TARGET_OFFSET		455
#define REG_TARGET_MIN			456
#define REG_TARGET_MAX			458
#define REG_TARGET_HYSTERESIS		460
#define REG_TARGET_FILTER		461
#define REG_TARGET_PRESENT		462
#define REG_HOME_SEQ1			480
#define REG_HOME_SEQ2			481
#define REG_HOME_SEQ3			482
#define REG_HOME_SEQ4			483
#define	REG_HOME_OFFSET			490
#define	REG_HOME_SPEED			491
#define	REG_HOME_ACC			492
#define REG_HOME_TORQUE			493
#define REG_HOME_DONE_MODE		494
#define REG_HOME_CHANGE			495
#define REG_APPL_CONTROL		600
#define REG_APPL_STATUS			601
#define REG_APPL_RUNTIME		602
#define REG_APPL_VERSION		603
#define REG_APPL_DATA1			620
#define REG_APPL_DATA2			621
#define REG_APPL_DATA3			622
#define REG_APPL_DATA4			623
#define REG_APPL_DATA5			624
#define REG_APPL_DATA6			625
#define REG_APPL_DATA7			626
#define REG_APPL_DATA8			627
#define REG_DEBUG1			640
#define REG_DEBUG2			641
#define REG_DEBUG3			642
#define REG_DEBUG4			643
#define REG_DEBUG5			644
#define REG_DEBUG6			645
#define REG_DEBUG7			646
#define REG_DEBUG8			647
#define REG_EVENT1_CONTROL		680
#define REG_EVENT2_CONTROL		681
#define REG_EVENT3_CONTROL		682
#define REG_EVENT4_CONTROL		683
#define REG_EVENT5_CONTROL		684
#define REG_EVENT6_CONTROL		685
#define REG_EVENT7_CONTROL		686
#define REG_EVENT8_CONTROL		687
#define REG_EVENT1_TRGREG		700
#define REG_EVENT2_TRGREG		701
#define REG_EVENT3_TRGREG		702
#define REG_EVENT4_TRGREG		703
#define REG_EVENT5_TRGREG		704
#define REG_EVENT6_TRGREG		705
#define REG_EVENT7_TRGREG		706
#define REG_EVENT8_TRGREG		707
#define REG_EVENT1_TRGDATA		720
#define REG_EVENT2_TRGDATA		721
#define REG_EVENT3_TRGDATA		722
#define REG_EVENT4_TRGDATA		723
#define REG_EVENT5_TRGDATA		724
#define REG_EVENT6_TRGDATA		725
#define REG_EVENT7_TRGDATA		726
#define REG_EVENT8_TRGDATA		727
#define REG_EVENT1_SRCREG		740
#define REG_EVENT2_SRCREG		741
#define REG_EVENT3_SRCREG		742
#define REG_EVENT4_SRCREG		743
#define REG_EVENT5_SRCREG		744
#define REG_EVENT6_SRCREG		745
#define REG_EVENT7_SRCREG		746
#define REG_EVENT8_SRCREG		747
#define REG_EVENT1_SRCDATA		760
#define REG_EVENT2_SRCDATA		761
#define REG_EVENT3_SRCDATA		762
#define REG_EVENT4_SRCDATA		763
#define REG_EVENT5_SRCDATA		764
#define REG_EVENT6_SRCDATA		765
#define REG_EVENT7_SRCDATA		766
#define REG_EVENT8_SRCDATA		767
#define REG_EVENT1_DSTREG		780
#define REG_EVENT2_DSTREG		781
#define REG_EVENT3_DSTREG		782
#define REG_EVENT4_DSTREG		783
#define REG_EVENT5_DSTREG		784
#define REG_EVENT6_DSTREG		785
#define REG_EVENT7_DSTREG		786
#define REG_EVENT8_DSTREG		787
#define REG_REC_STATE			900
#define REG_REC_TRIGGER			901
#define REG_REC_PERIOD			902
#define REG_REC_PRECEDING		903
#define REG_REC_OFFSET			904
#define REG_REC_CH1			905
#define REG_REC_CH2			906
#define REG_REC_CH3			907
#define REG_REC_CH4			908
#define REG_REC_MEM_CH1			1000
#define REG_REC_MEM_CH2			2000
#define REG_REC_MEM_CH3			3000
#define REG_REC_MEM_CH4			4000
