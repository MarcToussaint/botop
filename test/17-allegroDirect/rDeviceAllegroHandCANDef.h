#ifndef __RDEVICEALLEGROHANDCANDEF_H__
#define __RDEVICEALLEGROHANDCANDEF_H__

#define MAX_DOF 16

typedef struct tagDeviceMemory_AllegroHand
{
    int enc_actual[MAX_DOF];
    short pwm_actual[MAX_DOF];
    short pwm_demand[MAX_DOF];
} AllegroHand_DeviceMemory_t;

#endif // __RDEVICEALLEGROHANDCANDEF_H__
