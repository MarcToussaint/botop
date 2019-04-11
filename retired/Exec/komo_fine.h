#include <KOMO/komo.h>

//===============================================================================

struct KOMO_fineManip : KOMO{
  KOMO_fineManip(const rai::KinematicWorld& K) : KOMO(K){}

  void setFineGrasp(double time, const char *endeff, const char *object, const char* gripper);
  void setFinePlace(double time, const char *endeff, const char *object, const char* placeRef, const char* gripper);
  void setFinePlaceFixed(double time, const char *endeff, const char *object, const char *placeRef, const rai::Transformation& worldPose, const char* gripper);

  void setFineLift(double time, const char *endeff);
  void setFineHoming(double time, const char *gripper);

  void setIKGrasp(const char *endeff, const char *object);
  void setIKPlace(const char *object, const char* onto, const rai::Transformation &worldPose);
  void setIKPlaceFixed(const char *object, const rai::Transformation& worldPose);
  void setIKPush(const char* stick, const char* object, const char* table);

  void setGoTo(const arr& q, const StringA& joints, const char *endeff, double up=.2, double down=.8);

  void setIKAbstractTask(const StringA &facts, int verbose=0);
};
