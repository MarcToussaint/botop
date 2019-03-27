#include <Msg/MotionReference.h>
#include <Geo/geo.h>

void callTrajectoryService(const Msg_MotionReference& msg);

Msg_MotionReference planPath_IK(const StringA& cmd, const rai::Transformation& where=NoTransformation, bool fromCurrent=true);

void solveLGP();
