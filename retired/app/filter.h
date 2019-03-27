#include <Core/thread.h>
#include <Kin/kin.h>
#include <Geo/geoms.h>

//===============================================================================

struct PerceptSimple;
typedef rai::Array<PerceptSimple*> PerceptSimpleL;

struct PerceptSimple : GLDrawer{
  enum Type { PT_cluster, PT_plane, PT_box, PT_mesh, PT_alvar, PT_optitrackmarker, PT_optitrackbody, PT_end };

  rai::GeomStore& store;
  int geomID = -1;
  int frameID = -1;
  rai::Transformation pose;
  double precision = 1.; //1 is maximum; decays as a precision in kalman filtering
  PerceptSimple() : store(_GeomStore()), pose(0) {}
  PerceptSimple(const rai::Geom& geom, const rai::Transformation& pose)
    : store(_GeomStore()), geomID(geom.ID), pose(pose) {
  }
  virtual ~PerceptSimple(){}

  virtual double fuse(PerceptSimple* other, double alpha=.2);
  virtual void write(ostream& os) const;
  virtual void glDraw(OpenGL& gl);
  virtual PerceptSimple* newClone() const{ return new PerceptSimple(*this); }

};
stdOutPipe(PerceptSimple)

//===============================================================================

struct FilterSimple : Thread{
  rai::KinematicWorld K;
  FrameL objects;
  PerceptSimpleL percepts_display;
  double time=0.;

  Var<PerceptSimpleL> percepts_input;     //perception sensors
  Var<PerceptSimpleL> percepts_filtered;
  Var<arr> currentQ;                      //joint/gripper/base sensors
  Var<double> currentGripper;
  Var<rai::Transformation> robotBase;

  Var<StringA> switches;                  //receive kinSwitch command from policy
  Var<double> timeToGo;                   //
  Var<rai::KinematicWorld> filterWorld;   //publish the filtered world?
  Var<bool> problemPerceived = {this, "problemPerceived"};

  FilterSimple(double dt=.01);
  ~FilterSimple(){
  }

  void open(){}
  void close(){}

  void step();
};

