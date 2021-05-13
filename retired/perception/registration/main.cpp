#include <Control/TaskControlThread.h>

#include <Gui/viewer.h>
#include <RealSense/RealSenseThread.h>

#include <FlatVision/explainBackground.h>
#include <FlatVision/explainNovels.h>
#include <FlatVision/registrationCalibration.h>
#include <FlatVision/helpers.h>
#include <Gui/opengl.h>
#include <Gui/viewer.h>

//===========================================================================

void online(){
  Var<floatA> cam_depth; //camera output
  Var<byteA> cam_color;  //camera output
  Var<arr> cam_pose;     //camera pose
  Var<arr> cam_Fxypxy;   //camera parameters

  Var<arr> regError;

  PlotViewer plot(regError);

  //-- camera thread
  auto cam = make_shared<RealSenseThread>(cam_color, cam_depth);
  cam_depth.waitForNextRevision();
  cam_Fxypxy.set() = cam->depth_fxypxy;
  cout <<"RS calib=" <<cam_Fxypxy.get()() <<endl;

  //methods
  ExplainBackground exBackground;
  ExplainNovelPercepts exNovel;

  OpenGL glNothing("INPUT", 100, 100);

  byteA labels;
  byteA _cam_color;
  floatA _cam_depth;
  arr _cam_fxypxy, _cam_pose;
  int cL=80,cR=110,cT=60,cB=100;

  for(uint k=0;;k++){
    cam_depth.waitForNextRevision();
    //    rai::wait(.5);
    cout <<k <<endl;

    //-- crop
    _cam_color = cam_color.get()->sub(cT,-cB-1,cL,-cR-1,0,-1);
    _cam_depth = cam_depth.get()->sub(cT,-cB-1,cL,-cR-1);
    _cam_fxypxy = cam_Fxypxy.get();
    _cam_pose = cam_pose.get();
    _cam_fxypxy(2) -= cL;
    _cam_fxypxy(3) -= cT;

    pixelColorNormalizeIntensity(_cam_color);

    exBackground.verbose=1;
    exBackground.compute(labels, _cam_color, _cam_depth);

    exNovel.verbose=1;
    exNovel.compute(labels, _cam_color, _cam_depth);

    int key=glNothing.update();
    if(key=='q') break;
  }
  glNothing.pressedkey=0;

  //extract object from cluster
  FlatPercept& flat = exNovel.flats.first();
  ptr<Object> obj = createObjectFromPercept(flat, labels, _cam_color, _cam_depth, _cam_pose, _cam_fxypxy, OT_box);

  cout <<"CREATED OBJECT: " <<*obj <<endl;

  for(uint k=0;;k++){
    cam_depth.waitForNextRevision();
//    rai::wait(.5);
    cout <<k <<endl;

    //-- crop
    _cam_color = cam_color.get()->sub(cT,-cB-1,cL,-cR-1,0,-1);
    _cam_depth = cam_depth.get()->sub(cT,-cB-1,cL,-cR-1);
    _cam_fxypxy = cam_Fxypxy.get();
    _cam_pose = cam_pose.get();
    _cam_fxypxy(2) -= cL;
    _cam_fxypxy(3) -= cT;

    pixelColorNormalizeIntensity(_cam_color);

    //-- registration
    auto reg = registrationCalibration(_cam_color, _cam_depth, floatA(),
                                       obj->color, obj->depth, obj->mask, false, 50, 1);
    cout <<"calib=" <<reg.calib <<" depthError=" <<reg.depthError <<" matchError=" <<reg.matchError <<endl;
    regError.set() = ARR(reg.matchError, reg.depthError);

    int key=glNothing.update();
    if(key=='q') break;
  }

  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc, char * argv[]){

  online();

  return 0;
}
