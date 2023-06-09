#include <Kin/kin.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

#include <Optim/NLP_Solver.h>

#include <Gui/opengl.h>

#include <Perception/opencv.h>
#include <Geo/depth2PointCloud.h>

//===========================================================================

void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P){
  arr KR = P.sub(0,2,0,2);
  t = ~P.col(3);
  KR = inverse(KR);
  lapack_RQ(K, R, KR);
}

//===========================================================================

void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");

  C["bellybutton"]->name = "dot0";
  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];

  BotOp bot(C, false);

  rai::Graph data;

  uint nDots=5;
  for(uint d=0;d<nDots;d++){
    for(uint k=0;k<5; k++){
      rai::Frame* dot = C[STRING("dot" <<d)];

      KOMO komo;
      komo.setConfig(C, false);
      komo.setTiming(1., 1, 1., 0);
      komo.addControlObjective({}, 0, 1e-1);

      arr dotPos = .2*(rand(3)-.5);
      dotPos(2) -= .35;
      komo.addObjective({}, FS_positionRel, {dot->name, cam->name}, OT_eq, {1e0}, dotPos);
      komo.addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_ineq, {{1,3},{0.,0.,-1.}}, {0.,0.,.1});
      komo.addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_eq, {{2,3},{1.,0.,0.,0.,1.,0.}}, {});

      double wristAngle = rnd.uni(-2.3, 2.3);
      komo.addObjective({}, FS_qItself, {mount->name}, OT_eq, {1e0}, {wristAngle});

      auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
      cout <<dot->name <<' ' <<k <<' ' <<dotPos <<' ' <<wristAngle <<' ' <<*ret <<endl;
      if(!ret->feasible){
        k--;
        LOG(0) <<"RRRRRRRRRRRRRRRRRRRRRRRRRRREPEATING";
        continue;
      }

      bot.move(komo.getPath_qOrg(), {1.});
      for(;bot.getTimeToEnd()>0.;) bot.sync(C);

      bot.hold(false, true);
      for(uint t=0;t<2;t++) bot.sync(C);

      rai::Graph& dat = data.addSubgraph(dot->name+k);
      byteA img;
      floatA depth;
      bot.getImageAndDepth(img, depth, cam->name);
      dat.add("mountPose", mount->getPose());
      dat.add("camPose", cam->getPose());
      dat.add("camFxypxy", bot.getCameraFxypxy(cam->name));
      dat.add("dotPosition", dot->getPosition());
      dat.add("img", img);
      dat.add("depth", depth);
    }
  }

  data.write(FILE("dat.g"), "\n", 0, -1, false, true);
}

//===========================================================================

arr getHsvBlobImageCoords(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter){
  //blur
  cv::blur(rgb, rgb, cv::Size(3,3));

  //convert to BGR -> RGBPYBIND11_PYTHON_VERSION -> HSV
  cv::Mat hsv, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

  //find red areas
  cv::inRange(hsv,
              cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
              cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

  //find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  if(!contours.size()) return {};

  //get largest contour
  arr sizes(contours.size());
  for(uint i=0; i<contours.size(); i++) sizes(i) = cv::contourArea(cv::Mat(contours[i]));
  uint largest = argmax(sizes);

  //draw the contour interior into the mask
  mask = cv::Scalar(0);
  cv::drawContours(mask, contours, largest, cv::Scalar(128), cv::FILLED);

  // grab the depth values and mean x,y coordinates
  floatA depthValues;
  double objX=0.,objY=0.;
  for(int y=0;y<mask.rows;y++) for(int x=0;x<mask.cols;x++){
    if(mask.at<byte>(y,x)){
      float d = depth.at<float>(y,x);
      if(d>.1 && d<1.){
        depthValues.append(d);
        objX += x;
        objY += y;
      }
    }
  }

  arr blobPosition;
  if(depthValues.N>200.){
    objX /= double(depthValues.N);
    objY /= double(depthValues.N);

    // median
    double objDepth = depthValues.median_nonConst();
    // mean
//    double objDepth = sum(depthValues)/double(depthValues.N);

    if(objDepth>.1 && objDepth < 1.){ //accept new position only when object is in reasonable range
      // image coordinates
      blobPosition= {objX, objY, objDepth};


      // world coordinates
      //cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates
    }
  }

  if(rgb.total()>0 && depth.total()>0){
    if(contours.size()){
      cv::drawContours( rgb, contours, largest, cv::Scalar(0,0,255), 1, 8);
      cv::drawContours( depth, contours, largest, cv::Scalar(0), 1, 8);
    }
    cv::imshow("rgb", rgb);
    cv::imshow("depth", depth); //white=1meters
    cv::imshow("mask", mask);
    int key = cv::waitKey(1);
  }

  return blobPosition;
}

//===========================================================================

void computeCalibration(){
  rai::Graph data("dat.g");

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter");

  arr U(0,4), X(0,3);

  OpenGL disp;
  for(rai::Node *n:data){
    rai::Graph& dat = n->graph();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(dat.get<byteA>("img"));
    cv::Mat depth = CV(dat.get<floatA>("depth"));

    if(rgb.rows != depth.rows) continue;

    // blur
    arr u = getHsvBlobImageCoords(rgb, depth, hsvFilter);

    rai::Transformation mountPose(dat.get<arr>("mountPose"));
    rai::Transformation camPose(dat.get<arr>("camPose"));
    rai::Vector dotWorld(dat.get<arr>("dotPosition"));
    arr Fxypxy = dat.get<arr>("camFxypxy");

    // camera coordinates
    arr xCam=u;
    depthData2point(xCam, Fxypxy); //transforms the point to camera xyz coordinates

    arr dotCam = (dotWorld / camPose).getArr();
    //camPose.applyOnPoint(pos);

    arr x = (dotWorld / mountPose).getArr();

    cout <<"blob position: " <<xCam <<' ' <<dotCam <<' ' <<xCam-dotCam <<' ' <<Fxypxy <<endl;

    disp.watchImage(dat.get<byteA>("img"), false, 1.);

    //collect data
    u(1) = double(rgb.rows-1)-u(1);
    u(2) *= -1.;
    u(0) *= u(2);
    u(1) *= u(2);
    u.append(1.);
    U.append(u);
    X.append(x);
  }

  //-- multiple iterations
  arr Pinv, K, R, t;
  for(uint k=0;k<1;k++){
    Pinv = ~X * U * inverse_SymPosDef(~U*U);
    decomposeInvProjectionMatrix(K, R, t, Pinv);
    for(uint i=0;i<X.d0;i++){
      double ei = sqrt(sumOfSqr(X[i] - Pinv*U[i]));
      cout <<"   error on data " <<i <<": " <<ei;
      if(ei>.05){ X[i]=0.; U[i]=0.; cout <<" -- removed"; }
      cout <<endl;
    }
    double err = sqrt(sumOfSqr(U*~Pinv - X)/double(X.d0));
    cout <<"total ERROR = " <<err <<endl;
    if(err<.01) break;
  }

  //-- output
  rai::Transformation T;
  T.rot.setMatrix(~R);
  T.pos=t;
  arr Fxypxy = {K(0,0), K(1,1), K(0,2), K(1,2)};
  cout <<"*** total Pinv:\n" <<Pinv <<endl;
  cout <<"*** camera intrinsics K:\n" <<K <<endl;
  cout <<"*** camera Fxypxy :\n" <<Fxypxy <<endl;
  cout <<"*** camera world pose: " <<T <<endl;
  cout <<"*** camera world pos: " <<T.pos <<endl;
  cout <<"*** camera world rot: " <<T.rot <<endl;


}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  collectData();

  computeCalibration();

//  demoCalibration();

  LOG(0) <<" === bye bye ===\n";

  return 0;
}
