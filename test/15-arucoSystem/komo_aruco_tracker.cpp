#include "komo_aruco_tracker.h"

#include <Kin/frame.h>

KomoArucoTracker::KomoArucoTracker(rai::Configuration& C, str obj_name){
  //-- grab camera frames
  for(rai::Frame* f:C.frames){
    if(f->name.startsWith("cam")){
      CHECK_EQ(f->name, STRING("cam" <<cams.N), "cameras need to be enumerated consecutively");
      cams.append(f);
      Fxycxy.append(f->ats->get<arr>("fxycxy"));
      Distortion.append(f->ats->get<arr>("distortion"));
    }
  }

  obj = C.getFrame(obj_name);
  FrameL sub = obj->getSubtqree();
  for(rai::Frame* f:sub){
    if(f->name.startsWith("ar")){
      CHECK_EQ(f->name, STRING("ar" <<arucos.N), "FOR NOW arucos need to be enumerated consecutively");
      arucos.append(f);
    }
  }

  //add dots to each ar corner
  for(rai::Frame* ar:arucos){
    int aruco_id = ar->ats->get<double>("aruco_id");
    C.addFrame(STRING("arc_" <<aruco_id <<"_0"))->setShape(rai::ST_sphere, {.001}).setParent(ar).setRelativePosition({-.0175,+.0175,.0});
    C.addFrame(STRING("arc_" <<aruco_id <<"_1"))->setShape(rai::ST_sphere, {.001}).setParent(ar).setRelativePosition({+.0175,+.0175,.0});
    C.addFrame(STRING("arc_" <<aruco_id <<"_2"))->setShape(rai::ST_sphere, {.001}).setParent(ar).setRelativePosition({+.0175,-.0175,.0});
    C.addFrame(STRING("arc_" <<aruco_id <<"_3"))->setShape(rai::ST_sphere, {.001}).setParent(ar).setRelativePosition({-.0175,-.0175,.0});
  }
}

/*
  //-- setup calib frames
  FrameL calibs;
  arr calib_q;

//DANGER: this needs to be in right order, so that joint IDs match - TODO: make more robust
#if 1
  //add translational calibration joints to all arucos
  for(rai::Frame *ar:arucos){
    ar->insertPreLink();
    calibs.append(ar);
    cout <<" -- making stable dof: " <<ar->name <<endl;
    ar->setJoint(rai::JT_trans3);
    // ar->joint->isStable = true;
    // calib_q.append(ar->joint->getDofState());
  }
#endif
  arr q_org = C.getJointState();

  //add camera calibration joints
  for(rai::Frame* cam:cams){
    calibs.append(cam);
    cout <<" -- making stable dof: " <<cam->name <<endl;
    cam->setJoint(rai::JT_free);
    cam->joint->isStable = true;
    calib_q.append(cam->joint->getDofState());
  }

  cout <<C.getJointNames() <<endl;

  //================ create komo

  //-- find maxT
  uint maxT=0;
  for(rai::Node *d:data){ uint t = d->graph().get<int>("t"); if(t>maxT) maxT = t; }
  cout <<" maxT: " <<maxT <<endl;

  //-- setup KOMO, one slice for each datapoint
  KOMO komo(C, maxT+1, 1, 0, false);

  //-- add objectives for each data point
  for(rai::Node *n:data){
    rai::Graph& dat = n->graph();

    uint t = dat.get<int>("t");

    if(n->key=="con"){ //add a point view constraint
      arr p = dat.get<arr>("p");
      str arc = dat.get<str>("aruco_corner");
      int c = dat.get<int>("cam");

      if(undistort_points){
        arr P = arr{{3,3}, {Fxycxy(c,0), 0., Fxycxy(c,2), 0., Fxycxy(c,1), Fxycxy(c,3), 0., 0., 1.}};
        std::vector<cv::Point2d> p_in = { cv::Point2d{p(0), p(1)} };
        std::vector<cv::Point2d> p_out;
        cv::undistortImagePoints(p_in, p_out, CV(P), CV(Distortion[c]));
        p = arr{p_out[0].x, p_out[0].y};
      }

      // if(arc.endsWith("0")){
      komo.addObjective({t+1.}, make_shared<F_PointView>(p, Fxycxy[c]), { STRING("arc_"<<arc), cams(c)->name }, OT_sos, {1e2});
      // }

    }else if(n->key=="q"){  //set joint state configuration
      arr q = dat.get<arr>("q");
      q_org({7,14}) = q;
      if(!t){
        komo.setConfiguration_qOrg(t, (q_org, calib_q));
      }else{
        komo.setConfiguration_qOrg(t, q_org);
      }
    }else NIY;
  }

  //-- select dofs to be optimized
  {
    DofL dofs;
    if(C){
      for(auto c:cams) dofs.append(komo.timeSlices(0, c->ID)->joint);
    }
    if(calibrate_arucos){
      // for(auto ar:arucos) dofs.append(komo.timeSlices(0, ar->ID)->joint);
      for(auto ar:arucos){
        for(uint s=0;s<komo.timeSlices.d0;s++){
          rai::Joint * j = komo.timeSlices(s, ar->ID)->joint;
          if(j->active) dofs.append(j);
        }
      }
    }
    if(obj_name){
      for(uint s=0;s<komo.timeSlices.d0;s++){
        rai::Joint * j = komo.timeSlices(s, obj->ID)->joint;
        if(j->active) dofs.append(j);
      }
    }

    komo.pathConfig.selectJoints(dofs);

    //    dofs = komo.pathConfig.getDofs(komo.pathConfig.frames, true, false, false);
    cout <<"-- selected dofs: " <<endl;
    for(auto* d: dofs) cout <<d->frame->time <<' ' <<d->frame->name <<endl;
  }

  komo.addQuaternionNorms();

  // cout <<komo.report() <<endl;

  komo.run_prepare(0.);
  cout <<"== initial parameters (camera, dots): " <<komo.x <<endl;
  komo.view(true, "before optim");
  // komo.pathConfig.animate();
  komo.opt.animateOptimization = 1;

  rai::NLP_Solver sol;
  sol.setProblem(komo.nlp());
  sol.setInitialization(komo.x.copy());
  sol.opt->set_stopTolerance(1e-6);
  sol.opt->set_verbose(4);
  auto ret = sol.solve();
  cout <<komo.report(false) <<endl; //reports match per feature..
  cout <<"-- result: " <<*ret <<endl;
  cout <<"== optimized parameters (camera, dots): " <<ret->x <<endl;

  if(C){
    for(uint c=0;c<cams.N;c++) cout <<"   camera " <<c <<": " <<ret->x({7*c, 7*c+7}) <<endl;
  }

  //-- report positions
  //  for(uint t=0;t<komo.T;t++){
  //    rai::Frame *f = komo.timeSlices(t, viewPoint->ID);
  //    cout <<t <<' ' <<f->name <<f->getPosition() <<endl;
  //    for(rai::Frame *c:calibs){
  //      rai::Frame *f = komo.timeSlices(t, c->ID);
  //      cout <<t <<' ' <<f->name <<f->getPosition() <<endl;
  //    }
  //  }

  komo.view(true, "after optim");


  rai::CameraView V(komo.pathConfig);
  V.setCamera(komo.pathConfig.getFrame("cam0"));
  byteA rgb;
  floatA depth;
  V.computeImageAndDepth(rgb, depth);
  OpenGL gl;
  gl.watchImage(rgb, true);
}
*/
