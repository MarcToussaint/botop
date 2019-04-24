#include <Perception/opencv.h>

#include "objectManager.h"
#include "registrationCalibration.h"

#include <Kin/frame.h>

ObjectManager::ObjectManager(){
}

ObjectManager::~ObjectManager(){
}

Object& ObjectManager::getObject(PixelLabel pixelLabel){
  for(ptr<Object>& o:objects()) if(o->pixelLabel==pixelLabel) return *o;
  HALT("");
  return *objects().first();
}

bool sortByAvgHeight(const ptr<Object>& a, const ptr<Object>& b){
  return a->depth_avg < b->depth_avg;
}

void ObjectManager::explainFlatObjectPixels(byteA& pixelLabels,
                                            const byteA& cam_color, const floatA& cam_depth,
                                            const byteA& model_segments, const floatA& model_depth){
  auto O = objects.set();

  //-- initialize flat render buffers
  uint H=pixelLabels.d0, W=pixelLabels.d1;
  flat_segments.resize(H,W).setZero();
  flat_depth.resize(H,W)=2.;
  flat_mask.resize(H,W).setZero();
  flat_color.resize(H,W,3).setZero();

  //-- sort by height
  O().sort(sortByAvgHeight);

  //-- loop through objects
  for(std::shared_ptr<Object>& obj:O()){
#if 0
    //-- registration and calibration, including cam_mask!
    floatA cam_mask(H,W);
    cam_mask.setZero();
    for(uint i=0;i<cam_mask.N;i++){
      byte l = pixelLabels.elem(i);
      if(l==PL_background || l==PL_unexplained) cam_mask.elem(i)=1.;
    }
    auto reg = registrationCalibration(cam_color, cam_depth, cam_mask,
                                       obj->color, obj->depth, obj->mask, true, 20, 1);
//    cout <<"calib=" <<reg.calib <<" depthError=" <<reg.depthError <<" matchError=" <<reg.matchError <<endl;
    int dx = int(reg.calib(0)+.5), dy=int(reg.calib(1)+.5);

    clip(dx, -20, 20);
    clip(dy, -20, 20);

    //-- apply shift to object model
    shiftRect(obj->rect, dx, dy, H, W);
    cv::Mat cv_mask = CV(obj->mask);
    floatA M = rai::Array<float>(2,3,{1.f, 0.f, (float)dx, 0.f, 1.f, (float)dy});
    cv::warpAffine(cv_mask, cv_mask, CV(M), cv_mask.size() );
    //obj->mask.shift(dx+dy*W);
    obj->depth.shift(dx+dy*W);
    obj->color.shift(3*(dx+dy*W));
#endif

    //-- render into flat model and label pixels
    for(uint x=obj->rect(0);x<obj->rect(2);x++) for(uint y=obj->rect(1);y<obj->rect(3);y++){
//    for(uint x=0;x<W;x++) for(uint y=0;y<H;y++){
      float m = obj->mask(y, x);
      float d = obj->depth(y, x);
      byte* c = &obj->color(y, x,0);

      if(m > .5 && m>flat_mask(y,x) && d<flat_depth(y,x)){
        flat_segments(y,x) = obj->pixelLabel;
        flat_mask(y,x) = m;
        flat_depth(y,x) = d;
        memmove(&flat_color(y,x,0), c, 3);

        if(pixelLabels(y,x)==PL_unexplained){ //object MAY NOT explain background or noise
          pixelLabels(y,x) = obj->pixelLabel;
        }
      }
    }
  }

  //-- loop through objects to label 'closeToObject' pixels
  for(std::shared_ptr<Object>& obj:O()){
    //-- label close pixels as close
    uintA rect = obj->rect;
    extendRect(rect, 5, pixelLabels.d0, pixelLabels.d1);
    for(uint x=rect(0);x<rect(2);x++) for(uint y=rect(1);y<rect(3);y++){
      if(pixelLabels(y,x)==PL_unexplained){
        pixelLabels(y,x) = (PL_closeToObject|obj->pixelLabel);
      }
    }
  }
}

void ObjectManager::adaptFlatObjects(byteA& pixelLabels,
                                     const byteA& cam_color, const floatA& cam_depth,
                                     const byteA& model_segments, const floatA& model_depth,
                                     const arr& cam_fxypxy){
  auto O = objects.set();

  //-- loop through objects
  for(std::shared_ptr<Object>& obj:O()){

    obj->age++;
    double alpha=.5; //adaptation rate
    if(obj->age>10) alpha=.1;
//    if(obj->age>50) alpha=.01;

    //-- smooth the mask
    cv::Mat cv_mask = CV(obj->mask);
    cv::blur(cv_mask.clone(), cv_mask, cv::Size(3,3));
    uintA rect = obj->rect;
    extendRect(rect, 5, pixelLabels.d0, pixelLabels.d1);

    //-- adapt mask
    for(uint x=rect(0);x<rect(2);x++) for(uint y=rect(1);y<rect(3);y++){
      //mask
      float& m = obj->mask(y, x);
      if(pixelLabels(y,x)==obj->pixelLabel
         || pixelLabels(y,x)==(PL_closeToObject|obj->pixelLabel)){
        m = (1.-alpha)*m + alpha * 1.;
      }else if(pixelLabels(y,x)==PL_background || pixelLabels(y,x)==PL_noise){ //!= obj->pixelLabel is wrong: obj can be occluded by another... have an 'occluded mask' for every object??
        m = (1.-alpha)*m + alpha * 0.;
      }else{
        //don't adapt when PL_closeToObject to ANOTHER object
      }
    }

    //-- object rect
    obj->rect = nonZeroRect(obj->mask, .5);

    //-- adapt depth and color
    for(uint x=obj->rect(0);x<obj->rect(2);x++) for(uint y=obj->rect(1);y<obj->rect(3);y++){
      if(pixelLabels(y,x)==obj->pixelLabel){
        float& d = obj->depth(y,x);
        if(d<.1) d = cam_depth(y,x); //d was previously no signal depth
        else if(cam_depth(y,x)>.1) d = (1.-alpha)*d + alpha * cam_depth(y,x);
        for(uint i=0;i<3;i++){
          byte& c = obj->color(y,x,i);
          c = (1.-alpha)*c + alpha * cam_color(y,x,i);
        }
      }
    }

    //-- object's min, max, avg depth and size
    recomputeObjMinMaxAvgDepthSize(obj);

    //-- compute rotated bounding box
    computeRotateBoundingBox(obj->polygon, cam_color, obj->mask, obj->rect);

    create3DfromFlat(obj, OT_poly, obj->pixelLabel, pixelLabels, cam_color, cam_depth, cam_fxypxy);

    //-- is healthy or should be killed?
    if(obj->size<400.) obj->unhealthy++;
    else if(obj->unhealthy>0) obj->unhealthy--;
  }
}

rai::Frame *getFrame(rai::KinematicWorld& C, rai::Frame *frame_guess, const char* name){
  //find existing frame for this object?
  rai::Frame *f=frame_guess;
  if(f){
    CHECK_EQ(C.frames(f->ID), f, "");
    CHECK_EQ(f->name, name, "");
  }else{
    f = C.getFrameByName(name, false);
  }

  //need to create a new frame for this object
  if(!f) {
    f = new rai::Frame(C);
    f->name=name;
    new rai::Shape(*f);
    f->shape->type() = rai::ST_mesh;
    f->shape->visual = true;
    LOG(0) <<"creating new shape '" <<name <<"'";
  }
  return f;
}

void ObjectManager::injectNovelObjects(rai::Array<FlatPercept>& flats,
                                       const byteA& labels,
                                       const byteA& cam_color, const floatA& cam_depth,
                                       const arr& cam_pose, const arr& cam_fxypxy){

  if(changeCount>0){
    changeCount--;
    return;
  }

  auto O = objects.set();
  uint k=0;
  for(FlatPercept& flat : flats) if(O().N<3){
#if 1
    ptr<Object> obj = createObjectFromPercept(flat, labels, cam_color, cam_depth, cam_pose, cam_fxypxy, OT_box);
    obj->object_ID = objIdCount++;
    obj->pixelLabel = PixelLabel(PL_objects + obj->object_ID);
    O->append(obj);
    LOG(0) <<"novel object: " <<obj->object_ID <<" #O=" <<O().N;
#endif

    changeCount=30;
    k++;
  }
}

void ObjectManager::removeUnhealthyObject(rai::KinematicWorld& C){
  auto O = objects.set();

  //-- remove unhealth objects
  for(uint i=O().N;i--;){
    if(O().elem(i)->unhealthy>10){
      rai::Frame *f = O().elem(i)->frame; //C.frames.elem(O().elem(i)->frame_ID);
      C.frames.remove(f->ID);
      {
        uint i=0;
        for(rai::Frame *f: C.frames) f->ID = i++;
      }
      C.checkConsistency();
      cout <<"removing frame '" <<f->name <<"'" <<endl;
      O().remove(i);
      changeCount=30;
    }
  }
}

void ObjectManager::displayLabelsAsPCL(PixelLabel label, const byteA& labels, const floatA& cam_depth, const arr& cam_pose, const arr& cam_fxypxy, rai::KinematicWorld& config){
  arr V = getPCLforLabels(label, labels, cam_depth, cam_pose, cam_fxypxy);
  rai::Frame *f=getFrame(config, 0, STRING("pcl_"<<(int)label));
  f->shape->visual = true;
  f->X.set(cam_pose);
  cout <<V.N <<endl;
  f->shape->mesh().clear();
  f->shape->mesh().V = V;
  f->shape->mesh().C = ARR(.5, 1., 1.);
}



//if(&directSync){
//  ptr<Object> obj = createObjectFromPercept(flat, labels, cam_color, cam_depth, cam_pose, cam_fxypxy, OT_pcl);
//  obj->pixelLabel = PixelLabel(PL_objects + obj->object_ID);
//  rai::Frame *f=getFrame(directSync, 0, STRING("perc_"<<k));
//  f->shape->visual = false;
//  f->X = obj->pose;
//  f->shape->mesh() = obj->mesh;
//  f->shape->mesh().C = ARR(.5, 1., 1.);
//}


void ObjectManager::syncWithConfig(rai::KinematicWorld& C){

  auto O = objects.set();
  for(ptr<Object>& obj:O()){
    rai::Frame *f=getFrame(C, obj->frame, STRING("obj_"<<obj->object_ID));
    obj->frame = f;

    //update the frame's parameters
    f->X = obj->pose;
    f->shape->mesh() = obj->mesh;
    f->shape->mesh().C = ARR(1., .5, .0, .5);
    f->ats.getNew<int>("label") = PL_objects+obj->object_ID;
  }
}

void ObjectManager::displayLabels(const byteA& labels, const byteA& cam_color){
  cv::Mat cv_disp = CV(cam_color).clone();
  cv::Scalar col(255.,0.,0.);

  auto O=objects.set();
  for(ptr<Object>& obj:O()){
    std::stringstream text;
    text <<"obj " <<obj->object_ID;
    cv::rectangle(cv_disp, cv::Point(obj->rect(0), obj->rect(1)), cv::Point(obj->rect(2), obj->rect(3)), col, 2, 8, 0 );
    cv::putText(cv_disp, text.str(), cv::Point(obj->rect(0), obj->rect(1) - 2), cv::FONT_HERSHEY_PLAIN, 1., col);
    for(uint x=obj->rect(0);x<obj->rect(2);x++) for(uint y=obj->rect(1);y<obj->rect(3);y++){
      cv::Vec3b& rgb = cv_disp.at<cv::Vec3b>(cv::Point(x,y));
      float m = obj->mask(y, x);
      rgb *= 1.-m;
      rgb += m*cv::Vec3b(255,0,0);
    }

    cv::Scalar colo(0,0,1.);

    for(uint i=1;i<obj->polygon.d0;i++)
      cv::line(cv_disp,
               cv::Point(obj->polygon(i-1,0), obj->polygon(i-1,1)),
               cv::Point(obj->polygon(i,0), obj->polygon(i,1)), colo, 2, 8 );


  }

  cv::cvtColor(cv_disp, cv_disp, cv::COLOR_RGB2BGR);
  cv::imshow("ObjectManager", cv_disp);

  if(flat_color.N){
    cv::Mat cv_flat= CV(flat_color).clone();
    cv::cvtColor(cv_flat, cv_flat, cv::COLOR_RGB2BGR);
    cv::imshow("ObjectManager-FlatWorld", cv_flat);
  }

  cv::waitKey(1);
}

void ObjectManager::printObjectInfos(){
  cout <<"----- OBJECTS -----" <<endl;
  for(std::shared_ptr<Object>& obj:objects.set()()){ obj->write(cout); cout <<endl; }
}

