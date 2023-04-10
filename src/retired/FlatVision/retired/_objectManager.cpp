void ObjectManager::explainObjectPixels(byteA& pixelLabels,
                                        const byteA& cam_color, const floatA& cam_depth,
                                        const byteA& model_segments, const floatA& model_depth){
  auto O = objects.set();
  for(ptr<Object>& obj:O()){
    ExplainObject exObj;
    exObj.label = obj->pixelLabel;
    exObj.compute(pixelLabels, cam_color, cam_depth, model_segments, model_depth);

    //do something with exObj.calib!!
  }
}

bool ObjectManager::mergePerceptIntoObjects(FlatPercept& perc,
                                            const byteA& labels,
                                            const byteA& cam_color, const floatA& cam_depth,
                                            const byteA& model_segments, const floatA& model_depth){

  auto objSet = objects.set();

  //-- count pixel overlabs with model segmentation
  uint percN=0;
  uintA counts;
  counts.resize(PL_max+1).setZero();
  for(uint x=perc.rect(0);x<perc.rect(2);x++)
    for(uint y=perc.rect(1);y<perc.rect(3);y++)
      if(labels(y,x)==perc.label){
        percN++;
        byte l = flat_segments(y,x);
        if(l & PL_objects){
          counts(l)++;
        }
      }
  PixelLabel objPixelLabel = (PixelLabel)counts.maxIndex();
//  cout <<"bestObject=" <<std::hex <<objPixelLabel <<endl;
  if(objPixelLabel==PL_unexplained) return false;

  //-- not sufficient overlap? -> create novel object
  double overlap=double(counts(objPixelLabel))/double(percN);
//  cout <<"overlap=" <<overlap <<endl;
  if(overlap < 0.05){
    return false; //merge was no success
  }

  //-- we're fusing!
  Object& obj = getObject(objPixelLabel);

  //-- joint bounding box
  uintA rect(4);
  rect(0) = std::min(perc.rect(0), obj.rect(0));
  rect(1) = std::min(perc.rect(1), obj.rect(1));
  rect(2) = std::max(perc.rect(2), obj.rect(2));
  rect(3) = std::max(perc.rect(3), obj.rect(3));

  //-- percept mask
  floatA percMask = convert<float>(labels==(byte)perc.label);

  //-- compute overlab
  floatA andMask = percMask % obj.mask;
  float A = sum(andMask) / std::min(sum(percMask), sum(obj.mask));
  if(A<0.2){
    return false;
  }

  //-- registration and calibration with TWO masks!
  auto reg = registrationCalibration(cam_color, cam_depth, percMask, obj.color, obj.depth, obj.mask, true, 1);
  cout <<"calib= " <<reg.calib <<endl;


  floatA newMask = obj.mask;
  for(uint x=rect(0);x<rect(2);x++) for(uint y=rect(1);y<rect(3);y++){
    if(percMask(y, x)) newMask(y,x)=1.;
    else if(flat_segments(y, x)==obj.pixelLabel) newMask(y,x)=0.;
  }

  //-- copy back to object
  obj.rect = rect;

  //-- smoothed mask update
  float alpha=.5;
  obj.mask = (1.f-alpha) * obj.mask + alpha*newMask;

  //-- for now, just copy new depth and color... TODO: fusion
  obj.depth = cam_depth;
  obj.color = cam_color;

  return true; //merge was a success
}

void ObjectManager::createNewObjectFromPercept(FlatPercept& p,
                                               const byteA& labels,
                                               const byteA& cam_color, const floatA& cam_depth,
                                               const arr& cam_pose, const arr& cam_fxypxy){
  ptr<Object> obj = novelPerceptToObject(p, labels, cam_color, cam_depth, cam_pose, cam_fxypxy, OT_box);
  obj->object_ID = objIdCount++;
  obj->pixelLabel = PixelLabel(PL_objects + obj->object_ID);
  objects.set()->append(obj);
  LOG(0) <<"novel object: " <<obj->object_ID;
}

void ObjectManager::processNovelPercepts(rai::Array<FlatPercept>& flats,
                                         const byteA& labels,
                                         const byteA& cam_color, const floatA& cam_depth,
                                         const byteA& model_segments, const floatA& model_depth,
                                         const arr& cam_pose, const arr& cam_fxypxy){

  for(FlatPercept& p:flats){
    bool success = mergePerceptIntoObjects(p, labels, cam_color, cam_depth, model_segments, model_depth);

    //create novel object if it can't be merged
    if(!success){
      if(objIdCount<=0){
        createNewObjectFromPercept(p,labels, cam_color, cam_depth, cam_pose, cam_fxypxy);
      }
    }
  }

}

void ObjectManager::updateObjectPose(PixelLabel label, const arr& calib, const arr& cam_fxypxy){
  ptr<Object> obj;
  auto O = objects.set();
  for(ptr<Object>& o:O()) if(o->pixelLabel==label){ obj=o; break; }
  if(!obj){
    LOG(-1) <<"can't find object";
    return;
  }

  double step=.5;
  obj->pose.pos.x += step * calib(0)/cam_fxypxy(0);
  obj->pose.pos.y += step * calib(1)/cam_fxypxy(1);
  //    obj->pose.pos.z += 0.2*step * calib(0,2);
  //    obj->pose.rot.addX(0.2*step*calib(0,3));
  //    obj->pose.rot.addY(0.2*step*calib(0,4));
  //    cout <<"obj post =" <<obj->pose <<endl;
}

