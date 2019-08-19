#pragma once

#include <Geo/mesh.h>
#include <Core/thread.h>
#include <Kin/kin.h>

#include "helpers.h"

struct ObjectManager{
  uint objIdCount=0;
  uint flatIdCount=0;
  uint changeCount=0;

  rai::Array<ptr<FlatPercept>> flats;
  Var<rai::Array<ptr<Object>>> objects;

  byteA flat_segments;
  floatA flat_depth;
  floatA flat_mask;
  byteA flat_color;

  ObjectManager(Var<rai::Array<ptr<Object>>>& _objects);
  ~ObjectManager();

  Object& getObject(PixelLabel pixelLabel);

  //-- flat world methods

  void renderFlatObject(int H, int W);

  void assignPerceptsToObjects(rai::Array<FlatPercept>& flats,
                               const byteA& labels);

  void injectNovelObjects(rai::Array<FlatPercept>& flats,
                          const byteA& labels, const byteA& cam_color, const floatA& cam_depth);

  void adaptFlatObjects(byteA& pixelLabels,
                        const byteA& cam_color, const floatA& cam_depth,
                        const uintA &cam_crop, const arr& cam_PInv, const floatA& background);

  void removeUnhealthyObject(rai::KinematicWorld& C);

  void syncWithConfig(rai::KinematicWorld& C);


  //-- display tools

  void displayLabelsAsPCL(PixelLabel label,
                          const byteA& labels, const floatA& cam_depth,
                          const arr& cam_pose, const arr& cam_fxypxy,
                          rai::KinematicWorld& config);

  void displayLabels(const byteA& labels,
                     const byteA& cam_color);

  void printObjectInfos();

};
