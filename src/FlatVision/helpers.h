#pragma once

#include <Core/array.h>
#include <Geo/geo.h>
#include <Geo/mesh.h>
#include <Kin/kin.h>

enum PixelLabel : byte { PL_unexplained=0x00, PL_nosignal=0x01, PL_noise=0x02, PL_toofar=0x03,
                         PL_background=0x10,
                         PL_robot=0x20,
                         PL_novelPercepts=0x40,
                         PL_objects=0x80,
                         PL_closeToObject=0xc0,
                         PL_max=0xff};

struct FlatPercept{
  PixelLabel label;
  double x=0.,y=0.;
  double radius=0.;
  uintA rect;
  uintA polygon;
};

struct Object{
  uint object_ID=0;
  rai::Frame *frame=0;
  PixelLabel pixelLabel;

  uint age=0;
  int unhealthy=0;

  //2D shape
  double size=0.;
  double depth_avg=0., depth_min=0., depth_max=0.;
  uintA rect;
  uintA polygon;
  floatA mask; //has size (rect(2),rect(3)); number \in[0,1] indicate where object should be
  floatA depth;
  byteA color;

  //3D shape
  arr bbCenter; //bounding box
  arr bbSize; //bounding box

  rai::Transformation pose=0;
  rai::Mesh mesh;

  void write(ostream& os) const;
};
stdOutPipe(Object)

enum NovelObjectType { OT_pcl, OT_box, OT_poly };

uintA nonZeroRect(floatA& mask, double threshold);

void shiftRect(uintA& rect, int dx, int dy, uint H, uint W);
void extendRect(uintA& rect, uint pad, uint H, uint W);

void computeRotateBoundingBox(uintA& polygon, const byteA& img, const floatA& mask, const uintA& rect);

void recomputeObjMinMaxAvgDepthSize(ptr<Object> obj);

void pixelColorNormalizeIntensity(byteA&);

ptr<Object> createObjectFromPercept(const FlatPercept& flat,
                                 const byteA& labels, const byteA& cam_color, const floatA& cam_depth,
                                 const arr& cam_pose, const arr& fxypxy,
                                 NovelObjectType type);

void create3DfromFlat(ptr<Object> obj, NovelObjectType type, PixelLabel label,
                      const byteA& labels, const byteA& cam_color, const floatA& cam_depth,
                      const arr& fxypxy);


arr getPCLforLabels(PixelLabel label,
                    const byteA& labels, const floatA& cam_depth,
                    const arr& cam_pose, const arr& fxypxy);
