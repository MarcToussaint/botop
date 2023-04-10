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

enum PerceptStatus { PS_fresh=0, PS_merged, PS_unmerged };

struct FlatPercept{
  PixelLabel label;
  double x=0.,y=0.;
  double radius=0.;
  double size=0;
  intA rect;
  intA polygon;
  intA hull;
  PerceptStatus done=PS_fresh;
};

struct Object{
  uint object_ID=0;
  rai::Frame *frame=0;
  PixelLabel pixelLabel;

  uint age=0;
  int unhealthy=0;

  //2D shape
  double size=0.;
  double depth_avg=0., depth_min=0., depth_max=0., depth_minFiltered = 0.;
  intA rect;
  intA polygon;
  floatA rotatedBBox;  //(center, v1, v2, v3, angle)
  floatA mask; //has size (rect(2),rect(3)); number \in[0,1] indicate where object should be
  floatA depth;
  byteA color;

  //3D shape
  rai::Transformation pose=0;
  arr boxSize; //bounding box
  rai::Mesh mesh;

  uint colorIndex;

  void write(ostream& os) const;
};
stdOutPipe(Object)

enum NovelObjectType { OT_pcl, OT_box, OT_poly };

intA nonZeroRect(floatA& mask, double threshold);

void shiftRect(intA& rect, int dx, int dy, int H, int W);

void extendRect(intA& rect, int pad, int H, int W);

void computePolyAndRotatedBoundingBox(intA& polygon, floatA& rotatedBBox, const floatA& mask);

void recomputeObjMinMaxAvgDepthSize(std::shared_ptr<Object> obj);

void determineObjectMainColor(std::shared_ptr<Object> obj, const arr& fixedColors);

void pixelColorNormalizeIntensity(byteA&);

arr getPCLforLabels(PixelLabel label,
                    const byteA& labels, const floatA& cam_depth,
                    const arr& cam_pose, const arr& fxypxy);


arr projectPointFromCameraToWorld(arr x, const arr& PInv);

namespace cv{
  template<typename _T> class Point_;
  typedef Point_<int> Point2i;
  typedef Point2i Point;
}

void conv_pointVec_arr(intA& pts, const std::vector<cv::Point>& cv_pts);
