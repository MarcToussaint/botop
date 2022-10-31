#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include "helpers.h"

#include <Perception/depth2PointCloud.h>
#include <Gui/opengl.h>
#include <iomanip>

void shiftRect(intA& rect, int dx, int dy, int H, int W){
  if(dx+(int)rect(0)>0) rect(0)+=dx; else rect(0)=0;
  if(dy+(int)rect(1)>0) rect(1)+=dy; else rect(1)=0;
  if(dx+(int)rect(2)<=(int)W) rect(2)+=dx; else rect(2)=W;
  if(dy+(int)rect(3)<=(int)H) rect(3)+=dy; else rect(3)=H;
}

void extendRect(intA& rect, int pad, int H, int W){
  if(rect(0)>pad)    rect(0)-=pad; else rect(0)=0;
  if(rect(1)>pad)    rect(1)-=pad; else rect(1)=0;
  if(rect(2)+pad<=W) rect(2)+=pad; else rect(2)=W;
  if(rect(3)+pad<=H) rect(3)+=pad; else rect(3)=H;
}

intA nonZeroRect(floatA& mask, double threshold){
  int yH=1, yL=mask.d0-1;
  int xH=1, xL=mask.d1-1;
  for(int y=0;y<(int)mask.d0;y++) for(int x=0;x<(int)mask.d1;x++){
    if(mask(y,x)>threshold){
      if(y>yH) yH=y;
      if(y<yL) yL=y;
      if(x>xH) xH=x;
      if(x<xL) xL=x;
    }
  }
  if(xH>xL && yH>yL){
    return ARRAY<int>(xL,yL,xH+1,yH+1);
  }
  return ARRAY<int>(0,0,0,0);
}

void recomputeObjMinMaxAvgDepthSize(std::shared_ptr<Object> obj){
  double D=0., S=0., Dmin=2., Dmax=0.;
  floatA depthTmp;
  for(int x=obj->rect(0);x<obj->rect(2);x++) for(int y=obj->rect(1);y<obj->rect(3);y++){
    float& m = obj->mask(y, x);
    float& d = obj->depth(y,x);

    if(d > .4){
      if(m > .5){
        if(d>Dmax) Dmax=d;
        if(d<Dmin) Dmin=d;
        depthTmp.insertInSorted(d); // TODO this is horribly inefficient, but a simple sort gives me memory errors?!?!?!?!?!
      }
      D += m*d;
      S += m;
    }
  }
  obj->depth_avg = D/S;
  obj->size = S;
  obj->depth_min = Dmin;
  obj->depth_max = Dmax;

  float lowerPercentile = 0.05;
  float upperPercentile = 0.1;
  uint indexL = floor(lowerPercentile*(float)depthTmp.N);
  uint indexU = floor(upperPercentile*(float)depthTmp.N);
  if(depthTmp.N) {
    floatA tmpA = depthTmp.sub(indexL, indexU);
    float smallestDepthFiltered = sum(tmpA)/float(tmpA.N);
    obj->depth_minFiltered = smallestDepthFiltered;
  } else {
    obj->depth_minFiltered = obj->depth_max;
  }
}

void create3DfromFlat(std::shared_ptr<Object> obj, NovelObjectType type, const arr& fxypxy){
  //get (top) center
  arr center = zeros(3);
  double sum=0.;
  for(int x=obj->rect(0);x<obj->rect(2);x++) for(int y=obj->rect(1);y<obj->rect(3);y++){
    float m = obj->mask(y,x);
    if(m>.5){
      float d = obj->depth(y,x);
      center(0) += m*x;
      center(1) += m*y;
      center(2) += m*d;
      sum += m;
    }
  }
  center /= sum;
  depthData2point(center, fxypxy);
  obj->pose.pos = center;

  if(type==OT_pcl){
    //-- translate dense depth to 3D point clound
    floatA _depth = obj->depth.sub(obj->rect(1), obj->rect(3)-1, obj->rect(0), obj->rect(2)-1);
    floatA _mask = obj->mask.sub(obj->rect(1), obj->rect(3)-1, obj->rect(0), obj->rect(2)-1);
    arr V;
    depthData2pointCloud(V, _depth, fxypxy(0), fxypxy(1), fxypxy(2)-obj->rect(0), fxypxy(3)-obj->rect(1));
    V.reshape(V.N/3, 3);
    uint maskN=0;
    for(uint i=0;i<_mask.N;i++) if(_mask.elem(i)>.5) maskN++;
    obj->mesh.clear();
    obj->mesh.V.resize(maskN, 3);
    for(uint i=0,labelN=0;i<_mask.N;i++) if(_mask.elem(i)>.5){
      memmove(&obj->mesh.V(labelN,0), &V(i,0), 3*V.sizeT);
      labelN++;
    }

  }else{

    arr polygon;
    if(obj->depth_max<.1) return;

    if(type==OT_box){
      //create box polygon
      polygon = arr(4,3,{obj->rect(0)-0., obj->rect(1)-0., obj->depth_max+.001,
                         obj->rect(2)-1., obj->rect(1)-0., obj->depth_max+.001,
                         obj->rect(2)-1., obj->rect(3)-1., obj->depth_max+.001,
                         obj->rect(0)-0., obj->rect(3)-1., obj->depth_max+.001});
      if(obj->rect(2)-obj->rect(0)<2 || obj->rect(3)-obj->rect(1)<2) polygon.clear();
    }else if(type==OT_poly){
      //create contour polygon
      polygon.resize(obj->polygon.d0, 3);
      for(uint i=0;i<polygon.d0;i++){
        polygon(i,0) = obj->polygon(i,0);
        polygon(i,1) = obj->polygon(i,1);
        polygon(i,2) = obj->depth_max; //cam_depth(obj->polygon(i,1), obj->polygon(i,0));
      }
    }else NIY;

    for(uint i=0;i<polygon.d0;i++) depthData2point(&polygon(i,0), fxypxy.p);

    //-- create mesh from polygon
    obj->mesh.clear();
    if(polygon.d0 > 2){
      obj->mesh.V.append(polygon);
      for(uint i=0;i<polygon.d0;i++) polygon(i,2) = -obj->depth_min+.001;
      obj->mesh.V.append(polygon);
      obj->mesh.makeConvexHull();
    }

    //-- set center
    if(obj->mesh.V.N){
//      obj->pose.pos = obj->mesh.center();
      obj->mesh.translate(-center);
//      obj->mesh.setSphere();
//      obj->mesh.scale(.1);
    }

  }

}

ptr<Object> createObjectFromPercept(const FlatPercept& flat,
                                    const byteA& labels,
                                    const byteA& cam_color, const floatA& cam_depth,
                                    const arr& cam_pose, const arr& fxypxy,
                                    NovelObjectType type){
  cv::Mat cv_cam_depth = CV(cam_depth);

  std::shared_ptr<Object> obj = make_shared<Object>();

  //-- 2D properties
  cv::Rect cv_rect(cv::Point(flat.rect(0),flat.rect(1)), cv::Point(flat.rect(2), flat.rect(3)));
  obj->rect = flat.rect;
  obj->polygon = flat.hull;
  obj->mask = convert<float>(labels==(byte)flat.label);
  obj->depth = cam_depth;
  obj->color = cam_color;

  //-- object's min, max, avg depth and size
  recomputeObjMinMaxAvgDepthSize(obj);

  //-- create object's 3D shape
  obj->pose.setZero();
  create3DfromFlat(obj, type, fxypxy);


  obj->mesh.clear();
  obj->mesh.C = id2color(obj->object_ID);
  //set the pose to be the camera pose:
  if(cam_pose.N) obj->pose = rai::Transformation(cam_pose) * obj->pose;

  return obj;
}



arr getPCLforLabels(PixelLabel label,
                    const byteA& labels, const floatA& cam_depth,
                    const arr& cam_pose, const arr& fxypxy){

  uint n=0;
  for(byte l:labels) if(l==label) n++;

  //-- translate dense depth to 3D point clound
  arr V(n,3);
  int H=cam_depth.d0, W=cam_depth.d1;
  n=0;
  for(int i=0;i<H;i++) for(int j=0;j<W;j++){
    if(labels(i,j)==label){
      V(n,0) = j;
      V(n,1) = i;
      V(n,2) = cam_depth(i,j);
      n++;
    }
  }
  CHECK_EQ(n, V.d0, "");

  for(uint i=0;i<V.d0;i++) depthData2point(&V(i,0), fxypxy.p);

  return V;
}



void Object::write(std::ostream& os) const{
  os <<std::setw(3) <<std::setprecision(2) <<object_ID <<':';
  os <<" D=[" <<std::setw(4) <<depth_min <<' ' <<std::setw(4) <<depth_avg <<' '<<std::setw(4) <<depth_max <<']';
  os <<" S=" <<std::setw(5) <<(int)size;
  os <<" age=" <<std::setw(3) <<age;
  os <<" h=" <<unhealthy;
}


void pixelColorNormalizeIntensity(byteA& img){
  byte* rgb;
  int r,g,b,sum,norm;
  for(uint i=0;i<img.N;i+=3){
    rgb = img.p+i;
    r = int(rgb[0]);
    g = int(rgb[1]);
    b = int(rgb[2]);
    sum=100+r+g+b;
    norm=100+128+128+128;
    r = (r*norm)/sum;
    g = (g*norm)/sum;
    b = (b*norm)/sum;
    if(r>255) r=255;
    if(g>255) g=255;
    if(b>255) b=255;
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
  }
}

void computePolyAndRotatedBoundingBox(intA& polygon, floatA& rotatedBBox, const floatA& mask){
  cv::Mat cv_mask = CV(mask);
//  cv::Rect cv_rect(cv::Point(rect(0),rect(1)), cv::Point(rect(2), rect(3)));
  cv::Mat cv_mask_crop = cv_mask;
  if(cv_mask_crop.total()<=10) return;

  //-- compute contours
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat bin = (cv_mask_crop >= .9f);
  cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  //-- preprocess contours
  uint C  = contours.size();
  if(!C) return;
  int largest=-1;
  std::vector<double> size(C);
  for(uint i=0; i<C; i++){
    size[i] = cv::contourArea(cv::Mat(contours[i]));
    if(largest<0 || size[i]>size[largest]) largest=i;
  }

  //-- compute rotated bb for largest
  cv::RotatedRect minRect;
  minRect = cv::minAreaRect( cv::Mat(contours[largest]) );
  cv::Point2f vertices[4];
  minRect.points(vertices);
  rotatedBBox = ARRAY<float>(minRect.center.x, minRect.center.y,
                             vertices[0].x, vertices[0].y,
                             vertices[1].x, vertices[1].y,
                             vertices[2].x, vertices[2].y,
                             minRect.angle);

  std::vector<cv::Point> contours_hull;
  cv::convexHull( cv::Mat(contours_hull), contours[largest], false );
  conv_pointVec_arr(polygon, contours_hull);
}

void conv_pointVec_arr(intA& pts, const std::vector<cv::Point>& cv_pts){
  pts.resize(cv_pts.size(), 2);
  for(uint j=0;j<pts.d0;j++){
    pts(j,0) = cv_pts[j].x;
    pts(j,1) = cv_pts[j].y;
  }

}

arr projectPointFromCameraToWorld(arr x, const arr& PInv) {
  x(0) *= x(2);
  x(1) *= x(2);
  // make homogeneous coordinate
  x.append(1.0);
  return PInv*x;
}

void determineObjectMainColor(std::shared_ptr<Object> obj, const arr& fixedColors) {
  cv::Mat cv_mask = CV(obj->mask);
  cv::Mat bin = (cv_mask >= .9f);
  cv::Mat convertedColor;
  cv::cvtColor(CV(obj->color), convertedColor, CV_BGR2Lab);
  cv::Scalar mean = cv::mean(convertedColor, bin);

  double minDistance = std::numeric_limits<double>::infinity();
  uint index = 0;
  for(uint i = 0; i < fixedColors.d0; i++) {
    double n = 0.0;
    for(uint j = 0; j < 3; j++) {
      n += (fixedColors(i,j)-mean(j))*(fixedColors(i,j)-mean(j));
    }
    if(n < minDistance) {
      minDistance = n;
      index = i;
    }
  }
  obj->colorIndex = index;
}

