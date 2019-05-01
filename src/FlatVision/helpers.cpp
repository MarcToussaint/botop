#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include "helpers.h"

#include <Perception/depth2PointCloud.h>
#include <Gui/opengl.h>
#include <iomanip>

void shiftRect(uintA& rect, int dx, int dy, uint H, uint W){
  if(dx+(int)rect(0)>0) rect(0)+=dx; else rect(0)=0;
  if(dy+(int)rect(1)>0) rect(1)+=dy; else rect(1)=0;
  if(dx+(int)rect(2)<=(int)W) rect(2)+=dx; else rect(2)=W;
  if(dy+(int)rect(3)<=(int)H) rect(3)+=dy; else rect(3)=H;
}

void extendRect(uintA& rect, uint pad, uint H, uint W){
  if(rect(0)>pad)    rect(0)-=pad; else rect(0)=0;
  if(rect(1)>pad)    rect(1)-=pad; else rect(1)=0;
  if(rect(2)+pad<=W) rect(2)+=pad; else rect(2)=W;
  if(rect(3)+pad<=H) rect(3)+=pad; else rect(3)=H;
}

uintA nonZeroRect(floatA& mask, double threshold){
  uint yH=1, yL=mask.d0-1;
  uint xH=1, xL=mask.d1-1;
  for(uint y=0;y<mask.d0;y++) for(uint x=0;x<mask.d1;x++){
    if(mask(y,x)>threshold){
      if(y>yH) yH=y;
      if(y<yL) yL=y;
      if(x>xH) xH=x;
      if(x<xL) xL=x;
    }
  }
  if(xH>xL && yH>yL){
    return TUP(xL,yL,xH+1,yH+1);
  }
  return TUP(0,0,0,0);
}

void recomputeObjMinMaxAvgDepthSize(ptr<Object> obj){
  double D=0., S=0., Dmin=2., Dmax=0.;
  for(uint x=obj->rect(0);x<obj->rect(2);x++) for(uint y=obj->rect(1);y<obj->rect(3);y++){
    float& m = obj->mask(y, x);
    float& d = obj->depth(y,x);

    if(d>.1){
      if(m>.5){
        if(d>Dmax) Dmax=d;
        if(d<Dmin) Dmin=d;
      }
      D += m*d;
      S += m;
    }
  }
  obj->depth_avg = D/S;
  obj->size = S;
  obj->depth_min = Dmin;
  obj->depth_max = Dmax;
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
  create3DfromFlat(obj, type, flat.label, labels, cam_color, cam_depth, fxypxy);


  obj->mesh.clear();
  obj->mesh.C = id2color(obj->object_ID);
  //set the pose to be the camera pose:
  if(cam_pose.N) obj->pose.set(cam_pose);
  if(obj->mesh.V.N) obj->bbCenter = obj->mesh.getCenter().getArr();

  return obj;
}

void create3DfromFlat(ptr<Object> obj, NovelObjectType type, PixelLabel label,
                      const byteA& labels,
                      const byteA& cam_color, const floatA& cam_depth, const arr& fxypxy){
  if(type==OT_pcl){
    //-- translate dense depth to 3D point clound
    floatA _depth = cam_depth.sub(obj->rect(1), obj->rect(3)-1, obj->rect(0), obj->rect(2)-1);
    byteA _labels = labels.sub(obj->rect(1), obj->rect(3)-1, obj->rect(0), obj->rect(2)-1);
    arr V;
    depthData2pointCloud(V, _depth, fxypxy(0), fxypxy(1), fxypxy(2)-obj->rect(0), fxypxy(3)-obj->rect(1));
    V.reshape(V.N/3, 3);
    uint labelN=0;
    for(uint i=0;i<_labels.N;i++) if(_labels.elem(i)==label) labelN++;
    obj->mesh.clear();
    obj->mesh.V.resize(labelN, 3);
    for(uint i=0,labelN=0;i<_labels.N;i++) if(_labels.elem(i)==label){
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
  }

}


arr getPCLforLabels(PixelLabel label,
                    const byteA& labels, const floatA& cam_depth,
                    const arr& cam_pose, const arr& fxypxy){

  uint n=0;
  for(byte l:labels) if(l==label) n++;

  //-- translate dense depth to 3D point clound
  arr V(n,3);
  uint H=cam_depth.d0, W=cam_depth.d1;
  n=0;
  for(uint i=0;i<H;i++) for(uint j=0;j<W;j++){
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


void computeRotateBoundingBox(uintA& polygon, const byteA& img, const floatA& mask, const uintA& rect){
  cv::Mat cv_mask = CV(mask);
//  cv::Rect cv_rect(cv::Point(rect(0),rect(1)), cv::Point(rect(2), rect(3)));
  cv::Mat cv_mask_crop = cv_mask;
  if(cv_mask_crop.total()<=10) return;

  //-- compute contours
  std::vector<std::vector<cv::Point> > cv_contours;
  cv::Mat bin = (cv_mask_crop >= .9f);
  cv::findContours(bin, cv_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  //-- preprocess contours
  uint C  = cv_contours.size();
  if(!C) return;

  std::vector<std::vector<cv::Point> > contours_poly(C);
  std::vector<std::vector<cv::Point> > contours_hull(C);
  std::vector<double> size(C);
  for(uint i=0; i<C; i++){
    cv::approxPolyDP( cv::Mat(cv_contours[i]), contours_poly[i], 3, true );
//    cv::convexHull( cv::Mat(contours_poly[i]), contours_hull[i], false );
    size[i] = cv::contourArea(cv::Mat(contours_poly[i]));
  }

  //-- find largest
  int l=-1;
  for(uint i=0; i<C; i++) if(l<0 || size[i]>size[l]) l=i;

  //-- compute rotated bb for largest
  cv::RotatedRect minRect;
  minRect = cv::minAreaRect( cv::Mat(contours_poly[l]) );
  cv::convexHull( cv::Mat(contours_poly[l]), contours_hull[l], false );

  { //display
    cv::Mat cv_img = CV(img).clone();
    cv::Scalar colo(1,0,0);
    cv::drawContours( cv_img, contours_poly, l, colo, 2, 8);
    cv::drawContours( cv_img, contours_hull, l, colo, 2, 8);
    //rectangle( cv_color, boundRect[i].tl(), boundRect[i].br(), colo, 2, 8, 0 );
    //circle( cv_color, center[i], (int)radius[i], colo, 2, 8, 0 );

    //draw rotated rectangle
//    cv::Point2f rect_points[4];
//    minRect.points( rect_points );
//    for(int j=0; j<4; j++) cv::line(cv_img, rect_points[j], rect_points[(j+1)%4], colo, 1, 8 );

    cv::imshow("rotated BB", cv_img);
    cv::waitKey(1);
  }

  //-- assign to output polygon
#if 0
  cv::Point2f pts[4];
  minRect.points(pts);
  polygon.resize(4,2);
  for(uint i=0;i<4;i++){
    CHECK_LE(pts[i].x, 2*img.d1+1, "");
    CHECK_LE(pts[i].y, 2*img.d0+1, "");
    CHECK_GE(pts[i].x, -1., "");
    CHECK_GE(pts[i].y, -1., "");
    polygon(i,0) = pts[i].x;
    polygon(i,1) = pts[i].y;
  }
#else
  conv_pointVec_arr(polygon, contours_hull[l]);
#endif
}

void conv_pointVec_arr(uintA& pts, const std::vector<cv::Point>& cv_pts){
  pts.resize(cv_pts.size(), 2);
  for(uint j=0;j<pts.d0;j++){
    pts(j,0) = cv_pts[j].x;
    pts(j,1) = cv_pts[j].y;
  }

}
