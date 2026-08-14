#include "BaslerThread.h"

#ifdef RAI_BASLER

#include <pylon/PylonIncludes.h>
#include <pylon/ImageFormatConverter.h>

#ifdef RAI_OPENCV

#include <Perception/opencv.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#endif

namespace rai {

struct sBaslerThread{
    std::shared_ptr<Pylon::CInstantCameraArray> cameras;
    std::shared_ptr<Pylon::CImageFormatConverter> converter;
    Pylon::CGrabResultPtr ptrGrabResult;
    byteA rgb;
    double startTime;
};


BaslerThread::BaslerThread(uint nCams)
    : Thread("BaslerThread"), color(nCams) {
  threadOpen(true);
  threadLoop();
}


BaslerThread::~BaslerThread(){
  LOG(0) <<"BASLER DTOR - " <<timer.report();
  threadClose();
}

void BaslerThread::open() {
    s = new sBaslerThread;

    Pylon::PylonInitialize();
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devices;
    CHECK(tlFactory.EnumerateDevices(devices)>=0, "No camera present");
    uint n = devices.size();
    if(color.N<n) n=color.N;
    LOG(0) <<"opening " <<n <<" basler cameras (" <<devices.size() <<" found)";
    s->cameras = make_shared<Pylon::CInstantCameraArray>(n);
    for(uint i=0; i<n; i++) {
        (*s->cameras)[i].Attach( tlFactory.CreateDevice( devices[i] ) );
        // (*s->cameras)[i].OutputQueueSize = 10;
        const Pylon::CDeviceInfo& info = (*s->cameras)[i].GetDeviceInfo();
        LOG(0) <<"Device " <<i <<" model: " << info.GetModelName() <<" serial numer: " <<info.GetSerialNumber() <<" name: " <<info.GetFullName();
    }

    s->cameras->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

    s->converter = std::make_shared<Pylon::CImageFormatConverter>();
    s->converter->OutputPixelFormat = Pylon::PixelType_RGB8packed;
    s->converter->OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
    s->startTime = rai::clockTime();
}

void BaslerThread::close(){
    s->cameras.reset();
    Pylon::PylonTerminate();
    arr revs(color.N);
    for(uint i=0;i<color.N;i++) revs(i) = double(color(i).getRevision());
    LOG(0) <<"frameCounts: " <<revs <<" frameRates: " <<revs/(rai::clockTime()-s->startTime);
    rai::wait(.1);
    delete s;
}

void BaslerThread::step() {
    s->cameras->RetrieveResult( 5000, s->ptrGrabResult, Pylon::TimeoutHandling_ThrowException );
    timer.tic(1);
    if (s->ptrGrabResult->GrabSucceeded()) {
        int camera_id = s->ptrGrabResult->GetCameraContext();
        // cout << "Camera " << cameraContextValue << ": " << (*s->cameras)[cameraContextValue].GetDeviceInfo().GetModelName() << endl;
        Pylon::CPylonDataComponent imageDataComponent = s->ptrGrabResult->GetFirstImageDataComponent();
        s->rgb.resize(imageDataComponent.GetHeight(), imageDataComponent.GetWidth(), 3);
        s->converter->Convert( s->rgb.p, s->rgb.N, imageDataComponent );
        timer.tic(2);

#if 0 //without resizing
        color(cameraContextValue).set() = s->rgb;
#else //with half resizing
        static byteA resized;
        resized.resize(s->rgb.d0/2, s->rgb.d1/2, 3);
        cv::resize(CV(s->rgb), CV(resized), cv::Size(resized.d1, resized.d0), 0, 0, cv::INTER_LINEAR);
        color(camera_id).set() = resized;
#endif
        timer.tic(3);
    }
}

#else //BASLER

  namespace rai{

    BaslerThread::BaslerThread(uint nCams)
  : Thread("BaslerThread"), color(nCams) { NICO }
BaslerThread::~BaslerThread(){ NICO }
void BaslerThread::open(){ NICO }
void BaslerThread::close(){ NICO }
void BaslerThread::step(){ NICO }

#endif

}
