#include "BaslerThread.h"

#if 1 //def RAI_PYLON

#include <pylon/PylonIncludes.h>

#ifdef RAI_OPENCV

#include <Perception/opencv.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

void cv_process(byteA& rgb, const byteA& bayer, bool downscale){
    static byteA buf;
    buf.resize(bayer.d0, bayer.d1, 3);
    cv::cvtColor(CV(bayer), CV(buf), cv::COLOR_BayerRG2BGR);

    if(downscale){
        rgb.resize(bayer.d0/2, bayer.d1/2, 3);
        // cv::pyrDown(CV(buf), CV(rgb));
        cv::resize(CV(buf), CV(rgb), cv::Size(rgb.d1, rgb.d0), 0, 0, cv::INTER_LINEAR);
    }else{
        rgb = buf;
    }
}

#endif

namespace rai {

struct sBaslerThread{
    std::shared_ptr<Pylon::CInstantCameraArray> cameras;
    Pylon::CGrabResultPtr ptrGrabResult;
    byteA bayer;
};


BaslerThread::BaslerThread(uint nCams)
    : Thread("BaslerThread"), color(nCams) {
  threadOpen(true);
  threadLoop();
}


BaslerThread::~BaslerThread(){
  LOG(0) <<"BASLER DTOR - cycle report: " <<timer.report();
  threadClose();
}

void BaslerThread::open() {
    s = new sBaslerThread;

    Pylon::PylonInitialize();
    Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devices;
    if (tlFactory.EnumerateDevices( devices ) == 0){
        throw RUNTIME_EXCEPTION( "No camera present." );
    }
    uint n = devices.size();
    if(color.N<n) n=color.N;
    LOG(0) <<"opening " <<n <<" basler cameras (" <<devices.size() <<" found)";
    s->cameras = make_shared<Pylon::CInstantCameraArray>( n );
    for(uint i=0; i<n; i++) {
        (*s->cameras)[i].Attach( tlFactory.CreateDevice( devices[i] ) );
        const Pylon::CDeviceInfo& info = (*s->cameras)[i].GetDeviceInfo();
        LOG(0) <<"Device " <<i <<" model: " << info.GetModelName() <<" serial numer: " <<info.GetSerialNumber() <<" name: " <<info.GetFullName();
    }

    s->cameras->StartGrabbing();
}

void BaslerThread::close(){
    s->cameras.reset();
    Pylon::PylonTerminate();
    rai::wait(.1);
    delete s;
}

void BaslerThread::step() {
    s->cameras->RetrieveResult( 5000, s->ptrGrabResult, Pylon::TimeoutHandling_ThrowException );
    if (s->ptrGrabResult->GrabSucceeded()) {
        intptr_t cameraContextValue = s->ptrGrabResult->GetCameraContext();
        // cout << "Camera " << cameraContextValue << ": " << (*s->cameras)[cameraContextValue].GetDeviceInfo().GetModelName() << endl;
        Pylon::CPylonDataComponent imageDataComponent = s->ptrGrabResult->GetFirstImageDataComponent();

        s->bayer.resize(imageDataComponent.GetHeight(), imageDataComponent.GetWidth()); //THIS IS  bayer!!
        memmove(s->bayer.p, imageDataComponent.GetData(), s->bayer.N);

        {
            auto rgb = color(cameraContextValue).set();
            cv_process(rgb(), s->bayer, true);
        }
    }

}

#else //BASLER

RealSenseCamera::RealSenseCamera(String serialNumber, bool captureColor, bool captureDepth) { NICO }

MultiRealSenseThread::MultiRealSenseThread(const StringA& cameraNames, bool captureColor, bool captureDepth) : Thread("MultiRealSenseThread"){ NICO }
MultiRealSenseThread::~MultiRealSenseThread() { NICO }
uint MultiRealSenseThread::getNumberOfCameras() { NICO }
void MultiRealSenseThread::open(){ NICO }
void MultiRealSenseThread::close(){ NICO }
void MultiRealSenseThread::step(){ NICO }

#endif

}
