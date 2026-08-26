#include <Gui/opengl.h>
#include <Core/util.h>

#include <BotOp/bot.h>
#include <RealSense/RealSenseThread.h>
#include <Basler/BaslerThread.h>

// COMMAND LINE:   QT_QPA_PLATFORM=xcb ./pylonviewer

#ifdef RAI_BASLER

#include <pylon/PylonIncludes.h>

using namespace Pylon;

int direct(){
    OpenGL gl;
    byteA rgb;

    shared_ptr<CInstantCameraArray> cameras;
    CGrabResultPtr ptrGrabResult;

    CycleTimer tim;

    //-- open
    uint n=0;
    try
    {
        PylonInitialize();
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices( devices ) == 0)
        {
            throw RUNTIME_EXCEPTION( "No camera present." );
        }
        n = devices.size();
        cameras = make_shared<CInstantCameraArray>(n);
        for (size_t i = 0; i <n; ++i)
        {
            (*cameras)[i].Attach( tlFactory.CreateDevice( devices[i] ) );
            cout << "Using device: " << (*cameras)[i].GetDeviceInfo().GetModelName() << endl;
        }
        cout << endl;

        cameras->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    catch (const GenericException& e)
    {
        std::cerr << "An exception occurred." << endl
                  << e.GetDescription() << endl;
        PylonTerminate();
        return 1;
    }

    //-- step
    uint t=0;
    arr counts = zeros(n);
    double startTime = rai::clockTime();
    int key=0;
    while(cameras->IsGrabbing()){
        tim.tic(0);
        cameras->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException );
        tim.tic(1);
        if (ptrGrabResult->GrabSucceeded()) {
            int camera_id= ptrGrabResult->GetCameraContext();
            // cout << "Camera " << camera_id << ": " << (*cameras)[camera_id].GetDeviceInfo().GetModelName() << endl;
            counts(camera_id) += 1.;
            cout <<"iteration " <<t <<" img revisions: " <<counts <<" frame rates: " <<counts/(rai::clockTime()-startTime) <<endl;
            CPylonDataComponent imageDataComponent = ptrGrabResult->GetFirstImageDataComponent();
            uint H = imageDataComponent.GetHeight(), W=imageDataComponent.GetWidth();

            if(!rgb.N) rgb.resize(n, H, W);
            memmove(rgb.p+camera_id*W*H, imageDataComponent.GetData(), W*H);
            rgb.reshape(n*H,W);
            if(!(t%10)) key = gl.watchImage(rgb, false, .2);
            rgb.reshape(n,H,W);
            if(key=='q') break;
        }else{
            cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << endl;
        }
        tim.tic(2);
        t++;
        if(t>10000) break;
    }

    //-- close
    cameras.reset();
    PylonTerminate();

    cout <<"CYCLE TIMES: " <<tim.report() <<endl;

    return 0;
}

#endif

void thread(){

    uint n=3;
    rai::BaslerThread basler(n);
    rai::Array<OpenGL> gl(n);

    rai::wait(.5);

    CycleTimer tim;
    byteA buffer;
    double startTime = rai::clockTime();
    for(uint t=0;t<200;t++){
        tim.tic(0);

        rai::wait(.05);
        // basler.color(0).waitForNextRevision();

        tim.tic(1);

        int key=0;
        arr revs(basler.color.N);
        for(uint i=0;i<basler.color.N;i++) revs(i) = double(basler.color(i).getRevision());
        cout <<"iteration " <<t <<" img revisions: " <<revs <<" frame rates: " <<revs/(rai::clockTime()-startTime) <<endl;

        if(!(t%1)){
            for(uint i=0;i<n;i++){
                buffer = basler.color(i).get()();
                if(buffer.nd==3){
                    key = gl(i).watchImage(buffer, false, .5);
                }
                if(key=='q') break;
            }
        }

        tim.tic(2);

        if(key=='q') break;
    }
    basler.threadClose();
    cout <<"DISPLAY timer:   " <<tim.report() <<endl;
    cout <<"BASLER timer: " <<basler.timer.report() <<endl;

    // uint j=2;
    // for(uint i=0;i<n;i++){
    //     auto colorGet = basler.color(i).get();
    //     write_png(colorGet(), STRING("img-"<<i<<"-"<<j<<".png"), false);
    // }
}

void botop(){
    rai::Configuration C;
    C.addFile("scene.yml");

    rai::setParameter("botsim/verbose", 0);
    BotOp bot(C, false);

    bot.launch_Basler(3);
    bot.launch_arucos();

    // CycleTimer tim;
    byteA buffer;
    uint H, W;
    OpenGL gl;
    for(uint t=0;;t++){
        int key=0;
        if(!(t%10)){
            key = bot.sync(C, 0.);
            if(key=='q') break;
        }
        if(!(t%1)){
            bot.basler->color(0).waitForNextRevision();
            if(!buffer.N){
                auto g = bot.basler->color(0).get();
                H = g->d0; W=g->d1;
                buffer.resize(bot.basler->color.N, H*W*3).setZero();
            }
            if(buffer.N){
                for(uint i=0;i<bot.basler->color.N;i++) buffer[i] = bot.basler->color(i).get()();
                buffer.reshape(bot.basler->color.N*H, W, 3);
                key = gl.watchImage(buffer, false, .3);
                buffer.reshape(bot.basler->color.N, H*W*3);
                if(key=='q') break;
            }
        }
    }
}

int main(int argn, char** argv){
    // direct();
    // thread();
    botop();
    return 0;
}

// int main(int argn, char** argv){
//   arr x;
//   return 0;
// }
