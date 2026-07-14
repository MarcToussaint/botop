#include <pylon/PylonIncludes.h>
#include <Gui/opengl.h>
#include <Core/util.h>

#include <BotOp/bot.h>
#include <RealSense/RealSenseThread.h>
#include <Basler/BaslerThread.h>

using namespace Pylon;

int direct(){
    OpenGL gl;
    byteA rgb;

    shared_ptr<CInstantCameraArray> cameras;
    CGrabResultPtr ptrGrabResult;

    CycleTimer ct;

    //-- open
    try
    {
      PylonInitialize();
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices( devices ) == 0)
        {
            throw RUNTIME_EXCEPTION( "No camera present." );
        }
        cameras = make_shared<CInstantCameraArray>( devices.size() );
        for (size_t i = 0; i < cameras->GetSize(); ++i)
        {
            (*cameras)[i].Attach( tlFactory.CreateDevice( devices[i] ) );

            // Print the model name of the camera.
            cout << "Using device: " << (*cameras)[i].GetDeviceInfo().GetModelName() << endl;
        }
        cout << endl;

        cameras->StartGrabbing();
    }
    catch (const GenericException& e)
    {
      std::cerr << "An exception occurred." << endl
            << e.GetDescription() << endl;
	PylonTerminate();
        return 1;
    }

    //-- step

	while(cameras->IsGrabbing()){
	  ct.cycleStart();
            cameras->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException );
	    ct.cycleDone();
            if (ptrGrabResult->GrabSucceeded())
            {
                intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
                cout << "Camera " << cameraContextValue << ": " << (*cameras)[cameraContextValue].GetDeviceInfo().GetModelName() << endl;
                cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;
                CPylonDataComponent imageDataComponent = ptrGrabResult->GetFirstImageDataComponent();

		rgb.resize(imageDataComponent.GetHeight(), imageDataComponent.GetWidth()); //THIS IS  bayer!!
		memmove(rgb.p, imageDataComponent.GetData(), rgb.N);
		int key = gl.watchImage(rgb, false, 1.);
		if(key=='q') break;
            }
            else
            {
                cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

	//-- close
	cameras.reset();
    // Releases all pylon resources.
    PylonTerminate();

    cout <<"CYCLE TIMES: " <<ct.report() <<endl;

    return 0;
}

void thread(){

    OpenGL gl0, gl1;

    rai::BaslerThread basler(2);

    CycleTimer tim;
    for(;;){

        basler.color(1).waitForNextRevision();

        tim.cycleStart();
        int key=0;
        {
            auto colorGet = basler.color(0).get();
            key = gl0.watchImage(colorGet(), false, .25);
        }
        {
            auto colorGet = basler.color(1).get();
            key = gl1.watchImage(colorGet(), false, .25);
        }

        tim.cycleDone();

        if(key=='q') break;
    }
    cout <<"DISPLAY timer:   " <<tim.report() <<endl;
    cout <<"timer: " <<basler.timer.report() <<endl;
}

void botop(){
    rai::Configuration C;
    C.addFile("scene.yml");

    rai::setParameter("botsim/verbose", 0);
    BotOp bot(C, false);

    bot.launch_Basler(4);
    bot.launch_arucos();

    for(;;){
        int key = bot.sync(C);
        if(key=='q') break;
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
