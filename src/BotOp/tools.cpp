#include "tools.h"

#include <Optim/NLP_Solver.h>
#include <Gui/opengl.h>
#include <MarkerVision/cvTools.h>
#include <Geo/depth2PointCloud.h>

bool Move_IK::go(){
    auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
    cout <<*ret <<endl;
    if(!ret->feasible){
        return false;
    }else{
        qT = komo.getPath_qOrg()[-1];
        C.setJointState(qT);
        if(askForOK && C.view(true, "Move_IK\ngo?")=='q') return false;

        bot.moveTo(qT, 2.);
        for(;bot.getTimeToEnd()>0.;) bot.sync(C);
    }
    return true;
}

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C, const char* camName, const char* blobName, const arr& hsvFilter, const arr& Pinv, int verbose){
    OpenGL disp;
    byteA img;
    floatA depth;
    bot.getImageAndDepth(img, depth, camName);
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter);
    if(verbose>1) disp.watchImage(img, false);
    if(verbose>0) LOG(0) <<"dot in image coords: " <<u;
    if(!u.N) return false;

    arr x;
    if(Pinv.N){
        makeHomogeneousImageCoordinate(u, img.d0);
        x = Pinv*u;
    }else{
        x = u;
        depthData2point(x, bot.getCameraFxypxy(camName));
    }
    if(verbose>0) LOG(0)  <<"dot in cam coords: " <<x <<endl;
    C[camName]->get_X().applyOnPoint(x);
    if(verbose>0) LOG(0)  <<"dot in world coords: " <<x <<endl;

    C[blobName]->setPosition(x);
    if(verbose>0 && C.view(true, "sense_HsvBlob\ngo?")=='q') return false;

    return true;
}
