#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

#include <LGP/optLGP.h>
#include <KOMO/komo.h>


void illustrate(){

  OpenGL gl("Red Ball Scenes", 1600, 800);

  uint N=7;
  rai::Array<rai::KinematicWorld> K(N);
  for(uint i=0;i<N;i++){
    K(i).init(STRING("problem-0"<<i+1<<".g"));
    K(i).orsDrawMarkers=false;
    gl.addView(i, glStandardScene, NULL);
    gl.addSubView(i, K(i));
    gl.views(i).camera.setDefault();
    gl.views(i).camera.focus(1., 0., .7);
    gl.views(i).text <<"problem " <<i+1;
  }
  gl.setSubViewTiles(4,2);

  gl.watch();
}


void solve1(){
  rai::KinematicWorld K("problem-01.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick");
  L.addAgent("baxterL");
  L.addAgent("baxterR");
  L.addFact({"table","table1"});
  L.addFact({"pusher", "stickTip"});
  L.addFact({"partOf", "stickTip", "stick"});

  OptLGP lgp(K, L);

//  lgp.verbose = 0;
//  rai::timerStart();
//  for(uint d=1;d<10;d++){
//    lgp.buildTree(d);
//    MNodeL all = lgp.root->getAll();
//    cout <<"d= " <<d <<" #= " <<all.N <<" t= " <<rai::timerRead(true) <<endl;
//  }

//  lgp.updateDisplay();
//  lgp.player();

//  lgp.optFixedSequence("(grasp baxterR stick) \
//                        (push stick stickTip redBall table1) \
//                        (grasp baxterL redBall) \
//                       ", true);

  lgp.run();

//  rai::wait();

//  lgp.renderToVideo();
}

void solve3(){
  rai::KinematicWorld K("problem-03.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick1");
  L.addObject("wall1");
  L.addAgent("baxterL");
  L.addAgent("baxterR");
  L.addFact({"table","table1"});
  L.addFact({"table","wall1"});
  L.addFact({"wall","wall1"});
  L.addFact({"pusher", "stick1"});

  OptLGP lgp(K, L);

  lgp.run();
//  lgp.optFixedSequence("(grasp baxterR stick1) \
//                       (push2 stick1 redBall table1) \
//                       (slideAlong stick1 redBall wall1) \
//                       (grasp baxterL redBall) \
//                       ", true);
////                       (place stick1 redBall wall1) \

  rai::wait();

  lgp.renderToVideo();
}

void solve4(){
  rai::KinematicWorld K("problem-04.g");
//  makeConvexHulls(K.frames);
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("block");
  L.addAgent("baxterL");
  L.addAgent("baxterR");

  OptLGP lgp(K, L);

  lgp.optFixedSequence("(grasp baxterR block) \
                       (grasp baxterL redBall) \
                       ", true, true);

  rai::wait();

  lgp.renderToVideo();
}

void solve5(){
  rai::KinematicWorld K("problem-05.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick");
  L.addObject("paperHandle");
  L.addFact({"pusher", "stick"});
  L.addObject({"paper"});

//  L.addFact({"partOf", "stickTip", "stick"});
  L.addFact({"table","table1"});
  L.addFact({"table","paper"});
  L.addAgent("baxterL");
  L.addAgent("baxterR");

  OptLGP lgp(K, L);

  lgp.optFixedSequence("(grasp baxterR stick) \
                       (push2 stick redBall table1) \
                       (place stick redBall paper) \
                       (grasp baxterL paperHandle) \
                       (place baxterR stick table1) \
                       (grasp baxterR redBall) \
                       ", true);
//                       (place baxterL paperHandle table1) \

  rai::wait();

  lgp.renderToVideo();

//  rai::KinematicWorld& Klast = *lgp.displayFocus->komoProblem(3)->configurations.last();
//  for(;;){
//    animateConfiguration(Klast);
//    Klast.watch(true);
//  }
}

//  solve3();
//  solve4();
void solve6(){
  rai::KinematicWorld K("problem-06.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick");
  L.addObject("box");
  L.addFact({"pusher", "stickTip"});
  L.addFact({"partOf", "stickTip", "stick"});
  L.addFact({"table","table1"});
  L.addFact({"table","tableR"});
  L.addFact({"table","tableL"});
  L.addFact({"table","box"});
  L.addAgent("baxterL");
  L.addAgent("baxterR");

  OptLGP lgp(K, L);

  lgp.optFixedSequence("(grasp baxterR stick) \
                       (push stick stickTip redBall box) \
                       (drop redBall box table1) \
                       (place world redBall table1) \
                       (push stick stickTip redBall table1) \
                       (grasp baxterL redBall) \
                       ", true);

  rai::wait();

  lgp.renderToVideo();
}

void solve7(){
  rai::KinematicWorld K("problem-07.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick");
  L.addObject("paperHandle");
  L.addFact({"pusher", "stick"});
  L.addObject({"paper"});

  L.addFact({"table","table1"});
  L.addFact({"table","paper"});
  L.addAgent("baxterL");
  L.addAgent("baxterR");

  OptLGP lgp(K, L);

  lgp.optFixedSequence("(slide baxterL paper table1) \
                       (place2 redBall paper) \
                       (place baxterL paper table1) \
                       (grasp baxterL paper) \
                       (grasp baxterR redBall) \
                       ", true);

                       /*
                                            (place baxterR stick table1) \
                                            (grasp baxterL paperHandle) \
                                            (place baxterL paperHandle table1) \
                        */
  rai::wait();

  lgp.renderToVideo();

//  rai::KinematicWorld& Klast = *lgp.displayFocus->komoProblem(3)->configurations.last();
//  for(;;){
//    animateConfiguration(Klast);
//    Klast.watch(true);
//  }
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  illustrate();
//  solve1();
//  solve3();
//  solve4();
//  solve5();
//  solve6();
  solve7();

  return 0;
}
