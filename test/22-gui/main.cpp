#include <BotOp/bot.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <Kin/frame.h>
#include <KOMO/pathTools.h>
#include <Kin/viewer.h>
#include <Geo/fclInterface.h>

//===========================================================================


double dist(rai::Frame* a, rai::Frame* b){
  CHECK_EQ(&a->C, &b->C, "");
  arr y = a->C.eval(FS_negDistance, {a->name, b->name});
  return -y.elem();
}

void test() {
  rai::Configuration C;
  C.addFile("../../../puzzle-scenes/scenes-30a/default.g");

  BotOp bot(C, false);
  bot.home(C);

  C.get_viewer()->setupEventHandler(true);
  // C.get_viewer()->ensure_gl().reportEvents=true;
  // C.view(true," this is the one");


  rai::Frame *ego = C.getFrame("ego");
  rai::Frame *grasped=0;
  LOG(0) <<"objects:";
  FrameL objs;
  for(rai::Frame* f: C.frames){
    if(f!=ego && f->getAts().findNode("logical")){
      objs.append(f);
      cout <<' ' <<f->name;
    }
  }
  cout <<endl;

  bot.wait(C);
  bool cursorLocked=false;
  bool stop=false;
  for(;!stop;){
    bot.sync(C, -1.);
    rai::wait(.02);

    {
      arr q, qDot;
      double ctrlTime;
      bot.getState(q, qDot, ctrlTime);

      // rai::Frame *f = C.get_viewer()->getEventCursorFrame();
      // if(f) cout <<"hovering over: " <<f->name <<endl;

      arr cursor = C.get_viewer()->getEventCursor();
      if(cursor.N==6){
        cursor.resizeCopy(2);
        arr delta = cursor - q;

	double l = length(delta);
	if(!cursorLocked && l<.2) cursorLocked=true;
	// else if(cursorLocked && l>2.)  cursorLocked=false;

	if(cursorLocked){
	  double cap = .5;
	  if(l>cap) delta *= cap/l;
	  q += delta;
	  bot.move(q.reshape(1,-1), {.1}, true, ctrlTime);
	}
      }
    }
    StringA events = C.get_viewer()->getEvents();

    for(str& e:events){
      // --- pick
      if(e=="key down space"){
        CHECK(!grasped, "");
        rai::Frame *nearest=0;
        double d_nearest=1e10;

	for(rai::Frame *f: objs) {
	  double d = dist(ego,f);
	  if(d<d_nearest){ nearest=f;  d_nearest=d; }
	}
	if(nearest){
	  LOG(0) <<"nearest: " <<nearest->name <<' ' <<d_nearest;
	  bot.attach("ego", nearest->name);
	  grasped=nearest;
	  grasped->setColor({1.,.5,0});
	}else{
	  LOG(0) <<"no nearest object found";
	}
      }
      if(e=="key up space"){
        if(grasped){
          bot.detach(grasped->name);
          grasped->setColor({.9});
          grasped=0;
        }
      }

      // --- glue
      if(e=="key down tab"){
        rai::Frame *nearest1=0, *nearest2=0;
        double d_nearest1=1e10, d_nearest2=1e10;

	for(rai::Frame *f:objs) {
	  double d = dist(ego,f);
	  if(d<d_nearest1){
	    if(d_nearest1<d_nearest2){  nearest2=nearest1;  d_nearest2=d_nearest1;  }
	    nearest1=f;  d_nearest1=d;
	  }else if(d<d_nearest2){  nearest2=f;  d_nearest2=d;  }
	}
	if(nearest1 && nearest2){
	  double d = dist(nearest1, nearest2);
	  LOG(0) <<"nearest objects: " <<nearest1->name <<' ' <<nearest2->name <<" dist: " <<d;
	  if(d<.1){
	    if(nearest1->parent == nearest2 || nearest2->parent == nearest1){
	      LOG(0) <<"are already glued!";
	    }else{
	      LOG(0) <<"gluing";
	      bot.attach(nearest1->name, nearest2->name);
	      nearest1->setColor({0.,.5,1.});
	      nearest2->setColor({0.,.5,1.});
	    }
	  }
	}else{
	  LOG(0) <<"no 2 nearest objects found";
	}
      }

      // --- quit
      if(e=="key down q"){
        stop=true;
        break;
      }
    }

  }
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  test();

  return 0;
}
