#include <BotOp/bot.h>
#include <Core/graph.h>
#include <Livox/livox.h>
#include <Gamepad/gamepad.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <cstring> // for memcpy

#define ON_REAL true

void control_loop()
{
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/ranger/ranger_simplified.g"));
  BotOp bot(C, ON_REAL);
  GamepadInterface G;
  rai::Livox lidar;
  double timeCost = 1;
  double max_disp = .3;
  double max_disp_ang = .3;

  // For saving data
  std::time_t timestamp = std::time(nullptr);
  std::string data_dir = "log_data" + std::to_string(timestamp);
  std::filesystem::create_directory(data_dir);
  arr last_q = bot.get_q();

  while(1)
  {
    G.gamepadState.waitForNextRevision();
    if(G.quitSignal.get()) break;

    double x_rel = G.gamepadState.get()().elem(4) * -1;
    double y_rel = G.gamepadState.get()().elem(3) * -1;
    double phi_rel = G.gamepadState.get()().elem(5) * -1;
    if (x_rel < 0.2 && x_rel > -0.2) x_rel = 0.0;
    if (y_rel < 0.2 && y_rel > -0.2) y_rel = 0.0;
    if (phi_rel < 0.2 && phi_rel > -0.2) phi_rel = 0.0;

    arr current_q = bot.get_q();

    float theta = current_q.elem(2);
    double x_rel_rot = x_rel * cos(theta) - y_rel * sin(theta);
    double y_rel_rot = x_rel * sin(theta) + y_rel * cos(theta);

    arr rel_target = {x_rel_rot * max_disp, y_rel_rot * max_disp, phi_rel * max_disp_ang};
    arr target_q = current_q + rel_target;
    bot.moveTo(target_q, {timeCost}, true);
    bot.sync(C, 0.);
    lidar.pull(C);

    float d_x = current_q.elem(0) - last_q.elem(0);
    float d_y = current_q.elem(1) - last_q.elem(1);
    float eucl_dist = sqrt(d_x*d_x + d_y*d_y);
    float ang_diff = fmodf(fabsf(fmodf(current_q.elem(2) - last_q.elem(2) + M_PI, 2 * M_PI) - M_PI), 2 * M_PI);

    if (eucl_dist > .5 || ang_diff > M_PI*0.333333) {
      std::time_t timestamp = std::time(nullptr);
      std::string filename = data_dir + "/" + std::to_string(timestamp) + "pc.txt";

      std::ofstream file(filename);
      if (file.is_open()) {
        file << current_q.elem(0) << " " << current_q.elem(1) << " " << current_q.elem(2) << "\n";
        for (const auto& point : lidar.points_message.points) {
          file << point.x << " " << point.y << " " << point.z << "\n";
        }
        file.close();
      }
      last_q = bot.get_q();
    }
  }
}

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  control_loop();

  return 0;
}
