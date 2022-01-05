# How to use BotOp

##  examples

* The tests 01, 03, and 04 are *not* examples for using BotOp
  (instead, they helped developing and testing underlying things)

* bin/bot/main.cpp is the prime example of minimalistic BotOp usage

* 05-moveTo:test_bot() is the simplest template for generating a
  move-to-endpose (with the `move(path, duration)` command, and with
  the `moveLeap(endpose, timingCost)` command)

* 06-fastPath is a standard example for sending a KOMO-computed path
  to the robot, then the duration is known/fixed (with the `move(path,
  duration)` command). The whole optitrack code could be removed. The
  example executed the same path multiple times, with increasing
  speed.

* 09-rndPoses is very similar to 06-fastPath: uses KOMO to compute a
  path, and send it, but this time with the `moveAutoTimed(path,
  maxVel)` command.

* 08-pnp also uses `moveAutoTimed` to send KOMO-computed paths. It is
  also an example (but not good code) to iterate through a sequence of
  phases, where depending on the phase, and open/close gripper command
  is send to the robot after executing the path, and an attach is done
  in the model configuration `C`.


## Commanding motion

* `BotOp` has currently 4 methods for commanding motions:
```
  double move(const arr& path, const arr& vels, const arr& times, bool override=false);
  double move(const arr& path, const arr& times, bool override=false);
  void moveAutoTimed(const arr& path, double maxVel=1., double maxAcc=1.); //double timeCost);
  double moveLeap(const arr& q_target, double timeCost=1.);
```

* All methods eventually call the first one `move(path, vels, times,
  override)`, which explicitly defines a cubic spline with given
  waypoints at given times. By default this is appended to the
  previously commanded motion; override with discared previously
  commanded motion and smoothly transition into the new spline.

  The provided path can be very coarse (e.g., just a single final
  waypoint), or fine.
  
  But this method is often inconvenient to call
  directly, as the waypoint vels and timing is unknown.

* `move(path, times)` uses a (small) optimization method to fill in the missing velocities and timings:
  * if timings={}, the method uses full timing optimization to get a timings and velocities of for all waypoints
  * if timings={duration} (a single number!), the timings are assumed to be a regular grid from 0,...,duration and timings are not optimized; only vels are filled in (almost trivially). *This is the default if you know the desired duration of the path*
  * otherwise, timings are fully specified and only vels are filled in
  
* `moveAutoTimed(path,maxVel,maxAcc)` uses a simple heuristic to
  decide on the total duration of the path: by computing max
  finite-difference vel and acc along the given path and time scaling
  the duration accordingly. Then `move(path,{duration})` is
  called. *This is the default if you don't want to fix the duration
  of the path.* Note, this method works only for fine resolution paths
  as it uses finite-differences (e.g. KOMO-comuted).

* Commanding motion to a single final pose `q` can now be done in
  several ways: `move(~q,{duration})` (the transpose makes a path from
  the vector), or `move(~q, {})` (I haven't tested this much..?), or
  finally `moveLeap(q, 1.)`. The last uses also uses timing
  optimization to decide on the optimal duration of the cubic spline
  from now to `q` (and should be equivalent the previous?)

## Main loop

* the `move` commands (also open/close) are non-blocking. Your main loop needs to check when commands are finished.

* The `getTimeToEnd` method returns the remaining time of the motion buffer (spline), and negative if done

* It is fully optional whether/when you want to keep your model configuration `C` in sync with the real robot during motion. For open loop execution that's not necessary. But just for display it is useful to keep `C` in sync and update its display. The `step(C)` method does that (bad naming, it should be called `syncModelAndUpdateDisplay). For convenience, `step(C)` also returns false when the user enters ESC/ENTER/q in the display or the motion is done.

## Executing longer sequences

* There is no good infrastructure/code yet for coordinating execution of several phases, perhaps even reactively... work in progress...

* The current recommendation is to iterate daft sense-plan-act:
  * Perceive objects (e.g., optitrack)
  * Use KOMO to compute keyframes
  * For each keyframe
	* Use KOMO to compute a path
    * command the path
    * wait for it to be finished, then open/close gripper, and accordingly attach in the model configuration (See 08-pnp example)
  * iterate

## Optitrack calibration

* The optitrack system itself should be calibrated with the metal angle in the center of the table, pointing forward-right from the robot perspective.

* 11-calibStation contains the code to calibrate the station. The three methods `collectData`, `computeCalibration`, and `demoCalibration` should be executed separately one-by-one, manually commenting the other methods

* `collectData` moves the robot along some waypoints and collects data throughout the path. The data includes the robot joint angles `q`, and the robot L/R gripper poses as perceived by optitrack (I think, using the zero optitrack_baseFrame??)

* `computeCalibration` computes optimal transforms (using 'morphology optimization') for
  * world -> optitrack_base
  * table_base -> r_panda_base (only XYPhi)
  * l_robotiq_base -> l_robotiq_optitrackMarker
  * r_robotiq_base -> r_robotiq_optitrackMarker
The console output of that should be copy-and-pasted into `rai-robotModels/scenarios/pandasTable-calibrated.g` (this is the default configuration read by `BotOp`!). E.g. it might look like this:
```
Include 'pandasTable.g'
optitrack_base (world) { Q:[0.00080153, -0.0714338, 0.599567, -0.999964, -0.00129352, -0.00254922, 0.00796131] }
Edit r_panda_base { Q:<[0.400543, -0.296918, 0, 0.708273, 0, 0, 0.705939]> }
Edit l_robotiq_optitrackMarker { Q:<[0.000345134, 0.047121, 0.00455253, 1, 0, 0, 0]> }
Edit r_robotiq_optitrackMarker { Q:<[-0.00329198, 0.0481341, 0.00473349, 1, 0, 0, 0]> }
```

* As a consequence, the l_panda_base will always be the reference with position `[-.4, -.3, 0]`. Everything else is calibrated relative to this.


## cfg parameters

* There should be a `git/botop/local.cfg` on your robot machine! That one is read *before* any local rai.cfg file. Depending on the robot station, it should have different parameters for useRobotiq and useArm and useOptitrack

* all `getParameter` from the code:
```
  bool useGripper = rai::getParameter<bool>("bot/useGripper", true);
  bool robotiq = rai::getParameter<bool>("bot/useRobotiq", true);
  rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "both");
  
  if(rai::getParameter<bool>("bot/useOptitrack", false)){
  mocap = libmotioncapture::MotionCapture::connect("optitrack", rai::getParameter<rai::String>("optitrack/host", "130.149.82.29").p);
  arr X = rai::getParameter<arr>("optitrack_baseFrame");

  Kp_freq = rai::getParameter<arr>("Franka/Kp_freq", ARR(20., 20., 20., 20., 10., 15., 10.));
  Kd_ratio = rai::getParameter<arr>("Franka/Kd_ratio", ARR(.6, .6, .4, .4, .1, .5, .1));
  friction = rai::getParameter<arr>("Franka/friction", zeros(7));

  noise_th = rai::getParameter<double>("botemu/noise_th", -1.);
```

## Calibrating joint frictions, debugging tracking error, gains

...later...
