#!/usr/bin/env python3

import robotic as ry
import numpy as np
import time
import cv2

class CalibrationScene:
    cams = []
    arucos = []
    aruco_ids = []
    obj = None

    Fxycxy = []
    Distortion = []

    def __init__(self, C: ry.Config, obj_name):
        self.C = C
        F = C.getFrames()

        for f in F:
            name: str = f.name
            i = ord(name[-1])-ord('0')
            atts = f.getAttributes()
            if name.startswith("camera_") and i>=0 and i<=9:
                self.cams.append(f)
                self.Fxycxy.append(atts["fxycxy"])
                self.Distortion.append(atts["distortion"])

            if "aruco_id" in atts:
                self.arucos.append(f)
                self.aruco_ids.append(int(atts["aruco_id"]))

        if obj_name is not None:
            self.obj = C.getFrame(obj_name)

    def report(self):
        print('cameras:', [f.name for f in self.cams])
        print('camera Fxycxy:', self.Fxycxy)
        print('camera Distortion:', self.Distortion)
        print('arucos:', [f.name for f in self.arucos])
        print('aruco_ids:', self.aruco_ids)

def get_random_pose(CS: CalibrationScene):
    gripper = CS.C.getFrame("r_gripper")
    komo = ry.KOMO(CS.C, 1, 1, 0, False)
    komo.addControlObjective([1.], 0, 1e-2)

    # pick random cam
    cam = np.random.choice(CS.cams)
    print('=== camera:', cam.name)
    offset = .1*np.random.randn(3)
    komo.addObjective([1.], ry.FS.positionRel, [gripper.name, cam.name], ry.OT.sos, [1., 1., 0.], offset)
    komo.addObjective([1.], ry.FS.positionRel, [cam.name, gripper.name], ry.OT.sos, [1., 0., 1.])
    komo.addObjective([1.], ry.FS.positionRel, [cam.name, gripper.name], ry.OT.ineq, [0., -1., 0.], [0., .3, 0.])

    sol = ry.NLP_Solver()
    sol.setProblem(komo.nlp())
    ret = sol.solve()
    print(ret)

    if(ret.feasible):
        CS.C.setJointState(ret.x)
        return ret.x
    else:
        print('--- FAILED ---')
        return None


def run(CS: CalibrationScene, real = False):
    ry.set_params({"botsim/verbose": 0, "bot/useAudio": True})
    C = CS.C

    if real:
        bot = ry.BotOp(C, True)
    else:
        bot = ry.BotOp(C, False)

    ncam = 3
    if ncam>0:
        bot.launch_Basler(ncam)
        V = ry.ImageViewer()
        # bot.launch_arucos()
        # bot.launch_arucoObjTracker(C, "obj")

    if real:
        bot.launch_robots(C, True)
        bot.hold(floating=True, damping=False)

    q_data = []

    c = 0
    while True:
        key = bot.sync(C, 0.)
        if key==ord('q'):
            break

        q = get_random_pose(CS)

        if q is None:
            continue

        bot.moveTo(q)
        key = bot.wait(C, True, True)
        if key==ord('q'):
            break

        bot.sound(9, .5)
        q = bot.get_q()

        rgbs = []
        for i in range(ncam):
            rgbs.append(bot.getImage(f'camera_{i}'))

        if ncam>0:
            rgb = np.hstack(rgbs)
            print('rgb shape: ', rgb.shape)
            key = V.view(rgb, zoom=.45)
            if key==ord('q'):
                break

        for i in range(ncam):
            cv2.imwrite(f'data/img_{c:04d}_{i}.png', rgbs[i])
            q_data.append(q)

            c += 1

    np.savetxt('data/q_data.txt', q_data)

    if real:
        bot.home(C)

if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)

    C = ry.Config()
    C.addFile("calibration_scene.g")
    CS = CalibrationScene(C, "obj")
    CS.report()

    run(CS, real=True)
