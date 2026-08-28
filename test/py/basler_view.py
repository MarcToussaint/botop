#!/usr/bin/env python3

import robotic as ry
import numpy as np
import time

def run():
    C = ry.Config()
    ry.set_params({"botsim/verbose": 0})
    bot = ry.BotOp(C, False)

    bot.launch_Basler(3)
    V = ry.ImageViewer()

    while True:
        time.sleep(.001)

        rgb1 = bot.getImage('camera_0')
        rgb2 = bot.getImage('camera_1')
        rgb3 = bot.getImage('camera_2')

        if rgb1.shape == rgb2.shape and rgb1.shape==rgb3.shape:
            rgb = np.hstack((rgb1, rgb2, rgb3))
            print('rgb shapes: ', rgb1.shape, rgb2.shape, rgb3.shape)
            key = V.view(rgb, zoom=.45)
            if key==ord('q'):
                break


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)
    run()

