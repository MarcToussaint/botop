# minimal trossen hardware test

useReal = False

import robotic as ry
import numpy as np
import matplotlib.pylab as plt


def park():
    C = ry.Config()
    C.addFile("trossen-scene.yml") # the ipAddress is defined in that file!

    bot = ry.BotOp(C, useReal)
    bot.moveTo(np.zeros(7))
    bot.wait(C)


def hold(floating=True):
    C = ry.Config()
    C.addFile("trossen-scene.yml") # the ipAddress is defined in that file!

    bot = ry.BotOp(C, useReal)
    bot.hold(floating=floating, damping=False)
    bot.wait(C, True, False)


def run():
    C = ry.Config()
    C.addFile("trossen-scene.yml") # the ipAddress is defined in that file!
    limits = C.getJointLimits()

    bot = ry.BotOp(C, useReal)

    # manual launching:
    # bot = ry.BotOp(C, useRealRobot=useReal, auto_launch=False)
    # bot.launch_simulation(C)
    # bot.launch_trossen(["0.0.0"], [])

    bot.setControllerWriteData(1) # continuously writes q real and ref to a file
    bot.home(C)

    q0 = bot.get_qHome()
    T = 10
    path = np.tile(q0, (T, 1))
    path += 0.3 * np.random.randn(*path.shape)
    path[-1] = q0
    path = np.clip(path, limits[0], limits[1])
    bot.move(path, [.5*T])

    bot.wait(C)


def plot():
    if useReal:
        X = np.loadtxt('z.trossen.dat')
    else:
        X = np.loadtxt('z.sim.dat')

    fig, ax = plt.subplots(1)
    ax.plot(X[:, 0], X[:, 1:8], label='real')
    ax.plot(X[:, 0], X[:, 8:16], label='ref')
    ax.set(xlabel='ctrl_time', ylabel='joint_angles')
    fig.legend()
    plt.show()


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)
    run()
    plot()
    # park()
    # hold(floating=True)
