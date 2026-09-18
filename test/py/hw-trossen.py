# low-level hardware test

import robotic as ry
import numpy as np

C = ry.Config()
C.addFile("../20-trossen/scene.yml")
limits = C.getJointLimits()

bot = ry.BotOp(C, True, True)

# bot.hold(True, False)
# bot.wait(C, True, False)
# exit(0)

bot.home(C)

q0 = bot.get_qHome()
T = 10
path = np.tile(q0, (T, 1))
path += 0.3 * np.random.randn(*path.shape)
path[-1] = q0
path = np.clip(path, limits[0], limits[1])
bot.move(path, [.5*T])

bot.wait(C, True, False)
