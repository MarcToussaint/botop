import sys, os
sys.path.append(os.path.expanduser('~/git/botop/build'))
import libry as ry
import numpy as np
import time

ry.params_add({ 'physx/motorKp': 10000., 'physx/motorKd': 1000., "bot/useArm": "both"})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
C.view(False, 'this is your workspace data structure C -- NOT THE SIMULTATION')

bot = ry.BotOp(C, True)

bot.home(C)

del bot
