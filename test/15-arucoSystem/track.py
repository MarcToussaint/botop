import robotic as ry

def testKomoTracker():
        C = ry.Config()
        C.addFile('/home/mtoussai/git/tests/calibration/station_reduced.g')

        bot = ry.BotOp(C, False)
        bot.launch_Basler(3)
        bot.launch_arucos()
        bot.launch_arucoObjTracker(C, "obj")

        while(True):
                key = bot.sync(C, .0)
                if key==ord('q'):
                        break


testKomoTracker()
