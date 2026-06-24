Include: '../../rai-robotModels/scenarios/pandasTable-calibrated.g'

stick (l_gripper){
    Q:<d(21 1 0 0) t(0 0 -.141)>
    shape:ssBox, size:[.015 .015 .5 .005] }

        stickTip (stick) { Q:[0 0 -.245], shape:sphere, size:[.008], color[1 1 .6 .3] }

puck (table){ Q:[.0 .4 .08]
            shape:ssCylinder size:[.06 .06 .005] color:[1 1 .6] }

target (table){ Q:[-.6 .3 .08]
            shape:ssCylinder size:[.06 .06 .005] color:[.6 1 .6] }
