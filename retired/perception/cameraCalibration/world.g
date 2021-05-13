#Include = '../../model/pandaStation.g'

Include = '../../model/pandaStation/pandaStation.g'

#frame calibrationMarkerL(L_panda_hand_1) {shape:sphere Q:<t(-0.03 0 0.035) d(180 1 0 0) d(-90 0 0 1)> size:[0.015] visual color:[1 0 0]}
#frame calibrationMarkerR(R_panda_hand_1) {shape:sphere Q:<t(0.03 0 0.035) d(180 1 0 0) d(-90 0 0 1)> size:[0.015] visual color:[1 0 0]}

frame calibrationVolumeR(base) { shape:ssBox size:[.35 .25 .3 .01] color:[.5 0 0 .2] Q:<t(0. 0.52 0.2) d(0 1 0 0)>, noVisual }
frame calibrationVolumeL(base) { shape:ssBox size:[.35 .25 .3 .01] color:[.5 0 0 .2] Q:<t(0. 0.52 0.2) d(0 1 0 0)>, noVisual }

#frame cameraField(base) { shape:ssBox size:[.8 .4 .6 .01] color:[0 0.5 0 .2] Q:<t(0.0 0.6 0.6) d(0 1 0 0)>, noVisual }
