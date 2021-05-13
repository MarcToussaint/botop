frame world { X:<t(0 0 0)> }

#box
frame box{ X:<t(0 0 .74)> }

frame (box){ Q:<t( .54 0 -.02)> shape:ssBox size:[.04 .78 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.54 0 -.02)> shape:ssBox size:[.04 .78 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .54 0 -.52)> shape:ssBox size:[.04 .78 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.54 0 -.52)> shape:ssBox size:[.04 .78 .08 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(0 -.41 -.02)> shape:ssBox size:[1.04 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0  .41 -.02)> shape:ssBox size:[1.04 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0  .41 -.52)> shape:ssBox size:[1.04 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0 -.41 -.52)> shape:ssBox size:[1.04 .04 .08 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(-.54 -.41 -.17)> shape:ssBox size:[.04 .04 .78 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .54 -.41 -.17)> shape:ssBox size:[.04 .04 .78 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.54  .41 -.27)> shape:ssBox size:[.04 .04 .58 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .54  .41 -.27)> shape:ssBox size:[.04 .04 .58 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(-.54 -.41 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t( .54  .41 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t(-.54  .41 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t( .54 -.41 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }

#base
frame base (box){ Q:<t(0 -.23 .10)> }
frame (base){ Q:<t(.01 .0  -.04) > shape:ssBox size:[1.12 .08 .08 .01] color:[.6 .6 .6] }
frame (base){ Q:<t(.0 -.12 -.04) > shape:ssBox size:[1.12 .08 .08 .01] color:[.6 .6 .6] }


#table
frame table(world){ Q:<t(0 .42 .769)> shape:ssBox size:[2. 1.2 .018 .005] color:[.4 .3 .2] contact logical:{tableS}}

#camera stand
frame camstand (box){ Q:<t(0 -.35 .4)> }
frame camstand1 (camstand){ Q:<t(-.05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame camstand2 (camstand){ Q:<t( .05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame camstand3 (camstand){ Q:<t(-.05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame camstand4 (camstand){ Q:<t( .05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }

frame camstand5 (camstand){ Q:<t(-.05 .36 .9)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame camstand6 (camstand){ Q:<t( .05 .36 .9)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame camstand7 (camstand){ Q:<t(-.05 .36 .8)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame camstand8 (camstand){ Q:<t( .05 .36 .8)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }

frame camstand8 (camstand){ Q:<t(-.05 .77 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }
frame camstand10 (camstand){ Q:<t( .05 .77 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }

frame camstand11 (camstand){ Q:<t(-.05 .12 .6) d(-30 1 0 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }
frame camstand12 (camstand){ Q:<t( .05 .12 .6) d(-30 1 0 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }

frame camera(world){ Q:<t(-0.0141646 0.330469 1.81481)> shape:marker, size:[.1], focalLength:0.895, width:640, height:360, zRange:[.1 10] } #-0.0107846 0.35387 1.76293 #0 .33 1.789 # -0.0146756 0.355877 1.77138

#-0.014799 0.332922 1.80108
#-0.0143835 0.333404 1.80027

frame tableMarker(world){ Q:<t(0 0 .778)> shape:marker, size:[.1], color=[1 0 0] }

Prefix = "L_"
Include = 'panda.g'

Prefix = "R_"
Include = 'panda.g'

Edit L_panda_link0 (base) { joint:rigid Q:<t(-.3 0 0) d(90 0 0 1)> }
Edit R_panda_link0 (base) { joint:rigid Q:<t( .3 0 0) d(90 0 0 1)> }

#Edit L_panda_joint1 { q= -.5 }
#Edit R_panda_joint1 { q=  .5 }
Edit L_panda_joint2 { q= -.5 }
Edit R_panda_joint2 { q= -.5 }
Edit L_panda_joint4 { q= -1.7 }
Edit R_panda_joint4 { q= -1.7 }
#Edit L_panda_joint7 { q= 1.0 }
#Edit R_panda_joint7 { q= 1.0 }


#Edit L_panda_leftfinger_1 { visual!, noVisual }
#Edit L_panda_rightfinger_1 { visual!, noVisual }
#Edit R_panda_leftfinger_1 { visual!, noVisual }
#Edit R_panda_rightfinger_1 { visual!, noVisual }


frame endeffR (R_panda_hand_1) { Q:<t(0 0 .103) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] logical:{gripper}}
frame endeffL (L_panda_hand_1) { Q:<t(0 0 .103) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] logical:{gripper}}

#frame endeffL_fingerL (L_panda_rightfinger_1) { Q:<t(0 0 0.0446) d(180 1 0 0) d(90 0 0 1)> shape:marker size:[.02] color:[0 0 1] }
#frame endeffL_fingerR (L_panda_leftfinger_1) { Q:<t(0 0 0.0446) d(180 1 0 0) d(90 0 0 1)> shape:marker size:[.02] color:[0 0 1] }

#frame endeffR_fingerL (R_panda_rightfinger_1) { Q:<t(0 0 0.0446) d(180 1 0 0) d(90 0 0 1)> shape:marker size:[.02] color:[0 0 1] }
#frame endeffR_fingerR (R_panda_leftfinger_1) { Q:<t(0 0 0.0446) d(180 1 0 0) d(90 0 0 1)> shape:marker size:[.02] color:[0 0 1] }

Delete L_panda_hand>panda_finger_joint1
Delete L_panda_hand>panda_finger_joint2
Delete L_panda_finger_joint1
Delete L_panda_finger_joint2
Delete L_panda_leftfinger_1
Delete L_panda_leftfinger_0
Delete L_panda_rightfinger_1
Delete L_panda_rightfinger_0

Delete R_panda_hand>panda_finger_joint1
Delete R_panda_hand>panda_finger_joint2
Delete R_panda_finger_joint1
Delete R_panda_finger_joint2
Delete R_panda_leftfinger_1
Delete R_panda_leftfinger_0
Delete R_panda_rightfinger_1
Delete R_panda_rightfinger_0




#frame calibrationMarkerL(L_panda_hand_joint) {shape:sphere Q:<t(-0.0313 0 0.03685) d(180 1 0 0) d(-90 0 0 1)> size:[0.01585] visual color:[1 0 0]}
#frame calibrationMarkerR(R_panda_hand_joint) {shape:sphere Q:<t(0.0313 0 0.03685) d(180 1 0 0) d(-90 0 0 1)> size:[0.01585] visual color:[1 0 0]}
frame calibrationMarkerRShape(R_panda_hand_joint) {shape:cylinder Q:<t(0.01835 0 0.0365) d(90 0 1 0)> size:[0.0005 0.0095] visual color:[0 1 0]}
frame calibrationMarkerR(R_panda_hand_joint) {shape:marker Q:<t(0.01835 0 0.0365) d(180 1 0 0) d(-90 0 0 1)> size:[0.02] color:[1 0 0]}
frame calibrationMarkerLShape(L_panda_hand_joint) {shape:cylinder Q:<t(-0.01835 0 0.0365) d(90 0 1 0)> size:[0.0005 0.0095] visual color:[0 1 0]}
frame calibrationMarkerL(L_panda_hand_joint) {shape:marker Q:<t(-0.01835 0 0.0365) d(180 1 0 0) d(-90 0 0 1)> size:[0.02] color:[1 0 0]}




#frame goalTable (table) { Q:<t(0.1 0.0 0.01) > shape:ssBox size:[0.05 0.05 .018 .005] color:[0 1 0] contact:0 logical:{tableS} }
frame goalTable (table) { Q:<t(0.4 0.3 0.01) > shape:ssBox size:[0.3 0.3 .018 .005] color:[0 1 0] contact:-1 logical:{tableS} }


#Delete L_panda_leftfingerCollision
#Delete L_panda_rightfingerCollision

#Delete R_panda_leftfingerCollision
#Delete R_panda_rightfingerCollision

#Edit L_panda_finger_joint1 {joint_scale: 0.2}
#Edit R_panda_finger_joint1 {joint_scale: 0.2}




