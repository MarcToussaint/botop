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
frame table(world){ Q:<t(0 .42 .769)> shape:ssBox size:[2. 1.2 .018 .005] color:[.4 .3 .2] }

#camera stand
frame camstand (box){ Q:<t(0 -.35 .4)> }
frame (camstand){ Q:<t(-.05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(-.05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(-.05 .36 .9)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 .36 .9)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(-.05 .36 .8)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 .36 .8)> shape:ssBox size:[.02 .84 .02 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(-.05 .77 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 .77 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(-.05 .12 .6) d(-30 1 0 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 .12 .6) d(-30 1 0 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }

frame camera(world){ Q:<t(-0.0107846 0.35387 1.76293)> shape:marker, size:[.1], focalLength:0.895, width:640, height:360, zRange:[.1 10] } #-0.0107846 0.35387 1.76293 #0 .33 1.789

frame tableMarker(world){ Q:<t(0 0 .778)> shape:marker, size:[.1], color=[1 0 0] }

Prefix = "L_"
Include='../rai-robotModels/panda/panda.g'

Prefix = "R_"
Include='../rai-robotModels/panda/panda.g'

Edit L_panda_link0 (base) { joint:rigid Q:<t(-.3 0 0) d(90 0 0 1)> }
Edit R_panda_link0 (base) { joint:rigid Q:<t( .3 0 0) d(90 0 0 1)> }

#Edit L_panda_joint1 { q= -.5 }
#Edit R_panda_joint1 { q=  .5 }
Edit L_panda_joint2 { q= -.5 }
Edit R_panda_joint2 { q= -.5 }
#Edit L_panda_joint4 { q= -1.5 }
#Edit R_panda_joint4 { q= -1.5 }
#Edit L_panda_joint7 { q= 1.0 }
#Edit R_panda_joint7 { q= 1.0 }


Edit L_panda_leftfinger_1 { visual!, noVisual }
Edit L_panda_rightfinger_1 { visual!, noVisual }
Edit R_panda_leftfinger_1 { visual!, noVisual }
Edit R_panda_rightfinger_1 { visual!, noVisual }

frame endeffR (R_panda_hand_1) { Q:<t(0 0 .09) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] }
frame endeffL (L_panda_hand_1) { Q:<t(0 0 .09) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] }
