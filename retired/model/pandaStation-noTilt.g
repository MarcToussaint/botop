frame world { X:<t(0 0 0)> }

#box
frame box{ X:<t(0  0 .74)> }

frame (box){ Q:<t(0  .54 -.02)> shape:ssBox size:[.78 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0 -.54 -.02)> shape:ssBox size:[.78 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0  .54 -.52)> shape:ssBox size:[.78 .04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(0 -.54 -.52)> shape:ssBox size:[.78 .04 .08 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(-.41 0 -.02)> shape:ssBox size:[.04 1.04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .41 0 -.02)> shape:ssBox size:[.04 1.04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.41 0 -.52)> shape:ssBox size:[.04 1.04 .08 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .41 0 -.52)> shape:ssBox size:[.04 1.04 .08 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(-.41 -.54 -.17)> shape:ssBox size:[.04 .04 .78 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.41  .54 -.17)> shape:ssBox size:[.04 .04 .78 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .41 -.54 -.27)> shape:ssBox size:[.04 .04 .58 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( .41  .54 -.27)> shape:ssBox size:[.04 .04 .58 .01] color:[.4 .4 .4] }

frame (box){ Q:<t(-.41 -.54 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t(-.41  .54 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t( .41 -.54 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }
frame (box){ Q:<t( .41  .54 -.63)> shape:cylinder size:[.04 .04 .14 .05] color:[.6 .6 .6] }

frame (box){ Q:<t( 0  .54 -.27) d(-45 0 1 0)> shape:ssBox size:[.04 .04 .7 .01] color:[.4 .4 .4] }
frame (box){ Q:<t( 0 -.54 -.27) d(-45 0 1 0)> shape:ssBox size:[.04 .04 .7 .01] color:[.4 .4 .4] }
#frame (box){ Q:<t( 0 -.54 -.27)> shape:ssBox size:[.04 .04 .42 .01] color:[.9 .4 .4] }

frame (box){ Q:<t(-.26 -.28 -.23) d(-45 1 0 0)> shape:ssBox size:[.04 .04 .8 .01] color:[.4 .4 .4] }
frame (box){ Q:<t(-.26  .28 -.23) d( 45 1 0 0)> shape:ssBox size:[.04 .04 .8 .01] color:[.4 .4 .4] }
#frame (box){ Q:<t(-.26  .28 -.23) > shape:ssBox size:[.04 .04 .5 .01] color:[.8 .4 .4] }

#base
frame base (box){ Q:<t(-.23 0 .10)> }
frame (base){ Q:<t( .01 .0 -.04) > shape:ssBox size:[.08 1.12 .08 .01] color:[.6 .6 .6] }
frame (base){ Q:<t(-.12 .0 -.04) > shape:ssBox size:[.08 1.12 .08 .01] color:[.6 .6 .6] }


#table
frame table(world){ Q:<t(.42 0 .769)> shape:ssBox size:[1.2 2. .018 .005] color:[.4 .3 .2] }

#camera stand
frame camstand (box){ Q:<t(-.35 0 .4)> }
frame (camstand){ Q:<t(-.05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(-.05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05 -.05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t( .05  .05 .3)> shape:ssBox size:[.02 .02 1.2 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(.36 -.05 .9)> shape:ssBox size:[.84 .02 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(.36  .05 .9)> shape:ssBox size:[.84 .02 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(.36 -.05 .8)> shape:ssBox size:[.84 .02 .02 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(.36  .05 .8)> shape:ssBox size:[.84 .02 .02 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(.77 -.05 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(.77  .05 .85)> shape:ssBox size:[.02 .02 .08 .005] color:[.4 .4 .4] }

frame (camstand){ Q:<t(.12 -.05 .6) d(30 0 1 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }
frame (camstand){ Q:<t(.12  .05 .6) d(30 0 1 0)> shape:ssBox size:[.02 .02 .7 .005] color:[.4 .4 .4] }

frame camera(world){ Q:<t(.33  0 1.789) d(-90 0 0 1)> shape:marker, size:[.1], focalLength:1.281, width:480, height:270, zRange:[.1 10] }

frame tableMarker(world){ Q:<t(.0  0 .778)> shape:marker, size:[.1], color=[1 0 0] }

Prefix = "L_"
Include='../rai-robotModels/panda/panda.g'

Prefix = "R_"
Include='../rai-robotModels/panda/panda.g'

Edit L_panda_link0 (base) { joint:rigid Q:<t(0  .3 0)> }
Edit R_panda_link0 (base) { joint:rigid Q:<t(0 -.3 0)> }

#Edit L_panda_joint1 { q= -.5 }
#Edit R_panda_joint1 { q=  .5 }
#Edit L_panda_joint2 { q= -1.5 }
#Edit R_panda_joint2 { q= -1.5 }
#Edit L_panda_joint4 { q= -1.5 }
#Edit R_panda_joint4 { q= -1.5 }
#Edit L_panda_joint7 { q= 1.0 }
#Edit R_panda_joint7 { q= 1.0 }


frame endeffR (R_panda_hand_0) { Q:<t(0 0 .09) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] }
frame endeffL (L_panda_hand_0) { Q:<t(0 0 .09) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.02] color:[1 1 0] }
