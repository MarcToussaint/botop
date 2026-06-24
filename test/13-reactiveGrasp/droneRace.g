world {}

## drone
drone (world) { joint:trans3 X:<t(0 .5 1)> }

 (drone){ Q:<d(45 0 0 1)> shape:ssBox size:[0.094 0.005 0.005 0.001] color:[.9 .9 .9] mass:0.015}
 (drone){ Q:<d(-45 0 0 1)> shape:ssBox size:[0.094 0.005 0.005 0.001] color:[.9 .9 .9] mass:0.015}
 (drone){ shape:marker, size:[.1] }
m1(drone){ Q:[ 0.0325 -0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }
m2(drone){ Q:[-0.0325 -0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }
m3(drone){ Q:[-0.0325  0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }
m4(drone){ Q:[ 0.0325  0.0325 0.0] shape:cylinder size:[.02 .005] color:[.9 .9 .9] }


## cage

wall0 (world) { Q:[0 -1.5 1.3] shape:ssBox, size:[3. .1 2.6 .02], color:[.9 .9 .9 .2] }
#wall1 (world) { Q:[0  1.5 1.3] shape:ssBox, size:[3. .1 2.6 .02], color:[.9 .9 .9 .2] }
wall2 (world) { Q:[-1.5 0 1.3] shape:ssBox, size:[.1 3. 2.6 .02], color:[.9 .9 .9 .2] }
#wall3 (world) { Q:[ 1.5 0 1.3] shape:ssBox, size:[.1 3. 2.6 .02], color:[.9 .9 .9 .2] }

## targets

target0 (world) { X:<d(-45 0 0 1) t(0 1 .5)> shape:marker size:[.1] color:[.9 .9 .9 .5] }

  (target0) { Q:[0 0  .2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target0) { Q:[0 0 -.2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target0) { Q:[0  .2 0] shape:ssBox, size:[.05 .05 .45 .02] }
  (target0) { Q:[0 -.2 0] shape:ssBox, size:[.05 .05 .45 .02] }

target1 (world) { X:<d(-135 0 0 1) t(0 1 1)> shape:marker size:[.1] color:[.9 .9 .9 .5] }

  (target1) { Q:[0 0  .2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target1) { Q:[0 0 -.2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target1) { Q:[0  .2 0] shape:ssBox, size:[.05 .05 .45 .02] }
  (target1) { Q:[0 -.2 0] shape:ssBox, size:[.05 .05 .45 .02] }

target2 (world) { X:<d(135 0 0 1) t(0 1 .5)> shape:marker size:[.1] color:[.9 .9 .9 .5] }

  (target2) { Q:[0 0  .2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target2) { Q:[0 0 -.2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target2) { Q:[0  .2 0] shape:ssBox, size:[.05 .05 .45 .02] }
  (target2) { Q:[0 -.2 0] shape:ssBox, size:[.05 .05 .45 .02] }

target3 (world) { X:<d( 45 0 0 1) t(0 1 1)> shape:marker size:[.1] color:[.9 .9 .9 .5] }

  (target3) { Q:[0 0  .2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target3) { Q:[0 0 -.2] shape:ssBox, size:[.05 .45 .05 .02] }
  (target3) { Q:[0  .2 0] shape:ssBox, size:[.05 .05 .45 .02] }
  (target3) { Q:[0 -.2 0] shape:ssBox, size:[.05 .05 .45 .02] }
