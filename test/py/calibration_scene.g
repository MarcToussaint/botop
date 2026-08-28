Include: <$RAI_PATH/scenarios/pandasTable.g>

Edit(l_panda_base) { franka_ip: "" }
Edit(r_panda_base) { franka_ip: "172.17.0.2" }

obj_base: { X: [ 0., 0., .8] }
obj(obj_base): { joint: free, limits: [-3.2,-3.2,-3.2,-1.1,-1.1,-1.1,-1.1,3.2,3.2,3.2,1.1,1.1,1.1,1.1], Q: [.1, .2, .1], shape: ssBox, size: [.17, .17, .17, .005], color: [1., .5, .0], mass: .2, sampleUniform: 1., contact: 1 }
# obj_axes(obj): { shape: marker, size: [.2], simulation: false }

ar0(obj): { Q: [.086, .0572, .0572, 1., 0., 1., 0.], shape: quad, size: [.045, .045], aruco_id: 0 }
ar1(obj): { Q: [.086, -.0572, -.0572 , 0., 1., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 1 }
ar2(obj): { Q: [-.086, .0572 , -.0572 , 1., 0., -1., 0.], shape: quad, size: [.045, .045], aruco_id: 2 }
ar3(obj): { Q: [-.086, -.0572 , .0572 , 0., 1., 0., -1.], shape: quad, size: [.045, .045], aruco_id: 3 }
ar4(obj): { Q: [.0572 , .086, .0572 , 0., 0., 1., 1.], shape: quad, size: [.045, .045], aruco_id: 4 }
ar5(obj): { Q: [-.0572 , .086, -.0572 , 1., -1., 0., 0.], shape: quad, size: [.045, .045], aruco_id: 5 }
ar6(obj): { Q: [.0572 , -.086, -.0572 , 0., 0., 1., -1.], shape: quad, size: [.045, .045], aruco_id: 6 }
ar7(obj): { Q: [-.0572 , -.086, .0572 , 1., 1., 0., 0.], shape: quad, size: [.045, .045], aruco_id: 7 }
ar8(obj): { Q: [.0572 , .0572 , .086, 1., 0., 0., -1.], shape: quad, size: [.045, .045], aruco_id: 8 }
ar9(obj): { Q: [-.0572 , -.0572 , .086, 1., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 9 }
ar10(obj): { Q: [.0572 , -.0572 , -.086, 0., 1., 1., 0.], shape: quad, size: [.045, .045], aruco_id: 10 }
ar11(obj): { Q: [-.0572 , .0572 , -.086, 0., 1., -1., 0.], shape: quad, size: [.045, .045], aruco_id: 11 }

art31(r_panda_hand_joint): { Q: "t(-.019 .0 .030) d(180 0 0 1) d(90 0 1 0) d(-90 0 0 1)", shape: quad, size: [.045, .045], aruco_id: 48 }
art32(r_panda_hand_joint): { Q: "t(.019 .0 .030) d(180 0 0 1) d(-90 0 1 0) d(90 0 0 1)", shape: quad, size: [.045, .045], aruco_id: 43 }
art33(r_panda_hand_joint): { Q: "t(.019 .054 .042) d(180 0 0 1) d(-90 0 1 0) d(90 0 0 1)", shape: quad, size: [.045, .045], aruco_id: 29 }
art34(r_panda_hand_joint): { Q: "t(.019 -.054 .042) d(180 0 0 1) d(-90 0 1 0) d(90 0 0 1)", shape: quad, size: [.045, .045], aruco_id: 30 }

rig_bar0(table): { Q: [-.9, .6, -.07], shape: ssBox, size: [0.02, 0.4, 0.04, .002], color: [.6] }
rig_bar1(table): { Q: [-.3, .6, -.07], shape: ssBox, size: [0.02, 0.4, 0.04, .002], color: [.6] }
rig_bar2(table): { Q: [.3, .6, -.07], shape: ssBox, size: [0.02, 0.4, 0.04, .002], color: [.6] }
rig_bar3(table): { Q: [.9, .6, -.07], shape: ssBox, size: [0.02, 0.4, 0.04, .002], color: [.6] }

rig_base(table): { Q: [.0, .79, -.05] }
rig_pole0(rig_base): { Q: [.9, .0, .5], shape: ssBox, size: [0.02, 0.02, 1., .002] }
rig_pole1(rig_base): { Q: [.0, .0, .5], shape: ssBox, size: [0.02, 0.02, 1., .002] }
rig_pole2(rig_base): { Q: [-.9, .0, .5], shape: ssBox, size: [0.02, 0.02, 1., .002] }
rig_top(rig_base): { Q: [.0, .0, .99], shape: ssBox, size: [1.8, 0.02, .02, .002] }
rig3(rig_base): { Q: [.0, .0, .01], shape: ssBox, size: [1.8, 0.02, .02, .002] }
#rig3(rig_base): { Q: [.0, .3, .01], shape: ssBox, size: [1., 0.02, .02, .002] }
#rig3(rig_base): { Q: [.5, .15, .01], shape: ssBox, size: [.02, 0.32, .02, .002] }
#rig3(rig_base): { Q: [-.5, .15, .01], shape: ssBox, size: [.02, 0.32, .02, .002] }
rig3(rig_base): { Q: "t(.9 -.09 .13) d(-30 1 0 0)", shape: ssBox, size: [.02, 0.02, .3, .002] }
rig3(rig_base): { Q: "t(-.9 -.09 .13) d(-30 1 0 0)", shape: ssBox, size: [.02, 0.02, .3, .002] }
rig3(rig_base): { Q: "t( .81 .0 .13) d( 30 0 1 0) t(0 0 .01)", shape: ssBox, size: [.02, 0.02, .28, .002] }
rig3(rig_base): { Q: "t(-.81 .0 .13) d(-30 0 1 0) t(0 0 .01)", shape: ssBox, size: [.02, 0.02, .28, .002] }


#cam0(rig_top): { Q: "t(.0 .0 .0) d(180 0 0 1) d(-90 1 0 0) d(-45 1 0 0)", shape: marker, size: [.1], width: 800, height: 600, heightAngle: 68 }

#cam1(rig_pole0): { Q: "t(.0 .0 .3) d(180 0 0 1) d(-90 1 0 0) d(45 0 1 0) d(-30 1 0 0)", shape: marker, size: [.1], width: 800, height: 600, focalLength: 1.47 }
#cam2(rig_pole0): { Q: "t(.0 .0 -.3) d(180 0 0 1) d(-90 1 0 0) d(45 0 1 0) d(10 1 0 0)", shape: marker, size: [.1], width: 800, height: 600, focalLength: 1.47 }

#cam3(rig_pole2): { Q: "t(.0 .0 .3) d(180 0 0 1) d(-90 1 0 0) d(-45 0 1 0) d(-30 1 0 0)", shape: marker, size: [.1], width: 800, height: 600, focalLength: 1.47 }
#cam4(rig_pole2): { Q: "t(.0 .0 -.3) d(180 0 0 1) d(-90 1 0 0) d(-45 0 1 0) d(10 1 0 0)", shape: marker, size: [.1], width: 800, height: 600, heightAngle: 68 }

art11(table): { Q: [.88 , .58 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 31 }
art12(table): { Q: [.4 , .58 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 36 }
art13(table): { Q: [.0 , .58 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 41 }
art14(table): { Q: [.75 , -.1 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 35 }
art15(table): { Q: [.05 , -.1 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 25 }

art21(table): { Q: [.283 , -.277 , .0505, 0., 0., 0., 1.], shape: quad, size: [.045, .045], aruco_id: 37 }
art22(table): { Q: [.517 , -.277 , .0505, 1., 0., 0., 0.], shape: quad, size: [.045, .045], aruco_id: 42 }
        
camera_0(table): {
  basler: 40788612,
  Q: [0.918802, 0.779245, 0.639062, 0.115265, 0.67314, 0.600194, -0.416371]
  shape: marker, size: [.1],
  width: 1296, height: 972,
  # fxycxy: [1009.12, 1011.12, 638.403, 505.29], distortion: [-0.25926, 0.161273, 0.00234894, 0.000532364, -0.0813702]
  fxycxy: [1011.99, 1012.06, 645.588, 471.646], distortion: [-0.251808, 0.140477, -6.81627e-05, 7.48484e-05, -0.0597704]
  #fxycxy: [1013.6617031960608, 1014.1728954833683, 641.0030346256076, 471.55660368940636]
  #distortion: [-0.2540915125466794, 0.14076989020155176, 6.834847377053347e-05, 0.00014755920014026806, -0.05743168173510058]
}

camera_1(table): {
  basler: 41956220,
  Q: [0.0708059, 0.76234, 0.630606, 0.122105, 0.693827, -0.561165, 0.434498]
  shape: marker, size: [.1],
  width: 1296, height: 972,
  # fxycxy: [1010.66, 1010.83, 628.079, 452.757], distortion: [-0.260142, 0.148042, 0.00157564, -0.0018581, -0.0600145]
  fxycxy: [1013.77, 1013.7, 636.466, 440.192], distortion: [-0.249784, 0.131404, 0.000299183, -0.000376537, -0.0478976]
  #fxycxy: [988.9491690061556, 989.4141040159345, 623.1037849361142, 439.66447559070514]
  #distortion: [-0.2413961154003396, 0.11674053733915875, 0.0002463650206466761, 0.0004825219920498637, -0.04322697453856177]
}

camera_2(table): {
  basler: 41958511,
  Q: [0.517265, 0.792152, 0.983084, 0.429903, 0.902334, 0.0138988, -0.0279781]
  shape: marker, size: [.1],
  width: 1296, height: 972,
  # fxycxy: [905.292, 888.172, 644.021, 406.256], distortion: [-0.116113, -0.180158, 0.00819024, -0.000189636, 0.306537]
  fxycxy: [1014.02, 1014.4, 629.798, 446.112], distortion: [-0.248996, 0.128033, 0.000196656, -0.000330586, -0.0433509]
  #fxycxy: [1260.05912806823, 1256.8118203928045, 674.0255235561862, 443.7420249071231]
  #distortion: [-0.3862890596111546, 0.3735525760450747, 0.006015829348812757, -0.0014634538456106223, -0.20684906840089493]
}

Include: <calib_cams_intrinsics.yml>
Include: <calib_cams.yml>
Include: <calib_arucos.yml>
Include: <calib_joints.yml>
