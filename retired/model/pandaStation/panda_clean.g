frame panda_link0 	{  }
frame panda_link0_1(panda_link0) 	{  shape:mesh  mesh:'meshes/visual/link0.ply'  visual, color=[.9 .9 .9] }
frame panda_link0_0(panda_link0) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link0.stl' noVisual, contact:-2  }
frame panda_link0>panda_joint1(panda_link0) 	{  Q:<0 0 0.333 0.707107 0 -0.707107 0> }
frame panda_joint1(panda_link0>panda_joint1) 	{  joint:hingeX ctrl_H:1 limits=[  -2.8973 2.8973 2.175 87 1  ]  ctrl_limits:[ 2.175 87 1 ] }
frame panda_link1_1(panda_joint1) 	{  shape:mesh  mesh:'meshes/visual/link1.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link1_0(panda_joint1) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link1.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link1>panda_joint2(panda_joint1) 	{  Q:<-0 -0 -0 -0.707107 1.11022e-16 -5.55112e-17 -0.707107> }
frame panda_joint2(panda_link1>panda_joint2) 	{  joint:hingeX ctrl_H:1 limits=[  -1.7628 1.7628 2.175 87 1  ]  ctrl_limits:[ 2.175 87 1 ] }
frame panda_link2_1(panda_joint2) 	{  shape:mesh  mesh:'meshes/visual/link2.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link2_0(panda_joint2) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link2.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link2>panda_joint3(panda_joint2) 	{  Q:<-0 -0.316 0 -0.707107 -1.11022e-16 -5.55112e-17 0.707107> }
frame panda_joint3(panda_link2>panda_joint3) 	{  joint:hingeX ctrl_H:1 limits=[  -2.8973 2.8973 2.175 87 1  ]  ctrl_limits:[ 2.175 87 1 ] }
frame panda_link3_1(panda_joint3) 	{  shape:mesh  mesh:'meshes/visual/link3.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link3_0(panda_joint3) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link3.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link3>panda_joint4(panda_joint3) 	{  Q:<1.83187e-17 -0 -0.0825 -0.707107 -1.11022e-16 -5.55112e-17 0.707107> }
frame panda_joint4(panda_link3>panda_joint4) 	{  joint:hingeX ctrl_H:1 limits=[  -3.0718 -0.0698 2.175 87 1  ]  ctrl_limits:[ 2.175 87 1 ] }
frame panda_link4_1(panda_joint4) 	{  shape:mesh  mesh:'meshes/visual/link4.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link4_0(panda_joint4) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link4.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link4>panda_joint5(panda_joint4) 	{  Q:<-1.83187e-17 0.384 0.0825 -0.707107 1.11022e-16 -5.55112e-17 -0.707107> }
frame panda_joint5(panda_link4>panda_joint5) 	{  joint:hingeX ctrl_H:1 limits=[  -2.8973 2.8973 2.61 12 1  ]  ctrl_limits:[ 2.61 12 1 ] }
frame panda_link5_1(panda_joint5) 	{  shape:mesh  mesh:'meshes/visual/link5.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link5_0(panda_joint5) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link5.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link5>panda_joint6(panda_joint5) 	{  Q:<-0 -0 -0 -0.707107 -1.11022e-16 -5.55112e-17 0.707107> }
frame panda_joint6(panda_link5>panda_joint6) 	{  joint:hingeX ctrl_H:1 limits=[  -0.0175 3.7525 2.61 12 1  ]  ctrl_limits:[ 2.61 12 1 ] }
frame panda_link6_1(panda_joint6) 	{  shape:mesh  mesh:'meshes/visual/link6.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link6_0(panda_joint6) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link6.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link6>panda_joint7(panda_joint6) 	{  Q:<1.95399e-17 -0 -0.088 -0.707107 -1.11022e-16 -5.55112e-17 0.707107> }
frame panda_joint7(panda_link6>panda_joint7) 	{  joint:hingeX ctrl_H:1 limits=[  -2.8973 2.8973 2.61 12 1  ]  ctrl_limits:[ 2.61 12 1 ] }
frame panda_link7_1(panda_joint7) 	{  shape:mesh  mesh:'meshes/visual/link7.ply' Q:<-0 -0 -0 -0.707107 0 -0.707107 0>  visual, color=[.9 .9 .9] }
frame panda_link7_0(panda_joint7) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/link7.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 -0.707107 0> }
frame panda_link7>panda_joint8(panda_joint7) 	{  Q:<0.107 0 2.37588e-17 -0.707107 0 -0.707107 0> }
frame panda_joint8(panda_link7>panda_joint8) 	{  joint:rigid ctrl_H:1 }
frame panda_link8>panda_hand_joint(panda_joint8) 	{  Q:<0 0 0 0.92388 0 0 -0.382683> }
frame panda_hand_joint(panda_link8>panda_hand_joint) 	{  joint:rigid ctrl_H:1 }
frame panda_hand_1(panda_hand_joint) 	{  shape:mesh  mesh:'meshes/visual/hand.ply'  visual, color=[.9 .9 .9] }
frame panda_hand_0(panda_hand_joint) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/hand.stl' noVisual, contact:-2  }
frame panda_hand>panda_finger_joint1(panda_hand_joint) 	{  Q:<0 0 0.0584 0.707107 0 0 0.707107> }
frame panda_hand>panda_finger_joint2(panda_hand_joint) 	{  Q:<0 0 0.0584 0.707107 0 0 -0.707107> }
frame panda_finger_joint1(panda_hand>panda_finger_joint1) 	{  joint:transX ctrl_H:1 limits=[  0 0.04 0.2 20 1  ]  ctrl_limits:[ 0.2 20 1 ] }
frame panda_finger_joint2(panda_hand>panda_finger_joint2) 	{  joint:transX ctrl_H:1 limits=[  0 0.04 0.2 20 1  ] mimic:(panda_finger_joint1)  ctrl_limits:[ 0.2 20 1 ] }
frame panda_leftfinger_1(panda_finger_joint1) 	{  shape:mesh  mesh:'meshes/visual/finger.ply' Q:<-0 -0 -0 -0.707107 0 0 0.707107>  visual, color=[.9 .9 .9] }
frame panda_leftfinger_0(panda_finger_joint1) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/finger.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 0 0.707107> }
frame panda_rightfinger_1(panda_finger_joint2) 	{  shape:mesh  mesh:'meshes/visual/finger.ply' Q:<-0 -0 -0 -0.707107 0 0 0.707107>  visual, color=[.9 .9 .9] }
frame panda_rightfinger_0(panda_finger_joint2) 	{  shape:mesh  color:[ 0.8 0.2 0.2 0.2 ]  mesh:'meshes/collision/finger.stl' noVisual, contact:-2  Q:<-0 -0 -0 -0.707107 0 0 -0.707107> }

