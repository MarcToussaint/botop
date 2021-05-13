Delete panda_link0_0
Delete panda_link1_0
Delete panda_link2_0
Delete panda_link3_0
Delete panda_link4_0
Delete panda_link5_0
Delete panda_link6_0
Delete panda_link7_0
Delete panda_hand_0
Delete panda_leftfinger_0
Delete panda_rightfinger_0

frame (panda_link0)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.1 .1] Q:<t(-.04 .0 .03) d(90 0 1 0)>, noVisual, contact:-2  }

frame (panda_joint1)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.2 .08] Q:<d(90 0 1 0) t(0 0 -.15)>, noVisual, contact:-2  }
frame (panda_joint3)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.2 .08] Q:<d(90 0 1 0) t(0 0 -.15)>, noVisual, contact:-2  }
frame (panda_joint5)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.22 .08] Q:<d(90 0 1 0) t(0 .02 -.2)>, noVisual, contact:-2  }

frame (panda_joint2)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.12 .08] Q:<d(90 0 1 0) t(0 0 .0)>, noVisual, contact:-2  }
frame (panda_joint4)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.12 .08] Q:<d(90 0 1 0) t(0 0 .0)>, noVisual, contact:-2  }
frame (panda_joint6)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.1 .07] Q:<d(90 0 1 0) t(0 .0 -.04)>, noVisual, contact:-2  }
frame (panda_joint7)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.1 .07] Q:<d(90 0 1 0) t(0 .0 .01)>, noVisual, contact:-2  }

frame handCollision (panda_hand_joint)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.15 .06] Q:<d(90 1 0 0) t(0 .01 .0)>, noVisual, contact:-3  }

#frame panda_leftfingerCollision(panda_finger_joint1)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.03 .014] Q:<d(0 1 0 0) t(.015 .0 .03)>, noVisual, contact:-2  }
#frame panda_rightfingerCollision(panda_finger_joint2)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.03 .014] Q:<d(0 1 0 0) t(.015 .0 .03)>, noVisual, contact:-2  }


frame panda_leftfingerCollision(panda_hand_joint)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.07 .017] Q:<t(0.0 .05 .09)>, noVisual, contact:-2  }
frame panda_rightfingerCollision(panda_hand_joint)	{ shape:capsule color:[ 0.8 0.2 0.2 0.2 ] size:[.07 .017] Q:<t(0.0 -.05 .09)>, noVisual, contact:-2  }
