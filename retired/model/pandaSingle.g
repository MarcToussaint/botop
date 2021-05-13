frame world { X:<t(0 0 0)> }
frame box{ X:<t(0  0 .74)> }
frame base (box){ Q:<t(-.225 0 .08)> } #d(45 0 1 0)> }

Prefix: "R_"
Include: '../rai-robotModels/panda/panda.g'
Prefix!

Edit R_panda_link0 (base) { Q:<t(0 -.3 0)> }
Edit R_panda_joint2 { q= -.5 }

frame endeffR (R_panda_hand_joint) { Q:<t(0 0 .11) d(180 1 0 0) d(-90 0 0 1)> shape:marker size:[.1] color:[1 0 1] }


  
