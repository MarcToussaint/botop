Include: '../../rai-robotModels/scenarios/pandasTable-calibrated.g'

box (table){ joint: rigid, shape: ssBox, size: [.06,.15,.09,.01], Q:[-.0,-.0,.095] }

target (table) {joint: rigid, shape: ssBox, size: [.4,.4,.1,.01], Q:[-.4,.2,.0] }

 b1(optitrack_base){ shape:marker, size:[.1] }
