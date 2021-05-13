frame hook { X:<t(.3 .3 .3)> }
frame stick (hook) {
    shape=ssBox size=[.704 .025 .04 .01] color=[.6 .3 0] contact, percept, logical={ object }
    Q=<T d(-90 0 0 1) t(-.352 0 .03)> }
shape stickTip (hook) { rel=<T d(-90 0 0 1) t(0 -.1 .03) d(90 0 0 1)> type=ssBox size=[.20 .026 .04 0.01] color=[.6 .3 0], logical={ object, pusher }, contact }
