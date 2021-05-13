Include = 'problem-shared.g'

### ball

frame Hook { X:<t(.3 .3 .3)> }
frame KukaBase { }

joint (KukaBase base) { type:rigid, B:<t( 0 0 .02)> }

### hook

#frame stick { type=5 size=[.2 .2 .2] }
#joint (table1 nostick) { to=<T t(0 0 .02) t(-.2 -.7 .02)> type=rigid}

frame table1 (KukaBase){ type=ssBox, Q=<T d(-2 0 0 1) t(.82 .03 -.085)>, size=[1.914 1.83 .2 .02], color=[.3 .3 .3], contact, percept, logical={ table } }

frame stick (Hook) {
    shape=ssBox size=[.704 .025 .04 .01] color=[.6 .3 0] contact, percept, logical={ object }
    Q=<T d(-90 0 0 1) t(-.352 0 .03)> }
shape stickTip (Hook) { rel=<T d(-90 0 0 1) t(0 -.1 .03) d(90 0 0 1)> type=ssBox size=[.20 .026 .04 0.01] color=[.6 .3 0], logical={ object, pusher }, contact }

frame redBall(table1) {
    shape=ssBox, size=[.06 .06 .06 .02] color=[1 0 0],
    contact, percept, logical={ object },
    joint=rigid, Q=<T t(0 0 .1) t(.0 .7 .03)>
}

frame RigidBody01(table1) {
    shape=ssBox, size=[.06 .06 .06 .02] color=[1 1 0],
    contact, percept, logical={ object },
    joint=rigid, Q=<T t(0 0 .1) t(.0 .0 .03)>
}

