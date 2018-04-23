# LGP-execution

## Quick Start

I usually set `export MAKEFLAGS=-j4 -k`

```
git clone git@github.com:MarcToussaint/LGP-execution.git
cd LGP-execution

git submodule init
git submodule update

#ensure you got the Ubuntu packages installed, e.g.:
make -C rai -j1 printUbuntu

make -C src/Sim
roscore
cd 01-simulator; make && ./x.exe &
cd ../02-rndPolicy; make && ./x.exe
```

To test rai only:
```
cd rai; make runTests
```


