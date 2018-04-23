# LGP-execution

## Quick Start

```
git clone git@github.com:MarcToussaint/LGP-execution.git
cd LGP-execution

git submodule init
git submodule update

#ensure you got the Ubuntu packages installed, e.g.:
cd rai; make printUbuntu

cd src/Sim; make
cd 01-simulator; make; ./x.exe
cd 02-rndPolicy; make; ./x.exe
```

To test rai only:
```
cd rai; make runTests
```


