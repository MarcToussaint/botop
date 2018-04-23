# LGP-execution

## Quick Start

```
git clone git@github.com:MarcToussaint/LGP-execution.git
cd LGP-execution

git submodule init
git submodule update

make -j1 initUbuntuPackages  # calls sudo apt-get install; you can always interrupt

cd src/Sim; make
cd 01-simulator; make; ./x.exe
cd 02-rndPolicy; make; ./x.exe
```

