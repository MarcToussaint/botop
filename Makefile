BASE = rai

DEPEND = KOMO Core Algo Geo Kin Gui Optim

test_paths = $(shell find test -maxdepth 3 -name 'Makefile' -printf "%h ")

build: $(DEPEND:%=inPath_makeLib/%) $(test_paths:%=inPath_make/%)

initUbuntuPackages: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntu: $(DEPEND:%=inPath_printUbuntuPackages/%) printUbuntuPackages

clean: $(DEPEND:%=inPath_clean/%) $(test_paths:%=inPath_clean/%)$

runTests: $(test_paths:%=inPath_make/%) $(test_paths:%=inPath_run/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
