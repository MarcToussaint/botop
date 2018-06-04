BASE = rai

DEPEND = KOMO Core Algo Geo Kin Gui Optim Exec Sim

build: $(DEPEND:%=inPath_makeLib/%)

initUbuntuPackages: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntu: $(DEPEND:%=inPath_printUbuntuPackages/%) printUbuntuPackages

clean: $(DEPEND:%=inPath_clean/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
