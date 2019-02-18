BASE = rai

DEPEND = KOMO Core Algo Geo Kin Gui Optim Exec Sim

test_paths = $(shell find test -mindepth 2 -maxdepth 2 -name 'Makefile' -printf "%h ")


build: $(DEPEND:%=inPath_makeLib/%)

initUbuntuPackages: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntu: $(DEPEND:%=inPath_printUbuntuPackages/%) printUbuntuPackages

clean: $(DEPEND:%=inPath_clean/%)

tests: $(test_paths:%=inPath_make/%)

runTests: tests
	@rm -f z.test-report
	@find test -mindepth 1 -maxdepth 1 -type d \
		-exec rai/build/run-path.sh {} \;

include $(BASE)/build/generic.mk

.NOTPARALLEL:
