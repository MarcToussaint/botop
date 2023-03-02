BASE = rai
BASE2 = src

DEPEND = KOMO Core Algo Geo Kin Gui Optim Perception

test_paths = $(shell find test -mindepth 2 -maxdepth 2 -name 'Makefile' -printf "%h ")

src_paths =  $(shell find src -mindepth 1 -maxdepth 1 -type d -not -name 'retired' -printf "%f ")


build: $(DEPEND:%=inPath_makeLib/%)

installUbuntuAll: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

unityAll: $(src_paths:%=inPath_unity/%)

clean: $(DEPEND:%=inPath_clean/%)

tests: $(test_paths:%=inPath_make/%)

runTests: tests
	@rm -f z.test-report
	@find test -mindepth 1 -maxdepth 1 -type d \
		-exec rai/makeutils/run-path.sh {} \;

include $(BASE)/makeutils/generic.mk

.NOTPARALLEL:
