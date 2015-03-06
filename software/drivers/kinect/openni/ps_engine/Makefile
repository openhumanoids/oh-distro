# Author: Suat Gedikli
# Date:   10 August 2011
all: installed

GIT_DIR = build/ps_engine
GIT_URL = https://github.com/PrimeSense/Sensor.git
GIT_PATCH = willow.patch disable_sse disable_LogWriteToFile fix-includes fix-mac-arch
# ros branch as of 2011-01-05
#GIT_REVISION=a28e94eed8d4191370db
GIT_REVISION=342e334c612fa3ccb4b7 # Version 5.0.3.3 Stable version - Jul 5th 2011

PLATFORM=Linux-x86

include ../git_checkout.mk

installed: download
	cd $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist && ./RedistMaker && cd -
	touch installed
	
clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*

wipe:
	rm -rf build installed patched rospack_nosubdirs

