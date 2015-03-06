# Author: Suat Gedikli
# Date:   10 August 2011
all: installed

GIT_DIR = build/openni
GIT_URL = https://github.com/OpenNI/OpenNI.git
GIT_PATCH = willow.patch libGL disable-mono disable_sse silence_assert fix-mac-arch #fix-java
#GIT_REVISION=d50d0e3fb0a5913185ea # unstable 1.1.0.41
#GIT_REVISION=ee31b48b2e0ea6a81fd4 # unstable 1.3.2.3
GIT_REVISION=f8467404e3e433ded1c7 # stable 1.3.2.1

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

