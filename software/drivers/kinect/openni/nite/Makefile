all: nite

ARCH = $(shell uname -m)
ifeq ($(ARCH), i686)
ARCH_DIR = x86
TARBALL     = primesense-nite-unstable-build-for-ubuntu-10-10-x86-32-bit-v1-3-1.tar.bz2
else ifeq ($(ARCH), i486)
ARCH_DIR = x86
TARBALL     = primesense-nite-unstable-build-for-ubuntu-10-10-x86-32-bit-v1-3-1.tar.bz2
else ifeq ($(ARCH), i386)
ARCH_DIR = x86
TARBALL     = primesense-nite-unstable-build-for-ubuntu-10-10-x86-32-bit-v1-3-1.tar.bz2
else ifeq ($(ARCH), x86_64)
ARCH_DIR = x64
TARBALL     = primesense-nite-unstable-build-for-ubuntu-10-10-x64-64-bit-v1-3-1.tar.bz2
endif
BIN_DIR = $(SOURCE_DIR)/bin-pc/$(ARCH_DIR)
TARBALL_URL = http://pr.willowgarage.com/downloads/openni/$(TARBALL)

UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/Nite-1.3.1.5
TARBALL_PATCH = 

NITE_LIB=$(shell echo $$(eval rospack find nite | sed 's/\//\\\//g')\\/lib)

include download_unpack_build.mk

INSTALL_DIR = .
nite: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) build $(INSTALL_DIR)/bin $(INSTALL_DIR)/lib $(INSTALL_DIR)/include $(INSTALL_DIR)/XnVFeatures $(INSTALL_DIR)/XnVHandGenerator $(INSTALL_DIR)/installed
	rm -f *.bz2

wipe: clean
