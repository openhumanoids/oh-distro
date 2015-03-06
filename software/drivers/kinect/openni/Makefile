# Author: Suat Gedikli
# Date:   25. February 2011


BUILD_NUMBER=3
OPENNI_VERSION=1.3.2.1
PSENGINE_VERSION=5.0.3.3
NITE_VERSION=1.3.1.5

ARCHITECTURE=$(shell uname -m)
ifeq (${ARCHITECTURE},x86_64)
        ARCH=amd64
else 
        ARCH=i386
endif

DISTRO=$(shell lsb_release -sc)

OPENNI_PACKAGE_NAME=ros-openni-dev-${OPENNI_VERSION}~${DISTRO}_$(ARCH)${BUILD_NUMBER}
ENGINE_PACKAGE_NAME=ros-ps_engine-dev-${PSENGINE_VERSION}~${DISTRO}_$(ARCH)${BUILD_NUMBER}
NITE_PACKAGE_NAME=ros-nite-dev-${NITE_VERSION}~${DISTRO}_$(ARCH)${BUILD_NUMBER}

all: ps_engine_lib nite_lib

debian: debian_openni debian_engine debian_nite

debian_openni : $(OPENNI_PACKAGE_NAME).deb

debian_engine : $(ENGINE_PACKAGE_NAME).deb

debian_nite : $(NITE_PACKAGE_NAME).deb

install_openni: openni_lib
	sudo mkdir -p /usr/include/openni \
								/etc/openni \
								/usr/share/openni/doc \
								/usr/share/openni/samples \
								/usr/lib/pkgconfig
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Config/* /etc/openni/
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Lib/*.so /usr/lib/
	sudo cp -r ./openni/build/openni/Platform/Linux-x86/Redist/Include/* /usr/include/openni/
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Bin/* /usr/bin/
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Documentation/html/* /usr/share/openni/doc/
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Bin/Release/Sample-* /usr/share/openni/samples/
	sudo cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Bin/Release/NiViewer /usr/share/openni/samples/
	sudo sed s/__VERSION__/${OPENNI_VERSION}~${DISTRO}/ ./CONTROL/openni.pc > /usr/lib/pkgconfig/openni-dev.pc
	sudo /bin/bash ./CONTROL/openni_postinst configure

install_engine: install_openni ps_engine_lib
	sudo cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Lib/*.so /usr/lib/
	sudo cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Config/*.ini /etc/openni/
	sudo cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Bin/XnSensorServer /usr/bin/
	sudo cp -f ./CONTROL/55-primesense-usb.rules /etc/udev/rules.d/
	sudo cp -f ./CONTROL/blacklist-psengine.conf /etc/modprobe.d/
	sudo sed s/__VERSION__/${PSENGINE_VERSION}~${DISTRO}/ ./CONTROL/ps_engine.pc > /usr/lib/pkgconfig/ps-engine.pc
	sudo /bin/bash ./CONTROL/engine_postinst configure

install_nite: install_openni nite_lib
	sudo mkdir -p /usr/include/nite
	sudo cp -f ./nite/build/Nite-1.3.1.5/Bin/*.so /usr/lib/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_0/Bin/*.so /usr/lib/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_1/Bin/*.so /usr/lib/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_0/Bin/*.so /usr/lib/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_1/Bin/*.so /usr/lib/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Include/* /usr/include/nite/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Data/*.xml /etc/openni/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_1/Data/* /etc/openni/
	sudo cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_1/Data/* /etc/openni/
	sudo /bin/bash ./CONTROL/nite_postinst configure

install: install_engine install_nite

uninstall_openni:
	sudo /bin/bash ./CONTROL/openni_prerm remove
	sudo rm -rf /usr/lib/libnimCodecs.so \
						/usr/lib/libnimMockNodes.so \
						/usr/lib/libnimRecorder.so \
						/usr/lib/libOpenNI.so \
						/usr/lib/libOpenNI.jni.so \
						/usr/include/openni \
						/usr/lib/pkgconfig/openni-dev.pc \
						/etc/openni \
						/usr/bin/niLicense \
						/usr/bin/niReg \
						/usr/share/openni \						

uninstall_engine:
	sudo /bin/bash ./CONTROL/engine_prerm remove
	sudo rm -rf /etc/udev/rules.d/55-primesense-usb.rules \
						/etc/modprobe.d/blacklist-psengine.conf \
						/etc/openni/GlobalDefaults.ini \
						/usr/lib/libXnCore.so \
						/usr/lib/libXnDDK.so \
						/usr/lib/libXnDeviceFile.so \
						/usr/lib/libXnDeviceSensorV2.so \
						/usr/lib/libXnFormats.so \
						/usr/bin/XnSensorServer

uninstall_nite:
	sudo /bin/bash ./CONTROL/nite_prerm remove
	sudo rm -rf /etc/openni/h.dat \
						/etc/openni/Sample-User.xml \
						/etc/openni/Nite.ini \
						/etc/openni/FeatureExtraction.ini \
						/etc/openni/s.dat \
						/etc/openni/Sample-Tracking.xml \
						/etc/openni/Sample-Scene.xml \
						/usr/lib/libXnVFeatures*.so \
						/usr/lib/libXnVHandGenerator*.so \
						/usr/lib/libXnVNite.so \
						/usr/include/nite

uninstall: uninstall_nite uninstall_engine uninstall_openni

$(OPENNI_PACKAGE_NAME).deb: ./CONTROL/openni_control ./CONTROL/openni_postinst ./CONTROL/openni_prerm openni_lib
	mkdir -p $(OPENNI_PACKAGE_NAME)/DEBIAN \
					 $(OPENNI_PACKAGE_NAME)/usr/bin \
					 $(OPENNI_PACKAGE_NAME)/usr/include/openni \
	         $(OPENNI_PACKAGE_NAME)/usr/lib \
	         $(OPENNI_PACKAGE_NAME)/etc/openni \
	         $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig \
	         $(OPENNI_PACKAGE_NAME)/usr/share/openni/doc \
	         $(OPENNI_PACKAGE_NAME)/usr/share/openni/samples
	cp -f ./CONTROL/openni_postinst $(OPENNI_PACKAGE_NAME)/DEBIAN/postinst
	cp -f ./CONTROL/openni_prerm $(OPENNI_PACKAGE_NAME)/DEBIAN/prerm
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Config/* $(OPENNI_PACKAGE_NAME)/etc/openni/
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Lib/*.so $(OPENNI_PACKAGE_NAME)/usr/lib/
	cp -r ./openni/build/openni/Platform/Linux-x86/Redist/Include/* $(OPENNI_PACKAGE_NAME)/usr/include/openni/
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Bin/* $(OPENNI_PACKAGE_NAME)/usr/bin/
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Documentation/html/* $(OPENNI_PACKAGE_NAME)/usr/share/openni/doc/
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Bin/Release/Sample-* $(OPENNI_PACKAGE_NAME)/usr/share/openni/samples/
	cp -f ./openni/build/openni/Platform/Linux-x86/Redist/Samples/Bin/Release/NiViewer $(OPENNI_PACKAGE_NAME)/usr/share/openni/samples/
	@sed s/__VERSION__/${OPENNI_VERSION}~${DISTRO}/ ./CONTROL/openni_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(OPENNI_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${OPENNI_VERSION}~${DISTRO}/ ./CONTROL/openni.pc > $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig/openni-dev.pc
	@dpkg-deb -b $(OPENNI_PACKAGE_NAME)

openni_lib:
	cd openni && $(MAKE)

$(ENGINE_PACKAGE_NAME).deb: ps_engine_lib
	mkdir -p $(ENGINE_PACKAGE_NAME)/DEBIAN \
					 $(ENGINE_PACKAGE_NAME)/usr/bin \
					 $(ENGINE_PACKAGE_NAME)/usr/lib \
					 $(ENGINE_PACKAGE_NAME)/etc/openni \
					 $(ENGINE_PACKAGE_NAME)/etc/udev/rules.d \
					 $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig
	cp -f ./CONTROL/engine_postinst $(ENGINE_PACKAGE_NAME)/DEBIAN/postinst
	cp -f ./CONTROL/engine_prerm $(ENGINE_PACKAGE_NAME)/DEBIAN/prerm
	cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Lib/*.so $(ENGINE_PACKAGE_NAME)/usr/lib/
	cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Config/*.ini $(ENGINE_PACKAGE_NAME)/etc/openni/
	cp -f ./ps_engine/build/ps_engine/Platform/Linux-x86/Redist/Bin/XnSensorServer $(ENGINE_PACKAGE_NAME)/usr/bin/
	cp -f ./CONTROL/55-primesense-usb.rules $(ENGINE_PACKAGE_NAME)/etc/udev/rules.d/
	cp -f ./CONTROL/blacklist-psengine.conf $(ENGINE_PACKAGE_NAME)/etc/modprobe.d/
	@sed s/__VERSION__/${PSENGINE_VERSION}~${DISTRO}/ ./CONTROL/engine_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(ENGINE_PACKAGE_NAME)/DEBIAN/control
	@sed -i s/__OPENNI_VERSION__/${OPENNI_VERSION}~${DISTRO}/ $(ENGINE_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${PSENGINE_VERSION}~${DISTRO}/ ./CONTROL/ps_engine.pc > $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig/ps-engine.pc
	@dpkg-deb -b $(ENGINE_PACKAGE_NAME)

ps_engine_lib: openni_lib
	cd ps_engine && $(MAKE)

$(NITE_PACKAGE_NAME).deb : nite_lib
	mkdir -p $(NITE_PACKAGE_NAME)/DEBIAN \
					 $(NITE_PACKAGE_NAME)/usr/lib \
					 $(NITE_PACKAGE_NAME)/usr/include/nite \
					 $(NITE_PACKAGE_NAME)/etc/openni \
					 $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig
	cp -f ./CONTROL/nite_postinst $(NITE_PACKAGE_NAME)/DEBIAN/postinst
	cp -f ./CONTROL/nite_prerm $(NITE_PACKAGE_NAME)/DEBIAN/prerm
	cp -f ./nite/build/Nite-1.3.1.5/Bin/*.so $(NITE_PACKAGE_NAME)/usr/lib/
	cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_0/Bin/*.so $(NITE_PACKAGE_NAME)/usr/lib/
	cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_1/Bin/*.so $(NITE_PACKAGE_NAME)/usr/lib/
	cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_0/Bin/*.so $(NITE_PACKAGE_NAME)/usr/lib/
	cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_1/Bin/*.so $(NITE_PACKAGE_NAME)/usr/lib/
	cp -f ./nite/build/Nite-1.3.1.5/Include/* $(NITE_PACKAGE_NAME)/usr/include/nite/
	cp -f ./nite/build/Nite-1.3.1.5/Data/*.xml $(NITE_PACKAGE_NAME)/etc/openni/
	cp -f ./nite/build/Nite-1.3.1.5/Features_1_3_1/Data/* $(NITE_PACKAGE_NAME)/etc/openni/
	cp -f ./nite/build/Nite-1.3.1.5/Hands_1_3_1/Data/* $(NITE_PACKAGE_NAME)/etc/openni/
	@sed s/__VERSION__/${NITE_VERSION}~${DISTRO}/ ./CONTROL/nite_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(NITE_PACKAGE_NAME)/DEBIAN/control
	@sed -i s/__OPENNI_VERSION__/${OPENNI_VERSION}~${DISTRO}/ $(NITE_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${NITE_VERSION}~${DISTRO}/ ./CONTROL/nite.pc > $(OPENNI_PACKAGE_NAME)/usr/lib/pkgconfig/nite-dev.pc
	@dpkg-deb -b $(NITE_PACKAGE_NAME)

nite_lib: openni_lib
	cd nite && $(MAKE)

clean:
	make -C ./openni wipe
	rm -rf $(OPENNI_PACKAGE_NAME)
	rm -f $(OPENNI_PACKAGE_NAME).deb
	make -C ./ps_engine wipe
	rm -rf $(ENGINE_PACKAGE_NAME)
	rm -f $(ENGINE_PACKAGE_NAME).deb
	make -C ./nite wipe
	rm -rf $(NITE_PACKAGE_NAME)
	rm -f $(NITE_PACKAGE_NAME).deb

