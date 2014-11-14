
.PHONY: clean

multisense_lib:
	mkdir -p sensor_api/build
	cd sensor_api/build && cmake ..
	${MAKE} -C sensor_api/build
	mkdir -p lib
	cp -a sensor_api/bin/libMultiSense* lib/
	mkdir -p include
	cp sensor_api/source/LibMultiSense/*.hh include/
	touch multisense_lib

clean:
	rm -rf sensor_api/build
	rm -rf sensor_api/bin
	rm -f multisense_lib
	rm -rf include
	rm -rf lib
