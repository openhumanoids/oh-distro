
.PHONY: clean

multisense_lib:
	#mkdir -p build
	#cd build && tar -xzf ../2012-01-11_SensorPodExample_release-1_0.tgz
	mkdir -p sensor_pod_source/SensorPodExample/build
	cd sensor_pod_source/SensorPodExample/build && cmake ..
	${MAKE} -C sensor_pod_source/SensorPodExample/build
	mkdir -p lib
	cp sensor_pod_source/SensorPodExample/bin/*.so lib
	mkdir -p include/LibThread
	mkdir -p include/LibPlatform
	mkdir -p include/LibSensorPodCommunications
	cp sensor_pod_source/SensorPodExample/source/LibSensorPodCommunications/*.h include/LibSensorPodCommunications
	cp sensor_pod_source/SensorPodExample/sharedTech/utilities/LibThread/*.hh   include/LibThread
	cp sensor_pod_source/SensorPodExample/sharedTech/utilities/LibPlatform/*.hh include/LibPlatform
	touch multisense_lib

clean:
	rm -rf sensor_pod_source/SensorPodExample/build
	rm -rf sensor_pod_source/SensorPodExample/bin
	rm -f multisense_lib
	rm -rf include
	rm -rf lib
