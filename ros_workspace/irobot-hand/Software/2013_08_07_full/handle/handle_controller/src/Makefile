
all:
	arm-commonos-linux-gnueabi-g++ controller.cpp ../../handle_lib/src/packing.cpp -Wall -pthread -ggdb3 -o handle_controller

clean:
	rm *~ handle_controller

install:
	scp handle_controller root@armH-palm-1:
