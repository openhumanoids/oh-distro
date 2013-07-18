LCMLIB=  ../../lib/libfbntypes.a 
PSOCKET=../PracticalSocket/PracticalSocket.cpp

all: listenSocket manualRecon manualLCM listenTestLCM writeTestLCM

listenSocket: listenSocket.cpp $(PSOCKET)
	g++ -Wall -ansi -pedantic -g listenSocket.cpp $(PSOCKET) -o listenSocket

manualRecon: manualRecon.cpp $(PSOCKET)
	g++ manualRecon.cpp $(PSOCKET) -o manualRecon

manualLCM: manualLCM.cpp $(LCMLIB)  
	g++ manualLCM.cpp -o manualLCM `pkg-config --cflags --libs lcm`  $(LCMLIB)  -I../../../../lcmtypes/src/lcmtypes/

listenTestLCM: listenTestLCM.cpp $(LCMLIB)
	g++ listenTestLCM.cpp -o listenTestLCM `pkg-config --cflags --libs lcm`  $(LCMLIB)

writeTestLCM: writeTestLCM.cpp $(LCMLIB)  
	g++ writeTestLCM.cpp -o writeTestLCM `pkg-config --cflags --libs lcm`  $(LCMLIB)  -I../../../../lcmtypes/src/lcmtypes/

clean:
	rm -f listenSocket manualRecon manualLCM listenTestLCM