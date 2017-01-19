#include "drc_utils/LCMTuner.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(){
	using namespace LCMTuner_namespace;
	LCMTuner lcmTuner("test1");
	double x = 0;
	double min_val = 0;
	double max_val = 10;
	std::cout << "LCMTuner name is " << lcmTuner.name << std::endl;
	lcmTuner.add_tunable_variable("x", x, min_val, max_val);
	std::cout << "starting lcm tuner" << std::endl;
	lcmTuner.start();
	std::cout << "lcm tuner started, entering infinite loop" << std::endl;

	double y = 1;
	LCMTuner lcmTuner_2("test2");
	lcmTuner_2.add_tunable_variable("y", y, min_val, max_val);
	lcmTuner_2.start();

	while(true){
		std::cout << "-------------" << std::endl;
		std::cout << "x = " << x << std::endl;
		std::cout << "y = " << y << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	};
}