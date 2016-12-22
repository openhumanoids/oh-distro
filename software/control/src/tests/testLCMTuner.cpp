#include "drc_utils/LCMTuner.hpp"
#include <iostream>

int main(){
	using namespace LCMTuner_namespace;
	LCMTuner lcmTuner("testTuner");
	double x = 0;
	double min_val = 0;
	double max_val = 10;
	std::cout << "LCMTuner name is " << lcmTuner.name << std::endl;
	lcmTuner.add_tunable_variable("test_var", x, min_val, max_val);
	std::cout << "starting lcm tuner" << std::endl;
	lcmTuner.start();
	std::cout << "lcm tuner started, entering infinite loop" << std::endl;

	while(true){};
}