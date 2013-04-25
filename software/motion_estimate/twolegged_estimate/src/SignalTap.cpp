

#include "SignalTap.hpp"
#include <iostream>



// creates a new_logging file
DataFileLogger::DataFileLogger() {
	
	
}

void DataFileLogger::Open(std::string filename) {
	
	fs.open(filename.c_str());
	
}

void DataFileLogger::Close() {
	fs.close();
	
}

void DataFileLogger::log(std::string data) {
	//return std::ofstream();
	fs << data;
}


SchmittTrigger::SchmittTrigger(double lt, double ht, long delay) {
	low_threshold = lt;
	high_threshold = ht;
	time_delay = delay;
	Reset();
}

void SchmittTrigger::Reset() {
	current_status = false;
	previous_time = 0; // how to handle this when it gets called for the first time.. should we have a flag and then a reset status
	timer = 0;
	first_call = true;
}

void SchmittTrigger::UpdateState(long present_time, double value) {
	if (first_call) {
		first_call = false;
		previous_time = present_time;
	}
	std::cout << "ST: " << value << "N , timer: " << timer << " us, status is: " << current_status << std::endl;
	if (current_status)
		{
			if (value <= low_threshold)
			{
				std::cout << "below threshold\n";
				if (timer > time_delay) {
					current_status = false;
				} else {
					timer += (present_time-previous_time);
				}
			} else {
				timer = 0;
			}
			
		} else {
			if (value >= high_threshold) {
				std::cout << "above threshold\n";
				if (timer > time_delay) {
					current_status = true;
				} else {
					timer += (present_time-previous_time);
				}
			} else {
				timer = 0;
			}
		}
	previous_time = present_time;
}

float SchmittTrigger::getState() {
	return (current_status ? 1.f : 0.f);
}
