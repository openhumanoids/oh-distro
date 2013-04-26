

#include "SignalTap.hpp"
#include <iostream>
#include <sstream>



// creates a new_logging file
DataFileLogger::DataFileLogger() {
	
	
}

void DataFileLogger::Open(bool not_suppress, std::string filename) {
	suppress_logger = !not_suppress;
	
	if (!suppress_logger)
		fs.open(filename.c_str());
	
}

void DataFileLogger::Close() {
  if (!suppress_logger)
    fs.close();
	
}

void DataFileLogger::log(std::string data) {
  if (!suppress_logger)
    fs << data;
}

void DataFileLogger::operator<<(std::string to_log) {
  if (!suppress_logger)
    log(to_log);
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
	
	// first call flag is used to intialize the timer, to enable delay measuring
	first_call = true;
}

void SchmittTrigger::UpdateState(long present_time, double value) {
	if (first_call) {
		first_call = false;
		previous_time = present_time;
	}
	//std::cout << "ST: " << value << "N , timer: " << timer << " us, status is: " << current_status << std::endl;
	if (current_status)
		{
			if (value <= low_threshold)
			{
				//std::cout << "below threshold\n";
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
				//std::cout << "above threshold\n";
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

BipolarSchmittTrigger::BipolarSchmittTrigger(double lt, double ht, long delay) {
	_trigger = new SchmittTrigger(lt, ht, delay);
	//_lowside  = new SchmittTrigger(-lt, -ht, delay);
	
	prev_value = 0.;
}

BipolarSchmittTrigger::~BipolarSchmittTrigger() {
	std::cout << "Closing out a BipolarSchmittTrigger object\n";
	
	delete _trigger;
	//delete _lowside;
}

int sign(double value) {
	if (value >= 0)
		return 1;
	return -1;
}

void BipolarSchmittTrigger::UpdateState(long present_time, double value) {
	// Think we need to reset on a sign flip event
	//if (sign(value) != sign(prev_value)) {
	//	_trigger->Reset();
	//}
	
	_trigger->UpdateState(present_time, abs(value));
	//_lowside->UpdateState(present_time, value);
	prev_value = value;
}

void BipolarSchmittTrigger::Reset() {
	_trigger->Reset();
	//_lowside->Reset();
}

float BipolarSchmittTrigger::getState() {
	if (_trigger->getState() > 0.9f)
		return 1.f;
	return 0.f;
}

NumericalDiff::NumericalDiff() {
  prev_time = 0;
  setSize(1);
}

void NumericalDiff::setSize(int len) {
  prev_sample.resize(len);
  size = len;
}

Eigen::VectorXd NumericalDiff::diff(double time, const Eigen::VectorXd &sample) {
  Eigen::VectorXd returnval(size);
  
  returnval = (sample - prev_sample)/(time - prev_time);
  
  prev_time = time;
  prev_sample = sample;
  
  return returnval;
}
