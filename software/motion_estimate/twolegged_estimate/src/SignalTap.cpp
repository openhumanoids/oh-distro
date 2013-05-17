

#include "SignalTap.hpp"
#include <iostream>
#include <sstream>



// creates a new_logging file
DataFileLogger::DataFileLogger() {
  suppress_logger = false;
}

DataFileLogger::DataFileLogger(std::string filename) {
	suppress_logger = false;
	fs.open(filename.c_str());
}

void DataFileLogger::Open(bool not_suppress, std::string filename) {
	suppress_logger = !not_suppress;
	
	if (!suppress_logger) {
		fs.open(filename.c_str());
	}
	
}

void DataFileLogger::Close() {
  if (!suppress_logger) {
    fs.close();
  }
	
}

void DataFileLogger::log(std::string data) {
  if (!suppress_logger) {
    fs << data;
  }
}

void DataFileLogger::operator<<(std::string to_log) {
  if (!suppress_logger) {
    log(to_log);
  }
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
	storedvalue = 0.;
	
	// first call flag is used to intialize the timer, to enable delay measuring
	first_call = true;
}

void SchmittTrigger::UpdateState(long present_time, double value) {
	if (first_call) {
		first_call = false;
		previous_time = present_time;
	}
	if (present_time < previous_time) {
		std::cout << "Warning SchmittTrigger object is jumping back in time -- behaviour unpredictable.\n";
	}
	
	storedvalue = value;
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

double SchmittTrigger::getCurrentValue() {
	return storedvalue;
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

// TODO -- made a second definition somewhere else, make this the only one
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
  first_pass = true;
  prev_time = 0;
  setSize(1);
}

void NumericalDiff::setSize(int len) {
  prev_sample.resize(len);
  size = len;
}

/*
Eigen::VectorXd NumericalDiff::diff(const double &time, const Eigen::VectorXd &sample) {


	std::cout << "NumericalDiff::diff(double , Eigen::VectorXd ) - DONT use this function -- to be depreciated ASAP\n";

	return Eigen::VectorXd();
}*/


Eigen::VectorXd NumericalDiff::diff(const unsigned long long &ts, const Eigen::VectorXd &sample) {
	if (first_pass && sample.size() != size) {
		setSize(sample.size());
		first_pass = false;

		std::cout << "NumericalDiff::diff -- Size of the vector to be used automatically adjusted on the first pass.\n";
	}

  Eigen::VectorXd returnval(size);
  
  returnval = (sample - prev_sample)/(ts - prev_time)*1E-6;
  
  prev_time = ts;
  prev_sample = sample;
  
  return returnval;
}

void NumericalDiff::diff(const unsigned long long &ts, int count, double sample[]) {

	Eigen::VectorXd data(count);
	Eigen::VectorXd returndata(count);


	for (int i=0;i<count;i++) {
		data(i) = sample[i];
	}

	returndata = diff(ts,data);

	for (int i=0;i<count;i++) {
		sample[i] = returndata(i);
	}
}


TrapezoidalInt::TrapezoidalInt() {
	setSize(1);
	u_stime = 0;
	Init();
}


void TrapezoidalInt::Init() {
	first_pass = true;
	size_set = false;
}

/*
TrapezoidalInt::~TrapezoidalInt() {

}
*/

void TrapezoidalInt::setSize(const int &s) {
	assert(s>-1);
	size_set = true;
	size = s;
	int_dx.resize(s);
	prev_dx.resize(s);

	prev_dx.setZero();
	reset_in_time();

}

void TrapezoidalInt::reset_in_time() {
	int_dx.setZero();
}

Eigen::VectorXd TrapezoidalInt::integrate(const unsigned long long &u_ts, const Eigen::VectorXd &dx) {
	// TODO -- This function should use the u_time stamp from zero. Then it can also be used as an integral time counter and makes best possible use of the available time variable dynamic range

	if (u_ts < u_stime) {
		std::cout << "TrapezoidalInt::integrate is jumping back in time. This is not supposed to happen -- behavior will be unpredictable.\n";
		u_stime = u_ts;
	}

	// Eigen does not ensure self assigned computations x = x + 1
	Eigen::VectorXd newval(size);
	newval = int_dx + 0.5*(dx + prev_dx)*(u_ts-u_stime)*1E-6;
	int_dx = newval;

	u_stime = u_ts;
	prev_dx = dx;

	return int_dx;
}

Eigen::VectorXd TrapezoidalInt::integrate(const unsigned long long &ts, const int &num_joints, const double samples[]) {
	if (first_pass && !size_set) {
		setSize(num_joints);
		first_pass = false;

		std::cout << "Size of TrapezoidalInt was not set, but automatically adjusted to the first received vector size of: " << num_joints << "\n";
	}

	Eigen::VectorXd to_int(num_joints);

	for (int i=0;i<num_joints;i++) {
		to_int(i) = samples[i];
	}

	return integrate(ts, to_int);
}


RateChange::RateChange() {
	std::cout << "RateChange -- assuming rate of 1Hz\n";
	setDesiredPeriod_us(0,(unsigned long)1E6);
}

RateChange::RateChange(const unsigned long &period_us) {
	setDesiredPeriod_us(0, period_us);
}

void RateChange::setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us) {
	desired_period_us = period_us;
	prev_u_time = start_u_time;
}

bool RateChange::checkNewRateTrigger(const unsigned long long &cur_u_time) {

	if ((cur_u_time - prev_u_time) >= desired_period_us) {
		// This is a rate transition trigger event
		prev_u_time = cur_u_time;

		return true;
	}

	return false;
}
