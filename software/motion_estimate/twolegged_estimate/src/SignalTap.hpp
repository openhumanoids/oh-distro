#ifndef SIGNALTAP_HPP_
#define SIGNALTAP_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>

// objects and functions for generic operations on data

class DataFileLogger {
private:
	std::ofstream fs;
	bool suppress_logger;
	
public:
	DataFileLogger();
	
	void Open(bool not_suppress, std::string filename);
	void Close();
	
	void log(std::string data);
	void operator<<(std::string to_log);
};

class SchmittTrigger {
private:
	bool current_status;
	long timer;
	long time_delay;
	long previous_time;
	double low_threshold;
	double high_threshold;
	bool first_call;
	
public:
	SchmittTrigger(double lt, double ht, long delay);
	void UpdateState(long present_time, double value);
	void Reset();
	float getState();
};

#endif /*SIGNALTAP_HPP_*/
