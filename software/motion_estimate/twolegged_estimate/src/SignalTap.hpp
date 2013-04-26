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

class BipolarSchmittTrigger {
private:
	double prev_value;
	SchmittTrigger* _trigger;
	//SchmittTrigger* _lowside;
public:
	BipolarSchmittTrigger(double lt, double ht, long delay);
	~BipolarSchmittTrigger();
	void UpdateState(long present_time, double value);
	void Reset();
	float getState();
	
};


// This class should be updated to support vectored signals, rather than the present single channel
// Maybe the signals should be handled in a vector, as that will separate this module from the Eignen requirement
class NumericalDiff {
private:
	Eigen::VectorXd prev_sample;
	int size;
	double prev_time;
public:
	NumericalDiff();
	void setSize(int size);
	
	Eigen::VectorXd diff(double time, const Eigen::VectorXd &sample);
	
	
};

#endif /*SIGNALTAP_HPP_*/
