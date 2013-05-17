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
	DataFileLogger(std::string filename);
	
	void Open(bool not_suppress, std::string filename);
	void Close();
	
	void log(std::string data);
	void operator<<(std::string to_log);
};

class SchmittTrigger {
private:
	bool current_status;
	double storedvalue;
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
	double getCurrentValue();
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
	unsigned long long prev_time;

	bool first_pass;
public:
	NumericalDiff();
	void setSize(int size);
	
	//Eigen::VectorXd diff(const double &time, const Eigen::VectorXd &sample);

	Eigen::VectorXd diff(const unsigned long long &ts, const Eigen::VectorXd &sample);
	void diff(const unsigned long long &ts, int count, double sample[]);
	
};

class TrapezoidalInt {
private:
	Eigen::VectorXd int_dx;
	Eigen::VectorXd prev_dx;

	unsigned long long u_stime;
	int size;

	bool first_pass;
	bool size_set;


	void Init();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TrapezoidalInt();
	//~TrapezoidalInt();

	void setSize(const int &s);

	Eigen::VectorXd integrate(const unsigned long long &u_ts, const Eigen::VectorXd &dx);
	Eigen::VectorXd integrate(const unsigned long long &ts, const int &num_joints, const double samples[]);

	void reset_in_time();

};

class RateChange {
private:
	//int rate_ratio;
	//int stage_counter;
	unsigned long long prev_u_time;

	unsigned long desired_period_us;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	RateChange();
	RateChange(const unsigned long &period_us);

	void setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us);

	bool checkNewRateTrigger(const unsigned long long &cur_u_time);

};

#endif /*SIGNALTAP_HPP_*/
