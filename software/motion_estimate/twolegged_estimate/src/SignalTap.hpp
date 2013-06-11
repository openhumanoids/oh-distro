#ifndef SIGNALTAP_HPP_
#define SIGNALTAP_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <algorithm>
#include <stdio.h>

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
	SchmittTrigger();
	SchmittTrigger(double lt, double ht, long delay);
	void setParameters(double lt, double ht, long delay);
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
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	NumericalDiff();
	void setSize(int size);
	
	//Eigen::VectorXd diff(const double &time, const Eigen::VectorXd &sample);

	Eigen::VectorXd diff(const unsigned long long &ts, const Eigen::VectorXd &sample);
	void diff(const unsigned long long &ts, int count, double sample[]);
	
};

class DistributedDiff {
private:
	Eigen::VectorXd timespans;
	Eigen::VectorXd weights;
	int individual_diffs;
	int size;
	int hist_len;
	bool sizeset;
	bool hist_len_set;
	unsigned long max_hist_utime;
	unsigned long period;
	int firstpasses;

	Eigen::VectorXd cum_sum;
	Eigen::VectorXd cum_sum_buf;

	Eigen::VectorXd alldiffs;

	// This could be built into a single struct if you bored
	boost::circular_buffer<Eigen::VectorXd*> _state_hist;
	boost::circular_buffer<unsigned long long> utimes;

	void addDataToBuffer(const unsigned long long &u_ts, const Eigen::VectorXd &samples);
	Eigen::VectorXd searchHistElement(const unsigned long &delta_u_ts, unsigned long *actual_delta_u_ts);
	Eigen::VectorXd findDifferentialFromLatest(const unsigned long &desired_hist_ut);
	Eigen::VectorXd FindDifferentials();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DistributedDiff();
	~DistributedDiff();

	bool ready();

	void setSize(int s);

	void InitializeTaps(int hist_length, const long &per, const Eigen::VectorXd &w, const Eigen::VectorXd &t);
	void ParameterFileInit();

	Eigen::VectorXd diff(const unsigned long long &u_ts, const Eigen::VectorXd &samples);

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
	void setStateTo(const Eigen::VectorXd &set_val);

	Eigen::VectorXd integrate(const unsigned long long &u_ts, const Eigen::VectorXd &dx);
	Eigen::VectorXd integrate(const unsigned long long &ts, const int &num_joints, const double samples[]);

	Eigen::VectorXd getVal();

	void reset_in_time();

};

class RateChange {
private:
	unsigned long long prev_u_time;
	unsigned long desired_period_us;
	int size;

	Eigen::VectorXd state;
	Eigen::VectorXd int_vals;
	TrapezoidalInt generic_integrator;
	NumericalDiff generic_diff;

	bool checkNewRateTrigger(const unsigned long long &cur_u_time);


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	RateChange();
	RateChange(const unsigned long &period_us);

	void setSize(const int &s);

	void setDesiredPeriod_us(const unsigned long long &start_u_time, const unsigned long &period_us);
	bool genericRateChange(const unsigned long long &uts, const Eigen::VectorXd &samples, Eigen::VectorXd &returnval);

};

class MedianFilter {
private:
	boost::circular_buffer<double> data;

	int len;
	bool lengthset;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MedianFilter();
	void setLength(const int &length);
	double processSample(const double &sample);
};


template<int N>
class BlipFilter {
private:
  typedef Eigen::Matrix<double,N,1> VecType;

  int mMedianFilterLength;
  double mMinBlipMagnitude;
  boost::circular_buffer<VecType> mData;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BlipFilter();
  void setMedianFilterLength(const int iLength);
  void setMinBlipMagnitude(const double iMag);
  VecType getValue(const VecType& iSample);
};


#endif /*SIGNALTAP_HPP_*/
