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
	
	
public:
	DataFileLogger();
	
	void Open(std::string filename);
	void Close();
	
	void log(std::string data);
};


#endif /*SIGNALTAP_HPP_*/
