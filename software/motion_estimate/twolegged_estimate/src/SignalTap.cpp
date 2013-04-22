

#include "SignalTap.hpp"



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
