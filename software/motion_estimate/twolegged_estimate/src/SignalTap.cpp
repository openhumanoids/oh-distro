

#include "SignalTap.hpp"

Filter::Filter() {
	//std::cout << "A new filtering object has been created\n";
	//samples = Eigen::MatrixXd(10,1);
	samples.resize(10);
	
	//std::cout << "The size of the samples vector is: " << samples.size() << std::endl;
}


LowPassFilter::LowPassFilter() {
	std::cout << "A new LowPassFilter object has been created\n";
		
}


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
