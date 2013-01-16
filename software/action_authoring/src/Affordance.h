#pragma once

#include <string>

class Affordance {
private:
	std::string m_name;

public:
	Affordance(std::string name);
	void setName(std::string name);
	std::string getName() { return m_name; }
};
