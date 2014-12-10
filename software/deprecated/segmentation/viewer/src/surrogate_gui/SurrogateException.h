/*
 * SurrogateException.h
 *
 *  Created on: Jan 23, 2012
 *      Author: mfleder
 */

#ifndef SURROGATEEXCEPTION_H_
#define SURROGATEEXCEPTION_H_

#include <stdexcept>
#include <string>
#include <exception>
#include <iostream>
#include <signal.h>
#include <execinfo.h>
#include <stdlib.h>
using namespace std;

namespace surrogate_gui
{

	class SurrogateException : public std::runtime_error
	{
		public:
			SurrogateException(const std::string &message) : std::runtime_error(message)
			{
				void *array[25];
				int nSize = backtrace(array, 25);
				char ** symbols = backtrace_symbols(array,nSize);

				for (int i = 0; i < nSize; i++)
					cout << symbols[i] << endl;
			}

		virtual ~SurrogateException() throw(){};
	};

} //namespace surrogate_gui


#endif /* SURROGATEEXCEPTION_H_ */
