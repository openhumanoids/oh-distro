/*
   Copyright 2012 Cameron Tinker

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <QtGui/QApplication>
#include "mainwindow.h"
#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>

using namespace action_authoring;
using namespace boost;

void runLcmHandleLoop(const shared_ptr<lcm::LCM> theLcm)
{
	//lcm loop
	cout << "\nstarting lcm loop" << endl;
	while (0 == theLcm->handle());

}

int main(int argc, char *argv[])
{
	//construct an lcm object
	shared_ptr<lcm::LCM> theLcm(new lcm::LCM());
	if (!theLcm->good())
	{
		cerr << "Cannot create lcm object" << endl;
	    return -1;
	}

	boost::thread testThread = boost::thread(runLcmHandleLoop, theLcm);

	//=======GUI stuff
    QApplication a(argc, argv);
    MainWindow w(theLcm);
    w.show();
    
    return a.exec();
}


