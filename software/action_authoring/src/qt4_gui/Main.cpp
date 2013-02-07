#include "MainWindow.h"

#include <QtGui/QApplication>
#include <lcm/lcm-cpp.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace state;
using namespace action_authoring;
using namespace affordance;

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
    
    //=======GUI 
    QApplication a(argc, argv);
    MainWindow w(theLcm);
    w.show();
    return a.exec();
}
