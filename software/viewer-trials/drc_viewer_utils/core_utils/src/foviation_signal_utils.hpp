#ifndef FOVIATION_SIGNAL_UTILS_HPP
#define FOVIATION_SIGNAL_UTILS_HPP

#include <iostream>
#include <boost/signals2.hpp> 
#include <boost/function.hpp>
#include <boost/bind.hpp>


using namespace std;
namespace visualization_utils {


  // A global keyboard signal with the keyval integer and a boolean 
  // indicating if it's a key-press or a key-release.
  typedef boost::signals2::signal<void(void*,string,bool)> RendererFoviationSignal; 
  typedef boost::shared_ptr<RendererFoviationSignal> RendererFoviationSignalRef;

    // should this be interface class RendererFoviationSignalHandler?
    class RendererFoviationSignalHandler 
    {
      public:
        RendererFoviationSignalHandler(RendererFoviationSignalRef signalRef, boost::function<void(void*,string,bool)> func):_user_callback(func)
        {
          _connection =  signalRef->connect(boost::bind(&RendererFoviationSignalHandler::callback,this,_1,_2,_3));
        };
        
        ~RendererFoviationSignalHandler(){
          _connection.disconnect();
        }

        void callback(void* user_data,string renderer_name, bool toggle)
        {
          /*if(toggle) 
          {
            cout << "Foviation Requested From: " <<  renderer_name << endl;
          }
          else {
            cout << "UnFoviation Requested From: " << renderer_name << endl;
          }*/
          if(!_user_callback.empty())
             _user_callback(user_data,renderer_name,toggle); 
          else
             cerr << "ERROR in visualization_utils::RendererFoviationSignalHandler:  user callback function reference is empty!" << endl;
        }
        
        private:
        boost::signals2::connection _connection; 
        boost::function<void (void*,string,bool)> _user_callback; 
        
    };

} // end namespace 


#endif //FOVIATION_SIGNAL_UTILS_HPP
