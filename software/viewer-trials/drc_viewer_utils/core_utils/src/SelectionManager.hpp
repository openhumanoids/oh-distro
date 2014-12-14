#ifndef SELECTION_MANAGER_HPP
#define SELECTION_MANAGER_HPP

#include "keyboard_signal_utils.hpp"
#include <algorithm> // using std::find

using namespace std;

namespace visualization_utils
{

 // A class than manages a list of strings of selected objects in the renderer space. 
 // Each selection is the result of a auctioning process that competes for a mouse click. 
 // On selection the selected objects id is appended to the list of strings in this class.
 // Supports, shift + selection for selecting multiple objects.
 
 
 // Differentiate between marker selection and item selection?
 // Only one marker can be selected at any given point.

 
    class SelectionManager 
    {
      public:
        SelectionManager(KeyboardSignalRef signalRef)
        {
          // make a connection to the keyboard signal
          _connection =  signalRef->connect(boost::bind(&SelectionManager::callback,this,_1,_2)); 
        };
        
        ~SelectionManager(){
          _connection.disconnect();
        };
        
        // signal callback
        void callback(int keyval, bool is_pressed)
        {
          // keep track of latest keyboard state
          _is_key_pressed = is_pressed;
          _latest_keyval = keyval;
          
          //if((_is_key_pressed)&&(_latest_keyval == SHIFT_L||_latest_keyval == SHIFT_R ))
            //_selection_history.clear();
        };
        
        //core methods
        void add(std::string &id)
        {
        
          std::vector<std::string>::iterator found;
          found = std::find(_selection_history.begin(), _selection_history.end(), id);
          if(found == _selection_history.end())
          {   
            if((_is_key_pressed)&&(_latest_keyval == SHIFT_L||_latest_keyval == SHIFT_R ))
            {
              _selection_history.push_back(id); // shift select
            }
            else{
              _selection_history.clear();
              _selection_history.push_back(id);
            }
          }
        };
        
        bool remove(std::string &id)
        {
          std::vector<std::string>::iterator found;
          found = std::find(_selection_history.begin(), _selection_history.end(), id);
          if(found == _selection_history.end())
          {      
           return false;
          }
          else
          {
            _selection_history.erase(found);
            return true;
          }
        };  
        
        void clear()
        {
         _selection_history.clear();
        };      
        
        // useful for debug
        void print()
        {
          cout<< "Selection Order\n";
         for(size_t i=0;i<_selection_history.size();i++)
         {
          cout << i << " : " << _selection_history[i] << endl;
         }
        };
        
        
        //access methods
        int get_selection_cnt(void)
        {
            return _selection_history.size();
        };
        
        bool last_has_token(std::string token)
        {
          std::string id;
          get_last(id);
          size_t found;
          found=id.find(token);
          //latest selection contains token
          return (found!=std::string::npos);
        };
         
        bool get_last(std::string &id)
        {
          if(_selection_history.size()>0)
          {
            int ind = _selection_history.size()-1;
            id = _selection_history[ind];
            return true;
          }
          return false;
        };
        
        bool is_selected(std::string &id)
        {
          std::vector<std::string>::const_iterator found;
          found = std::find (_selection_history.begin(), _selection_history.end(), id);
          bool success=(found != _selection_history.end());
          return success;
        };
        
        int get_selection_order(std::string &id)
        {
          std::vector<std::string>::const_iterator found;
          found = std::find (_selection_history.begin(), _selection_history.end(), id);
          if(found != _selection_history.end())
          {
            int index  = (found - _selection_history.begin());
            return index+1;
          }
          return -1;
        };
        
        bool is_shift_pressed()
        {
         return ((_is_key_pressed)&&(_latest_keyval == SHIFT_L||_latest_keyval == SHIFT_R ));
        };
        
        //core data object    
        std::vector<std::string> _selection_history;
        //std::string _marker_selection; TODO:??? 
        private:
         boost::signals2::connection _connection; 
         
         // latest key state
         bool _is_key_pressed;
         int _latest_keyval;
        
    };
}; // end namespace


#endif //SELECTION_MANAGER_HPP
