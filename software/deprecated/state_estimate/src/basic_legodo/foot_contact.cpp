#include "foot_contact.hpp"

using namespace std;

foot_contact::foot_contact (){
   initialized_ = false;
   mode_ = -1;
   
   verbose_ = 1;
}

std::string print_variables(int mode_, bool left_contact, bool right_contact){
  std::stringstream ss;
  ss << mode_ << " | " << left_contact << "" << right_contact;
  return ss.str();
}

int foot_contact::update (bool left_contact, bool right_contact ){
  string pv = print_variables(mode_,left_contact,right_contact);
  
  if (!initialized_){
    if (left_contact && right_contact){
      std::cout << pv << " | Initializing. both in contact with left foot as primary\n";
      mode_ = 0;
      initialized_ = true;
      return 2;
    }else{
      std::cout << pv << " | Not initialized yet: both feet are not in contact\n";
      return -1; 
    }
  }
  
  if (mode_ ==0){
    if (left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << " - primary left, both in contact. still\n";
      return 2;
    }else if (left_contact  && !right_contact){
      if (verbose_ >= 2) std::cout << pv << " | primary left still in contact but right now raised\n";
      mode_ = 2;
      return 2;
    }else if (!left_contact  && right_contact){
      if (verbose_ >= 1) std::cout << pv << " | switching primary to right. left now raised. LEG ODOM SWITCH\n";
      mode_ = 3;
      return 1;
    }else{
      std::cout << "Unknown Transition: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
      int blah;
      cin >> blah;
    }
  }
  
  if (mode_ == 1){
    if (left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << " - primary right, both in contact. still\n";
      return 3;
    }else if (left_contact  && !right_contact){
      if (verbose_ >= 1) std::cout << pv <<  " | switching primary to left. right now raised. LEG ODOM SWITCH\n";
      mode_ = 2;
      return 0;
    }else if (!left_contact  && right_contact){
      if (verbose_ >= 2) std::cout << pv << " | primary right still in contact but left now raised\n";
      mode_ = 3;
      return 3;
    }else{
      std::cout << "Unknown Transition: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
      int blah;
      cin >> blah;
    }
  }
  
  
  if (mode_ == 2){
    if (left_contact  && !right_contact){
      if (verbose_ >= 3) std::cout << pv << " - primary left in contact and right raised. still\n";
      return 2;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 2) std::cout << pv << " | primary left still in contact. right now in contact again\n";
      mode_ = 0;
      return 2;
    }else{
      std::cout << "Unknown Transition: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
      int blah;
      cin >> blah;
    }
  }
  
  if (mode_ == 3){
    if (!left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << " - primary right in contact and left raised. still\n";
      return 3;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 2) std::cout << pv << " | primary right still in contact. left now in contact again\n";
      mode_ = 1;
      return 3;
    }else{
      std::cout << "Unknown Transition: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
      int blah;
      cin >> blah;
    }
  }
  
  std::cout << "Unknown fallthrough: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
  int blah;
  cin >> blah;  
  return -2;
}