// 1. I think I saw drc-foot-state publish that both feet were in contact when it was actually in the air
// 2. Obviously the foot will not make perfectly flat contact with the ground - esp with BDI's walking mode
//    Thus integrating JK or even just using constraints between consequtive JK measurements seems quite flawed

#include <iostream>
#include <sstream>      // std::stringstream



class foot_contact {
  public:
    foot_contact ();
    
    // returns: 
    // -2 logic in error 
    // -1 not initialized yet
    // 0 transitioning onto left foot now. switch leg odom to left
    // 1 transitioning onto right foot now. switch leg odom to right
    // 2 continuing to have left as primary foot [system an initialize from here]
    // 3 continuing to have right as primary foot
    int update(bool left_contact, bool right_contact);


  private:
    // initialization condition is both feet in contact with the ground
    bool initialized_;
    
    // -1 unknown mode
    // 0 both in contact, left has been for longer
    // 1 both in contact, right has been for longer
    // 2 left in contact, right raised
    // 3 right in contact, left raised
    // 4 both raised - not possible --- except...
    int mode_;    
    
    // level of verbosity:
    // 0 - say nothing except pre-initialization
    // 1 - say when the primary foot is switched and pre-initialization
    // 2 - say when a mode transition
    // 3 - say something each iteration
    int verbose_;

};