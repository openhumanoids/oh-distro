#ifndef FOOTCONTACTALT_H_
#define FOOTCONTACTALT_H_

#include "estimate_tools/SignalTap.hpp"
#define LEFTFOOT  0
#define RIGHTFOOT 1


enum footid_alt { 
  F_UNKNOWN=-1,
  F_LEFT  =0,
  F_RIGHT =1,
};

namespace TwoLegs {

class FootContactAlt {
  private:

    // Parameters:
    float schmitt_level_;
    int64_t transition_timeout_;

    /////////////////////////////////////////////////
     
    footid_alt standing_foot;
    float expectedweight;
    int64_t transition_timespan;
    bool foottransitionintermediateflag;

    float l_foot_force_z;
    float r_foot_force_z;

    int64_t lcmutime;
    int64_t deltautime;

    float getPrimaryFootZforce();
    float getSecondaryFootZforce();

    SchmittTrigger* left_contact_state_strong_;
    SchmittTrigger* right_contact_state_strong_;

    bool verbose_;
  public:     
    FootContactAlt (bool _log_data_files, const float atlasWeight);

    void terminate();

    int DetectFootTransition(int64_t utime, float leftz, float rightz);

    footid_alt getStandingFoot();
    // Used internally to change the active foot for motion estimation
    void setStandingFoot(footid_alt foot);

    // Return which foot is which - these are based on the RIGHTFOOT and LEFTFOOT defines in TwoLegsEstiamte_types.h
    footid_alt getSecondaryFoot();

    float leftContactStatus();
    float rightContactStatus();

    void updateSingleFootContactStates(long utime, const double left_force, const double right_force);
};

}

#endif /*FOOTCONTACTALT_H_*/
