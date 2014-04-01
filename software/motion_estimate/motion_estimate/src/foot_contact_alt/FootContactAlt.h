#ifndef FOOTCONTACTALT_H_
#define FOOTCONTACTALT_H_

#include "estimate_tools/SignalTap.hpp"

// TODO: these enum duplicate one another and also something in Dehann's classifier, clean this up
#define LEFTFOOT  0
#define RIGHTFOOT 1

enum footid_alt { 
  F_UNKNOWN=-1,
  F_LEFT  =0,
  F_RIGHT =1,
};


// a variable used ephemerially to capture foot transition decisions
// the primary_foot_ is transitioned to the corresponding foot by the integration
enum contact_status_id { 
  F_STATUS_UNKNOWN = -1,
  F_LEFT_NEW   = 0, // just decided that left is in (primary) contact
  F_RIGHT_NEW   = 1, // just decided that right is in (primary) contact
  F_LEFT_FIXED = 2, // left continues to be in primary contact
  F_RIGHT_FIXED = 3, // right continues to be in primary contact
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

    contact_status_id DetectFootTransition(int64_t utime, float leftz, float rightz);

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
