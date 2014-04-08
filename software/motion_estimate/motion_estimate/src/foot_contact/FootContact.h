#ifndef FOOTCONTACT_H_
#define FOOTCONTACT_H_

#include "estimate_tools/SignalTap.hpp"
#define LEFTFOOT  0
#define RIGHTFOOT 1


enum footid { 
  FOOT_UNKNOWN=-1,
  FOOT_LEFT  =0,
  FOOT_RIGHT =1,
};

namespace TwoLegs {

class FootContact {
  private:
    // Parameters:
    float schmitt_level_;
    int64_t transition_timeout_;

    /////////////////////////////////////////////////
    footid standing_foot;
    float atlas_weight_;
    int64_t transition_timespan;
    bool foottransitionintermediateflag;

    float l_foot_force_z;
    float r_foot_force_z;

    int64_t lcmutime;
    int64_t deltautime;

    float getPrimaryFootZforce();
    float getSecondaryFootZforce();


  public:     
    FootContact (bool log_data_files_, float atlas_weight_, float schmitt_level_);

    void terminate();

    footid DetectFootTransition(int64_t utime, float leftz, float rightz);

    footid getStandingFoot();
    // Used internally to change the active foot for motion estimation
    void setStandingFoot(footid foot);

    // Return which foot is which - these are based on the RIGHTFOOT and LEFTFOOT defines in TwoLegsEstiamte_types.h
    footid getSecondaryFoot();

    float leftContactStatus();
    float rightContactStatus();

    void updateSingleFootContactStates(long utime, const double left_force, const double right_force);
};

}

#endif /*FOOTCONTACT_H_*/
