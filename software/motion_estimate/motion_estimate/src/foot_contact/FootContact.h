#ifndef FOOTCONTACT_H_
#define FOOTCONTACT_H_

#include "Footsteps.h"
#include "estimate_tools/SignalTap.hpp"

namespace TwoLegs {

class FootContact {
  private:

    // Parameters:
    double low_foot_contact_thresh_;
    double high_foot_contact_thresh_;
    long foot_contact_delay_;

    double loadsplit_level_;
    double min_standing_force_;

    int64_t standing_transition_timeout_;

    int64_t schmitt_level_;
    int64_t transition_timeout_;

    /////////////////////////////////////////////////
     
    int standing_foot;
    float expectedweight;
    int64_t transition_timespan;
    bool foottransitionintermediateflag;
    int64_t standing_timer;
    int64_t standing_delay;
    //bool both_feet_in_contact;
    bool standingintermediate;

    DataFileLogger datafile;
    DataFileLogger footcontactfile;

    SchmittTrigger* _left_contact_state;
    SchmittTrigger* _right_contact_state;

    footforces leftforces;
    footforces rightforces;

    int64_t lcmutime;
    int64_t deltautime;

    float getPrimaryFootZforce();
    float getSecondaryFootZforce();


  public:     
    FootContact (bool _log_data_files, const float atlasWeight);

    void terminate();

    footstep DetectFootTransistion(int64_t utime, float leftz, float rightz);

    int getStandingFoot();
    // Used internally to change the active foot for motion estimation
    void setStandingFoot(int foot);

    // Return which foot is which - these are based on the RIGHTFOOT and LEFTFOOT defines in TwoLegsEstiamte_types.h
    int primary_foot();
    int secondary_foot();

    float leftContactStatus();
    float rightContactStatus();

    void updateSingleFootContactStates(long utime, const double left_force, const double right_force);
};

}

#endif /*FOOTCONTACT_H_*/
