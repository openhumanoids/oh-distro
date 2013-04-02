#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    

    void doDemo();
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    drc::affordance_plus_t getAffordancePlus(std::vector<double> &xyzrpy, int uid);
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
  cout << "Finished setting up\n";
}

drc::affordance_plus_t Pass::getAffordancePlus(std::vector<double> &xyzrpy, int uid){ 
  drc::affordance_plus_t p;
  
  drc::affordance_t a;
  a.utime =0;
  a.map_id =0;
  a.uid =uid;
  a.otdf_type ="cylinder";
  a.aff_store_control = drc::affordance_t::NEW;
  a.nstates =0;

  a.param_names.push_back("x");
  a.params.push_back(xyzrpy[0]);
  a.param_names.push_back("y");
  a.params.push_back(xyzrpy[1]);
  a.param_names.push_back("z");
  a.params.push_back(xyzrpy[2]);

  a.param_names.push_back("roll");
  a.params.push_back( xyzrpy[3]);
  a.param_names.push_back("pitch");
  a.params.push_back( xyzrpy[4]);
  a.param_names.push_back("yaw");
  a.params.push_back( xyzrpy[5] );

  a.param_names.push_back("radius");
  a.params.push_back(0.020000);
  a.param_names.push_back("length");
  a.params.push_back(0.15);
  a.param_names.push_back("mass");
  a.params.push_back(1.0); // unknown
  a.nparams =a.params.size();
  a.nstates =0;

  
  p.aff = a;
  
  std::vector< std::vector< float > > points;
  std::vector< float > pt0 = { 0.1, 0.1 ,  0.2};
  std::vector< float > pt1 = { 0.1, 0.1 , -0.2};
  std::vector< float > pt2 = { 0.1, -0.1 , -0.2};
  points.push_back(pt0); points.push_back(pt1); points.push_back(pt2);
  p.npoints=points.size();
  p.points =points;

  std::vector< int > triangle = {0,1,2};    
  p.triangles.push_back( triangle );
  p.ntriangles =p.triangles.size();
    
  return p;
}

void Pass::doDemo(){
  std::vector<double> xyzrpy = {1. , 2. , 3., 0. , 1.571 , 3.142};
  drc::affordance_plus_t a0 = getAffordancePlus(xyzrpy, 0);
  std::vector<double> xyzrpy1 = {2. , 2. , 3., 0.57 , 1.571 , 3.142};
  drc::affordance_plus_t a1 = getAffordancePlus(xyzrpy1, 1);

  drc::affordance_plus_collection_t aff_plus_coll;
  aff_plus_coll.name  = "Map Name";
  aff_plus_coll.utime = 0;
  aff_plus_coll.map_id =0;
  
  aff_plus_coll.affs_plus.push_back( a0);
  aff_plus_coll.affs_plus.push_back( a1);
  aff_plus_coll.naffs =2;
  lcm_->publish("AFFORDANCE_PLUS_COLLECTION",&aff_plus_coll);
  
  drc::affordance_collection_t aff_coll;
  aff_coll.name  = "Map Name";
  aff_coll.utime = 0;
  aff_coll.map_id =0;
  
  aff_coll.affs.push_back( a0.aff);
  aff_coll.affs.push_back( a1.aff);
  aff_coll.naffs =2;
  
  lcm_->publish("AFFORDANCE_COLLECTION",&aff_coll);
}

int main( int argc, char** argv ){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Demo Ready" << endl << "============================" << endl;
  app.doDemo();
  return 0;
}
