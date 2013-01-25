/*
 * AffordanceState.h
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCE_STATE_H
#define AFFORDANCE_STATE_H

#include "affordance/ModelState.h"
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/unordered_map.hpp>

namespace affordance
{

  typedef std::pair<const int32_t, const int32_t> GlobalUID;
  
  /**Mutable class representing the state of an affordance*/
  class AffordanceState : public ModelState<AffordanceState>
  {

    //------enums and typedefs
  public:
    enum OTDF_TYPE  {CYLINDER 	= drc::affordance_t::CYLINDER,
		     LEVER 	= drc::affordance_t::LEVER,
		     SPHERE	= drc::affordance_t::SPHERE,
		     BOX 	= drc::affordance_t::BOX,
		     UNKNOWN};
    
    /**standardizing the naming for common fields in drc::affordance_t. 
       These should be used as keys in the _params map*/
    static std::string  X_NAME, Y_NAME, Z_NAME, ROLL_NAME, PITCH_NAME, YAW_NAME,
      RADIUS_NAME, LENGTH_NAME, WIDTH_NAME, HEIGHT_NAME,  R_COLOR_NAME, G_COLOR_NAME, B_COLOR_NAME;
    
    //------------fields

  private:
    static const boost::unordered_map<int16_t, OTDF_TYPE> idToEnum;
    static boost::unordered_map<int16_t, OTDF_TYPE> initIdEnumMap(); //to initialiaze idToEnum

    
  public: //should make get / private set methods for these
    //mimicking lcm
    int64_t    _map_utime;
    int32_t    _map_id;
    
    /**which object in the scene?*/
    int32_t    _object_id;
    
    /**type of object*/
    OTDF_TYPE  _otdf_id;
    
    /**informal name for the affordance*/
    std::string _name;
    
    /**{name --> value} maps*/
    boost::unordered_map<std::string, double> _params, //geometrical properties
                                             _states;
    std::vector< int32_t > _ptinds;
    
    //-----------constructor/destructor
  public:
    AffordanceState(const drc::affordance_t *affordanceMsg);
    AffordanceState(const std::string &name,
		    const int &objId = 0, const int &mapId = 0,
		    const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)),
		    const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));
    AffordanceState(const AffordanceState &other);
    AffordanceState& operator=( const AffordanceState& rhs );
    
    void initHelper(const drc::affordance_t *msg);
    virtual ~AffordanceState();
    
    //observers
  public:
    void toMsg(drc::affordance_t *affordanceMsg) const;
    
    
    //ModelState interface 
    virtual GlobalUID getGlobalUniqueId() const;
    virtual std::string getName() const;

    virtual Eigen::Vector3f getColor() const;
    virtual Eigen::Vector3f getXYZ() const;    
    virtual Eigen::Vector3f getRPY() const;    

    virtual bool isAffordance() const;
    virtual bool isManipulator() const;
    virtual bool hasChildren() const; //any
    virtual bool hasParent() const; //1 or more
    virtual void getChildren(std::vector<boost::shared_ptr<const AffordanceState> > &children) const;
    virtual void getParents(std::vector<boost::shared_ptr<const AffordanceState> > &children) const;
    virtual void getCopy(AffordanceState &copy) const;

    //--these methods throw exceptions if we don't
    //have these fields defined
    double radius() const;
    double length() const;
    double width() const;
    double height() const;

    //useful
    bool hasRPY() const;
    
    //helpers
  private:
    static void assertContainsKey(const boost::unordered_map<std::string, double> &map,
				  const std::string &key);
  public:
    static std::string toStrFromMap(boost::unordered_map<std::string,double> m);
    template <class T> static std::string toStr(T t);
    
  };
  
  std::ostream& operator<<( std::ostream& out, const AffordanceState& other );
  
  typedef boost::shared_ptr<AffordanceState> AffPtr;
  typedef boost::shared_ptr<const AffordanceState> AffConstPtr;  

} //namespace affordance

#endif /* AFFORDANCE_STATE_H */
