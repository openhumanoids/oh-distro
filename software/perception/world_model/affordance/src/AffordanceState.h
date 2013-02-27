/*
 * AffordanceState.h
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCE_STATE_H
#define AFFORDANCE_STATE_H

#include <affordance/ModelState.h>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <otdf_lcm_utils/otdf_lcm_utils.h>

namespace affordance
{

  typedef std::pair<const int32_t, const int32_t> GlobalUID;
  
  /**Mutable class representing the state of an affordance*/
  class AffordanceState : public ModelState
  {

    //------enums and typedefs
  public:

    typedef std::string OTDF_TYPE; //type is a string / relative filename
    static OTDF_TYPE CYLINDER;  //relative file name for cylinder
    static OTDF_TYPE BOX; //todo: read these in from the right directory
    static OTDF_TYPE SPHERE;
    static OTDF_TYPE LEVER;
    static OTDF_TYPE UNKNOWN;
    
    static const boost::unordered_set<OTDF_TYPE> supportedOtdfTypes;
    static boost::unordered_set<OTDF_TYPE> getSupportedOtdfTypes();

    /**standardizing the naming for common fields in drc::affordance_t. 
       These should be used as keys in the _params map*/
    static std::string  X_NAME, Y_NAME, Z_NAME, ROLL_NAME, PITCH_NAME, YAW_NAME,
      RADIUS_NAME, LENGTH_NAME, WIDTH_NAME, HEIGHT_NAME,  R_COLOR_NAME, G_COLOR_NAME, B_COLOR_NAME;
    
    //------------fields
  public: //should make get / private set methods for these
    //mimicking lcm
    int64_t    _utime;
    int32_t    _map_id;
    
    /**which object in the scene?*/
    int32_t    _uid;
        
    /**{name --> value} maps*/
    boost::unordered_map<std::string, double> _params, //geometrical properties
                                             _states;
    std::vector< int32_t > _ptinds;
    
  private:
    /**type of affordance.  this is a relative otdf file name w/o
     the otdf extension at the end.  e.g. "cylinder"*/
    OTDF_TYPE _otdf_type;

    //-----------constructor/destructor
  public:
    AffordanceState(const drc::affordance_t *affordanceMsg);
    AffordanceState(const int &uid = 0, const int &mapId = 0,
                    const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)),
                    const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));
    AffordanceState(const AffordanceState &other);
    AffordanceState& operator=( const AffordanceState& rhs );
    virtual ~AffordanceState();    

  private:
    void initHelper(const drc::affordance_t *msg);

    //mutators
  public:
    void fromMsg(const drc::affordance_t *msg);

    void setFrame(const KDL::Frame &frame);
    void setColor(const Eigen::Vector3f &color);

	     
    void setToBox(const double length = 0, const double width = 0,
		  const double height = 0,
		  const int &uid = 0, const int &mapId = 0,
		  const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)),
		  const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));
		  
		  

    void setToSphere(const double radius,
		     const int &uid = 0, const int &mapId = 0,
		     const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)),
		     const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));


    void setToCylinder(const double length = 0, const double radius = 0,
		       const int &uid = 0, const int &mapId = 0,
		       const KDL::Frame &frame = KDL::Frame(KDL::Vector(0,0,0)), 
      		       const Eigen::Vector3f &color = Eigen::Vector3f(1,0,0));
    
    void setType(const OTDF_TYPE &type);
    
    void clear();

    //observers
  public:
    void toMsg(drc::affordance_t *affordanceMsg) const;
    void toURDF(std::string &urdf_xml_string) const;
    OTDF_TYPE getType() const;

    //ModelState interface 
    virtual GlobalUID getGlobalUniqueId() const;
    OTDF_TYPE getOTDFType() const;
    virtual std::string getName() const;

    virtual Eigen::Vector3f getColor() const;
    virtual Eigen::Vector3f getXYZ() const;    
    virtual Eigen::Vector3f getRPY() const;    

    virtual bool isAffordance() const;
    virtual bool isManipulator() const;
    virtual bool hasChildren() const; //any
    virtual bool hasParent() const; //1 or more
    virtual void getChildren(std::vector<ModelStateConstPtr> &children) const;
    virtual void getParents(std::vector<ModelStateConstPtr> &parents) const; 
    virtual void getCopy(ModelState &copy) const;

    //--these methods throw exceptions if we don't
    //have these fields defined
    double radius() const;
    double length() const;
    double width() const;
    double height() const;

    //useful
    bool hasRadius() const;    
    bool hasLength() const;
    bool hasWidth() const;
    bool hasHeight() const;
    bool hasRPY() const;

    //helpers
  private:
    static void assertContainsKey(const boost::unordered_map<std::string, double> &map,
				  const std::string &key);
  public:
    static std::string toStrFromMap(boost::unordered_map<std::string,double> m);
    static void printSupportedOtdfTypes();
    
  };
  
  std::ostream& operator<<( std::ostream& out, const AffordanceState& other );
  
  typedef boost::shared_ptr<AffordanceState> AffPtr;
  typedef boost::shared_ptr<const AffordanceState> AffConstPtr;  

} //namespace affordance

#endif /* AFFORDANCE_STATE_H */
