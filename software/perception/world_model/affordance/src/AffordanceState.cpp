/*
 * AffordanceState.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#include "AffordanceState.h"
#include "boost/assign.hpp"
#include <iostream>

using namespace affordance;
using namespace Eigen;
using namespace boost;
using namespace std;


//--------set common name fields for drc::affordance_t

string AffordanceState::X_NAME 		= "x";
string AffordanceState::Y_NAME 		= "y";
string AffordanceState::Z_NAME 		= "z";
string AffordanceState::ROLL_NAME 	= "roll";
string AffordanceState::PITCH_NAME 	= "pitch";
string AffordanceState::YAW_NAME 	= "yaw";
string AffordanceState::RADIUS_NAME     = "radius";
string AffordanceState::LENGTH_NAME     = "length";
string AffordanceState::WIDTH_NAME      = "width";
string AffordanceState::HEIGHT_NAME     = "height";
string AffordanceState::R_COLOR_NAME  	= "r_color";
string AffordanceState::G_COLOR_NAME  	= "g_color";
string AffordanceState::B_COLOR_NAME  	= "b_color";

AffordanceState::OTDF_TYPE AffordanceState::CYLINDER  = "cylinder";
AffordanceState::OTDF_TYPE AffordanceState::LEVER  	  = "lever";
AffordanceState::OTDF_TYPE AffordanceState::SPHERE    = "sphere";
AffordanceState::OTDF_TYPE AffordanceState::BOX  	  = "box";
AffordanceState::OTDF_TYPE AffordanceState::UNKNOWN   = "unknown";

const unordered_set<AffordanceState::OTDF_TYPE> AffordanceState::supportedOtdfTypes = getSupportedOtdfTypes();

unordered_set<AffordanceState::OTDF_TYPE> AffordanceState::getSupportedOtdfTypes()
{
  unordered_set<OTDF_TYPE> s;
  vector<string> files;
  otdf::get_filenames_from_otdf_models_dir(files);
  for (uint i = 0; i < files.size(); i++)
    s.insert(files[i]);
  return s;
}

void AffordanceState::printSupportedOtdfTypes()
{
  cout << "\n=====Supported OTDF TYPES:======" << endl;
  for(unordered_set<OTDF_TYPE>::iterator i = supportedOtdfTypes.begin();
      i != supportedOtdfTypes.end();
      ++i)
    {
      cout << *i << endl;
    }
  cout << "===========" << endl;
}

/**Constructs an AffordanceState from an lcm message.*/
AffordanceState::AffordanceState(const drc::affordance_t *msg) 
{
	initHelper(msg);
}

/**copy constructor by using toMsg and then the constructor*/
AffordanceState::AffordanceState(const AffordanceState &other)
{
	drc::affordance_t msg;
	other.toMsg(&msg);
	initHelper(&msg);
}

/**Constructs an affordance and sets the name, objId,mapid, frame, and color as specified
@param otdf_type the OTDF type for this affordance
@param unique object id.  must be unique for the map
@param mapId
@param frame transformation in the map
@param rgb color values from [0,1]
*/
AffordanceState::AffordanceState(const int &uid, const int &mapId,
                                 const KDL::Frame &frame,
                                 const Eigen::Vector3f &color)
  : _map_id(mapId), _uid(uid), _otdf_type(AffordanceState::UNKNOWN)
{
  setFrame(frame);
  setColor(color);
}


AffordanceState& AffordanceState::operator=( const AffordanceState& rhs )
{
  if (this == &rhs) // protect against invalid self-assignment
    return *this;
  
  //convert to message and then call fromMsg
  drc::affordance_t msg;
  rhs.toMsg(&msg);
  fromMsg(&msg);
 
  return *this;
}

//======================MUTATORS
/**sets the state of this to that of msg*/
void AffordanceState::fromMsg(const drc::affordance_t *msg)
{
  clear();  //clear any object state
  initHelper(msg);
}

void AffordanceState::setFrame(const KDL::Frame &frame)
{
  //---set xyz roll pitch yaw from the frame
  _params[X_NAME] = frame.p[0];
  _params[Y_NAME] = frame.p[1];
  _params[Z_NAME] = frame.p[2];
  
  double roll,pitch,yaw;
  frame.M.GetRPY(roll,pitch,yaw);
  _params[ROLL_NAME] 	= roll;
  _params[PITCH_NAME] = pitch;
  _params[YAW_NAME] 	= yaw;
}

void AffordanceState::setColor(const Eigen::Vector3f &color)
{
  _params[R_COLOR_NAME] = color[0];
  _params[G_COLOR_NAME] = color[1];
  _params[B_COLOR_NAME] = color[2];  
}


void AffordanceState::clear()
{
  _utime = 0;
  _map_id = 0;
  _uid = 0;
  
  // mfallon: Should bounding box elements be set to something here?

  _params.clear();  
  _states.clear();
  _points.clear();
  _triangles.clear();
  
  _otdf_type = AffordanceState::UNKNOWN;
}

void AffordanceState::setToBox(const double length, const double width,
			       const double height,
			       const int &uid, const int &mapId,
			       const KDL::Frame &frame,
			       const Eigen::Vector3f &color)
{
  clear();
  
  _params[LENGTH_NAME] = length;
  _params[WIDTH_NAME] = width;
  _params[HEIGHT_NAME] = height;
  _otdf_type = AffordanceState::BOX;

  //set rest of the fields
  _uid = uid;
  _map_id = mapId;
  setFrame(frame);
  setColor(color);;    
}



void AffordanceState::setToSphere(const double radius,
				  const int &uid, const int &mapId,
				  const KDL::Frame &frame,
				  const Eigen::Vector3f &color)
{
  clear();

  _params[RADIUS_NAME] = radius;
  _otdf_type = AffordanceState::SPHERE;
  //set rest of the fields
  _uid = uid;
  _map_id = mapId;
  setFrame(frame);
  setColor(color);;    
}


void AffordanceState::setToCylinder(const double length, const double radius,
                                    const int &uid, const int &mapId,
                                    const KDL::Frame &frame,
                                    const Eigen::Vector3f &color)
{
  clear();

  _params[LENGTH_NAME] = length;
  _params[RADIUS_NAME] = radius;
  _otdf_type = AffordanceState::CYLINDER;
  //set rest of the fields
  _uid = uid;
  _map_id = mapId;
  setFrame(frame);
  setColor(color);;    
}


void AffordanceState::setType(const AffordanceState::OTDF_TYPE &type)
{
  //do checks to make sure the relevant fields are defined
  _otdf_type = type;

  if (type == AffordanceState::CYLINDER && 
      !(hasRadius() && hasLength()))
    throw ArgumentException("State for cylinder not defined");
  if (type == AffordanceState::SPHERE &&
      !hasRadius())
    throw ArgumentException("State for sphere not defined");
  if (type == AffordanceState::BOX &&
      !(hasWidth() && hasHeight() && hasWidth()))
    throw ArgumentException("State for box not defined");
}


/**used by constructor and copy constructor*/
void AffordanceState::initHelper(const drc::affordance_t *msg)
{
  if ((_states.size() != 0 || _params.size() != 0 ) || (_points.size() != 0 || _triangles.size() != 0 ) )
    throw ArgumentException("shouldn't call init if these fields aren't empty");
  
  _utime 	= msg->utime;
  _map_id 	= msg->map_id;
  _uid 	= msg->uid;

  memcpy(_bounding_pos, msg->bounding_pos, 3*sizeof(float));
  memcpy(_bounding_rpy, msg->bounding_rpy, 3*sizeof(float));
  memcpy(_bounding_lwh, msg->bounding_lwh, 3*sizeof(float));

  _otdf_type 		= msg->otdf_type;
  _points 	= msg->points;
  _triangles       = msg->triangles;
    
  //argument check
  if (supportedOtdfTypes.find(msg->otdf_type) == supportedOtdfTypes.end())
    {
      printSupportedOtdfTypes();
      throw InvalidOtdfID(string("not recognized: ") 
			  + msg->otdf_type 
			  + string("  : otdf_type =  ") + msg->otdf_type );
      
    }
  
  _otdf_type = msg->otdf_type;
  
  for(int i = 0; i < msg->nstates; i++)
    _states[msg->state_names[i]] = msg->states[i];

  for (int i = 0; i < msg->nparams; i++)
    _params[msg->param_names[i]] = msg->params[i];
}

AffordanceState::~AffordanceState()
{
}


//------methods-------
/**convert this to a drc_affordacne_t lcm message*/
// @comment: mfallon: wouldn't assignment be quicker here than push_back
void AffordanceState::toMsg(drc::affordance_t *msg) const
{
	msg->utime 	= _utime;
	msg->map_id		= _map_id;
	msg->uid 	= _uid;
	msg->otdf_type		= _otdf_type;

	memcpy(msg->bounding_pos, _bounding_pos, 3*sizeof(float));
	memcpy(msg->bounding_rpy, _bounding_rpy, 3*sizeof(float));
	memcpy(msg->bounding_lwh, _bounding_lwh, 3*sizeof(float));

	unordered_map<string,double>::const_iterator iter;
	//params
	msg->nparams = _params.size();
	for(iter = _params.begin(); iter != _params.end(); ++iter)
	{
		msg->param_names.push_back(iter->first);
		msg->params.push_back(iter->second);
	}

	//states
	msg->nstates = _states.size();
	for(iter = _states.begin(); iter != _states.end(); ++iter)
	{
		msg->state_names.push_back(iter->first);
		msg->states.push_back(iter->second);
	}

	//points
	msg->npoints = _points.size();
	for(uint i = 0; i < _points.size(); i++)
		msg->points.push_back(_points[i]);
        //triangles
        msg->ntriangles = _triangles.size();
        for(uint i = 0; i < _triangles.size(); i++)
                msg->triangles.push_back(_triangles[i]);
}

/**convert from this AffordanceState into a urdf xml string representation*/
void AffordanceState::toURDF(string &urdf_xml_string) const
{
  	drc::affordance_t msg;
	toMsg(&msg);
	otdf::AffordanceLcmMsgToUrdfString(msg, urdf_xml_string);
}

/**@return x,y,z or throws an exception if any of those are not present*/
Vector3f AffordanceState::getXYZ() const
{
	//defensive checks
	assertContainsKey(_params, X_NAME);
	assertContainsKey(_params, Y_NAME);
	assertContainsKey(_params, Z_NAME);

	//using find method b/c operator[] isn't a const method
	return Vector3f(_params.find(X_NAME)->second,
					_params.find(Y_NAME)->second,
					_params.find(Z_NAME)->second);
}


/**@return true if we have roll/pitch/yaw parameters.  false otherwise*/
bool AffordanceState::hasRPY() const
{
	return (_params.find(ROLL_NAME) != _params.end() &&
			_params.find(PITCH_NAME) != _params.end() &&
			_params.find(YAW_NAME) != _params.end());
}


/**@return true if we radius defined.  false otherwise*/
bool AffordanceState::hasRadius() const
{
  return _params.find(RADIUS_NAME) != _params.end();
}

/**@return true if we length defined.  false otherwise*/
bool AffordanceState::hasLength() const
{
  return _params.find(LENGTH_NAME) != _params.end();
}

/**@return true if we width defined.  false otherwise*/
bool AffordanceState::hasWidth() const
{
  return _params.find(WIDTH_NAME) != _params.end();
}

/**@return true if we height defined.  false otherwise*/
bool AffordanceState::hasHeight() const
{
  return _params.find(HEIGHT_NAME) != _params.end();
}


/**@return roll,pitch,yaw or 0,0,0 none of those are not present*/
Vector3f AffordanceState::getRPY() const
{
  //using find method b/c operator[] isn't a const method
  return hasRPY()
    ? Vector3f(_params.find(ROLL_NAME)->second,
	       _params.find(PITCH_NAME)->second,
	       _params.find(YAW_NAME)->second)
    : Vector3f(0,0,0);
}


/**@return radius or throws exception if not present*/
double AffordanceState::radius() const
{
	assertContainsKey(_params, RADIUS_NAME);
	return _params.find(RADIUS_NAME)->second;
}

/**@return length or throws exception if not present*/
double AffordanceState::length() const
{
	assertContainsKey(_params, LENGTH_NAME);
	return _params.find(LENGTH_NAME)->second;
}

/**@return width or throws exception if not present*/
double AffordanceState::width() const
{
	assertContainsKey(_params, WIDTH_NAME);
	return _params.find(WIDTH_NAME)->second;
}


/**@return height or throws exception if not present*/
double AffordanceState::height() const
{
	assertContainsKey(_params, HEIGHT_NAME);
	return _params.find(HEIGHT_NAME)->second;
}


void AffordanceState::assertContainsKey(const unordered_map<string, double> &map,
					   	   	   	   	   	const string &key)
{
	if (map.find(key) == map.end())
		throw KeyNotFoundException("Key = " + key + " not found");
}

string AffordanceState::toStrFromMap(unordered_map<string,double> m)
{
	stringstream s;
	for(unordered_map<string,double>::const_iterator it = m.begin();
	    it != m.end(); ++it)
	{
		s << "\t(" << it->first << ", "
 		  << it->second << ")\n";
	}
	return s.str();
}

AffordanceState::OTDF_TYPE AffordanceState::getType() const
{
  return _otdf_type;
}


namespace affordance
{
/**operator << */
  ostream& operator<<(ostream& out, const AffordanceState& other )
  {
    out << "=====Affordance " << other.getType() << "========" << endl;
    out << "(mapId, uid, otdfType) = (" << other._map_id << ", "
        << other._uid << ", " << other.getType() << ")\n";
    out << "------params: \n" << AffordanceState::toStrFromMap(other._params) << endl;;
    out << "------states: \n" << AffordanceState::toStrFromMap(other._states) << endl;
    return out;
  }


} //namespace affordance

//=============================================
//=============================================
//=============================================
//=============================================
//================Model State API==============
//=============================================
//=============================================
//=============================================
GlobalUID AffordanceState::getGlobalUniqueId() const
{
  return GlobalUID(_map_id, _uid);
}

std::string AffordanceState:: getName() const
{
    return _otdf_type + ToString::toStr(_uid);
}

AffordanceState::OTDF_TYPE AffordanceState:: getOTDFType() const
{
  return _otdf_type;
}

Vector3f AffordanceState::getColor() const
{
  return Vector3f(_params.find(R_COLOR_NAME)->second,
		  _params.find(G_COLOR_NAME)->second,
		  _params.find(B_COLOR_NAME)->second);
}
  

bool AffordanceState::isAffordance() const 
{  return true; }

bool AffordanceState::isManipulator() const 
{  return false; }

bool AffordanceState::hasChildren() const 
{
  throw NotImplementedException("aff state");
  return true; 
}

bool AffordanceState::hasParent() const 
{
  throw NotImplementedException("aff state");
  return true; 
}

void AffordanceState::getChildren(vector<ModelStateConstPtr> &children) const 
{
  throw NotImplementedException("aff state");
}

void AffordanceState::getParents(vector<ModelStateConstPtr> &parents) const 
{
  throw NotImplementedException("aff state");
}

void AffordanceState::getCopy(ModelState &copy) const 
{
  throw NotImplementedException("aff state");
}

