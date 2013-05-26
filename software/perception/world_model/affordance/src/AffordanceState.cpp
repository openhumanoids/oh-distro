/*
 * AffordanceState.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#include "AffordanceState.h"
#include <urdf/model.h>
#include <kinematics/kinematics_model_gfe.h>
#include "boost/assign.hpp"
#include <iostream>
#include <visualization_utils/GlKinematicBody.hpp>

using namespace affordance;
using namespace Eigen;
using namespace boost;
using namespace std;
//using namespace urdf;

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
AffordanceState::OTDF_TYPE AffordanceState::CAR  	  = "car";
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


static KDL::Frame poseToKDL(const otdf::Pose &p)
{
  return KDL::Frame(KDL::Rotation::Quaternion(p.rotation.x, 
                                              p.rotation.y, 
                                              p.rotation.z, 
                                              p.rotation.w),
               KDL::Vector(p.position.x, 
                           p.position.y, 
                           p.position.z));
}

static AffPtr geometryToAff(shared_ptr<otdf::Geometry> g, 
                            const KDL::Frame &frame,
                            const string &friendly_name)
{
  Eigen::Vector3f color(1,0,0); //todo
  if(g->type == otdf::Geometry::SPHERE )
    {                  
      shared_ptr< otdf::Sphere > sphere = shared_dynamic_cast< otdf::Sphere >(g);
      AffPtr s (new AffordanceState());
      s->setToSphere(sphere->radius,
                     0,0, //uid, mapid
                     frame,
                     color,
                     friendly_name);             
      return s;
    } 
  if (g->type == otdf::Geometry::BOX )
    {
      shared_ptr< otdf::Box > box = shared_dynamic_cast< otdf::Box >(g);
      AffPtr b (new AffordanceState());
      b->setToBox(box->dim.x, //length
                  box->dim.y, //width
                  box->dim.z, //height
                  0,0, //uid, mapid
                  frame,
                  color,
                  friendly_name);
      return b;
    } 
  if (g->type == otdf::Geometry::CYLINDER )
    {
      shared_ptr< otdf::Cylinder > cylinder = shared_dynamic_cast< otdf::Cylinder >(g);
      AffPtr c (new AffordanceState());
      c->setToCylinder(cylinder->length,
                       cylinder->radius, 
                       0,0,  //uid, mapid
                       frame,
                       color,
                       friendly_name);
      return c;
    }          
  return AffPtr();
}


/*static AffPtr urdfCollToAffBoxCylSphere(const shared_ptr<urdf::Collision> urdfColl,
                                        const string &friendly_name)
{
  return geometryToAff(urdfColl->geometry,
                       poseToKDL(urdfColl->origin),
                       friendly_name);  
}
*/



boost::unordered_map<string,
                     shared_ptr<visualization_utils::GlKinematicBody> >
                     _otdfTypeToBody;



/**split this into primitive types
@return true if succeeds. false otherwise*/
bool AffordanceState::toBoxesCylindersSpheres(vector<boost::shared_ptr<AffordanceState> > &affs)
{
  affs.clear();

  //--get urdf string for this thing
  shared_ptr<visualization_utils::GlKinematicBody> asGLKBody;

  if (_otdfTypeToBody.find(_otdf_type) != _otdfTypeToBody.end())
    {
      asGLKBody = _otdfTypeToBody[_otdf_type];
    }
  else
    {
      string xml_string;
      if(!otdf::get_xml_string_from_file(_otdf_type, xml_string))
        {
          cout << "\n couldn't get xml string: " 
               << _otdf_type << endl;
          return false;
        }
      
      shared_ptr<otdf::ModelInterface> instance = otdf::parseOTDF(xml_string);
      asGLKBody = shared_ptr<visualization_utils::GlKinematicBody>
        (new visualization_utils::GlKinematicBody(instance));

      cout << "\n read otdf xml from disk.  caching GLK body" << endl;
      _otdfTypeToBody[_otdf_type] = asGLKBody;
   }

  //need to set the state w/ joint positions, which we assume are in the states
  unordered_map<string,double>::const_iterator iter;
  std::map<string, double> jointpos;
  for(iter = _states.begin(); iter != _states.end(); ++iter)
    {
      jointpos[iter->first] = iter->second;
    }
  
  asGLKBody->set_state(getOriginFrame(), jointpos);
  
  
  std::map<string, shared_ptr<otdf::Link> > otdfLinksMap = asGLKBody->get_otdf_links_map();
  vector<visualization_utils::LinkFrameStruct> linkFrames = asGLKBody->get_link_tfs();
  for(uint i = 0; i < linkFrames.size();i++)
    {
      //get the next link tf, name, and link
      visualization_utils::LinkFrameStruct nextLinkTf = linkFrames[i];
      string nextLinkName = nextLinkTf.name;
      shared_ptr<otdf::Link> link = otdfLinksMap[nextLinkName];     

      //defensive check
      if (link == shared_ptr<otdf::Link>())
        {
          cout << "\n\n link was null" << endl;
          continue;
        }
      
      //go thru the collision groups
      for(std::map<string, shared_ptr<vector<shared_ptr<otdf::Collision> > > >::iterator iter = link->collision_groups.begin();
          iter != link->collision_groups.end();
          ++iter)
        {
          string nextCollisionGroupName = iter->first;
          shared_ptr<vector<shared_ptr<otdf::Collision> > >nextCollGroup = iter->second;

          //go thru the collision objects
          for(uint n = 0; n < nextCollGroup->size(); n++)
            {
              shared_ptr<otdf::Collision> cObj = nextCollGroup->at(n);

              //-compute the world tf
              KDL::Frame inWorldFrame = getOriginFrame()*nextLinkTf.frame*poseToKDL(cObj->origin);

              //add as affordance
              affs.push_back(geometryToAff(cObj->geometry, inWorldFrame, 
                                           nextLinkName + "/" + nextCollisionGroupName));              
            }
        }
    }
  return true;
}




/**@param urdf_filename urdf to parse
@param affs affordances of boxes, cylinders, spheres that 
we parse from the given urdf
void AffordanceState::getBoxesCylindersSpheres(const string &urdf_filename,
                                               vector<AffPtr> &affs)
{
   //----load urdf from file
  urdf::Model model;
  if(!model.initString(kinematics::Kinematics_Model_GFE::urdf_filename_to_xml_string(urdf_filename)))
    {
      cout << "\ngetBoxesCylindersSpheres: Couldn't load urdf\n" << endl;
    }
 
  //parse
  vector<shared_ptr< Link> > links;
  model.getLinks(links);
  
  //loop
  for( unsigned int i = 0; i < links.size(); i++ )
    {
      for( std::map<string, shared_ptr<vector<shared_ptr<Collision> > > >::iterator it = links[i]->collision_groups.begin(); 
           it != links[i]->collision_groups.end(); it++ )
        {
          for( unsigned int j = 0; j < it->second->size(); j++ )
            {
              if( (*it->second)[ j ] == NULL )
                {
                  cout << endl << links[i]->name << " null " << endl;
                  continue;
                }
              
              AffPtr a = urdfCollToAffBoxCylSphere((*it->second)[j],
                                                   links[ i ]->name);
              if (a != AffPtr())
                affs.push_back(a);
              else
                cout << "\n null element" << endl;
            }
        }
      
      if( links[ i ]->collision == NULL )
        {
          cout << "\n null collision group" << endl;
          continue;
        }
   
      AffPtr a = urdfCollToAffBoxCylSphere(links[ i ]->collision,
                                           links[ i ]->name);
      if (a != AffPtr())
        affs.push_back(a);    
    }
}
*/

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
  _origin_xyz[0] = frame.p[0];
  _origin_xyz[1] = frame.p[1];
  _origin_xyz[2] = frame.p[2];
  
  double roll,pitch,yaw;
  frame.M.GetRPY(roll,pitch,yaw);
  _origin_rpy[0] = roll;
  _origin_rpy[1] = pitch;
  _origin_rpy[2] = yaw;
}

KDL::Frame AffordanceState::getOriginFrame() const
{
  return KDL::Frame(KDL::Rotation::RPY(_origin_rpy[0],
                                       _origin_rpy[1],
                                       _origin_rpy[2]),
                    KDL::Vector(_origin_xyz[0],
                                _origin_xyz[1],
                                _origin_xyz[2]));
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
//  _points.clear();
//  _triangles.clear();
  
  _otdf_type = AffordanceState::UNKNOWN;
  _modelfile.clear();
  _friendly_name.clear();
}

void AffordanceState::setToBox(const double length, const double width,
                               const double height,
                               const int &uid, const int &mapId,
                               const KDL::Frame &frame,
                               const Eigen::Vector3f &color,
                               const std::string &friendly_name)
{
  clear();
  
  _params[LENGTH_NAME] = length;
  _params[WIDTH_NAME] = width;
  _params[HEIGHT_NAME] = height;
  _otdf_type = AffordanceState::BOX;
  _friendly_name = friendly_name;
  _modelfile.clear();

  //set rest of the fields
  _uid = uid;
  _map_id = mapId;
  setFrame(frame);
  setColor(color);;    
}



void AffordanceState::setToSphere(const double radius,
                                  const int &uid, const int &mapId,
                                  const KDL::Frame &frame,
                                  const Eigen::Vector3f &color,
                                  const string &friendly_name)
{
  clear();

  _params[RADIUS_NAME] = radius;
  _otdf_type = AffordanceState::SPHERE;
  _friendly_name = friendly_name;
  _modelfile.clear();

  //set rest of the fields
  _uid = uid;
  _map_id = mapId;
  setFrame(frame);
  setColor(color);;    
}


void AffordanceState::setToCylinder(const double length, 
                                    const double radius,
                                    const int &uid, const int &mapId,
                                    const KDL::Frame &frame,
                                    const Eigen::Vector3f &color,
                                    const string &friendly_name)
{
  clear();

  _params[LENGTH_NAME] = length;
  _params[RADIUS_NAME] = radius;
  _otdf_type = AffordanceState::CYLINDER;
  _friendly_name = friendly_name;
  _modelfile.clear();
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
  _modelfile.clear();
  _friendly_name.clear();

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
  if ((_states.size() != 0 || _params.size() != 0 ))// || (_points.size() != 0 || _triangles.size() != 0 ) )
    throw ArgumentException("shouldn't call init if these fields aren't empty");
  
  _utime 	= msg->utime;
  _map_id 	= msg->map_id;
  _uid 	= msg->uid;

  memcpy(_origin_xyz, msg->origin_xyz, 3*sizeof(double));
  memcpy(_origin_rpy, msg->origin_rpy, 3*sizeof(double));

  memcpy(_bounding_xyz, msg->bounding_xyz, 3*sizeof(double));
  memcpy(_bounding_rpy, msg->bounding_rpy, 3*sizeof(double));
  memcpy(_bounding_lwh, msg->bounding_lwh, 3*sizeof(double));

  //_points 	= msg->points;
  //_triangles       = msg->triangles;
    
  //argument check
  if (supportedOtdfTypes.find(msg->otdf_type) == supportedOtdfTypes.end())
    {
      printSupportedOtdfTypes();
      throw InvalidOtdfID(string("not recognized: ") 
			  + msg->otdf_type 
			  + string("  : otdf_type =  ") + msg->otdf_type );
      
    }
  
  _otdf_type = msg->otdf_type;
  _modelfile = msg->modelfile;
  _friendly_name = msg->friendly_name;

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
    msg->modelfile = _modelfile;
    msg->friendly_name = _friendly_name;

	memcpy(msg->origin_xyz, _origin_xyz, 3*sizeof(double));
	memcpy(msg->origin_rpy, _origin_rpy, 3*sizeof(double));

	memcpy(msg->bounding_xyz, _bounding_xyz, 3*sizeof(double));
	memcpy(msg->bounding_rpy, _bounding_rpy, 3*sizeof(double));
	memcpy(msg->bounding_lwh, _bounding_lwh, 3*sizeof(double));

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

/*
	//points
	msg->npoints = _points.size();
	for(uint i = 0; i < _points.size(); i++)
		msg->points.push_back(_points[i]);
        //triangles
        msg->ntriangles = _triangles.size();
        for(uint i = 0; i < _triangles.size(); i++)
                msg->triangles.push_back(_triangles[i]);
*/
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
        return Vector3f(_origin_xyz[0],_origin_xyz[1],_origin_xyz[2]);
}


/**@return true if we have roll/pitch/yaw parameters.  false otherwise*/
bool AffordanceState::hasRPY() const
{
        return true;
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
  return Vector3f(_origin_rpy[0],_origin_rpy[1],_origin_rpy[2]);
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

string AffordanceState::getFriendlyName() const
{
  return _friendly_name;
}


namespace affordance
{
/**operator << */
  ostream& operator<<(ostream& out, const AffordanceState& other )
  {
    out << "=====Affordance \"" << other.getFriendlyName() << "\" : type = " << other.getType() << "========" << endl;
    out << "time = " << other._utime << endl;
    out << "(mapId, uid, otdfType) = (" << other._map_id << ", "
        << other._uid << ", " << other.getType() << ")\n";
    out << "------params: \n" << AffordanceState::toStrFromMap(other._params) << endl;
    out << "------states: \n" << AffordanceState::toStrFromMap(other._states) << endl;
    out << "-----Frame:\n" << ToString::toStr(other.getOriginFrame()) << endl; 
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
    return _friendly_name + "/" 
      + _otdf_type + ToString::toStr(_uid);
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

