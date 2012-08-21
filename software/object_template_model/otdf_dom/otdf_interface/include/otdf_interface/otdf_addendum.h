
#ifndef OTDF_INTERFACE_OTDF_ADDENDUM_H
#define OTDF_INTERFACE_OTDF_ADDENDUM_H

#include <string>
#include <vector>
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "expression_parsing.h"
#include "joint.h"
#include "link.h"
#include "color.h"

namespace otdf
{

  class Param
  {
  public:
    Param() { this->clear(); };
    Param(std::string &_name,double default_value ):name(_name), value(default_value) {};
    std::string name;
    double value;

    void clear()
    {
      value = 0;
    };

    bool initXml(TiXmlElement *config)
    {
      const char *name = config->Attribute("name");
      if (!name)
      {
	std::cerr << "ERROR: Param must contain a name attribute" << std::endl;
	return false;
      }
      this->name = name;
		
      double value;
      const char* default_value_str = config->Attribute("default_value");
      if (default_value_str == NULL){
	std::cout << "param: no default_value for param, defaults to 0"<< std::endl;
	value = 0;
      }
      else
      {
	try
	{
	value = boost::lexical_cast<double>(default_value_str);
	}
	catch (boost::bad_lexical_cast &e)
	{
	  std::cerr << "ERROR: param default value"<< default_value_str <<"is not a double"<< std::endl;
	  return false;
	}
      }
      this->value = value;  
      return true;
    };

  }; // end class

  class Bounding_volume : public BaseEntity
  {
  public:
    Bounding_volume() { this->clear(); };
    Pose origin;
    boost::shared_ptr<Geometry> geometry;

    void clear()
    {
      this->origin.clear();
      this->geometry.reset();
    };
    void update()
    {
      this->origin.update();
      this->geometry->update();
    };
    std::string getEntityType() const{
    std::string str = "Bounding_volume";
    return str;};

    void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
    
    boost::shared_ptr<BaseEntity> getParent() const
  {return parent_link_.lock();};
  
    void setParent(boost::shared_ptr<BaseEntity> parent);
    void setParentJoint(boost::shared_ptr<Joint> child);
    void addChild(boost::shared_ptr<BaseEntity> child);
    void addChildJoint(boost::shared_ptr<Joint> child);
    
    
//     Parent Joint element
//       explicitly stating "parent" because we want directional-ness for tree structure
//       every link can have one parent
    boost::shared_ptr<Joint> parent_joint;
  // 
    std::vector<boost::shared_ptr<Joint> > child_joints;
    std::vector<boost::shared_ptr<BaseEntity> > child_links;
    
    private:
    boost::weak_ptr<BaseEntity> parent_link_;

  };
  
  //TODO: link_pattern and joint_pattern classes. How do you convert them to a set of links and joints?
  // link_pattern itself contains a vector of links.



}// end namespace

#endif
