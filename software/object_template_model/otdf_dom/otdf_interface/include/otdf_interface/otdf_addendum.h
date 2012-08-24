
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
  
  //link_pattern and joint_pattern classes. 
  class Joint_pattern
  {
  public:

    Joint_pattern() { this->clear(); };

    std::string name;
    int noofrepetitions;
    Pose origin; 
    /// transform from Parent Link frame to Joint frame
    //Pose  parent_to_joint_origin_transform;
    Pose pattern_offset;
    boost::shared_ptr<Joint> joint_template;
    std::vector<boost::shared_ptr<Joint> > joint_set;
    
    std::vector<bool> expression_flags;
    std::vector<exprtk::expression<double> >  local_expressions;

    enum
    {
      UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    } type;

    /// \brief     type_       meaning of axis_
    /// ------------------------------------------------------
    ///            UNKNOWN     unknown type
    ///            REVOLUTE    rotation axis
    ///            PRISMATIC   translation axis
    ///            FLOATING    N/A
    ///            PLANAR      plane normal axis
    ///            FIXED       N/A
    
    /// child Link element
    ///   child link frame is the same as the Joint frame
    std::string child_link_pattern_name;
    std::string child_link_pattern_type;
    /// parent Link element
    ///   origin specifies the transform from Parent Link to Joint Frame
    std::string parent_link_name;
    std::string parent_type;
    
    bool initXml(TiXmlElement* xml, ParamTable_t &symbol_table);
    void clear()
    {
      this->joint_template.reset();
      this->joint_set.clear();
      
      this->origin.clear();
      this->pattern_offset.clear();

      this->child_link_pattern_name.clear();
      this->parent_link_name.clear();
    
      this->type = UNKNOWN;
      
      exprtk::expression<double> expression; 
      exprtk::parser<double> parser;
      parser.compile("1",expression);
      local_expressions.push_back(expression);// initialise local_expressions
      expression_flags.push_back(false); //for no of repetetions
      noofrepetitions =1;      
    };
    
    void update()
    {
      this->origin.update();
      this->pattern_offset.update();
  
      joint_template->update();
	if(expression_flags[0]){
		this->noofrepetitions = this->local_expressions[0].value();
	}   
	//Update all joints in joint_set
	if (this->noofrepetitions < joint_set.size()) {
	  for (unsigned int i=noofrepetitions; i < joint_set.size(); i++)
	  {
	      joint_set.pop_back();
	  }    
	}
	else if(this->noofrepetitions > joint_set.size()) {
	  for (unsigned int i=noofrepetitions; i < joint_set.size(); i++)
	  {
	      boost::shared_ptr<Joint> temp; 
	      temp.reset(new Joint(*joint_template));
	      std::ostringstream str;   
	      str << name << "_" << i; // append ID to joint name
	      temp->name =str.str();
		if(i==0)
		  temp->parent_to_joint_origin_transform = origin;     
		else{
		  temp->parent_to_joint_origin_transform.position = pattern_offset.position + this->joint_set[i-1]->parent_to_joint_origin_transform.position;
		  temp->parent_to_joint_origin_transform.rotation = pattern_offset.rotation*(this->joint_set[i-1]->parent_to_joint_origin_transform.rotation);
		}  
	      this->joint_set.push_back(temp);
	  }    
	}
      
    };
  };

 class Link_pattern : public BaseEntity
 {
  public:
    Link_pattern() { this->clear(); };
    
    int noofrepetitions;
    
    //send inertial,visual and collision tags of link_template to link_template->initXml
    boost::shared_ptr<Link> link_template; 

    
    std::vector<bool> expression_flags;
    std::vector<exprtk::expression<double> >  local_expressions;

    void clear()
    {
      this->link_template.reset();
      this->link_set.clear();
          
      exprtk::expression<double> expression; 
      exprtk::parser<double> parser;
      parser.compile("1",expression);
      local_expressions.push_back(expression);// initialise local_expressions
      expression_flags.push_back(false); //for no of repetetions
      noofrepetitions =1;      
    };
    
    void update()
    {
     
     link_template->update();
      if(expression_flags[0]){
	      this->noofrepetitions = this->local_expressions[0].value();
      }   
       //Update all links in link_set  
      if (this->noofrepetitions < link_set.size()) {
         for (unsigned int i=noofrepetitions; i < link_set.size(); i++)
         {
            link_set.pop_back();
         }    
       }
       else if(this->noofrepetitions > link_set.size()) {
         for (unsigned int i=noofrepetitions; i < link_set.size(); i++)
         {
             boost::shared_ptr<Link> temp; 
             temp.reset(new Link(*link_template));
             std::ostringstream str;   
             str << name << "_" << i; // append ID to pattern name
             temp->name =str.str();    
             this->link_set.push_back(temp);
         }    
       }
       
    };
    
    std::string getEntityType() const
    {
        std::string str = "Link_pattern";
        return str;
    };

    void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
    
    boost::shared_ptr<BaseEntity> getParent() const {
         return parent_link_.lock();
    };
  
    void setParent(boost::shared_ptr<BaseEntity> parent)
    {
       this->parent_link_ = parent;
    };
    //void setParentJoint(boost::shared_ptr<Joint> child);
    //void addChild(boost::shared_ptr<BaseEntity> child);
    //void addChildJoint(boost::shared_ptr<Joint> child);

    void setParentJointPattern(boost::shared_ptr<Joint_pattern>  parent)
    {
      this->parent_joint_pattern = parent;
    };

//void addChild(boost::shared_ptr<BaseEntity> child)
//{
//  this->child_links.push_back(child);
//};

//void addChildJoint(boost::shared_ptr<Joint> child)
//{
//  this->child_joints.push_back(child);
//};
    
    boost::shared_ptr<std::vector<boost::shared_ptr<Link > > > getLinkSet() const{
          boost::shared_ptr<std::vector<boost::shared_ptr<Link > > > ptr;
	  ptr.reset(new std::vector<boost::shared_ptr<Link > >(this->link_set));
          //ptr = this->link_set;
          return ptr;
    };
    

    boost::shared_ptr<Joint_pattern> parent_joint_pattern;
    std::vector<boost::shared_ptr<Link> > link_set;
    
    //     Parent Joint element
//       explicitly stating "parent" because we want directional-ness for tree structure
//       every link can have one parent
//    boost::shared_ptr<Joint> parent_joint;
    //std::vector<boost::shared_ptr<Joint> > parent_joint_set;
    //std::vector<boost::shared_ptr<Joint> > child_joints;
    //std::vector<boost::shared_ptr<BaseEntity> > child_links;
    
    private:
    boost::weak_ptr<BaseEntity> parent_link_;

  };
  
  
  



}// end namespace

#endif
