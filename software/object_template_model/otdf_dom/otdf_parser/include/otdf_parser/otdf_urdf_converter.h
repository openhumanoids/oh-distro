#ifndef OTDF_URDF_CONVERTER_H
#define OTDF_URDF_CONVERTER_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/function.hpp>
#include <otdf_interface/model.h>


namespace otdf{

void addGeometry(boost::shared_ptr<const Geometry> geom,TiXmlElement* element);
void addVisual(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element);
void addInertial(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element);
void addDummyInertial(double x, double y,double z,double r,double p,double yaw,TiXmlElement* element);
void addCollision(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element);
void addChildEntities(boost::shared_ptr<const BaseEntity> entity,TiXmlElement* robot);
void addParentJoint(boost::shared_ptr<const BaseEntity> entity, boost::shared_ptr<const BaseEntity> child, double &r, double &p, double &y,TiXmlElement* robot);
void addChildJoints(boost::shared_ptr<const BaseEntity> entity,TiXmlElement* robot);

std::string convertObjectInstanceToURDFstring(boost::shared_ptr<ModelInterface> object);
void convertObjectInstanceToURDFfile(boost::shared_ptr<ModelInterface> object);
}

#endif
