#ifndef POINT_CONTACT_RELATION_H
#define POINT_CONTACT_RELATION_H

#include "boost/shared_ptr.hpp"

namespace action_authoring
{

/**todo: add comment*/
class PointContactRelation::RelationState
{
    //------------fields
 private:

    //------------Constructor--------
 public:
    PointContactRelation();

  //---------------Accessors

  //mutators
  
}; //class PointContactRelation
 
 typedef boost::shared_ptr<PointContactRelation> PointContactRelationPtr;
 typedef boost::shared_ptr<const PointContactRelation> PointContactRelationConstPtr;

} //namespace action_authoring

#endif //POINT_CONTACT_RELATION_H