#ifndef OFFSET_RELATION_H
#define OFFSET_RELATION_H

#include "boost/shared_ptr.hpp"

namespace action_authoring
{

/**todo: add comment*/
class OffsetRelation::RelationState
{
    //------------fields
 private:

    //------------Constructor--------
 public:
    OffsetRelation();

  //---------------Accessors

  //mutators
  
}; //class OffsetRelation
 
 typedef boost::shared_ptr<OffsetRelation> OffsetRelationPtr;
 typedef boost::shared_ptr<const OffsetRelation> OffsetRelationConstPtr;

} //namespace action_authoring


#endif //OFFSET_RELATION_H