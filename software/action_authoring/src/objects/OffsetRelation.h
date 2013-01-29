#ifndef OFFSET_RELATION_H
#define OFFSET_RELATION_H

#include "boost/shared_ptr.hpp"

namespace action_authoring
{

/**todo: add comment*/
class OffsetRelation : RelationState
{
    //------------fields
 private:

    //------------Constructor--------
 public:
    OffsetRelation();

  //---------------Accessors
    virtual std::string getState() const; // returns a user-friendly string that explains the state of relation
    virtual std::string getPrompt() const; // prompts the user for the next field to set to complete the relation

  //mutators
  
}; //class OffsetRelation
 
 typedef boost::shared_ptr<OffsetRelation> OffsetRelationPtr;
 typedef boost::shared_ptr<const OffsetRelation> OffsetRelationConstPtr;

} //namespace action_authoring


#endif //OFFSET_RELATION_H
