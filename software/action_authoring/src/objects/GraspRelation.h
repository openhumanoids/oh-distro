#ifndef GRASP_RELATION_H
#define GRASP_RELATION_H

#include "boost/shared_ptr.hpp"

namespace action_authoring
{

/**todo: add comment*/
class GraspRelation : RelationState
{
    //------------fields
 private:

    //------------Constructor--------
 public:
    GraspRelation();

  //---------------Accessors
    virtual std::string getState() const; // returns a user-friendly string that explains the state of relation
    virtual std::string getPrompt() const; // prompts the user for the next field to set to complete the relation

  //mutators
  
}; //class GraspRelation
 
 typedef boost::shared_ptr<GraspRelation> GraspRelationPtr;
 typedef boost::shared_ptr<const GraspRelation> GraspRelationConstPtr;

} //namespace action_authoring

#endif //GRASP_RELATION_H
