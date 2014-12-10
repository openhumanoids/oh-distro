#ifndef OFFSET_RELATION_H
#define OFFSET_RELATION_H

#include <boost/shared_ptr.hpp>
#include <action_authoring/RelationState.h>
#include <kdl/frames.hpp>

namespace action_authoring
{

/**todo: add comment*/
class OffsetRelation : public RelationState
{
    //------------fields
private:
    KDL::Frame _frame;

    //------------Constructor--------
public:
    OffsetRelation();

    //----------Mutators
    void setFrame(KDL::Frame f) { _frame = f; }

    //---------------Accessors
    KDL::Frame getFrame() { return _frame; }
    virtual std::string getState() const; // returns a user-friendly string that explains the state of relation
    virtual std::string getPrompt() const; // prompts the user for the next field to set to complete the relation

    //mutators

}; //class OffsetRelation

typedef boost::shared_ptr<OffsetRelation> OffsetRelationPtr;
typedef boost::shared_ptr<const OffsetRelation> OffsetRelationConstPtr;

} //namespace action_authoring


#endif //OFFSET_RELATION_H
