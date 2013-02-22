#ifndef OFFSET_RELATION_H
#define OFFSET_RELATION_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace action_authoring
{

/**todo: add comment*/
class OffsetRelation : RelationState
{
    //------------fields
private:
    Eigen::Vector3f _translation;
    Eigen::Vector4f _rotation;

    //------------Constructor--------
public:
    OffsetRelation();

    Eigen::Vector3f getTranslation()
    {
        return _translation;
    }
    Eigen::Vector4f getRotation()
    {
        return _rotation;
    }

    //----------Mutators
    void setTranslation(Eigen::Vector3f t)
    {
        _translation = t;
    }
    void setRotation(Eigen::Vector4f r)
    {
        _rotation = r;
    }

    //---------------Accessors
    virtual std::string getState() const; // returns a user-friendly string that explains the state of relation
    virtual std::string getPrompt() const; // prompts the user for the next field to set to complete the relation

    //mutators

}; //class OffsetRelation

typedef boost::shared_ptr<OffsetRelation> OffsetRelationPtr;
typedef boost::shared_ptr<const OffsetRelation> OffsetRelationConstPtr;

} //namespace action_authoring


#endif //OFFSET_RELATION_H
