#ifndef POINT_CONTACT_RELATION_H
#define POINT_CONTACT_RELATION_H

#include <boost/shared_ptr.hpp>
#include <action_authoring/RelationState.h>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <Eigen/Core>

#include <iostream>

namespace action_authoring
{

/**todo: add comment*/
class PointContactRelation : public RelationState
{
public:
    typedef enum
    {
      EQUAL = drc::contact_goal_t::REL_EQUAL,
      LT    = drc::contact_goal_t::REL_LESS_THAN,
      GT    = drc::contact_goal_t::REL_GREATER_THAN,
      UNDEFINED = drc::contact_goal_t::UNDEFINED
    } InequalityType;

    static std::string EQUAL_STR, 
      LESS_THAN_STR, 
      GREATER_THAN_STR,
      UNDEFINED_STR;

    //------------fields
private:
    Eigen::Vector3f _point1;
    Eigen::Vector3f _point2;
    InequalityType _x_relation;
    InequalityType _y_relation;
    InequalityType _z_relation;
    Eigen::Vector3f _offset;

    //------------Constructor--------
public:
    PointContactRelation(Eigen::Vector3f p1 = Eigen::Vector3f(0.0, 0.0, 0.0), 
                         Eigen::Vector3f p2 = Eigen::Vector3f(0.0, 0.0, 0.0));

    //---helper static methods
    static InequalityType strToType(const std::string &s);
    static std::string typeToStr(const InequalityType &type);

    //---------------Accessors
    Eigen::Vector3f getPoint1() const
    {
        return _point1;
    }
    Eigen::Vector3f getPoint2() const
    {
        return _point2;
    }

    double getXOffset() const { return _offset.x(); }
    double getYOffset() const { return _offset.y(); }
    double getZOffset() const { return _offset.z(); }
    Eigen::Vector3f getOffset() const { return _offset; }
    InequalityType getXInequality() const { return _x_relation; }
    InequalityType getYInequality() const { return _y_relation; }
    InequalityType getZInequality() const { return _z_relation; }

    //mutators
    void setPoint1(const Eigen::Vector3f &p1)
    {
        _point1 = p1;
    }
    void setPoint2(const Eigen::Vector3f &p2)
    {
        _point2 = p2;
    }
    void setOffset(const Eigen::Vector3f offset)
    {           
      _offset = offset;
    }
    void setXInequality(const InequalityType &t) { _x_relation = t; }
    void setYInequality(const InequalityType &t) { _y_relation = t; }
    void setZInequality(const InequalityType &t) { _z_relation = t; }

    virtual std::string getPrompt() const;
    virtual std::string getState() const;

}; //class PointContactRelation

typedef boost::shared_ptr<PointContactRelation> PointContactRelationPtr;
typedef boost::shared_ptr<const PointContactRelation> PointContactRelationConstPtr;

} //namespace action_authoring

#endif //POINT_CONTACT_RELATION_H
