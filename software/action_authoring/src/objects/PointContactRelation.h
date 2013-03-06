#ifndef POINT_CONTACT_RELATION_H
#define POINT_CONTACT_RELATION_H

#include <boost/shared_ptr.hpp>
#include <action_authoring/RelationState.h>
#include <Eigen/Core>

namespace action_authoring
{

/**todo: add comment*/
class PointContactRelation : public RelationState
{
    typedef enum
    {
        EQUAL = 0,
        LT = 1,
        GT = 2
    } InequalityType;

    //------------fields
private:
    Eigen::Vector3f _point1;
    Eigen::Vector3f _point2;
    InequalityType _x_relation;
    InequalityType _y_relation;
    InequalityType _z_relation;
    double _tolerance; 

    //------------Constructor--------
public:
    PointContactRelation();
    PointContactRelation(Eigen::Vector3f p1, Eigen::Vector3f p2);

    //---------------Accessors
    Eigen::Vector3f getPoint1() const
    {
        return _point1;
    }
    Eigen::Vector3f getPoint2() const
    {
        return _point2;
    }

    double getTolerance() { return _tolerance; }
    InequalityType getXInequality() { return _x_relation; }
    InequalityType getYInequality() { return _y_relation; }
    InequalityType getZInequality() { return _z_relation; }

    //mutators
    void setPoint1(Eigen::Vector3f p1)
    {
        _point1 = p1;
    }
    void setPoint2(Eigen::Vector3f p2)
    {
        _point2 = p2;
    }
    void setTolerance(double tol) { _tolerance = tol; }
    void setXInequality(InequalityType t) { _x_relation = t; }
    void setYInequality(InequalityType t) { _y_relation = t; }
    void setZInequality(InequalityType t) { _z_relation = t; }

    virtual std::string getPrompt() const;
    virtual std::string getState() const;

}; //class PointContactRelation

typedef boost::shared_ptr<PointContactRelation> PointContactRelationPtr;
typedef boost::shared_ptr<const PointContactRelation> PointContactRelationConstPtr;

} //namespace action_authoring

#endif //POINT_CONTACT_RELATION_H
