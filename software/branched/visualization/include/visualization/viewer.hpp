#ifndef _VS_VIS_VIEWER_HPP
#define _VS_VIS_VIEWER_HPP
/**
 * @file viewer.hpp
 * @brief Handles the communication the the bot-viewer. Sending object collections and links.
 * @author Hordur Johannsson
 * @version $Id: $
 *
 * (c) 2010 Massachusetts Institute of Technology
 *
 */

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <lcm/lcm.h>
#include <isam/isam.h>

#include "visualization/collections.hpp"
#include "visualization/pointcloud.hpp"

class ObjectCollection;
class LinkCollection;
class PointCloudCollection;
class CovCollection;
class TextCollection;

/**
 * Handles communication with a viewer to display data from the map database.
 */ 
class Viewer
{
public:
    Viewer(lcm_t* lcm);
  
    virtual ~Viewer();
    virtual void sendCollection(const ObjectCollection & collection, bool reset = false);
    virtual void sendCollection(const LinkCollection & collection, bool reset = false);
    virtual void sendCollection(const PointCloudCollection & collection, bool reset = false);
    virtual void sendCollection(const CovCollection & collection, bool reset = false);
    virtual void sendCollection(const TextCollection & collection, bool reset = false);

    void setConfig (const vs_collection_config_t config);

    typedef boost::shared_ptr<Viewer> Ptr;
    typedef boost::shared_ptr<const Viewer> ConstPtr;
private:
    lcm_t* m_lcm;     
};

class Collection
{
public:
    Collection() {}
    Collection(int id, const std::string & name) : m_id(id), m_name(name) {}
    virtual ~Collection() {}
    int id() const {return m_id;}
    const std::string & name() const {return m_name;}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) = 0;
    virtual void clear() = 0;
private:
    int m_id;
    std::string m_name;
};

/**
 * Contains a timed pose
 */
struct Pose3dTime
{
    Pose3dTime(int64_t utime, const isam::Pose3d & pose) : utime(utime), pose(pose) {}
    int64_t utime;
    isam::Pose3d pose;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CovTime
{
    CovTime(int64_t utime, const Eigen::MatrixXd & cov) : utime(utime), cov(cov) {}
    int64_t utime;
    Eigen::MatrixXd cov;
};


struct Link
{
    Link(int64_t id, int32_t collection1, int64_t id1,
             int32_t collection2, int64_t id2)
             : id(id),
               collection1(collection1), id1(id1),
               collection2(collection2), id2(id2) {}
    int64_t id;
    int32_t collection1;
    int64_t id1;
    int32_t collection2;
    int64_t id2;
};

struct Text
{
    Text(int64_t id, int32_t collection_id, int64_t object_id, const std::string & text)
      : id(id),
        collection_id(collection_id), object_id(object_id),
        text(text) {}
    int64_t id;
    int32_t collection_id;
    int64_t object_id;
    std::string text;
};

class ObjectCollection : public Collection
{
public:
    typedef std::vector<Pose3dTime, Eigen::aligned_allocator<Pose3dTime> > CollectionType;

    ObjectCollection(int id, const std::string & name, int type=VS_OBJ_COLLECTION_T_AXIS3D) : Collection(id,name),m_first(true),m_type(type) {}
    virtual ~ObjectCollection() {}
    void add(const Pose3dTime & poseTime) { m_poses.push_back(poseTime); }
    void add(int64_t utime, const isam::Pose3d & pose) {
        add(Pose3dTime(utime, pose));
    }
    void add(int64_t utime, const Eigen::Isometry3d & pose) {
        add(Pose3dTime(utime, isam::Pose3d(pose)));
    }
    const CollectionType & poses() const {return m_poses;}
    CollectionType & poses() {return m_poses;}
    const Pose3dTime & operator()(size_t i) const {return m_poses[i];}
    virtual void clear() {m_poses.clear();}
    size_t size() const {return m_poses.size();}
    int type() const {return m_type;}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        if (m_first)
        {
            reset = true;
            m_first = false;
        }

        viewer->sendCollection(*this,reset);
    };

    static boost::shared_ptr<ObjectCollection> makeCollection(int id, const std::string & name)
    {
        return boost::make_shared<ObjectCollection>(id, name);
    }

    typedef boost::shared_ptr<ObjectCollection> Ptr;
    typedef boost::shared_ptr<const ObjectCollection> ConstPtr;
private:
    CollectionType m_poses;
    bool m_first;
    int m_type;
};

class LinkCollection : public Collection
{
public:
    // LinkCollection() : m_first(true) {}
    LinkCollection(int id, const std::string & name) : Collection(id,name), m_first(true) {}
    virtual ~LinkCollection() {}
    
    void add(int64_t id, int collection1, int64_t id1,
             int collection2, int64_t id2) {
        m_links.push_back(Link(id, collection1, id1, collection2, id2));
    }
    const std::vector<Link> & links() const {return m_links;}
    std::vector<Link> & links() {return m_links;}
    const Link & operator()(size_t i) const {return m_links[i];}
    virtual void clear() {m_links.clear();}
    size_t size() const {return m_links.size();}
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        if (m_first)
        {
            reset = true;
            m_first = false;
        }
      
        viewer->sendCollection(*this,reset);
    };

    static boost::shared_ptr<LinkCollection> makeCollection(int id, const std::string & name)
    {
        return boost::make_shared<LinkCollection>(id, name);
    }   

    typedef boost::shared_ptr<LinkCollection> Ptr;
    typedef boost::shared_ptr<const LinkCollection> ConstPtr;
private:
    std::vector<Link> m_links;  
    bool m_first;
};

class PointCloudCollection : public Collection
{
public:
    PointCloudCollection(int id,
                         const std::string & name,
                         int objectCollectionId,
                         int type = VS_POINT3D_LIST_COLLECTION_T_POINT)
        : Collection(id,name),
          m_objectCollectionId(objectCollectionId),
          m_first(true),
          m_type(type) {}
    PointCloudCollection(const PointCloudCollection& collection) 
      : Collection(collection), 
        m_pointClouds(collection.m_pointClouds),
        m_objectCollectionId(collection.m_objectCollectionId),
        m_first(collection.m_first)
    { }

    virtual ~PointCloudCollection() {}
        
    void add(boost::shared_ptr<PointCloudPtr > pointCloud) {
        boost::mutex::scoped_lock scoped_lock(mutex);         
        m_pointClouds.push_back(pointCloud);
    }
    boost::shared_ptr<PointCloudPtr > operator()(size_t i) const {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      return m_pointClouds[i];
    }
    virtual void clear() {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      m_pointClouds.clear();
    }
    size_t size() const {
      boost::mutex::scoped_lock scoped_lock(mutex); 
      return m_pointClouds.size(); 
    }
    int type() const {
      return m_type;
    }
    int objectCollectionId() const { return m_objectCollectionId; }
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        boost::mutex::scoped_lock scoped_lock(mutex);
        if (m_first)
        {
            reset = true;
            m_first = false;
        }
        PointCloudCollection collection(*this);
        scoped_lock.unlock();

        viewer->sendCollection(collection, reset);
        clear();
    };
    
    static boost::shared_ptr<PointCloudCollection> makeCollection(int id, const std::string & name, int objectCollectionId)
    {
        return boost::make_shared<PointCloudCollection>(id, name, objectCollectionId);
    }

    typedef boost::shared_ptr<PointCloudCollection> Ptr;
    typedef boost::shared_ptr<const PointCloudCollection> ConstPtr;
private:    
    mutable boost::mutex mutex;  
    std::vector<boost::shared_ptr<PointCloudPtr > > m_pointClouds;
    const int m_objectCollectionId;
    bool m_first;
    int m_type;
};

class CovCollection : public Collection
{
public:
    CovCollection(int id, const std::string & name, int objectCollectionId)
    : Collection(id,name), m_objectCollectionId(objectCollectionId), m_first(true) {}
    virtual ~CovCollection() {}
    void add(const CovTime & covTime) { m_covs.push_back(covTime); }
    void add(int64_t utime, const Eigen::MatrixXd & cov) {
        add(CovTime(utime, cov));
    }
    const std::vector<CovTime> & cov() const {return m_covs;}
    std::vector<CovTime> & cov() {return m_covs;}
    const CovTime & operator()(size_t i) const {return m_covs[i];}
    virtual void clear() {m_covs.clear();}
    size_t size() const {return m_covs.size();}
    int objectCollectionId() const { return m_objectCollectionId; }
    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        if (m_first)
        {
            reset = true;
            m_first = false;
        }

        viewer->sendCollection(*this,reset);
    };

    static boost::shared_ptr<CovCollection> makeCollection(int id, const std::string & name, int objectCollectionId)
    {
        return boost::make_shared<CovCollection>(id, name, objectCollectionId);
    }

    typedef boost::shared_ptr<CovCollection> Ptr;
    typedef boost::shared_ptr<const CovCollection> ConstPtr;
private:
    std::vector<CovTime> m_covs;
    const int m_objectCollectionId;
    bool m_first;
};

class TextCollection : public Collection
{
public:
    TextCollection(int id,
                   const std::string & name,
                   int type = 0)
        : Collection(id,name),
          m_first(true),
          m_type(type) {}

    TextCollection(const TextCollection& collection)
      : Collection(collection),
        m_texts(collection.m_texts),
        m_first(collection.m_first),
        m_type(collection.m_type)
    { }

    virtual ~TextCollection() {}

    void add(int64_t id, int collection_id, int64_t object_id, const std::string & text) {
      m_texts.push_back(Text(id, collection_id, object_id, text));
    }

    Text operator()(size_t i) const {
      boost::mutex::scoped_lock scoped_lock(mutex);
      return m_texts[i];
    }
    virtual void clear() {
      boost::mutex::scoped_lock scoped_lock(mutex);
      m_texts.clear();
    }
    size_t size() const {
      boost::mutex::scoped_lock scoped_lock(mutex);
      return m_texts.size();
    }
    int type() const {
      return m_type;
    }

    virtual void sendCollection(boost::shared_ptr<Viewer> viewer, bool reset) {
        boost::mutex::scoped_lock scoped_lock(mutex);
        if (m_first)
        {
            reset = true;
            m_first = false;
        }
        TextCollection collection(*this);
        scoped_lock.unlock();

        viewer->sendCollection(collection, reset);
        clear();
    };
    typedef boost::shared_ptr<TextCollection> Ptr;
    typedef boost::shared_ptr<const TextCollection> ConstPtr;
private:
    mutable boost::mutex mutex;
    std::vector<Text> m_texts;
    bool m_first;
    int m_type;
};

#endif
