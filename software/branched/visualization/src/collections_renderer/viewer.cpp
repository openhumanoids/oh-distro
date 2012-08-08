#include "visualization/viewer.hpp"

//Viewer::Viewer(MapEstimate* m, lcm_t* lcm) : m_estimate(m), m_lcm(lcm)
Viewer::Viewer(lcm_t* lcm) : m_lcm(lcm)
{
}

Viewer::~Viewer()
{
}
void Viewer::sendCollection(const ObjectCollection & collection, bool reset)
{
    vs_obj_collection_t objs;
    size_t n = collection.size();
    if (n > 0) {
        objs.id = collection.id();
        objs.name = (char*) collection.name().c_str();
        objs.type = collection.type();
        objs.reset = reset;
        objs.nobjs = n;
        vs_obj_t poses[n];
        for (size_t i = 0; i < n; i++) {
            const isam::Pose3d & pose = collection(i).pose;
            poses[i].id = collection(i).utime;
            poses[i].x = pose.x();
            poses[i].y = pose.y();
            poses[i].z = pose.z();
            poses[i].yaw = pose.yaw();
            poses[i].pitch = pose.pitch();
            poses[i].roll = pose.roll();
        }
        objs.objs = poses;
        vs_obj_collection_t_publish(m_lcm, "OBJ_COLLECTION", &objs);
    }
}

void Viewer::sendCollection(const LinkCollection & collection, bool reset)
{ 
    vs_link_collection_t linkCollection;
    int n = collection.size();
    if (n > 0) {
        linkCollection.id = collection.id();
        linkCollection.name = (char*) collection.name().c_str();
        linkCollection.type = 0; // unused
        linkCollection.reset = reset;
        linkCollection.nlinks = n;
        vs_link_t links[n];
        for (int i = 0; i < n; i++) {
            const Link & link = collection(i);
            links[i].id = link.id;
            links[i].collection1 = link.collection1;
            links[i].id1 = link.id1;
            links[i].collection2 = link.collection2;
            links[i].id2 = link.id2;            
        }
        linkCollection.links = links; 
        vs_link_collection_t_publish(m_lcm, "LINK_COLLECTION", &linkCollection);
    }        
}

void Viewer::sendCollection(const PointCloudCollection & collection, bool reset)
{
    vs_point3d_list_collection_t point_lists;

    size_t m = collection.size();
    point_lists.id = collection.id();
    point_lists.name = (char *)collection.name().c_str();
    point_lists.type = collection.type();
    point_lists.reset = reset;
    point_lists.nlists = m;
    vs_point3d_list_t point_list[m];
    
    for(size_t i=0; i<collection.size(); i++)
    {
        PointCloudPtr pointCloud =  *(collection(i));
                
        vs_point3d_list_t* points = &(point_list[i]);
        int64_t scantime = pointCloud->utime();
        points->ncolors = 0;
        points->colors = NULL;
        points->nnormals = 0;
        points->normals = NULL;
        points->npointids = 0;
        points->pointids = NULL;

        if (pointCloud) {
          size_t k = pointCloud->size();
          bool has_color = (pointCloud->colors().size() == k);

          vs_point3d_t* entries = new vs_point3d_t[k];
          vs_color_t* colors = NULL;
          if (has_color) {
            points->ncolors = k;
            colors = new vs_color_t[k];
          }

          points->id = scantime;
          points->collection = collection.objectCollectionId();
          points->element_id = scantime;
          points->npoints = k;
          for (size_t j=0;j<k;j++) {
              entries[j].x = pointCloud->points()[j].x();
              entries[j].y = pointCloud->points()[j].y();
              entries[j].z = pointCloud->points()[j].z();           

              if (has_color) {
                colors[j].r = pointCloud->colors()[j].r;
                colors[j].g = pointCloud->colors()[j].g;
                colors[j].b = pointCloud->colors()[j].b;
              }
          }
          points->points = entries;         
          points->colors = colors;
        }
        else          
        {
          points->id = 0;
          points->collection = 0;
          points->element_id = 0;
          points->npoints = 0;        
          points->points = 0;
        }
    }
    
    point_lists.point_lists = point_list;
    vs_point3d_list_collection_t_publish(m_lcm,"POINTS_COLLECTION",&point_lists);
    for (int i=0;i<point_lists.nlists;i++) {
        delete [] point_lists.point_lists[i].points;
        delete [] point_lists.point_lists[i].colors;
    }   
}

void Viewer::sendCollection(const CovCollection & collection, bool reset)
{
    vs_cov_collection_t covs;
    size_t n = collection.size();
    if (n > 0) {
        covs.id = collection.id();
        covs.name = (char*) collection.name().c_str();
        covs.type = VS_COV_COLLECTION_T_ELLIPSOID;
        covs.reset = reset;
        covs.ncovs = n;
        vs_cov_t cov[n];

        for (size_t i = 0; i < n; i++) {
            const Eigen::MatrixXd & cov_mat = collection(i).cov;
            cov[i].id = collection(i).utime;
            cov[i].collection = collection.objectCollectionId();
            cov[i].element_id = collection(i).utime;
            cov[i].n = 6;
            double* entries = new double[cov[i].n];

            size_t entry_idx = 0;
            for (size_t row=0; row<3; row++)
              for (size_t col=0; col<3; col++)
                if (row <= col)
                  entries[entry_idx++] = cov_mat(row,col);

            cov[i].entries = entries;
        }
        covs.covs = cov;
        vs_cov_collection_t_publish(m_lcm, "COV_COLLECTION", &covs);

        for (size_t i = 0; i < n; i++) {
        	delete [] cov[i].entries;
        }

    }
}

void Viewer::sendCollection(const TextCollection & collection, bool reset)
{
    vs_text_collection_t texts;
    size_t n = collection.size();
    if (n > 0) {
        texts.id = collection.id();
        texts.name = (char*) collection.name().c_str();
        texts.type = VS_OBJ_COLLECTION_T_POSE;
        texts.reset = reset;
        texts.n = n;
        vs_text_t text[n];

        for (size_t i = 0; i < n; i++) {
            const Text & txt = collection(i);
            text[i].id = txt.id;
            text[i].collection_id = txt.collection_id;
            text[i].object_id = txt.object_id;
            text[i].text = const_cast<char*>(txt.text.c_str()); // save to const cast here because publish takes a const.
        }
        texts.texts = text;
        vs_text_collection_t_publish(m_lcm, "TEXT_COLLECTION", &texts);
    }
}


void Viewer::setConfig (const vs_collection_config_t config)
{
    vs_collection_config_t_publish(m_lcm, "COLLECTION_CONFIG", &config);
}

