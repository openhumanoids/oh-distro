#include "convert_octomap.hpp"

ConvertOctomap::ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_):
    lcm_(lcm_), co_cfg_(co_cfg_){
      
  verbose_ = 0; // 3 lots, 2 some, 1 v.important
  tree_  = new ColorOcTree(co_cfg_.octomap_resolution);
  std::cout << "Resolution: " << co_cfg_.octomap_resolution << "  ====================" << endl;

  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

  // define colors
  yellow = new ColorOcTreeNode::Color(250,250,0);
  green = new ColorOcTreeNode::Color(0,102,0);
  blue = new ColorOcTreeNode::Color(0,0,255);
}

void ConvertOctomap::updateOctree(pronto::PointCloud* &cloud, octomap::ColorOcTree* tree){
  //  Takes about 2.5 sec to convert 400scans or 400k points into an octree

  // double res = 0.1; // used 0.1 and blur of 0.1 was too sharp
  std::cout << "Resolution: " << co_cfg_.octomap_resolution << endl;
  double maxrange = -1;
  int max_scan_no = -1;
  unsigned char compression = 0;

  // 1. Convert to octomap graph:
  ScanGraph* graph = convertPointCloudToScanGraph(cloud);

  timeval start; 
  timeval stop; 

  cout << "===========================\nUpdating tree...\n===========================\n";

  gettimeofday(&start, NULL);  // start timer
  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;
  bool discretize = false;

  ColorOcTree* tmp_tree = new ColorOcTree(co_cfg_.octomap_resolution);

  tree->enableChangeDetection(true);  
  tmp_tree->enableChangeDetection(true);  

  // get default sensor model values:
  OcTree emptyTree(0.1);
  double clampingMin = emptyTree.getClampingThresMin();
  double clampingMax = emptyTree.getClampingThresMax();
  double probMiss = emptyTree.getProbMiss();
  double probHit = emptyTree.getProbHit();

  tree->setClampingThresMin(clampingMin);
  tree->setClampingThresMax(clampingMax);
  tree->setProbHit(probHit);
  tree->setProbMiss(probMiss);

  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
      
    tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);
    tmp_tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);

    if (compression == 2){
      tree->toMaxLikelihood();
      tmp_tree->toMaxLikelihood();
      tree->prune();
      tmp_tree->prune();
    }

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }

  gettimeofday(&stop, NULL);  // stop timer
  
  double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

  cout << "Time to Insert: " << time_to_insert << " seconds." << endl;

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;
  
  // iterate through the new nodes of the tree
  tree->expand();
  tmp_tree->expand();

  for(ColorOcTree::tree_iterator it=tmp_tree->begin_tree(),
      end=tmp_tree->end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      point3d coord = tmp_tree->keyToCoord(it.getKey());
      if (tmp_tree->isNodeOccupied(*it)) {
        
        ColorOcTreeNode* n = tree->setNodeValue(coord, true);
        n = tree->updateNode(coord, true);

        n->setColor(*blue); // set color to blue
      }
    }
  }
  tree->updateInnerOccupancy();

  /*
  //DEBUGGING:
  std::string old_path = getDataPath(); 
  static char octree_col_name[50];
  sprintf(octree_col_name, "octomap-pre-colored.ot");
  std::string col_path  = string( old_path + "/" +  octree_col_name) ;
  cout << "Saving color octree to: " << col_path << endl;;
  tree->write(col_path);
  publishOctree(tmp_tree, "OCTOMAP");*/
  
  delete tmp_tree;
  tree->prune();
}

ScanGraph* ConvertOctomap::convertPointCloudToScanGraph(pronto::PointCloud* &cloud)
{
  // Convert cloud to octomap graph:
  ScanGraph* graph = new ScanGraph();
  octomap::Pointcloud* scan = new octomap::Pointcloud();
  //float x, y, z, roll, pitch, yaw;
  //pose6d pose(x, y, z, roll, pitch, yaw);
  pose6d pose(0, 0, 0, 0, 0, 0);
  //std::cout << "Pose "<< pose << " found.\n";

  for (size_t i = 0; i < cloud->points.size(); i++){
    scan->push_back(cloud->points[i].x,
                                 cloud->points[i].y,
                                 cloud->points[i].z);
  }
  graph->addNode(scan, pose);
  graph->connectPrevious();

  return graph;
}

void ConvertOctomap::publishOctree(ColorOcTree* tree, std::string channel){

  octomap_raw_t msg;
  msg.utime = bot_timestamp_now();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = 0;
    }
    msg.transform[i][i] = 1;
  }

  tree->expand();

  std::stringstream datastream;
  tree->writeBinaryConst(datastream);
  std::string datastring = datastream.str();
  msg.data = (uint8_t *) datastring.c_str();
  msg.length = datastring.size();
  octomap_raw_t_publish(lcm_->getUnderlyingLCM(), channel.c_str(), &msg);
}

void ConvertOctomap::colorChanges(ColorOcTree& tree, int idx){
  unsigned int changedOccupied = 0;
  unsigned int changedFree = 0;
  unsigned int missingChanged = 0;

  tree.expand();

  // iterate through the changed nodes
  KeyBoolMap::const_iterator it;
  bool tmp;
  for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
    ColorOcTreeNode* node = tree.search(it->first);
    if (node != NULL) 
    {
      if (tree.isNodeOccupied(node)) {
        // new occupied space
        if (idx == 0)
          node->setColor(*yellow); // set color to yellow
        else
          node->setColor(*green); // set color to green
        changedOccupied += 1;
      }
      else {
        // rays from center to detected surface (new free space, unknown before).
        changedFree += 1;
      }
    }
    else 
    {
      missingChanged +=1;
    }
  }

  cout << "----------non leaves included----------" << endl;
  cout << "Colored nodes (new occ space): " << changedOccupied << endl;
  cout << "Empty nodes (new free space): " << changedFree << endl;
  cout << "---------------------------------------" << endl;

  tree.prune();
}

void ConvertOctomap::printChangesByColor(ColorOcTree& tree){
  unsigned int fromFreeToOccupied = 0;
  unsigned int fromOccupiedToFree = 0;
  unsigned int alreadyOccupied = 0;

  tree.expand();

  // iterate through the entire tree
  for(ColorOcTree::tree_iterator it=tree.begin_tree(),
      end=tree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      //NOTE: leaves have size = resolution.
      ColorOcTreeNode* node = tree.search(it.getKey());
      ColorOcTreeNode::Color c = node->getColor();
      if (tree.isNodeOccupied(node)) {
        if (c == *blue)
          alreadyOccupied += 1;
        else if (c == *green) 
          fromFreeToOccupied += 1;
        else if (c == *yellow)
          fromOccupiedToFree += 1;
      }
    } 
  }

  cout << "----------non leaves excluded----------" << endl;
  cout << "fromFreeToOccupied: " << fromFreeToOccupied << endl;
  cout << "fromOccupiedToFree: " << fromOccupiedToFree << endl;
  cout << "alreadyOccupied: " << alreadyOccupied << endl;
  cout << "---------------------------------------" << endl;

  tree.prune();
}

void ConvertOctomap::printChangesAndActual(ColorOcTree& tree){
  unsigned int changedOccupied = 0;
  unsigned int changedFree = 0;
  unsigned int actualOccupied = 0;
  unsigned int actualFree = 0;
  unsigned int missingChanged = 0;

  tree.expand();

  // iterate through the changed nodes
  KeyBoolMap::const_iterator it;
  for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
    ColorOcTreeNode* node = tree.search(it->first);
    if (node != NULL) {
      if (tree.isNodeOccupied(node)) {
        changedOccupied += 1;
      }
      else {
        changedFree += 1;
      }
    } else {
      missingChanged +=1;
    }
  }

  unsigned int notLeaf = 0;

  // iterate through the entire tree
  for(ColorOcTree::tree_iterator it=tree.begin_tree(),
      end=tree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      if (tree.isNodeOccupied(*it)) {
        actualOccupied += 1;
      }
      else {
        actualFree += 1;
      }
    }
    else
      notLeaf += 1;
  }

  cout << "Expanded tree size: " << tree.size() <<" nodes." << endl;
  
  cout << "Change detection: " << changedOccupied << " occ; " << changedFree << " free; "<< missingChanged << " missing;" << endl;
  cout << "Number of changes: " << changedOccupied+changedFree << endl;
  cout << "Actual: " << actualOccupied << " occ; " << actualFree << " free; " << notLeaf << " not leaf; " << endl;
  cout << "Number of actual: " << actualOccupied+actualFree+notLeaf << endl;

  tree.prune();
}


void backupFiles(){
  std::string old_path = getDataPath();
  
  time_t rawtime;
  struct tm * timeinfo;  
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );  
  static char newdir_name[150];
  sprintf(newdir_name, "backup-%d-%02d-%02d-%02d-%02d-%02d",
    1900 + timeinfo->tm_year,
    timeinfo->tm_mon,    
    timeinfo->tm_mday, timeinfo->tm_hour,
    timeinfo->tm_min, timeinfo->tm_sec);
  std::string new_path  = string( old_path + "/" +  newdir_name) ;
  std::cout << new_path << "\n";
  mkdir( new_path.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
  
  // if the files exist move them:
  rename( string( old_path + "/octomap.pcd" ).c_str() , 
          string( new_path + "/octomap.pcd" ).c_str() );  
  rename( string( old_path + "/octomap.bt" ).c_str() , 
          string( new_path + "/octomap.bt" ).c_str() );  
  rename( string( old_path + "/octomap.bt_blurred" ).c_str() , 
          string( new_path + "/octomap.bt_blurred" ).c_str() );     
}

void ConvertOctomap::doWork(pronto::PointCloud* &cloud){
  // set default name for octree visualization
  doWork(cloud, "OCTOMAP");
}

void ConvertOctomap::doWork(pronto::PointCloud* &cloud, string octree_channel){
  int idx = 0;

  if (octree_channel != "OCTOMAP_REF")
  {
    tree_->resetChangeDetection();
    idx = 1;    
  }
  updateOctree(cloud, tree_);

  publishOctree(tree_, octree_channel);
  colorChanges(*tree_, idx);
  
  std::string old_path = getDataPath(); 
 
  // write binary tree to bt 
  static char octree_bin_name[50];
  sprintf(octree_bin_name, "octomap-%d.bt", idx);
  std::string binary_path  = string( old_path + "/" +  octree_bin_name) ;
  cout << "Saving binary octree to: " << binary_path << endl;
  tree_->writeBinary(binary_path);
  // write color tree to ot
  static char octree_col_name[50];
  sprintf(octree_col_name, "octomap-colored-%d.ot", idx);
  std::string col_path  = string( old_path + "/" +  octree_col_name) ;
  cout << "Saving color octree to: " << col_path << endl;;
  tree_->write(col_path);

  /*
  octomap::OcTree * tree_blurred;
  if (co_cfg_.blur_map){
    timeval start; 
    timeval stop; 
    gettimeofday(&start, NULL);  // start timer  
    tree_->toMaxLikelihood();
    tree_->expand();
    printf("Blurring octomap\n");
    double minNegLogLike;
    
    tree_blurred = octomap_utils::octomapBlur(tree_, co_cfg_.blur_sigma, &minNegLogLike);
    gettimeofday(&stop, NULL);  // stop timer
    double time_to_blur = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
    if (verbose_>=1) cout << "time to insert scans: " << time_to_blur << " sec" << endl;
    // publishOctree(tree_blurred,"OCTOMAP_BLURRED");
   
    // Backup  old octomaps and write new ones:
    backupFiles();
    
    std::stringstream s1;
    s1 <<  getDataPath() <<   "/octomap.pcd" ;
    //printf("Saving original point cloud to: %s\n", s1.str().c_str());
    pc_vis_->writePCD(s1.str(), *cloud);
    //pcl::PCDWriter writer;
    //writer.write (s1.str(), *cloud, true); // binary =true
    //cout << "Finished writing "<< cloud->points.size() <<" points to:\n" << s1.str() <<"\n";
    
    //s << "_blurred"; // " << co_cfg_.blur_sigma ;
    //std::cout << "Saving blurred map to: " << s.str() << " with blur sigma of " << co_cfg_.blur_sigma << "\n" ;
    //octomap_utils::saveOctomap(tree_blurred, s.str().c_str(), minNegLogLike);      
  } 
  */
}
