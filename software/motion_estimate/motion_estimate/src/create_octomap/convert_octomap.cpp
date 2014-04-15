#include "convert_octomap.hpp"

ConvertOctomap::ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_):
    lcm_(lcm_), co_cfg_(co_cfg_){
      
  verbose_ = 1; // 3 lots, 2 some, 1 v.important
  tree_  = new OcTree(1); // set else where
}


OcTree* ConvertOctomap::convertPointCloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  //  Takes about 2.5 sec to convert 400scans or 400k points into an octree
  
  bool write_output_detailed = false;
  double res = 0.1; // used 0.1 and blur of 0.1 was too sharp
  std::cout << "using a base resolution of " << res << "  ========================\n";
  double maxrange = -1;
  int max_scan_no = -1;
  unsigned char compression = 0;

  
  std::string path = "/tmp/";
  
            
  // 2. Convert to octomap graph:
  ScanGraph* graph = new ScanGraph();
  Pointcloud* scan = new Pointcloud();
  //float x, y, z, roll, pitch, yaw;
  //pose6d pose(x, y, z, roll, pitch, yaw);
  pose6d pose(0, 0, 0, 0, 0, 0);
  //std::cout << "Pose "<< pose << " found.\n";
  for (size_t i = 0; i < cloud->points.size(); i++){
    scan->push_back(cloud->points[i].x,
                                 cloud->points[i].y,
                                 cloud->points[i].z);
  }
  if (verbose_>=1) std::cout << "Added points to octomap graph\n";
  graph->addNode(scan, pose);
  graph->connectPrevious();
  if (write_output_detailed){
    std::string filename_out = path + "example_sweep_cloud_400scans.graph";
    graph->writeBinary(filename_out);
  }
            
  std::string treeFilename = path + "example_sweep_cloud_400scans.bt";
  timeval start; 
  timeval stop; 
  std::string treeFilenameOT = treeFilename + ".ot";
  std::string treeFilenameMLOT = treeFilename + "_ml.ot";

  
  unsigned int num_points_in_graph = 0;
  if (max_scan_no > 0) {
    num_points_in_graph = graph->getNumPoints(max_scan_no-1);
    if (verbose_>=3) cout << "Data points in graph up to scan " << max_scan_no << ": " << num_points_in_graph << endl;
  }
  else {
    num_points_in_graph = graph->getNumPoints();
    if (verbose_>=1) cout << "Data points in graph: " << num_points_in_graph << endl;
  }
  std::ofstream logfile;
  if (write_output_detailed){
    logfile.open((treeFilename+".log").c_str());
    logfile << "# Memory of processing " << "graph" << " over time\n";
    logfile << "# Resolution: "<< res <<"; compression: " << int(compression) << "; scan endpoints: "<< num_points_in_graph << std::endl;
    logfile << "# [scan number] [bytes octree] [bytes full 3D grid]\n";
  }

  if (verbose_>=1) cout << "Creating tree\n===========================\n";
  OcTree* tree = new OcTree(res);


  gettimeofday(&start, NULL);  // start timer
  unsigned int numScans = graph->size();
  unsigned int currentScan = 1;
  for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

    tree->insertScan(**scan_it, maxrange, (compression==1));
    if (compression == 2){
      tree->toMaxLikelihood();
      tree->prune();
    }

    if (write_output_detailed)
      logfile << currentScan << " " << tree->memoryUsage() << " " << tree->memoryFullGrid() << "\n";

    if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
      break;

    currentScan++;
  }
  gettimeofday(&stop, NULL);  // stop timer
  
  double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

  // get rid of graph in mem before doing anything fancy with tree (=> memory)
  delete graph;
  if (logfile.is_open())
    logfile.close();


  if (verbose_>=1) cout << "Done building tree.\n";
  if (verbose_>=1) cout << "time to insert scans: " << time_to_insert << " sec" << endl;
  if (verbose_>=3) cout << "time to insert 100.000 points took: " << time_to_insert/ ((double) num_points_in_graph / 100000) << " sec (avg)" << endl << endl;

  // Additional work:
  unsigned int numThresholded, numOther;
  tree->calcNumThresholdedNodes(numThresholded, numOther);

  std::cout << "Full tree\n" << "===========================\n";
  cout << "Tree size: " << tree->size() <<" nodes (" <<numThresholded <<" thresholded, "<< numOther << " other)\n";

  unsigned int memUsage = tree->memoryUsage();
  unsigned int memFullGrid = tree->memoryFullGrid();
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
  double x, y, z;
  tree->getMetricSize(x, y, z);
  cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";

  std::cout << "Pruned tree (lossless compression)\n" << "===========================\n";
  tree->prune();
  tree->calcNumThresholdedNodes(numThresholded, numOther);
  memUsage = tree->memoryUsage();

  cout << "Tree size: " << tree->size() <<" nodes (" <<numThresholded<<" thresholded, "<< numOther << " other)\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << endl;

  if (write_output_detailed) tree->write(treeFilenameOT);

  std::cout << "Pruned max-likelihood tree (lossy compression)\n" << "===========================\n";
  tree->toMaxLikelihood();
  tree->prune();
  tree->calcNumThresholdedNodes(numThresholded, numOther);
  memUsage = tree->memoryUsage();
  cout << "Tree size: " << tree->size() <<" nodes (" <<numThresholded<<" thresholded, "<< numOther << " other)\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << endl;


  if (write_output_detailed){
    cout << "\nWriting tree files\n===========================\n";
    tree->write(treeFilenameMLOT);
    std::cout << "Full Octree (pruned) written to "<< treeFilenameOT << std::endl;
    std::cout << "Full Octree (max.likelihood, pruned) written to "<< treeFilenameOT << std::endl;
    
    // This is the formal acutally used by mav est:
    tree->writeBinary(treeFilename);
    std::cout << "Bonsai tree written to "<< treeFilename << std::endl;
    cout << endl;  
  }
            
  return tree;
}


void ConvertOctomap::publishOctree(OcTree* tree, std::string channel){

  octomap_raw_t msg;
  msg.utime = bot_timestamp_now();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = 0;
    }
    msg.transform[i][i] = 1;
  }

  std::stringstream datastream;
  tree->writeBinaryConst(datastream);
  std::string datastring = datastream.str();
  msg.data = (uint8_t *) datastring.c_str();
  msg.length = datastring.size();
  octomap_raw_t_publish(lcm_->getUnderlyingLCM(), channel.c_str(), &msg);
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



void ConvertOctomap::doWork(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
            
  tree_ = convertPointCloudToOctree(cloud);
  publishOctree(tree_,"OCTOMAP");

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
    printf("Saving original point cloud to: %s\n", s1.str().c_str());
    pcl::PCDWriter writer;
    writer.write (s1.str(), *cloud, true); // binary =true
    cout << "Finished writing "<< cloud->points.size() <<" points to:\n" << s1.str() <<"\n";
    
    std::stringstream s;
    s <<  getDataPath() <<   "/octomap.bt" ;
    printf("Saving original map to: %s\n", s.str().c_str());
    tree_->writeBinary(s.str().c_str());
    s << "_blurred"; // " << co_cfg_.blur_sigma ;
    std::cout << "Saving blurred map to: " << s.str() << " with blur sigma of " << co_cfg_.blur_sigma << "\n" ;
    octomap_utils::saveOctomap(tree_blurred, s.str().c_str(), minNegLogLike);  
      
  }
  
  
  
}
