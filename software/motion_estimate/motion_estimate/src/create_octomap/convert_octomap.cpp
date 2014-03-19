#include "convert_octomap.hpp"

ConvertOctomap::ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_):
    lcm_(lcm_), co_cfg_(co_cfg_){
      
  verbose_ = 1; // 3 lots, 2 some, 1 v.important
  
}


OcTree* ConvertOctomap::convertPointCloudToOctree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  //  Takes about 2.5 sec to convert 400scans or 400k points into an octree
  
  bool write_output_detailed = false;
  double res = 0.1;
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


void ConvertOctomap::doWork(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  std::string path = "/tmp/";
  std::cout << "Processing cloud with " << cloud->width * cloud->height
            << " points" << std::endl;  
            
  OcTree* tree = convertPointCloudToOctree(cloud);
  publishOctree(tree,"OCTOMAP");

  octomap::OcTree * tree_blurred;
  if (co_cfg_.blur_map){
    timeval start; 
    timeval stop; 
    gettimeofday(&start, NULL);  // start timer  
    tree->toMaxLikelihood();
    tree->expand();
    printf("Blurring octomap\n");
    double minNegLogLike;
    
    tree_blurred = octomap_utils::octomapBlur(tree, co_cfg_.blur_sigma, &minNegLogLike);
    gettimeofday(&stop, NULL);  // stop timer
    double time_to_blur = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
    if (verbose_>=1) cout << "time to insert scans: " << time_to_blur << " sec" << endl;
    // publishOctree(tree_blurred,"OCTOMAP_BLURRED");
  
    if (co_cfg_.write_output){
      std::stringstream s;
      s <<  getDataPath() <<   "/octomap.bt" ;
      printf("Saving original map to: %s\n", s.str().c_str());
      tree->writeBinary(s.str().c_str());
      s << "_blurred_" << co_cfg_.blur_sigma ;
      printf("Saving blurred map to: %s\n", s.str().c_str());
      octomap_utils::saveOctomap(tree_blurred, s.str().c_str(), minNegLogLike);  
    }
  }
  
  
  if (co_cfg_.repeat_period > 0) {
    std::cout << "Republishing unblurred octomap to LCM with period "<< co_cfg_.repeat_period << " sec\n";
    while (1) {
      usleep(1e6 * co_cfg_.repeat_period);
      fprintf(stderr, ".");
      publishOctree(tree,"OCTOMAP");
      // if (co_cfg_.blur_map) publishOctree(tree_blurred,"OCTOMAP_BLURRED");
    }
  }  
}