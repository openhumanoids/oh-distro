#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <Eigen/Geometry>

#include <lcmtypes/drc_map_image_t.h>
#include <lcmtypes/drc_map_params_t.h>


#include <ConciseArgs>
using namespace std;
using namespace boost;

  lcm_t* subscribe_lcm;

void read_log(std::string poses_files,std::string channel){

    vector< vector <float> > scan_stack;
  
  int counter=0;
  string line;
  ifstream myfile (poses_files.c_str());
  if (myfile.is_open()){

    int64_t Preutime;
    int64_t Postutime = 0;

    counter =0;
    while ( myfile.good() ){

      getline (myfile,line);
      counter++;
      if (line.size() > 4){
	char_separator<char> sep(" ");
          tokenizer< char_separator<char> > tokens(line, sep);
          int count=0;
          vector <float> scan;
          int expected_scan_size=0;
          BOOST_FOREACH (const string& t, tokens) {
              float val = atof ( t.c_str() );
              scan.push_back(val);
            count++;
          }
          scan_stack.push_back(scan);
       
      }else{
        cout << "[ "<< line << "] ... short\n";
        //exit(-1);
      }
      
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",poses_files.c_str());
    return;
  }

  cout << "read " << scan_stack.size() << " rows\n";
  cout << "read " << scan_stack[0].size()<< " cols\n";
  
 
    cout << "Publishing height map (downsample=)...\n";
    drc_map_image_t heightMapMsg;
    heightMapMsg.utime =0;// bot_timestamp_now();
    heightMapMsg.width = 111;//heightMap.mWidth;
    heightMapMsg.height = 124;// heightMap.mHeight;
    heightMapMsg.row_bytes = sizeof(float)*heightMapMsg.width;
    heightMapMsg.total_bytes = heightMapMsg.height*heightMapMsg.row_bytes;
//    heightMapMsg.scale_x =0.08;// (px-p0).norm();
//    heightMapMsg.scale_y =0.08;// (py-p0).norm();
    
//    double t2l[][4]={ {0.08,0,0,-4.7631},{0,0.08,0, -4.43}, {0,0,1,-0.779999999999}, {0,0,0,1}};
    double t2l[][4]={ {1,0,0,0},{0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
heightMapMsg.format = DRC_MAP_IMAGE_T_FORMAT_GRAY_FLOAT32;
    
heightMapMsg.compression = DRC_MAP_IMAGE_T_COMPRESSION_NONE;
heightMapMsg.channels = 1;// unused currently
     
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        heightMapMsg.transform[i][j] =
          t2l[i][j];//heightMap.mTransformToLocal(i,j);
      }
    }
    //heightMapMsg.heights = heightMap.mData;
    
    float heights[heightMapMsg.total_bytes];
    
  for (int i = 0; i < heightMapMsg.width; i++) {
        for (int j = 0; j < heightMapMsg.height; j++) { //
	heights[j*heightMapMsg.width+i] = scan_stack[   j][ i ];

	}
  }
  heightMapMsg.data =(uint8_t*) heights;
    drc_map_image_t_publish(subscribe_lcm,channel.c_str(), &heightMapMsg);
    
//    mLcm->publish(mHeightMapChannel, &heightMapMsg);  
  
  
  
}




int main(int argc, char ** argv) {
  string filename = "gradmap.txt";
  string channel = "COST_MAP";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(filename, "f", "filename","filename");
  opt.add(channel, "c", "channel","channel");
  opt.parse();
  std::cout << "filename: " << filename << "\n";  
  std::cout << "channel: " << channel << "\n";  
  
  subscribe_lcm =lcm_create(NULL);

  read_log(filename,channel);
  return 0;
}
