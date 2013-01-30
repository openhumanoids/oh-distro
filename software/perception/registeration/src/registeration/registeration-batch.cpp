#include <dirent.h>
#include "registeration.hpp"
#include <ConciseArgs>

using namespace std;

class RegApp{
  public:
    RegApp(boost::shared_ptr<lcm::LCM> &publish_lcm);
    
    ~RegApp(){
    }
    
    void doRegisterationBatch(std::string ref_filename);
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    Reg::Ptr reg;
};


void read_features(std::string fname,
    std::vector<ImageFeature>& features ){

  printf( "About to read: %s - ",fname.c_str());
  int counter=0;
  string line0;
  std::ifstream myfile (fname.c_str());
  if (myfile.is_open()){

    getline (myfile,line0);
    //cout << line0 << " is first line\n";

    counter =0;
    while ( myfile.good() ){
      string line;
      getline (myfile,line);
      if (line.size() > 4){
        ImageFeature f;
        int i,track_id;
        double v[15];
        int d[3];
        int res = sscanf(line.c_str(), "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d",&i,
            &(v[0]), &(v[1]), &(v[2]), &(v[3]), &(v[4]), // id, uv, base_uv
            &(v[5]), &(v[6]), &(v[7]),  &(v[8]), &(v[9]), &(v[10]),// uvd xyz
            &(v[11]), &(v[12]), &(v[13]), &(v[14]), // xyzd
            &(d[0]), &(d[1]), &(d[2])        );

        f.track_id=v[0]; f.uv[0]=v[1];f.uv[1]=v[2];
        f.base_uv[0]=v[3];f.base_uv[1]=v[4];
        f.uvd[0]=v[5];f.uvd[1]=v[6];f.uvd[2]=v[7];
        f.xyz[0]=v[8];f.xyz[1]=v[9];f.xyz[2]=v[10];
        f.xyzw[0]=v[11];f.xyzw[1]=v[12];f.xyzw[2]=v[13];f.xyzw[3]=v[14];
        f.color[0] = d[0];f.color[1] = d[1];f.color[2] = d[2];

        /*
        cout << line << " is line\n";
        cout << "i: " << i <<"\n";
        cout << "f.track_id: " << f.track_id <<"\n";
        cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
        cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
        cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
        cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
        cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
        cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";
         */
        features.push_back(f);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open features file\n%s",fname.c_str());
    return;
  }
  cout << "read " << features.size() << " features\n";
}


RegApp::RegApp(boost::shared_ptr<lcm::LCM> &lcm_):          
    lcm_(lcm_){
  reg = Reg::Ptr (new Reg (lcm_));
}
  

void RegApp::doRegisterationBatch(std::string ref_filename){
  std::vector<string> futimes;
  std::vector<string> utimes_strings;
  
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (".")) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      string fname = ent->d_name;
      if(fname.size() > 5){
        if (fname.compare(fname.size()-4,4,"feat") == 0){ 
          printf ("%s\n", ent->d_name);
          futimes.push_back( fname.substr(0,21) );
          utimes_strings.push_back( fname.substr(5,16) );
        }
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    exit(-1);
  }
  
  istringstream temp_buffer( ref_filename );
  string main_fname;
  temp_buffer >> main_fname; 
  
  string temp = main_fname.substr(5,16);
  istringstream temp_buffer2( temp);
  int64_t main_utime;
  temp_buffer2 >> main_utime ; 
  
  cout << main_fname << " fname\n";
  cout << main_utime << " utime\n";
  
  stringstream ifile0, featfile0;
  ifile0 << main_fname << "_left.png";
  featfile0 << main_fname << ".feat";
  cv::Mat img0 = cv::imread( ifile0.str(), CV_LOAD_IMAGE_GRAYSCALE );
  std::vector<ImageFeature> features0;
  read_features(featfile0.str(), features0);
  
  for (size_t i= 0 ; i<  futimes.size(); i++){
    istringstream buffer(utimes_strings[i]);
    int64_t utime;
    buffer >> utime; 

    cout << i << " count\n";
    cout << "doing: " << i << " - "<<futimes[i] <<"\n";
    cout << " and   " << utimes_strings[i] <<"\n";
    cout << " utime " << utime << "\n";
    stringstream ifile1, featfile1;
    ifile1 << futimes[i] << "_left.png";
    featfile1 << futimes[i] << ".feat";

    /// 1. Read in imgs and featues:
    cv::Mat img1 = cv::imread( ifile1.str(), CV_LOAD_IMAGE_GRAYSCALE );
  
    std::vector<ImageFeature> features1;
    read_features(featfile1.str(), features1);
    
    //FrameMatchPtr match =  
    reg->align_images(img0, img1, features0, features1, main_utime, utime );

    int incoming;
    cin >> incoming;
    cout << "end\n";
  }
}


int 
main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "registeration-app");
  string reference="CAMERALEFT";
  parser.add(reference, "r", "reference", "Reference filename root");
  parser.parse();
  cout << reference << " is reference\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  RegApp app(lcm);
  
  app.doRegisterationBatch(reference);
  cout << "registeration batch is done" << endl << endl;

  return 0;
}