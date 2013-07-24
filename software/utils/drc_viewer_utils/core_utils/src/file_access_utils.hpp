#ifndef FILE_ACCESS_UTILS_HPP
#define FILE_ACCESS_UTILS_HPP

#include <iostream>
#include <errno.h>
#include <dirent.h>
#include <lcmtypes/bot_core.h>
#include <path_util/path_util.h>

using namespace std;


namespace visualization_utils
{

//===============================================================================
// FILE ACCESS
  
  inline static bool get_xmlstring_from_file(const std::string& filename, std::string &xml_string)
  {
    // get the entire file
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      return true;
    }
    else
    {
     // ROS_ERROR("Could not open file [%s] for parsing.",filename.c_str());
     std::cerr << "Could not open file ["<<filename.c_str()<<"] for parsing."<< std::endl;
      return false;
    }
  }

      
  // this function should go into otdf_utils library
  inline static int get_OTDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "otdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  }
   //-------------------------------------------------------------------------------
  // this function should go into otdf_utils library
  inline static int get_URDF_or_SDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if((fn.substr(fn.find_last_of(".") + 1) == "urdf")||(fn.substr(fn.find_last_of(".") + 1) == "sdf")) 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  }
  
  inline static int get_URDF_filenames_from_dir (std::string dir, std::vector<std::string> &files)
  {
      DIR *dp;
      struct dirent *dirp;
      if((dp  = opendir(dir.c_str())) == NULL) {
          cout << "Error(" << errno << ") opening " << dir << endl;
          return errno;
      }

      while ((dirp = readdir(dp)) != NULL) {
        std::string fn =string(dirp->d_name);
        if(fn.substr(fn.find_last_of(".") + 1) == "urdf") 
          files.push_back(fn.substr(0,fn.find_last_of(".")));
      }
      closedir(dp);
      return 0;
  } 
  
   

}

#endif
