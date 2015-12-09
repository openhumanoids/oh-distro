#include <registration-test/icp_testing_routine.hpp>
#include "statistics.h"

/**
  * Code for ICP testing... It calls the icp_testing_routine...
  */
/*
int main(int argc, char const *argv[])
{
  Eigen::MatrixXf transfs_registr;
  int ret = icp_testing_routine(argc, argv, transfs_registr);
  if(ret == -1)
  	return ret;

  cout << "Transformations: " << transfs_registr << endl;

  float et = translationError(1, 2, 1, 5);

  cout << "Transl error: " << et << endl;

  float er = rotationError(1, 2, -40, 1, 5, 30);

  cout << "Rot error: " << er << endl;

  return 0;
}*/

class App{
  public:
    App(const int inputc, const char **inputv, Eigen::MatrixXf &transf_matrix);
    
    ~App(){
    }

    int ret; //integer value qualifying valid/invalid arguments

    RegistrationRoutine* registrationRoutine_;   

  private: 
};    

App::App(const int inputc, const char **inputv, Eigen::MatrixXf &transf_matrix){
  registrationRoutine_ = new RegistrationRoutine();

  ret = registrationRoutine_->validateArgs(inputc, inputv);
  if (ret == 0)
    registrationRoutine_->doRoutine(transf_matrix);
}

int main(const int argc, const char **argv){
  Eigen::MatrixXf registr_res;
  App fo= App(argc, argv, registr_res);

  if (fo.ret == -1)
    return fo.ret;

  cout << "Final matrix:" << endl << registr_res.block(0,0,3,6) << endl;

  string out_file;
  out_file.append(fo.registrationRoutine_->cfg_.homedir);
  out_file.append("/logs/multisenselog__2015-11-16/results/registr_transf_new.txt");
  writeTransformToFile(registr_res, out_file, fo.registrationRoutine_->cfg_.tot_clouds);

  return fo.ret;
}