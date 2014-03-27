#include <iostream>
#include <inttypes.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class IIRNotch{
  public:
    IIRNotch();
    
    ~IIRNotch(){
    }    
    
    void setCoeffs(int harmonic);
    // iir filter cooeffs
    Eigen::Vector3d b;
    Eigen::Vector3d a;
    
    // carry over inputs/outputs
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    
    double processSample(double input);

  private:
};