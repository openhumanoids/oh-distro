#include <estimate_tools/iir_notch.hpp>

IIRNotch::IIRNotch(){
  // defaults for 85Hz notch
  b << 0.7872805555, -1.3603002156, 0.7872805555;
  a << 1.0000000000, -1.3603002156, 0.5745611110; 
  
  x = Eigen::VectorXd(2);
  x << 0,0;
  y = Eigen::VectorXd(2);
  y << 0,0;
}
  
void IIRNotch::setCoeffs(int harmonic){
  // which 85Hz harmonic to filter
  if (harmonic==1){ //85Hz
    b << 0.7872805555, -1.3603002156, 0.7872805555;
    a << 1.0000000000, -1.3603002156, 0.5745611110;
  }else if (harmonic==2){ // 170Hz
    b << 0.6283781802, -0.6054469941,  0.6283781802;
    a << 1.0000000000, -0.6054469941,  0.2567563604;
  }else if (harmonic==3){ // 340Hz
    b << 0.3547365716, 0.3801547205, 0.3547365716;
    a << 1.0000000000,  0.3801547205, -0.2905268567;
  }
}

double IIRNotch::processSample(double input){
  bool v = false;
  
  Eigen::Vector3d x_temp ( input , x(0),  x(1) );
  Eigen::Vector3d y_temp (      0, y(0),  y(1) );
  if(v)  std::cout << input << "\n";
  if(v)  std::cout << x_temp.transpose() << " x_temp\n";
  if(v)  std::cout << y_temp.transpose() << " y_temp\n";
  if(v)  std::cout << b.transpose() << " b\n";
  if(v)  std::cout << a.transpose() << " a\n";
  
  if(v){  
    Eigen::Vector3d bit =  x_temp.cross(b);
    std::cout << bit.transpose() << " bit\n";  
  }
  
  
  double output =  (x_temp.dot(b)) -  (y_temp.dot(a));
  double temp_x = x(0);
  x << input , temp_x  ;
  double temp_y = y(0);
  y <<  output, temp_y ; 
  
  if(v)  std::cout << x.transpose() << " x\n";
  if(v)  std::cout << y.transpose() << " y\n\n";
  
  return output;
}
