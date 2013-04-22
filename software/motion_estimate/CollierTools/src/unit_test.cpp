#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "Filter.hpp"
#include "FusionStabLoop.hpp"

using namespace std;

int main() {
  cout << "Test driver for Filter objects.\n";

  cout << "Reading random data from file\n";
  
  double data;
  char loc[32];
  int r,line;
  
  FILE * fp;
  
  fp = fopen("./testing/1000_randn.csv", "r");
  
  
  if (fp == NULL)
  {
      printf ("Error opening the file\n\n'");
      exit(EXIT_FAILURE);
  }
  
  // Create the filtering object, with pointer to parent functions
  LowPassFilter lpfilter;
  Filter* _filter = &lpfilter;
  
  line = 0;
  
  
  r = fscanf(fp, "%lg\n", &data);
  while (r != EOF) {
	  //cout << data << ", " << lpfilter.processSample(data) << endl;
	  r = fscanf(fp, "%lg\n", &data);
  }
  fclose(fp);
  
  cout << "\n" << "Now creating a new TrapezoidalIntegrator object.\n";
  
  TrapezoidalIntegrator integrator(3,1/30.);
  
  Eigen::Vector3d test_values;
  test_values << 0.5,0.4,0.3;
  
  for (int i=0;i<10;i++) { integrator.integrate(test_values); }
  
  cout << "First integrated step: " << integrator.integrate(test_values) << endl;
  
  
  
  cout << endl << "Now testing the FusionStabLoop\n";
  
  FusionStabLoop stab_vel(3);
  
  
  
  
  return 0;
}

