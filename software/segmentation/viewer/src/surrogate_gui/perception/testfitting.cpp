#include "distTransform.h"
#include <iostream>

using namespace std;

int main(){
  float f[125]={};
  int idxarr[125];
  for(int i=0;i<125;i++) {
    f[i]=1;
    idxarr[i] = -1;
  }
  f[35]=0;
  f[66]=0;
  distTransform(f,idxarr,5,5,5);
  for(int i=0;i<5;i++) {
    for(int j=0;j<5;j++){
      for(int k=0;k<5;k++){
        cout << f[i*5*5+j*5+k] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }

  for(int i=0;i<5;i++) {
    for(int j=0;j<5;j++){
      for(int k=0;k<5;k++){
        cout << idxarr[i*5*5+j*5+k] << " ";
      }
      cout << endl;
    }
    cout << endl;
  }
}

