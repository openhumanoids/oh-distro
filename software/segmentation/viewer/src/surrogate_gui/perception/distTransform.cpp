/*=========================================================================
Copyright 2009 Rensselaer Polytechnic Institute
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
=========================================================================*/

// ----
// ----  Extract the thinness metric based skeleton. 
// ----  Uses Toriwaki and Saito's DT algorithm. 
// ----  
// ----  Implementation by : Xiaosong, RPI
// ----
// ----  Input : Binary 3D volume with sizes. 
// ----  Output: volume file with distance transform, 
// ----          DT is zero for all object voxels; DT is scaled to 0-255.
// ----
// $Id: distTransform.cpp 598 2009-05-11 21:27:59Z galbreath $

// converted to float and added idxarr by Paul Ilardi 6/2013

#include "distTransform.h"

void distTransform(float *f, int *idxarr, int L, int M, int N) 
{
  
  int i,j,k,n;
 //float *buff , df, db, d, w;
  
  //float  df, db; //d 
  long df,db;
  float *fDist,*buff;
  // modify by xl
  long idx, slsz, sz;
  
  slsz = L*M;		// slice size
  sz = slsz*N;

  fDist = new float[L*M*N];

  for (idx = 0; idx < slsz*N; idx++) {
      //if (f[idx] > 0)   fDist[idx] = 0;
	  //else fDist[idx] = 5000;
	  if (f[idx] > 0)   fDist[idx] = 5000;
	  else {
            fDist[idx] = 0;
            idxarr[idx] = idx;
          }
  }

  int maxdim = MAX(L,M);
  maxdim = MAX(maxdim,N);
  

    //buff = new float[maxdim+10];
  buff = new float[maxdim+10];
  int* buffidx = new int[maxdim+10];

  // Using Algorithm 3 from Appendix 

  // Step 1  forward scan
  int best_idx=-2;
  for (k = 0; k < N; k++)
    for (j = 0; j < M; j++)
    {
       df = L;
       for (i = 0; i < L; i++)
       {
         idx = k*slsz + j*L + i;
         if (fDist[idx] !=0)
           df = df + 1;
         else{
           best_idx=idx;
           df = 0;
         }
         fDist[idx] = df*df;
         idxarr[idx] = best_idx;
       }
     }
 
  //  Step 1 backward scan
  best_idx=-2;
  for (k = 0; k < N; k++)
    for (j = 0; j < M; j++)
    {
      db = L;
      for (i = L-1; i >=0; i--)
      {
        idx = k*slsz + j*L + i;
        if (fDist[idx] !=0)
          db = db + 1;
        else{
          db = 0; 
          best_idx = idx;
        }
        if((db*db)<fDist[idx]) idxarr[idx] = best_idx;
        fDist[idx] = MIN(fDist[idx],db*db);
      }
    }

  // Step 2
 
  long d,w;  // add by xiao liang

  for (k = 0; k < N; k++)
    for (i = 0; i < L; i++)
    {
      for (j =0; j < M; j++){
        buff[j] = fDist[k*slsz + j*L +i];
        buffidx[j] = idxarr[k*slsz + j*L +i];
      }
    
      for (j = 0; j < M; j++)
      {
        d = buff[j];
        int dIdx = buffidx[j];
        if (d != 0)
        {
          int rmax, rstart, rend;
          rmax = (int) floor(sqrt((double)d)) + 1;
          rstart = MIN(rmax, (j-1));
          rend = MIN(rmax, (M-j));
          for (n = -rstart; n < rend; n++)
          {
              if (j+n >= 0 && j+n < M)
              {
                w = buff[j+n] + n*n;
                if (w < d)  {
                  d = w;
                  dIdx = buffidx[j+n];
                }
              }
          }
        }
        idx = k*slsz + j*L +i;
        fDist[idx] = d;
        idxarr[idx] = dIdx;
      }
    }

  // Step 3
  for (j = 0; j < M; j++)
    for (i = 0; i < L; i++)
    {
      for (k =0; k < N; k++){
        buff[k] = fDist[k*slsz + j*L +i];
        buffidx[k] = idxarr[k*slsz + j*L +i];
      }
    
      for (k = 0; k < N; k++)
      {
        int dIdx = buffidx[k];
        d = buff[k];
        if (d != 0)
        {
          int rmax, rstart, rend;
          rmax = (int) floor(sqrt((double)d)) + 1;
          rstart = MIN(rmax, (k-1));
          rend = MIN(rmax, (N-k));
          for (n = -rstart; n < rend; n++)
          {
              if (k+n >= 0 && k+n < N)
              {
                w = buff[k+n] + n*n;
                if (w < d)  {
                  d = w;
                  dIdx = buffidx[k+n];
                }
              }
          }
        }
        idx = k*slsz + j*L +i;
        fDist[idx] = d;
        idxarr[idx] = dIdx;
      }
    }

  for (idx = 0; idx < slsz*N; idx++) {
      fDist[idx] = sqrt(float(fDist[idx]));
  }


    /*double dMax = 0;
  for(idx=0; idx<sz; idx++)   {  // Scale the dist result to 255
	  if (fDist[idx] > dMax)   dMax = fDist[idx];
  }
  for(idx=0; idx<sz; idx++)   {  // Scale the dist result to 255
	   // f[idx] = fDist[idx] * 255/ dMax;
	   f[idx] = (unsigned char) fDist[idx]; // by xiao (long)
           }*/
    for(idx=0; idx<sz; idx++) f[idx] = fDist[idx];
    //printf("I am DT");
  // release memory by xiao liang
  delete []buff;
  delete []fDist;
  delete []buffidx;
  return;
}






