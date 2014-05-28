#include "mex.h"
#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <math.h>

// input args: srcframe, dstframe , src coordinate
// output args: dst coordinate
void mexFunction(
    int nlhs, mxArray *plhs[],
    int nrhs, const mxArray *prhs[])
{
    
   lcm_t *lcm;
   BotParam * param;
   BotFrames * frames;
   
   lcm = bot_lcm_get_global (NULL);
   if(!lcm)
     mexPrintf("Error getting LCM\n");
   param = bot_param_get_global (lcm, 0);
   if(!param)
     mexPrintf("Error getting param\n");
   frames = bot_frames_get_global (lcm, param);
   //frames = bot_frames_new (lcm, param);
   if(!frames)
     mexPrintf("Error getting frames\n");
   
   for (int i=0;i<30;i++)
     lcm_handle(lcm);
   char* src = mxArrayToString(prhs[0]);
   char* dst = mxArrayToString(prhs[1]);
   double* xyz_src = mxGetPr(prhs[2]);
   
   
   //mxCreateNumericMatrix(m, n, classid,   ComplexFlag)
   plhs[0] = mxCreateNumericMatrix(3, 1, mxDOUBLE_CLASS, mxREAL); 
   double* xyz_dst = (double*)mxGetData(plhs[0]);
   bot_frames_transform_vec (frames, src, dst, xyz_src, xyz_dst);
   mexPrintf("%f %f %f %f %f %f\n", xyz_src[0], xyz_src[1], xyz_src[2], xyz_dst[0], xyz_dst[1], xyz_dst[2]);

   mxFree(src);
   mxFree(dst);
   
    
   //bot_frames_destroy(frames);
   //bot_param_destroy(param);
   //lcm_destroy(lcm);
}


